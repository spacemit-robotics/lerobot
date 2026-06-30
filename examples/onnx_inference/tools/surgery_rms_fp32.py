#!/usr/bin/env python3
"""RMSNorm 子图 fp32 化手术(通用,任意 fp16 ONNX)。

刀法与 fp16_2cam_noscatterpre_noscatterden_rmsfp32 的 prefill 完全一致
(逆向自该模型):
  x(fp16) ─ Cast→FLOAT ─→ Pow(2) → ReduceMean → Add(eps) → Sqrt → Div(x,·) → Mul(·,w)
                     └────────────────────────────────────┘(x 同源接 Div)
  Mul 输出 ─ Cast→FLOAT16 ─→ 原消费者
区间内 fp16 initializer(Add 的 eps、Mul 的 weight)就地转 fp32;
Pow 的指数输入类型独立(ONNX Pow T/T1),不需转。

背景:fp16 下 mean(x²) 累加失真(EP kernel 精度不足),逐层级联放大,
见 docs/export_and_surgery.md。本脚本通用于任意含 RMSNorm 子图的 fp16 ONNX。
最终 K3 部署模型对 prefill_lm 应用本手术(RMSNorm fp32 数值矫正)。

用法:
  python export/surgery_rms_fp32.py --in  models/onnx/<dir>/prefill_lm.onnx \
                                    --out models/onnx/<dir2>/prefill_lm.onnx
手术后自动跑 PC CPU smoke test(随机输入,对比手术前后输出量级)。
"""

import argparse
import os

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper


def find_rms_groups(g):
    """匹配 RMSNorm 子图,支持两种导出式:
    除法式(prefill):Pow(x,2)→ReduceMean→Add→Sqrt→Div(x,·)[→Mul(·,w)]
    倒数式(denoise):Pow(x,2)→ReduceMean→Add→Sqrt→Div(1,·)→Mul(x,·)
    返回 [(pow,rm,add,sqrt,div,tail_mul,x_input_slots)],
    x_input_slots = [(node, input_idx), ...] 需改接入口 Cast 的位置。
    """
    init_names = {i.name for i in g.initializer}
    in2nodes = {}
    for n in g.node:
        for i in n.input:
            in2nodes.setdefault(i, []).append(n)

    def sole_consumer(t, op):
        cs = [c for c in in2nodes.get(t, []) if c.op_type == op]
        return cs[0] if len(cs) == 1 else None

    groups = []
    for p in g.node:
        if p.op_type != "Pow":
            continue
        rm = sole_consumer(p.output[0], "ReduceMean")
        if rm is None:
            continue
        add = sole_consumer(rm.output[0], "Add")
        if add is None:
            continue
        sq = sole_consumer(add.output[0], "Sqrt")
        if sq is None:
            continue
        dv = sole_consumer(sq.output[0], "Div")
        if dv is None:
            continue
        x = p.input[0]
        slots = [(p, 0)]
        if dv.input[0] == x:
            # 除法式:x/std,尾随 ×weight 的 Mul 可选纳入
            slots.append((dv, 0))
            ml = sole_consumer(dv.output[0], "Mul")
        elif dv.input[0] in init_names:
            # 倒数式:1/std,后接 Mul(x, rsqrt) 必须存在且引用同源 x
            ml = sole_consumer(dv.output[0], "Mul")
            if ml is None:
                continue
            x_idx = next((i for i, t in enumerate(ml.input) if t == x), None)
            if x_idx is None:
                continue
            slots.append((ml, x_idx))
        else:
            continue
        groups.append((p, rm, add, sq, dv, ml, slots))
    return groups


def make_fp32ify(g):
    """返回 fp32ify(node, input_idx)：把节点的第 idx 个输入的常量复制为 fp32 副本（_fp32 后缀），
    并更新节点输入指向新名字。保留原 fp16 initializer 不动。"""
    init_map = {i.name: i for i in g.initializer}
    const_nodes = {n.output[0]: n for n in g.node if n.op_type == "Constant"}
    created = {}

    def fp32ify(node, input_idx):
        name = node.input[input_idx]
        if name in created:
            node.input[input_idx] = created[name]
            return True
        new_name = name + "_fp32"
        init = init_map.get(name)
        if init is not None and init.data_type == TensorProto.FLOAT16:
            arr = numpy_helper.to_array(init).astype(np.float32)
            g.initializer.append(numpy_helper.from_array(arr, new_name))
            created[name] = new_name
            node.input[input_idx] = new_name
            return True
        cn = const_nodes.get(name)
        if cn is not None:
            for a in cn.attribute:
                if a.name == "value" and a.t.data_type == TensorProto.FLOAT16:
                    arr = numpy_helper.to_array(a.t).astype(np.float32)
                    a.t.CopyFrom(numpy_helper.from_array(arr))
                    cn.output[0] = new_name
                    created[name] = new_name
                    node.input[input_idx] = new_name
                    return True
        return False

    return fp32ify


def fix_mixed_div_fp16_initializers(g):
    """Fix Div nodes left by fp16 conversion with one fp16 constant input."""
    init_map = {i.name: i for i in g.initializer}
    producer = {output: node for node in g.node for output in node.output}
    consumers = {}
    for node in g.node:
        for name in node.input:
            consumers.setdefault(name, []).append(node)
    fp32ify = make_fp32ify(g)
    fixed = 0

    def is_fp32(name):
        init = init_map.get(name)
        if init is not None:
            return init.data_type == TensorProto.FLOAT
        node = producer.get(name)
        if node is None or node.op_type != "Cast":
            return False
        return any(attr.name == "to" and attr.i == TensorProto.FLOAT for attr in node.attribute)

    def cast_to_fp16(name):
        node = producer.get(name)
        if node is None or node.op_type != "Cast" or len(consumers.get(name, [])) != 1:
            return False
        for attr in node.attribute:
            if attr.name == "to" and attr.i == TensorProto.FLOAT:
                attr.i = TensorProto.FLOAT16
                return True
        return False

    for node in g.node:
        if node.op_type != "Div" or len(node.input) != 2:
            continue
        left, right = node.input
        left_init = init_map.get(left)
        right_init = init_map.get(right)
        if left_init is not None and left_init.data_type == TensorProto.FLOAT16 and is_fp32(right):
            fixed += int(cast_to_fp16(right) or fp32ify(node, 0))
        if right_init is not None and right_init.data_type == TensorProto.FLOAT16 and is_fp32(left):
            fixed += int(cast_to_fp16(left) or fp32ify(node, 1))
    return fixed


def surgery(model):
    g = model.graph
    groups = find_rms_groups(g)
    if not groups:
        return surgery_rmsnormalization_ops(model)
    fp32ify = make_fp32ify(g)
    # 注意:protobuf repeated field 的 append 是拷贝语义,重建 node 列表会使
    # groups 中持有的节点引用失效——必须先完成全部对象级修改,最后一次性重建插 Cast
    cast_before = {}  # id(pow_node) -> cast_in
    cast_after = {}  # id(tail_node) -> cast_out
    for k, (p, rm, add, sq, dv, ml, slots) in enumerate(groups):
        x = p.input[0]
        cast_in_out = f"{x}__rms_fp32_{k}"
        base_name = p.name.rsplit("/", 1)[0] if "/" in p.name else p.name
        cast_before[id(p)] = helper.make_node(
            "Cast", [x], [cast_in_out], name=f"{base_name}/CastToFp32ForRms", to=TensorProto.FLOAT
        )
        for node, idx in slots:
            node.input[idx] = cast_in_out
        # 区间内 fp16 常量转 fp32:Add 的 eps、Div 的 1.0(倒数式)、Mul 的 weight
        # (initializer 或 Constant 节点两种来源,主链 tensor 名不会命中)
        for node in (p, rm, add, sq, dv) + ((ml,) if ml is not None else ()):
            for idx in range(len(node.input)):
                fp32ify(node, idx)
        tail = ml if ml is not None else dv
        # 出口:tail 输出改名,Cast 回 fp16 接回原消费者
        old_out = tail.output[0]
        new_out = f"{old_out}__fp32"
        tail.output[0] = new_out
        cast_after[id(tail)] = helper.make_node(
            "Cast", [new_out], [old_out], name=f"{base_name}/CastRmsBackToFp16", to=TensorProto.FLOAT16
        )
    nodes = list(g.node)
    g.ClearField("node")
    for n in nodes:
        if id(n) in cast_before:
            g.node.append(cast_before[id(n)])
        g.node.append(n)
        if id(n) in cast_after:
            g.node.append(cast_after[id(n)])
    # 链上 tensor 类型已变,清空 value_info 让 ORT 重推断
    g.ClearField("value_info")
    n_div_const = fix_mixed_div_fp16_initializers(g)
    return len(groups), len(cast_before) + len(cast_after), n_div_const


def surgery_rmsnormalization_ops(model):
    """fp32 化 ONNX opset 24 的 RMSNormalization 节点。

    新版 torch/onnx 会把 RMSNorm 直接导出成 RMSNormalization 算子,不再展开成
    Pow→ReduceMean→Add→Sqrt→Div。这里保持节点输出名不变,只在节点前后插入
    Cast,并把 scale initializer 复制为 fp32。
    """
    g = model.graph
    nodes_to_rewrite = [n for n in g.node if n.op_type == "RMSNormalization"]
    assert nodes_to_rewrite, "未找到 RMSNorm 子图或 RMSNormalization 算子"

    fp32ify = make_fp32ify(g)
    cast_before = {}
    cast_after = {}
    for index, node in enumerate(nodes_to_rewrite):
        x = node.input[0]
        old_out = node.output[0]
        base_name = node.name.rsplit("/", 1)[0] if "/" in node.name else node.name
        if not base_name:
            base_name = f"RMSNormalization_{index}"

        cast_in_out = f"{x}__rms_op_fp32_{index}"
        node.input[0] = cast_in_out
        cast_before[id(node)] = helper.make_node(
            "Cast", [x], [cast_in_out], name=f"{base_name}/CastToFp32ForRms", to=TensorProto.FLOAT
        )
        for input_index in range(1, len(node.input)):
            fp32ify(node, input_index)

        new_out = f"{old_out}__fp32"
        node.output[0] = new_out
        cast_after[id(node)] = helper.make_node(
            "Cast", [new_out], [old_out], name=f"{base_name}/CastRmsBackToFp16", to=TensorProto.FLOAT16
        )

    nodes = list(g.node)
    g.ClearField("node")
    for node in nodes:
        if id(node) in cast_before:
            g.node.append(cast_before[id(node)])
        g.node.append(node)
        if id(node) in cast_after:
            g.node.append(cast_after[id(node)])
    g.ClearField("value_info")
    n_div_const = fix_mixed_div_fp16_initializers(g)
    return len(nodes_to_rewrite), len(cast_before) + len(cast_after), n_div_const


def smoke(path_a, path_b):
    import onnxruntime as ort

    rng = np.random.default_rng(0)
    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_BASIC

    sess_b = ort.InferenceSession(path_b, so, providers=["CPUExecutionProvider"])
    feed = {}
    for i in sess_b.get_inputs():
        shape = [d if isinstance(d, int) else 1 for d in i.shape]
        if i.type == "tensor(bool)":
            feed[i.name] = np.ones(shape, dtype=bool)
        elif "int64" in i.type:
            feed[i.name] = rng.integers(0, 100, shape).astype(np.int64)
        elif "int32" in i.type:
            feed[i.name] = rng.integers(0, 100, shape).astype(np.int32)
        elif "float16" in i.type:
            feed[i.name] = rng.standard_normal(shape).astype(np.float16)
        else:
            feed[i.name] = rng.standard_normal(shape).astype(np.float32)
    out_b = sess_b.run(None, feed)
    del sess_b

    try:
        sess_a = ort.InferenceSession(path_a, so, providers=["CPUExecutionProvider"])
        out_a = sess_a.run(None, feed)
        del sess_a
    except Exception as exc:  # noqa: BLE001
        print(f"smoke: 原始模型无法被 CPU ORT 加载,已验证手术后模型可运行; 跳过前后对比: {exc}")
        return

    a, b = out_a[0].astype(np.float64).ravel(), out_b[0].astype(np.float64).ravel()
    cos = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12))
    print(
        f"smoke(随机输入,仅量级参考): cos={cos:.6f}  max|Δ|={np.abs(a - b).max():.4f}  range={a.max() - a.min():.2f}"
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--out", dest="out", required=True)
    ap.add_argument("--no-smoke", action="store_true")
    args = ap.parse_args()

    model = onnx.load(args.inp)
    n_groups, n_cast, n_div_const = surgery(model)
    onnx.checker.check_model(model, full_check=False)
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    onnx.save(model, args.out)
    print(
        f"手术完成: {n_groups} 组 RMSNorm fp32 化,新增 {n_cast} 个 Cast,"
        f"修正 {n_div_const} 个 Div fp16 常量 → {args.out}"
    )
    if not args.no_smoke:
        smoke(args.inp, args.out)


if __name__ == "__main__":
    main()
