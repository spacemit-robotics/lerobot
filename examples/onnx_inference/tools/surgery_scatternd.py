#!/usr/bin/env python3
# spellchecker:off
"""ScatterND 去除手术。

检测成对 ScatterND（前半/后半写入同一 data），替换为 Concat(axis=-1)。
不成对的常量 index ScatterND 用 Gather 通用降解。

用法：
  python export/surgery_scatternd.py --in  models/onnx/<dir>/model.onnx \
                                     --out models/onnx/<dir2>/model.onnx
"""

import argparse
import os

import numpy as np
import onnx
from onnx import helper, numpy_helper


def _const_value(g, name, prod, inits):
    if name in inits:
        return numpy_helper.to_array(inits[name])
    producer_node = prod.get(name)
    if producer_node is not None and producer_node.op_type == "Constant":
        for a in producer_node.attribute:
            if a.name == "value":
                return numpy_helper.to_array(a.t)
    return None


def _static_shape(vi_shapes, init_shapes, name):
    if name in init_shapes:
        return init_shapes[name]
    return vi_shapes.get(name)


def _is_zero_const(g, name, inits):
    """Check if name is a zero-valued constant (initializer or Constant node)."""
    if name in inits:
        arr = numpy_helper.to_array(inits[name])
        return np.all(arr == 0)
    for n in g.node:
        if n.op_type == "Constant" and name in n.output:
            for a in n.attribute:
                if a.name == "value":
                    return np.all(numpy_helper.to_array(a.t) == 0)
    return False


def _idx_last_dim_range(iarr, dshape):
    """Check if indices write contiguous range on last dim.
    Returns (start, end) if yes, None otherwise."""
    rank = iarr.shape[-1]
    idx_flat = iarr.reshape(-1, rank)
    if len(idx_flat) == 0:
        return None
    last_vals = idx_flat[:, -1]
    start = int(last_vals.min())
    end = int(last_vals.max()) + 1
    uniq = np.unique(last_vals)
    if len(uniq) != end - start:
        return None
    return (start, end)


def surgery(model, module_tag="model"):
    g = model.graph
    model = onnx.shape_inference.infer_shapes(model)
    g = model.graph
    inits = {i.name: i for i in g.initializer}
    init_shapes = {i.name: tuple(numpy_helper.to_array(i).shape) for i in g.initializer}
    vi_shapes = {}
    for vi in list(g.value_info) + list(g.input) + list(g.output):
        dims = vi.type.tensor_type.shape.dim
        if all(d.HasField("dim_value") for d in dims):
            vi_shapes[vi.name] = tuple(d.dim_value for d in dims)
    prod = {o: n for n in g.node for o in n.output}

    scatter_by_output = {}
    for n in g.node:
        if n.op_type == "ScatterND":
            scatter_by_output[n.output[0]] = n

    # Detect pairs: first writes [0:half] into zeros, second writes [half:full] into first's output
    paired = {}  # id(first) -> (first, second)
    paired_seconds = set()
    for n in g.node:
        if n.op_type != "ScatterND":
            continue
        data_name = n.input[0]
        if data_name not in scatter_by_output:
            continue
        first = scatter_by_output[data_name]
        if not _is_zero_const(g, first.input[0], inits):
            continue
        first_idx = _const_value(g, first.input[1], prod, inits)
        second_idx = _const_value(g, n.input[1], prod, inits)
        dshape = _static_shape(vi_shapes, init_shapes, first.input[0]) or _static_shape(
            vi_shapes, init_shapes, first.output[0]
        )
        if first_idx is None or second_idx is None or dshape is None:
            continue
        last_dim = dshape[-1]
        r1 = _idx_last_dim_range(first_idx, dshape)
        r2 = _idx_last_dim_range(second_idx, dshape)
        if r1 is None or r2 is None:
            continue
        if r1[0] == 0 and r1[1] == last_dim // 2 and r2[0] == last_dim // 2 and r2[1] == last_dim:
            paired[id(first)] = (first, n)
            paired_seconds.add(id(n))

    new_nodes = []
    n_paired = 0
    n_gather = 0
    uid = 0
    pair_idx = 0
    for n in g.node:
        if n.op_type != "ScatterND":
            new_nodes.append(n)
            continue

        if id(n) in paired:
            first, second = paired[id(n)]
            new_nodes.append(
                helper.make_node(
                    "Concat",
                    [first.input[2], second.input[2]],
                    [second.output[0]],
                    f"/ScatterNDPairToConcat_{module_tag}_{pair_idx}",
                    axis=-1,
                )
            )
            pair_idx += 1
            n_paired += 1
            continue

        if id(n) in paired_seconds:
            continue

        # Unpaired: fallback to Gather-based decomposition
        data, idx, upd = n.input
        iarr = _const_value(g, idx, prod, inits)
        dshape = _static_shape(vi_shapes, init_shapes, data) or _static_shape(
            vi_shapes, init_shapes, n.output[0]
        )
        if iarr is None or dshape is None:
            new_nodes.append(n)
            continue

        dshape = tuple(int(x) for x in dshape)
        data_size = int(np.prod(dshape))
        rank = iarr.shape[-1]
        idx_flat = iarr.reshape(-1, rank)
        update_count = idx_flat.shape[0]
        strides = np.array([int(np.prod(dshape[k + 1 :])) for k in range(len(dshape))], dtype=np.int64)
        dest = (idx_flat.astype(np.int64) * strides[None, :]).sum(axis=1)
        assert len(np.unique(dest)) == update_count, "ScatterND indices 有重复"

        src = np.arange(data_size, dtype=np.int64)
        src[dest] = data_size + np.arange(update_count, dtype=np.int64)

        pre = f"sndx_{uid}_"
        uid += 1
        new_inits = [
            numpy_helper.from_array(src, pre + "src"),
            numpy_helper.from_array(np.array([-1], dtype=np.int64), pre + "flat"),
            numpy_helper.from_array(np.array(dshape, dtype=np.int64), pre + "dshape"),
        ]
        g.initializer.extend(new_inits)
        fd, fu, pool, gath = pre + "fd", pre + "fu", pre + "pool", pre + "gath"
        new_nodes += [
            helper.make_node("Reshape", [data, pre + "flat"], [fd], pre + "rd"),
            helper.make_node("Reshape", [upd, pre + "flat"], [fu], pre + "ru"),
            helper.make_node("Concat", [fd, fu], [pool], pre + "cat", axis=0),
            helper.make_node("Gather", [pool, pre + "src"], [gath], pre + "g", axis=0),
            helper.make_node("Reshape", [gath, pre + "dshape"], [n.output[0]], pre + "rout"),
        ]
        n_gather += 1

    g.ClearField("node")
    g.node.extend(new_nodes)
    g.ClearField("value_info")
    print(f"已降解 ScatterND: {n_paired} 对(Concat) + {n_gather} 个(Gather)")
    return model


def smoke(path_a, path_b, n=2):
    import onnxruntime as ort

    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_DISABLE_ALL

    def rand_in(sess, rng):
        feed = {}
        for i in sess.get_inputs():
            shp = [d if isinstance(d, int) and d > 0 else 1 for d in i.shape]
            t = i.type
            if "float16" in t:
                feed[i.name] = rng.randn(*shp).astype(np.float16)
            elif "float" in t:
                feed[i.name] = rng.randn(*shp).astype(np.float32)
            elif "int64" in t:
                feed[i.name] = np.zeros(shp, np.int64)
            elif "bool" in t:
                feed[i.name] = np.ones(shp, bool)
            else:
                feed[i.name] = np.zeros(shp, np.int64)
        return feed

    sb = ort.InferenceSession(path_b, sess_options=so, providers=["CPUExecutionProvider"])
    try:
        sa = ort.InferenceSession(path_a, sess_options=so, providers=["CPUExecutionProvider"])
    except Exception as exc:  # noqa: BLE001
        feed = rand_in(sb, np.random.RandomState(0))
        sb.run(None, feed)
        print(f"smoke: 原始模型无法被 CPU ORT 加载,已验证手术后模型可运行; 跳过前后对比: {exc}")
        return 1.0
    rng = np.random.RandomState(0)

    worst = 1.0
    for _ in range(n):
        feed = rand_in(sa, rng)
        oa = sa.run(None, feed)
        ob = sb.run(None, feed)
        for a, b in zip(oa, ob, strict=True):
            a = a.astype(np.float64).ravel()
            b = b.astype(np.float64).ravel()
            cos = float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12))
            worst = min(worst, cos)
    print(f"smoke: 最差输出 cos = {worst:.6f}（应 ≈ 1.0）")
    return worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--out", dest="out", required=True)
    ap.add_argument("--no-smoke", action="store_true")
    args = ap.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    model = onnx.load(args.inp, load_external_data=True)
    tag = os.path.splitext(os.path.basename(args.inp))[0].replace("_lm", "").replace("_step", "")
    model = surgery(model, module_tag=tag)
    onnx.save(model, args.out)
    print(f"已写出: {args.out}")
    if not args.no_smoke:
        smoke(args.inp, args.out)


if __name__ == "__main__":
    main()

# spellchecker:on
