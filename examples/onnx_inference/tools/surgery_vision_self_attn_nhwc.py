#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Fuse SmolVLA vision hot subgraphs for the EP204 custom kernels."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import onnx
from onnx import helper

CUSTOM_DOMAIN = "liteany.stereo"
CUSTOM_OP = "VisionSelfAttnNHWC"
SELF_ATTN_RE = re.compile(r"^(?P<prefix>/encoder/layers\.(?P<layer>\d+)/self_attn)")
MLP_RE = re.compile(r"^(?P<prefix>/encoder/layers\.(?P<layer>\d+)/mlp)")


def attr_ints(node: onnx.NodeProto, name: str) -> list[int] | None:
    for attr in node.attribute:
        if attr.name == name:
            return list(helper.get_attribute_value(attr))
    return None


def make_vision_self_attn_function(default_opset: int) -> onnx.FunctionProto:
    nodes = [
        helper.make_node("Transpose", ["Q"], ["Q_bhsd"], name="Q_Transpose", perm=[0, 2, 1, 3]),
        helper.make_node("Transpose", ["K"], ["K_bhds"], name="K_Transpose", perm=[0, 2, 3, 1]),
        helper.make_node("Transpose", ["V"], ["V_bhsd"], name="V_Transpose", perm=[0, 2, 1, 3]),
        helper.make_node("MatMul", ["Q_bhsd", "K_bhds"], ["Score"], name="QK_MatMul"),
        helper.make_node("Mul", ["Score", "Scale"], ["ScaledScore"], name="Scale_Mul"),
        helper.make_node("Softmax", ["ScaledScore"], ["Prob"], name="Softmax", axis=-1),
        helper.make_node("MatMul", ["Prob", "V_bhsd"], ["Ctx_bhsd"], name="AV_MatMul"),
        helper.make_node("Transpose", ["Ctx_bhsd"], ["Y"], name="Out_Transpose", perm=[0, 2, 1, 3]),
    ]
    return helper.make_function(
        CUSTOM_DOMAIN,
        CUSTOM_OP,
        inputs=["Q", "K", "V", "Scale"],
        outputs=["Y"],
        nodes=nodes,
        opset_imports=[helper.make_operatorsetid("", default_opset)],
        doc_string="Vision encoder self-attention fallback for EP204 VisionSelfAttnNHWC.",
    )


def parse_layers(value: str) -> set[int] | None:
    if value == "all":
        return None
    return {int(item) for item in value.split(",") if item.strip()}


def fuse_attention(
    model: onnx.ModelProto,
    selected_layers: set[int] | None,
    by_name: dict[str, onnx.NodeProto],
    remove_names: set[str],
    insert_before: dict[str, onnx.NodeProto],
) -> int:
    processed_prefixes: set[str] = set()
    fused_count = 0

    for node in model.graph.node:
        match = SELF_ATTN_RE.match(node.name)
        if not match:
            continue
        layer = int(match.group("layer"))
        if selected_layers is not None and layer not in selected_layers:
            continue
        prefix = match.group("prefix")
        if prefix in processed_prefixes:
            continue

        q_reshape = by_name.get(prefix + "/Reshape")
        k_reshape = by_name.get(prefix + "/Reshape_1")
        v_reshape = by_name.get(prefix + "/Reshape_2")
        tq = by_name.get(prefix + "/Transpose")
        tv = by_name.get(prefix + "/Transpose_1")
        tk = by_name.get(prefix + "/Transpose_2")
        qk = by_name.get(prefix + "/MatMul")
        scale = by_name.get(prefix + "/Mul")
        softmax = by_name.get(prefix + "/Softmax")
        av = by_name.get(prefix + "/MatMul_1")
        tout = by_name.get(prefix + "/Transpose_3")
        nodes = [q_reshape, k_reshape, v_reshape, tq, tv, tk, qk, scale, softmax, av, tout]
        if any(item is None for item in nodes):
            continue
        assert (
            q_reshape
            and k_reshape
            and v_reshape
            and tq
            and tv
            and tk
            and qk
            and scale
            and softmax
            and av
            and tout
        )

        if tq.op_type != "Transpose" or attr_ints(tq, "perm") != [0, 2, 1, 3]:
            continue
        if tv.op_type != "Transpose" or attr_ints(tv, "perm") != [0, 2, 1, 3]:
            continue
        if tk.op_type != "Transpose" or attr_ints(tk, "perm") != [0, 2, 3, 1]:
            continue
        if tout.op_type != "Transpose" or attr_ints(tout, "perm") != [0, 2, 1, 3]:
            continue
        if (
            qk.op_type != "MatMul"
            or scale.op_type != "Mul"
            or softmax.op_type != "Softmax"
            or av.op_type != "MatMul"
        ):
            continue
        if list(tq.input) != [q_reshape.output[0]]:
            continue
        if list(tk.input) != [k_reshape.output[0]]:
            continue
        if list(tv.input) != [v_reshape.output[0]]:
            continue
        if list(qk.input) != [tq.output[0], tk.output[0]]:
            continue
        if scale.input[0] != qk.output[0]:
            continue
        if softmax.input[0] != scale.output[0]:
            continue
        if list(av.input) != [softmax.output[0], tv.output[0]]:
            continue
        if list(tout.input) != [av.output[0]]:
            continue

        remove_names.update(item.name for item in [tq, tv, tk, qk, scale, softmax, av, tout])
        fused = helper.make_node(
            CUSTOM_OP,
            inputs=[q_reshape.output[0], k_reshape.output[0], v_reshape.output[0], scale.input[1]],
            outputs=[tout.output[0]],
            name=prefix + "/corr_func_nhwc",
            domain=CUSTOM_DOMAIN,
        )
        fused.attribute.extend(
            [
                helper.make_attribute("layout", "BSHD"),
                helper.make_attribute("q_perm", [0, 2, 1, 3]),
                helper.make_attribute("k_perm", [0, 2, 3, 1]),
                helper.make_attribute("v_perm", [0, 2, 1, 3]),
                helper.make_attribute("out_perm", [0, 2, 1, 3]),
            ]
        )
        insert_before[tq.name] = fused
        processed_prefixes.add(prefix)
        fused_count += 1

    return fused_count


def fuse_gelu(
    model: onnx.ModelProto,
    selected_layers: set[int] | None,
    by_name: dict[str, onnx.NodeProto],
    remove_names: set[str],
    insert_before: dict[str, onnx.NodeProto],
) -> int:
    processed_prefixes: set[str] = set()
    fused_count = 0

    for node in model.graph.node:
        match = MLP_RE.match(node.name)
        if not match:
            continue
        layer = int(match.group("layer"))
        if selected_layers is not None and layer not in selected_layers:
            continue
        prefix = match.group("prefix")
        if prefix in processed_prefixes:
            continue

        mul0 = by_name.get(prefix + "/Mul")
        mul1 = by_name.get(prefix + "/Mul_1")
        mul2 = by_name.get(prefix + "/Mul_2")
        add0 = by_name.get(prefix + "/Add")
        mul3 = by_name.get(prefix + "/Mul_3")
        tanh = by_name.get(prefix + "/Tanh")
        add1 = by_name.get(prefix + "/Add_1")
        mul4 = by_name.get(prefix + "/Mul_4")
        mul5 = by_name.get(prefix + "/Mul_5")
        gelu_nodes = [mul0, mul1, mul2, add0, mul3, tanh, add1, mul4, mul5]
        if any(item is None for item in gelu_nodes):
            continue
        assert mul0 and mul1 and mul2 and add0 and mul3 and tanh and add1 and mul4 and mul5

        if [item.op_type for item in gelu_nodes] != [
            "Mul",
            "Mul",
            "Mul",
            "Add",
            "Mul",
            "Tanh",
            "Add",
            "Mul",
            "Mul",
        ]:
            continue

        x_name = prefix + "/fc1/Add_output_0"
        if list(mul0.input) != [x_name, x_name]:
            continue
        if list(mul1.input) != [x_name, mul0.output[0]]:
            continue
        if mul2.input[1] != mul1.output[0]:
            continue
        if list(add0.input) != [x_name, mul2.output[0]]:
            continue
        if mul3.input[1] != add0.output[0]:
            continue
        if list(tanh.input) != [mul3.output[0]]:
            continue
        if add1.input[1] != tanh.output[0]:
            continue
        if list(mul4.input) != [x_name, add1.output[0]]:
            continue
        if mul5.input[1] != mul4.output[0]:
            continue

        remove_names.update(item.name for item in gelu_nodes)
        insert_before[mul0.name] = helper.make_node(
            "Gelu",
            inputs=[x_name],
            outputs=[mul5.output[0]],
            name=prefix + "/Gelu",
            approximate="tanh",
        )
        processed_prefixes.add(prefix)
        fused_count += 1

    return fused_count


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--in", dest="input", type=Path, required=True, help="Input vision_encoder.onnx")
    parser.add_argument(
        "--out", dest="output", type=Path, required=True, help="Output patched vision_encoder.onnx"
    )
    parser.add_argument("--layers", default="all", help="Comma-separated layer ids, or all")
    parser.add_argument("--no-smoke", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model = onnx.load(str(args.input), load_external_data=True)
    selected_layers = parse_layers(args.layers)
    by_name = {node.name: node for node in model.graph.node}
    remove_names: set[str] = set()
    insert_before: dict[str, onnx.NodeProto] = {}

    fused_count = fuse_attention(model, selected_layers, by_name, remove_names, insert_before)
    gelu_fused_count = fuse_gelu(model, selected_layers, by_name, remove_names, insert_before)
    if fused_count == 0:
        raise RuntimeError(f"no {CUSTOM_OP} patterns found in {args.input}")

    new_nodes = []
    for node in model.graph.node:
        if node.name in insert_before:
            new_nodes.append(insert_before[node.name])
        if node.name not in remove_names:
            new_nodes.append(node)

    del model.graph.node[:]
    model.graph.node.extend(new_nodes)

    if not any(opset.domain == CUSTOM_DOMAIN for opset in model.opset_import):
        model.opset_import.extend([helper.make_operatorsetid(CUSTOM_DOMAIN, 1)])

    if gelu_fused_count:
        for opset in model.opset_import:
            if opset.domain == "" and opset.version < 20:
                opset.version = 20
                break

    default_opset = 24
    for opset in model.opset_import:
        if opset.domain == "":
            default_opset = opset.version
            break

    kept_functions = [
        function
        for function in model.functions
        if not (function.domain == CUSTOM_DOMAIN and function.name == CUSTOM_OP)
    ]
    del model.functions[:]
    model.functions.extend(kept_functions)
    model.functions.extend([make_vision_self_attn_function(default_opset)])

    args.output.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(args.output))
    print(f"fused_layers={fused_count}")
    print(f"gelu_fused_layers={gelu_fused_count}")
    print(f"removed_nodes={len(remove_names)}")
    print(f"output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
