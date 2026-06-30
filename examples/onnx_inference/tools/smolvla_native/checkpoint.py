"""加载 SmolVLA checkpoint（safetensors）。

三套方案共用。提供两个层面的接口：
- load_raw(path)：返回完整 state_dict (500 keys)，保留原始 key 名（含 "model." 前缀）
- split_by_prefix：按 sub-module 拆分（VLAFlowMatching 顶层 / vlm / lm_expert）
"""

from pathlib import Path

import torch
from safetensors.torch import safe_open

# 顶层 5 个 nn.Linear（state_proj、action_*），fp32
TOPLEVEL_KEYS = (
    "state_proj",
    "action_in_proj",
    "action_out_proj",
    "action_time_mlp_in",
    "action_time_mlp_out",
)


def load_raw(path: str | Path) -> dict[str, torch.Tensor]:
    """读完整 checkpoint state_dict。"""
    path = Path(path)
    sd = {}
    with safe_open(path, framework="pt") as f:
        for k in f.keys():  # noqa: SIM118 - safe_open is not directly iterable on older versions.
            sd[k] = f.get_tensor(k)
    return sd


def split_by_section(state_dict: dict[str, torch.Tensor]) -> dict[str, dict[str, torch.Tensor]]:
    """按 model.{toplevel|vlm_with_expert.{vlm|lm_expert}} 拆分。

    返回:
        {
            "toplevel":  state_proj/action_* 的 weight/bias，键去掉 "model." 前缀
            "vlm":       vlm_with_expert.vlm.* 的所有 keys，键去掉 "model.vlm_with_expert.vlm." 前缀
            "lm_expert": vlm_with_expert.lm_expert.* 的所有 keys，键去掉 "model.vlm_with_expert.lm_expert." 前缀
        }
    """
    out = {"toplevel": {}, "vlm": {}, "lm_expert": {}}
    vlm_prefix = "model.vlm_with_expert.vlm."
    expert_prefix = "model.vlm_with_expert.lm_expert."
    for k, v in state_dict.items():
        if k.startswith(vlm_prefix):
            out["vlm"][k[len(vlm_prefix) :]] = v
        elif k.startswith(expert_prefix):
            out["lm_expert"][k[len(expert_prefix) :]] = v
        elif k.startswith("model."):
            short = k[len("model.") :]
            top = short.split(".")[0]
            if top in TOPLEVEL_KEYS:
                out["toplevel"][short] = v
            else:
                raise ValueError(f"unrecognized toplevel key: {k}")
        else:
            raise ValueError(f"key not starting with 'model.': {k}")
    return out


def remap_vlm_for_native(vlm_sd: dict[str, torch.Tensor]) -> tuple[dict, dict, dict]:
    """把 split_by_section 的 vlm 段拆给 native 三个子模块。

    输入 key 已去掉 'model.vlm_with_expert.vlm.' 前缀，剩下：
      model.vision_model.*, model.connector.*, model.text_model.*, lm_head.weight
    """
    vision_sd, connector_sd, text_sd = {}, {}, {}
    for k, v in vlm_sd.items():
        if k.startswith("model.vision_model."):
            vision_sd[k[len("model.vision_model.") :]] = v
        elif k.startswith("model.connector."):
            connector_sd[k[len("model.connector.") :]] = v
        elif k.startswith("model.text_model."):
            text_sd[k[len("model.text_model.") :]] = v
    return vision_sd, connector_sd, text_sd


def summary(state_dict: dict[str, torch.Tensor]) -> str:
    """打印 checkpoint 摘要（参数量、dtype 分布）。"""
    n_params = sum(v.numel() for v in state_dict.values())
    dtypes = {}
    for v in state_dict.values():
        dtypes[str(v.dtype)] = dtypes.get(str(v.dtype), 0) + v.numel()
    lines = [f"total tensors: {len(state_dict)}", f"total params:  {n_params / 1e6:.2f}M"]
    for d, n in dtypes.items():
        lines.append(f"  {d:20s} {n / 1e6:.2f}M  ({100 * n / n_params:.1f}%)")
    return "\n".join(lines)
