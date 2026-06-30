#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import torch
import torch.nn.functional as functional
from onnx import TensorProto, helper, load, save
from torch import nn

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))

from smolvla_native.checkpoint import (  # noqa: E402
    load_raw,
    remap_vlm_for_native,
    split_by_section,
)
from smolvla_native.connector import SmolVLMConnector  # noqa: E402
from smolvla_native.siglip_vision import SiglipVisionModel  # noqa: E402

from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy  # noqa: E402


def make_att_2d_masks_onnx(pad_masks: torch.Tensor, att_masks: torch.Tensor) -> torch.Tensor:
    cumsum = torch.cumsum(att_masks.to(dtype=torch.int64), dim=1)
    att_2d_masks = cumsum[:, None, :] <= cumsum[:, :, None]
    pad_2d_masks = pad_masks[:, None, :].to(dtype=torch.bool) & pad_masks[:, :, None].to(dtype=torch.bool)
    return att_2d_masks & pad_2d_masks


def sinusoidal_time_embedding_onnx(
    timestep: torch.Tensor,
    dimension: int,
    min_period: float,
    max_period: float,
) -> torch.Tensor:
    fraction = torch.linspace(0.0, 1.0, dimension // 2, dtype=torch.float32, device=timestep.device)
    period = min_period * (max_period / min_period) ** fraction
    scaling_factor = 1.0 / period * 2 * math.pi
    sin_input = scaling_factor[None, :] * timestep.to(dtype=torch.float32)[:, None]
    return torch.cat([torch.sin(sin_input), torch.cos(sin_input)], dim=1)


TOKENS_PER_CAMERA = 64
DEFAULT_CHECKPOINT = (
    Path(__file__).resolve().parents[1] / "models/pytorch/smolvla/checkpoints/100000/pretrained_model"
)
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[1] / "models/onnx/smolvla-fp32"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export SmolVLA checkpoint to 4 ONNX subgraphs.")
    parser.add_argument(
        "--checkpoint", type=Path, default=DEFAULT_CHECKPOINT, help="SmolVLA pretrained_model dir"
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR, help="Output ONNX directory")
    parser.add_argument("--opset", type=int, default=18, help="ONNX opset version")
    parser.add_argument("--device", default="cpu", help="Export device")
    parser.add_argument(
        "--keep-dtype", action="store_true", help="Do not cast the policy to fp32 before export"
    )
    parser.add_argument(
        "--num-cameras",
        type=int,
        default=None,
        help="Camera count encoded in ONNX prefix. Default: infer from checkpoint input_features + empty_cameras.",
    )
    parser.add_argument(
        "--validate-load", action="store_true", help="Validate exported models with ONNX Runtime"
    )
    parser.add_argument(
        "--module",
        choices=["vision", "connector", "prefill", "denoise", "all"],
        default="all",
        help="Export one subgraph or all subgraphs",
    )
    parser.add_argument(
        "--no-fast-vision",
        action="store_true",
        help="Use the original transformers vision/connector export instead of the K3 fast native export",
    )
    return parser.parse_args()


def load_policy(checkpoint: Path, device: str, keep_dtype: bool) -> SmolVLAPolicy:
    checkpoint = checkpoint.expanduser()
    if not checkpoint.is_dir():
        raise FileNotFoundError(
            f"SmolVLA checkpoint directory does not exist: {checkpoint}. "
            "Check the model download step and the pretrained_model symlink."
        )
    if not (checkpoint / "config.json").is_file():
        raise FileNotFoundError(f"SmolVLA checkpoint config.json not found under: {checkpoint}")

    policy = SmolVLAPolicy.from_pretrained(checkpoint)
    policy.eval()
    policy.to(device)
    if not keep_dtype:
        policy.float()
    return policy


def infer_num_cameras(policy: SmolVLAPolicy, override: int | None) -> int:
    if override is not None:
        if override <= 0:
            raise ValueError("--num-cameras must be positive")
        return override

    image_keys = [key for key in policy.config.input_features if key.startswith("observation.images.")]
    empty_cameras = int(getattr(policy.config, "empty_cameras", 0))
    num_cameras = len(image_keys) + empty_cameras
    if num_cameras <= 0:
        raise ValueError("Cannot infer camera count from policy.config.input_features")
    return num_cameras


def save_export_config(output_dir: Path, checkpoint: Path, policy: SmolVLAPolicy, num_cameras: int) -> None:
    image_keys = [key for key in policy.config.input_features if key.startswith("observation.images.")]
    config = {
        "checkpoint": str(checkpoint),
        "format": "smolvla_4model",
        "models": ["vision_encoder.onnx", "connector.onnx", "prefill_lm.onnx", "denoise_step.onnx"],
        "image_keys": image_keys,
        "empty_cameras": int(getattr(policy.config, "empty_cameras", 0)),
        "num_cameras": num_cameras,
        "tokens_per_camera": TOKENS_PER_CAMERA,
        "chunk_size": policy.config.chunk_size,
        "n_action_steps": policy.config.n_action_steps,
        "max_state_dim": policy.config.max_state_dim,
        "max_action_dim": policy.config.max_action_dim,
        "tokenizer_max_length": policy.config.tokenizer_max_length,
        "num_steps": policy.config.num_steps,
        "action_dim": policy.config.output_features["action"].shape[0],
        "num_vlm_layers": policy.model.vlm_with_expert.num_vlm_layers,
    }
    with (output_dir / "export_config.json").open("w", encoding="utf-8") as file:
        json.dump(config, file, indent=2)


class VisionEncoderModule(nn.Module):
    def __init__(self, policy: SmolVLAPolicy):
        super().__init__()
        self.vision_model = policy.model.vlm_with_expert.get_vlm_model().vision_model

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        return self.vision_model(pixel_values=image.to(dtype=self.vision_model.dtype)).last_hidden_state


class ConnectorModule(nn.Module):
    def __init__(self, policy: SmolVLAPolicy):
        super().__init__()
        self.connector = policy.model.vlm_with_expert.get_vlm_model().connector

    def forward(self, image_hidden_states: torch.Tensor) -> torch.Tensor:
        return self.connector(image_hidden_states)


def load_native_vision_connector(checkpoint: Path, device: torch.device) -> tuple[nn.Module, nn.Module]:
    safetensors_path = checkpoint / "model.safetensors"
    if not safetensors_path.is_file():
        raise FileNotFoundError(safetensors_path)

    state_dict = load_raw(safetensors_path)
    split_state = split_by_section(state_dict)
    vision_state, connector_state, _ = remap_vlm_for_native(split_state["vlm"])

    vision_model = SiglipVisionModel().to(device=device, dtype=torch.float32).eval()
    connector = SmolVLMConnector().to(device=device, dtype=torch.float32).eval()
    vision_model.load_state_dict(
        {key: value.to(device=device, dtype=torch.float32) for key, value in vision_state.items()},
        strict=True,
    )
    connector.load_state_dict(
        {key: value.to(device=device, dtype=torch.float32) for key, value in connector_state.items()},
        strict=True,
    )
    return vision_model, connector


class PrefillLmModule(nn.Module):
    def __init__(self, policy: SmolVLAPolicy):
        super().__init__()
        self.model = policy.model
        self.lang_embedding = policy.model.vlm_with_expert.get_vlm_model().text_model.get_input_embeddings()
        self.state_proj = policy.model.state_proj
        self.vlm_with_expert = policy.model.vlm_with_expert

    def forward(
        self,
        image_embs: torch.Tensor,
        lang_tokens: torch.Tensor,
        lang_masks: torch.Tensor,
        state: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        image_embs = image_embs * torch.tensor(
            image_embs.shape[-1] ** 0.5, dtype=image_embs.dtype, device=image_embs.device
        )
        bsize = image_embs.shape[0]
        image_pad_masks = torch.ones(bsize, image_embs.shape[1], dtype=torch.bool, device=image_embs.device)
        image_att_masks = torch.zeros(bsize, image_embs.shape[1], dtype=torch.bool, device=image_embs.device)

        lang_embs = self.lang_embedding(lang_tokens)
        lang_embs = lang_embs * math.sqrt(lang_embs.shape[-1])
        lang_masks = lang_masks.to(dtype=torch.bool)
        lang_att_masks = torch.zeros(bsize, lang_embs.shape[1], dtype=torch.bool, device=lang_embs.device)

        state_embs = self.state_proj(state)
        state_embs = state_embs[:, None, :] if state_embs.ndim == 2 else state_embs
        state_pad_masks = torch.ones(bsize, state_embs.shape[1], dtype=torch.bool, device=state_embs.device)
        state_att_masks = torch.ones(bsize, state_embs.shape[1], dtype=torch.bool, device=state_embs.device)

        prefix_embs = torch.cat([image_embs, lang_embs, state_embs], dim=1)
        prefix_pad_masks = torch.cat([image_pad_masks, lang_masks, state_pad_masks], dim=1)
        prefix_att_masks = torch.cat([image_att_masks, lang_att_masks, state_att_masks], dim=1)

        prefix_att_2d_masks = make_att_2d_masks_onnx(prefix_pad_masks, prefix_att_masks)
        position_ids = torch.cumsum(prefix_pad_masks.to(dtype=torch.int64), dim=1) - 1

        _, past_key_values = self.vlm_with_expert.forward(
            attention_mask=prefix_att_2d_masks,
            position_ids=position_ids,
            past_key_values=None,
            inputs_embeds=[prefix_embs, None],
            use_cache=self.model.config.use_cache,
            fill_kv_cache=True,
        )
        past_keys = torch.stack(
            [past_key_values[layer]["key_states"] for layer in range(len(past_key_values))]
        )
        past_values = torch.stack(
            [past_key_values[layer]["value_states"] for layer in range(len(past_key_values))]
        )
        return past_keys, past_values, prefix_pad_masks


class DenoiseStepModule(nn.Module):
    def __init__(self, policy: SmolVLAPolicy):
        super().__init__()
        self.model = policy.model

    def forward(
        self,
        x_t: torch.Tensor,
        timestep: torch.Tensor,
        prefix_pad_masks: torch.Tensor,
        past_keys: torch.Tensor,
        past_values: torch.Tensor,
    ) -> torch.Tensor:
        past_key_values = {
            layer: {"key_states": past_keys[layer], "value_states": past_values[layer]}
            for layer in range(past_keys.shape[0])
        }
        action_embs = self.model.action_in_proj(x_t)
        time_embs = sinusoidal_time_embedding_onnx(
            timestep,
            self.model.vlm_with_expert.expert_hidden_size,
            self.model.config.min_period,
            self.model.config.max_period,
        )
        time_embs = time_embs.to(dtype=action_embs.dtype)[:, None, :].expand_as(action_embs)
        suffix_embs = torch.cat([action_embs, time_embs], dim=2)
        suffix_embs = self.model.action_time_mlp_in(suffix_embs)
        suffix_embs = functional.silu(suffix_embs)
        suffix_embs = self.model.action_time_mlp_out(suffix_embs)

        batch_size = suffix_embs.shape[0]
        suffix_len = suffix_embs.shape[1]
        suffix_pad_masks = torch.ones(batch_size, suffix_len, dtype=torch.bool, device=suffix_embs.device)
        suffix_att_masks = torch.ones(batch_size, suffix_len, dtype=torch.bool, device=suffix_embs.device)

        prefix_len = prefix_pad_masks.shape[1]
        prefix_pad_2d_masks = (
            prefix_pad_masks[:, None, :].to(dtype=torch.bool).expand(batch_size, suffix_len, prefix_len)
        )
        suffix_att_2d_masks = make_att_2d_masks_onnx(suffix_pad_masks, suffix_att_masks)
        full_att_2d_masks = torch.cat([prefix_pad_2d_masks, suffix_att_2d_masks], dim=2)
        prefix_offsets = torch.sum(prefix_pad_masks.to(dtype=torch.int64), dim=-1)[:, None]
        position_ids = prefix_offsets + torch.cumsum(suffix_pad_masks.to(dtype=torch.int64), dim=1) - 1

        outputs_embeds, _ = self.model.vlm_with_expert.forward(
            attention_mask=full_att_2d_masks,
            position_ids=position_ids,
            past_key_values=past_key_values,
            inputs_embeds=[None, suffix_embs],
            use_cache=self.model.config.use_cache,
            fill_kv_cache=False,
        )
        suffix_out = outputs_embeds[1]
        suffix_out = suffix_out[:, -self.model.config.chunk_size :]
        suffix_out = suffix_out.to(dtype=torch.float32)
        return self.model.action_out_proj(suffix_out)


def export_model(
    module: nn.Module,
    output_path: Path,
    inputs: tuple[torch.Tensor, ...],
    input_names: list[str],
    output_names: list[str],
    opset: int,
    dynamic_axes: dict[str, dict[int, str]] | None = None,
) -> None:
    print(f"Exporting {output_path.name} ...")
    module.eval()
    with torch.no_grad():
        torch.onnx.export(
            module,
            inputs,
            output_path,
            input_names=input_names,
            output_names=output_names,
            opset_version=opset,
            dynamic_axes=dynamic_axes,
            do_constant_folding=True,
            dynamo=False,
        )
    fix_scatternd_update_dtype(output_path)


def fix_scatternd_update_dtype(model_path: Path) -> None:
    model = load(model_path)
    producer_by_output = {output: node for node in model.graph.node for output in node.output}
    patched = False

    for node in list(model.graph.node):
        if node.op_type != "ScatterND":  # spellchecker:disable-line
            continue
        data_name = node.input[0]
        updates_name = node.input[2]
        if "vision_model/embeddings/ScatterND" in node.name:  # spellchecker:disable-line
            data_cast_output = f"{data_name}_cast_to_int64"
            updates_cast_output = f"{updates_name}_cast_to_int64"
            data_cast_node = helper.make_node(
                "Cast", [data_name], [data_cast_output], name=f"{node.name}_CastData", to=TensorProto.INT64
            )
            updates_cast_node = helper.make_node(
                "Cast",
                [updates_name],
                [updates_cast_output],
                name=f"{node.name}_CastUpdates",
                to=TensorProto.INT64,
            )
            insert_at = list(model.graph.node).index(node)
            model.graph.node.insert(insert_at, data_cast_node)
            model.graph.node.insert(insert_at + 1, updates_cast_node)
            node.input[0] = data_cast_output
            node.input[2] = updates_cast_output
            patched = True
            continue
        data_node = producer_by_output.get(data_name)
        updates_node = producer_by_output.get(updates_name)
        if data_node is None or updates_node is None:
            continue
        data_attr = next((attr for attr in data_node.attribute if attr.name == "value"), None)
        updates_attr = next((attr for attr in updates_node.attribute if attr.name == "value"), None)
        if data_attr is None or updates_attr is None:
            continue
        data_type = data_attr.t.data_type
        updates_type = updates_attr.t.data_type
        if data_type == updates_type or data_type == TensorProto.UNDEFINED:
            continue

        cast_output = f"{updates_name}_cast_to_scatter_data"
        cast_node = helper.make_node(
            "Cast", [updates_name], [cast_output], name=f"{node.name}_CastUpdates", to=data_type
        )
        insert_at = list(model.graph.node).index(node)
        model.graph.node.insert(insert_at, cast_node)
        node.input[2] = cast_output
        patched = True

    if patched:
        save(model, model_path)


def merge_to_single_file(model_path: Path) -> None:
    data_path = Path(str(model_path) + ".data")
    if not data_path.exists():
        return
    model = load(model_path, load_external_data=True)
    save(model, model_path)
    data_path.unlink()


def validate_load(output_dir: Path) -> None:
    import onnxruntime as ort

    for name in ["vision_encoder.onnx", "connector.onnx", "prefill_lm.onnx", "denoise_step.onnx"]:
        path = output_dir / name
        session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        inputs = [(item.name, item.shape, item.type) for item in session.get_inputs()]
        outputs = [(item.name, item.shape, item.type) for item in session.get_outputs()]
        print(f"Validated {name}: inputs={inputs}, outputs={outputs}")


def main() -> None:
    args = parse_args()
    os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    policy = load_policy(args.checkpoint, args.device, args.keep_dtype)
    device = torch.device(args.device)
    dtype = next(policy.parameters()).dtype
    vision_model = policy.model.vlm_with_expert.get_vlm_model().vision_model
    vision_dtype = vision_model.dtype
    num_cameras = infer_num_cameras(policy, args.num_cameras)
    image_tokens = TOKENS_PER_CAMERA * num_cameras

    hidden_size = policy.model.vlm_with_expert.config.text_config.hidden_size
    vision_hidden_size = vision_model.config.hidden_size
    prefix_len = image_tokens + policy.config.tokenizer_max_length + 1
    num_layers = policy.model.vlm_with_expert.num_vlm_layers
    kv_heads = policy.model.vlm_with_expert.config.text_config.num_key_value_heads
    head_dim = policy.model.vlm_with_expert.config.text_config.head_dim

    image = torch.randn(1, 3, 512, 512, dtype=vision_dtype, device=device)
    image_hidden_states = torch.randn(1, 1024, vision_hidden_size, dtype=dtype, device=device)
    image_embs = torch.randn(1, image_tokens, hidden_size, dtype=dtype, device=device)
    lang_tokens = torch.zeros(1, policy.config.tokenizer_max_length, dtype=torch.long, device=device)
    lang_masks = torch.ones(1, policy.config.tokenizer_max_length, dtype=torch.long, device=device)
    state = torch.randn(1, policy.config.max_state_dim, dtype=dtype, device=device)
    x_t = torch.randn(1, policy.config.chunk_size, policy.config.max_action_dim, dtype=dtype, device=device)
    timestep = torch.ones(1, dtype=dtype, device=device)
    prefix_pad_masks = torch.ones(1, prefix_len, dtype=torch.bool, device=device)
    past_keys = torch.randn(num_layers, 1, prefix_len, kv_heads, head_dim, dtype=dtype, device=device)
    past_values = torch.randn(num_layers, 1, prefix_len, kv_heads, head_dim, dtype=dtype, device=device)

    if args.no_fast_vision:
        vision_export_module: nn.Module = VisionEncoderModule(policy)
        connector_export_module: nn.Module = ConnectorModule(policy)
        vision_input = image
        connector_input = image_hidden_states
    else:
        vision_export_module, connector_export_module = load_native_vision_connector(args.checkpoint, device)
        vision_input = torch.randn(1, 3, 512, 512, dtype=torch.float32, device=device)
        connector_input = torch.randn(1, 1024, vision_hidden_size, dtype=torch.float32, device=device)

    if args.module in ("vision", "all"):
        export_model(
            vision_export_module,
            args.output_dir / "vision_encoder.onnx",
            (vision_input,),
            ["image"],
            ["image_hidden_states"],
            args.opset,
            dynamic_axes={"image": {0: "batch"}, "image_hidden_states": {0: "batch"}},
        )
        merge_to_single_file(args.output_dir / "vision_encoder.onnx")
    if args.module in ("connector", "all"):
        export_model(
            connector_export_module,
            args.output_dir / "connector.onnx",
            (connector_input,),
            ["image_hidden_states"],
            ["image_embs"],
            args.opset,
            dynamic_axes={"image_hidden_states": {0: "batch"}, "image_embs": {0: "batch"}},
        )
        merge_to_single_file(args.output_dir / "connector.onnx")
    if args.module in ("prefill", "all"):
        export_model(
            PrefillLmModule(policy),
            args.output_dir / "prefill_lm.onnx",
            (image_embs, lang_tokens, lang_masks, state),
            ["image_embs", "lang_tokens", "lang_masks", "state"],
            ["past_keys", "past_values", "prefix_pad_masks"],
            args.opset,
        )
    if args.module in ("denoise", "all"):
        export_model(
            DenoiseStepModule(policy),
            args.output_dir / "denoise_step.onnx",
            (x_t, timestep, prefix_pad_masks, past_keys, past_values),
            ["x_t", "timestep", "prefix_pad_masks", "past_keys", "past_values"],
            ["v_t"],
            args.opset,
        )
    save_export_config(args.output_dir, args.checkpoint, policy, num_cameras)

    if args.validate_load:
        validate_load(args.output_dir)

    print(
        f"Done. ONNX models written to {args.output_dir} (num_cameras={num_cameras}, prefix_len={prefix_len})"
    )


if __name__ == "__main__":
    main()
