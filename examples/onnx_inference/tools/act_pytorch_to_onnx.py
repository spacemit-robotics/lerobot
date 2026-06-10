#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Export a LeRobot ACT policy to ONNX.

ACT inference is a single forward pass (no autoregressive KV-cache loop), so we
export it as one graph by default (Plan A). An optional backbone/transformer
split (Plan B) is provided for heterogeneous acceleration.

What stays OUTSIDE ONNX (handled by the runtime):
  - input normalization / output unnormalization
  - temporal ensembling and the action queue

Inference path notes (see modeling_act.ACT.forward):
  - The VAE encoder is NOT used at inference; the latent is a fixed zero vector.
  - The 2D sinusoidal camera position embedding only depends on the feature-map
    H/W, which is fixed by the input image size, so it is baked into the graph.

Usage:
  python tools/act_pytorch_to_onnx.py \
      --checkpoint models/pytorch/act/checkpoints/100000/pretrained_model \
      --output-dir models/onnx/act-fp32

  # Also emit Plan B (backbone + transformer) graphs
  python tools/act_pytorch_to_onnx.py --checkpoint <ckpt> --split backbone

  # Skip numerical validation (faster)
  python tools/act_pytorch_to_onnx.py --checkpoint <ckpt> --no-validate
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch
from torch import Tensor, nn

from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy

EXAMPLE_DIR = Path(__file__).resolve().parents[1]


class ACTInferenceModule(nn.Module):
    """Inference-only ACT graph (whole model = Plan A).

    Inputs are already-normalized tensors. The module reproduces the inference
    branch of ``ACT.forward`` with the latent fixed to zeros and the VAE encoder
    removed. Multi-camera input is a fixed-size stacked tensor so the graph has
    no Python-level list iteration.

    Args:
        images: (B, n_cam, 3, H, W) normalized images, or None if no cameras.
        state:  (B, state_dim) normalized robot state, or None.
        env_state: (B, env_dim) normalized environment state, or None.
    Returns:
        actions: (B, chunk_size, action_dim)
    """

    def __init__(self, policy: ACTPolicy):
        super().__init__()
        self.config = policy.config
        self.model = policy.model

    def forward(
        self,
        images: Tensor | None = None,
        state: Tensor | None = None,
        env_state: Tensor | None = None,
    ) -> Tensor:
        model = self.model
        cfg = self.config

        if images is not None:
            batch_size = images.shape[0]
        elif env_state is not None:
            batch_size = env_state.shape[0]
        else:
            batch_size = state.shape[0]
        device = next(model.parameters()).device

        # Latent is a fixed zero vector at inference (VAE encoder unused).
        latent_sample = torch.zeros([batch_size, cfg.latent_dim], dtype=torch.float32, device=device)

        # Build the prefix (single-token) sequence first: latent (+ state/env).
        # These are few (1-3 tokens) so a small stack is fine.
        prefix_tokens = [model.encoder_latent_input_proj(latent_sample)]
        if cfg.robot_state_feature:
            prefix_tokens.append(model.encoder_robot_state_input_proj(state))
        if cfg.env_state_feature:
            prefix_tokens.append(model.encoder_env_state_input_proj(env_state))
        # (n_prefix, B, C)
        encoder_in_tokens = torch.stack(prefix_tokens, dim=0)
        # 1D learned pos embed has exactly n_prefix rows.
        encoder_in_pos_embed = model.encoder_1d_feature_pos_embed.weight[: encoder_in_tokens.shape[0]].unsqueeze(1)

        if cfg.image_features:
            n_cam = images.shape[1]
            # Concatenate each camera's flattened feature map as a whole block
            # (seq_cam, B, C) instead of splitting into per-token tensors. This
            # keeps the ONNX Concat fan-in tiny (n_prefix + n_cam) instead of
            # ~600, which the SpaceMIT EP / xslim PPQ executor require.
            cam_token_blocks = [encoder_in_tokens]
            cam_pos_blocks = [encoder_in_pos_embed]
            for cam_idx in range(n_cam):
                img = images[:, cam_idx]
                cam_features = model.backbone(img)["feature_map"]
                cam_pos = model.encoder_cam_feat_pos_embed(cam_features).to(dtype=cam_features.dtype)
                cam_features = model.encoder_img_feat_input_proj(cam_features)
                cam_features = cam_features.flatten(2).permute(2, 0, 1)  # (S,B,C)
                cam_pos = cam_pos.flatten(2).permute(2, 0, 1)  # (S,B,C)
                cam_token_blocks.append(cam_features)
                cam_pos_blocks.append(cam_pos)
            encoder_in_tokens = torch.cat(cam_token_blocks, dim=0)
            encoder_in_pos_embed = torch.cat(cam_pos_blocks, dim=0)

        encoder_out = model.encoder(encoder_in_tokens, pos_embed=encoder_in_pos_embed)
        decoder_in = torch.zeros(
            (cfg.chunk_size, batch_size, cfg.dim_model),
            dtype=encoder_in_pos_embed.dtype,
            device=device,
        )
        decoder_out = model.decoder(
            decoder_in,
            encoder_out,
            encoder_pos_embed=encoder_in_pos_embed,
            decoder_pos_embed=model.decoder_pos_embed.weight.unsqueeze(1),
        )
        decoder_out = decoder_out.transpose(0, 1)
        actions = model.action_head(decoder_out)
        return actions


class ACTBackboneModule(nn.Module):
    """Plan B part 1: ResNet backbone only (conv-heavy subgraph).

    Args:
        images: (B*n_cam, 3, H, W) normalized images.
    Returns:
        feature_map: (B*n_cam, C_feat, H', W') raw ResNet layer4 features.
    """

    def __init__(self, policy: ACTPolicy):
        super().__init__()
        self.backbone = policy.model.backbone

    def forward(self, images: Tensor) -> Tensor:
        return self.backbone(images)["feature_map"]


class ACTTransformerModule(nn.Module):
    """Plan B part 2: projections + transformer + head (matmul-heavy subgraph).

    Args:
        cam_features: (B, n_cam, C_feat, H', W') stacked ResNet features.
        state:        (B, state_dim) normalized robot state, or None.
        env_state:    (B, env_dim) normalized environment state, or None.
    Returns:
        actions: (B, chunk_size, action_dim)
    """

    def __init__(self, policy: ACTPolicy):
        super().__init__()
        self.config = policy.config
        self.model = policy.model

    def forward(
        self,
        cam_features: Tensor | None = None,
        state: Tensor | None = None,
        env_state: Tensor | None = None,
    ) -> Tensor:
        model = self.model
        cfg = self.config

        if cam_features is not None:
            batch_size = cam_features.shape[0]
        elif env_state is not None:
            batch_size = env_state.shape[0]
        else:
            batch_size = state.shape[0]
        device = next(model.parameters()).device

        latent_sample = torch.zeros([batch_size, cfg.latent_dim], dtype=torch.float32, device=device)
        prefix_tokens = [model.encoder_latent_input_proj(latent_sample)]
        if cfg.robot_state_feature:
            prefix_tokens.append(model.encoder_robot_state_input_proj(state))
        if cfg.env_state_feature:
            prefix_tokens.append(model.encoder_env_state_input_proj(env_state))
        encoder_in_tokens = torch.stack(prefix_tokens, dim=0)
        encoder_in_pos_embed = model.encoder_1d_feature_pos_embed.weight[: encoder_in_tokens.shape[0]].unsqueeze(1)

        if cfg.image_features:
            n_cam = cam_features.shape[1]
            # Concat each camera's feature map as a whole block (see
            # ACTInferenceModule) to keep the ONNX Concat fan-in tiny.
            cam_token_blocks = [encoder_in_tokens]
            cam_pos_blocks = [encoder_in_pos_embed]
            for cam_idx in range(n_cam):
                feat = cam_features[:, cam_idx]
                cam_pos = model.encoder_cam_feat_pos_embed(feat).to(dtype=feat.dtype)
                feat = model.encoder_img_feat_input_proj(feat)
                feat = feat.flatten(2).permute(2, 0, 1)
                cam_pos = cam_pos.flatten(2).permute(2, 0, 1)
                cam_token_blocks.append(feat)
                cam_pos_blocks.append(cam_pos)
            encoder_in_tokens = torch.cat(cam_token_blocks, dim=0)
            encoder_in_pos_embed = torch.cat(cam_pos_blocks, dim=0)

        encoder_out = model.encoder(encoder_in_tokens, pos_embed=encoder_in_pos_embed)
        decoder_in = torch.zeros(
            (cfg.chunk_size, batch_size, cfg.dim_model),
            dtype=encoder_in_pos_embed.dtype,
            device=device,
        )
        decoder_out = model.decoder(
            decoder_in,
            encoder_out,
            encoder_pos_embed=encoder_in_pos_embed,
            decoder_pos_embed=model.decoder_pos_embed.weight.unsqueeze(1),
        )
        decoder_out = decoder_out.transpose(0, 1)
        return model.action_head(decoder_out)


def load_policy(checkpoint: str) -> ACTPolicy:
    """Load an ACT policy from a local checkpoint dir or a Hub repo id."""
    policy = ACTPolicy.from_pretrained(checkpoint)
    policy.eval()
    return policy


def _image_hw(config: ACTConfig) -> tuple[int, int]:
    """Infer (H, W) from the first image feature, fall back to 480x640."""
    for ft in config.image_features.values():
        c, h, w = ft.shape
        return int(h), int(w)
    return 480, 640


def make_dummy_inputs(config: ACTConfig, batch_size: int = 1):
    """Build a dummy normalized input set matching the policy's features."""
    inputs: dict[str, Tensor] = {}
    if config.image_features:
        n_cam = len(config.image_features)
        h, w = _image_hw(config)
        inputs["images"] = torch.randn(batch_size, n_cam, 3, h, w)
    if config.robot_state_feature:
        dim = config.robot_state_feature.shape[0]
        inputs["state"] = torch.randn(batch_size, dim)
    if config.env_state_feature:
        dim = config.env_state_feature.shape[0]
        inputs["env_state"] = torch.randn(batch_size, dim)
    return inputs


def _ordered_io(config: ACTConfig):
    """Return (input_names, dynamic_axes) for the whole-model graph."""
    input_names: list[str] = []
    dynamic_axes: dict[str, dict[int, str]] = {}
    if config.image_features:
        input_names.append("images")
        dynamic_axes["images"] = {0: "batch"}
    if config.robot_state_feature:
        input_names.append("state")
        dynamic_axes["state"] = {0: "batch"}
    if config.env_state_feature:
        input_names.append("env_state")
        dynamic_axes["env_state"] = {0: "batch"}
    dynamic_axes["actions"] = {0: "batch"}
    return input_names, dynamic_axes


def export_whole(policy: ACTPolicy, out_path: Path, opset: int, dynamo: bool = False) -> Path:
    """Plan A: export the whole inference graph to a single ONNX file."""
    module = ACTInferenceModule(policy).eval()
    dummy = make_dummy_inputs(policy.config)
    input_names, dynamic_axes = _ordered_io(policy.config)
    args = tuple(dummy[name] for name in input_names)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        module,
        args,
        str(out_path),
        input_names=input_names,
        output_names=["actions"],
        dynamic_axes=dynamic_axes,
        opset_version=opset,
        do_constant_folding=True,
        dynamo=dynamo,
    )
    print(f"[plan A] wrote {out_path}")
    return out_path


def export_split(policy: ACTPolicy, out_dir: Path, opset: int, dynamo: bool = False):
    """Plan B: export backbone and transformer as two separate ONNX files."""
    config = policy.config
    if not config.image_features:
        raise ValueError("Plan B split requires image features (a vision backbone).")

    n_cam = len(config.image_features)
    h, w = _image_hw(config)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Part 1: backbone (B*n_cam, 3, H, W) -> feature_map
    backbone = ACTBackboneModule(policy).eval()
    bb_in = torch.randn(n_cam, 3, h, w)
    bb_path = out_dir / "act_backbone.onnx"
    torch.onnx.export(
        backbone,
        (bb_in,),
        str(bb_path),
        input_names=["images"],
        output_names=["feature_map"],
        dynamic_axes={"images": {0: "batch_cam"}, "feature_map": {0: "batch_cam"}},
        opset_version=opset,
        do_constant_folding=True,
        dynamo=dynamo,
    )
    print(f"[plan B] wrote {bb_path}")

    # Probe feature-map shape so the transformer graph gets a concrete dummy.
    with torch.no_grad():
        feat = backbone(bb_in)
    c_feat, hf, wf = feat.shape[1], feat.shape[2], feat.shape[3]

    # Part 2: transformer (B, n_cam, C_feat, H', W') + state -> actions
    transformer = ACTTransformerModule(policy).eval()
    tf_inputs = {"cam_features": torch.randn(1, n_cam, c_feat, hf, wf)}
    input_names = ["cam_features"]
    dynamic_axes = {"cam_features": {0: "batch"}, "actions": {0: "batch"}}
    if config.robot_state_feature:
        tf_inputs["state"] = torch.randn(1, config.robot_state_feature.shape[0])
        input_names.append("state")
        dynamic_axes["state"] = {0: "batch"}
    if config.env_state_feature:
        tf_inputs["env_state"] = torch.randn(1, config.env_state_feature.shape[0])
        input_names.append("env_state")
        dynamic_axes["env_state"] = {0: "batch"}
    tf_path = out_dir / "act_transformer.onnx"
    torch.onnx.export(
        transformer,
        tuple(tf_inputs[name] for name in input_names),
        str(tf_path),
        input_names=input_names,
        output_names=["actions"],
        dynamic_axes=dynamic_axes,
        opset_version=opset,
        do_constant_folding=True,
        dynamo=dynamo,
    )
    print(f"[plan B] wrote {tf_path}")
    return bb_path, tf_path


def validate_whole(policy: ACTPolicy, onnx_path: Path, atol: float, rtol: float) -> bool:
    """Compare the ONNX whole-graph output against the PyTorch reference."""
    try:
        import onnxruntime as ort
    except ImportError:
        print("onnxruntime not installed; skipping numerical validation", file=sys.stderr)
        return True

    module = ACTInferenceModule(policy).eval()
    dummy = make_dummy_inputs(policy.config)
    input_names, _ = _ordered_io(policy.config)
    with torch.no_grad():
        ref = module(*[dummy[name] for name in input_names]).cpu().numpy()

    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    feeds = {name: dummy[name].cpu().numpy() for name in input_names}
    got = sess.run(["actions"], feeds)[0]

    max_abs = float(np.abs(ref - got).max())
    ok = np.allclose(ref, got, atol=atol, rtol=rtol)
    status = "OK" if ok else "MISMATCH"
    print(f"[validate] max|diff|={max_abs:.3e} atol={atol} rtol={rtol} -> {status}")
    return ok


def parse_args():
    parser = argparse.ArgumentParser(description="Export an ACT policy to ONNX")
    parser.add_argument(
        "--checkpoint",
        required=True,
        help="Path to an ACT pretrained_model dir or a Hub repo id",
    )
    parser.add_argument(
        "--output-dir",
        default=str(EXAMPLE_DIR / "models" / "onnx" / "act-fp32"),
        help="Output directory for the exported ONNX file(s)",
    )
    parser.add_argument(
        "--split",
        choices=["none", "backbone"],
        default="none",
        help="none=whole graph (Plan A); backbone=also emit backbone+transformer (Plan B)",
    )
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset version")
    parser.add_argument(
        "--dynamo",
        action="store_true",
        help="Use the new torch.export/dynamo ONNX exporter (default: legacy TorchScript exporter, "
        "more compatible with older onnxruntime / SpaceMIT EP)",
    )
    parser.add_argument("--no-validate", action="store_true", help="Skip numerical validation")
    parser.add_argument(
        "--atol",
        type=float,
        default=1e-3,
        help="Absolute tolerance for validation",
    )
    parser.add_argument(
        "--rtol",
        type=float,
        default=1e-3,
        help="Relative tolerance for validation",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out_dir = Path(args.output_dir)
    if not out_dir.is_absolute():
        out_dir = EXAMPLE_DIR / out_dir
    out_dir = out_dir.resolve()

    print(f"Loading ACT policy from: {args.checkpoint}")
    policy = load_policy(args.checkpoint)
    cfg = policy.config
    print(
        f"  cameras={len(cfg.image_features)} "
        f"state={cfg.robot_state_feature.shape[0] if cfg.robot_state_feature else 0} "
        f"chunk={cfg.chunk_size} action_dim={cfg.action_feature.shape[0]}"
    )

    whole_path = out_dir / "act.onnx"
    export_whole(policy, whole_path, args.opset, dynamo=args.dynamo)

    if args.split == "backbone":
        export_split(policy, out_dir, args.opset, dynamo=args.dynamo)

    if not args.no_validate:
        ok = validate_whole(policy, whole_path, args.atol, args.rtol)
        if not ok:
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
