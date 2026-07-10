#!/usr/bin/env python3

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Process-local SpineTorch ACT FP16 extensions for unmodified LeRobot sources."""

from __future__ import annotations

import copy
import json
import logging
import os
import sys
import time
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any

import einops
import torch
from torch import Tensor, nn
from torchvision.ops.misc import FrozenBatchNorm2d

try:
    from torch.utils import mkldnn as mkldnn_utils
except ImportError:  # pragma: no cover - depends on the Torch build
    mkldnn_utils = None

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACT, ACTPolicy
from lerobot.utils.constants import ACTION, OBS_ENV_STATE, OBS_IMAGES, OBS_STATE


def _spinednn_torch_plugin_enabled() -> bool:
    plugin = sys.modules.get("spinednn_torch_plugin")
    is_enabled = getattr(plugin, "is_enabled", None)
    if not callable(is_enabled):
        return False
    try:
        return bool(is_enabled())
    except Exception:
        return False


def _env_flag(name: str) -> bool:
    return os.environ.get(name, "").lower() in {"1", "true", "yes", "on"}


def _use_spinednn_visual_context() -> bool:
    return _env_flag("SPINEDNN_TORCH_PLUGIN_VISUAL_CONTEXT")


def _fuse_conv_frozen_batch_norm(conv: nn.Conv2d, bn: FrozenBatchNorm2d) -> None:
    """Fold FrozenBatchNorm2d into the preceding convolution in place."""
    weight = conv.weight.detach()
    bias = conv.bias.detach() if conv.bias is not None else torch.zeros_like(bn.running_mean)

    bn_weight = bn.weight.detach()
    bn_bias = bn.bias.detach()
    bn_running_mean = bn.running_mean.detach()
    bn_running_var = bn.running_var.detach()

    scale = bn_weight * torch.rsqrt(bn_running_var + bn.eps)
    fused_weight = weight * scale.reshape(-1, 1, 1, 1)
    fused_bias = (bias - bn_running_mean) * scale + bn_bias

    conv.weight = nn.Parameter(fused_weight)
    conv.bias = nn.Parameter(fused_bias)

    # Reset BN to identity so repeated calls remain numerically stable.
    bn.weight.data.fill_(1)
    bn.bias.data.zero_()
    bn.running_mean.data.zero_()
    bn.running_var.data.fill_(1 - bn.eps)


def _fuse_frozen_batch_norms_in_module(module: nn.Module) -> None:
    children = list(module.named_children())
    for idx, (_, child) in enumerate(children[:-1]):
        next_child = children[idx + 1][1]
        if isinstance(child, nn.Conv2d) and isinstance(next_child, FrozenBatchNorm2d):
            _fuse_conv_frozen_batch_norm(child, next_child)

    for child in module.children():
        _fuse_frozen_batch_norms_in_module(child)


def _replace_fused_frozen_batch_norms_with_identity(module: nn.Module) -> None:
    for name, child in list(module.named_children()):
        if isinstance(child, FrozenBatchNorm2d):
            setattr(module, name, nn.Identity())
            continue
        _replace_fused_frozen_batch_norms_with_identity(child)


def _prepare_backbone_image(img: Tensor, input_channels: int | None = None) -> Tensor:
    if img.ndim == 4:
        if input_channels is not None:
            if img.shape[1] == input_channels:
                return img.contiguous()
            if img.shape[-1] == input_channels:
                return img.permute(0, 3, 1, 2).contiguous()

        img = img.contiguous()

    return img


def _to_mkldnn_if_needed(img: Tensor) -> Tensor:
    if img.is_mkldnn:
        return img
    return img.to_mkldnn(img.dtype)


def _invalidate_mkldnn_vision_path(self) -> None:
    self._mkldnn_backbone = None
    self._mkldnn_encoder_img_feat_input_proj = None
    self._mkldnn_vision_dtype = None
    self._spinednn_visual_contexts.clear()


def _invalidate_inference_caches(self) -> None:
    self._cached_empty_latent = None
    self._cached_decoder_input = None
    self._cached_cam_pos_embed.clear()


def _optimize_vision_path_for_inference(self) -> None:
    if (
        self.config.dtype != "float16"
        or self.training
        or not self.config.image_features
        or self._vision_path_optimized
    ):
        return

    _fuse_frozen_batch_norms_in_module(self.backbone)
    self.backbone.to(memory_format=torch.channels_last)
    self.encoder_img_feat_input_proj.to(memory_format=torch.channels_last)
    self._invalidate_mkldnn_vision_path()
    self._invalidate_inference_caches()
    self._vision_path_optimized = True


def _get_zero_latent(self, batch_size: int, dtype: torch.dtype, device: torch.device) -> Tensor:
    if (
        self._cached_empty_latent is None
        or self._cached_empty_latent.shape[0] != batch_size
        or self._cached_empty_latent.dtype != dtype
        or self._cached_empty_latent.device != device
    ):
        self._cached_empty_latent = torch.zeros(
            (batch_size, self.config.latent_dim),
            dtype=dtype,
            device=device,
        )
    return self._cached_empty_latent


def _get_decoder_input(self, batch_size: int, dtype: torch.dtype, device: torch.device) -> Tensor:
    if (
        self._cached_decoder_input is None
        or self._cached_decoder_input.shape[1] != batch_size
        or self._cached_decoder_input.dtype != dtype
        or self._cached_decoder_input.device != device
    ):
        self._cached_decoder_input = torch.zeros(
            (self.config.chunk_size, batch_size, self.config.dim_model),
            dtype=dtype,
            device=device,
        )
    return self._cached_decoder_input


def _should_use_mkldnn_vision_fast_path(self, img: Tensor) -> bool:
    return (
        self.config.image_features
        and self.config.dtype == "float16"
        and not self.training
        and mkldnn_utils is not None
        and torch.backends.mkldnn.enabled
        and img.device.type == "cpu"
        and img.dtype == torch.float16
    )


def _ensure_mkldnn_vision_modules(self, dtype: torch.dtype) -> None:
    if self._mkldnn_backbone is not None and self._mkldnn_vision_dtype == dtype:
        return

    backbone = copy.deepcopy(self.backbone).eval()
    _replace_fused_frozen_batch_norms_with_identity(backbone)
    encoder_img_feat_input_proj = copy.deepcopy(self.encoder_img_feat_input_proj).eval()

    self._mkldnn_backbone = mkldnn_utils.to_mkldnn(backbone, dtype=dtype)
    self._mkldnn_encoder_img_feat_input_proj = mkldnn_utils.to_mkldnn(
        encoder_img_feat_input_proj,
        dtype=dtype,
    )
    self._mkldnn_vision_dtype = dtype


def _should_use_spinednn_visual_context(self, img: Tensor) -> bool:
    return (
        _use_spinednn_visual_context()
        and _spinednn_torch_plugin_enabled()
        and self.config.image_features
        and not self.training
        and img.device.type == "cpu"
        and img.dtype == torch.float16
        and img.ndim == 4
    )


def _ensure_spinednn_visual_context(self, img: Tensor) -> Any:
    input_shape = tuple(int(dim) for dim in img.shape)
    cache_key = (img.dtype, input_shape)
    context = self._spinednn_visual_contexts.get(cache_key)
    if context is not None:
        return context

    import spinednn_torch_plugin as plugin

    context = plugin.ops.create_visual_context(
        self.backbone,
        self.encoder_img_feat_input_proj,
        input_shape,
        architecture="resnet18",
    )
    self._spinednn_visual_contexts[cache_key] = context
    return context


def _get_camera_position_embedding(self, cam_features: Tensor) -> Tensor:
    _, _, height, width = cam_features.shape
    cache_key = (height, width, cam_features.device)
    cam_pos_embed = self._cached_cam_pos_embed.get(cache_key)
    if cam_pos_embed is None:
        cam_pos_embed = self.encoder_cam_feat_pos_embed(cam_features[:1])
        self._cached_cam_pos_embed[cache_key] = cam_pos_embed
    return cam_pos_embed.to(dtype=cam_features.dtype)


def _extract_image_features(self, img: Tensor) -> Tensor:
    img = _prepare_backbone_image(img, self.backbone.conv1.in_channels)

    if self._should_use_spinednn_visual_context(img):
        context = self._ensure_spinednn_visual_context(img)
        return context.forward(img)

    if self._should_use_mkldnn_vision_fast_path(img):
        self._ensure_mkldnn_vision_modules(img.dtype)
        mkldnn_img = _to_mkldnn_if_needed(img)
        cam_features = self._mkldnn_backbone(mkldnn_img)["feature_map"]
        return self._mkldnn_encoder_img_feat_input_proj(cam_features).to_dense()

    cam_features = self.backbone(img)["feature_map"]
    return self.encoder_img_feat_input_proj(cam_features)


def _extract_image_tokens(self, img: Tensor) -> tuple[Tensor, Tensor]:
    cam_features = self._extract_image_features(img)
    cam_pos_embed = self._get_camera_position_embedding(cam_features)
    cam_features = einops.rearrange(cam_features, "b c h w -> (h w) b c")
    cam_pos_embed = einops.rearrange(cam_pos_embed, "b c h w -> (h w) b c")
    return cam_features, cam_pos_embed


def _extract_multi_image_tokens(self, images: list[Tensor]) -> list[tuple[Tensor, Tensor]]:
    if len(images) == 1:
        return [self._extract_image_tokens(images[0])]

    prepared_images = [_prepare_backbone_image(img, self.backbone.conv1.in_channels) for img in images]
    first_shape = prepared_images[0].shape
    if any(img.shape != first_shape for img in prepared_images[1:]):
        return [self._extract_image_tokens(img) for img in images]

    merged_images = torch.cat(prepared_images, dim=0)
    cam_features = self._extract_image_features(merged_images)
    cam_pos_embed = self._get_camera_position_embedding(cam_features)

    feature_chunks = cam_features.split(first_shape[0], dim=0)
    return [
        (
            einops.rearrange(feature_chunk, "b c h w -> (h w) b c"),
            einops.rearrange(cam_pos_embed, "b c h w -> (h w) b c"),
        )
        for feature_chunk in feature_chunks
    ]


def _build_encoder_inputs(
    self,
    latent_token: Tensor,
    robot_state: Tensor | None = None,
    env_state: Tensor | None = None,
    image_tokens: list[tuple[Tensor, Tensor]] | None = None,
) -> tuple[Tensor, Tensor]:
    token_blocks = [latent_token.unsqueeze(0)]
    pos_embed_blocks = [self.encoder_1d_feature_pos_embed.weight[0:1].unsqueeze(1)]

    next_1d_pos_index = 1
    if robot_state is not None:
        token_blocks.append(robot_state.unsqueeze(0))
        pos_embed_blocks.append(
            self.encoder_1d_feature_pos_embed.weight[next_1d_pos_index : next_1d_pos_index + 1].unsqueeze(1)
        )
        next_1d_pos_index += 1

    if env_state is not None:
        token_blocks.append(env_state.unsqueeze(0))
        pos_embed_blocks.append(
            self.encoder_1d_feature_pos_embed.weight[next_1d_pos_index : next_1d_pos_index + 1].unsqueeze(1)
        )

    if image_tokens is not None:
        for cam_features, cam_pos_embed in image_tokens:
            token_blocks.append(cam_features)
            pos_embed_blocks.append(cam_pos_embed)

    return torch.cat(token_blocks, dim=0), torch.cat(pos_embed_blocks, dim=0)


def _act_forward(self, batch: dict[str, Tensor]) -> tuple[Tensor, tuple[Tensor, Tensor] | tuple[None, None]]:
    """A forward pass through the Action Chunking Transformer (with optional VAE encoder).

    `batch` should have the following structure:
    {
        [robot_state_feature] (optional): (B, state_dim) batch of robot states.

        [image_features]: (B, n_cameras, C, H, W) batch of images.
            AND/OR
        [env_state_feature]: (B, env_dim) batch of environment states.

        [action_feature] (optional, only if training with VAE): (B, chunk_size, action dim) batch of actions.
    }

    Returns:
        (B, chunk_size, action_dim) batch of action sequences
        Tuple containing the latent PDF's parameters (mean, log(σ²)) both as (B, L) tensors where L is the
        latent dimension.
    """
    if self.config.use_vae and self.training:
        assert ACTION in batch, (
            "actions must be provided when using the variational objective in training mode."
        )

    use_fp16_inference_path = self.config.dtype == "float16" and not self.training
    if use_fp16_inference_path:
        self._optimize_vision_path_for_inference()

    batch_size = batch[OBS_IMAGES][0].shape[0] if OBS_IMAGES in batch else batch[OBS_ENV_STATE].shape[0]

    # Prepare the latent for input to the transformer encoder.
    if self.config.use_vae and ACTION in batch and self.training:
        # Prepare the input to the VAE encoder: [cls, *joint_space_configuration, *action_sequence].
        cls_embed = einops.repeat(
            self.vae_encoder_cls_embed.weight, "1 d -> b 1 d", b=batch_size
        )  # (B, 1, D)
        if self.config.robot_state_feature:
            robot_state_embed = self.vae_encoder_robot_state_input_proj(batch[OBS_STATE])
            robot_state_embed = robot_state_embed.unsqueeze(1)  # (B, 1, D)
        action_embed = self.vae_encoder_action_input_proj(batch[ACTION])  # (B, S, D)

        if self.config.robot_state_feature:
            vae_encoder_input = [cls_embed, robot_state_embed, action_embed]  # (B, S+2, D)
        else:
            vae_encoder_input = [cls_embed, action_embed]
        vae_encoder_input = torch.cat(vae_encoder_input, axis=1)

        # Prepare fixed positional embedding.
        # Note: detach() shouldn't be necessary but leaving it the same as the original code just in case.
        pos_embed = self.vae_encoder_pos_enc.clone().detach()  # (1, S+2, D)

        # Prepare key padding mask for the transformer encoder. We have 1 or 2 extra tokens at the start of the
        # sequence depending whether we use the input states or not (cls and robot state)
        # False means not a padding token.
        cls_joint_is_pad = torch.full(
            (batch_size, 2 if self.config.robot_state_feature else 1),
            False,
            device=batch[OBS_STATE].device,
        )
        key_padding_mask = torch.cat([cls_joint_is_pad, batch["action_is_pad"]], axis=1)  # (bs, seq+1 or 2)

        # Forward pass through VAE encoder to get the latent PDF parameters.
        cls_token_out = self.vae_encoder(
            vae_encoder_input.permute(1, 0, 2),
            pos_embed=pos_embed.permute(1, 0, 2),
            key_padding_mask=key_padding_mask,
        )[0]  # select the class token, with shape (B, D)
        latent_pdf_params = self.vae_encoder_latent_output_proj(cls_token_out)
        mu = latent_pdf_params[:, : self.config.latent_dim]
        # This is 2log(sigma). Done this way to match the original implementation.
        log_sigma_x2 = latent_pdf_params[:, self.config.latent_dim :]

        # Sample the latent with the reparameterization trick.
        latent_sample = mu + log_sigma_x2.div(2).exp() * torch.randn_like(mu)
    else:
        # When not using the VAE encoder, we set the latent to be all zeros.
        mu = log_sigma_x2 = None
        # TODO(rcadene, alexander-soare): remove call to `.to` to speedup forward ; precompute and use buffer
        if use_fp16_inference_path:
            latent_sample = self._get_zero_latent(
                batch_size,
                batch[OBS_STATE].dtype,
                batch[OBS_STATE].device,
            )
        else:
            latent_sample = torch.zeros([batch_size, self.config.latent_dim], dtype=torch.float32).to(
                batch[OBS_STATE].device
            )

    # Prepare transformer encoder inputs.
    if use_fp16_inference_path:
        latent_token = self.encoder_latent_input_proj(latent_sample)
        robot_state_token = None
        env_state_token = None
        if self.config.robot_state_feature:
            robot_state_token = self.encoder_robot_state_input_proj(batch[OBS_STATE])
        if self.config.env_state_feature:
            env_state_token = self.encoder_env_state_input_proj(batch[OBS_ENV_STATE])

        image_token_blocks = (
            self._extract_multi_image_tokens(batch[OBS_IMAGES]) if self.config.image_features else []
        )
        encoder_in_tokens, encoder_in_pos_embed = self._build_encoder_inputs(
            latent_token,
            robot_state=robot_state_token,
            env_state=env_state_token,
            image_tokens=image_token_blocks,
        )
    else:
        encoder_in_tokens = [self.encoder_latent_input_proj(latent_sample)]
        encoder_in_pos_embed = list(self.encoder_1d_feature_pos_embed.weight.unsqueeze(1))
        if self.config.robot_state_feature:
            encoder_in_tokens.append(self.encoder_robot_state_input_proj(batch[OBS_STATE]))
        if self.config.env_state_feature:
            encoder_in_tokens.append(self.encoder_env_state_input_proj(batch[OBS_ENV_STATE]))

        if self.config.image_features:
            for img in batch[OBS_IMAGES]:
                cam_features = self.backbone(img)["feature_map"]
                cam_pos_embed = self.encoder_cam_feat_pos_embed(cam_features).to(dtype=cam_features.dtype)
                cam_features = self.encoder_img_feat_input_proj(cam_features)
                cam_features = einops.rearrange(cam_features, "b c h w -> (h w) b c")
                cam_pos_embed = einops.rearrange(cam_pos_embed, "b c h w -> (h w) b c")
                encoder_in_tokens.extend(list(cam_features))
                encoder_in_pos_embed.extend(list(cam_pos_embed))

        encoder_in_tokens = torch.stack(encoder_in_tokens, axis=0)
        encoder_in_pos_embed = torch.stack(encoder_in_pos_embed, axis=0)

    # Forward pass through the transformer modules.
    encoder_out = self.encoder(encoder_in_tokens, pos_embed=encoder_in_pos_embed)
    # TODO(rcadene, alexander-soare): remove call to `device` ; precompute and use buffer
    if use_fp16_inference_path:
        decoder_in = self._get_decoder_input(
            batch_size,
            encoder_in_pos_embed.dtype,
            encoder_in_pos_embed.device,
        )
    else:
        decoder_in = torch.zeros(
            (self.config.chunk_size, batch_size, self.config.dim_model),
            dtype=encoder_in_pos_embed.dtype,
            device=encoder_in_pos_embed.device,
        )
    decoder_out = self.decoder(
        decoder_in,
        encoder_out,
        encoder_pos_embed=encoder_in_pos_embed,
        decoder_pos_embed=self.decoder_pos_embed.weight.unsqueeze(1),
    )

    # Move back to (B, S, C).
    decoder_out = decoder_out.transpose(0, 1)

    actions = self.action_head(decoder_out)

    return actions, (mu, log_sigma_x2)


_MODEL_RUNTIME_INSTALLED = False
_RECORD_RUNTIME_INSTALLED = False
_ORIGINAL_CONFIG_FROM_PRETRAINED = PreTrainedConfig.from_pretrained.__func__
_ORIGINAL_MAKE_POLICY = None
_ORIGINAL_MAKE_PRE_POST_PROCESSORS = None
_ORIGINAL_PREDICT_ACTION = None


def _initialize_spine_state(model: ACT) -> None:
    model._vision_path_optimized = False
    model._mkldnn_backbone = None
    model._mkldnn_encoder_img_feat_input_proj = None
    model._mkldnn_vision_dtype = None
    model._spinednn_visual_contexts = {}
    model._cached_empty_latent = None
    model._cached_decoder_input = None
    model._cached_cam_pos_embed = {}


def install_model_runtime() -> None:
    """Install process-local ACT method overrides without modifying files under src/lerobot."""
    global _MODEL_RUNTIME_INSTALLED
    if _MODEL_RUNTIME_INSTALLED:
        return

    ACTConfig.dtype = "float16"
    ACT._invalidate_mkldnn_vision_path = _invalidate_mkldnn_vision_path
    ACT._invalidate_inference_caches = _invalidate_inference_caches
    ACT._optimize_vision_path_for_inference = _optimize_vision_path_for_inference
    ACT._get_zero_latent = _get_zero_latent
    ACT._get_decoder_input = _get_decoder_input
    ACT._should_use_mkldnn_vision_fast_path = _should_use_mkldnn_vision_fast_path
    ACT._ensure_mkldnn_vision_modules = _ensure_mkldnn_vision_modules
    ACT._should_use_spinednn_visual_context = _should_use_spinednn_visual_context
    ACT._ensure_spinednn_visual_context = _ensure_spinednn_visual_context
    ACT._get_camera_position_embedding = _get_camera_position_embedding
    ACT._extract_image_features = _extract_image_features
    ACT._extract_image_tokens = _extract_image_tokens
    ACT._extract_multi_image_tokens = _extract_multi_image_tokens
    ACT._build_encoder_inputs = _build_encoder_inputs
    ACT.forward = _act_forward
    _MODEL_RUNTIME_INSTALLED = True


def prepare_policy_for_spinetorch(policy: ACTPolicy) -> ACTPolicy:
    """Convert a loaded ACT policy to the optimized FP16 inference runtime."""
    install_model_runtime()
    policy.config.dtype = "float16"
    _initialize_spine_state(policy.model)
    policy.eval()
    policy.model._optimize_vision_path_for_inference()
    policy.half()
    policy.reset()
    return policy


def _sanitize_cli_overrides(policy_kwargs: dict[str, Any]) -> dict[str, Any]:
    sanitized = dict(policy_kwargs)
    overrides = sanitized.get("cli_overrides")
    if overrides:
        sanitized["cli_overrides"] = [item for item in overrides if "dtype" not in item]
    return sanitized


def _fp16_config_from_pretrained(cls, pretrained_name_or_path, **policy_kwargs):
    """Load old or converted ACT configs while keeping dtype process-local."""
    policy_kwargs = _sanitize_cli_overrides(policy_kwargs)
    model_path = Path(str(pretrained_name_or_path)).expanduser()
    config_path = model_path / "config.json"

    if config_path.is_file():
        config_data = json.loads(config_path.read_text())
        if "dtype" in config_data:
            config_data.pop("dtype")
            with TemporaryDirectory(prefix="lerobot-act-fp16-config-") as tmp_dir:
                Path(tmp_dir, "config.json").write_text(json.dumps(config_data))
                config = _ORIGINAL_CONFIG_FROM_PRETRAINED(cls, tmp_dir, **policy_kwargs)
        else:
            config = _ORIGINAL_CONFIG_FROM_PRETRAINED(cls, pretrained_name_or_path, **policy_kwargs)
    else:
        config = _ORIGINAL_CONFIG_FROM_PRETRAINED(cls, pretrained_name_or_path, **policy_kwargs)

    if isinstance(config, ACTConfig):
        config.dtype = "float16"
    return config


def load_fp16_policy(model_path: str | Path, device: str = "cpu") -> ACTPolicy:
    """Load an ACT checkpoint through the process-local FP16 runtime."""
    install_model_runtime()
    config = _fp16_config_from_pretrained(PreTrainedConfig, model_path)
    config.device = device
    policy = ACTPolicy.from_pretrained(model_path, config=config, local_files_only=True)
    return prepare_policy_for_spinetorch(policy)


def load_policy_for_fp16_conversion(model_path: str | Path, device: str = "cpu") -> ACTPolicy:
    """Load weights without applying inference-only fusion before checkpoint conversion."""
    config = _fp16_config_from_pretrained(PreTrainedConfig, model_path)
    config.dtype = "float32"
    config.device = device
    return ACTPolicy.from_pretrained(model_path, config=config, local_files_only=True)


def _with_fp16_preprocessor_overrides(overrides: dict[str, Any] | None) -> dict[str, Any]:
    merged = dict(overrides or {})
    device_overrides = dict(merged.get("device_processor", {}))
    device_overrides.setdefault("float_dtype", "float16")
    merged["device_processor"] = device_overrides
    return merged


def _make_fp16_policy(cfg, *args, **kwargs):
    if isinstance(cfg, ACTConfig):
        cfg.dtype = "float16"
    policy = _ORIGINAL_MAKE_POLICY(cfg, *args, **kwargs)
    if isinstance(policy, ACTPolicy):
        return prepare_policy_for_spinetorch(policy)
    return policy


def _make_fp16_pre_post_processors(policy_cfg, pretrained_path=None, **kwargs):
    if isinstance(policy_cfg, ACTConfig):
        kwargs["preprocessor_overrides"] = _with_fp16_preprocessor_overrides(
            kwargs.get("preprocessor_overrides")
        )
    return _ORIGINAL_MAKE_PRE_POST_PROCESSORS(
        policy_cfg,
        pretrained_path=pretrained_path,
        **kwargs,
    )


def _policy_has_cached_action(policy: Any) -> bool:
    if not isinstance(policy, ACTPolicy):
        return False
    if getattr(policy.config, "temporal_ensemble_coeff", None) is not None:
        return False
    action_queue = getattr(policy, "_action_queue", None)
    return action_queue is not None and len(action_queue) > 0


def _predict_timing_enabled() -> bool:
    return os.environ.get("LEROBOT_PREDICT_TIMING", "").lower() in {"1", "true", "yes", "on"}


def _predict_action_fp16(
    observation,
    policy,
    device,
    preprocessor,
    postprocessor,
    use_amp,
    task=None,
    robot_type=None,
):
    timing_enabled = _predict_timing_enabled()
    if _policy_has_cached_action(policy):
        start = time.perf_counter()
        with torch.inference_mode():
            action = postprocessor(policy._action_queue.popleft())
        if timing_enabled:
            logging.info(
                "Predict action timing path=cached total=%.1fms", (time.perf_counter() - start) * 1e3
            )
        return action

    start = time.perf_counter()
    action = _ORIGINAL_PREDICT_ACTION(
        observation=observation,
        policy=policy,
        device=device,
        preprocessor=preprocessor,
        postprocessor=postprocessor,
        use_amp=use_amp,
        task=task,
        robot_type=robot_type,
    )
    if timing_enabled:
        logging.info("Predict action timing path=full total=%.1fms", (time.perf_counter() - start) * 1e3)
    return action


def _enable_spinednn_plugin() -> None:
    import spinednn_torch_plugin as plugin

    plugin.only_enable(["addmm", "bmm", "softmax", "layer_norm", "relu", "add"])
    logging.info("Enabled SpineDNN torch plugin for ACT FP16 inference.")


def install_record_runtime() -> None:
    """Install FP16 overrides used by the custom lerobot-record entry point."""
    global _RECORD_RUNTIME_INSTALLED
    global _ORIGINAL_MAKE_POLICY, _ORIGINAL_MAKE_PRE_POST_PROCESSORS, _ORIGINAL_PREDICT_ACTION
    if _RECORD_RUNTIME_INSTALLED:
        return

    install_model_runtime()
    PreTrainedConfig.from_pretrained = classmethod(_fp16_config_from_pretrained)

    from lerobot.policies import factory
    from lerobot.utils import control_utils

    _ORIGINAL_MAKE_POLICY = factory.make_policy
    _ORIGINAL_MAKE_PRE_POST_PROCESSORS = factory.make_pre_post_processors
    _ORIGINAL_PREDICT_ACTION = control_utils.predict_action
    factory.make_policy = _make_fp16_policy
    factory.make_pre_post_processors = _make_fp16_pre_post_processors
    control_utils.predict_action = _predict_action_fp16

    _enable_spinednn_plugin()

    from lerobot.scripts import lerobot_record

    lerobot_record.make_policy = _make_fp16_policy
    lerobot_record.make_pre_post_processors = _make_fp16_pre_post_processors
    lerobot_record.predict_action = _predict_action_fp16
    _RECORD_RUNTIME_INSTALLED = True
