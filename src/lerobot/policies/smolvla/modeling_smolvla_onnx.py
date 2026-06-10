#!/usr/bin/env python

# ruff: noqa: E501, F401, F403, F541, F841

# Copyright 2025 HuggingFace Inc. team. All rights reserved.
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
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

"""
SmolVLA ONNX Inference Implementation

This module provides ONNX-based inference for SmolVLA policy models.
It loads the 9 ONNX model components and orchestrates the inference pipeline.

Usage:
    policy = SmolVLAONNXPolicy(
        config=config,
        onnx_model_dir="examples/onnx_inference/models/onnx/smolvla_base_onnx",
        use_spacemit_ep=False  # Set True for SpacemiT accelerated inference
    )
    action = policy.select_action(batch)
"""

import os
from collections import deque
from pathlib import Path
from typing import TypedDict, Unpack

import numpy as np
import onnxruntime as ort
import torch
from torch import Tensor

from lerobot.policies.pretrained import PreTrainedPolicy
from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
from lerobot.policies.utils import populate_queues
from lerobot.utils.constants import ACTION, OBS_IMAGES, OBS_LANGUAGE_ATTENTION_MASK, OBS_LANGUAGE_TOKENS, OBS_STATE


class ActionSelectKwargs(TypedDict, total=False):
    inference_delay: int | None
    prev_chunk_left_over: Tensor | None
    execution_horizon: int | None


def pad_vector(vector: Tensor, max_dim: int) -> Tensor:
    """Pad vector to max_dim with zeros."""
    if vector.shape[-1] >= max_dim:
        return vector[..., :max_dim]
    pad_size = max_dim - vector.shape[-1]
    return torch.nn.functional.pad(vector, (0, pad_size), value=0.0)


class SmolVLAONNXPolicy(PreTrainedPolicy):
    """
    ONNX-based SmolVLA policy for efficient inference.
    
    This class loads the 9 ONNX model components exported from SmolVLA:
    - smolvlm_vision.onnx: Vision encoder
    - smolvlm_text.onnx: Text encoder
    - smolvlm_expert_prefill.onnx: Action expert prefill stage
    - smolvlm_expert_decode.onnx: Action expert decode stage
    - state_projector.onnx: State projection
    - time_in_projector.onnx: Time input projection
    - time_out_projector.onnx: Time output projection
    - action_in_projector.onnx: Action input projection
    - action_out_projector.onnx: Action output projection
    """

    config_class = SmolVLAConfig
    name = "smolvla_onnx"

    def __init__(
        self,
        config: SmolVLAConfig,
        onnx_model_dir: str | Path,
        use_spacemit_ep: bool = False,
        **kwargs,
    ):
        """
        Initialize ONNX-based SmolVLA policy.
        
        Args:
            config: SmolVLA configuration
            onnx_model_dir: Directory containing the 9 ONNX model files
            use_spacemit_ep: Whether to use SpacemiT execution provider for acceleration
        """
        super().__init__(config)
        config.validate_features()
        self.config = config
        self.onnx_model_dir = Path(onnx_model_dir)
        
        # Setup ONNX Runtime providers
        if use_spacemit_ep:
            try:
                import spacemit_ort  # noqa: F401 - Required to register SpaceMITExecutionProvider
            except ImportError:
                raise ImportError(
                    "spacemit_ort package is required for SpaceMIT acceleration. "
                    "Install with: pip install --index-url https://git.spacemit.com/api/v4/projects/33/packages/pypi/simple spacemit-ort"
                )
            self.providers = ['SpaceMITExecutionProvider', 'CPUExecutionProvider']
            print("[SmolVLA ONNX] Using SpaceMIT accelerated execution provider")
        else:
            self.providers = ['CPUExecutionProvider']
            print("[SmolVLA ONNX] Using standard CPU execution provider")
        
        # Load all ONNX models
        self._load_onnx_models()
        self.reset()
        
        print(f"[SmolVLA ONNX] Loaded {len(self.sessions)} ONNX models from {onnx_model_dir}")

    def _load_onnx_models(self):
        """Load all 9 ONNX model components."""
        self.sessions = {}
        
        model_files = {
            'vision': 'smolvlm_vision.onnx',
            'text': 'smolvlm_text.onnx',
            'expert_prefill': 'smolvlm_expert_prefill.onnx',
            'expert_decode': 'smolvlm_expert_decode.onnx',
            'state_proj': 'state_projector.onnx',
            'time_in_proj': 'time_in_projector.onnx',
            'time_out_proj': 'time_out_projector.onnx',
            'action_in_proj': 'action_in_projector.onnx',
            'action_out_proj': 'action_out_projector.onnx',
        }
        
        # SpaceMIT EP has limited AI cores - only use it for the heaviest model (vision)
        # Other models either crash (expert_decode) or are too small to benefit
        spacemit_compatible = {'vision'}
        
        sess_options = ort.SessionOptions()
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        
        for key, filename in model_files.items():
            model_path = self.onnx_model_dir / filename
            if not model_path.exists():
                raise FileNotFoundError(f"ONNX model not found: {model_path}")
            
            # Only use SpaceMIT EP for compatible heavy models
            if key in spacemit_compatible and 'SpaceMITExecutionProvider' in self.providers:
                providers = self.providers
            else:
                providers = ['CPUExecutionProvider']
            
            try:
                session = ort.InferenceSession(
                    str(model_path),
                    sess_options=sess_options,
                    providers=providers
                )
                self.sessions[key] = session
                active = session.get_providers()
                print(f"[SmolVLA ONNX] Loaded {filename} ({active[0]})")
            except Exception as e:
                raise RuntimeError(f"Failed to load {filename}: {e}")

    def reset(self):
        """Reset the policy state (called when environment resets)."""
        self._queues = {
            ACTION: deque(maxlen=self.config.n_action_steps),
        }

    def _prepare_batch(self, batch: dict[str, Tensor]) -> dict[str, Tensor]:
        """Prepare batch for inference (handle any special preprocessing)."""
        # TODO: Add pi_aloha adaptation if needed
        return batch

    def prepare_images(self, batch: dict[str, Tensor]) -> tuple[Tensor, Tensor]:
        """
        Prepare images for vision encoder.
        
        Returns:
            images: Tensor of shape (batch_size, num_images, 3, H, W)
            img_masks: Tensor of shape (batch_size, num_images)
        """
        images_list = []
        masks_list = []
        
        for key in batch:
            if key.startswith(f"{OBS_IMAGES}."):
                img = batch[key]
                if img.ndim == 3:  # (C, H, W)
                    img = img.unsqueeze(0)  # (1, C, H, W)
                images_list.append(img)
                masks_list.append(torch.ones(img.shape[0], dtype=torch.bool, device=img.device))
        
        if not images_list:
            raise ValueError("No images found in batch")
        
        # Stack images
        images = torch.stack(images_list, dim=1)  # (batch_size, num_images, C, H, W)
        img_masks = torch.stack(masks_list, dim=1)  # (batch_size, num_images)
        
        return images, img_masks

    def prepare_state(self, batch: dict[str, Tensor]) -> Tensor:
        """Prepare and pad robot state."""
        state = batch[OBS_STATE][:, -1, :] if batch[OBS_STATE].ndim > 2 else batch[OBS_STATE]
        state = pad_vector(state, self.config.max_state_dim)
        return state

    def _to_numpy(self, tensor: Tensor) -> np.ndarray:
        """Convert PyTorch tensor to numpy array."""
        if isinstance(tensor, np.ndarray):
            return tensor
        return tensor.detach().cpu().numpy()

    def _from_numpy(self, array: np.ndarray, device: str = "cpu") -> Tensor:
        """Convert numpy array to PyTorch tensor."""
        return torch.from_numpy(array).to(device)

    def _run_onnx_session(self, session_key: str, inputs: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        """Run an ONNX session with given inputs."""
        session = self.sessions[session_key]
        
        # Get input names from session
        input_names = [inp.name for inp in session.get_inputs()]
        
        # Prepare inputs in correct order
        onnx_inputs = {name: inputs[name] for name in input_names if name in inputs}
        
        # Run inference
        outputs = session.run(None, onnx_inputs)
        
        # Map outputs to names
        output_names = [out.name for out in session.get_outputs()]
        return {name: output for name, output in zip(output_names, outputs)}

    def _get_action_chunk(
        self, batch: dict[str, Tensor], noise: Tensor | None = None, **kwargs: Unpack[ActionSelectKwargs]
    ) -> Tensor:
        """
        Generate action chunk using ONNX models.
        
        This orchestrates the full inference pipeline based on actual ONNX model interfaces:
        1. Vision encoding (per image)
        2. Text encoding
        3. State projection
        4. Combine embeddings for expert prefill
        5. Expert prefill (generates KV cache)
        6. Iterative denoising with expert decode
        """
        # Prepare inputs
        for k in batch:
            if k in self._queues and k != ACTION:
                batch[k] = torch.stack(list(self._queues[k]), dim=1)

        images, img_masks = self.prepare_images(batch)
        state = self.prepare_state(batch)
        lang_tokens = batch[f"{OBS_LANGUAGE_TOKENS}"]
        lang_masks = batch[f"{OBS_LANGUAGE_ATTENTION_MASK}"]

        batch_size = state.shape[0]
        device = state.device

        # Convert to numpy for ONNX
        state_np = self._to_numpy(state)
        lang_tokens_np = self._to_numpy(lang_tokens)

        # 1. Vision encoding - process each image separately
        # Vision model expects: image [1, 3, 512, 512] -> output_embeds [1, 64, 960]
        vision_embeds_list = []
        for i in range(images.shape[1]):  # For each camera
            img = images[:, i, :, :, :]  # [batch, 3, H, W]
            # Resize to 512x512 if needed
            if img.shape[-2:] != (512, 512):
                img = torch.nn.functional.interpolate(img, size=(512, 512), mode='bilinear', align_corners=False)
            img_np = self._to_numpy(img)
            
            vision_outputs = self._run_onnx_session('vision', {'image': img_np})
            vision_embeds_list.append(vision_outputs['output_embeds'])  # [1, 64, 960]
        
        # Concatenate vision embeddings from all cameras
        vision_embeds = np.concatenate(vision_embeds_list, axis=1)  # [1, num_cameras*64, 960]

        # 2. Text encoding
        # Text model expects: tokens [batch, seq_len] -> output_embeds [batch, seq_len, 960]
        text_outputs = self._run_onnx_session('text', {'tokens': lang_tokens_np})
        text_embeds = text_outputs['output_embeds']  # [batch, seq_len, 960]

        # 3. State projection
        # State model expects: state [1, 32] -> output [1, 960]
        state_outputs = self._run_onnx_session('state_proj', {'state': state_np})
        state_embeds = state_outputs['3']  # [1, 960]
        state_embeds = np.expand_dims(state_embeds, axis=1)  # [1, 1, 960]

        # 4. Combine embeddings: [vision_embeds, text_embeds, state_embeds]
        # Total sequence length should be 177 (as seen in prefill model)
        vlm_embeds = np.concatenate([vision_embeds, text_embeds, state_embeds], axis=1)  # [1, 177, 960]
        
        # Create attention mask and position IDs for prefill
        seq_len = vlm_embeds.shape[1]
        attention_mask = np.ones((batch_size, seq_len, seq_len), dtype=bool)
        position_ids = np.arange(seq_len, dtype=np.int64).reshape(1, -1)

        # 5. Expert prefill - generates KV cache
        # Prefill expects: vlm_embeds [1, 177, 960], attention_mask [1, 177, 177], position_ids [1, 177]
        # Returns: vlm_output_embeds [1, 177, 960] + 16 layers of KV cache
        prefill_outputs = self._run_onnx_session('expert_prefill', {
            'vlm_embeds': vlm_embeds,
            'attention_mask': attention_mask,
            'position_ids': position_ids,
        })
        
        # Extract KV cache from prefill
        past_key_values = {}
        for i in range(16):  # 16 layers
            past_key_values[f'past_key_{i}'] = prefill_outputs[f'present_key_{i}']
            past_key_values[f'past_value_{i}'] = prefill_outputs[f'present_value_{i}']

        # 6. Initialize noise for flow matching
        chunk_size = 50  # From ONNX model shapes
        if noise is None:
            actions_shape = (batch_size, chunk_size, self.config.max_action_dim)
            noise = torch.randn(actions_shape, device=device)
        
        x_t = self._to_numpy(noise)

        # 7. Iterative denoising
        num_steps = self.config.num_steps if hasattr(self.config, 'num_steps') else 10
        dt = -1.0 / num_steps

        for step in range(num_steps):
            time = 1.0 + step * dt
            time_scalar = np.array([[time]], dtype=np.float32)

            # Project action through action_in
            # action_in expects: action [1, 50, 32] -> output [1, 50, 720]
            action_in_outputs = self._run_onnx_session('action_in_proj', {'action': x_t})
            action_embeds = action_in_outputs['5']  # [1, 50, 720]

            # Create time embedding (concatenate action_embeds with time)
            # time_in expects: time [1, 50, 1440] -> output [1, 50, 720]
            time_expanded = np.tile(time_scalar, (1, chunk_size, 720))  # [1, 50, 720]
            time_action_concat = np.concatenate([action_embeds, time_expanded], axis=-1)  # [1, 50, 1440]
            
            time_in_outputs = self._run_onnx_session('time_in_proj', {'time': time_action_concat})
            time_embeds = time_in_outputs['5']  # [1, 50, 720]

            # time_out projection
            time_out_outputs = self._run_onnx_session('time_out_proj', {'time': time_embeds})
            expert_embeds = time_out_outputs['5']  # [1, 50, 720]

            # Expert decode with KV cache
            # Create attention mask and position IDs for decode
            decode_seq_len = chunk_size
            total_seq_len = seq_len + decode_seq_len  # 177 + 50 = 227
            decode_attention_mask = np.ones((batch_size, decode_seq_len, total_seq_len), dtype=bool)
            decode_position_ids = np.arange(seq_len, seq_len + decode_seq_len, dtype=np.int64).reshape(1, -1)

            # Prepare decode inputs
            decode_inputs = {
                'expert_embeds': expert_embeds,
                'attention_mask': decode_attention_mask,
                'position_ids': decode_position_ids,
            }
            decode_inputs.update(past_key_values)

            # Run expert decode
            decode_outputs = self._run_onnx_session('expert_decode', decode_inputs)
            expert_output_embeds = decode_outputs['expert_output_embeds']  # [1, 50, 720]

            # Update KV cache for next iteration
            for i in range(16):
                past_key_values[f'past_key_{i}'] = decode_outputs[f'present_key_{i}']
                past_key_values[f'past_value_{i}'] = decode_outputs[f'present_value_{i}']

            # Project back to action space
            # action_out expects: action [1, 50, 720] -> output [1, 50, 32]
            action_out_outputs = self._run_onnx_session('action_out_proj', {'action': expert_output_embeds})
            v_t = action_out_outputs['5']  # [1, 50, 32]

            # Update x_t (flow matching step)
            x_t = x_t + dt * v_t

        # Convert back to PyTorch
        actions = self._from_numpy(x_t, device=device)

        # Unpad actions to original action dimension
        # action_out model output dim is 32 (padded), real action dim comes from config or state dim
        if self.config.action_feature is not None:
            original_action_dim = self.config.action_feature.shape[0]
            actions = actions[:, :, :original_action_dim]
        # else: return full 32-dim output (user can slice as needed)

        return actions

    @torch.no_grad()
    def predict_action_chunk(
        self, batch: dict[str, Tensor], noise: Tensor | None = None, **kwargs: Unpack[ActionSelectKwargs]
    ) -> Tensor:
        """Predict full action chunk (for RTC or direct chunk execution)."""
        batch = self._prepare_batch(batch)
        self._queues = populate_queues(self._queues, batch, exclude_keys=[ACTION])
        actions = self._get_action_chunk(batch, noise, **kwargs)
        return actions

    @torch.no_grad()
    def select_action(
        self, batch: dict[str, Tensor], noise: Tensor | None = None, **kwargs: Unpack[ActionSelectKwargs]
    ) -> Tensor:
        """
        Select a single action given environment observations.
        
        This method manages actions in a queue and only calls predict_action_chunk
        when the queue is empty.
        """
        batch = self._prepare_batch(batch)
        self._queues = populate_queues(self._queues, batch, exclude_keys=[ACTION])

        if len(self._queues[ACTION]) == 0:
            actions = self._get_action_chunk(batch, noise)
            # Queue has shape (n_action_steps, batch_size, action_dim)
            self._queues[ACTION].extend(actions.transpose(0, 1)[: self.config.n_action_steps])

        return self._queues[ACTION].popleft()

    def forward(self, batch: dict[str, Tensor]) -> dict[str, Tensor]:
        """Forward pass (not used for inference-only ONNX policy)."""
        raise NotImplementedError("ONNX policy is inference-only. Use select_action() instead.")

    def get_optim_params(self) -> dict:
        """Get optimizer parameters (not applicable for ONNX inference)."""
        return {}

    def get_inference_stats(self) -> dict:
        """Get statistics about loaded ONNX models."""
        stats = {
            'num_models': len(self.sessions),
            'providers': self.providers,
            'models': {}
        }
        
        for key, session in self.sessions.items():
            inputs = [(inp.name, inp.shape, inp.type) for inp in session.get_inputs()]
            outputs = [(out.name, out.shape, out.type) for out in session.get_outputs()]
            stats['models'][key] = {
                'inputs': inputs,
                'outputs': outputs,
            }
        
        return stats
