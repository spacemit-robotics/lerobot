"""手写 SigLIP-Base 视觉编码器（与 SmolVLM2-500M 的 vision_model 一致）。

配置（来自 model.safetensors 实测）：
  hidden_size       = 768
  num_layers        = 12
  num_heads         = 12 (head_dim=64)
  patch_size        = 16
  image_size        = 512  → 32x32 = 1024 patches
  intermediate_size = 3072 (MLP FFN)
  activation        = GELU
  norm              = LayerNorm (with bias)

每层 self_attn 都有 bias（不同于 Llama decoder）。MLP 是 fc1+GELU+fc2（不是 SwiGLU）。
"""

from __future__ import annotations

import torch
import torch.nn.functional as functional
from torch import nn


class SiglipEmbeddings(nn.Module):
    def __init__(self, hidden_size: int = 768, patch_size: int = 16, num_positions: int = 1024):
        super().__init__()
        self.patch_embedding = nn.Conv2d(
            in_channels=3,
            out_channels=hidden_size,
            kernel_size=patch_size,
            stride=patch_size,
            bias=True,
        )
        self.position_embedding = nn.Embedding(num_positions, hidden_size)
        self.register_buffer("position_ids", torch.arange(num_positions).unsqueeze(0), persistent=False)

    def forward(self, pixel_values: torch.Tensor) -> torch.Tensor:
        # (B, 3, H, W) -> (B, hidden, h, w) -> (B, hidden, N) -> (B, N, hidden)
        x = self.patch_embedding(pixel_values)
        x = x.flatten(2).transpose(1, 2)
        return x + self.position_embedding(self.position_ids[:, : x.shape[1]])


class SiglipAttention(nn.Module):
    """标准 multi-head self-attention（不是 GQA）。"""

    def __init__(self, hidden_size: int = 768, num_heads: int = 12):
        super().__init__()
        self.hidden_size = hidden_size
        self.num_heads = num_heads
        self.head_dim = hidden_size // num_heads
        self.q_proj = nn.Linear(hidden_size, hidden_size, bias=True)
        self.k_proj = nn.Linear(hidden_size, hidden_size, bias=True)
        self.v_proj = nn.Linear(hidden_size, hidden_size, bias=True)
        self.out_proj = nn.Linear(hidden_size, hidden_size, bias=True)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        b, n, _ = x.shape
        q = self.q_proj(x).view(b, n, self.num_heads, self.head_dim).transpose(1, 2)
        k = self.k_proj(x).view(b, n, self.num_heads, self.head_dim).transpose(1, 2)
        v = self.v_proj(x).view(b, n, self.num_heads, self.head_dim).transpose(1, 2)

        # (B, H, N, D) @ (B, H, D, N) -> (B, H, N, N)
        attn = (q @ k.transpose(-2, -1)) * (self.head_dim**-0.5)
        attn = functional.softmax(attn.to(torch.float32), dim=-1).to(x.dtype)
        out = attn @ v  # (B, H, N, D)
        out = out.transpose(1, 2).reshape(b, n, self.hidden_size)
        return self.out_proj(out)


class SiglipMLP(nn.Module):
    def __init__(self, hidden_size: int = 768, intermediate_size: int = 3072):
        super().__init__()
        self.fc1 = nn.Linear(hidden_size, intermediate_size, bias=True)
        self.fc2 = nn.Linear(intermediate_size, hidden_size, bias=True)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.fc2(functional.gelu(self.fc1(x), approximate="tanh"))


class SiglipEncoderLayer(nn.Module):
    def __init__(self, hidden_size: int = 768, num_heads: int = 12, intermediate_size: int = 3072):
        super().__init__()
        self.layer_norm1 = nn.LayerNorm(hidden_size, eps=1e-6)
        self.self_attn = SiglipAttention(hidden_size, num_heads)
        self.layer_norm2 = nn.LayerNorm(hidden_size, eps=1e-6)
        self.mlp = SiglipMLP(hidden_size, intermediate_size)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = x + self.self_attn(self.layer_norm1(x))
        x = x + self.mlp(self.layer_norm2(x))
        return x


class SiglipEncoder(nn.Module):
    def __init__(
        self, num_layers: int = 12, hidden_size: int = 768, num_heads: int = 12, intermediate_size: int = 3072
    ):
        super().__init__()
        self.layers = nn.ModuleList(
            [SiglipEncoderLayer(hidden_size, num_heads, intermediate_size) for _ in range(num_layers)]
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        for layer in self.layers:
            x = layer(x)
        return x


class SiglipVisionModel(nn.Module):
    """SigLIP-Base 视觉编码器：image (B, 3, 512, 512) -> tokens (B, 1024, 768)。"""

    def __init__(
        self,
        hidden_size: int = 768,
        num_layers: int = 12,
        num_heads: int = 12,
        intermediate_size: int = 3072,
        patch_size: int = 16,
        image_size: int = 512,
    ):
        super().__init__()
        num_positions = (image_size // patch_size) ** 2
        self.embeddings = SiglipEmbeddings(hidden_size, patch_size, num_positions)
        self.encoder = SiglipEncoder(num_layers, hidden_size, num_heads, intermediate_size)
        self.post_layernorm = nn.LayerNorm(hidden_size, eps=1e-6)

    def forward(self, pixel_values: torch.Tensor) -> torch.Tensor:
        x = self.embeddings(pixel_values)
        x = self.encoder(x)
        x = self.post_layernorm(x)
        return x  # (B, 1024, 768)
