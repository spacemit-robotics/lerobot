"""SmolVLM 的 modality projector / connector：vision tokens → text hidden 空间。

结构（来自实测权重）：
- 单一 Linear: weight (960, 12288), bias=False

输入：(B, N=1024, 768) vision tokens
计算：
  1. pixel_shuffle by factor=4: reshape (B, 1024, 768) -> (B, 64, 16*768=12288)
     等价于把 32x32 grid 的 patch 每 4x4 一组合并
  2. modality_projection: Linear 12288 -> 960

输出：(B, 64, 960) 给 text 主干。3 个 camera 拼起来就是 192 个 image tokens。
"""

from __future__ import annotations

import torch
from torch import nn


class SmolVLMConnector(nn.Module):
    """对齐 lerobot SmolVLM2 的 model.connector。

    参数：
        scale_factor:    pixel_shuffle 的因子，SmolVLM2-500M 用 4
        vision_hidden:   SigLIP 输出维度 768
        text_hidden:     VLM text_model hidden 960
    """

    def __init__(
        self,
        scale_factor: int = 4,
        vision_hidden: int = 768,
        text_hidden: int = 960,
    ):
        super().__init__()
        self.scale_factor = scale_factor
        in_dim = vision_hidden * (scale_factor**2)
        self.modality_projection = nn.Sequential()
        self.modality_projection.proj = nn.Linear(in_dim, text_hidden, bias=False)

    def pixel_shuffle(self, x: torch.Tensor) -> torch.Tensor:
        """(B, N, D) 视觉 token，按 scale_factor 在空间维度合并。

        SmolVLM 的实现：N=H*W（H==W），把 H/s x W/s 的 grid 每个 cell 由 s*s 个 patch 组成，
        合并这些 patch 的 feature 到 cell 上。等价于
            reshape (B, H, W, D) -> (B, H/s, s, W/s, s, D)
                  permute to     (B, H/s, W/s, s, s, D)
                  reshape to     (B, (H/s)*(W/s), s*s*D)
        """
        b, n, d = x.shape
        side = int(n**0.5)
        s = self.scale_factor
        assert side * side == n, f"N={n} not a square"
        assert side % s == 0, f"side {side} not divisible by scale_factor {s}"
        x = x.view(b, side, side, d)
        x = x.view(b, side // s, s, side // s, s, d)
        x = x.permute(0, 1, 3, 2, 4, 5).contiguous()
        x = x.view(b, (side // s) * (side // s), s * s * d)
        return x

    def forward(self, vision_tokens: torch.Tensor) -> torch.Tensor:
        """(B, 1024, 768) -> (B, 64, 960)。"""
        x = self.pixel_shuffle(vision_tokens)
        return self.modality_projection.proj(x)
