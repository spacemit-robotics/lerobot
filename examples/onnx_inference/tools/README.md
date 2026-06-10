# Tools

该目录存放模型导出、量化命令说明、精度对比和 C++ 部署辅助工具。

除非单独说明，以下命令均在 `examples/onnx_inference/` 目录下执行。

## 当前保留文件

```text
tools/
├── act_pytorch_to_onnx.py      # ACT PyTorch checkpoint 导出为 ONNX FP32
├── compare_act_onnx.py         # ACT ONNX 输出与 PyTorch / 参考结果对比
├── export_norm_stats.py        # 导出 ACT 归一化 / 标定参数给 C++ 使用
└── make_act_test_inputs.py     # 生成 ACT C++ 离线测试输入
```

## 导出工具（在 x86 设备运行以加速）

### 导出依赖安装

```bash
pip install -e ../../
pip install torch torchvision onnx onnxruntime safetensors spacemit_ort
```

### ACT PyTorch -> ONNX FP32

```bash
python tools/act_pytorch_to_onnx.py \
  --checkpoint models/pytorch/act/checkpoints/100000/pretrained_model \
  --output-dir models/onnx/act-fp32
```

## 量化工具（在 x86 设备运行以加速）

当前目录默认使用 **xslim 动态量化（`--dynq`）** 生成 INT8 模型。

### 安装 xslim

```bash
pip install xslim --index-url https://git.spacemit.com/api/v4/projects/33/packages/pypi/simple
```

### ACT 动态量化

```bash
python -m xslim \
  -i models/onnx/act-fp32/act.onnx \
  -o models/onnx/act-int8/act.q.onnx \
  --dynq
```

## 精度对比工具

### ACT

默认不加 `--with-torch` 时只运行 ONNX，用于在 K3 上检查模型是否可执行、
输出 shape 和 ONNX 延迟：

```bash
python tools/compare_act_onnx.py \
  --onnx models/onnx/act-fp32/act.onnx \
  --checkpoint models/pytorch/act/checkpoints/100000/pretrained_model \
  --use-spacemit-ep --ep-threads 8 --ep-affinity "8;9;10;11;12;13;14;15"
```

如果需要真正和 PyTorch checkpoint 做精度对比，在具备完整 PyTorch / lerobot
环境的机器上增加 `--with-torch`：

```bash
python tools/compare_act_onnx.py \
  --onnx models/onnx/act-fp32/act.onnx \
  --checkpoint models/pytorch/act/checkpoints/100000/pretrained_model \
  --with-torch
```

也可以先保存 PyTorch 参考输出，再在 K3 上加载 `.npy` 参考输出对比：

```bash
python tools/compare_act_onnx.py \
  --onnx models/onnx/act-fp32/act.onnx \
  --checkpoint models/pytorch/act/checkpoints/100000/pretrained_model \
  --with-torch --save-ref /tmp/act_ref.npy

python tools/compare_act_onnx.py \
  --onnx models/onnx/act-fp32/act.onnx \
  --checkpoint models/pytorch/act/checkpoints/100000/pretrained_model \
  --use-spacemit-ep --ep-threads 8 --ep-affinity "8;9;10;11;12;13;14;15" \
  --ref-npy /tmp/act_ref.npy
```

如果需要验证 INT8 产物，可将 `--onnx` 替换为：

```bash
models/onnx/act-int8/act.q.onnx
```

## 其他工具

### 导出 ACT 归一化 / 标定参数

```bash
python tools/export_norm_stats.py \
  --checkpoint models/pytorch/act/checkpoints/100000/pretrained_model \
  --output models/onnx/act-fp32/act_norm_stats.txt
```

### 生成 ACT C++ 离线输入

```bash
python tools/make_act_test_inputs.py \
  --onnx models/onnx/act-fp32/act.onnx \
  --stats models/onnx/act-fp32/act_norm_stats.txt \
  --out-dir cpp/inputs
```
