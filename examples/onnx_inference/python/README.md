# Python

## 简介

Python ONNX benchmark 与真机测试入口。

## 目录

```text
python/
├── act_evaluate.py      # ACT 真机测试入口
├── act_benchmark.py     # ACT dummy benchmark 入口
├── utils/
│   ├── __init__.py      # utils 模块入口
│   └── act_runtime.py   # Python ONNX 运行时公共辅助函数
└── README.md            # 当前目录说明、benchmark 与真机命令
```

## 环境

```bash
# 创建虚拟环境
python -m venv ~/.lerobot-venv
source ~/.lerobot-venv/bin/activate

# 安装 python 依赖
pip install torch==2.7.1
pip install torchvision==0.22.0
pip install pyarrow==23.0.0
pip install "lerobot[feetech]"

# 安装 onnxruntime / spacemit onnxruntime
pip install onnxruntime spacemit_ort safetensors
```

## 前置准备

onnx 模型和 pytorch 检查点：

- ACT FP32：`models/onnx/act-fp32/act.onnx`
- ACT INT8：`models/onnx/act-int8/act.q.onnx`
- ACT checkpoint：`models/pytorch/act/checkpoints/100000/pretrained_model`

## benchmark

### ACT

#### FP32 测试

```bash
cd python
python act_benchmark.py \
  --onnx ../models/onnx/act-fp32/act.onnx \
  --checkpoint ../models/pytorch/act/checkpoints/100000/pretrained_model \
  --use-spacemit-ep --ep-threads 8 --ep-affinity "8;9;10;11;12;13;14;15" \
  --warmup 5 --iters 20
```

#### INT8 测试

```bash
cd python
python act_benchmark.py \
  --onnx ../models/onnx/act-int8/act.q.onnx \
  --checkpoint ../models/pytorch/act/checkpoints/100000/pretrained_model \
  --use-spacemit-ep --ep-threads 8 --ep-affinity "8;9;10;11;12;13;14;15" \
  --warmup 5 --iters 20
```

## 真机运行

```bash
cd python
python act_evaluate.py \
  --onnx ../models/onnx/act-int8/act.q.onnx \
  --checkpoint ../models/pytorch/act/checkpoints/100000/pretrained_model \
  --port /dev/ttyACM0 --cam top=15 --cam wrist=13 \
  --use-spacemit-ep --ep-threads 8 --ep-affinity "8;9;10;11;12;13;14;15"
```
