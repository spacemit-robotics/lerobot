# C++

## 简介

C++ ONNX benchmark 与真机测试入口。

## 目录

```text
cpp/
├── act_benchmark.cpp                     # ACT dummy benchmark 入口
├── act_evaluate.cpp                      # ACT 真机/离线测试入口
├── CMakeLists.txt                        # C++ 构建入口
├── inputs/                               # ACT 离线测试输入样例
├── utils/
│   ├── common.h                          # Timer / npy 读取等公共工具
│   ├── act_runtime.h                     # ACT ONNX 运行时公共封装
│   └── act_stats.h                       # ACT 归一化 / 标定参数解析
└── README.md                             # 当前目录说明、构建与运行命令
```

## 构建环境

K3 板端推荐安装系统包，不要把 SpaceMIT ORT SDK 目录提交到仓库：

```bash
sudo apt-get install -y spacemit-onnxruntime python3-spacemit-ort
```

## 构建

```bash
cd cpp
mkdir -p build && cd build
cmake ..
make -j4
```

如需使用其他 ORT SDK 路径：

```bash
cmake .. -DSPACEMIT_ORT_DIR=/path/to/spacemit-ort-sdk
```

硬件版 ACT：

```bash
rm -rf build && mkdir build && cd build
cmake .. -DACT_ROBOT_HW=ON
make act_evaluate -j4
```

## 前置准备

ONNX 模型和离线输入：

- `models/onnx/act-fp32/act.onnx` 或 `models/onnx/act-int8/act.q.onnx`
- `models/onnx/act-fp32/act_norm_stats.txt`
- `cpp/inputs/images.npy`
- `cpp/inputs/state_deg.npy`

## benchmark

### ACT

```bash
./act_benchmark ../../models/onnx/act-fp32/act.onnx \
  --images-npy ../inputs/images.npy \
  --state-npy ../inputs/state_deg.npy \
  -s -t 8 -a "8;9;10;11;12;13;14;15" \
  -n 20 -w 3
```

> 将模型替换为 `../../models/onnx/act-int8/act.q.onnx` 测试 ACT INT8 性能。

## 真机运行

```bash
./act_evaluate ../../models/onnx/act-int8/act.q.onnx \
  --stats ../../models/onnx/act-fp32/act_norm_stats.txt \
  --port /dev/ttyACM0 --cam top=15 --cam wrist=13 \
  --fps 30 --episode-time 180 \
  -s -t 8 -a "8;9;10;11;12;13;14;15"
```
