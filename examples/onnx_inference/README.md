# onnx_inference

ONNX 推理示例目录，包含 ACT 策略模型的 Python 和 C++ 部署通路。

## 目录结构

```text
onnx_inference/
├── models/
│   ├── onnx/                       # onnx 模型目录
│   │   ├── act-fp32/
│   │   └── act-int8/
│   └── pytorch/                    # pytorch 模型目录
│       └── act/
├── tools/                          # 模型转换、模型量化、精度对比工具
│   ├── act_pytorch_to_onnx.py      # ACT PyTorch checkpoint 导出为 ONNX FP32
│   ├── compare_act_onnx.py         # ACT ONNX 与 PyTorch checkpoint 输出结果对比
│   ├── export_norm_stats.py        # 导出 ACT 归一化 / 标定参数给 C++ 使用
│   ├── make_act_test_inputs.py     # 生成 ACT C++ 离线测试输入
│   └── README.md
├── cpp/                            # C++ 推理链路
│   ├── act_evaluate.cpp
│   ├── act_benchmark.cpp
│   ├── CMakeLists.txt
│   └── README.md
├── python/                         # python 推理链路
│   ├── act_evaluate.py
│   ├── act_benchmark.py
│   └── README.md
└── README.md
```

## 性能数据

### Python Benchmark

| 模型     | 配置      | 平均延迟  |
| -------- | --------- | --------- |
| act-fp32 | EP 8 线程 | 1290.0 ms |
| act-int8 | EP 8 线程 | 210.9 ms  |

### C++ Benchmark

| 模型     | 配置      | 平均延迟  |
| -------- | --------- | --------- |
| act-fp32 | EP 8 线程 | 1288.3 ms |
| act-int8 | EP 8 线程 | 195.1 ms  |
