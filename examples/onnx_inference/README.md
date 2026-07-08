# onnx_inference

onnx 推理示例目录，提供 ACT 与 SmolVLA 策略模型的模型转换、数值验证、python 推理和 c++ 真机部署流程。

## 目录结构

```text
onnx_inference/
├── models/                         # 模型存放目录
├── tools/                          # 模型转换、模型量化、精度对比工具
├── cpp/                            # c++ 推理链路
├── python/                         # python 推理链路
├── requirements-act-pc.txt         # ACT pc 导出 / 量化 / 对比依赖
├── requirements-smolvla-pc.txt     # SmolVLA pc 导出 / 量化 / 对比依赖
├── requirements-act.txt            # ACT k3 python 依赖
├── requirements-smolvla.txt        # SmolVLA k3 python 依赖
└── README.md
```

## 使用入口

- 模型导出、量化、算子手术、数值对比：[tools/README.md](tools/README.md)。
- python benchmark、离线测试、真机运行：[python/README.md](python/README.md)。
- c++ benchmark、离线测试、真机运行：[cpp/README.md](cpp/README.md)。

## 复现资源

- 补丁版 spacemit ort sdk：[spacemit-ort.riscv64.2.0.4_yyx.tar.gz](https://archive.spacemit.com/spacemit-ai/model_zoo/vla/smolvla/spacemit-ort-sdk/spacemit-ort.riscv64.2.0.4_yyx.tar.gz)
- ACT / SmolVLA pytorch 和 onnx 模型：[tools/README.md](tools/README.md)。

## 部署步骤

### ACT

1. [安装 python 依赖](tools/README.md#act-install)。
2. [下载已测试模型用于复现 benchmark](tools/README.md#act-models)，可跳过。
3. [从 safetensors checkpoint 导出 onnx fp32 模型](tools/README.md#act-export-fp32)。
4. [量化得到 int8 模型](tools/README.md#act-quant-int8)。
5. [对比 pytorch / onnx 输出](tools/README.md#act-compare)。
6. [python 推理](python/README.md#act-pipeline)，推荐。
7. [c++ 推理](cpp/README.md#act-pipeline)。

### SmolVLA

1. [安装 python 依赖](tools/README.md#smolvla-install)。
2. [下载已测试模型用于复现 benchmark](tools/README.md#smolvla-models)，可跳过。
3. [从 safetensors checkpoint 导出 onnx fp32 模型](tools/README.md#smolvla-export-fp32)。
4. [下载补丁版 spacemit ort sdk](tools/README.md#smolvla-ort-sdk)。
5. [将 fp32 模型量化为 fp16](tools/README.md#smolvla-convert-fp16)，[并执行 fp16 算子手术](tools/README.md#smolvla-surgery-fp16)。
6. [对比 fp32 / fp16 输出](tools/README.md#smolvla-compare)。
7. [导出 smolvla_runtime.txt](tools/README.md#smolvla-runtime-metadata)。
8. [python 推理](python/README.md#smolvla-pipeline)。
9. [c++ 推理](cpp/README.md#smolvla-pipeline)，推荐。

## 性能数据

k3 性能测试数据参考。

### python benchmark

| 模型         | 配置      | 平均延迟  |
| ------------ | --------- | --------- |
| act-fp32     | ep 8 线程 | 1290.0 ms |
| act-int8     | ep 8 线程 | 210.9 ms  |
| smolvla-fp32 | ep 8 线程 | 8762.5 ms |

### c++ benchmark

| 模型         | 配置         | 平均延迟  |
| ------------ | ------------ | --------- |
| act-fp32     | ep 8 线程    | 1288.3 ms |
| act-int8     | ep 8 线程    | 195.1 ms  |
| smolvla-fp32 | ep 8 线程    | 9023.9 ms |
| smolvla-fp16 | ep204 8 线程 | 1127.5 ms |

SmolVLA 数据为 2 路相机模型测试结果，`--denoise-steps 10`，统计完整推理平均耗时，不包含模型加载。
