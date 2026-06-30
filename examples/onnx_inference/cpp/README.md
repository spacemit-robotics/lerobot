# c++

c++ onnx benchmark 与真机测试入口。以下命令除非单独说明，均在 `examples/onnx_inference/` 目录下执行。

## ACT pipeline

### 前置文件

```bash
# 模型
ls -l models/onnx/act-fp32/act.onnx
ls -l models/onnx/act-int8/act.q.onnx

# 输入
ls -l models/onnx/act-fp32/act_norm_stats.txt
ls -l cpp/inputs/images.npy
ls -l cpp/inputs/state_deg.npy
```

模型下载、onnx 导出、int8 量化、输入生成见 [tools/README.md](../tools/README.md#act-pipeline)。

### 安装依赖

```bash
sudo apt-get update
sudo apt-get install -y spacemit-onnxruntime python3-spacemit-ort
```

### 构建

k3 板端可使用系统 onnx runtime 和 spacemit ep：

```bash
cd cpp
mkdir -p build && cd build
cmake ..
make -j4
```

如需指定 ort sdk：

```bash
cmake .. -DSPACEMIT_ORT_DIR=/path/to/spacemit-ort-sdk
```

真机测试编译：

```bash
cd cpp
rm -rf build && mkdir build && cd build
cmake .. -DACT_ROBOT_HW=ON
make act_evaluate -j4
```

`ACT_ROBOT_HW=ON` 会打开 ACT 真机入口编译；不传时只编译离线 benchmark。

### benchmark

```bash
cd cpp/build
./act_benchmark ../../models/onnx/act-fp32/act.onnx \
  --images-npy ../inputs/images.npy \
  --state-npy ../inputs/state_deg.npy \
  -s -t 8 -a "8;9;10;11;12;13;14;15" \
  -n 20 -w 3
```

参数说明：

- `--images-npy`：离线图像输入。
- `--state-npy`：离线机械臂 state 输入。
- `-s`：启用 spacemit ep；不传则使用 cpu。
- `-t`：线程数，示例为 `8`。
- `-a`：线程绑核列表，示例为 `8;9;10;11;12;13;14;15`。
- `-n`：正式计时次数，示例为 `20`。
- `-w`：warmup 次数，示例为 `3`。

将模型替换为 `../../models/onnx/act-int8/act.q.onnx` 可测试 ACT int8。

### 真机运行

```bash
cd cpp/build
./act_evaluate ../../models/onnx/act-int8/act.q.onnx \
  --stats ../../models/onnx/act-fp32/act_norm_stats.txt \
  --port /dev/ttyACM0 --camera top=15 --camera wrist=13 \
  --fps 30 --episode-time 180 \
  -s -t 8 -a "8;9;10;11;12;13;14;15"
```

参数说明：

- `--stats`：ACT 归一化和 so-101 标定元数据，需提前生成。
- `--port`：机械臂串口，默认 `/dev/ttyACM0`。
- `--camera`：相机名和设备编号，与数采和训练时一致。
- `--fps`：动作控制频率，默认 `30`。
- `--episode-time`：真机运行时长，默认 `180` 秒。

## SmolVLA pipeline

### 前置文件

```bash
# 模型
ls -l models/onnx/smolvla-fp16-surgeried/vision_encoder.onnx
ls -l models/onnx/smolvla-fp16-surgeried/connector.onnx
ls -l models/onnx/smolvla-fp16-surgeried/prefill_lm.onnx
ls -l models/onnx/smolvla-fp16-surgeried/denoise_step.onnx

# 运行时
ls -l models/onnx/smolvla_runtime.txt
```

模型下载、导出、量化、手术和 `smolvla_runtime.txt` 生成见 [tools/README.md](../tools/README.md#smolvla-pipeline)。

### 安装依赖

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config libopencv-dev
```

默认使用补丁版 spacemit ort sdk，如果只使用系统 onnx runtime / spacemit ep，则额外安装系统包：

```bash
sudo apt-get install -y spacemit-onnxruntime python3-spacemit-ort
```

### 构建

为保证速度和数值正确，fp16 推理必须链接补丁版 spacemit ort sdk：

```bash
cd cpp
SPACEMIT_ORT_DIR=~/spacemit-ort.riscv64.2.0.3_yyx ./build_smolvla_robot_cpp.sh EP203
```

可选 ep：

- `EP203`：使用 `SPACEMIT_ORT_DIR` 或 `~/spacemit-ort.riscv64.2.0.3_yyx`
- `SYSTEM`：使用系统 onnx runtime / spacemit ep

### benchmark

测试完整推理延迟时显式打开 `--infer-every-tick`，避免 action queue 复用动作：

```bash
cd cpp
./run_smolvla_robot_pipeline.sh \
  --port /dev/ttyACM0 \
  --camera top=15 --camera wrist=13 \
  --warmup 1 --max-iters 3 --infer-every-tick --n-action-steps 50
```

参数说明：

- `--model-dir`：SmolVLA onnx 目录，默认 `models/onnx/smolvla-fp16-surgeried`。
- `--runtime`：运行时 metadata，默认 `models/onnx/smolvla_runtime.txt`。
- `--port`：机械臂串口，默认 `/dev/ttyACM0`。
- `--camera`：相机名和设备编号。
- `--spacemit-ort-dir`：spacemit ort sdk 目录，默认 `~/spacemit-ort.riscv64.2.0.3_yyx`。
- `--use-spacemit-ep`：脚本默认开启；如需 cpu 推理，传 `--cpu`。
- `--n-action-steps`：每次推理后放入 action queue 的动作步数，默认读取 `smolvla_runtime.txt`。
- `--denoise-steps`：去噪循环次数，默认 `10`；更大更慢但更接近参考采样。
- `--seed`：denoise 初始噪声种子，默认 `0`；python / c++ 使用同一套确定性噪声生成器。
- `--warmup`：正式计时前的合成输入 warmup 次数，默认 `0`。
- `--infer-every-tick`：每个控制周期都完整推理，用于完整推理 benchmark；真机常规运行通常不需要。

> 更多参数可运行 `./run_smolvla_robot_pipeline.sh --help` 查看。

### dry-run

`dry-run` 模式连接摄像头和机械臂，获取真实输入，但不会下发动作给电机。

```bash
cd cpp
./run_smolvla_robot_pipeline.sh \
  --port /dev/ttyACM0 \
  --camera top=15 --camera wrist=13 \
  --dry-run --max-iters 1 --n-action-steps 50 --print-actions
```

3cam 模型使用对应的模型目录、runtime metadata 和相机映射：

```bash
cd cpp
./run_smolvla_robot_pipeline.sh \
  --model-dir models/onnx/smolvla-3cam-fp16-surgeried \
  --runtime models/onnx/smolvla_runtime_3cam.txt \
  --port /dev/ttyACM0 \
  --camera camera1=3 --camera camera2=1 --camera camera3=5 \
  --dry-run --max-iters 1 --print-actions
```

### 真机运行

```bash
cd cpp
./run_smolvla_robot_pipeline.sh \
  --port /dev/ttyACM0 \
  --camera top=15 --camera wrist=13 \
  --n-action-steps 25 --print-actions
```

`--n-action-steps` 会影响 action queue 的刷新频率。推荐从 `25` 开始测试；如果动作过于频繁地重新规划，可适当增大该值，如果需要更快响应环境变化，可适当减小该值。不同任务、相机视角和训练数据分布下，最佳值可能不同。
