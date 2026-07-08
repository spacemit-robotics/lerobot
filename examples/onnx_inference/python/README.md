# python

python onnx benchmark、离线 smoke test 与真机测试入口。以下命令除非单独说明，均在 `examples/onnx_inference/` 目录下执行。

## 通用环境

```bash
python -m venv ~/.lerobot-venv
source ~/.lerobot-venv/bin/activate
```

## ACT pipeline

### 安装 python 依赖

```bash
pip install -r requirements-act.txt
```

### 前置文件

```bash
ls -l models/onnx/act-fp32/act.onnx
ls -l models/onnx/act-int8/act.q.onnx
ls -l models/pytorch/act/checkpoints/100000/pretrained_model
```

### benchmark

fp32：

```bash
cd python
python act_benchmark.py \
  --model-dir ../models/onnx/act-fp32 \
  --checkpoint ../models/pytorch/act/checkpoints/100000/pretrained_model \
  --use-spacemit-ep \
  --ep-threads 8 --ep-affinity "8;9;10;11;12;13;14;15" \
  --warmup 5 --iters 20
```

参数说明：

- `--model-dir`：ACT onnx 模型目录。
- `--checkpoint`：pytorch `pretrained_model` 目录，用于读取 config 和归一化参数。
- `--use-spacemit-ep`：使用 spacemit ep；默认开启。
- `--cpu`：强制使用 cpu ep。
- `--ep-threads`：ep 线程数，默认 `8`。
- `--ep-affinity`：ep 绑核列表，默认 `8;9;10;11;12;13;14;15`。
- `--warmup`：正式计时前 warmup 次数，默认 `5`。
- `--iters`：正式计时次数，默认 `20`。

int8：

```bash
cd python
python act_benchmark.py \
  --model-dir ../models/onnx/act-int8 \
  --checkpoint ../models/pytorch/act/checkpoints/100000/pretrained_model \
  --use-spacemit-ep \
  --ep-threads 8 --ep-affinity "8;9;10;11;12;13;14;15" \
  --warmup 5 --iters 20
```

### 真机运行

```bash
cd python
python act_evaluate.py \
  --model-dir ../models/onnx/act-int8 \
  --checkpoint ../models/pytorch/act/checkpoints/100000/pretrained_model \
  --port /dev/ttyACM0 --camera top=15 --camera wrist=13 \
  --use-spacemit-ep \
  --ep-threads 8 --ep-affinity "8;9;10;11;12;13;14;15"
```

参数说明：

- `--port`：机械臂串口，默认 `/dev/ttyACM0`。
- `--camera`：相机名和设备编号，与数采时一致。
- `--camera-fps`：相机采集帧率，默认 `30`，与数采时一致。
- `--fps`：动作控制频率，默认 `30`。
- `--episode-time`：真机运行时长，默认 `180` 秒。

## SmolVLA pipeline

### 安装 python 依赖

```bash
pip install -r requirements-smolvla.txt
```

### 前置文件

```bash
ls -l models/onnx/smolvla-fp32/
ls -l models/onnx/smolvla-fp16-surgeried/
ls -l models/pytorch/smolvla/checkpoints/100000/pretrained_model
```

### 使用建议

python 运行 SmolVLA 时推荐使用 fp32 模型，并通过系统 `spacemit_ort` 包注册 spacemit ep。FP16 图修复版依赖补丁版 spacemit ort sdk 才能保证数值正确；python 进程中延迟加载补丁版 `libspacemit_ep.so` 可能触发 static TLS 或 onnxruntime wheel 兼容问题，因此 fp16 部署建议使用 [c++ pipeline](../cpp/README.md#smolvla-pipeline)。

### benchmark

不连接机械臂和相机，只用合成 observation 跑完整 4-model pipeline：

```bash
cd python
python smolvla_evaluate.py \
  --no-robot --iters 1 \
  --model-dir ../models/onnx/smolvla-fp32 \
  --checkpoint ../models/pytorch/smolvla/checkpoints/100000/pretrained_model \
  --use-spacemit-ep \
  --ep-threads 8 --ep-affinity "8;9;10;11;12;13;14;15" \
  --warmup 1 \
  --denoise-steps 10 \
  --n-action-steps 50 \
  --infer-every-tick \
  --print-actions
```

参数说明：

- `--no-robot`：使用合成 observation，不连接机械臂和相机。
- `--denoise-steps`：每个 action chunk 的去噪循环次数，默认 `10`；更大更慢但更接近参考采样。
- `--n-action-steps`：每次推理后放入 action queue 的动作步数，默认读取 checkpoint。
- `--infer-every-tick`：每个控制周期都重新跑一次完整 4-model pipeline。用于 benchmark 时可以测到真实 full-inference 延迟。

### dry-run

`dry-run` 模式会连接机械臂和相机、读取真实 observation，但不会下发动作：

```bash
cd python
python smolvla_evaluate.py \
  --dry-run --max-iters 1 \
  --port /dev/ttyACM0 \
  --camera top=/dev/video15 --camera wrist=/dev/video13 \
  --camera-fps 30 --camera-fourcc MJPG \
  --use-spacemit-ep \
  --ep-threads 8 --ep-affinity "8;9;10;11;12;13;14;15" \
  --model-dir ../models/onnx/smolvla-fp32 \
  --checkpoint ../models/pytorch/smolvla/checkpoints/100000/pretrained_model \
  --denoise-steps 10 \
  --n-action-steps 50 \
  --print-actions
```

参数说明：

- `--dry-run`：连接真实机械臂和相机、读取真实 observation，但不下发动作。
- `--max-iters`：最大控制循环次数，默认不限制；dry-run 建议先用 `1`。
- `--port`：机械臂串口，连接硬件时必填。
- `--camera`：相机名和设备路径，必须与训练和 `smolvla_runtime.txt` 的 `image_keys` 后缀一致。
- `--camera-fps`：相机采集帧率，默认 `25`。
- `--camera-fourcc`：相机格式，默认 `MJPG`。

### 真机运行

去掉 dry-run 限制后即为真机执行：

```bash
cd python
python smolvla_evaluate.py \
  --port /dev/ttyACM0 \
  --camera top=/dev/video15 --camera wrist=/dev/video13 \
  --camera-fps 30 --camera-fourcc MJPG \
  --use-spacemit-ep \
  --ep-threads 8 --ep-affinity "8;9;10;11;12;13;14;15" \
  --model-dir ../models/onnx/smolvla-fp32 \
  --checkpoint ../models/pytorch/smolvla/checkpoints/100000/pretrained_model \
  --denoise-steps 10 \
  --n-action-steps 50 \
  --print-actions
```
