# tools

该目录存放模型导出、量化、数值对比和其他部署辅助工具。除非单独说明，以下命令均在 `examples/onnx_inference/` 目录下执行。

## 通用环境

pc 环境：

```
conda create -n lerobot-venv python=3.11 -y
conda activate lerobot-venv
```

k3 环境：

```bash
python -m venv ~/.lerobot-venv
source ~/.lerobot-venv/bin/activate
```

## ACT pipeline

<a id="act-install"></a>

### 安装 python 依赖

pc 导出、量化和数值对比：

```bash
pip install -r requirements-act-pc.txt
```

k3 推理和数值对比：

```bash
pip install -r requirements-act.txt
```

<a id="act-models"></a>

### 复现模型下载（可选）

```bash
mkdir -p /tmp/act_models/extract models/pytorch models/onnx

curl -L \
  https://archive.spacemit.com/spacemit-ai/model_zoo/vla/act/onnx/models.tar.gz \
  -o /tmp/act_models/act_models.tar.gz

tar -xzf /tmp/act_models/act_models.tar.gz -C /tmp/act_models/extract

mv /tmp/act_models/extract/models/pytorch/act models/pytorch/act
mv /tmp/act_models/extract/models/onnx/act-fp32 models/onnx/act-fp32
mv /tmp/act_models/extract/models/onnx/act-int8 models/onnx/act-int8
```

<a id="act-export-fp32"></a>

### safetensors checkpoint -> onnx fp32

推荐在 pc 执行：

```bash
python tools/act_pytorch_to_onnx.py \
  --checkpoint models/pytorch/act/checkpoints/100000/pretrained_model \
  --output-dir models/onnx/act-fp32
```

参数说明：

- `--checkpoint`：ACT safetensors checkpoint 目录。
- `--output-dir`：导出的 onnx 目录。

<a id="act-quant-int8"></a>

### onnx fp32 -> int8

推荐在 pc 执行：

```bash
python -m xslim \
  -i models/onnx/act-fp32/act.onnx \
  -o models/onnx/act-int8/act.q.onnx \
  --dynq
```

参数说明：

- `-i`：输入 fp32 onnx 文件。
- `-o`：输出 int8 onnx 文件。
- `--dynq`：启用 int8 动态量化。

<a id="act-compare"></a>

### 数值对比

k3 上只运行 onnx，用于检查模型是否可执行、输出 shape 和 onnx 延迟：

```bash
python tools/compare_act_onnx.py \
  --model-dir models/onnx/act-fp32 \
  --checkpoint models/pytorch/act/checkpoints/100000/pretrained_model \
  --use-spacemit-ep \
  --ep-threads 8 --ep-affinity "8;9;10;11;12;13;14;15"
```

参数说明：

- `--use-spacemit-ep`：使用 spacemit ep；脚本默认开启。
- `--cpu`：强制使用 cpu ep。
- `--ep-threads`：ep 线程数，默认 `4`。
- `--ep-affinity`：ep 绑核列表，默认空字符串。

pc 上和 pytorch checkpoint 做数值对比：

```bash
python tools/compare_act_onnx.py \
  --model-dir models/onnx/act-fp32 \
  --checkpoint models/pytorch/act/checkpoints/100000/pretrained_model \
  --cpu --with-torch
```

也可以先保存 pytorch 参考输出，再在 k3 上加载 `.npy` 参考输出：

```bash
python tools/compare_act_onnx.py \
  --model-dir models/onnx/act-fp32 \
  --checkpoint models/pytorch/act/checkpoints/100000/pretrained_model \
  --cpu --with-torch --save-ref /tmp/act_ref.npy

python tools/compare_act_onnx.py \
  --model-dir models/onnx/act-fp32 \
  --checkpoint models/pytorch/act/checkpoints/100000/pretrained_model \
  --use-spacemit-ep \
  --ep-threads 8 --ep-affinity "8;9;10;11;12;13;14;15" \
  --ref-npy /tmp/act_ref.npy
```

验证 int8 产物时，将 `--model-dir` 替换为：

```bash
models/onnx/act-int8
```

### 导出 c++ 辅助文件

```bash
python tools/export_norm_stats.py \
  --checkpoint models/pytorch/act/checkpoints/100000/pretrained_model \
  --output models/onnx/act-fp32/act_norm_stats.txt

python tools/make_act_test_inputs.py \
  --model-dir models/onnx/act-fp32 \
  --stats models/onnx/act-fp32/act_norm_stats.txt \
  --out-dir cpp/inputs
```

生成文件说明：

- `models/onnx/act-fp32/act_norm_stats.txt`：c++ ACT 运行时使用的元数据文件。ACT onnx 图只包含网络本体，不包含图像 / state 归一化、action 反归一化和 SO-101 电机标定；c++ 离线测试和真机入口会从该文件读取相机顺序、输入输出维度、chunk 大小、`--n-action-steps` 默认值、均值 / 标准差和电机标定。
- `cpp/inputs/images.npy`：C++ 离线 benchmark / smoke test 使用的图像输入，shape 为 `(n_cam, 3, H, W)`，已经按 ACT 训练配置完成图像归一化。
- `cpp/inputs/state_deg.npy`：C++ 离线测试使用的机械臂 state，单位和 LeRobot SO-101 运行时一致，关节为 degrees，夹爪为 `0..100`；C++ 会再按 `act_norm_stats.txt` 做 MEAN_STD 归一化。
- `cpp/inputs/ref_action0.txt`：Python ONNX 参考输出，用于和 C++ 离线输出逐项对比，确认 C++ 预处理、推理和反归一化链路没有偏差。

## SmolVLA pipeline

<a id="smolvla-install"></a>

### 安装 python 依赖

pc 导出、量化和数值对比：

```bash
pip install -r requirements-smolvla-pc.txt
```

k3 推理和数值对比：

```bash
pip install -r requirements-smolvla.txt
```

<a id="smolvla-models"></a>

### 复现模型下载（可选）

```bash
mkdir -p models/pytorch models/onnx /tmp/smolvla_models

curl -L \
  https://archive.spacemit.com/spacemit-ai/model_zoo/vla/smolvla/models/pytorch/so101_smolvla_pick_green_cube_2cam.tar.gz \
  -o /tmp/smolvla_models/so101_smolvla_pick_green_cube_2cam.tar.gz
curl -L \
  https://archive.spacemit.com/spacemit-ai/model_zoo/vla/smolvla/models/onnx/so101_smolvla_pick_green_cube_2cam_100k_fp32.tar.gz \
  -o /tmp/smolvla_models/so101_smolvla_pick_green_cube_2cam_100k_fp32.tar.gz
curl -L \
  https://archive.spacemit.com/spacemit-ai/model_zoo/vla/smolvla/models/onnx/so101_smolvla_pick_green_cube_2cam_100k_fp16_surgeried.tar.gz \
  -o /tmp/smolvla_models/so101_smolvla_pick_green_cube_2cam_100k_fp16_surgeried.tar.gz

tar -xzf /tmp/smolvla_models/so101_smolvla_pick_green_cube_2cam.tar.gz -C models/pytorch
tar -xzf /tmp/smolvla_models/so101_smolvla_pick_green_cube_2cam_100k_fp32.tar.gz -C models/onnx
tar -xzf /tmp/smolvla_models/so101_smolvla_pick_green_cube_2cam_100k_fp16_surgeried.tar.gz -C models/onnx

ln -sfn so101_smolvla_pick_green_cube_2cam_100k_fp32 models/onnx/smolvla-fp32
ln -sfn so101_smolvla_pick_green_cube_2cam_100k_fp16_surgeried models/onnx/smolvla-fp16-surgeried
mkdir -p models/pytorch/smolvla/checkpoints/100000
ln -sfn ../../../so101_smolvla_pick_green_cube_2cam/checkpoints/100000/pretrained_model \
  models/pytorch/smolvla/checkpoints/100000/pretrained_model
```

<a id="smolvla-ort-sdk"></a>

### spacemit ort sdk 下载

只做 pc 侧 onnx 导出时可以跳过本节。在 k3 上跑 spacemit ep 或 c++ 真机 pipeline 时，需要下载并解压补丁版 spacemit ort sdk：

```bash
mkdir -p /tmp/smolvla_models

curl -L \
  https://archive.spacemit.com/spacemit-ai/model_zoo/vla/smolvla/spacemit-ort-sdk/spacemit-ort.riscv64.2.0.4_yyx.tar.gz \
  -o /tmp/smolvla_models/spacemit-ort.riscv64.2.0.4_yyx.tar.gz

tar -xzf /tmp/smolvla_models/spacemit-ort.riscv64.2.0.4_yyx.tar.gz -C ~
export SPACEMIT_ORT_DIR=~/spacemit-ort.riscv64.2.0.4_yyx
```

<a id="smolvla-export-fp32"></a>

### safetensors checkpoint -> onnx fp32

SmolVLA 导出为 4 个子图，部署时按顺序串起来：

- `vision_encoder`：每路相机图像各跑一次，得到视觉特征。
- `connector`：把视觉特征转成 image token，多路相机 token 按顺序拼接。
- `prefill_lm`：把 image token、任务文本和当前机械臂 state 编成 KV cache。
- `denoise_step`：复用 KV cache 做多步 denoise，得到 action chunk。

数据流：

```text
images[N] -> vision_encoder -> connector -> concat image_embs
concat image_embs + lang_tokens + lang_masks + state -> prefill_lm -> KV cache + prefix mask
noise x_t + timestep + KV cache + prefix mask -> denoise_step x denoise-steps -> actions
```

推荐在 pc 执行：

```bash
python tools/export_smolvla_4model_to_onnx.py \
  --checkpoint models/pytorch/smolvla/checkpoints/100000/pretrained_model \
  --output-dir models/onnx/smolvla-fp32 \
  --num-cameras 2 \
  --validate-load
```

参数说明：

- `--num-cameras`：写入 onnx 前缀的相机槽位数；默认从 checkpoint 的 `input_features + empty_cameras` 推断。
- `--validate-load`：导出后用 onnxruntime 加载检查；默认关闭。

<a id="smolvla-convert-fp16"></a>

### onnx fp32 -> fp16

推荐在 pc 执行：

```bash
python tools/convert_smolvla_fp32_to_fp16.py \
  --input-dir models/onnx/smolvla-fp32 \
  --output-dir models/onnx/smolvla-fp16
```

<a id="smolvla-surgery-fp16"></a>

### fp16 算子手术

为避免 fp16 模型在 spacemit ep 上推理出现漂移或者数值溢出，需要对 fp16 模型进行手术。

推荐在 pc 执行：

```bash
python tools/surgery_smolvla_fp16.py \
  --input-dir models/onnx/smolvla-fp16 \
  --output-dir models/onnx/smolvla-fp16-surgeried
```

手术会对三个子图做如下处理：

- `vision_encoder`：将 self-attention 核心融合为 `VisionSelfAttnNHWC`，并将 MLP 中的 GELU 近似子图替换为 ONNX `Gelu(approximate="tanh")`。
- `prefill_lm` / `denoise_step`：将 RMSNorm 子图中的关键计算提升为 fp32，并把 scatter 写入算子降解为等价子图。

<a id="smolvla-compare"></a>

### 数值对比

pc cpu 对比 fp32 与 fp16 手术版：

```bash
python tools/compare_smolvla_onnx.py \
  --fp32-dir models/onnx/smolvla-fp32 \
  --fp16-dir models/onnx/smolvla-fp16-surgeried \
  --cpu \
  --num-cameras 2 \
  --denoise-steps 10
```

k3 上验证 fp16 手术版 spacemit ep 数值：

```bash
python tools/compare_smolvla_onnx.py \
  --fp32-dir models/onnx/smolvla-fp32 \
  --fp16-dir models/onnx/smolvla-fp16-surgeried \
  --use-spacemit-ep \
  --spacemit-ort-dir ~/spacemit-ort.riscv64.2.0.4_yyx \
  --ep-threads 8 --ep-affinity "8;9;10;11;12;13;14;15" \
  --num-cameras 2 \
  --denoise-steps 10
```

<a id="smolvla-runtime-metadata"></a>

### 导出 c++ runtime metadata

c++ SmolVLA 真机入口需要任务 token、归一化参数和 so-101 标定。

首次部署或更换机械臂后，先校准 so-101 follower。`--robot.id` 会决定标定文件名，后续 `--calibration` 需要指向同一个 id 生成的 json：

```bash
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=my_awesome_follower_arm
```

校准完成后确认标定文件存在：

```bash
ls ~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_awesome_follower_arm.json
```

然后导出 c++ runtime metadata：

```bash
python tools/export_smolvla_runtime.py \
  --checkpoint models/pytorch/smolvla/checkpoints/100000/pretrained_model \
  --output models/onnx/smolvla_runtime.txt \
  --task "Place the green cube into the box" \
  --num-cameras 2 \
  --calibration ~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_awesome_follower_arm.json
```

参数说明：

- `--task`：部署时使用的任务文本，必填；需与训练 / 测试任务一致。
- `--calibration`：so-101 follower 标定 json，真机 c++ 部署必填。

`smolvla_runtime.txt` 中的 `image_keys` 来自 checkpoint 的 `input_features`。在线推理时相机参数名称需要与这些 key 的后缀一致，例如 `observation.images.top` 对应 `top`，`observation.images.camera1` 对应 `camera1`。如果 `NUM_CAMERAS` 大于真实 `image_keys` 数量，c++ 和 python 推理会把剩余槽位填成空图，用于兼容训练时配置的 empty camera。
