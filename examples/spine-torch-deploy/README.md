# SpineTorch ACT FP16 deployment

This directory provides K3 SpineTorch ACT FP16 inference without changing files under `src/lerobot`.
The dedicated entry points install process-local Python extensions for the ACT model, processor overrides,
SpineDNN plugin, and cached action path. A normal LeRobot process remains unchanged.

## Directory layout

- `spine_runtime.py`: process-local ACT FP16 model and inference extensions.
- `lerobot_record_fp16.py`: custom `lerobot-record` entry point that installs the extensions before parsing
  the normal LeRobot arguments.
- `run_lerobot_fp16_a100x8.sh`: configures the K3 AI cores and dispatches `lerobot-record` to the custom
  entry point.
- `convert_act_model_to_fp16.py`: creates a separate FP16 checkpoint without changing the FP32 model.
- `benchmark_act_dummy_inference.py`: benchmarks ACT with synthetic state and camera observations.
- `test/test_act_k3_fp16_record.py`: focused tests for the process-local runtime and cached action path.

## Prerequisites

Install the runtime libraries once on the K3 system:

```bash
sudo apt update
sudo apt install -y libopenblas0-pthread libomp5
```

Use a dedicated Python 3.12 environment for the latest SpineTorch packages. The validated environment
path is `~/.lerobot-venv-torh2.8`, with Torch 2.8.0, torchvision 0.23.0, and
`spine-torch-extension 2.8.0+spacemit.0`.
The historical `torh2.8` spelling is kept for compatibility with the validated K3 deployment environment.

Install the validated K3 wheels directly from SpaceMIT PyPI. The direct references pin the correct
Python 3.12 RISC-V wheels and verify their SHA256 hashes:

```bash
source ~/.lerobot-venv-torh2.8/bin/activate

python -m pip install --no-deps \
  "torch @ https://git.spacemit.com/api/v4/projects/33/packages/pypi/files/51484d46602b0e9537a5eb83c251ecd2227cea571127c2560cb8411bca154a42/torch-2.8.0%2Bspacemit.2-cp312-cp312-linux_riscv64.whl#sha256=51484d46602b0e9537a5eb83c251ecd2227cea571127c2560cb8411bca154a42" \
  "spine-torch-extension @ https://git.spacemit.com/api/v4/projects/33/packages/pypi/files/81ac3ccb44810bc6d8a89ff136daeda3b1527fa0c103d3c986059594f9e76a97/spine_torch_extension-2.8.0%2Bspacemit.0-cp312-cp312-linux_riscv64.whl#sha256=81ac3ccb44810bc6d8a89ff136daeda3b1527fa0c103d3c986059594f9e76a97"
```

The Torch wheel filename contains `2.8.0+spacemit.2`, while its package metadata reports `2.8.0`.
Using the exact PyPI file reference also avoids selecting another Torch 2.8 build from the same index.

No patch application or source-tree modification is required.
At startup, the dedicated entry point verifies the upstream `lerobot-record` module bindings before
installing its process-local overrides. It exits with an explicit compatibility error if that call path changes.

## Convert the model

```bash
source ~/.lerobot-venv-torh2.8/bin/activate

python examples/spine-torch-deploy/convert_act_model_to_fp16.py \
  --input-path outputs/train/so101_act_pick_green_cube_amp/checkpoints/100000/pretrained_model \
  --output-path outputs/train/so101_act_pick_green_cube_amp/checkpoints/100000/pretrained_model_fp16
```

## Benchmark FP16 inference

```bash
source ~/.lerobot-venv-torh2.8/bin/activate

./examples/spine-torch-deploy/run_lerobot_fp16_a100x8.sh \
  python examples/spine-torch-deploy/benchmark_act_dummy_inference.py \
  --model-path outputs/train/so101_act_pick_green_cube_amp/checkpoints/100000/pretrained_model_fp16 \
  --device cpu \
  --use-half \
  --keep-mkldnn \
  --use-spinednn \
  --warmup 3 \
  --iters 5
```

## Run `lerobot-record`

Use the dedicated launcher for the normal `lerobot-record` command and point the policy at the converted
FP16 checkpoint:

```bash
./examples/spine-torch-deploy/run_lerobot_fp16_a100x8.sh \
  lerobot-record \
  [robot and dataset arguments] \
  --policy.path=outputs/train/so101_act_pick_green_cube_amp/checkpoints/100000/pretrained_model_fp16 \
  --policy.device=cpu
```

The launcher enables FP16 for this process, so `--policy.dtype=float16` is unnecessary. It is accepted and
ignored for compatibility with earlier commands. Robot, camera, and dataset arguments are unchanged from
the standard LeRobot recording workflow.

## Validate the integration

```bash
python -m pytest -q examples/spine-torch-deploy/test/test_act_k3_fp16_record.py
```

After the command exits, there is nothing to revert: the extensions existed only in that Python process.
