<!-- Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd. -->

# Mars README

本文介绍 Mars 机器人数采、训练、推理的完整最小链路：

- 机器人侧启动 `mars_host`
- 操作者电脑侧启动 `MarsClient`
- 通过：
  - `SO101 leader` 控 arm
  - 键盘控 base
- 验证 host/client 路径已经真正打到真实底盘库
- 录制 LeRobot 数据集
- 使用 `lerobot-train` 训练策略
- 使用训练好的策略进行推理与评测

Mars 当前采用 **LeKiwi 风格的 host/client 架构**：

- 开发板作为 host：运行 `MarsHost`，负责连接并直接控制机械臂、底盘、相机等机器人侧硬件
- 操作机作为 client：运行 `MarsClient`，负责遥操作输入、策略推理等上层控制逻辑
- 机械臂由 leader arm 控制
- 底盘由键盘控制

## 1. 架构说明

- `MarsHost`
  - 运行在机器人侧
  - 直接实例化 `Mars`
  - `Mars` 内部：
    - arm 走 Feetech / SO101 控制
    - base 走 `MarsBaseAdapter` -> vendored `third_party/chassis` 共享库
- `MarsClient`
  - 运行在操作端电脑
  - 通过 ZMQ 发 action / 收 observation

因此，只要 `mars_host` 正常启动，并且 `MarsBaseAdapter` 成功加载真实底盘库，host/client 路径就会自动走到真实底盘控制。

## 2. 前置条件

### Mars Client

需要具备：

- Mars 已接好机械臂和底盘，并且被赋予正确权限（666）
- 相机设备节点正确，例如 `/dev/video0` `/dev/video2`
- Mars 底盘控制库已经编译完成，例如 `third_party/chassis/build/libchassis.so`
- 已安装当前版本 LeRobot 及所需依赖

### Mars Host

需要具备：

- 已安装当前版本 LeRobot 及所需依赖
- ZeroMQ 相关依赖
- 如需执行遥操作，确保主导臂已连接、键盘相关功能键正常
- 能访问 Mars Client IP

## 3. 遥操作

### Step A. 启动 host

进入仓库：

```bash
cd lerobot
```

构建底盘共享库：

```bash
./scripts/build_mars_chassis.sh
```

默认会产出：

- `third_party/chassis/build/libchassis.so`

启动 Host：

```bash
python -m lerobot.robots.mars.mars_host \
  --robot.id=my_mars \
  --robot.port=/dev/ttyACM0 \
  --robot.base_driver=drv_uart_esp32 \
  --robot.base_dev_path=/dev/ttyACM1 \
  --robot.base_baud=115200 \
  --robot.base_type=diff_2wd \
  --robot.base_left_wheel_gain=1.08 \
  --host.port_zmq_cmd=5565 \
  --host.port_zmq_observations=5566 \
  --host.connection_time_s=3600 \
  --robot.cameras='{
    "front": {"type":"opencv","index_or_path":"/dev/video2","width":640,"height":480,"fps":30,"fourcc":"MJPG"},
    "wrist": {"type":"opencv","index_or_path":"/dev/video0","width":640,"height":480,"fps":30,"fourcc":"MJPG"}
  }'
```

常用 host 启动参数含义如下：

- `--robot.port`：机器人侧 follower arm 串口
- `--robot.base_dev_path`：机器人侧底盘串口
- `--robot.base_control_library_path`：可选；当你不使用默认 vendored 库路径时，再手动指定 `.so` 的绝对路径
- `--robot.base_driver`：底盘驱动类型；当前真实底盘默认用 `drv_uart_esp32`
- `--robot.base_baud`：UART 波特率
- `--robot.base_type`：底盘模型；当前示例使用 `diff_2wd`
- `--host.port_zmq_cmd`：host 接收 client action 的端口
- `--host.port_zmq_observations`：host 对外发送 observation 的端口
- `--host.connection_time_s`：host 运行时长上限；到时会自动退出
- `--robot.cameras`：相机配置。建议在 UVC 相机上显式设置 `fourcc="MJPG"`，让相机直接输出 motion-JPEG。这样通常能显著降低 USB 带宽压力，并更容易稳定跑到 `640x480@30fps`；否则很多摄像头默认使用 `YUYV`，在同样分辨率下可能只能跑到 25fps 左右，导致 host 侧观测帧率不稳、录制掉帧或推理显示卡顿。

如果选择使用 RPMSG 驱动，把 UART 相关参数替换为：

```bash
--robot.base_driver=drv_rpmsg_esos \
--robot.base_rpmsg_ctrl_dev=/dev/rpmsg_ctrl0 \
--robot.base_rpmsg_data_dev=/dev/rpmsg0 \
--robot.base_rpmsg_service_name=rpmsg:motor_ctrl \
--robot.base_rpmsg_local_addr=1003 \
--robot.base_rpmsg_remote_addr=1002
```

### Step B. Host 端修改遥操脚本参数

遥操脚本位于：

- `examples/mars/teleoperate.py`

至少修改以下代码片段：

```bash
robot_config = MarsClientConfig(remote_ip="mars_host_remote_ip", id="my_mars")
teleop_arm_config = SO101LeaderConfig(port="/dev/ttyACM0", id="my_mars_leader")
```

### Step C. Host 端启动 teleop

```bash
cd lerobot
python examples/mars/teleoperate.py
```

遥操路径为：

- SO101 leader arm -> arm action
- keyboard -> base action

base 默认键位来自 `MarsClientConfig.teleop_keys`：

- `w`: forward
- `s`: backward
- `a`: left
- `d`: right
- `z`: rotate_left
- `x`: rotate_right
- `r`: speed_up
- `f`: speed_down
- `q`: 退出 teleop 循环；脚本会在退出前先发送一次零底盘速度命令

需要注意的是，对于两轮差速底盘，左右平移功能不支持。

## 4. 数据集采集

数采脚本位于：

- `examples/mars/record.py`

数采之前启动 Mars Host，并需要修改以下代码片段：

```bash
REMOTE_IP = "mars_host_remote_ip"
ROBOT_ID = "my_mars"
LEADER_PORT = "/dev/ttyACM0"
LEADER_ID = "my_mars_leader"
KEYBOARD_ID = "my_keyboard"

NUM_EPISODES = 30
FPS = 30
EPISODE_TIME_SEC = 600
RESET_TIME_SEC = 30
TASK_DESCRIPTION = "pick and place the cube on the orange box"
HF_REPO_ID = "hf_username/mars-pick-place-move"
PUSH_TO_HUB = False
RESUME = False
```

录制完成后，脚本会：

- `dataset.finalize()`
- 当 `PUSH_TO_HUB=True` 且本次确实保存了 episode 时，才会执行 `dataset.push_to_hub()`

续采功能：

- `RESUME = True`：在已有数据集上继续录制
- `RESUME = False`：创建一个全新的数据集

## 5. 训练

直接复用 LeRobot 通用训练入口：

```bash
lerobot-train \ 
  --policy.type=act \
  --policy.repo_id=hf_username/mars_act_pick_place \
  --dataset.repo_id=hf_username/mars-pick-place \
  --dataset.root=datasets/mars-pick-place \
  --output_dir=outputs/train/mars_act_pick_place \
  --job_name=mars_act_pick_place \
  --batch_size=4 \
  --steps=100000 \
  --policy.device=cuda
```

恢复训练示例：

```bash
lerobot-train \
  --config_path=outputs/train/act_mars/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

## 6. 推理 / 评测

推理脚本位于：

- `examples/mars/evaluate.py`

推理之前启动 Mars Host，并需要修改以下代码片段：

```bash
REMOTE_IP = "mars_host_remote_ip"
ROBOT_ID = "my_mars"
TRAIN_DATASET_REPO_ID = "hf_username/mars-pick-place-move"
NUM_EPISODES = 2
FPS = 30
EPISODE_TIME_SEC = 60
RESET_TIME_SEC = 20
TASK_DESCRIPTION = "pick and place the cube on the orange box"
HF_MODEL_ID = "outputs/train/mars_act_pick_place_move/checkpoints/100000/pretrained_model"
HF_DATASET_ID = "hf_username/mars-pick-place-move-eval"
PUSH_TO_HUB = False
```
执行推理：

```bash
cd lerobot
python examples/mars/evaluate.py
```