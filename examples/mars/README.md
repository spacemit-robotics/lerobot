<!-- Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd. -->

# Mars README

这份说明对应当前仓库里的 **Mars + `MarsHost` + `MarsClient` + 真实底盘 adapter** 路径，覆盖从真机联调到数采、训练、推理的完整最小链路：

- 机器人侧启动 `mars_host`
- 操作者电脑侧启动 `MarsClient`
- 通过：
  - `SO101 leader` 控 arm
  - 键盘控 base
- 验证 host/client 路径已经真正打到真实底盘库
- 录制 LeRobot 数据集
- 使用 `lerobot-train` 训练策略
- 使用训练好的策略进行推理与评测

Mars 当前按 **LeKiwi 风格 host/client 架构**工作：

- 开发板作为 host：`bianbu@10.0.91.173`
- 操作机作为 client：运行 `MarsClient`
- leader arm 控机械臂
- keyboard 控底盘

## 1. 架构说明

当前路径是：

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

### 机器人侧

需要具备：

- Mars arm 已接好，SO101 follower 臂可被 LeRobot 访问
- Mars 底盘控制库已经编译完成，例如 `third_party/chassis/build/libchassis.so`
- 底盘 UART 串口设备可访问（Mars 当前默认按 UART 方式驱动）
- 相机设备节点正确，例如 `/dev/video0` `/dev/video2`
- 机器人侧已安装当前版本 LeRobot 及所需依赖

### 操作端电脑侧

需要具备：

- 当前版本 LeRobot
- ZeroMQ 相关依赖
- leader arm 可访问
- 键盘 teleop 可用
- 能访问机器人 IP

开始前还应确认：

- 本地代码已同步到开发板
- 开发板能正常启动 `mars_host`
- follower arm 串口和底盘串口已区分
- 操作机侧 leader arm 串口可访问
- 已安装 LeRobot 所需依赖和 Feetech 支持

## 3. 最小启动步骤

### Step A. 在机器人侧启动 host

先在机器人侧进入仓库：

```bash
cd /home/anny/workspaces/lerobot-0.5.0
```

如果你使用当前仓库里已 vendored 的底盘源码，先构建一次共享库：

```bash
./scripts/build_mars_chassis.sh
```

默认会产出：

- `third_party/chassis/build/libchassis.so`

当前 `MarsBaseAdapter` 会优先自动查找：

- `third_party/chassis/build/libchassis.so`
- `third_party/chassis/build/libbase.so`

因此在大多数情况下，**不再需要显式传 `--robot.base_control_library_path`**。

然后按 UART 方式启动：

```bash
python -m lerobot.robots.mars.mars_host \
  --robot.id=my_mars \
  --robot.port=/dev/ttyACM0 \
  --robot.base_driver=drv_uart_esp32 \
  --robot.base_dev_path=/dev/ttyACM1 \
  --robot.base_baud=115200 \
  --robot.base_type=diff_2wd \
  --host.port_zmq_cmd=5565 \
  --host.port_zmq_observations=5566 \
  --host.connection_time_s=3600 \
  --robot.cameras='{
    "front": {"type":"opencv","index_or_path":"/dev/video2","width":640,"height":480,"fps":30},
    "wrist": {"type":"opencv","index_or_path":"/dev/video0","width":480,"height":640,"fps":30,"rotation":"ROTATE_90"}
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
- `--robot.cameras`：相机配置

如果分辨率跑不上去，就让相机用motion-JPEG传输帧，很多UVC相机默认YUYV在640x480只能到25fps。

```bash
--robot.cameras='{
    "front": {"type":"opencv","index_or_path":"/dev/video2","width":640,"height":480,"fps":30,"fourcc":"MJPG"},
    "wrist": {"type":"opencv","index_or_path":"/dev/video0","width":480,"height":640,"fps":30,"rotation":"ROTATE_90","fourcc":"MJPG"}
  }'
```

如果选择使用 RPMSG 驱动，把 UART 相关参数替换为：

```bash
--robot.base_driver=drv_rpmsg_esos \
--robot.base_rpmsg_ctrl_dev=/dev/rpmsg_ctrl0 \
--robot.base_rpmsg_data_dev=/dev/rpmsg0 \
--robot.base_rpmsg_service_name=rpmsg:motor_ctrl \
--robot.base_rpmsg_local_addr=1003 \
--robot.base_rpmsg_remote_addr=1002
```

### Step B. 在操作端修改示例脚本参数

仓库里当前用于 host/client teleop 的最小示例：

- `examples/mars/teleoperate.py`

需要至少改这几个参数：

- `MarsClientConfig(remote_ip="192.168.1.10", ...)`
- `SO101LeaderConfig(port="/dev/ttyACM1", ...)`

把它们改成你自己的：

- 机器人 IP
- leader arm 串口
- robot / leader id

### Step C. 在操作端启动 teleop

```bash
cd /home/anny/workspaces/lerobot-0.5.0
python examples/mars/teleoperate.py
```

## 4. 控制方式

当前最小联调脚本沿用 LeKiwi 风格：

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

底盘 action 语义为：

- `x.vel`
- `y.vel`
- `theta.vel`

这与当前真实底盘版 `Mars` / `MarsHost` / `MarsClient` 已保持一致。

但要注意：如果你的底盘类型是 `diff_2wd`，那么实际底盘并不是全向底盘。

- `x.vel`：前后运动
- `theta.vel`：原地旋转
- `y.vel`：在当前语义层仍然保留，但在 `diff_2wd` 上不应理解为“真正的侧向平移”

因此首次测试时，建议把 `a/d` 当成“当前 teleop 语义映射中的左右控制输入”，并**用真机实际运动结果确认方向与预期是否一致**，不要先验假设它一定等价于全向底盘的 lateral motion。

## 5. 数采

Mars 数采示例：

- `examples/mars/record.py`

该脚本复用 LeKiwi 思路：

- `teleop=[leader_arm, keyboard]`
- `record_loop(...)`
- 由 `robot.action_features` 和 `robot.observation_features` 自动生成数据集特征

运行前至少修改这些常量：

- `REMOTE_IP`
- `ROBOT_ID`
- `LEADER_PORT`
- `LEADER_ID`
- `HF_REPO_ID`
- `TASK_DESCRIPTION`

录制完成后，脚本会：

- `dataset.finalize()`
- `dataset.push_to_hub()`

如果你希望走统一 CLI，也可以直接使用 `lerobot-record`，核心思路与 `examples/mars/record.py` 一致。

## 6. 训练

Mars 当前建议先直接复用 LeRobot 通用训练入口：

```bash
lerobot-train \
  --dataset.repo_id=<hf_username>/<mars_dataset_repo_id> \
  --policy.type=act \
  --output_dir=outputs/train/act_mars \
  --job_name=act_mars \
  --policy.device=cuda \
  --wandb.enable=true \
  --policy.repo_id=<hf_username>/<mars_policy_repo_id>
```

说明：

- `ACT` 会根据数据集中的状态、动作、相机特征自动适配输入输出维度
- 如果不用 wandb，可设 `--wandb.enable=false`
- 如果不想训练完成后自动上传模型，可加 `--policy.push_to_hub=false`

恢复训练示例：

```bash
lerobot-train \
  --config_path=outputs/train/act_mars/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

## 7. 推理 / 评测

Mars 推理与评测示例：

- `examples/mars/evaluate.py`

该脚本流程为：

1. 连接 `MarsClient`
2. 从 Hub 加载策略
3. 从**训练数据集**加载 `dataset_stats`
4. 用 `record_loop(...)` 执行策略并记录评测 episode

运行前需要修改这些常量：

- `REMOTE_IP`
- `ROBOT_ID`
- `HF_MODEL_ID`
- `TRAIN_DATASET_REPO_ID`
- `HF_DATASET_ID`
- `TASK_DESCRIPTION`

注意：

- `TRAIN_DATASET_REPO_ID` 必须指向训练该策略所使用的数据集，不能直接用新建评测集的统计量
- 评测脚本会把推理 episode 保存为新的 `LeRobotDataset`，便于回放和比对

## 8. 推荐最小落地顺序

建议按下面顺序推进：

1. 先确认 `teleoperate.py` 真机稳定
2. 用 `record.py` 录一个小规模数据集
3. 用 `lerobot-train` 训练 `ACT`
4. 用 `evaluate.py` 跑少量评测 episode
5. 再根据结果决定是否继续做通用 CLI 封装

## 9. 当前约束

- `lerobot_teleoperate.py` 对多 teleop 场景支持仍偏弱，Mars 目前更适合先保留专用示例脚本
- `lerobot_record.py` 已能较好支撑 LeKiwi 风格的多输入记录路径
- Mars 训练流程当前直接复用通用 `lerobot-train`，未新增 Mars 专属训练脚本
