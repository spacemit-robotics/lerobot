# 底盘控制库 (Chassis Control)

## 项目简介

底盘控制库提供统一的机器人底盘运动控制 API，支持多种底盘类型和通信驱动。通过抽象的接口设计，开发者可以使用相同的 API 控制不同类型的底盘（两轮差速、四轮麦轮、全向轮等），无需关心底层通信细节。

## 功能特性

**支持的底盘类型：**

- 两轮差速 (DIFF_2WD)
- 四轮差速 (DIFF_4WD)
- 四轮麦克纳姆轮 (MECANUM_4WD)
- 三轮全向轮 (OMNI_3WD)
- 四轮全向轮 (OMNI_4WD)

**支持的驱动：**

| 驱动名称         | 配置类型               | 说明                   |
| ---------------- | ---------------------- | ---------------------- |
| `drv_uart_esp32` | `chassis_uart_config`  | UART 串口通信          |
| `drv_rpmsg_esos` | `chassis_rpmsg_config` | RPMSG 与 ESOS 小核通信 |

**核心功能：**

- 速度控制（线速度、角速度）
- 里程计获取（位姿、速度）
- 刹车与电机放松
- 分层配置架构，易于扩展新驱动

**暂不支持：**

- CAN 总线驱动（可扩展）
- 以太网驱动（可扩展）

## 快速开始

### 环境准备

- CMake >= 3.10
- C99 编译器（GCC/Clang）
- pthreads 库

### 构建编译

```bash
# 在 SDK 根目录下
cd components/control/base
mkdir build && cd build
cmake ..
make
```

或使用 SDK 统一构建脚本：

```bash
# 在 SDK 根目录下
source build/envsetup.sh
# 按需选择 target 并构建
```

### 运行示例

```c
#include "chassis.h"

// 1. 配置底盘（UART驱动）
struct chassis_uart_config config = {
    .base = {
        .type = CHASSIS_TYPE_DIFF_2WD,
        .wheel_diameter = 0.067f,  // 67mm
        .wheel_base = 0.33f,       // 330mm
        .wheel_track = 0.0f,
        .max_speed = 1.0f,
        .max_angular = 3.14f,
    },
    .dev_path = "/dev/ttyUSB0",
    .baud = 115200,
};

// 2. 创建底盘
struct chassis_dev *dev = chassis_alloc("drv_uart_esp32", &config);

// 3. 控制运动
chassis_velocity_t vel = { .vx = 0.2f, .wz = 0.0f };
chassis_set_velocity(dev, &vel);

// 4. 获取里程计
chassis_velocity_t cur_vel;
chassis_pose_t cur_pose;
chassis_get_odom(dev, &cur_vel, &cur_pose);

// 5. 停止并释放
chassis_brake(dev);
chassis_free(dev);
```

运行测试程序：

```bash
./build/test_chassis_uart
```

## 详细使用

详细的 API 参考、数据类型定义、配置说明请参阅：

- 头文件：`include/chassis.h`
- 协议文档：`PROTOCOL.md`

### API 概览

| 函数                     | 说明                 |
| ------------------------ | -------------------- |
| `chassis_alloc()`        | 创建并初始化底盘设备 |
| `chassis_set_velocity()` | 设置底盘目标速度     |
| `chassis_get_odom()`     | 获取里程计信息       |
| `chassis_brake()`        | 主动刹车             |
| `chassis_relax()`        | 放松电机             |
| `chassis_free()`         | 释放资源             |

### 错误码

| 宏                    | 值  | 说明         |
| --------------------- | --- | ------------ |
| `CHASSIS_OK`          | 0   | 成功         |
| `CHASSIS_ERR_ALLOC`   | -1  | 内存分配失败 |
| `CHASSIS_ERR_CONNECT` | -2  | 连接失败     |
| `CHASSIS_ERR_TIMEOUT` | -3  | 超时         |
| `CHASSIS_ERR_CONFIG`  | -4  | 配置错误     |
| `CHASSIS_ERR_PARAM`   | -5  | 参数错误     |

## 常见问题

**Q: 串口设备找不到？**

- 检查设备路径是否正确（如 `/dev/ttyUSB0`）
- 确认用户有串口访问权限：`sudo usermod -aG dialout $USER`

**Q: RPMSG 驱动初始化失败？**

- 确认 ESOS 小核固件已正确加载
- 检查 `/dev/rpmsg_ctrl0` 和 `/dev/rpmsg0` 设备是否存在

**Q: 如何添加新的驱动类型？**

- 参考 `src/drivers/` 下现有驱动实现
- 定义新的配置结构体（继承 `struct chassis_config`）
- 在 CMakeLists.txt 中添加驱动源文件

## 版本与发布

| 版本   | 日期 | 说明                              |
| ------ | ---- | --------------------------------- |
| v1.0.0 | -    | 初始版本，支持 UART 和 RPMSG 驱动 |

## 贡献方式

欢迎提交 Issue 和 Pull Request。

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准
