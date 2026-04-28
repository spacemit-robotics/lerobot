# Chassis Control Library

## Introduction

The Chassis Control Library provides a unified API for robot chassis motion control, supporting multiple chassis types and communication drivers. Through an abstracted interface design, developers can use the same API to control different types of chassis (two-wheel differential, four-wheel Mecanum, omnidirectional wheels, etc.) without worrying about underlying communication details.

## Features

**Supported Chassis Types:**

- Two-wheel Differential (DIFF_2WD)
- Four-wheel Differential (DIFF_4WD)
- Four-wheel Mecanum (MECANUM_4WD)
- Three-wheel Omnidirectional (OMNI_3WD)
- Four-wheel Omnidirectional (OMNI_4WD)

**Supported Drivers:**

| Driver Name      | Config Type            | Description                  |
| ---------------- | ---------------------- | ---------------------------- |
| `drv_uart_esp32` | `chassis_uart_config`  | UART serial communication    |
| `drv_rpmsg_esos` | `chassis_rpmsg_config` | RPMSG with ESOS co-processor |

**Core Features:**

- Velocity control (linear and angular velocity)
- Odometry retrieval (pose and velocity)
- Brake and motor relaxation
- Layered configuration architecture for easy driver extension

**Not Yet Supported:**

- CAN bus driver (extensible)
- Ethernet driver (extensible)

## Quick Start

### Prerequisites

- CMake >= 3.10
- C99 compiler (GCC/Clang)
- pthreads library

### Build

```bash
# From SDK root directory
cd components/control/base
mkdir build && cd build
cmake ..
make
```

Or use the SDK unified build script:

```bash
# From SDK root directory
source build/envsetup.sh
# Select target and build as needed
```

### Example

```c
#include "chassis.h"

// 1. Configure chassis (UART driver)
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

// 2. Create chassis
struct chassis_dev *dev = chassis_alloc("drv_uart_esp32", &config);

// 3. Control motion
chassis_velocity_t vel = { .vx = 0.2f, .wz = 0.0f };
chassis_set_velocity(dev, &vel);

// 4. Get odometry
chassis_velocity_t cur_vel;
chassis_pose_t cur_pose;
chassis_get_odom(dev, &cur_vel, &cur_pose);

// 5. Stop and release
chassis_brake(dev);
chassis_free(dev);
```

Run test program:

```bash
./build/test_chassis_uart
```

## Detailed Usage

For detailed API reference, data type definitions, and configuration instructions, please refer to:

- Header file: `include/chassis.h`
- Protocol document: `PROTOCOL.md`

### API Overview

| Function                 | Description                          |
| ------------------------ | ------------------------------------ |
| `chassis_alloc()`        | Create and initialize chassis device |
| `chassis_set_velocity()` | Set chassis target velocity          |
| `chassis_get_odom()`     | Get odometry information             |
| `chassis_brake()`        | Active brake                         |
| `chassis_relax()`        | Relax motors                         |
| `chassis_free()`         | Release resources                    |

### Error Codes

| Macro                 | Value | Description              |
| --------------------- | ----- | ------------------------ |
| `CHASSIS_OK`          | 0     | Success                  |
| `CHASSIS_ERR_ALLOC`   | -1    | Memory allocation failed |
| `CHASSIS_ERR_CONNECT` | -2    | Connection failed        |
| `CHASSIS_ERR_TIMEOUT` | -3    | Timeout                  |
| `CHASSIS_ERR_CONFIG`  | -4    | Configuration error      |
| `CHASSIS_ERR_PARAM`   | -5    | Parameter error          |

## FAQ

**Q: Serial device not found?**

- Check if the device path is correct (e.g., `/dev/ttyUSB0`)
- Ensure user has serial port access: `sudo usermod -aG dialout $USER`

**Q: RPMSG driver initialization failed?**

- Confirm ESOS co-processor firmware is properly loaded
- Check if `/dev/rpmsg_ctrl0` and `/dev/rpmsg0` devices exist

**Q: How to add a new driver type?**

- Refer to existing driver implementations in `src/drivers/`
- Define a new config struct (inheriting from `struct chassis_config`)
- Add driver source file in CMakeLists.txt

## Version & Release

| Version | Date | Description                                      |
| ------- | ---- | ------------------------------------------------ |
| v1.0.0  | -    | Initial release, supports UART and RPMSG drivers |

## Contributing

Issues and Pull Requests are welcome.

## License

Source files in this component are declared as Apache-2.0 in their headers. The `LICENSE` file in this directory shall prevail.
