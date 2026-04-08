/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum chassis_type {
    CHASSIS_TYPE_DIFF_2WD = 0,  /* 两轮差速 */
    CHASSIS_TYPE_DIFF_4WD,      /* 四轮差速 */
    CHASSIS_TYPE_MECANUM_4WD,   /* 四轮麦克纳姆 */
    CHASSIS_TYPE_OMNI_3WD,      /* 三轮全向 */
    CHASSIS_TYPE_OMNI_4WD,      /* 四轮全向 */
};

/* ==========================================================================
 * 1. Data Structures
 * ========================================================================== */

// 之前的 base_twist_t，现在改名更直观
// 描述底盘的运动意图
typedef struct {
    float vx;  // m/s, 前后 (Linear X)
    float vy;  // m/s, 左右 (Linear Y, 麦轮/全向轮用)
    float wz;  // rad/s, 旋转 (Angular Z)
} chassis_velocity_t;

// 之前的 base_pose_t
// 描述底盘在世界坐标系的位置
typedef struct {
    float x;
    float y;
    float yaw;  // 偏航角
} chassis_pose_t;

/**
 * @brief 底盘通用配置参数（所有底盘类型共用）
 */
struct chassis_config {
    // 底盘类型
    enum chassis_type type;

    // 运动学参数
    float wheel_diameter;  // 轮径 (m)
    float wheel_base;      // 轮距 (m), 差速为左右轮距，麦轮/全向为横向轮距
    float wheel_track;     // 前后轴距 (m), 四轮底盘使用
    float left_wheel_gain; // 左轮速度补偿系数，仅差速底盘使用

    // 限制
    float max_speed;    // 最大线速度 (m/s)
    float max_angular;  // 最大角速度 (rad/s)
};

/**
 * @brief UART驱动特定配置（用于串口通信的底盘）
 */
struct chassis_uart_config {
    struct chassis_config base;  // 通用配置

    // UART特定参数
    const char *dev_path;  // 串口路径 e.g., "/dev/ttyUSB0"
    uint32_t baud;         // 波特率
};

/**
 * @brief RPMSG ESOS驱动特定配置
 */
struct chassis_rpmsg_config {
    struct chassis_config base;  // 通用配置

    // RPMSG特定参数
    const char *ctrl_dev;      // 控制设备 e.g., "/dev/rpmsg_ctrl0"
    const char *data_dev;      // 数据设备 e.g., "/dev/rpmsg0"
    const char *service_name;  // 服务名称 e.g., "rpmsg:motor_ctrl"
    uint32_t local_addr;       // 本地地址 e.g., 1003
    uint32_t remote_addr;      // 远端地址 e.g., 1002
};

/* ==========================================================================
 * 2. Opaque Handle
 * ========================================================================== */

struct chassis_dev;

/* ==========================================================================
 * 3. API Functions
 * ========================================================================== */

/**
 * @brief 创建底盘实例
 * @param driver_name  驱动名称 e.g., "uart_diff", "uart_mecanum"
 * @param config       底盘配置（包含类型、通信、运动学参数）
 * @return 成功返回设备句柄，失败返回 NULL
 */
struct chassis_dev *chassis_alloc(const char *driver_name, void *args);

/**
 * @brief 设定底盘目标速度
 * @param vel 期望速度结构体
 */
int chassis_set_velocity(struct chassis_dev *dev, const chassis_velocity_t *vel);

/**
 * @brief 获取底盘里程计
 * @param out_vel  当前实际速度
 * @param out_pose 当前累计位置
 */
int chassis_get_odom(struct chassis_dev *dev, chassis_velocity_t *out_vel, chassis_pose_t *out_pose);

/**
 * @brief 周期性计算 (放入 100Hz 定时器)
 * 负责里程计积分、PID闭环计算
 */
void chassis_tick(struct chassis_dev *dev, float dt_s);

// 刹车与放松
void chassis_brake(struct chassis_dev *dev);
void chassis_relax(struct chassis_dev *dev);

void chassis_free(struct chassis_dev *dev);

/* ==========================================================================
 * 4. Error Codes
 * ========================================================================== */
// 错误码
#define CHASSIS_OK 0
#define CHASSIS_ERR_ALLOC -1
#define CHASSIS_ERR_CONNECT -2
#define CHASSIS_ERR_TIMEOUT -3
#define CHASSIS_ERR_CONFIG -4
#define CHASSIS_ERR_PARAM -5

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H */
