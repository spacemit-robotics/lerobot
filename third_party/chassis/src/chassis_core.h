/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CHASSIS_CORE_H
#define CHASSIS_CORE_H

/**
 * @file chassis_core.h
 * @brief Private header for chassis control component (internal use only)
 */

#include "../include/chassis.h"
#include <pthread.h>
#include <stddef.h>

/* ==========================================================================
 * 1. Virtual Operations Table (driver interface)
 * ========================================================================== */

struct chassis_ops {
    int (*init)(struct chassis_dev *dev);
    int (*start)(struct chassis_dev *dev);
    int (*stop)(struct chassis_dev *dev);
    int (*set_velocity)(struct chassis_dev *dev, const chassis_velocity_t *vel);
    void (*tick)(struct chassis_dev *dev, float dt_s);
    void (*brake)(struct chassis_dev *dev);
    void (*relax)(struct chassis_dev *dev);
    void (*free)(struct chassis_dev *dev);
};

/* ==========================================================================
 * 2. Device Structure (internal)
 * ========================================================================== */

struct chassis_dev {
    /* Identity */
    const char *name;

    /* Configuration (copy from user) */
    struct chassis_config config;

    /* Operations */
    const struct chassis_ops *ops;
    void *priv_data;

    /* Odometry state (updated by driver thread) */
    chassis_velocity_t cur_vel;
    chassis_pose_t cur_pose;
    pthread_mutex_t odom_lock;

    /* Runtime state */
    bool running;
    pthread_t rx_thread;
};

/* ==========================================================================
 * 3. Driver Registry
 * ========================================================================== */

typedef struct chassis_dev *(*chassis_factory_t)(
    const char *name, const struct chassis_config *config);

struct chassis_driver_info {
    const char *name;  // 驱动名称 e.g., "uart_diff"
    chassis_factory_t factory;
    struct chassis_driver_info *next;
};

void chassis_driver_register(struct chassis_driver_info *info);

/**
 * @brief Register a chassis driver (called at load time via constructor)
 * @param _name    Driver name string
 * @param _factory Factory function
 */
#define REGISTER_CHASSIS_DRIVER(_name, _factory)                               \
    static struct chassis_driver_info __drv_info_##_factory = {                \
        .name = _name, .factory = _factory, .next = NULL};                     \
    __attribute__((constructor)) static void __auto_reg_##_factory(void) {     \
        chassis_driver_register(&__drv_info_##_factory);                       \
    }

/* ==========================================================================
 * 4. Internal Helpers
 * ========================================================================== */

/**
 * @brief Allocate a chassis device with private data
 */
struct chassis_dev *chassis_dev_alloc(const char *name,
        const struct chassis_config *config, size_t priv_size);

/**
 * @brief Default free implementation
 */
void chassis_dev_free_default(struct chassis_dev *dev);

#endif /* CHASSIS_CORE_H */
