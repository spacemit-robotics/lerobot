/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file chassis_core.c
 * @brief Core implementation for chassis control component
 */

#include "chassis_core.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ==========================================================================
 * Driver Registry
 * ========================================================================== */

static struct chassis_driver_info *g_driver_list = NULL;

void chassis_driver_register(struct chassis_driver_info *info) {
    if (!info)
        return;
    info->next = g_driver_list;
    g_driver_list = info;
    printf("[CHASSIS] Registered driver: %s\n", info->name);
}

static struct chassis_driver_info *find_driver(const char *name) {
    struct chassis_driver_info *curr = g_driver_list;

    while (curr) {
        if (curr->name && name && strcmp(curr->name, name) == 0)
            return curr;
        curr = curr->next;
    }

    printf("[CHASSIS] Driver not found: %s\n", name ? name : "(null)");
    return NULL;
}

/* ==========================================================================
 * Device Allocation
 * ========================================================================== */

struct chassis_dev *chassis_dev_alloc(const char *name,
        const struct chassis_config *config, size_t priv_size) {
    struct chassis_dev *dev;
    void *priv = NULL;
    char *name_copy = NULL;

    dev = calloc(1, sizeof(*dev));
    if (!dev)
        return NULL;

    if (priv_size) {
        priv = calloc(1, priv_size);
        if (!priv) {
            free(dev);
            return NULL;
        }
        dev->priv_data = priv;
    }

    if (name) {
        size_t n = strlen(name);
        name_copy = calloc(1, n + 1);
        if (!name_copy) {
            free(priv);
            free(dev);
            return NULL;
        }
        memcpy(name_copy, name, n);
        name_copy[n] = '\0';
        dev->name = name_copy;
    }

    if (config) {
        dev->config = *config;
    }

    dev->running = false;

    /* Initialize mutex */
    pthread_mutex_init(&dev->odom_lock, NULL);

    /* Initialize odometry */
    memset(&dev->cur_vel, 0, sizeof(dev->cur_vel));
    memset(&dev->cur_pose, 0, sizeof(dev->cur_pose));

    return dev;
}

void chassis_dev_free_default(struct chassis_dev *dev) {
    if (!dev)
        return;

    pthread_mutex_destroy(&dev->odom_lock);

    if (dev->priv_data)
        free(dev->priv_data);
    if (dev->name)
        free((void *)dev->name);
    free(dev);
}

/* ==========================================================================
 * Public API Implementation
 * ========================================================================== */

struct chassis_dev *chassis_alloc(const char *driver_name, void *args) {
    struct chassis_driver_info *drv;
    const struct chassis_config *config = (const struct chassis_config *)args;

    if (!driver_name || !config)
        return NULL;

    drv = find_driver(driver_name);
    if (!drv || !drv->factory) {
        printf("[CHASSIS] No driver found: %s\n", driver_name);
        return NULL;
    }

    return drv->factory(driver_name, config);
}

int chassis_set_velocity(struct chassis_dev *dev,
        const chassis_velocity_t *vel) {
    if (!dev || !vel)
        return -EINVAL;

    if (dev->ops && dev->ops->set_velocity)
        return dev->ops->set_velocity(dev, vel);

    return -ENOSYS;
}

int chassis_get_odom(struct chassis_dev *dev,
        chassis_velocity_t *out_vel, chassis_pose_t *out_pose) {
    if (!dev)
        return -EINVAL;

    pthread_mutex_lock(&dev->odom_lock);

    if (out_vel)
        *out_vel = dev->cur_vel;
    if (out_pose)
        *out_pose = dev->cur_pose;

    pthread_mutex_unlock(&dev->odom_lock);

    return CHASSIS_OK;
}

void chassis_tick(struct chassis_dev *dev, float dt_s) {
    if (!dev)
        return;

    if (dev->ops && dev->ops->tick)
        dev->ops->tick(dev, dt_s);
}

void chassis_brake(struct chassis_dev *dev) {
    if (!dev)
        return;

    if (dev->ops && dev->ops->brake) {
        dev->ops->brake(dev);
        return;
    }

    /* Default: send zero velocity */
    chassis_velocity_t zero = {0.0f, 0.0f, 0.0f};
    chassis_set_velocity(dev, &zero);
}

void chassis_relax(struct chassis_dev *dev) {
    if (!dev)
        return;

    if (dev->ops && dev->ops->relax)
        dev->ops->relax(dev);
}

void chassis_free(struct chassis_dev *dev) {
    if (!dev)
        return;

    if (dev->running) {
        dev->running = false;
        if (dev->ops && dev->ops->stop)
            dev->ops->stop(dev);
    }

    if (dev->ops && dev->ops->free) {
        dev->ops->free(dev);
        return;
    }

    chassis_dev_free_default(dev);
}
