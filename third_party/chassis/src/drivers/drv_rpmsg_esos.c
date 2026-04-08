/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file drv_rpmsg_esos.c
 * @brief RPMsg driver for ESOS differential (2WD) chassis
 *
 * Protocol (compatible with existing ROS2 driver):
 * TX: "{dir_l},{speed_l:.3f};{dir_r},{speed_r:.3f}"
 *     - dir: 0=stop, 1=forward, 2=backward
 *     - speed: rev/s (rotations per second)
 *
 * RX: "{dir1},{speed1_mrs};{dir2},{speed2_mrs}"
 *     - speed_mrs: milli-revolutions/s
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>

#include "../chassis_core.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ==========================================================================
 * RPMsg ioctl definitions
 * ========================================================================== */

struct rpmsg_endpoint_info {
    char name[32];
    uint32_t src;
    uint32_t dst;
};

#define RPMSG_CREATE_EPT_IOCTL _IOW(0xb5, 0x1, struct rpmsg_endpoint_info)
#define RPMSG_DESTROY_EPT_IOCTL _IO(0xb5, 0x2)

/* ==========================================================================
 * Default Configuration
 * ========================================================================== */

#define DEFAULT_RPMSG_CTRL_DEV "/dev/rpmsg_ctrl0"
#define DEFAULT_RPMSG_DATA_DEV "/dev/rpmsg0"
#define DEFAULT_RPMSG_SERVICE_NAME "rpmsg:motor_ctrl"
#define DEFAULT_RPMSG_LOCAL_ADDR 1003
#define DEFAULT_RPMSG_REMOTE_ADDR 1002

/* ==========================================================================
 * Private Data Structure
 * ========================================================================== */

struct rpmsg_esos_priv {
    int rpmsg_fd;
    int rpmsg_ctrl_fd;

    /* RPMSG config */
    const char *ctrl_dev;
    const char *data_dev;
    const char *service_name;
    uint32_t local_addr;
    uint32_t remote_addr;

    /* Target velocity */
    float target_vx;
    float target_wz;
    pthread_mutex_t vel_lock;

    /* Wheel speed from feedback (m/s) */
    float wheel_left_mps;
    float wheel_right_mps;

    /* Timing */
    struct timeval last_odom_time;
    int send_interval_ms;
};

/* ==========================================================================
 * Helper Functions
 * ========================================================================== */

static void velocity_to_cmd(float v, float wheel_diam, int *out_dir,
                            float *out_speed) {
    if (fabsf(v) < 1e-3f) {
        *out_dir = 0;
        *out_speed = 0.0f;
        return;
    }
    *out_dir = (v > 0) ? 1 : 2;
    *out_speed = fabsf(v) / (M_PI * wheel_diam);
}

/* Forward declaration */
static int rpmsg_esos_set_velocity(struct chassis_dev *dev,
        const chassis_velocity_t *vel);

/* ==========================================================================
 * RPMsg Operations
 * ========================================================================== */

static int rpmsg_init(struct chassis_dev *dev) {
    struct rpmsg_esos_priv *priv = dev->priv_data;
    struct rpmsg_endpoint_info epinfo;

    printf("[CHASSIS-RPMSG-ESOS] Opening control device: %s\n", priv->ctrl_dev);

    priv->rpmsg_ctrl_fd = open(priv->ctrl_dev, O_RDWR);
    if (priv->rpmsg_ctrl_fd < 0) {
        printf("[CHASSIS-RPMSG-ESOS] Failed to open %s: %s\n",
                priv->ctrl_dev, strerror(errno));
        return -1;
    }

    memset(&epinfo, 0, sizeof(epinfo));
    strncpy(epinfo.name, priv->service_name, sizeof(epinfo.name) - 1);
    epinfo.src = priv->local_addr;
    epinfo.dst = priv->remote_addr;

    printf("[CHASSIS-RPMSG-ESOS] Creating endpoint: %s (src=%u, dst=%u)\n",
            priv->service_name, priv->local_addr, priv->remote_addr);

    if (ioctl(priv->rpmsg_ctrl_fd, RPMSG_CREATE_EPT_IOCTL, &epinfo) < 0) {
        printf("[CHASSIS-RPMSG-ESOS] Failed to create endpoint: %s\n",
                strerror(errno));
        close(priv->rpmsg_ctrl_fd);
        priv->rpmsg_ctrl_fd = -1;
        return -1;
    }

    priv->rpmsg_fd = open(priv->data_dev, O_RDWR);
    if (priv->rpmsg_fd < 0) {
        printf("[CHASSIS-RPMSG-ESOS] Failed to open %s: %s\n",
                priv->data_dev, strerror(errno));
        close(priv->rpmsg_ctrl_fd);
        priv->rpmsg_ctrl_fd = -1;
        return -1;
    }

    printf("[CHASSIS-RPMSG-ESOS] RPMsg initialized successfully\n");
    return 0;
}

static void rpmsg_cleanup(struct rpmsg_esos_priv *priv) {
    if (priv->rpmsg_fd >= 0) {
        close(priv->rpmsg_fd);
        priv->rpmsg_fd = -1;
    }
    if (priv->rpmsg_ctrl_fd >= 0) {
        close(priv->rpmsg_ctrl_fd);
        priv->rpmsg_ctrl_fd = -1;
    }
    printf("[CHASSIS-RPMSG-ESOS] RPMsg cleaned up\n");
}

/* ==========================================================================
 * Send Command
 * ========================================================================== */

static int send_velocity_cmd(struct chassis_dev *dev) {
    struct rpmsg_esos_priv *priv = dev->priv_data;
    float vx, wz;

    pthread_mutex_lock(&priv->vel_lock);
    vx = priv->target_vx;
    wz = priv->target_wz;
    pthread_mutex_unlock(&priv->vel_lock);

    float wheel_base = dev->config.wheel_base;
    float wheel_diam = dev->config.wheel_diameter;
    float left_wheel_gain = 1.0f;

    if (dev->config.type == CHASSIS_TYPE_DIFF_2WD) {
        left_wheel_gain = dev->config.left_wheel_gain;
    }

    float v_l = (vx - wz * wheel_base / 2.0f) * left_wheel_gain;
    float v_r = vx + wz * wheel_base / 2.0f;

    int dir_l, dir_r;
    float spd_l, spd_r;
    velocity_to_cmd(v_l, wheel_diam, &dir_l, &spd_l);
    velocity_to_cmd(v_r, wheel_diam, &dir_r, &spd_r);

    char cmd[64];
    int len =
        snprintf(cmd, sizeof(cmd), "%d,%.3f;%d,%.3f", dir_l, spd_l, dir_r, spd_r);

    if (priv->rpmsg_fd >= 0 && len > 0) {
        ssize_t written = write(priv->rpmsg_fd, cmd, len + 1);
        if (written < 0) {
            printf("[CHASSIS-RPMSG-ESOS] Write error: %s\n", strerror(errno));
            return -1;
        }
    }

    return 0;
}

/* ==========================================================================
 * Receive Thread
 * ========================================================================== */

static void update_odometry(struct chassis_dev *dev, float dt) {
    struct rpmsg_esos_priv *priv = dev->priv_data;
    float wheel_base = dev->config.wheel_base;

    float v_l = priv->wheel_left_mps;
    float v_r = priv->wheel_right_mps;

    float v = (v_r + v_l) / 2.0f;
    float w = (v_r - v_l) / wheel_base;

    pthread_mutex_lock(&dev->odom_lock);

    dev->cur_vel.vx = v;
    dev->cur_vel.vy = 0.0f;
    dev->cur_vel.wz = w;

    dev->cur_pose.yaw += w * dt;
    dev->cur_pose.x += v * cosf(dev->cur_pose.yaw) * dt;
    dev->cur_pose.y += v * sinf(dev->cur_pose.yaw) * dt;

    pthread_mutex_unlock(&dev->odom_lock);
}

static void parse_feedback(struct chassis_dev *dev, const char *buf) {
    struct rpmsg_esos_priv *priv = dev->priv_data;
    float wheel_diam = dev->config.wheel_diameter;

    int dir1 = 0, dir2 = 0;
    int speed1_mrs = 0, speed2_mrs = 0;

    if (sscanf(buf, "%d,%d;%d,%d", &dir1, &speed1_mrs, &dir2, &speed2_mrs) == 4) {
        /* Convert milli-revolutions/s -> m/s */
        float rps1 = speed1_mrs / 1000.0f;
        float rps2 = speed2_mrs / 1000.0f;

        float v1 = rps1 * M_PI * wheel_diam;
        float v2 = rps2 * M_PI * wheel_diam;

        if (dir1 == 2)
            v1 = -v1;
        else if (dir1 == 0)
            v1 = 0.0f;

        if (dir2 == 2)
            v2 = -v2;
        else if (dir2 == 0)
            v2 = 0.0f;

        priv->wheel_left_mps = v1;
        priv->wheel_right_mps = v2;
    }
}

static void *rx_thread_func(void *arg) {
    struct chassis_dev *dev = arg;
    struct rpmsg_esos_priv *priv = dev->priv_data;

    char recv_buf[256];
    struct pollfd pfd;
    struct timeval now, last_send;

    pfd.fd = priv->rpmsg_fd;
    pfd.events = POLLIN;

    gettimeofday(&last_send, NULL);
    gettimeofday(&priv->last_odom_time, NULL);

    printf("[CHASSIS-RPMSG-ESOS] RX thread started\n");

    while (dev->running) {
        int ret = poll(&pfd, 1, 10);
        if (ret < 0) {
            if (errno == EINTR)
                continue;
            printf("[CHASSIS-RPMSG-ESOS] Poll error: %s\n", strerror(errno));
            break;
        }

        if (ret > 0 && (pfd.revents & POLLIN)) {
            memset(recv_buf, 0, sizeof(recv_buf));
            ssize_t n = read(priv->rpmsg_fd, recv_buf, sizeof(recv_buf) - 1);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                    continue;
                printf("[CHASSIS-RPMSG-ESOS] Read error: %s\n", strerror(errno));
                break;
            }
            if (n > 0) {
                parse_feedback(dev, recv_buf);
            }
        }

        gettimeofday(&now, NULL);
        float dt = (now.tv_sec - priv->last_odom_time.tv_sec) +
                (now.tv_usec - priv->last_odom_time.tv_usec) / 1e6f;

        if (dt >= 0.02f) {
            update_odometry(dev, dt);
            priv->last_odom_time = now;
        }

        float send_dt = (now.tv_sec - last_send.tv_sec) +
                (now.tv_usec - last_send.tv_usec) / 1e6f;

        if (send_dt >= priv->send_interval_ms / 1000.0f) {
            send_velocity_cmd(dev);
            last_send = now;
        }
    }

    printf("[CHASSIS-RPMSG-ESOS] RX thread stopped\n");
    return NULL;
}

/* ==========================================================================
 * Driver Operations
 * ========================================================================== */

static int rpmsg_esos_init(struct chassis_dev *dev) {
    struct rpmsg_esos_priv *priv = dev->priv_data;

    if (rpmsg_init(dev) < 0) {
        printf("[CHASSIS-RPMSG-ESOS] Failed to initialize RPMsg\n");
        return CHASSIS_ERR_CONNECT;
    }

    printf("[CHASSIS-RPMSG-ESOS] Initialized: %s\n", dev->name);
    printf("[CHASSIS-RPMSG-ESOS] ctrl=%s, data=%s\n",
            priv->ctrl_dev ? priv->ctrl_dev : "(null)",
            priv->data_dev ? priv->data_dev : "(null)");
    printf("[CHASSIS-RPMSG-ESOS] wheel_diameter=%.3f m, wheel_base=%.3f m\n",
            dev->config.wheel_diameter, dev->config.wheel_base);

    return CHASSIS_OK;
}

static int rpmsg_esos_start(struct chassis_dev *dev) {
    dev->running = true;

    int ret = pthread_create(&dev->rx_thread, NULL, rx_thread_func, dev);
    if (ret != 0) {
        printf("[CHASSIS-RPMSG-ESOS] Failed to create RX thread: %s\n",
                strerror(ret));
        dev->running = false;
        return CHASSIS_ERR_ALLOC;
    }

    printf("[CHASSIS-RPMSG-ESOS] Started\n");
    return CHASSIS_OK;
}

static int rpmsg_esos_stop(struct chassis_dev *dev) {
    chassis_velocity_t zero = {0.0f, 0.0f, 0.0f};
    rpmsg_esos_set_velocity(dev, &zero);

    dev->running = false;

    if (dev->rx_thread) {
        pthread_join(dev->rx_thread, NULL);
        dev->rx_thread = 0;
    }

    printf("[CHASSIS-RPMSG-ESOS] Stopped\n");
    return CHASSIS_OK;
}

static int rpmsg_esos_set_velocity(struct chassis_dev *dev,
        const chassis_velocity_t *vel) {
    struct rpmsg_esos_priv *priv = dev->priv_data;

    pthread_mutex_lock(&priv->vel_lock);
    priv->target_vx = vel->vx;
    priv->target_wz = vel->wz;
    pthread_mutex_unlock(&priv->vel_lock);

    return CHASSIS_OK;
}

static void rpmsg_esos_brake(struct chassis_dev *dev) {
    chassis_velocity_t zero = {0.0f, 0.0f, 0.0f};
    rpmsg_esos_set_velocity(dev, &zero);
    send_velocity_cmd(dev);
}

static void rpmsg_esos_free(struct chassis_dev *dev) {
    struct rpmsg_esos_priv *priv;

    if (!dev)
        return;

    priv = dev->priv_data;
    if (priv) {
        rpmsg_cleanup(priv);
        pthread_mutex_destroy(&priv->vel_lock);
    }

    chassis_dev_free_default(dev);
}

static const struct chassis_ops rpmsg_esos_ops = {
    .init = rpmsg_esos_init,
    .start = rpmsg_esos_start,
    .stop = rpmsg_esos_stop,
    .set_velocity = rpmsg_esos_set_velocity,
    .brake = rpmsg_esos_brake,
    .free = rpmsg_esos_free,
};

/* ==========================================================================
 * Factory Function
 * ========================================================================== */

static struct chassis_dev *
rpmsg_esos_create(const char *name, const struct chassis_config *config) {
    struct chassis_dev *dev;
    struct rpmsg_esos_priv *priv;
    const struct chassis_rpmsg_config *rpmsg_config;

    if (!name || !config)
        return NULL;

    /* Cast to RPMSG-specific config */
    rpmsg_config = (const struct chassis_rpmsg_config *)config;

    dev = chassis_dev_alloc(name, &rpmsg_config->base, sizeof(*priv));
    if (!dev)
        return NULL;

    priv = dev->priv_data;
    dev->ops = &rpmsg_esos_ops;

    /* Use config values or defaults */
    priv->ctrl_dev =
        rpmsg_config->ctrl_dev ? rpmsg_config->ctrl_dev : DEFAULT_RPMSG_CTRL_DEV;
    priv->data_dev =
        rpmsg_config->data_dev ? rpmsg_config->data_dev : DEFAULT_RPMSG_DATA_DEV;
    priv->service_name = rpmsg_config->service_name ?
            rpmsg_config->service_name : DEFAULT_RPMSG_SERVICE_NAME;
    priv->local_addr = rpmsg_config->local_addr ?
            rpmsg_config->local_addr : DEFAULT_RPMSG_LOCAL_ADDR;
    priv->remote_addr = rpmsg_config->remote_addr ?
            rpmsg_config->remote_addr : DEFAULT_RPMSG_REMOTE_ADDR;

    priv->rpmsg_fd = -1;
    priv->rpmsg_ctrl_fd = -1;
    priv->send_interval_ms = 50;  /* 20Hz */

    pthread_mutex_init(&priv->vel_lock, NULL);

    if (rpmsg_esos_init(dev) != CHASSIS_OK) {
        chassis_dev_free_default(dev);
        return NULL;
    }

    if (rpmsg_esos_start(dev) != CHASSIS_OK) {
        rpmsg_esos_free(dev);
        return NULL;
    }

    return dev;
}

/* Register driver */
REGISTER_CHASSIS_DRIVER("drv_rpmsg_esos", rpmsg_esos_create);
