/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file drv_uart_diff.c
 * @brief UART driver for differential (2WD) chassis
 *
 * Protocol (compatible with existing ROS2 driver):
 * TX: "{dir_l},{speed_l:.2f};{dir_r},{speed_r:.2f}\n"
 *     - dir: 0=stop, 1=forward, 2=backward
 *     - speed: rev/s (rotations per second)
 *
 * RX: "...;{speed},{dir},{...};{speed},{dir},{...}\n"
 *     - Parse last two semicolon-separated fields
 *     - items[-3]=speed(rev/s), items[-2]=direction(0/1/2)
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#include "../chassis_core.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ==========================================================================
 * Private Data Structure
 * ========================================================================== */

struct uart_diff_priv {
    int fd;

    /* UART-specific config */
    const char *dev_path;
    uint32_t baud;

    /* Target velocity */
    float target_vx;
    float target_wz;
    pthread_mutex_t vel_lock;

    /* Wheel speed from feedback (m/s) */
    float wheel_left_mps;
    float wheel_right_mps;

    /* Timing */
    struct timeval last_odom_time;
    int send_interval_ms;  /* Send command interval (default 50ms = 20Hz) */
};

/* ==========================================================================
 * Helper Functions
 * ========================================================================== */

static speed_t baud_to_speed(uint32_t baud) {
    switch (baud) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 921600:
        return B921600;
    default:
        return B115200;
    }
}

static int serial_open(const char *path, uint32_t baud) {
    int fd = open(path, O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0) {
        printf("[CHASSIS-UART-DIFF] Failed to open %s: %s\n",
                path, strerror(errno));
        return -1;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) < 0) {
        close(fd);
        return -1;
    }

    cfmakeraw(&tio);
    speed_t sp = baud_to_speed(baud);
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);

    /* 8N1 */
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag |= CLOCAL | CREAD;

    /* Non-blocking read with timeout */
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tio) < 0) {
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

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

static float parse_motor_field(const char *field, float diam) {
    const char *items[16];
    int count = 0;
    char buf[128];
    char *saveptr = NULL;
    strncpy(buf, field, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *token = strtok_r(buf, ",", &saveptr);
    while (token && count < 16) {
        items[count++] = token;
        token = strtok_r(NULL, ",", &saveptr);
    }

    if (count < 2)
        return 0.0f;

    float speed_revs = atof(items[count - 3]);
    int dir = atoi(items[count - 2]);

    if (dir == 2)
        speed_revs = -speed_revs;
    else if (dir == 0)
        speed_revs = 0.0f;

    return speed_revs * M_PI * diam;
}

/* Forward declaration */
static int uart_diff_set_velocity(struct chassis_dev *dev,
        const chassis_velocity_t *vel);

/* ==========================================================================
 * Send Command
 * ========================================================================== */

static int send_velocity_cmd(struct chassis_dev *dev) {
    struct uart_diff_priv *priv = dev->priv_data;
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
    int len = snprintf(cmd, sizeof(cmd), "%d,%.2f;%d,%.2f\n",
            dir_l, spd_l, dir_r, spd_r);

    if (priv->fd >= 0 && len > 0) {
        ssize_t written = write(priv->fd, cmd, len);
        if (written < 0) {
            printf("[CHASSIS-UART-DIFF] Write error: %s\n", strerror(errno));
            return -1;
        }
    }

    return 0;
}

/* ==========================================================================
 * Receive Thread
 * ========================================================================== */

static void update_odometry(struct chassis_dev *dev, float dt) {
    struct uart_diff_priv *priv = dev->priv_data;
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

static void parse_feedback_line(struct chassis_dev *dev, const char *line) {
    struct uart_diff_priv *priv = dev->priv_data;
    float wheel_diam = dev->config.wheel_diameter;

    const char *fields[32];
    int count = 0;
    char buf[256];
    char *saveptr = NULL;
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *token = strtok_r(buf, ";", &saveptr);
    while (token && count < 32) {
        fields[count++] = token;
        token = strtok_r(NULL, ";", &saveptr);
    }

    if (count < 2)
        return;

    float v_l = parse_motor_field(fields[count - 2], wheel_diam);
    float v_r = parse_motor_field(fields[count - 1], wheel_diam);

    priv->wheel_left_mps = v_l;
    priv->wheel_right_mps = v_r;
}

static void *rx_thread_func(void *arg) {
    struct chassis_dev *dev = arg;
    struct uart_diff_priv *priv = dev->priv_data;

    char buf[512];
    int buf_pos = 0;

    struct timeval now, last_send;
    gettimeofday(&last_send, NULL);
    gettimeofday(&priv->last_odom_time, NULL);

    printf("[CHASSIS-UART-DIFF] RX thread started\n");

    while (dev->running) {
        if (priv->fd >= 0) {
            ssize_t n = read(priv->fd, buf + buf_pos, sizeof(buf) - buf_pos - 1);
            if (n > 0) {
                buf_pos += n;
                buf[buf_pos] = '\0';

                char *newline;
                while ((newline = strchr(buf, '\n')) != NULL) {
                    *newline = '\0';
                    if (strlen(buf) > 0) {
                        parse_feedback_line(dev, buf);
                    }

                    int remain = buf_pos - (newline - buf + 1);
                    if (remain > 0) {
                        memmove(buf, newline + 1, remain);
                    }
                    buf_pos = remain;
                    buf[buf_pos] = '\0';
                }
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

        usleep(1000);
    }

    printf("[CHASSIS-UART-DIFF] RX thread stopped\n");
    return NULL;
}

/* ==========================================================================
 * Driver Operations
 * ========================================================================== */

static int uart_diff_init(struct chassis_dev *dev) {
    struct uart_diff_priv *priv = dev->priv_data;

    if (priv->dev_path) {
        priv->fd = serial_open(priv->dev_path, priv->baud);
        if (priv->fd < 0) {
            printf("[CHASSIS-UART-DIFF] Failed to open serial port\n");
        }
    }

    printf("[CHASSIS-UART-DIFF] Initialized: %s\n", dev->name);
    printf("[CHASSIS-UART-DIFF] dev=%s, baud=%u\n",
            priv->dev_path ? priv->dev_path : "(null)", priv->baud);
    printf("[CHASSIS-UART-DIFF] wheel_diameter=%.3f m, wheel_base=%.3f m\n",
            dev->config.wheel_diameter, dev->config.wheel_base);

    return CHASSIS_OK;
}

static int uart_diff_start(struct chassis_dev *dev) {
    dev->running = true;

    int ret = pthread_create(&dev->rx_thread, NULL, rx_thread_func, dev);
    if (ret != 0) {
        printf("[CHASSIS-UART-DIFF] Failed to create RX thread: %s\n",
                strerror(ret));
        dev->running = false;
        return CHASSIS_ERR_ALLOC;
    }

    printf("[CHASSIS-UART-DIFF] Started\n");
    return CHASSIS_OK;
}

static int uart_diff_stop(struct chassis_dev *dev) {
    chassis_velocity_t zero = {0.0f, 0.0f, 0.0f};
    uart_diff_set_velocity(dev, &zero);

    dev->running = false;

    if (dev->rx_thread) {
        pthread_join(dev->rx_thread, NULL);
        dev->rx_thread = 0;
    }

    printf("[CHASSIS-UART-DIFF] Stopped\n");
    return CHASSIS_OK;
}

static int uart_diff_set_velocity(struct chassis_dev *dev,
        const chassis_velocity_t *vel) {
    struct uart_diff_priv *priv = dev->priv_data;

    pthread_mutex_lock(&priv->vel_lock);
    priv->target_vx = vel->vx;
    priv->target_wz = vel->wz;
    pthread_mutex_unlock(&priv->vel_lock);

    return CHASSIS_OK;
}

static void uart_diff_brake(struct chassis_dev *dev) {
    chassis_velocity_t zero = {0.0f, 0.0f, 0.0f};
    uart_diff_set_velocity(dev, &zero);
    send_velocity_cmd(dev);
}

static void uart_diff_free(struct chassis_dev *dev) {
    struct uart_diff_priv *priv;

    if (!dev)
        return;

    priv = dev->priv_data;
    if (priv) {
        if (priv->fd >= 0) {
            close(priv->fd);
            priv->fd = -1;
        }
        pthread_mutex_destroy(&priv->vel_lock);
    }

    chassis_dev_free_default(dev);
}

static const struct chassis_ops uart_diff_ops = {
    .init = uart_diff_init,
    .start = uart_diff_start,
    .stop = uart_diff_stop,
    .set_velocity = uart_diff_set_velocity,
    .brake = uart_diff_brake,
    .free = uart_diff_free,
};

/* ==========================================================================
 * Factory Function
 * ========================================================================== */

static struct chassis_dev *
uart_diff_create(const char *name, const struct chassis_config *config) {
    struct chassis_dev *dev;
    struct uart_diff_priv *priv;
    const struct chassis_uart_config *uart_config;

    if (!name || !config)
        return NULL;

    /* Cast to UART-specific config */
    uart_config = (const struct chassis_uart_config *)config;

    dev = chassis_dev_alloc(name, &uart_config->base, sizeof(*priv));
    if (!dev)
        return NULL;

    priv = dev->priv_data;
    dev->ops = &uart_diff_ops;

    /* Store UART-specific config in private data */
    priv->dev_path = uart_config->dev_path;
    priv->baud = uart_config->baud;
    priv->fd = -1;
    priv->send_interval_ms = 50;  /* 20Hz */

    pthread_mutex_init(&priv->vel_lock, NULL);

    /* Initialize and start */
    if (uart_diff_init(dev) != CHASSIS_OK) {
        chassis_dev_free_default(dev);
        return NULL;
    }

    if (uart_diff_start(dev) != CHASSIS_OK) {
        uart_diff_free(dev);
        return NULL;
    }

    return dev;
}

/* Register driver */
REGISTER_CHASSIS_DRIVER("drv_uart_esp32", uart_diff_create);
