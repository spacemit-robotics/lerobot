/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file test_chassis_forward_backward.c
 * @brief Simple forward/backward test program for UART differential chassis
 */

#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../include/chassis.h"

static volatile bool running = true;

static void signal_handler(int sig) {
    (void)sig;
    running = false;
}

static void usage(const char *prog) {
    printf("Usage: %s [OPTIONS]\n", prog);
    printf("\n");
    printf("Options:\n");
    printf("  -p, --port <path>       Serial port path (default: /dev/ttyUSB0)\n");
    printf("  -b, --baud <rate>       Baud rate (default: 115200)\n");
    printf("  -s, --speed <m/s>       Forward/backward speed (default: 0.20)\n");
    printf("  -d, --duration <sec>    Duration per move phase in seconds (default: 3.0)\n");
    printf("  -n, --cycles <count>    Number of forward/backward cycles (default: 3)\n");
    printf("  -h, --help              Show this help message\n");
    printf("\n");
    printf("Examples:\n");
    printf("  %s -p /dev/ttyUSB0 -s 0.25 -d 4 -n 2\n", prog);
}

static int send_velocity_for_duration(
        struct chassis_dev *chassis,
        const chassis_velocity_t *vel,
        int duration_ms,
        const char *label) {
    int elapsed_ms = 0;

    printf("-> %s: vx=%.3f m/s, vy=%.3f m/s, wz=%.3f rad/s, duration=%.1f s\n",
            label, vel->vx, vel->vy, vel->wz, duration_ms / 1000.0f);

    while (running && elapsed_ms < duration_ms) {
        int ret = chassis_set_velocity(chassis, vel);
        if (ret != CHASSIS_OK) {
            fprintf(stderr, "Failed to set velocity for %s: %d\n", label, ret);
            return ret;
        }

        usleep(100000); /* 100ms */
        elapsed_ms += 100;
    }

    return CHASSIS_OK;
}

static int stop_for_duration(struct chassis_dev *chassis, int duration_ms) {
    chassis_velocity_t stop = {0.0f, 0.0f, 0.0f};
    return send_velocity_for_duration(chassis, &stop, duration_ms, "stop");
}

int main(int argc, char *argv[]) {
    struct chassis_dev *chassis;
    struct chassis_uart_config config;

    const char *port = "/dev/ttyUSB0";
    uint32_t baud = 115200;
    float speed = 0.20f;
    float duration_s = 3.0f;
    int cycles = 3;

    for (int i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--port") == 0) &&
                i + 1 < argc) {
            port = argv[++i];
        } else if ((strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) &&
                i + 1 < argc) {
            baud = (uint32_t)atoi(argv[++i]);
        } else if ((strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--speed") == 0) &&
                i + 1 < argc) {
            speed = strtof(argv[++i], NULL);
        } else if ((strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--duration") == 0) &&
                i + 1 < argc) {
            duration_s = strtof(argv[++i], NULL);
        } else if ((strcmp(argv[i], "-n") == 0 || strcmp(argv[i], "--cycles") == 0) &&
                i + 1 < argc) {
            cycles = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown argument: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    if (speed < 0.0f) {
        speed = -speed;
    }
    if (duration_s <= 0.0f) {
        fprintf(stderr, "Duration must be greater than 0\n");
        return 1;
    }
    if (cycles <= 0) {
        fprintf(stderr, "Cycles must be greater than 0\n");
        return 1;
    }

    printf("=== Chassis Forward/Backward Test ===\n\n");
    printf("Port: %s\n", port);
    printf("Baud: %u\n", baud);
    printf("Speed: %.3f m/s\n", speed);
    printf("Duration per move phase: %.1f s\n", duration_s);
    printf("Cycles: %d\n\n", cycles);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    config = (struct chassis_uart_config){
        .base = {
            .type = CHASSIS_TYPE_DIFF_2WD,
            .wheel_diameter = 0.067f,
            .wheel_base = 0.33f,
            .wheel_track = 0.0f,
            .left_wheel_gain = 1.0f,
            .max_speed = 1.0f,
            .max_angular = 3.14f,
        },
        .dev_path = port,
        .baud = baud,
    };

    printf("[1] Creating chassis...\n");
    chassis = chassis_alloc("drv_uart_esp32", &config);
    if (!chassis) {
        fprintf(stderr, "Failed to create chassis\n");
        return 1;
    }
    printf("    Chassis created successfully\n\n");

    const int move_duration_ms = (int)(duration_s * 1000.0f);
    const int stop_duration_ms = 1000;

    for (int cycle = 0; running && cycle < cycles; cycle++) {
        printf("[2.%d] Cycle %d/%d\n", cycle + 1, cycle + 1, cycles);

        chassis_velocity_t forward = {speed, 0.0f, 0.0f};
        if (send_velocity_for_duration(chassis, &forward, move_duration_ms, "forward") !=
                CHASSIS_OK) {
            break;
        }

        if (running && stop_for_duration(chassis, stop_duration_ms) != CHASSIS_OK) {
            break;
        }

        chassis_velocity_t backward = {-speed, 0.0f, 0.0f};
        if (running &&
                send_velocity_for_duration(chassis, &backward, move_duration_ms, "backward") !=
                        CHASSIS_OK) {
            break;
        }

        if (running && stop_for_duration(chassis, stop_duration_ms) != CHASSIS_OK) {
            break;
        }
    }

    printf("\n[3] Braking chassis...\n");
    chassis_brake(chassis);

    chassis_velocity_t final_vel;
    chassis_pose_t final_pose;
    if (chassis_get_odom(chassis, &final_vel, &final_pose) == CHASSIS_OK) {
        printf("Final odometry:\n");
        printf("  Position: (%.3f, %.3f) m\n", final_pose.x, final_pose.y);
        printf("  Heading:  %.3f rad\n", final_pose.yaw);
        printf("  Velocity: vx=%.3f m/s, vy=%.3f m/s, wz=%.3f rad/s\n",
                final_vel.vx, final_vel.vy, final_vel.wz);
    }

    printf("\n[4] Cleaning up...\n");
    chassis_free(chassis);
    printf("Done\n");

    return 0;
}
