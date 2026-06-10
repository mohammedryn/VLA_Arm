// firmware/src/sensor_safety/sensor_comms.h
#pragma once
#include <stdint.h>

#define SENSOR_PACKET_MAGIC  0xBEEFCAFEUL

typedef struct __attribute__((packed)) {
    uint32_t magic;          // Always SENSOR_PACKET_MAGIC — framing marker
    uint32_t timestamp_us;   // micros() since Teensy boot
    uint8_t  contact_flag;   // 1 = contact oracle triggered (latched)
    float    contact_rms;    // Gyro RMS in deg/s
    uint8_t  estop_active;   // 1 = hard collision, RPi5 must stop
    float    accel[3];       // ax, ay, az in m/s^2
    float    gyro[3];        // gx, gy, gz in deg/s
    uint16_t tof_mm[64];     // 8x8 grid, row-major, mm; 0xFFFF = invalid zone
    uint16_t checksum;       // sum of bytes 0..165, truncated to uint16
} SensorStatus_t;             // 168 bytes total

static_assert(sizeof(SensorStatus_t) == 168, "SensorStatus_t must be 168 bytes");
