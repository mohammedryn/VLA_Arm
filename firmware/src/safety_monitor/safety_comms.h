// firmware/src/safety_monitor/safety_comms.h
#pragma once
#include <stdint.h>

#define SAFETY_MAGIC  0xBEEFCAFEUL

typedef struct __attribute__((packed)) {
    uint32_t magic;         // Always SAFETY_MAGIC — framing marker
    uint32_t timestamp_us;  // micros()
    uint8_t  contact_flag;  // 1 = contact oracle triggered (latched)
    float    contact_rms;   // Gyro RMS in deg/s (4 bytes)
    uint8_t  estop_active;  // 1 = this packet carries an ESTOP request
    uint16_t checksum;      // sum of bytes 0–13, truncated to uint16
} SafetyStatus_t;           // 16 bytes: 4+4+1+4+1+2

static_assert(sizeof(SafetyStatus_t) == 16, "SafetyStatus_t must be 16 bytes");
