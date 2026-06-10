// firmware/src/sensor_safety/sensor_config.h
#pragma once

// I2C bus (Teensy 4.1 default Wire: SDA=18, SCL=19)
#define IMU_I2C_ADDR            0x6B
#define TOF_I2C_ADDR            0x52
#define TOF_LPN                 2      // power enable, HIGH=powered
#define TOF_INT                 3      // data-ready, active LOW
#define TOF_UPDATE_HZ           15

// Contact oracle
#define CONTACT_WINDOW          8       // samples, ~38ms at 208Hz
#define SAFETY_CONTACT_THRESHOLD 3.5f   // deg/s RMS, calibrated value carried over

// Control loop
#define CONTROL_HZ              50
#define CONTROL_PERIOD_US       20000   // 20ms
