// firmware/src/sensor_safety/sensor_main.cpp
#include <Arduino.h>
#include <string.h>
#include "sensor_comms.h"
#include "sensor_config.h"
#include "../ism330dhcx_driver.h"
#include "../tof_driver.h"
#include "../contact_oracle.h"

static uint16_t compute_checksum(const uint8_t* data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

static elapsedMicros loop_timer;

void setup() {
    Serial.begin(115200);  // baud ignored by Teensy native USB CDC; kept for pyserial API compat

    imu_init();
    tof_init();
    contact_oracle_init(SAFETY_CONTACT_THRESHOLD);

    loop_timer = 0;
}

void loop() {
    if (loop_timer < CONTROL_PERIOD_US) return;
    loop_timer -= CONTROL_PERIOD_US;

    imu_fifo_read_batch();   // updates latest IMU sample, feeds contact_oracle_push()
    tof_check_ready();       // refreshes latest ToF frame if a new one is ready (~15Hz)

    bool estop = (contact_oracle_rms() > SAFETY_CONTACT_THRESHOLD * 3.0f);

    ImuData  imu = imu_get_latest();
    ToFFrame tof = tof_get_latest();

    SensorStatus_t pkt = {};
    pkt.magic        = SENSOR_PACKET_MAGIC;
    pkt.timestamp_us = micros();
    pkt.contact_flag = contact_oracle_triggered() ? 1 : 0;
    pkt.contact_rms  = contact_oracle_rms();
    pkt.estop_active = estop ? 1 : 0;
    pkt.accel[0] = imu.ax; pkt.accel[1] = imu.ay; pkt.accel[2] = imu.az;
    pkt.gyro[0]  = imu.gx; pkt.gyro[1]  = imu.gy; pkt.gyro[2]  = imu.gz;
    memcpy(pkt.tof_mm, tof.distances_mm, sizeof(pkt.tof_mm));
    pkt.checksum = compute_checksum((const uint8_t*)&pkt, sizeof(SensorStatus_t) - sizeof(pkt.checksum));

    Serial.write((const uint8_t*)&pkt, sizeof(SensorStatus_t));
}
