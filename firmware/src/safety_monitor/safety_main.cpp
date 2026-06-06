// firmware/src/safety_monitor/safety_main.cpp
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include "safety_comms.h"
#include "../ism330dhcx_driver.h"
#include "../contact_oracle.h"
#include "config.h"

// Contact oracle fires when 8-sample gyro RMS exceeds this (deg/s).
// Calibrate by watching contact_rms during normal motion vs. contact.
#define SAFETY_CONTACT_THRESHOLD  3.5f

static uint16_t compute_checksum(const uint8_t* data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

static void send_safety_packet(bool estop) {
    SafetyStatus_t pkt = {};
    pkt.magic        = SAFETY_MAGIC;
    pkt.timestamp_us = micros();
    pkt.contact_flag = contact_oracle_triggered() ? 1 : 0;
    pkt.contact_rms  = contact_oracle_rms();
    pkt.estop_active = estop ? 1 : 0;
    pkt.checksum     = compute_checksum(
        (const uint8_t*)&pkt,
        sizeof(SafetyStatus_t) - sizeof(pkt.checksum)
    );
    Serial.write((const uint8_t*)&pkt, sizeof(SafetyStatus_t));
}

void safety_task(void* pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz

    while (true) {
        imu_fifo_read_batch();  // Reads IMU FIFO, internally calls contact_oracle_push()

        // Escalate to ESTOP if contact RMS is way above threshold (hard collision)
        bool estop = (contact_oracle_rms() > SAFETY_CONTACT_THRESHOLD * 3.0f);

        send_safety_packet(estop);

        vTaskDelayUntil(&lastWakeTime, period);
    }
}

void setup() {
    WiFi.mode(WIFI_OFF);
    btStop();

    Serial.begin(USB_BAUD);
    delay(500);
    Serial.println("--- SAFETY MONITOR FIRMWARE ---");

    imu_init();
    contact_oracle_init(SAFETY_CONTACT_THRESHOLD);

    Serial.println("IMU ready. Starting safety task at 50Hz.");

    xTaskCreatePinnedToCore(safety_task, "SafetyTask", 4096, NULL, 10, NULL, 1);
    vTaskDelete(NULL);
}

void loop() {}
