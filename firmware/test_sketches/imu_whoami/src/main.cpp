#include <Arduino.h>
#include <SPI.h>

#define IMU_CS    5
#define IMU_MOSI  23
#define IMU_MISO  19
#define IMU_SCK   18
#define IMU_FREQ  1000000UL   // use 1MHz for bring-up; main firmware uses 10MHz

#define REG_WHO_AM_I  0x0F
#define EXPECTED_ID   0x6B

static uint8_t imu_read_reg(uint8_t reg) {
    uint8_t val;
    digitalWrite(IMU_CS, LOW);
    SPI.beginTransaction(SPISettings(IMU_FREQ, MSBFIRST, SPI_MODE0));
    SPI.transfer(reg | 0x80);  // bit7=1 for read
    val = SPI.transfer(0x00);
    SPI.endTransaction();
    digitalWrite(IMU_CS, HIGH);
    return val;
}

void setup() {
    Serial.begin(115200);
    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH);
    SPI.begin(IMU_SCK, IMU_MISO, IMU_MOSI, IMU_CS);
    delay(100);

    Serial.println("\n=== IMU WHO_AM_I Test ===");
    uint8_t id = imu_read_reg(REG_WHO_AM_I);
    Serial.printf("WHO_AM_I = 0x%02X  →  %s\n\n", id,
                  id == EXPECTED_ID ? "IMU OK: 0x6B — PASS"
                                    : "FAIL — expected 0x6B, check wiring");
}

void loop() {
    delay(2000);
    uint8_t id = imu_read_reg(REG_WHO_AM_I);
    Serial.printf("WHO_AM_I: 0x%02X (%s)\n", id,
                  id == EXPECTED_ID ? "OK" : "FAIL");
}
