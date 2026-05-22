#include "ism330dhcx_driver.h"
#include "config.h"
#include "contact_oracle.h"
#include <SPI.h>

// ISM330DHCX Register Definitions
#define ISM_WHO_AM_I         0x0F  // Expected value: 0x6B
#define ISM_CTRL1_XL         0x10  // Accelerometer ODR & range
#define ISM_CTRL2_G          0x11  // Gyroscope ODR & range
#define ISM_CTRL3_C          0x12  // Control register 3
#define ISM_FIFO_CTRL1       0x07  // FIFO watermark low
#define ISM_FIFO_CTRL2       0x08  // FIFO watermark high + config
#define ISM_FIFO_CTRL3       0x09  // FIFO batch data rates
#define ISM_FIFO_CTRL4       0x0A  // FIFO mode and config
#define ISM_FIFO_STATUS1     0x3A  // FIFO level low
#define ISM_FIFO_STATUS2     0x3B  // FIFO level high + flags
#define ISM_FIFO_DATA_OUT_TAG 0x78

#define GYRO_SENSITIVITY_MDPS_PER_LSB   8.75f
#define ACCEL_SENSITIVITY_MG_PER_LSB    0.061f
#define G_TO_MS2                        0.00980665f

static ImuData latest_imu = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// SPI helper: write one register
static void imu_write_reg(uint8_t reg, uint8_t val) {
    digitalWrite(IMU_SPI_CS, LOW);
    SPI.beginTransaction(SPISettings(IMU_SPI_FREQ, MSBFIRST, SPI_MODE0));
    SPI.transfer(reg & 0x7F);  // bit7=0 for write
    SPI.transfer(val);
    SPI.endTransaction();
    digitalWrite(IMU_SPI_CS, HIGH);
}

// SPI helper: read one register
static uint8_t imu_read_reg(uint8_t reg) {
    uint8_t val;
    digitalWrite(IMU_SPI_CS, LOW);
    SPI.beginTransaction(SPISettings(IMU_SPI_FREQ, MSBFIRST, SPI_MODE0));
    SPI.transfer(reg | 0x80);  // bit7=1 for read
    val = SPI.transfer(0x00);
    SPI.endTransaction();
    digitalWrite(IMU_SPI_CS, HIGH);
    return val;
}

void imu_init() {
    pinMode(IMU_SPI_CS, OUTPUT);
    digitalWrite(IMU_SPI_CS, HIGH);
    SPI.begin();
    delay(10);

    // Verify WHO_AM_I
    while (imu_read_reg(ISM_WHO_AM_I) != 0x6B) {
        Serial.println("IMU not found! Retrying IMU SPI WHO_AM_I check...");
        delay(500);
    }

    // CTRL3_C: BDU=1 (Block Data Update), IF_INC=1 (auto increment address for burst reads)
    imu_write_reg(ISM_CTRL3_C, 0x44);

    // CTRL1_XL: ODR_XL[3:0]=1010 (6.67kHz), FS_XL[1:0]=00 (±2g range) -> 0xA0
    imu_write_reg(ISM_CTRL1_XL, 0xA0);

    // CTRL2_G: ODR_G[3:0]=1010 (6.67kHz), FS_G[1:0]=00 (±250dps range) -> 0xA0
    imu_write_reg(ISM_CTRL2_G, 0xA0);

    // FIFO_CTRL3: Gyro and Accel batch at 6.67kHz
    // Gyro batch rate BDR_GY[3:0]=1010, Accel batch rate BDR_XL[3:0]=1010 -> 0xAA
    imu_write_reg(ISM_FIFO_CTRL3, 0xAA);

    // FIFO_CTRL4: FIFO Mode = Continuous (overwrites oldest on full) -> 0x06
    imu_write_reg(ISM_FIFO_CTRL4, 0x06);

    delay(5);  // ODR settling
}

bool imu_who_am_i() {
    return (imu_read_reg(ISM_WHO_AM_I) == 0x6B);
}

uint16_t imu_fifo_depth() {
    uint8_t status1 = imu_read_reg(ISM_FIFO_STATUS1);
    uint8_t status2 = imu_read_reg(ISM_FIFO_STATUS2);
    // Depth is stored in 10-bit format
    return (((uint16_t)(status2 & 0x03) << 8) | status1);
}

void imu_fifo_read_batch() {
    uint16_t samples = imu_fifo_depth();

    // Burst read all available FIFO words (each word is 7 bytes: 1 tag + 6 data)
    for (uint16_t i = 0; i < samples; i++) {
        uint8_t word[7];
        
        digitalWrite(IMU_SPI_CS, LOW);
        SPI.beginTransaction(SPISettings(IMU_SPI_FREQ, MSBFIRST, SPI_MODE0));
        SPI.transfer(ISM_FIFO_DATA_OUT_TAG | 0x80);
        for (int b = 0; b < 7; b++) {
            word[b] = SPI.transfer(0x00);
        }
        SPI.endTransaction();
        digitalWrite(IMU_SPI_CS, HIGH);

        uint8_t tag = word[0] >> 3;
        int16_t x = (int16_t)(word[1] | (word[2] << 8));
        int16_t y = (int16_t)(word[3] | (word[4] << 8));
        int16_t z = (int16_t)(word[5] | (word[6] << 8));

        if (tag == 0x01) {  // Gyroscope NC sample
            latest_imu.gx = (float)x * GYRO_SENSITIVITY_MDPS_PER_LSB / 1000.0f;
            latest_imu.gy = (float)y * GYRO_SENSITIVITY_MDPS_PER_LSB / 1000.0f;
            latest_imu.gz = (float)z * GYRO_SENSITIVITY_MDPS_PER_LSB / 1000.0f;
            
            // Push high-frequency gyro data directly to contact oracle
            contact_oracle_push(latest_imu.gx, latest_imu.gy, latest_imu.gz);
        } 
        else if (tag == 0x02) {  // Accelerometer NC sample
            latest_imu.ax = (float)x * ACCEL_SENSITIVITY_MG_PER_LSB * G_TO_MS2;
            latest_imu.ay = (float)y * ACCEL_SENSITIVITY_MG_PER_LSB * G_TO_MS2;
            latest_imu.az = (float)z * ACCEL_SENSITIVITY_MG_PER_LSB * G_TO_MS2;
        }
    }
}

ImuData imu_get_latest() {
    return latest_imu;
}
