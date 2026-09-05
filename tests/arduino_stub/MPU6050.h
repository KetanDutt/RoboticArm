/* Host stub of the MPU6050 device library (jrowberg/i2cdevlib), covering the
 * subset of the API this project uses. */
#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <I2Cdev.h>

#define MPU6050_ADDRESS_AD0_LOW 0x68
#define MPU6050_ADDRESS_AD0_HIGH 0x69
#define MPU6050_DEFAULT_ADDRESS MPU6050_ADDRESS_AD0_LOW

#define MPU6050_ACCEL_FS_2 0x00
#define MPU6050_ACCEL_FS_4 0x01
#define MPU6050_ACCEL_FS_8 0x02
#define MPU6050_ACCEL_FS_16 0x03

#define MPU6050_GYRO_FS_250 0x00
#define MPU6050_GYRO_FS_500 0x01
#define MPU6050_GYRO_FS_1000 0x02
#define MPU6050_GYRO_FS_2000 0x03

#define MPU6050_DLPF_BW_256 0x00
#define MPU6050_DLPF_BW_188 0x01
#define MPU6050_DLPF_BW_98 0x02
#define MPU6050_DLPF_BW_42 0x03
#define MPU6050_DLPF_BW_20 0x04
#define MPU6050_DLPF_BW_10 0x05
#define MPU6050_DLPF_BW_5 0x06

class MPU6050 {
public:
    MPU6050();
    MPU6050(uint8_t address);

    void initialize();
    bool testConnection();

    uint8_t getFullScaleGyroRange();
    void setFullScaleGyroRange(uint8_t range);
    uint8_t getFullScaleAccelRange();
    void setFullScaleAccelRange(uint8_t range);
    uint8_t getDLPFMode();
    void setDLPFMode(uint8_t bandwidth);
    uint8_t getRate();
    void setRate(uint8_t rate);
    bool getSleepEnabled();
    void setSleepEnabled(bool enabled);
    void setI2CMasterModeEnabled(bool enabled);
    void setClockSource(uint8_t source);

    void getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx,
                    int16_t *gy, int16_t *gz);
    void getMotion9(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx,
                    int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my,
                    int16_t *mz);
    void getAcceleration(int16_t *ax, int16_t *ay, int16_t *az);
    void getRotation(int16_t *gx, int16_t *gy, int16_t *gz);
    int16_t getTemperature();
};

#endif /* MPU6050_H */
