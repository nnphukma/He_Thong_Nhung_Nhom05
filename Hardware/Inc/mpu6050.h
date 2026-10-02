#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h" /* Thay bằng thư viện HAL tương ứng với chip của bạn (f1, f4, g4...) */
#include <stdint.h>

/* ============================================================
 * Constants & Settings
 * ============================================================ */
#define MPU6050_DEFAULT_ADDR 0xD0 // 0x68 << 1

#define MAHONY_KP 2.0f
#define MAHONY_KI 0.05f
#define GYRO_DEADZONE 0.05f

/* Pre-calculated Inverse of Sensitivities (Multiplication is faster than division) */
#define ACCEL_SENS_INV 0.000061035156f  // 1.0f / 16384.0f (±2g)
#define GYRO_SENS_INV  0.00763358778f   // 1.0f / 131.0f (±250 deg/s)

#define RAD_TO_DEG 57.29577951308232f
#define DEG_TO_RAD 0.017453292519943f

/* ============================================================
 * MPU6050 Object Structure
 * ============================================================ */
typedef struct {
    /* I2C Handle */
    I2C_HandleTypeDef *hi2c;
    uint16_t addr;

    /* Raw Data */
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    /* Scaled Data */
    float ax, ay, az; // [g]
    float gx, gy, gz; // [deg/s]

    /* Calibration (Bias) */
    float gx_calib, gy_calib, gz_calib;

    /* Mahony Filter Variables */
    float integral_x, integral_y, integral_z;
    
    /* Quaternion */
    float q0, q1, q2, q3;

    /* Euler Angles */
    float roll, pitch, yaw;

    /* Status */
    uint8_t dataReady;
} MPU6050_t;

/* ============================================================
 * Function Prototypes
 * ============================================================ */
uint8_t MPU6050_Init(MPU6050_t *mpu, I2C_HandleTypeDef *hi2c, uint16_t addr);
void MPU6050_Calibrate(MPU6050_t *mpu);
void MPU6050_ReadMotion6(MPU6050_t *mpu);
void MPU6050_UpdateQuaternion(MPU6050_t *mpu, float dt);
float MPU6050_AngleDiff(float target, float current);

#endif /* MPU6050_H */
