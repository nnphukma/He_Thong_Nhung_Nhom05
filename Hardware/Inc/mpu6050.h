#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"
#include <stdint.h>
#include <math.h>

#define MPU6050_ADDR       0xD0U
#define MPU6050_INT_PORT   GPIOB
#define MPU6050_PIN_INT    GPIO_PIN_12

#define ACCEL_SENS         16384.0f
#define GYRO_SENS          131.0f

#define GYRO_DEADZONE      0.0f

/* ============================================================
 * Kalman filter parameters (angle in degrees, rate in deg/s)
 *
 * KALMAN_Q_ANGLE : process noise of angle   [deg^2 / s]
 * KALMAN_Q_BIAS  : process noise of bias    [(deg/s)^2 / s]
 * KALMAN_R_MEAS  : accelerometer angle noise [deg^2]
 *
 * Tang R_MEAS  -> tin accel it hon, muot hon nhung cham hon.
 * Tang Q_ANGLE -> tin gyro it hon, bam accel nhanh hon.
 * ============================================================ */
#define KALMAN_Q_ANGLE     0.001f
#define KALMAN_Q_BIAS      0.003f
#define KALMAN_R_MEAS      0.03f

/*
 * Chi hieu chinh bang accelerometer khi |a| nam trong
 * khoang 1g +/- ACC_GATE (loai bo khi khung dang gia toc manh).
 */
#define ACC_GATE           0.25f

extern I2C_HandleTypeDef hi2c1;

/* Accelerometer [g] */
extern float AX;
extern float AY;
extern float AZ;

/* Gyroscope [deg/s] */
extern float GX;
extern float GY;
extern float GZ;

/* Gyroscope calibration bias [deg/s] */
extern float GX_calib;
extern float GY_calib;
extern float GZ_calib;

/* Raw accelerometer */
extern int16_t ax_raw;
extern int16_t ay_raw;
extern int16_t az_raw;

/* Raw gyroscope */
extern int16_t gx_raw;
extern int16_t gy_raw;
extern int16_t gz_raw;

/* Quaternion: q = [qw, qx, qy, qz]  (yaw = 0, tao tu roll/pitch Kalman) */
extern float qw;
extern float qx;
extern float qy;
extern float qz;

/* Goc Kalman [deg] - dung truc tiep cho bo dieu khien servo */
extern float roll_deg;
extern float pitch_deg;

/* MPU6050 status */
extern volatile uint8_t mpu6050_dataReady;
extern volatile uint32_t mpu6050_irqCount;
extern volatile uint32_t mpu6050_missedSamples;
extern uint8_t mpu6050_whoAmI;

/* MPU6050 functions */
uint8_t mpu6050_Init(void);
void mpu6050_Calibrate(void);
void mpu6050_TimingInit(void);
float mpu6050_GetDeltaTime(void);
uint8_t mpu6050_readMotion6(void);
void mpu6050_updateQuaternion(float dt);   /* giu ten cu: nay chay Kalman */
void MPU6050_SendUART(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* __MPU6050_H */