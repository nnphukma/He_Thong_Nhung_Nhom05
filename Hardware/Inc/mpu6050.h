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

#define MAHONY_KP          2.0f
#define MAHONY_KI          0.0f
#define GYRO_DEADZONE      0.0f

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

/* Quaternion: q = [qw, qx, qy, qz] */
extern float qw;
extern float qx;
extern float qy;
extern float qz;

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
void mpu6050_updateQuaternion(float dt);
void MPU6050_SendUART(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* __MPU6050_H */