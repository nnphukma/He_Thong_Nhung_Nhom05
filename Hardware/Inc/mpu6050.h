#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"
#include "math.h"
#include <stdint.h>


/* ============================================================
 * MPU6050 I2C
 * ============================================================ */

#define MPU6050_ADDR       0xD0U


/* ============================================================
 * Mathematical constants
 * ============================================================ */

#define RTD                57.29577951308232f


/* ============================================================
 * MPU6050 interrupt
 * ============================================================ */

#define MPU6050_INT_PORT   GPIOB
#define MPU6050_PIN_INT    GPIO_PIN_12


/* ============================================================
 * External I2C handle
 * ============================================================ */

extern I2C_HandleTypeDef hi2c1;


/* ============================================================
 * Public orientation data
 * ============================================================ */

/* Euler angles */
extern volatile float yaw;
extern float pitch;
extern float roll;


/* ============================================================
 * Raw / converted IMU data
 * ============================================================ */

/* Accelerometer [g] */
extern float AX;
extern float AY;
extern float AZ;

/* Gyroscope [deg/s] */
extern float GX;
extern float GY;
extern float GZ;


/* ============================================================
 * Gyroscope calibration
 * ============================================================ */

extern float GX_calib;
extern float GY_calib;
extern float GZ_calib;


/* ============================================================
 * Raw sensor values
 * ============================================================ */

extern int16_t ax_raw;
extern int16_t ay_raw;
extern int16_t az_raw;

extern int16_t gx_raw;
extern int16_t gy_raw;
extern int16_t gz;


/* ============================================================
 * MPU6050 status / debug
 * ============================================================ */

/* Data Ready flag from EXTI */
extern volatile uint8_t mpu6050_dataReady;

/* WHO_AM_I value */
extern uint8_t mpu6050_whoAmI;

/* Number of Data Ready interrupts */
extern volatile uint32_t mpu6050_irqCount;


/* ============================================================
 * Quaternion
 *
 * q0 = w
 * q1 = x
 * q2 = y
 * q3 = z
 * ============================================================ */

extern float q0;
extern float q1;
extern float q2;
extern float q3;


/* ============================================================
 * Functions
 * ============================================================ */

uint8_t mpu6050_Init(void);

void mpu6050_Calibrate(void);

void mpu6050_readMotion6(void);

/*
 * Mahony 6-axis sensor fusion.
 *
 * dt = th?i gian gi?a hai sample, don v? gi�y.
 */
void mpu6050_updateQuaternion(float dt);


/* ============================================================
 * Utility
 * ============================================================ */

float mpu6050_angleDiff(float target, float current);


/* ============================================================
 * External interrupt callback
 * ============================================================ */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


#endif /* __MPU6050_H */