#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"
#include <stdint.h>
#include <math.h>

/* ============================================================
 * Phan cung
 * ============================================================ */

#define MPU6050_ADDR            0xD0U           /* 0x68 << 1 (AD0 = GND) */
#define MPU6050_INT_PORT        GPIOB
#define MPU6050_PIN_INT         GPIO_PIN_12
#define MPU6050_I2C_TIMEOUT_MS  10U

/* ============================================================
 * Thanh ghi MPU6050
 * ============================================================ */

#define MPU6050_REG_SMPLRT_DIV  0x19U
#define MPU6050_REG_CONFIG      0x1AU
#define MPU6050_REG_GYRO_CFG    0x1BU
#define MPU6050_REG_ACCEL_CFG   0x1CU
#define MPU6050_REG_INT_PIN_CFG 0x37U
#define MPU6050_REG_INT_ENABLE  0x38U
#define MPU6050_REG_INT_STATUS  0x3AU
#define MPU6050_REG_ACCEL_XOUT  0x3BU
#define MPU6050_REG_PWR_MGMT_1  0x6BU
#define MPU6050_REG_WHO_AM_I    0x75U

/* ============================================================
 * Thong so lay mau / do nhay
 * ============================================================ */

#define MPU6050_SAMPLE_RATE_HZ  200U            /* 1 kHz / (1 + DIV)      */
#define MPU6050_DT_NOMINAL      (1.0f / (float)MPU6050_SAMPLE_RATE_HZ)
#define MPU6050_DT_MIN          0.001f
#define MPU6050_DT_MAX          0.050f

#define ACCEL_SENS              16384.0f        /* LSB/g     (+-2 g)      */
#define GYRO_SENS               131.0f          /* LSB/(deg/s) (+-250)    */

/* ============================================================
 * Bo loc Mahony
 *
 * MAHONY_KP : [1/s]  = 1 / hang so thoi gian.  Kp = 2 -> tau = 0.5 s
 *             Lon  -> tin accel nhieu hon (nhanh hoi tu, nhay nhieu rung)
 *             Nho  -> tin gyro nhieu hon (muot, nhung hoi tu cham)
 * MAHONY_KI : tich phan, bu bias gyro con sot (troi nhiet). 0 = tat.
 * ============================================================ */

#define MAHONY_KP               2.0f
#define MAHONY_KI               0.01f
#define MAHONY_I_LIMIT          0.10f           /* rad/s, chan windup     */

/* Chi tin accel khi |a| gan 1 g (bo qua khi bi gia toc / rung manh) */
#define ACC_TRUST_MIN_G         0.85f
#define ACC_TRUST_MAX_G         1.15f

/* ============================================================
 * Hieu chuan
 * ============================================================ */

#define MPU6050_CALIB_SAMPLES   500
#define MPU6050_CALIB_MAX_STD_DPS 1.0f          /* vuot -> coi la dang di chuyen */

extern I2C_HandleTypeDef hi2c1;

/* ============================================================
 * Du lieu cam bien
 * ============================================================ */

/* Gia toc [g] */
extern float AX, AY, AZ;

/* Toc do goc [deg/s] - DA TRU BIAS (gyro) */
extern float GX, GY, GZ;

/* Bias gyro do luc hieu chuan [deg/s] */
extern float GX_calib, GY_calib, GZ_calib;

/* Du lieu tho */
extern int16_t ax_raw, ay_raw, az_raw;
extern int16_t gx_raw, gy_raw, gz_raw;

/* Quaternion q = [qw, qx, qy, qz] */
extern float qw, qx, qy, qz;

/* Goc Euler [deg] (yaw khong dung vi khong co tu ke -> troi) */
extern float roll_deg;
extern float pitch_deg;

/* Trang thai */
extern volatile uint8_t  mpu6050_dataReady;
extern volatile uint32_t mpu6050_irqCount;
extern volatile uint32_t mpu6050_missedSamples;
extern volatile uint32_t mpu6050_i2cErrors;
extern uint8_t mpu6050_whoAmI;

/* ============================================================
 * Ham
 * ============================================================ */

uint8_t mpu6050_Init(void);                 /* 1 = OK, 0 = loi                 */
uint8_t mpu6050_Calibrate(void);            /* 1 = OK, 0 = dang di chuyen/loi  */
void    mpu6050_TimingInit(void);
float   mpu6050_GetDeltaTime(void);
uint8_t mpu6050_readMotion6(void);          /* 1 = OK, 0 = loi I2C             */
void    mpu6050_updateQuaternion(float dt);
void    mpu6050_updateEuler(void);
void    MPU6050_SendUART(void);             /* CSV quaternion (de debug)       */

void    HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* __MPU6050_H */
