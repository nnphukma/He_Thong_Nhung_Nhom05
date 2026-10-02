#include "mpu6050.h"
#include "usart.h"

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

/* ============================================================
 * Global sensor data
 * ============================================================ */

int16_t ax_raw = 0, ay_raw = 0, az_raw = 0;
int16_t gx_raw = 0, gy_raw = 0, gz_raw = 0;

float AX = 0.0f, AY = 0.0f, AZ = 0.0f;
float GX = 0.0f, GY = 0.0f, GZ = 0.0f;

float GX_calib = 0.0f;
float GY_calib = 0.0f;
float GZ_calib = 0.0f;

/* Quaternion */
float qw = 1.0f;
float qx = 0.0f;
float qy = 0.0f;
float qz = 0.0f;

/* MPU6050 status */
volatile uint8_t mpu6050_dataReady = 0;
volatile uint32_t mpu6050_irqCount = 0;
volatile uint32_t mpu6050_missedSamples = 0;

uint8_t mpu6050_whoAmI = 0x00;

/* DWT timing */
static uint32_t mpu6050_lastUpdateCycle = 0;
static uint8_t mpu6050_timingReady = 0;


/* ============================================================
 * DWT Timing
 * ============================================================ */

void mpu6050_TimingInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    mpu6050_lastUpdateCycle = DWT->CYCCNT;
    mpu6050_missedSamples = 0;
    mpu6050_timingReady = 1;
}


float mpu6050_GetDeltaTime(void)
{
    uint32_t now;
    uint32_t cycles;
    float dt;

    if (!mpu6050_timingReady)
        return 0.005f;

    now = DWT->CYCCNT;
    cycles = now - mpu6050_lastUpdateCycle;

    mpu6050_lastUpdateCycle = now;

    dt = (float)cycles / (float)HAL_RCC_GetHCLKFreq();

    /*
     * Safety range:
     * Normal value at 200 Hz ≈ 0.005 s
     */
    if (dt < 0.001f || dt > 0.05f)
        dt = 0.005f;

    return dt;
}


/* ============================================================
 * MPU6050 Initialization
 * ============================================================ */

uint8_t mpu6050_Init(void)
{
    uint8_t check;
    uint8_t mData;

    HAL_Delay(100);

    /* WHO_AM_I */
    if (HAL_I2C_Mem_Read(&hi2c1,
                         MPU6050_ADDR,
                         0x75,
                         1,
                         &check,
                         1,
                         10) != HAL_OK)
    {
        mpu6050_whoAmI = 0x00;
        return 0;
    }

    mpu6050_whoAmI = check;

    if (check != 0x68 && check != 0x74)
        return 0;

    /* Wake up MPU6050 */
    mData = 0x01;

    if (HAL_I2C_Mem_Write(&hi2c1,
                          MPU6050_ADDR,
                          0x6B,
                          1,
                          &mData,
                          1,
                          10) != HAL_OK)
    {
        return 0;
    }

    HAL_Delay(10);

    /* Sample Rate Divider
     * 1 kHz / (1 + 4) = 200 Hz
     */
    mData = 0x04;

    if (HAL_I2C_Mem_Write(&hi2c1,
                          MPU6050_ADDR,
                          0x19,
                          1,
                          &mData,
                          1,
                          10) != HAL_OK)
    {
        return 0;
    }

    /* DLPF */
    mData = 0x03;

    if (HAL_I2C_Mem_Write(&hi2c1,
                          MPU6050_ADDR,
                          0x1A,
                          1,
                          &mData,
                          1,
                          10) != HAL_OK)
    {
        return 0;
    }

    /* Gyroscope ±250 °/s */
    mData = 0x00;

    if (HAL_I2C_Mem_Write(&hi2c1,
                          MPU6050_ADDR,
                          0x1B,
                          1,
                          &mData,
                          1,
                          10) != HAL_OK)
    {
        return 0;
    }

    /* Accelerometer ±2g */
    mData = 0x00;

    if (HAL_I2C_Mem_Write(&hi2c1,
                          MPU6050_ADDR,
                          0x1C,
                          1,
                          &mData,
                          1,
                          10) != HAL_OK)
    {
        return 0;
    }

    /* INT pin:
     * Push-pull, active high
     */
    mData = 0x30;

    if (HAL_I2C_Mem_Write(&hi2c1,
                          MPU6050_ADDR,
                          0x37,
                          1,
                          &mData,
                          1,
                          10) != HAL_OK)
    {
        return 0;
    }

    /* Enable Data Ready interrupt */
    mData = 0x01;

    if (HAL_I2C_Mem_Write(&hi2c1,
                          MPU6050_ADDR,
                          0x38,
                          1,
                          &mData,
                          1,
                          10) != HAL_OK)
    {
        return 0;
    }

    return 1;
}


/* ============================================================
 * Gyroscope Calibration
 * ============================================================ */

void mpu6050_Calibrate(void)
{
    long gx_sum = 0;
    long gy_sum = 0;
    long gz_sum = 0;

    const int samples = 1000;

    uint8_t data[6];
    uint8_t dummy;

    HAL_Delay(100);

    for (int i = 0; i < samples; i++)
    {
        if (HAL_I2C_Mem_Read(&hi2c1,
                             MPU6050_ADDR,
                             0x43,
                             1,
                             data,
                             6,
                             10) == HAL_OK)
        {
            int16_t raw_gx = (int16_t)((data[0] << 8) | data[1]);
            int16_t raw_gy = (int16_t)((data[2] << 8) | data[3]);
            int16_t raw_gz = (int16_t)((data[4] << 8) | data[5]);

            gx_sum += raw_gx;
            gy_sum += raw_gy;
            gz_sum += raw_gz;
        }

        HAL_Delay(5);
    }

    GX_calib = ((float)gx_sum / samples) / GYRO_SENS;
    GY_calib = ((float)gy_sum / samples) / GYRO_SENS;
    GZ_calib = ((float)gz_sum / samples) / GYRO_SENS;

    /* Reset quaternion */
    qw = 1.0f;
    qx = 0.0f;
    qy = 0.0f;
    qz = 0.0f;

    mpu6050_dataReady = 0;

    /*
     * Clear MPU6050 interrupt status.
     */
    HAL_I2C_Mem_Read(&hi2c1,
                     MPU6050_ADDR,
                     0x3A,
                     1,
                     &dummy,
                     1,
                     10);
}


/* ============================================================
 * Read Accelerometer + Gyroscope
 *
 * Return:
 *   1 = successful I2C read
 *   0 = I2C read failed
 * ============================================================ */

uint8_t mpu6050_readMotion6(void)
{
    uint8_t buffer[14];

    /*
     * Read:
     *
     * 0x3B - ACCEL_XOUT_H
     * 0x3C - ACCEL_XOUT_L
     * 0x3D - ACCEL_YOUT_H
     * 0x3E - ACCEL_YOUT_L
     * 0x3F - ACCEL_ZOUT_H
     * 0x40 - ACCEL_ZOUT_L
     * 0x41 - TEMP_OUT_H
     * 0x42 - TEMP_OUT_L
     * 0x43 - GYRO_XOUT_H
     * 0x44 - GYRO_XOUT_L
     * 0x45 - GYRO_YOUT_H
     * 0x46 - GYRO_YOUT_L
     * 0x47 - GYRO_ZOUT_H
     * 0x48 - GYRO_ZOUT_L
     */

    if (HAL_I2C_Mem_Read(&hi2c1,
                         MPU6050_ADDR,
                         0x3B,
                         1,
                         buffer,
                         14,
                         10) != HAL_OK)
    {
        return 0;
    }

    /* Accelerometer raw */
    ax_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    ay_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
    az_raw = (int16_t)((buffer[4] << 8) | buffer[5]);

    /* Gyroscope raw */
    gx_raw = (int16_t)((buffer[8] << 8) | buffer[9]);
    gy_raw = (int16_t)((buffer[10] << 8) | buffer[11]);
    gz_raw = (int16_t)((buffer[12] << 8) | buffer[13]);

    /* Convert accelerometer to g */
    AX = (float)ax_raw / ACCEL_SENS;
    AY = (float)ay_raw / ACCEL_SENS;
    AZ = (float)az_raw / ACCEL_SENS;

    /* Convert gyroscope to °/s */
    GX = (float)gx_raw / GYRO_SENS;
    GY = (float)gy_raw / GYRO_SENS;
    GZ = (float)gz_raw / GYRO_SENS;

    return 1;
}


/* ============================================================
 * Mahony Quaternion Update
 * ============================================================ */

void mpu6050_updateQuaternion(float dt)
{
    float ax = AX;
    float ay = AY;
    float az = AZ;

    float gx = GX - GX_calib;
    float gy = GY - GY_calib;
    float gz = GZ - GZ_calib;

    /* Gyro deadzone */
    if (fabsf(gx) < GYRO_DEADZONE)
        gx = 0.0f;

    if (fabsf(gy) < GYRO_DEADZONE)
        gy = 0.0f;

    if (fabsf(gz) < GYRO_DEADZONE)
        gz = 0.0f;

    /* Normalize accelerometer */
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);

    if (acc_norm > 0.01f)
    {
        ax /= acc_norm;
        ay /= acc_norm;
        az /= acc_norm;

        /*
         * Estimated gravity direction
         */
        float halfvx = qx * qz - qw * qy;
        float halfvy = qw * qx + qy * qz;
        float halfvz = qw * qw - 0.5f + qz * qz;

        /*
         * Error between measured and estimated gravity
         */
        float halfex = ay * halfvz - az * halfvy;
        float halfey = az * halfvx - ax * halfvz;
        float halfez = ax * halfvy - ay * halfvx;

        /* Mahony integral term */
        static float integral_x = 0.0f;
        static float integral_y = 0.0f;
        static float integral_z = 0.0f;

        integral_x += MAHONY_KI * halfex * dt;
        integral_y += MAHONY_KI * halfey * dt;
        integral_z += MAHONY_KI * halfez * dt;

        /*
         * Correct gyro
         */
        gx += MAHONY_KP * halfex + integral_x;
        gy += MAHONY_KP * halfey + integral_y;
        gz += MAHONY_KP * halfez + integral_z;
    }

    /* °/s -> rad/s */
    gx *= 0.0174532925199433f;
    gy *= 0.0174532925199433f;
    gz *= 0.0174532925199433f;

    /* Quaternion derivative */
    float qDotw = 0.5f * (-qx * gx - qy * gy - qz * gz);
    float qDotx = 0.5f * (qw * gx + qy * gz - qz * gy);
    float qDoty = 0.5f * (qw * gy - qx * gz + qz * gx);
    float qDotz = 0.5f * (qw * gz + qx * gy - qy * gx);

    /* Integrate */
    qw += qDotw * dt;
    qx += qDotx * dt;
    qy += qDoty * dt;
    qz += qDotz * dt;

    /* Normalize quaternion */
    float q_norm = sqrtf(qw * qw +
                         qx * qx +
                         qy * qy +
                         qz * qz);

    if (q_norm > 0.0f)
    {
        qw /= q_norm;
        qx /= q_norm;
        qy /= q_norm;
        qz /= q_norm;
    }
}


/* ============================================================
 * UART CSV Debug
 *
 * Format:
 * Time,GX,GY,GZ,QW,QX,QY,QZ
 * ============================================================ */

void MPU6050_SendUART(void)
{
    static uint8_t headerSent = 0;

    char buffer[160];
    int len;

    if (!headerSent)
    {
        const char *header =
            "Time,GX,GY,GZ,QW,QX,QY,QZ\r\n";

        HAL_UART_Transmit(&huart1,
                          (uint8_t *)header,
                          strlen(header),
                          100);

        headerSent = 1;
    }

    len = snprintf(buffer,
                   sizeof(buffer),
                   "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
                   HAL_GetTick() / 1000UL,
                   GX - GX_calib,
                   GY - GY_calib,
                   GZ - GZ_calib,
                   qw,
                   qx,
                   qy,
                   qz);

    if (len > 0)
    {
        HAL_UART_Transmit(&huart1,
                          (uint8_t *)buffer,
                          len,
                          100);
    }
}


/* ============================================================
 * MPU6050 Data Ready EXTI
 * ============================================================ */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU6050_PIN_INT)
    {
        /*
         * If main loop has not processed the previous sample
         * before the next interrupt arrives, count it as missed.
         */
        if (mpu6050_dataReady)
            mpu6050_missedSamples++;

        mpu6050_dataReady = 1;
        mpu6050_irqCount++;
    }
}