/*
 * mpu6050.c
 *
 * MPU6050 + Mahony 6-axis Sensor Fusion
 *
 * Accelerometer:
 *      AX, AY, AZ  [g]
 *
 * Gyroscope:
 *      GX, GY, GZ  [deg/s]
 *
 * Sensor fusion:
 *      Accelerometer + Gyroscope
 *              ↓
 *          Mahony Filter
 *              ↓
 *          Quaternion
 *              ↓
 *        Euler Angles
 *              ↓
 *       Roll / Pitch / Yaw
 */


#include "mpu6050.h"
#include <stdint.h>
#include <math.h>


/* ============================================================
 * MPU6050 configuration
 * ============================================================ */

#define ACCEL_SENS      16384.0f       /* ±2g */

#define GYRO_SENS       131.0f         /* ±250 deg/s */


/* ============================================================
 * Mahony filter parameters
 *
 * Kp:
 *      Proportional correction.
 *      Giá trị lớn -> bám accelerometer nhanh hơn.
 *
 * Ki:
 *      Integral correction.
 *      Giúp loại bỏ sai lệch chậm / bias còn lại.
 * ============================================================ */

#define MAHONY_KP       2.0f
#define MAHONY_KI       0.05f


/* ============================================================
 * Gyroscope dead-zone
 *
 * Đơn vị: deg/s
 *
 * Giá trị rất nhỏ quanh 0 được xem là 0.
 * ============================================================ */

#define GYRO_DEADZONE   0.05f


/* ============================================================
 * Global sensor data
 * ============================================================ */

int16_t ax_raw = 0;
int16_t ay_raw = 0;
int16_t az_raw = 0;

int16_t gx_raw = 0;
int16_t gy_raw = 0;
int16_t gz = 0;


/* Accelerometer [g] */
float AX = 0.0f;
float AY = 0.0f;
float AZ = 0.0f;


/* Gyroscope [deg/s] */
float GX = 0.0f;
float GY = 0.0f;
float GZ = 0.0f;


/* ============================================================
 * Gyroscope calibration
 * ============================================================ */

float GX_calib = 0.0f;
float GY_calib = 0.0f;
float GZ_calib = 0.0f;


/* ============================================================
 * Euler angles
 * ============================================================ */

volatile float yaw = 0.0f;

float pitch = 0.0f;
float roll  = 0.0f;


/* ============================================================
 * Quaternion
 *
 * q0 = w
 * q1 = x
 * q2 = y
 * q3 = z
 *
 * Initial orientation = identity quaternion
 *
 *       q = [1, 0, 0, 0]
 * ============================================================ */

float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;


/* ============================================================
 * MPU6050 status
 * ============================================================ */

volatile uint8_t mpu6050_dataReady = 0;

uint8_t mpu6050_whoAmI = 0x00;

volatile uint32_t mpu6050_irqCount = 0;


/* ============================================================
 * MPU6050 INIT
 * ============================================================ */

uint8_t mpu6050_Init(void)
{
    uint8_t check;
    uint8_t mData;


    /* --------------------------------------------------------
     * Wait for MPU6050 power-up
     * -------------------------------------------------------- */

    HAL_Delay(100);


    /* --------------------------------------------------------
     * WHO_AM_I
     *
     * MPU6050 normally returns 0x68.
     * -------------------------------------------------------- */

    HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR,
        0x75,
        1,
        &check,
        1,
        10
    );

    mpu6050_whoAmI = check;


    if (check != 0x68 && check != 0x74)
    {
        return 0;
    }


    /* --------------------------------------------------------
     * Wake up MPU6050
     *
     * PWR_MGMT_1 = 0x01
     *
     * Clock source = PLL with X gyro reference
     * -------------------------------------------------------- */

    mData = 0x01;

    HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        0x6B,
        1,
        &mData,
        1,
        10
    );

    HAL_Delay(10);


    /* --------------------------------------------------------
     * Sample Rate
     *
     * Internal gyro rate = 1 kHz
     *
     * SMPLRT_DIV = 4
     *
     * Sample rate:
     *
     *      1000 / (1 + 4)
     *      = 200 Hz
     * -------------------------------------------------------- */

    mData = 0x04;

    HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        0x19,
        1,
        &mData,
        1,
        10
    );


    /* --------------------------------------------------------
     * DLPF
     *
     * CONFIG = 0x03
     * -------------------------------------------------------- */

    mData = 0x03;

    HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        0x1A,
        1,
        &mData,
        1,
        10
    );


    /* --------------------------------------------------------
     * Gyroscope Full Scale
     *
     * ±250 deg/s
     *
     * Sensitivity = 131 LSB/(deg/s)
     * -------------------------------------------------------- */

    mData = 0x00;

    HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        0x1B,
        1,
        &mData,
        1,
        10
    );


    /* --------------------------------------------------------
     * Accelerometer Full Scale
     *
     * ±2g
     *
     * Sensitivity = 16384 LSB/g
     * -------------------------------------------------------- */

    mData = 0x00;

    HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        0x1C,
        1,
        &mData,
        1,
        10
    );


    /* --------------------------------------------------------
     * INT_PIN_CFG
     *
     * 0x30:
     *
     * Bit 5 = LATCH_INT_EN
     * Bit 4 = INT_RD_CLEAR
     * -------------------------------------------------------- */

    mData = 0x30;

    HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        0x37,
        1,
        &mData,
        1,
        10
    );


    /* --------------------------------------------------------
     * Enable Data Ready interrupt
     * -------------------------------------------------------- */

    mData = 0x01;

    HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        0x38,
        1,
        &mData,
        1,
        10
    );


    return 1;
}


/* ============================================================
 * GYROSCOPE CALIBRATION
 * ============================================================ */

void mpu6050_Calibrate(void)
{
    long gx_sum = 0;
    long gy_sum = 0;
    long gz_sum = 0;

    const int samples = 500;

    uint8_t data[6];


    /*
     * MPU6050 PHẢI ĐƯỢC GIỮ YÊN trong quá trình calibration.
     */

    HAL_Delay(100);


    for (int i = 0; i < samples; i++)
    {
        /*
         * Đọc:
         *
         * GYRO_XOUT_H = 0x43
         *
         * 0x43 -> GX
         * 0x45 -> GY
         * 0x47 -> GZ
         */

        HAL_I2C_Mem_Read(
            &hi2c1,
            MPU6050_ADDR,
            0x43,
            1,
            data,
            6,
            10
        );


        int16_t raw_gx =
            (int16_t)((data[0] << 8) | data[1]);

        int16_t raw_gy =
            (int16_t)((data[2] << 8) | data[3]);

        int16_t raw_gz =
            (int16_t)((data[4] << 8) | data[5]);


        gx_sum += raw_gx;
        gy_sum += raw_gy;
        gz_sum += raw_gz;


        HAL_Delay(5);
    }


    /*
     * Convert raw gyro bias
     * sang deg/s.
     */

    GX_calib =
        ((float)gx_sum / (float)samples)
        / GYRO_SENS;

    GY_calib =
        ((float)gy_sum / (float)samples)
        / GYRO_SENS;

    GZ_calib =
        ((float)gz_sum / (float)samples)
        / GYRO_SENS;


    /* --------------------------------------------------------
     * Reset orientation
     * -------------------------------------------------------- */

    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    roll  = 0.0f;
    pitch = 0.0f;
    yaw   = 0.0f;


    /* Clear data ready flag */

    mpu6050_dataReady = 0;


    /* Clear MPU6050 interrupt status */

    uint8_t dummy;

    HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR,
        0x3A,
        1,
        &dummy,
        1,
        10
    );
}


/* ============================================================
 * READ MOTION 6
 *
 * Burst read 14 bytes:
 *
 * 0x3B:
 *
 * ACCEL_X
 * ACCEL_Y
 * ACCEL_Z
 * TEMP
 * GYRO_X
 * GYRO_Y
 * GYRO_Z
 * ============================================================ */

void mpu6050_readMotion6(void)
{
    uint8_t buffer[14];


    HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR,
        0x3B,
        1,
        buffer,
        14,
        10
    );


    /* --------------------------------------------------------
     * Accelerometer raw
     * -------------------------------------------------------- */

    ax_raw =
        (int16_t)((buffer[0] << 8) | buffer[1]);

    ay_raw =
        (int16_t)((buffer[2] << 8) | buffer[3]);

    az_raw =
        (int16_t)((buffer[4] << 8) | buffer[5]);


    /* --------------------------------------------------------
     * Gyroscope raw
     *
     * buffer[6..7] = temperature
     *
     * buffer[8..9]  = GX
     * buffer[10..11] = GY
     * buffer[12..13] = GZ
     * -------------------------------------------------------- */

    gx_raw =
        (int16_t)((buffer[8] << 8) | buffer[9]);

    gy_raw =
        (int16_t)((buffer[10] << 8) | buffer[11]);

    gz =
        (int16_t)((buffer[12] << 8) | buffer[13]);


    /* --------------------------------------------------------
     * Convert accelerometer
     *
     * ±2g -> 16384 LSB/g
     * -------------------------------------------------------- */

    AX = (float)ax_raw / ACCEL_SENS;
    AY = (float)ay_raw / ACCEL_SENS;
    AZ = (float)az_raw / ACCEL_SENS;


    /* --------------------------------------------------------
     * Convert gyroscope
     *
     * ±250 deg/s -> 131 LSB/(deg/s)
     * -------------------------------------------------------- */

    GX = (float)gx_raw / GYRO_SENS;
    GY = (float)gy_raw / GYRO_SENS;
    GZ = (float)gz / GYRO_SENS;
}


/* ============================================================
 * MAHONY 6-AXIS SENSOR FUSION
 *
 * Input:
 *
 *      AX AY AZ
 *      GX GY GZ
 *
 * Output:
 *
 *      q0 q1 q2 q3
 *
 *      roll
 *      pitch
 *      yaw
 *
 * ============================================================ */

void mpu6050_updateQuaternion(float dt)
{
    /* --------------------------------------------------------
     * Local variables
     * -------------------------------------------------------- */

    float ax = AX;
    float ay = AY;
    float az = AZ;

    float gx = GX - GX_calib;
    float gy = GY - GY_calib;
    float gz_val = GZ - GZ_calib;


    /* --------------------------------------------------------
     * Gyroscope dead-zone
     * -------------------------------------------------------- */

    if (fabsf(gx) < GYRO_DEADZONE)
        gx = 0.0f;

    if (fabsf(gy) < GYRO_DEADZONE)
        gy = 0.0f;

    if (fabsf(gz_val) < GYRO_DEADZONE)
        gz_val = 0.0f;


    /* --------------------------------------------------------
     * Normalize accelerometer
     *
     * Accelerometer should represent gravity:
     *
     *      sqrt(ax² + ay² + az²) ≈ 1g
     *
     * Nếu magnitude quá nhỏ thì bỏ correction.
     * -------------------------------------------------------- */

    float acc_norm =
        sqrtf(ax * ax + ay * ay + az * az);


    if (acc_norm > 0.01f)
    {
        ax /= acc_norm;
        ay /= acc_norm;
        az /= acc_norm;


        /* ----------------------------------------------------
         * Estimated gravity direction from quaternion
         *
         * halfvx, halfvy, halfvz
         * ---------------------------------------------------- */

        float halfvx =
            q1 * q3 - q0 * q2;

        float halfvy =
            q0 * q1 + q2 * q3;

        float halfvz =
            q0 * q0
            - 0.5f
            + q3 * q3;


        /* ----------------------------------------------------
         * Error = cross(measured gravity,
         *               estimated gravity)
         * ---------------------------------------------------- */

        float halfex =
            ay * halfvz
            - az * halfvy;

        float halfey =
            az * halfvx
            - ax * halfvz;

        float halfez =
            ax * halfvy
            - ay * halfvx;


        /* ----------------------------------------------------
         * Integral feedback
         *
         * Static variables preserve their values between
         * function calls.
         * ---------------------------------------------------- */

        static float integral_x = 0.0f;
        static float integral_y = 0.0f;
        static float integral_z = 0.0f;


        /*
         * Ki * error * dt
         */

        integral_x +=
            MAHONY_KI * halfex * dt;

        integral_y +=
            MAHONY_KI * halfey * dt;

        integral_z +=
            MAHONY_KI * halfez * dt;


        /*
         * Proportional + Integral correction
         */

        gx +=
            MAHONY_KP * halfex
            + integral_x;

        gy +=
            MAHONY_KP * halfey
            + integral_y;

        gz_val +=
            MAHONY_KP * halfez
            + integral_z;
    }


    /* --------------------------------------------------------
     * Gyroscope:
     *
     * deg/s -> rad/s
     * -------------------------------------------------------- */

    gx *= 0.0174532925199433f;
    gy *= 0.0174532925199433f;
    gz_val *= 0.0174532925199433f;


    /* --------------------------------------------------------
     * Quaternion derivative
     *
     * q_dot = 0.5 * q * gyro
     * -------------------------------------------------------- */

    float qDot0 =
        0.5f *
        (-q1 * gx
         - q2 * gy
         - q3 * gz_val);

    float qDot1 =
        0.5f *
        ( q0 * gx
         + q2 * gz_val
         - q3 * gy);

    float qDot2 =
        0.5f *
        ( q0 * gy
         - q1 * gz_val
         + q3 * gx);

    float qDot3 =
        0.5f *
        ( q0 * gz_val
         + q1 * gy
         - q2 * gx);


    /* --------------------------------------------------------
     * Integrate quaternion
     * -------------------------------------------------------- */

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;


    /* --------------------------------------------------------
     * Normalize quaternion
     * -------------------------------------------------------- */

    float q_norm =
        sqrtf(
            q0 * q0 +
            q1 * q1 +
            q2 * q2 +
            q3 * q3
        );


    if (q_norm > 0.0f)
    {
        q0 /= q_norm;
        q1 /= q_norm;
        q2 /= q_norm;
        q3 /= q_norm;
    }


    /* ========================================================
     * Convert Quaternion -> Euler angles
     * ======================================================== */

    /* --------------------------------------------------------
     * Roll
     *
     * Rotation around X
     * -------------------------------------------------------- */

    roll =
        atan2f(
            2.0f * (q0 * q1 + q2 * q3),
            1.0f - 2.0f * (q1 * q1 + q2 * q2)
        )
        * RTD;


    /* --------------------------------------------------------
     * Pitch
     *
     * Clamp asin input to [-1, +1]
     * to prevent numerical error.
     * -------------------------------------------------------- */

    float pitch_sin =
        2.0f * (q0 * q2 - q3 * q1);


    if (pitch_sin > 1.0f)
        pitch_sin = 1.0f;

    if (pitch_sin < -1.0f)
        pitch_sin = -1.0f;


    pitch =
        asinf(pitch_sin)
        * RTD;


    /* --------------------------------------------------------
     * Yaw
     *
     * Rotation around Z
     * -------------------------------------------------------- */

    yaw =
        atan2f(
            2.0f * (q0 * q3 + q1 * q2),
            1.0f - 2.0f * (q2 * q2 + q3 * q3)
        )
        * RTD;


    /* --------------------------------------------------------
     * Normalize Yaw to [-180, +180]
     * -------------------------------------------------------- */

    if (yaw > 180.0f)
        yaw -= 360.0f;

    if (yaw < -180.0f)
        yaw += 360.0f;
}


/* ============================================================
 * ANGLE DIFFERENCE
 * ============================================================ */

float mpu6050_angleDiff(float target, float current)
{
    float diff = target - current;


    if (diff > 180.0f)
        diff -= 360.0f;


    if (diff < -180.0f)
        diff += 360.0f;


    return diff;
}


/* ============================================================
 * MPU6050 DATA READY INTERRUPT
 * ============================================================ */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU6050_PIN_INT)
    {
        mpu6050_dataReady = 1;

        mpu6050_irqCount++;
    }
}