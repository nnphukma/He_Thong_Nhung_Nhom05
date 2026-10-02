#include "mpu6050.h"
#include <math.h>

/* ============================================================
 * Fast Inverse Square Root (Quake III Algorithm)
 * Tính 1/sqrt(x) nhanh hơn 4 lần so với dùng phép chia và sqrtf()
 * Dùng union để tránh lỗi Strict Aliasing trong C tiêu chuẩn.
 * ============================================================ */
static float fast_invSqrt(float x) {
    float halfx = 0.5f * x;
    union {
        float f;
        uint32_t i;
    } conv;
    
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));
    return conv.f;
}

/* ============================================================
 * MPU6050 INIT
 * ============================================================ */
uint8_t MPU6050_Init(MPU6050_t *mpu, I2C_HandleTypeDef *hi2c, uint16_t addr) {
    uint8_t check;
    uint8_t mData;

    /* Gắn context */
    mpu->hi2c = hi2c;
    mpu->addr = addr;

    /* Khởi tạo giá trị mặc định cho Filter */
    mpu->q0 = 1.0f; mpu->q1 = 0.0f; mpu->q2 = 0.0f; mpu->q3 = 0.0f;
    mpu->integral_x = 0.0f; mpu->integral_y = 0.0f; mpu->integral_z = 0.0f;
    mpu->gx_calib = 0.0f; mpu->gy_calib = 0.0f; mpu->gz_calib = 0.0f;
    mpu->dataReady = 0;

    HAL_Delay(100);

    /* Kiểm tra WHO_AM_I */
    HAL_I2C_Mem_Read(mpu->hi2c, mpu->addr, 0x75, 1, &check, 1, 10);
    if (check != 0x68 && check != 0x74) return 0;

    /* Đánh thức MPU6050 (PWR_MGMT_1) */
    mData = 0x01;
    HAL_I2C_Mem_Write(mpu->hi2c, mpu->addr, 0x6B, 1, &mData, 1, 10);
    HAL_Delay(10);

    /* Cài đặt Sample Rate (SMPLRT_DIV = 4 -> 200Hz) */
    mData = 0x04;
    HAL_I2C_Mem_Write(mpu->hi2c, mpu->addr, 0x19, 1, &mData, 1, 10);

    /* Cài đặt DLPF (CONFIG = 0x03) */
    mData = 0x03;
    HAL_I2C_Mem_Write(mpu->hi2c, mpu->addr, 0x1A, 1, &mData, 1, 10);

    /* Gyro Full Scale: ±250 deg/s */
    mData = 0x00;
    HAL_I2C_Mem_Write(mpu->hi2c, mpu->addr, 0x1B, 1, &mData, 1, 10);

    /* Accel Full Scale: ±2g */
    mData = 0x00;
    HAL_I2C_Mem_Write(mpu->hi2c, mpu->addr, 0x1C, 1, &mData, 1, 10);

    /* Cấu hình chân Ngắt INT (LATCH_INT_EN | INT_RD_CLEAR) */
    mData = 0x30;
    HAL_I2C_Mem_Write(mpu->hi2c, mpu->addr, 0x37, 1, &mData, 1, 10);

    /* Bật ngắt Data Ready */
    mData = 0x01;
    HAL_I2C_Mem_Write(mpu->hi2c, mpu->addr, 0x38, 1, &mData, 1, 10);

    return 1;
}

/* ============================================================
 * GYROSCOPE CALIBRATION
 * ============================================================ */
void MPU6050_Calibrate(MPU6050_t *mpu) {
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;
    const int samples = 500;
    uint8_t data[6];

    HAL_Delay(100);

    for (int i = 0; i < samples; i++) {
        HAL_I2C_Mem_Read(mpu->hi2c, mpu->addr, 0x43, 1, data, 6, 10);

        gx_sum += (int16_t)((data[0] << 8) | data[1]);
        gy_sum += (int16_t)((data[2] << 8) | data[3]);
        gz_sum += (int16_t)((data[4] << 8) | data[5]);

        HAL_Delay(5);
    }

    /* Tính trung bình và nhân với nghịch đảo độ nhạy */
    float sample_inv = 1.0f / (float)samples;
    mpu->gx_calib = (float)gx_sum * sample_inv * GYRO_SENS_INV;
    mpu->gy_calib = (float)gy_sum * sample_inv * GYRO_SENS_INV;
    mpu->gz_calib = (float)gz_sum * sample_inv * GYRO_SENS_INV;

    /* Reset Quaternion & góc */
    mpu->q0 = 1.0f; mpu->q1 = 0.0f; mpu->q2 = 0.0f; mpu->q3 = 0.0f;
    mpu->roll = 0.0f; mpu->pitch = 0.0f; mpu->yaw = 0.0f;
    mpu->dataReady = 0;

    /* Clear ngắt hiện tại */
    uint8_t dummy;
    HAL_I2C_Mem_Read(mpu->hi2c, mpu->addr, 0x3A, 1, &dummy, 1, 10);
}

/* ============================================================
 * READ MOTION 6
 * ============================================================ */
void MPU6050_ReadMotion6(MPU6050_t *mpu) {
    uint8_t buffer[14];

    HAL_I2C_Mem_Read(mpu->hi2c, mpu->addr, 0x3B, 1, buffer, 14, 10);

    mpu->ax_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    mpu->ay_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
    mpu->az_raw = (int16_t)((buffer[4] << 8) | buffer[5]);

    mpu->gx_raw = (int16_t)((buffer[8] << 8) | buffer[9]);
    mpu->gy_raw = (int16_t)((buffer[10] << 8) | buffer[11]);
    mpu->gz_raw = (int16_t)((buffer[12] << 8) | buffer[13]);

    /* Thay thế phép chia bằng phép nhân hằng số nghịch đảo */
    mpu->ax = (float)mpu->ax_raw * ACCEL_SENS_INV;
    mpu->ay = (float)mpu->ay_raw * ACCEL_SENS_INV;
    mpu->az = (float)mpu->az_raw * ACCEL_SENS_INV;

    mpu->gx = (float)mpu->gx_raw * GYRO_SENS_INV;
    mpu->gy = (float)mpu->gy_raw * GYRO_SENS_INV;
    mpu->gz = (float)mpu->gz_raw * GYRO_SENS_INV;
}

/* ============================================================
 * MAHONY SENSOR FUSION UPDATE
 * ============================================================ */
void MPU6050_UpdateQuaternion(MPU6050_t *mpu, float dt) {
    float ax = mpu->ax;
    float ay = mpu->ay;
    float az = mpu->az;

    float gx = mpu->gx - mpu->gx_calib;
    float gy = mpu->gy - mpu->gy_calib;
    float gz_val = mpu->gz - mpu->gz_calib;

    /* Gyro dead-zone */
    if (fabsf(gx) < GYRO_DEADZONE) gx = 0.0f;
    if (fabsf(gy) < GYRO_DEADZONE) gy = 0.0f;
    if (fabsf(gz_val) < GYRO_DEADZONE) gz_val = 0.0f;

    /* Bỏ qua tính toán nếu accel bằng 0 để tránh lỗi NaN */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        /* Sử dụng Fast Inverse Square Root thay cho chia sqrtf() */
        float norm = fast_invSqrt(ax * ax + ay * ay + az * az);
        ax *= norm;
        ay *= norm;
        az *= norm;

        float halfvx = mpu->q1 * mpu->q3 - mpu->q0 * mpu->q2;
        float halfvy = mpu->q0 * mpu->q1 + mpu->q2 * mpu->q3;
        float halfvz = mpu->q0 * mpu->q0 - 0.5f + mpu->q3 * mpu->q3;

        float halfex = ay * halfvz - az * halfvy;
        float halfey = az * halfvx - ax * halfvz;
        float halfez = ax * halfvy - ay * halfvx;

        /* Sử dụng biến integral từ struct thay vì static local */
        mpu->integral_x += MAHONY_KI * halfex * dt;
        mpu->integral_y += MAHONY_KI * halfey * dt;
        mpu->integral_z += MAHONY_KI * halfez * dt;

        gx += MAHONY_KP * halfex + mpu->integral_x;
        gy += MAHONY_KP * halfey + mpu->integral_y;
        gz_val += MAHONY_KP * halfez + mpu->integral_z;
    }

    /* Convert sang rad/s bằng phép nhân */
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz_val *= DEG_TO_RAD;

    float qDot0 = 0.5f * (-mpu->q1 * gx - mpu->q2 * gy - mpu->q3 * gz_val);
    float qDot1 = 0.5f * ( mpu->q0 * gx + mpu->q2 * gz_val - mpu->q3 * gy);
    float qDot2 = 0.5f * ( mpu->q0 * gy - mpu->q1 * gz_val + mpu->q3 * gx);
    float qDot3 = 0.5f * ( mpu->q0 * gz_val + mpu->q1 * gy - mpu->q2 * gx);

    mpu->q0 += qDot0 * dt;
    mpu->q1 += qDot1 * dt;
    mpu->q2 += qDot2 * dt;
    mpu->q3 += qDot3 * dt;

    /* Dùng Fast Inverse Square Root để chuẩn hóa Quaternion */
    float norm = fast_invSqrt(mpu->q0 * mpu->q0 + mpu->q1 * mpu->q1 + 
                              mpu->q2 * mpu->q2 + mpu->q3 * mpu->q3);
    mpu->q0 *= norm;
    mpu->q1 *= norm;
    mpu->q2 *= norm;
    mpu->q3 *= norm;

    /* Tính góc Euler - Nhân với RAD_TO_DEG thay vì chia */
    mpu->roll = atan2f(2.0f * (mpu->q0 * mpu->q1 + mpu->q2 * mpu->q3), 
                       1.0f - 2.0f * (mpu->q1 * mpu->q1 + mpu->q2 * mpu->q2)) * RAD_TO_DEG;

    float pitch_sin = 2.0f * (mpu->q0 * mpu->q2 - mpu->q3 * mpu->q1);
    if (pitch_sin > 1.0f) pitch_sin = 1.0f;
    if (pitch_sin < -1.0f) pitch_sin = -1.0f;
    mpu->pitch = asinf(pitch_sin) * RAD_TO_DEG;

    mpu->yaw = atan2f(2.0f * (mpu->q0 * mpu->q3 + mpu->q1 * mpu->q2), 
                      1.0f - 2.0f * (mpu->q2 * mpu->q2 + mpu->q3 * mpu->q3)) * RAD_TO_DEG;

    if (mpu->yaw > 180.0f) mpu->yaw -= 360.0f;
    if (mpu->yaw < -180.0f) mpu->yaw += 360.0f;
}

/* ============================================================
 * ANGLE DIFFERENCE
 * ============================================================ */
float MPU6050_AngleDiff(float target, float current) {
    float diff = target - current;
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}
