#include "mpu6050.h"
#include "usart.h"

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#define RAD2DEG   57.2957795130823f

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

/* Quaternion (tao tu roll/pitch, yaw = 0) */
float qw = 1.0f;
float qx = 0.0f;
float qy = 0.0f;
float qz = 0.0f;

/* Goc Kalman [deg] */
float roll_deg  = 0.0f;
float pitch_deg = 0.0f;

/* MPU6050 status */
volatile uint8_t mpu6050_dataReady = 0;
volatile uint32_t mpu6050_irqCount = 0;
volatile uint32_t mpu6050_missedSamples = 0;

uint8_t mpu6050_whoAmI = 0x00;

/* DWT timing */
static uint32_t mpu6050_lastUpdateCycle = 0;
static uint8_t mpu6050_timingReady = 0;


/* ============================================================
 * Kalman filter (2 state: angle, gyro bias) - 1 bo cho moi truc
 * ============================================================ */

typedef struct
{
    float angle;     /* goc uoc luong [deg]        */
    float bias;      /* bias gyro con du [deg/s]   */
    float P[2][2];   /* hiep phuong sai sai so     */
} Kalman_t;

static Kalman_t kRoll;
static Kalman_t kPitch;
static uint8_t  kalmanInitialized = 0;


static void Kalman_Reset(Kalman_t *k, float angle)
{
    k->angle = angle;
    k->bias  = 0.0f;

    k->P[0][0] = 1.0f;
    k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;
    k->P[1][1] = 1.0f;
}


/*
 * rate     : toc do goc gyro da tru bias hieu chuan [deg/s]
 * meas     : goc do tu accelerometer [deg]
 * useMeas  : 0 = chi predict (accel khong dang tin cay)
 */
static void Kalman_Step(Kalman_t *k, float rate, float meas, float dt, uint8_t useMeas)
{
    /* ---------- PREDICT ---------- */
    k->angle += dt * (rate - k->bias);

    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + KALMAN_Q_ANGLE);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += KALMAN_Q_BIAS * dt;

    /* ---------- UPDATE ---------- */
    if (useMeas)
    {
        float y = meas - k->angle;

        /* xu ly nhay vong +/-180 */
        if (y > 180.0f)  y -= 360.0f;
        if (y < -180.0f) y += 360.0f;

        float S  = k->P[0][0] + KALMAN_R_MEAS;
        float K0 = k->P[0][0] / S;
        float K1 = k->P[1][0] / S;

        k->angle += K0 * y;
        k->bias  += K1 * y;

        float P00 = k->P[0][0];
        float P01 = k->P[0][1];

        k->P[0][0] -= K0 * P00;
        k->P[0][1] -= K0 * P01;
        k->P[1][0] -= K1 * P00;
        k->P[1][1] -= K1 * P01;
    }

    /* Giu goc trong (-180, 180] */
    if (k->angle > 180.0f)   k->angle -= 360.0f;
    if (k->angle <= -180.0f) k->angle += 360.0f;
}


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
 * MPU6050 Initialization (khong doi so voi ban Mahony)
 * ============================================================ */

uint8_t mpu6050_Init(void)
{
    uint8_t check;
    uint8_t mData;

    HAL_Delay(100);

    /* WHO_AM_I */
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 10) != HAL_OK)
    {
        mpu6050_whoAmI = 0x00;
        return 0;
    }

    mpu6050_whoAmI = check;

    if (check != 0x68 && check != 0x74)
        return 0;

    /* Wake up MPU6050 (clock source = PLL gyro X) */
    mData = 0x01;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &mData, 1, 10) != HAL_OK)
        return 0;

    HAL_Delay(10);

    /* Sample Rate Divider: 1 kHz / (1 + 4) = 200 Hz */
    mData = 0x04;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x19, 1, &mData, 1, 10) != HAL_OK)
        return 0;

    /* DLPF */
    mData = 0x03;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &mData, 1, 10) != HAL_OK)
        return 0;

    /* Gyroscope ±250 °/s */
    mData = 0x00;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &mData, 1, 10) != HAL_OK)
        return 0;

    /* Accelerometer ±2g */
    mData = 0x00;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &mData, 1, 10) != HAL_OK)
        return 0;

    /* INT pin: push-pull, active high */
    mData = 0x30;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x37, 1, &mData, 1, 10) != HAL_OK)
        return 0;

    /* Enable Data Ready interrupt */
    mData = 0x01;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x38, 1, &mData, 1, 10) != HAL_OK)
        return 0;

    return 1;
}


/* ============================================================
 * Gyroscope Calibration (giu nguyen, chi reset them Kalman)
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
        if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, data, 6, 10) == HAL_OK)
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

    /* Reset trang thai uoc luong: lan update dau se khoi tao tu accel */
    kalmanInitialized = 0;

    qw = 1.0f;
    qx = 0.0f;
    qy = 0.0f;
    qz = 0.0f;

    roll_deg  = 0.0f;
    pitch_deg = 0.0f;

    mpu6050_dataReady = 0;

    /* Clear MPU6050 interrupt status */
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3A, 1, &dummy, 1, 10);
}


/* ============================================================
 * Read Accelerometer + Gyroscope
 *
 * Return: 1 = OK, 0 = I2C failed
 * ============================================================ */

uint8_t mpu6050_readMotion6(void)
{
    uint8_t buffer[14];

    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buffer, 14, 10) != HAL_OK)
        return 0;

    /* Accelerometer raw */
    ax_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    ay_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
    az_raw = (int16_t)((buffer[4] << 8) | buffer[5]);

    /* Gyroscope raw (bo qua buffer[6..7] = nhiet do) */
    gx_raw = (int16_t)((buffer[8]  << 8) | buffer[9]);
    gy_raw = (int16_t)((buffer[10] << 8) | buffer[11]);
    gz_raw = (int16_t)((buffer[12] << 8) | buffer[13]);

    /* Accelerometer -> g */
    AX = (float)ax_raw / ACCEL_SENS;
    AY = (float)ay_raw / ACCEL_SENS;
    AZ = (float)az_raw / ACCEL_SENS;

    /* Gyroscope -> °/s */
    GX = (float)gx_raw / GYRO_SENS;
    GY = (float)gy_raw / GYRO_SENS;
    GZ = (float)gz_raw / GYRO_SENS;

    return 1;
}


/* ============================================================
 * Kalman attitude update
 *
 * Giu ten mpu6050_updateQuaternion() de main.c khong phai sua.
 *
 * Quy uoc truc giong ban Mahony (ZYX, yaw = 0):
 *   roll  = atan2(ay, az)
 *   pitch = atan2(-ax, sqrt(ay^2 + az^2))
 * Hai cong thuc nay khop voi roll tinh tu quaternion
 *   roll = atan2(2(qw*qx + qy*qz), 1 - 2(qx^2 + qy^2))
 * ============================================================ */

void mpu6050_updateQuaternion(float dt)
{
    float gx = GX - GX_calib;
    float gy = GY - GY_calib;

    /* Gyro deadzone (hien dang = 0) */
    if (fabsf(gx) < GYRO_DEADZONE) gx = 0.0f;
    if (fabsf(gy) < GYRO_DEADZONE) gy = 0.0f;

    /* Goc do tu accelerometer */
    float acc_norm = sqrtf(AX * AX + AY * AY + AZ * AZ);

    if (acc_norm < 0.01f)
    {
        /* Du lieu accel khong hop le: chi predict neu da khoi tao */
        if (kalmanInitialized)
        {
            Kalman_Step(&kRoll,  gx, 0.0f, dt, 0);
            Kalman_Step(&kPitch, gy, 0.0f, dt, 0);
        }
        goto make_quaternion;
    }

    float accRoll  = atan2f(AY, AZ) * RAD2DEG;
    float accPitch = atan2f(-AX, sqrtf(AY * AY + AZ * AZ)) * RAD2DEG;

    if (!kalmanInitialized)
    {
        /* Lan dau: lay goc tu accel lam diem bat dau */
        Kalman_Reset(&kRoll,  accRoll);
        Kalman_Reset(&kPitch, accPitch);
        kalmanInitialized = 1;
    }
    else
    {
        /* Chi tin accel khi khung khong bi gia toc manh */
        uint8_t accOk = (fabsf(acc_norm - 1.0f) < ACC_GATE) ? 1 : 0;

        /*
         * Gyro X -> toc do roll, Gyro Y -> toc do pitch
         * (xap xi goc nho; du cho gimbal +/-30 do)
         */
        Kalman_Step(&kRoll,  gx, accRoll,  dt, accOk);
        Kalman_Step(&kPitch, gy, accPitch, dt, accOk);
    }

make_quaternion:

    roll_deg  = kRoll.angle;
    pitch_deg = kPitch.angle;

    /* Euler (yaw = 0) -> quaternion [qw qx qy qz] */
    float hr = 0.5f * roll_deg  / RAD2DEG;
    float hp = 0.5f * pitch_deg / RAD2DEG;

    float cr = cosf(hr);
    float sr = sinf(hr);
    float cp = cosf(hp);
    float sp = sinf(hp);

    qw =  cr * cp;
    qx =  sr * cp;
    qy =  cr * sp;
    qz = -sr * sp;
}


/* ============================================================
 * UART CSV Debug (giu nguyen dinh dang de script phan tich cu van dung)
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
        const char *header = "Time,GX,GY,GZ,QW,QX,QY,QZ\r\n";

        HAL_UART_Transmit(&huart1, (uint8_t *)header, strlen(header), 100);

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
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, len, 100);
    }
}


/* ============================================================
 * MPU6050 Data Ready EXTI
 * ============================================================ */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU6050_PIN_INT)
    {
        if (mpu6050_dataReady)
            mpu6050_missedSamples++;

        mpu6050_dataReady = 1;
        mpu6050_irqCount++;
    }
}