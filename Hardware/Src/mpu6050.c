#include "mpu6050.h"
#include "i2c.h"
#include "usart.h"

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#define DEG2RAD   0.0174532925199433f
#define RAD2DEG   57.2957795130823f

/* ============================================================
 * Bien toan cuc
 * ============================================================ */

int16_t ax_raw = 0, ay_raw = 0, az_raw = 0;
int16_t gx_raw = 0, gy_raw = 0, gz_raw = 0;

float AX = 0.0f, AY = 0.0f, AZ = 0.0f;
float GX = 0.0f, GY = 0.0f, GZ = 0.0f;       /* da tru bias */

float GX_calib = 0.0f;
float GY_calib = 0.0f;
float GZ_calib = 0.0f;

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;

float roll_deg  = 0.0f;
float pitch_deg = 0.0f;

volatile uint8_t  mpu6050_dataReady    = 0;
volatile uint32_t mpu6050_irqCount     = 0;
volatile uint32_t mpu6050_missedSamples = 0;
volatile uint32_t mpu6050_i2cErrors    = 0;
uint8_t mpu6050_whoAmI = 0x00;

/* DWT */
static uint32_t mpu6050_lastUpdateCycle = 0;
static uint8_t  mpu6050_timingReady = 0;

/* Mahony: thanh phan tich phan [rad/s] */
static float mahonyInt[3] = {0.0f, 0.0f, 0.0f};

/* So lan loi I2C lien tiep */
static uint8_t i2cFailStreak = 0;


/* ============================================================
 * I2C helper
 * ============================================================ */

static HAL_StatusTypeDef mpu6050_writeReg(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1,
                             MPU6050_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef mpu6050_readRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len,
                            MPU6050_I2C_TIMEOUT_MS);
}

static void i2cDelay(void)
{
    for (volatile uint32_t i = 0; i < 100; i++)
    {
        __NOP();
    }
}

/*
 * Phuc hoi bus I2C khi bi treo (loi hay gap tren STM32F1):
 *   - reset ngoai vi I2C1
 *   - phat toi da 9 xung SCL de slave nha SDA
 *   - tao dieu kien STOP
 *   - khoi tao lai I2C1
 */
static void mpu6050_i2cRecover(void)
{
    GPIO_InitTypeDef g = {0};

    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    g.Pin   = GPIO_PIN_6 | GPIO_PIN_7;           /* SCL, SDA */
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    i2cDelay();

    for (int i = 0; i < 9; i++)
    {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
            break;

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        i2cDelay();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        i2cDelay();
    }

    /* STOP: SDA 0 -> 1 khi SCL = 1 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    i2cDelay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    i2cDelay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    i2cDelay();

    MX_I2C1_Init();
}


/* ============================================================
 * DWT timing
 * ============================================================ */

void mpu6050_TimingInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    mpu6050_lastUpdateCycle = DWT->CYCCNT;
    mpu6050_missedSamples   = 0;
    mpu6050_timingReady     = 1;
}

float mpu6050_GetDeltaTime(void)
{
    uint32_t now, cycles;
    float dt;

    if (!mpu6050_timingReady)
        return MPU6050_DT_NOMINAL;

    now    = DWT->CYCCNT;
    cycles = now - mpu6050_lastUpdateCycle;      /* unsigned: an toan khi tran */
    mpu6050_lastUpdateCycle = now;

    dt = (float)cycles / (float)HAL_RCC_GetHCLKFreq();

    if (dt < MPU6050_DT_MIN || dt > MPU6050_DT_MAX)
        dt = MPU6050_DT_NOMINAL;

    return dt;
}


/* ============================================================
 * Khoi tao cam bien
 * ============================================================ */

uint8_t mpu6050_Init(void)
{
    uint8_t id;

    static const struct { uint8_t reg; uint8_t val; } cfg[] =
    {
        { MPU6050_REG_PWR_MGMT_1,  0x01U },   /* thuc day, clock = PLL gyro X    */
        { MPU6050_REG_SMPLRT_DIV,  (uint8_t)(1000U / MPU6050_SAMPLE_RATE_HZ - 1U) },
        { MPU6050_REG_CONFIG,      0x03U },   /* DLPF ~42 Hz                     */
        { MPU6050_REG_GYRO_CFG,    0x00U },   /* +-250 deg/s                     */
        { MPU6050_REG_ACCEL_CFG,   0x00U },   /* +-2 g                           */
        { MPU6050_REG_INT_PIN_CFG, 0x30U },   /* push-pull, active high, latch,
                                                 xoa khi doc bat ky thanh ghi    */
        { MPU6050_REG_INT_ENABLE,  0x01U },   /* Data Ready interrupt            */
    };

    HAL_Delay(100);

    if (mpu6050_readRegs(MPU6050_REG_WHO_AM_I, &id, 1) != HAL_OK)
    {
        mpu6050_whoAmI = 0x00;
        return 0;
    }

    mpu6050_whoAmI = id;

    if (id != 0x68 && id != 0x74)
        return 0;

    /* Reset mem cam bien: tranh trang thai cu sau khi MCU reset ma cam bien
     * van con nguon (INT dang bi latch o muc cao -> khong co canh len moi). */
    if (mpu6050_writeReg(MPU6050_REG_PWR_MGMT_1, 0x80U) != HAL_OK)
        return 0;

    HAL_Delay(100);

    for (uint32_t i = 0; i < sizeof(cfg) / sizeof(cfg[0]); i++)
    {
        if (mpu6050_writeReg(cfg[i].reg, cfg[i].val) != HAL_OK)
            return 0;

        if (i == 0)
            HAL_Delay(10);
    }

    mpu6050_dataReady = 0;
    return 1;
}


/* ============================================================
 * Quaternion ban dau tu gia toc (yaw = 0)
 *
 *   roll  = atan2(ay, az)
 *   pitch = atan2(-ax, sqrt(ay^2 + az^2))
 * ============================================================ */

static void mpu6050_initQuaternionFromAccel(float ax, float ay, float az)
{
    float n = sqrtf(ax * ax + ay * ay + az * az);

    mahonyInt[0] = mahonyInt[1] = mahonyInt[2] = 0.0f;

    if (n < ACC_TRUST_MIN_G || n > ACC_TRUST_MAX_G)
    {
        qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
        return;
    }

    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);

    qw =  cr * cp;
    qx =  sr * cp;
    qy =  cr * sp;
    qz = -sr * sp;
}


/* ============================================================
 * Hieu chuan gyro
 *
 * - Doc MPU6050_CALIB_SAMPLES mau, tinh bias gyro.
 * - Kiem tra do lech chuan: neu vuot nguong -> cam bien dang di chuyen.
 * - Dat quaternion ban dau theo huong trong luc do duoc.
 *
 * Return: 1 = OK, 0 = that bai (hay giu yen va goi lai)
 * ============================================================ */

static uint8_t calibStdOk(int64_t s, int64_t ss, int64_t n)
{
    /* var = ss/n - (s/n)^2  <=>  n*ss - s^2 <= thr^2 * n^2   (so nguyen) */
    int64_t thr = (int64_t)(MPU6050_CALIB_MAX_STD_DPS * GYRO_SENS);

    return ((ss * n - s * s) <= (thr * thr * n * n)) ? 1 : 0;
}

uint8_t mpu6050_Calibrate(void)
{
    int64_t sx = 0, sy = 0, sz = 0;
    int64_t sxx = 0, syy = 0, szz = 0;
    int32_t sax = 0, say = 0, saz = 0;
    int32_t n = 0;

    uint8_t b[14];
    uint8_t dummy;

    HAL_Delay(100);

    for (int i = 0; i < MPU6050_CALIB_SAMPLES; i++)
    {
        if (mpu6050_readRegs(MPU6050_REG_ACCEL_XOUT, b, 14) == HAL_OK)
        {
            int16_t rax = (int16_t)((b[0]  << 8) | b[1]);
            int16_t ray = (int16_t)((b[2]  << 8) | b[3]);
            int16_t raz = (int16_t)((b[4]  << 8) | b[5]);
            int16_t rgx = (int16_t)((b[8]  << 8) | b[9]);
            int16_t rgy = (int16_t)((b[10] << 8) | b[11]);
            int16_t rgz = (int16_t)((b[12] << 8) | b[13]);

            sax += rax; say += ray; saz += raz;

            sx += rgx; sy += rgy; sz += rgz;
            sxx += (int64_t)rgx * rgx;
            syy += (int64_t)rgy * rgy;
            szz += (int64_t)rgz * rgz;

            n++;
        }

        HAL_Delay(1000U / MPU6050_SAMPLE_RATE_HZ);
    }

    /* Du so mau + dung yen */
    if (n < (MPU6050_CALIB_SAMPLES * 9) / 10)
        return 0;

    if (!calibStdOk(sx, sxx, n) || !calibStdOk(sy, syy, n) || !calibStdOk(sz, szz, n))
        return 0;

    GX_calib = ((float)sx / (float)n) / GYRO_SENS;
    GY_calib = ((float)sy / (float)n) / GYRO_SENS;
    GZ_calib = ((float)sz / (float)n) / GYRO_SENS;

    mpu6050_initQuaternionFromAccel(((float)sax / (float)n) / ACCEL_SENS,
                                    ((float)say / (float)n) / ACCEL_SENS,
                                    ((float)saz / (float)n) / ACCEL_SENS);
    mpu6050_updateEuler();

    mpu6050_dataReady = 0;

    /* Doc INT_STATUS de nha chan INT (bi latch) -> co canh len ke tiep */
    mpu6050_readRegs(MPU6050_REG_INT_STATUS, &dummy, 1);

    return 1;
}


/* ============================================================
 * Doc accel + gyro (14 byte lien tuc)
 *
 * Return: 1 = OK, 0 = loi I2C
 * ============================================================ */

uint8_t mpu6050_readMotion6(void)
{
    uint8_t b[14];

    if (mpu6050_readRegs(MPU6050_REG_ACCEL_XOUT, b, 14) != HAL_OK)
    {
        mpu6050_i2cErrors++;

        if (++i2cFailStreak >= 3U)
        {
            i2cFailStreak = 0;
            mpu6050_i2cRecover();
        }

        return 0;
    }

    i2cFailStreak = 0;

    ax_raw = (int16_t)((b[0]  << 8) | b[1]);
    ay_raw = (int16_t)((b[2]  << 8) | b[3]);
    az_raw = (int16_t)((b[4]  << 8) | b[5]);

    gx_raw = (int16_t)((b[8]  << 8) | b[9]);
    gy_raw = (int16_t)((b[10] << 8) | b[11]);
    gz_raw = (int16_t)((b[12] << 8) | b[13]);

    AX = (float)ax_raw / ACCEL_SENS;
    AY = (float)ay_raw / ACCEL_SENS;
    AZ = (float)az_raw / ACCEL_SENS;

    /* Gyro: [deg/s], da tru bias */
    GX = (float)gx_raw / GYRO_SENS - GX_calib;
    GY = (float)gy_raw / GYRO_SENS - GY_calib;
    GZ = (float)gz_raw / GYRO_SENS - GZ_calib;

    return 1;
}


/* ============================================================
 * Mahony AHRS (chi gyro + accel)
 *
 * Moi thu tinh bang rad/s TRUOC khi cong sai so
 * (ban cu cong sai so khi gyro con o deg/s -> Kp hieu dung nho ~57 lan).
 *
 * MAHONY_KP la he so tren sai so that e = a x v, nen dung 2*Kp*halfe.
 * ============================================================ */

void mpu6050_updateQuaternion(float dt)
{
    float gx = GX * DEG2RAD;
    float gy = GY * DEG2RAD;
    float gz = GZ * DEG2RAD;

    float ax = AX;
    float ay = AY;
    float az = AZ;

    float accNorm = sqrtf(ax * ax + ay * ay + az * az);

    if (accNorm > ACC_TRUST_MIN_G && accNorm < ACC_TRUST_MAX_G)
    {
        float inv = 1.0f / accNorm;
        ax *= inv;
        ay *= inv;
        az *= inv;

        /* Huong trong luc uoc luong (mot nua) */
        float halfvx = qx * qz - qw * qy;
        float halfvy = qw * qx + qy * qz;
        float halfvz = qw * qw - 0.5f + qz * qz;

        /* Sai so (mot nua) = a x v */
        float halfex = ay * halfvz - az * halfvy;
        float halfey = az * halfvx - ax * halfvz;
        float halfez = ax * halfvy - ay * halfvx;

        if (MAHONY_KI > 0.0f)
        {
            const float ki = 2.0f * MAHONY_KI * dt;

            mahonyInt[0] += ki * halfex;
            mahonyInt[1] += ki * halfey;
            mahonyInt[2] += ki * halfez;

            for (int i = 0; i < 3; i++)
            {
                if (mahonyInt[i] >  MAHONY_I_LIMIT) mahonyInt[i] =  MAHONY_I_LIMIT;
                if (mahonyInt[i] < -MAHONY_I_LIMIT) mahonyInt[i] = -MAHONY_I_LIMIT;
            }

            gx += mahonyInt[0];
            gy += mahonyInt[1];
            gz += mahonyInt[2];
        }

        const float kp = 2.0f * MAHONY_KP;

        gx += kp * halfex;
        gy += kp * halfey;
        gz += kp * halfez;
    }

    /* Dao ham quaternion */
    float qDotw = 0.5f * (-qx * gx - qy * gy - qz * gz);
    float qDotx = 0.5f * ( qw * gx + qy * gz - qz * gy);
    float qDoty = 0.5f * ( qw * gy - qx * gz + qz * gx);
    float qDotz = 0.5f * ( qw * gz + qx * gy - qy * gx);

    qw += qDotw * dt;
    qx += qDotx * dt;
    qy += qDoty * dt;
    qz += qDotz * dt;

    float qn = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);

    if (qn > 0.0f)
    {
        float inv = 1.0f / qn;
        qw *= inv;
        qx *= inv;
        qy *= inv;
        qz *= inv;
    }
}


/* ============================================================
 * Quaternion -> roll / pitch [deg]
 *
 * v = huong trong luc trong he toa do cam bien:
 *   vx = 2(qx qz - qw qy)
 *   vy = 2(qw qx + qy qz)
 *   vz = qw^2 - qx^2 - qy^2 + qz^2
 *
 * Roll / pitch khong phu thuoc yaw -> yaw troi khong anh huong bo can bang.
 * ============================================================ */

void mpu6050_updateEuler(void)
{
    float vx = 2.0f * (qx * qz - qw * qy);
    float vy = 2.0f * (qw * qx + qy * qz);
    float vz = qw * qw - qx * qx - qy * qy + qz * qz;

    roll_deg  = atan2f(vy, vz) * RAD2DEG;
    pitch_deg = atan2f(-vx, sqrtf(vy * vy + vz * vz)) * RAD2DEG;
}


/* ============================================================
 * UART CSV (debug quaternion)
 *
 * Time_ms,Roll,Pitch,GX,GY,GZ,QW,QX,QY,QZ
 * Goi o toc do thap (vd 10 Hz): HAL_UART_Transmit la blocking.
 * ============================================================ */

void MPU6050_SendUART(void)
{
    static uint8_t headerSent = 0;

    char buffer[128];
    int len;

    if (!headerSent)
    {
        const char *header = "Time_ms,Roll,Pitch,GX,GY,GZ,QW,QX,QY,QZ\r\n";

        HAL_UART_Transmit(&huart1, (uint8_t *)header, strlen(header), 100);
        headerSent = 1;
    }

    len = snprintf(buffer, sizeof(buffer),
                   "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f\r\n",
                   (unsigned long)HAL_GetTick(),
                   roll_deg, pitch_deg,
                   GX, GY, GZ,
                   qw, qx, qy, qz);

    if (len > 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)len, 100);
    }
}


/* ============================================================
 * EXTI: MPU6050 Data Ready
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
