#include "gimbal.h"
#include "mpu6050.h"
#include "servo.h"

#include <math.h>

#define RAD2DEG   57.29577951308232f
#define DEG2RAD   0.017453292519943295f

/* ============================================================
 * State
 * ============================================================ */

float gimbal_roll_deg  = 0.0f;
float gimbal_pitch_deg = 0.0f;

static float target_roll_deg  = 0.0f;
static float target_pitch_deg = 0.0f;

static float integ_roll  = 0.0f;
static float integ_pitch = 0.0f;


/* ============================================================
 * Helpers
 * ============================================================ */

static float Gimbal_Clamp(float v, float lim)
{
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

#if GIMBAL_SENSOR_ON_PLATFORM
static float Gimbal_Deadband(float e, float db)
{
    if (e >  db) return e - db;
    if (e < -db) return e + db;
    return 0.0f;
}
#endif


/* ============================================================
 * Quaternion -> roll / pitch [deg]
 *
 * Convention matches the Mahony filter in mpu6050.c:
 * with the sensor flat, accelerometer = (0, 0, +1 g).
 *      roll  : rotation about X
 *      pitch : rotation about Y
 * Yaw is not needed and has no effect on roll/pitch.
 * ============================================================ */

static void Gimbal_GetEuler(float *roll, float *pitch)
{
    float sinp = 2.0f * (qw * qy - qx * qz);

    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;

    *pitch = asinf(sinp) * RAD2DEG;

    *roll = atan2f(2.0f * (qw * qx + qy * qz),
                   qw * qw - qx * qx - qy * qy + qz * qz) * RAD2DEG;
}


/* ============================================================
 * Align quaternion with gravity
 *
 * mpu6050_Calibrate() resets the quaternion to identity (= level).
 * If the base is tilted at power-up, the estimate would start wrong.
 * Initialize it from the accelerometer instead (yaw = 0).
 * ============================================================ */

static void Gimbal_AlignToGravity(void)
{
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    int n = 0;

    for (int i = 0; i < GIMBAL_ALIGN_SAMPLES; i++)
    {
        if (mpu6050_readMotion6())
        {
            sx += AX;
            sy += AY;
            sz += AZ;
            n++;
        }

        HAL_Delay(5);
    }

    if (n == 0)
        return;

    sx /= (float)n;
    sy /= (float)n;
    sz /= (float)n;

    /* Reject if the vector is not close to 1 g (device being shaken) */
    float norm = sqrtf(sx * sx + sy * sy + sz * sz);

    if (norm < 0.8f || norm > 1.2f)
        return;

    float roll  = atan2f(sy, sz);
    float pitch = atan2f(-sx, sqrtf(sy * sy + sz * sz));

    float cr = cosf(0.5f * roll);
    float sr = sinf(0.5f * roll);
    float cp = cosf(0.5f * pitch);
    float sp = sinf(0.5f * pitch);

    qw =  cr * cp;
    qx =  sr * cp;
    qy =  cr * sp;
    qz = -sr * sp;
}


/* ============================================================
 * One axis controller
 *
 * Returns the desired platform correction "u" in degrees,
 * relative to the servo center (before direction sign).
 *
 * err  : measured tilt minus target [deg]
 * rate : measured angular rate      [deg/s]
 * gain : servo gain, used in direct-compensation mode only
 * ============================================================ */

static float Gimbal_Axis(float *integ, float err, float rate, float dt, float gain)
{
#if GIMBAL_SENSOR_ON_PLATFORM

    (void)gain;   /* the integrator absorbs any servo gain error */

    float e = Gimbal_Deadband(err, GIMBAL_DEADBAND_DEG);

    /* Integral with clamp (anti-windup) */
    *integ += GIMBAL_KI * e * dt;
    *integ  = Gimbal_Clamp(*integ, GIMBAL_MAX_CORRECTION_DEG);

    float u = -(GIMBAL_KP * e + *integ + GIMBAL_KD * rate);

    return Gimbal_Clamp(u, GIMBAL_MAX_CORRECTION_DEG);

#else

    /* Sensor on the base: cancel the measured base tilt directly */
    (void)integ;
    (void)rate;
    (void)dt;

    return Gimbal_Clamp(-gain * err, GIMBAL_MAX_CORRECTION_DEG);

#endif
}


/* ============================================================
 * Public API
 * ============================================================ */

void Gimbal_Init(void)
{
    integ_roll  = 0.0f;
    integ_pitch = 0.0f;

    target_roll_deg  = 0.0f;
    target_pitch_deg = 0.0f;

    Servo_CenterAll();

    Gimbal_AlignToGravity();

    Gimbal_GetEuler(&gimbal_roll_deg, &gimbal_pitch_deg);

    /*
     * Drop samples that arrived during alignment AND release the MPU6050
     * latched INT line.
     *
     * INT is latched (INT_PIN_CFG = 0x30) and the EXTI is edge-triggered.
     * If the line is left high there is no new rising edge, so
     * mpu6050_dataReady is never set again and the main loop starves.
     * Order matters: clear the flag first, then read (a read clears the latch).
     */
    mpu6050_dataReady = 0;
    (void)mpu6050_readMotion6();
}


void Gimbal_SetTarget(float roll_deg, float pitch_deg)
{
    target_roll_deg  = roll_deg;
    target_pitch_deg = pitch_deg;
}


void Gimbal_Update(float dt)
{
    float roll, pitch;

    Gimbal_GetEuler(&roll, &pitch);

    gimbal_roll_deg  = roll;
    gimbal_pitch_deg = pitch;

    float u_roll  = Gimbal_Axis(&integ_roll,
                                roll - target_roll_deg,
                                GX - GX_calib,
                                dt,
                                GIMBAL_ROLL_GAIN);

    float u_pitch = Gimbal_Axis(&integ_pitch,
                                pitch - target_pitch_deg,
                                GY - GY_calib,
                                dt,
                                GIMBAL_PITCH_GAIN);

    Servo_SetAngle(SERVO_1, SERVO_1_CENTER_ANGLE + GIMBAL_ROLL_DIR  * u_roll);
    Servo_SetAngle(SERVO_2, SERVO_2_CENTER_ANGLE + GIMBAL_PITCH_DIR * u_pitch);
}