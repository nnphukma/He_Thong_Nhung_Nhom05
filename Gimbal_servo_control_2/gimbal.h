#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "main.h"
#include <stdint.h>

/* ============================================================
 * Gimbal configuration
 *
 * Axis mapping (matches main.c):
 *      SERVO_1 -> Roll  (rotation about MPU6050 X axis)
 *      SERVO_2 -> Pitch (rotation about MPU6050 Y axis)
 * ============================================================ */

/*
 * Sensor mounting:
 *   1 = MPU6050 is fixed on the PLATFORM (closed loop, PI controller).
 *       The controller drives the measured platform tilt to zero.
 *   0 = MPU6050 is fixed on the BASE (open loop / direct compensation).
 *       Servo angle = center - measured base tilt.
 */
#define GIMBAL_SENSOR_ON_PLATFORM   0

/*
 * Direction of each servo.
 *   +1 : increasing servo angle tilts the platform toward POSITIVE roll/pitch
 *   -1 : increasing servo angle tilts the platform toward NEGATIVE roll/pitch
 * If the platform runs away to its limit when tilted, flip the sign.
 */
#define GIMBAL_ROLL_DIR             (+1.0f)
#define GIMBAL_PITCH_DIR            (+1.0f)

/*
 * Servo gain (direct-compensation mode only, GIMBAL_SENSOR_ON_PLATFORM = 0).
 *
 * Servo_SetAngle() assumes 180 deg for 1000..2000 us, but a typical MG90S
 * turns only ~120 deg over that range (and the linkage can add its own
 * ratio), so 1 commanded degree moves the platform by less than 1 degree.
 * A gain > 1 compensates. 1.0 = no scaling; 1.5 is a starting guess for a
 * 120-deg servo.
 *
 * Calibrate: tilt the base by T deg (read gimbal_pitch_deg / gimbal_roll_deg),
 * measure the platform's remaining tilt E deg in the SAME direction as T,
 * then:   new_gain = old_gain * T / (T - E)
 */
#define GIMBAL_ROLL_GAIN            1.5f
#define GIMBAL_PITCH_GAIN           1.5f

/* PI(D) gains (closed-loop mode only) */
#define GIMBAL_KP                   0.30f   /* [-]        proportional on tilt error   */
#define GIMBAL_KI                   6.00f   /* [1/s]      integral on tilt error       */
#define GIMBAL_KD                   0.00f   /* [s]        damping on gyro rate         */

/* Tilt error smaller than this is ignored (reduces servo jitter) [deg] */
#define GIMBAL_DEADBAND_DEG         0.20f

/* Maximum servo correction around its center [deg] */
#define GIMBAL_MAX_CORRECTION_DEG   45.0f

/* Number of accelerometer samples (5 ms apart) used to align the
 * attitude estimate with gravity at start-up */
#define GIMBAL_ALIGN_SAMPLES        20

/* ============================================================
 * Debug / monitoring (Live Watch, UART)
 * ============================================================ */

extern float gimbal_roll_deg;      /* measured roll  [deg] */
extern float gimbal_pitch_deg;     /* measured pitch [deg] */

/* ============================================================
 * API
 * ============================================================ */

/*
 * Align the quaternion with gravity, reset the controller and center
 * both servos. Call after mpu6050_Init/Calibrate/TimingInit and Servo_Init.
 */
void Gimbal_Init(void);

/*
 * Call once per new IMU sample, right after mpu6050_updateQuaternion().
 * dt = sample period in seconds.
 */
void Gimbal_Update(float dt);

/*
 * Desired platform tilt in degrees (default 0, 0 = parallel to the ground).
 */
void Gimbal_SetTarget(float roll_deg, float pitch_deg);

#endif /* __GIMBAL_H */