#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include <stdint.h>

/* ============================================================
 * Servo configuration
 * ============================================================ */

#define SERVO_MIN_US       1000U
#define SERVO_MAX_US       2000U

#define SERVO_MIN_ANGLE    0.0f
#define SERVO_MAX_ANGLE    180.0f

#define SERVO_1_CENTER_ANGLE 100.0f
#define SERVO_2_CENTER_ANGLE  90.0f
/* ============================================================
 * Servo ID
 * ============================================================ */

typedef enum
{
    SERVO_1 = 0,
    SERVO_2 = 1

} Servo_ID;

extern float angle_sv1;
extern float angle_sv2;

/* ============================================================
 * Initialization
 * ============================================================ */

void Servo_Init(void);


/* ============================================================
 * Low-level control
 *
 * Set pulse width directly in microseconds.
 *
 * Example:
 *      Servo_SetPulseUs(SERVO_1, 1500);
 * ============================================================ */

void Servo_SetPulseUs(Servo_ID servo, uint16_t pulse_us);


/* ============================================================
 * Angle control
 *
 * 0°   -> 1000 us
 * 90°  -> 1500 us
 * 180° -> 2000 us
 * ============================================================ */

void Servo_SetAngle(Servo_ID servo, float angle);


/* ============================================================
 * Center both servos
 * ============================================================ */

void Servo_CenterAll(void);

#endif