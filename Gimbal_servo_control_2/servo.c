#include "servo.h"
#include "stm32f1xx_hal.h"
/* ============================================================
 * External timer handle
 *
 * TIM2:
 *      CH1 -> Servo 1 -> PA0
 *      CH2 -> Servo 2 -> PA1
 * ============================================================ */

extern TIM_HandleTypeDef htim2;

float angle_sv1 = 90.0f;
float angle_sv2 = 90.0f;

/* ============================================================
 * Servo_Init
 * ============================================================ */

void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    /*
     * Start both servos at center position.
     */

    Servo_CenterAll();
}


/* ============================================================
 * Servo_SetPulseUs
 * ============================================================ */

void Servo_SetPulseUs(Servo_ID servo, uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_US)
    {
        pulse_us = SERVO_MIN_US;
    }

    if (pulse_us > SERVO_MAX_US)
    {
        pulse_us = SERVO_MAX_US;
    }

    switch (servo)
    {
        case SERVO_1:
            __HAL_TIM_SET_COMPARE( &htim2, TIM_CHANNEL_1, pulse_us ); break;

        case SERVO_2:
            __HAL_TIM_SET_COMPARE( &htim2, TIM_CHANNEL_2, pulse_us ); break;

        default: break;
    }
}


/* ============================================================
 * Servo_SetAngle
 * ============================================================ */

void Servo_SetAngle(Servo_ID servo, float angle)
{
    uint16_t pulse_us;

    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    pulse_us = (uint16_t)(
        SERVO_MIN_US + (angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US)
    );

    Servo_SetPulseUs(servo, pulse_us);

    switch (servo)
    {
        case SERVO_1: angle_sv1 = angle; break;
        case SERVO_2: angle_sv2 = angle; break;
        default: break;
    }
}


/* ============================================================
 * Servo_CenterAll
 * ============================================================ */

void Servo_CenterAll(void)
{
    Servo_SetAngle(SERVO_1, SERVO_1_CENTER_ANGLE);
    Servo_SetAngle(SERVO_2, SERVO_2_CENTER_ANGLE);
}

