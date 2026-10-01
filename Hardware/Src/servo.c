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
    /*
     * Limit pulse width.
     */

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

            __HAL_TIM_SET_COMPARE(
                &htim2,
                TIM_CHANNEL_1,
                pulse_us
            );

            break;


        case SERVO_2:

            __HAL_TIM_SET_COMPARE(
                &htim2,
                TIM_CHANNEL_2,
                pulse_us
            );

            break;


        default:

            break;
    }
}


/* ============================================================
 * Servo_SetAngle
 * ============================================================ */

void Servo_SetAngle(Servo_ID servo, float angle)
{
    uint16_t pulse_us;


    /*
     * Limit angle.
     */

    if (angle < SERVO_MIN_ANGLE)
    {
        angle = SERVO_MIN_ANGLE;
    }

    if (angle > SERVO_MAX_ANGLE)
    {
        angle = SERVO_MAX_ANGLE;
    }


    /*
     * Convert:
     *
     *      0°   -> 1000 us
     *      90°  -> 1500 us
     *      180° -> 2000 us
     *
     * pulse = 1000 + angle * 1000 / 180
     */

    pulse_us =
        (uint16_t)(
            SERVO_MIN_US
            +
            (angle / 180.0f)
            * (SERVO_MAX_US - SERVO_MIN_US)
        );


    Servo_SetPulseUs(servo, pulse_us);
}


/* ============================================================
 * Servo_CenterAll
 * ============================================================ */

void Servo_CenterAll(void)
{
    Servo_SetPulseUs(
        SERVO_1,
        SERVO_CENTER_US
    );

    Servo_SetPulseUs(
        SERVO_2,
        SERVO_CENTER_US
    );
}


/* ============================================================
 * Servo_Test
 *
 * Simple sequential test:
 *
 *      Center
 *        ↓
 *      0°
 *        ↓
 *      90°
 *        ↓
 *      180°
 *        ↓
 *      90°
 *
 * Both servos move together.
 * ============================================================ */

void Servo_Test(void)
{
    /*
     * Center
     */

    Servo_SetAngle(SERVO_1, 90.0f);
    Servo_SetAngle(SERVO_2, 90.0f);

    HAL_Delay(1000);


    /*
     * 0°
     */

    Servo_SetAngle(SERVO_1, 0.0f);
    Servo_SetAngle(SERVO_2, 0.0f);

    HAL_Delay(1000);


    /*
     * 90°
     */

    Servo_SetAngle(SERVO_1, 90.0f);
    Servo_SetAngle(SERVO_2, 90.0f);

    HAL_Delay(1000);


    /*
     * 180°
     */

    Servo_SetAngle(SERVO_1, 180.0f);
    Servo_SetAngle(SERVO_2, 180.0f);

    HAL_Delay(1000);


    /*
     * Back to center
     */

    Servo_SetAngle(SERVO_1, 90.0f);
    Servo_SetAngle(SERVO_2, 90.0f);

    HAL_Delay(1000);
}