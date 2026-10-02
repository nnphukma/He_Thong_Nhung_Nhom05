#include "servo_control.h"

static float Clamp(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

void Servo_Init(ServoAxis_t *axis, volatile uint16_t *ccr_reg, float Kp, float Ki, float max_speed) {
    axis->ccr = ccr_reg;
    axis->Kp = Kp;
    axis->Ki = Ki;
    axis->integral = 0.0f;
    axis->integral_max = 200.0f;          // cho phep bu toi da ±200µs cho khau I
    axis->max_speed_us_per_s = max_speed;
    
    axis->current_pulse_us = SERVO_CENTER_US;
    axis->target_pulse_us = SERVO_CENTER_US;
    *(axis->ccr) = (uint16_t)SERVO_CENTER_US; // thiet lap servo ve vi tri can bang
}

void Servo_Update(ServoAxis_t *axis, float current_angle, float setpoint_angle, float dt) {
    // 1. BO dieu khien bu goc (P/PI)
    // Sai so e = Goc mong muon (0 do) - goc thuc te be nghiêng
    float error = setpoint_angle - current_angle;
    
    float p_term = axis->Kp * error;
    
    axis->integral += error * dt;
    axis->integral = Clamp(axis->integral, -axis->integral_max, axis->integral_max);
    float i_term = axis->Ki * axis->integral;
    
    float control_angle = p_term + i_term;
    
    // Quy doi tu goc nghieng sang do rong xung (khoang 1000-2000µs ung voi ±45 do -> 11.11 µs/do)
    float delta_pulse = control_angle * (1000.0f / 90.0f);
    
    axis->target_pulse_us = Clamp(SERVO_CENTER_US + delta_pulse, SERVO_MIN_US, SERVO_MAX_US);
    
    // 2. GIOI HAN TOC DO SERVO (SLEW-RATE LIMITER)
    float max_step = axis->max_speed_us_per_s * dt;
    float pulse_diff = axis->target_pulse_us - axis->current_pulse_us;
    
    if (pulse_diff > max_step) {
        axis->current_pulse_us += max_step;
    } else if (pulse_diff < -max_step) {
        axis->current_pulse_us -= max_step;
    } else {
        axis->current_pulse_us = axis->target_pulse_us;
    }
    
    // 3. XUAT PWM TRUC TIEP RA THANH GHI PHAN CUNG (µs)
    *(axis->ccr) = (uint16_t)axis->current_pulse_us;
}
