#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "stm32f10x.h"

// gioi han an toan nhong co khi servo RC (SG90/MG90S)
#define SERVO_CENTER_US     1500.0f  // xung vi tri 0 do thang bang (µs)
#define SERVO_MIN_US        1000.0f  // Xung gioi han cuc tieu (µs)
#define SERVO_MAX_US        2000.0f  // Xung gioi han cuc dai (µs)

typedef struct {
    volatile uint16_t *ccr;      // con tro tro toi thanh ghi xuat xung (TIM4->CCR1 or CCR2)
    float current_pulse_us;      // do rong xung thuc te dang xuat ra
    float target_pulse_us;       // do rong xung muc tieu bo dieu khien huong toi
    
    // tham so bu goc P/PI
    float Kp;                    // he so ty le
    float Ki;                    // he so tich phan
    float integral;              // tich luy sai so 
    float integral_max;          // gioi han chong tran tich phan (anti-windup)
    
    // gioi han toc do servo (Slew-rate limit)
    float max_speed_us_per_s;    // toc do toi da cho phep (µs/giây)
} ServoAxis_t;

void Servo_Init(ServoAxis_t *axis, volatile uint16_t *ccr_reg, float Kp, float Ki, float max_speed);
void Servo_Update(ServoAxis_t *axis, float current_angle, float setpoint_angle, float dt);

#endif
