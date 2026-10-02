#include "stm32f10x.h"
#include "servo_control.h"

ServoAxis_t servo_roll;
ServoAxis_t servo_pitch;

// Cau hình TIM4 phát xung PWM PB6 (Roll), PB7 (Pitch)
void TIM4_PWM_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // Cau hình PB6, PB7 là Alternate Function Output Push-Pull 50MHz (1011b = 0x0B)
    GPIOB->CRL &= ~((0x0F << 24) | (0x0F << 28));
    GPIOB->CRL |=  ((0x0B << 24) | (0x0B << 28));

    // Cài dat tan so 50Hz (chu ky 20ms)
    TIM4->PSC = 71;    // 72MHz / 72 = 1MHz (1 tick = 1µs)
    TIM4->ARR = 19999; // 20000 tick = 20000µs = 20ms (50Hz)

    // Cau hình PWM Mode 1 cho kenh 1 và kenh 2
    TIM4->CCMR1 |= (0x06 << 4) | TIM_CCMR1_OC1PE;
    TIM4->CCMR1 |= (0x06 << 12) | TIM_CCMR1_OC2PE;

    // Bat ngo ra kenh 1 (PB6) và kenh 2 (PB7)
    TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

    // Khoi dong TIM4
    TIM4->CR1 |= TIM_CR1_CEN;
}

// Cau hinh TIM2 ngat dinh ky dung 5ms (200Hz) de chay thuat toan dieu khien
void TIM2_Loop_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 71;    // 1 tick = 1µs
    TIM2->ARR = 4999;  // 5000 tick = 5ms (200Hz)
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->SR &= ~TIM_SR_UIF;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;
}

float sim_angle = 30.0f; // Ban d?u b? b? l?ch 30 d?

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;

        // Mô ph?ng ph?n h?i v?t lý: Servo quay kéo b? v? 0
        // Góc bù c?a servo càng l?n thì sim_angle càng b? tri?t tiêu
        float servo_offset_deg = (servo_roll.current_pulse_us - 1500.0f) * (90.0f / 1000.0f);
        float current_platform_angle = sim_angle + servo_offset_deg;

        // B? di?u khi?n d?c góc c?a m?t sàn b? (dã du?c kéo bù)
        Servo_Update(&servo_roll,  current_platform_angle, 0.0f, 0.005f);
        Servo_Update(&servo_pitch, current_platform_angle, 0.0f, 0.005f);
    }
}

int main(void)
{
    TIM4_PWM_Init();
    
    // Khoi tao: Kp = 1.0, Ki = 0.05, Toc do toi da = 1500 µs/s (chong giat nhông)
    Servo_Init(&servo_roll,  &(TIM4->CCR1), 1.0f, 0.05f, 1500.0f);
    Servo_Init(&servo_pitch, &(TIM4->CCR2), 1.0f, 0.05f, 1500.0f);
    
    TIM2_Loop_Init();

    while (1)
    {
        // CPU ranh, toan bo xu ly trong ngat thoi gian thuc 200Hz
    }
}