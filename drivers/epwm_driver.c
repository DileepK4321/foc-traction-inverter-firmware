#include "epwm_driver.h"
#include <stdio.h>

#define SYSTEM_CLOCK_HZ 200000000U // 200 MHz system clock (TI C2000 TMS320F28379D baseline)

void EPWM_Init(EPWM_Driver_t *pwm, unsigned int pwm_freq_hz, unsigned int deadtime_ns)
{
    // Center-aligned mode (Up-Down count): Period = SystemClock / (2 * PWM_Freq)
    pwm->period_ticks = SYSTEM_CLOCK_HZ/ (2*pwm_freq_hz);
    pwm->deadtime_ns = deadtime_ns;

    // Initialize Compare Registers to 50% duty cycle (idle state)
    pwm ->cmp_a = pwm -> period_ticks/2;
    pwm ->cmp_b = pwm -> period_ticks/2;
    pwm ->cmp_c = pwm -> period_ticks/2;
    pwm->is_enabled = 1;

    printf("[ePWM Driver] Configured Center-Aligned PWM @ %u Hz | Period: %u ticks | Dead-Time: %u ns\n", 
           pwm_freq_hz, pwm->period_ticks, pwm->deadtime_ns);
}

void EPWM_Update_Duty_Cycles(EPWM_Driver_t *pwm, DutyCycles_t duty) {
    if (!pwm->is_enabled) {
        printf("[ePWM Driver] Warning: Attempted to update duty cycles while PWM is TRIPPED!\n");
        return;
    }

    // Clamp duty cycle inputs between 0.0 and 1.0
    float d_a = duty.duty_a < 0.0f ? 0.0f : (duty.duty_a > 1.0f ? 1.0f : duty.duty_a);
    float d_b = duty.duty_b < 0.0f ? 0.0f : (duty.duty_b > 1.0f ? 1.0f : duty.duty_b);
    float d_c = duty.duty_c < 0.0f ? 0.0f : (duty.duty_c > 1.0f ? 1.0f : duty.duty_c);

    // Translate normalized duty percentage (0.0 - 1.0) into timer compare ticks
    pwm->cmp_a = (unsigned int)(d_a * pwm->period_ticks);
    pwm->cmp_b = (unsigned int)(d_b * pwm->period_ticks);
    pwm->cmp_c = (unsigned int)(d_c * pwm->period_ticks);
}

void EPWM_Trip_Zone_Shutdown(EPWM_Driver_t *pwm) {
    pwm->is_enabled = 0;
    pwm->cmp_a = 0;
    pwm->cmp_b = 0;
    pwm->cmp_c = 0;
    printf("[ePWM Driver] HARDWARE TRIP ZONE ACTIVATED: All PWM Outputs Forced LOW!\n");
}