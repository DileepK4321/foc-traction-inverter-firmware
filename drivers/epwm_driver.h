#ifndef EPWM_DRIVER_H
#define EPWM_DRIVER_H

#include "../motor/foc_math.h"

// Hardware ePWM configuration structure (simulating register layer)
typedef struct {
    unsigned int period_ticks;  // Timer Period register (TBPRD)
    unsigned int cmp_a;         // Compare A register (Phase A duty)
    unsigned int cmp_b;         // Compare B register (Phase B duty)
    unsigned int cmp_c;         // Compare C register (Phase C duty)
    unsigned int deadtime_ns;   // Dead-time insertion in nanoseconds
    unsigned char is_enabled;   // PWM trip status (1 = Active, 0 = Tripped)
} EPWM_Driver_t;



void EPWM_Init(EPWM_Driver_t *pwm, unsigned int pwm_freq_hz, unsigned int deadtime_ns);
void EPWM_Update_Duty_Cycles(EPWM_Driver_t *pwm, DutyCycles_t duty);
void EPWM_Trip_Zone_Shutdown(EPWM_Driver_t *pwm);

#endif  // EPWM_DRIVER_H