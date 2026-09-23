#ifndef ADC_ISR_H
#define ADC_ISR_H

#include "../motor/foc_math.h"
#include "../drivers/epwm_driver.h"
#include "../control/pi_controller.h"
#include "../safety/iso26262_fault_monitor.h"

// Full FOC System Context Structure
typedef struct {
    PhaseCurrents_t measured_currents;
    float rotor_angle_rad;
    float target_id; // Flux target (usually 0.0A for non-salient PMSM)
    float target_iq; // Torque target (from throttle input)
    float v_dc_bus;
    
    PI_Controller_t pi_d;
    PI_Controller_t pi_q;
    EPWM_Driver_t *pwm_driver;

    ISO26262_SafetyMonitor_t safety_monitor;
} FOC_Controller_t;

void FOC_System_Init(FOC_Controller_t *foc, EPWM_Driver_t *pwm_driver, float v_dc);
void ADC_ePWM_ISR_Handler(FOC_Controller_t *foc);

#endif // ADC_ISR_H