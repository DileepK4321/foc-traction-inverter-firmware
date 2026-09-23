#include "adc_isr.h"
#include <stdio.h>

void FOC_System_Init(FOC_Controller_t *foc, EPWM_Driver_t *pwm_driver, float v_dc) {
    foc->pwm_driver = pwm_driver;
    foc->v_dc_bus = v_dc;
    foc->target_id = 0.0f; // 0A for maximum efficiency
    foc->target_iq = 10.0f; // 10A torque command

    // Initialize PI Controllers for current loops (Kp=1.2, Ki=0.05, Max Volts = V_DC / sqrt(3))
    float max_voltage = v_dc * 0.57735f;
    PI_Init(&foc->pi_d, 1.2f, 0.05f, max_voltage);
    PI_Init(&foc->pi_q, 1.2f, 0.05f, max_voltage);

    // Initialize Level 2 Safety Monitor (Max 30A, 250V-450V, 3-tick debounce)
    ISO26262_Init(&foc->safety_monitor, 30.0f, 450.0f, 250.0f, 3);
}

// Simulated High-Priority Hardware Interrupt Handler
void ADC_ePWM_ISR_Handler(FOC_Controller_t *foc) {
    // 1. Read phase currents & angle (Simulated from hardware registers)
    PhaseCurrents_t i_abc = foc->measured_currents;

    FaultStatus_t fault = ISO26262_Check_Safety_Limits(&foc->safety_monitor, i_abc, foc->v_dc_bus, foc->pwm_driver);
    if (fault != FAULT_NONE) {
        // Abort control pipeline execution if system is in fault state
        return;
    }
    float angle = foc->rotor_angle_rad;

    // 2. Forward Clarke & Park Transforms
    Clarke_t i_clarke = Clarke_Transform(i_abc);
    Park_t i_park = Park_Transform(i_clarke, angle);

    // 3. PI Current Regulators -> Calculate required V_d and V_q
    Park_t v_park;
    v_park.d = PI_Execute(&foc->pi_d, foc->target_id, i_park.d);
    v_park.q = PI_Execute(&foc->pi_q, foc->target_iq, i_park.q);

    // 4. Inverse Park Transform -> Calculate V_alpha and V_beta
    Clarke_t v_alpha_beta = Inverse_Park_Transform(v_park, angle);

    // 5. Space Vector Modulation -> Calculate PWM Duty Cycles
    DutyCycles_t duty = SVPWM_Calculate(v_alpha_beta, foc->v_dc_bus);

    // 6. Write Duty Cycles directly to ePWM Registers
    EPWM_Update_Duty_Cycles(foc->pwm_driver, duty);
}