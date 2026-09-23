#include <stdio.h>
#include "motor/foc_math.h"
#include "drivers/epwm_driver.h"
#include "control/pi_controller.h"
#include "isr/adc_isr.h"

int main() {
    printf("=========================================================\n");
    printf("   CLOSED-LOOP FOC TRACTION INVERTER FIRMWARE SIMULATION  \n");
    printf("=========================================================\n\n");

    // 1. Initialize ePWM Hardware Driver (20 kHz PWM Frequency, 500 ns Dead-time)
    EPWM_Driver_t pwm_hardware;
    EPWM_Init(&pwm_hardware, 20000, 500);

    // 2. Initialize System Context (DC Bus = 400V)
    FOC_Controller_t foc_system;
    FOC_System_Init(&foc_system, &pwm_hardware, 400.0f);

    // Set Torque Demand (I_q target = 15.0 Amps)
    foc_system.target_id = 0.0f;   // 0A for maximum efficiency
    foc_system.target_iq = 15.0f;  // Torque request from throttle

    printf("\n--- Starting Closed-Loop Execution (Simulating 20kHz ADC ISR Ticks) ---\n\n");

    // 3. Simulate 5 ISR Execution Cycles
    for (int step = 1; step <= 5; step++) {
        // Simulate changing motor feedback (Currents ramping up towards target)
        foc_system.measured_currents.a = 5.0f + (step * 1.8f);
        foc_system.measured_currents.b = -2.5f - (step * 0.9f);
        foc_system.measured_currents.c = -2.5f - (step * 0.9f);
        foc_system.rotor_angle_rad = 0.10f * step; // Rotor position advancing

        printf("[ISR Tick #%d]\n", step);
        
        // Execute the High-Priority Interrupt Service Routine
        ADC_ePWM_ISR_Handler(&foc_system);

        // Print Hardware Register State
        printf("  -> PWM Compare Registers: CMPA=%u | CMPB=%u | CMPC=%u\n\n",
               pwm_hardware.cmp_a, pwm_hardware.cmp_b, pwm_hardware.cmp_c);
    }

    // 4. Test Overcurrent Hardware Trip Zone
    printf("--- Testing Safety State Machine ---\n");
    EPWM_Trip_Zone_Shutdown(&pwm_hardware);

    return 0;
}