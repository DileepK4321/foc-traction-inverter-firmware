#include "iso26262_fault_monitor.h"
#include <math.h>
#include <stdio.h>

void ISO26262_Init(ISO26262_SafetyMonitor_t *mon, float max_i, float max_v, float min_v, unsigned int debounce) {
    mon->max_current_a = max_i;
    mon->max_v_dc = max_v;
    mon->min_v_dc = min_v;
    mon->debounce_limit = debounce;
    mon->fault_counter = 0;
    mon->status = FAULT_NONE;

    printf("[ISO 26262 Safety] Level 2 Monitor Init | Max Current: %.1fA | DC Bus: %.1fV - %.1fV | Debounce: %u ticks\n",
           max_i, min_v, max_v, debounce);
}

FaultStatus_t ISO26262_Check_Safety_Limits(ISO26262_SafetyMonitor_t *mon, PhaseCurrents_t currents, float v_dc, EPWM_Driver_t *pwm) {
    FaultStatus_t current_sample_fault = FAULT_NONE;

    // 1. Level 2 Phase Overcurrent Check (|Ia|, |Ib|, |Ic| > Limit)
    if (fabsf(currents.a) > mon->max_current_a ||
        fabsf(currents.b) > mon->max_current_a ||
        fabsf(currents.c) > mon->max_current_a) {
        current_sample_fault |= FAULT_OVERCURRENT;
    }

    // 2. Level 2 DC Bus Overvoltage / Undervoltage Check
    if (v_dc > mon->max_v_dc) {
        current_sample_fault |= FAULT_OVERVOLTAGE;
    } else if (v_dc < mon->min_v_dc) {
        current_sample_fault |= FAULT_UNDERVOLTAGE;
    }

    // 3. Fault Debouncing & Safe-State Execution
    if (current_sample_fault != FAULT_NONE) {
        mon->fault_counter++;
        if (mon->fault_counter >= mon->debounce_limit) {
            mon->status = current_sample_fault;
            // Level 3 Hardware Shutdown: Force all PWM outputs LOW via Trip Zone
            EPWM_Trip_Zone_Shutdown(pwm);
            printf("[ISO 26262 Safety] CRITICAL FAULT DEBOUNCED (Code: 0x%02X). Emergency Safe-State Engaged!\n", mon->status);
        }
    } else {
        // Reset counter if transient noise clears prior to reaching debounce limit
        if (mon->fault_counter > 0) {
            mon->fault_counter--;
        }
    }

    return mon->status;
}