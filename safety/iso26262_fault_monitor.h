#ifndef ISO26262_FAULT_MONITOR_H
#define ISO26262_FAULT_MONITOR_H

#include "../motor/foc_math.h"
#include "../drivers/epwm_driver.h"

// ISO 26262 Fault Status Bitmask
typedef enum {
    FAULT_NONE             = 0x00,
    FAULT_OVERCURRENT      = 0x01,
    FAULT_OVERVOLTAGE      = 0x02,
    FAULT_UNDERVOLTAGE     = 0x04
} FaultStatus_t;

// Safety Monitor State Structure
typedef struct {
    float max_current_a;         // Max allowed phase current (e.g., 30.0 A)
    float max_v_dc;              // Max allowed DC bus voltage (e.g., 450.0 V)
    float min_v_dc;              // Min allowed DC bus voltage (e.g., 250.0 V)
    
    unsigned int debounce_limit; // Consecutive fault ticks needed to trigger trip
    unsigned int fault_counter;  // Active debounce counter
    
    FaultStatus_t status;        // Confirmed fault bitmask
} ISO26262_SafetyMonitor_t;

void ISO26262_Init(ISO26262_SafetyMonitor_t *mon, float max_i, float max_v, float min_v, unsigned int debounce);
FaultStatus_t ISO26262_Check_Safety_Limits(ISO26262_SafetyMonitor_t *mon, PhaseCurrents_t currents, float v_dc, EPWM_Driver_t *pwm);

#endif // ISO26262_FAULT_MONITOR_H