#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

typedef struct {
    float kp;          // Proportional gain
    float ki;          // Integral gain
    float integral;    // Accumulated integral state
    float out_max;     // Upper output limit (Volts)
    float out_min;     // Lower output limit (Volts)
} PI_Controller_t;

void PI_Init(PI_Controller_t *pi, float kp, float ki, float max_val);
float PI_Execute(PI_Controller_t *pi, float setpoint, float feedback);

#endif // PI_CONTROLLER_H