#include "pi_controller.h"

void PI_Init(PI_Controller_t *pi, float kp, float ki, float max_val) {
    pi->kp = kp;
    pi->ki = ki;
    pi->integral = 0.0f;
    pi->out_max = max_val;
    pi->out_min = -max_val;
}

float PI_Execute(PI_Controller_t *pi, float setpoint, float feedback) {
    float error = setpoint - feedback;
    
    // Proportional term
    float p_out = pi->kp * error;
    
    // Integral term update
    pi->integral += pi->ki * error;
    
    // Anti-windup clamping on integral term
    if (pi->integral > pi->out_max) pi->integral = pi->out_max;
    if (pi->integral < pi->out_min) pi->integral = pi->out_min;
    
    float output = p_out + pi->integral;
    
    // Output voltage clamping
    if (output > pi->out_max) output = pi->out_max;
    if (output < pi->out_min) output = pi->out_min;
    
    return output;
}