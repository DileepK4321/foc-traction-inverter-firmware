#include "foc_math.h"

#define ONE_BY_SQRT3 0.57735026919f

// 1. Forward Clarke Transform: (I_a, I_b, I_c) -> (I_alpha, I_beta)
Clarke_t Clarke_Transform(PhaseCurrents_t phase) {
    Clarke_t out;
    out.alpha = phase.a;
    out.beta  = (phase.a + 2.0f * phase.b) * ONE_BY_SQRT3;
    return out;
}

// 2. Forward Park Transform: (I_alpha, I_beta) -> (I_d, I_q)
Park_t Park_Transform(Clarke_t clarke, float angle_rad) {
    Park_t out;
    float cos_th = cosf(angle_rad);
    float sin_th = sinf(angle_rad);

    out.d =  clarke.alpha * cos_th + clarke.beta * sin_th;
    out.q = -clarke.alpha * sin_th + clarke.beta * cos_th;
    return out;
}

// 3. Inverse Park Transform: (V_d, V_q) -> (V_alpha, V_beta)
Clarke_t Inverse_Park_Transform(Park_t park, float angle_rad) {
    Clarke_t out;
    float cos_th = cosf(angle_rad);
    float sin_th = sinf(angle_rad);

    out.alpha = park.d * cos_th - park.q * sin_th;
    out.beta  = park.d * sin_th + park.q * cos_th;
    return out;
}

// 4. Space Vector PWM Duty Cycle Generator
DutyCycles_t SVPWM_Calculate(Clarke_t v_alpha_beta, float v_dc) {
    DutyCycles_t duty;
    
    // Calculate normalized phase voltages
    float v_a = v_alpha_beta.alpha;
    float v_b = -0.5f * v_alpha_beta.alpha + 0.8660254f * v_alpha_beta.beta;
    float v_c = -0.5f * v_alpha_beta.alpha - 0.8660254f * v_alpha_beta.beta;

    // Find min and max for neutral point shift (SVPWM center alignment)
    float v_max = v_a > v_b ? (v_a > v_c ? v_a : v_c) : (v_b > v_c ? v_b : v_c);
    float v_min = v_a < v_b ? (v_a < v_c ? v_a : v_c) : (v_b < v_c ? v_b : v_c);

    // Common mode voltage injection
    float v_com = 0.5f * (v_max + v_min);

    // Compute duty cycles normalized to [0.0, 1.0]
    duty.duty_a = (v_a - v_com) / v_dc + 0.5f;
    duty.duty_b = (v_b - v_com) / v_dc + 0.5f;
    duty.duty_c = (v_c - v_com) / v_dc + 0.5f;

    return duty;
}