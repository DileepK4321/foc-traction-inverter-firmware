#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <math.h>

// Structure to hold 3-phase AC currents
typedef struct {
    float a;
    float b;
    float c;
} PhaseCurrents_t;

// Structure for 2-phase stationary frame (\alpha, \beta)
typedef struct {
    float alpha;
    float beta;
} Clarke_t;

// Structure for 2-phase rotating frame (d, q)
typedef struct {
    float d;
    float q;
} Park_t;

// Structure for SVPWM output duty cycles (0.0 to 1.0)
typedef struct {
    float duty_a;
    float duty_b;
    float duty_c;
} DutyCycles_t;

// Function Prototypes
Clarke_t Clarke_Transform(PhaseCurrents_t phase);
Park_t Park_Transform(Clarke_t clarke, float angle_rad);
Clarke_t Inverse_Park_Transform(Park_t park, float angle_rad);
DutyCycles_t SVPWM_Calculate(Clarke_t v_alpha_beta, float v_dc);

#endif // FOC_MATH_H