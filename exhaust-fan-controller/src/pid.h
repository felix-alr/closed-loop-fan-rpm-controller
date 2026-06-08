#ifndef PID_H
#define PID_H

#include <stdbool.h>

// Structure, that holds PID parameters and the current controller state.
typedef struct PIDControllerInfoStruct {

    // Controller parameters
    float Kp;   // Controller gain
    float Ti;   // Reset time (0 -> no I part)
    float Td;   // Derivative time (0 -> no D part)
    float Tf;   // Filter time constant (0 -> no filter)
    bool arw;   // true: anti-windup active, false: anti-windup inactive
    bool i_part_active_prev; // Boolean indicating whether the I part was active in the previous compute step to only recompute the coefficients for the controll algorithm when actually needed.
    bool filter_active_prev; // Boolean indicating whether the filter was active in the previous compute step to only recompute the coefficients for the filter algorithm when actually needed.

    // Coefficients for the control algorithm
	float C[3];

    // Coefficients for filter algorithm
    float Cf[2];

	// Lower and upper bounds
	float m_min;
    float m_max;

	// Sample time
	float Ts;

    // Buffer for control error (filtered and unfiltered): e[0] is the most recent value, e[1] the one from the previous step, ...
	float e[3];
    float e_fil[3];

    // Buffer actuation value: m[0] is the most recent value, m[1] the one from the previous step, ...
	float m[3]; // Controller output

	// Current P, I, and D part
	float P;
	float I;
	float D;

} PIDControllerInfo;

void pid_init(PIDControllerInfo* pid_info);
bool pid_para_set(PIDControllerInfo* pid_info, float Kp, float Ti, float Td, float Tf, float Ts);
bool pid_limits_set(PIDControllerInfo* pid_info, float Mmin, float Mmax);
void pid_arw_set(PIDControllerInfo* pid_info, bool Arw);
void pid_execute(PIDControllerInfo* pid_info, float e, float* m);

void pid_util_update_coeff(PIDControllerInfo* pid_info, bool force_calculation);
bool pid_util_i_part_active(PIDControllerInfo* pid_info);
bool pid_util_filter_active(PIDControllerInfo* pid_info);
float pid_util_max(float a, float b);
float pid_util_min(float a, float b);

#endif