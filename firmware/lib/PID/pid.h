#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


// Structure, that holds PID parameters and the current controller state.
typedef struct PIDControllerInfoStruct {

    // Controller parameters
    float Kp;   // Controller gain
    float Ti;   // Reset time (0 -> no I part)
    bool i_part_active_prev; // Boolean indicating whether the I part was active in the previous compute step to only recompute the coefficients for the controll algorithm when actually needed.

    // Coefficients for the control algorithm
	float C[2];

	// Lower and upper bounds
	uint8_t m_min;
    uint8_t m_max;

	// Sample time
	float Ts;

    // Buffer for control error (filtered and unfiltered): e[0] is the most recent value, e[1] the one from the previous step, ...
	uint16_t e[2];

    // Buffer actuation value: m[0] is the most recent value, m[1] the one from the previous step, ...
	float m[2]; // Controller output

} PIDControllerInfo;

void pid_init(PIDControllerInfo* pid_info);
bool pid_para_set(PIDControllerInfo* pid_info, float Kp, float Ti, float Ts);
bool pid_limits_set(PIDControllerInfo* pid_info, uint8_t Mmin, uint8_t Mmax);
void pid_execute(PIDControllerInfo* pid_info, uint16_t e, uint8_t* m);

void pid_util_update_coeff(PIDControllerInfo* pid_info, bool force_calculation);
bool pid_util_i_part_active(PIDControllerInfo* pid_info);
float pid_util_max(float a, float b);
float pid_util_min(float a, float b);

#ifdef __cplusplus
}
#endif

#endif