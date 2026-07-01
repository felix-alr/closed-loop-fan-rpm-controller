#include "pid.h"

#include <limits.h>
#include<float.h>

/** Initializes a PID controller
 *
 *  @param pid_info points at die PIDControllerInfo structure
 *
 */
void pid_init(PIDControllerInfo* pid_info)
{
    int i;

    pid_info->Kp = 0.0f;
    pid_info->Ti = 0.0f;
    pid_info->Ts = 1.0f;
    pid_info->m_min = 0;
    pid_info->m_max = 1;

    pid_info->i_part_active_prev = true;
    
    for (i = 0; i < 2; i++) {
        pid_info->C[i] = 0.0f;
        pid_info->e[i] = 0.0f;
        pid_info->m[i] = 0.0f;
    }
}


/** Sets the parameters of the pid controller
 * 
 *  This function sets the controller parameters of the pid and calculates the coefficient for further computation.
 *  Furthermore, the function returns false if an invalid input is detected:
 *  - the sample time is invalid 
 *  - the filter time constant is invalid with respect to sample time (Ts < Tf, if Tf != 0.0f)
 *  - coefficient computation for PD controller-Berechnung auch für einen PD-Regler zulassen
 *  - the other time parameters are invalid (
 *
 *  If all inputs are valid:
 * 
 *  - Kp > 0
 *  - Ti >= 0 (Ti = 0: integral part of the controller is inactive)
 *  - Td >= 0 (Td = 0: differential part of the controller is inactive)
 *  - Tf >= 0 (Tf = 0: the filter is inactive)
 *  - Ts > 0
 *  - Tf > Ts if Tf = 0 (when the filter is active)
 *
 *  @param pid_info Pointer to the PIDControllerInfo structure
 *  @param Kp Controller gain
 *  @param Ti Reset time in seconds
 *  @param Ts Sample time in seconds
 *
 *  @returns true, if parameters were set, false otherwise.
 */
bool pid_para_set(PIDControllerInfo* pid_info, float Kp, float Ti, float Ts)
{
    if (Kp <= 0.0f || Ti < 0.0f || Ts <= 0.0f)
    {
        return false;
    }

    // Set parameters
    pid_info->Kp = Kp;
    pid_info->Ti = Ti;
    pid_info->Ts = Ts;

    pid_util_update_coeff(pid_info, true);

    return true;
}


/** Sets the upper and lower bound of the actuating value (output of controller)
 *
 *  @param pid_info Pointer to the PIDControllerInfo structure
 *  @param m_min Lower bound
 *  @param m_max Upper bound
 */
bool pid_limits_set(PIDControllerInfo* pid_info, uint8_t m_min, uint8_t m_max)
{
    if (m_min >= m_max) return false;

    pid_info->m_min = m_min;
    pid_info->m_max = m_max;

    return true;
}

/** Performs a compute step of the PID controller.
 *
 *  @param pid_info Pointer to the PIDControllerInfo structure
 *  @param e The current control error
 *  @param m Pointer to a float variable, which the computed actuation value will be written to
 */
void pid_execute(PIDControllerInfo* pid_info, uint16_t e, uint8_t* m)
{
    // Shift arrays m and e by one to free the first index
    pid_info->m[1] = pid_info->m[0];
    pid_info->e[1] = pid_info->e[0];

    // Set current values for e
    pid_info->e[0] = e;

    // Update coefficients for filter and PID controller
    pid_util_update_coeff(pid_info, false);

    // Use filtered or measured e-values depending on filter usage

    pid_info->m[0] = pid_info->m[1] + pid_info->C[0] * pid_info->e[0] + pid_info->C[1] * pid_info->e[1];

    // Limit output to control circuit using given bounds
    *m = pid_util_max(pid_util_min(pid_info->m[0], pid_info->m_max), pid_info->m_min);
}

/** Updates the control and filter algorithm's coefficients when needed.
* 
* @param pid_info Pointer to the PIDControllerInfo structure
* @param ForceCalculation true: force recomputing coefficients, false: recompute only if something has changed
*/
void pid_util_update_coeff(PIDControllerInfo* pid_info, bool ForceCalculation)
{
    bool i_part_active = pid_util_i_part_active(pid_info);

    // Check if i_part_active or filter_active has changed to only recompute coefficients when needed, unless calculation is forced
    if (pid_info->i_part_active_prev == i_part_active  && (ForceCalculation == false))
    {
        pid_info->i_part_active_prev = i_part_active;
        return;
    }

    pid_info->i_part_active_prev = i_part_active;

    // Compute coefficients with / without Ki depending on whether i_part_active
    float Ki = i_part_active ? pid_info->Kp * pid_info->Ts / (2.0f * pid_info->Ti) : 0.0f;

    pid_info->C[0] = pid_info->Kp + Ki;
    pid_info->C[1] = -pid_info->Kp + Ki;

}

/**
* 
* @param pid_info Pointer to the PIDControllerInfo structure
* 
* @returns whether the I part should be computed or not depending on whether bounds are exceeded.
*/
bool pid_util_i_part_active(PIDControllerInfo* pid_info)
{
    // !(bounds exceeded)
    return !(pid_info->m[0] >= pid_info->m_max || pid_info->m[0] <= pid_info->m_min);
}

/**
* @param a first number for comparison.
* @param b second number for comparison.
* 
* @returns a: if a > b, b: otherwise
*/
float pid_util_max(float a, float b)
{
    return a > b ? a : b;
}


/**
* @param a first number for comparison.
* @param b second number for comparison.
*
* @returns a: if a < b, b: otherwise
*/
float pid_util_min(float a, float b)
{
    return a < b ? a : b;
}