/******************************************************************************
 * @brief Implements the PIDController class.
 *
 * @file PIDController.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "PIDController.h"
#include "../../util/NumberOperations.hpp"

/******************************************************************************
 * @brief Construct a new PIDController::PIDController object.
 *
 * @param dKp - The proportional gain (Kp) component adjusts the control output in proportion to the current error.
 * @param dKi - The integral gain (Ki) component accumulates past errors to eliminate steady-state error.
 * @param dKd - The derivative gain (Kd) component anticipates future error by considering the rate of change of the error.
 * @param dKff - The feedforward gain (Kff) allows for predictive control, improving response time and minimizing overshoot.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
PIDController::PIDController(const double dKp, const double dKi, const double dKd, const double dKff)
{
    // Initialize member variable.
    m_dKp                    = dKp;
    m_dKi                    = dKi;
    m_dKd                    = dKd;
    m_dKff                   = dKff;
    m_dSetpoint              = 0.0;
    m_dErrorSum              = 0.0;
    m_dMaxError              = 0.0;
    m_dMaxIEffort            = 0.0;
    m_dMinEffort             = 0.0;
    m_dMaxEffort             = 0.0;
    m_dLastActual            = 0.0;
    m_dOutputRampRate        = 0.0;
    m_dLastControlOutput     = 0.0;
    m_dOutputFilter          = 0.0;
    m_dMaxSetpointDifference = 0.0;
    m_bFirstCalculation      = true;
    m_bReversed              = false;
}

/******************************************************************************
 * @brief Calculate the next control output given the current actual and a setpoint.
 *
 * @param dActual - The current actual position of the system.
 * @param dSetpoint - The desired setpoint of the controller.
 * @return double - The resultant PID controller output.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
double PIDController::Calculate(const double dActual, const double dSetpoint)
{
    // Create instance variables.
    double dPTermOutput;
    double dITermOutput;
    double dDTermOutput;
    double dFFTermOutput;
    double dOutput;

    // Update setpoint member variable.
    m_dSetpoint = dSetpoint;

    // Apply the sliding setpoint range adjust if necessary.
    if (m_dMaxSetpointDifference != 0)
    {
        // Clamp current setpoint value within a certain range of the current actual.
        m_dSetpoint = numops::Clamp(m_dSetpoint, dActual - m_dMaxSetpointDifference, dActual + m_dMaxSetpointDifference);
    }

    // Determine error from setpoint.
    double dError = dSetpoint - dActual;

    // Calculate feedforward term.
    dFFTermOutput = m_dKff * dSetpoint;
    // Calculate proportional term.
    dPTermOutput = m_dKp * dError;

    // Check if this is the first time doing a calculation.
    if (m_bFirstCalculation)
    {
        // Assume the process variable hold same as previous.
        m_dLastActual = dActual;
        // Assume the last output is the current time-independent output.
        m_dLastControlOutput = dPTermOutput + dFFTermOutput;

        // Set first calculation toggle to false.
        m_bFirstCalculation = false;
    }

    // Calculate derivative term.
    // Note, derivative is actually negative and "slows" the system if it's doing
    // the correct thing. Small gain values help prevent output spikes and overshoot.
    dDTermOutput  = -m_dKd * (dActual - m_dLastActual);
    m_dLastActual = dActual;

    // Calculate integral term.
    // The integral term is more complex. There's several things to factor in to make it easier to deal with.
    // 1. m_dMaxIEffort restricts the amount of output contributed by the integral term.
    // 2. prevent windup by not increasing errorSum if we're already running against our max Ioutput
    // 3. prevent windup by not increasing errorSum if output is output=maxOutput
    dITermOutput = m_dKi * m_dErrorSum;
    // Apply integral term limits.
    if (m_dMaxIEffort != 0)
    {
        // Clamp current integral term output.
        dITermOutput = numops::Clamp(dITermOutput, -m_dMaxIEffort, m_dMaxIEffort);
    }

    // Here's PID!
    dOutput = dFFTermOutput + dPTermOutput + dITermOutput + dDTermOutput;
    // Check if error is at not at limits.
    if (m_dMinEffort != m_dMaxEffort && !numops::Bounded(dOutput, m_dMinEffort, m_dMaxEffort))
    {
        // Reset the error to a sane level. Settings to the current error ensures a smooth transition when the P term
        // decreases enough for the I term to start acting upon the controller properly.
        m_dErrorSum = dError;
    }
    // Check if the error is within the allowed ramp rate compared to the last output.
    else if (m_dOutputRampRate != 0 && !numops::Bounded(dOutput, m_dLastControlOutput - m_dOutputRampRate, m_dLastControlOutput + m_dOutputRampRate))
    {
        // Reset the error to a sane level. Settings to the current error ensures a smooth transition when the P term
        // decreases enough for the I term to start acting upon the controller properly.
        m_dErrorSum = dError;
    }
    // Check if integral term output should be limited.
    else if (m_dMaxIEffort != 0)
    {
        // In addition to output limiting directly, we also want to prevent integral term wind-up. Restrict directly.
        m_dErrorSum = numops::Clamp(m_dErrorSum + dError, -m_dMaxError, m_dMaxError);
    }
    else
    {
        // Accumulate error.
        m_dErrorSum += dError;
    }

    // Restrict output to our specified ramp limits.
    if (m_dOutputRampRate != 0)
    {
        // Clamp output to ramp rate.
        dOutput = numops::Clamp(dOutput, m_dLastControlOutput - m_dOutputRampRate, m_dLastControlOutput + m_dOutputRampRate);
    }
    // Restrict output to our specified effort limits.
    if (m_dMinEffort != m_dMaxEffort)
    {
        // Clamp output to effort limits.
        dOutput = numops::Clamp(dOutput, m_dMinEffort, m_dMaxEffort);
    }
    // Reduce jitters and output spikes with an exponential rolling sum filter.
    if (m_dOutputFilter != 0)
    {
        // Filter output effort.
        dOutput = (m_dLastControlOutput * m_dOutputFilter) + (dOutput * (1 - m_dOutputFilter));
    }

    // Update last output member variable.
    m_dLastControlOutput = dOutput;

    // Return the PID controller calculated output effort.
    return dOutput;
}
