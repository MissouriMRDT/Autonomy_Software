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

/******************************************************************************
 * @brief Calculate the next control output given the current actual and using the
 *      previously set setpoint.
 *
 * @param dActual - The current actual position of the system.
 * @return double - The resultant PID controller output.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
double PIDController::Calculate(const double dActual)
{
    // Calculate and return the output from the PIDController using the same setpoint.
    return this->Calculate(dActual, m_dSetpoint);
}

/******************************************************************************
 * @brief Calculates the control output using the last provided setpoint and actual.
 *
 * @return double -
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
double PIDController::Calculate()
{
    // Calculate and return the output from the PIDController using the previous actual and setpoint.
    return this->Calculate(m_dLastActual, m_dSetpoint);
}

/******************************************************************************
 * @brief Resets the controller. This erases the integral term buildup, and removes
 *      derivative term on the next iteration.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
void PIDController::Reset()
{
    // Reset necessary values to reset the controller.
    m_bFirstCalculation = true;
    m_dErrorSum         = 0.0;
}

/******************************************************************************
 * @brief Configure the proportional gain parameter. This quickly responds to changes in setpoint,
 *      and provides most of the initial driving force to make corrections.
 *      Some systems can be used with only a P gain, and many can be operated with only PI.
 *      For position based controllers, Proportional gain the first parameter to tune, with integral gain second.
 *      For rate controlled systems, Proportional is often the second after feedforward.
 *
 * @param dKp - The proportional gain for the controller to use for calculation.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
void PIDController::SetProportional(const double dKp)
{
    // Assign member variables.
    m_dKp = dKp;

    // Check signs of gains.
    this->CheckGainSigns();
}

/******************************************************************************
 * @brief Configure the integral gain parameter. This is used for overcoming disturbances, and ensuring that
 *      the controller always minimizes error, when the control actual is close to the setpoint.
 *      Typically tuned second for "Position" based modes, and third for "Rate" or continuous based modes.
 *
 * @param dKi - The integral gain for the controller to use for calculation.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
void PIDController::SetIntegral(const double dKi)
{
    // Check if the current I gain is not zero and we need to scale our current error.
    if (m_dKi != 0)
    {
        // Scale the current error sum, to match the new integral gain.
        m_dErrorSum = m_dErrorSum * (m_dKi / dKi);
    }
    // Check if max integral term effort is enabled.
    if (m_dMaxIEffort != 0)
    {
        // Update max error from new integral and max effort.
        m_dMaxError = m_dMaxIEffort / dKi;
    }

    // Assign integral gain member variable.
    m_dKi = dKi;

    // Check the signs of the PID gains.
    this->CheckGainSigns();
}

/******************************************************************************
 * @brief Configure the derivative gain parameter. This acts as a brake or dampener on the control effort.
 *  The more the controller tries to change the value, the more it counteracts the effort. Typically this
 *  Parameter is tuned last and only used to reduce overshoot in systems. When using derivative gain, you
 *  can usually get a more responsive controller by over-tuning proportional gain so that small oscillations
 *  are visible and then bringing derivative gain up until the oscillations disappear.
 *
 * @param dKd - The derivative gain for the controller to use for calculation.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
void PIDController::SetDerivative(const double dKd)
{
    // Assign member variables.
    m_dKd = dKd;

    // Check signs of gains.
    this->CheckGainSigns();
}

/******************************************************************************
 * @brief Configure the feedforward gain parameter. This is excellent for velocity, rate,
 *      and other continuous control modes where you can expect a rough output value based
 *      solely on the setpoint. This should not be used in "position" based control modes.
 *
 * @param dKff - The feedforward gain for the controller to use for calculation.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
void PIDController::SetFeedforward(const double dKff)
{
    // Assign member variables.
    m_dKff = dKff;

    // Check signs of gains.
    this->CheckGainSigns();
}

/******************************************************************************
 * @brief Mutator for PID gains of the controller.
 *
 * @param dKp - The proportional gain for the controller to use for calculation.
 * @param dKi - The integral gain for the controller to use for calculation.
 * @param dKd - The derivative gain for the controller to use for calculation.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
void PIDController::SetPID(const double dKp, const double dKi, const double dKd)
{
    // Update member variables.
    m_dKp = dKp;
    m_dKd = dKd;

    // Call SetIntegral since is scales the accumulated error and checks signs.
    this->SetIntegral(dKi);
}

/******************************************************************************
 * @brief Mutator for PID and Feedforward gains of the controller.
 *
 * @param dKp - The proportional gain for the controller to use for calculation.
 * @param dKi - The integral gain for the controller to use for calculation.
 * @param dKd - The derivative gain for the controller to use for calculation.
 * @param dKff - The feedforward gain for the controller to use for calculation.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
void PIDController::SetPID(const double dKp, const double dKi, const double dKd, const double dKff)
{
    // Update member variables.
    m_dKp  = dKp;
    m_dKd  = dKd;
    m_dKff = dKff;

    // Call SetIntegral since is scales the accumulated error and checks signs.
    this->SetIntegral(dKi);
}

/******************************************************************************
 * @brief Mutator for the setpoint for the PID controller.
 *
 * @param dSetpoint - The setpoint that the controller should aim to get the actual to.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
void PIDController::SetSetpoint(const double dSetpoint)
{
    // Assign member variable.
    m_dSetpoint = dSetpoint;
}

/******************************************************************************
 * @brief Mutator for the maximum allowable distance of the setpoint from the actual value.
 *      This doesn't limit the setable value of the setpoint. It only limits the value that the
 *      controller is given specify the error from the setpoint.
 *
 * @param dMaxSetpointDifference - The allowable distance of the setpoint from the actual.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
void PIDController::SetMaxSetpointDifference(const double dMaxSetpointDifference)
{
    // Assign member variables.
    m_dMaxSetpointDifference = dMaxSetpointDifference;
}

/******************************************************************************
 * @brief Mutator for the max value of the integral term during PID calculation.
 *      This is useful for preventing integral wind-up runaways.
 *
 * @param dMaxIEffort - The max allowable value of the integral term.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
void PIDController::SetMaxIntegralEffort(const double dMaxIEffort)
{
    // Assign member variable.
    m_dMaxIEffort = dMaxIEffort;
}

/******************************************************************************
 * @brief Mutator for setting the minimum and maximum allowable values of the
 *      control output from the controller.
 *
 * @param dMinEffort - The minimum allowable output from the PID controller.
 * @param dMaxEffort - The maximum allowable output from the PID controller.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
void PIDController::SetOutputLimits(const double dMinEffort, const double dMaxEffort)
{
    // Check if the maximum is greater than the minimum.
    if (dMaxEffort > dMinEffort)
    {
        // Assign member variables.
        m_dMinEffort = dMinEffort;
        m_dMaxEffort = dMaxEffort;

        // Ensure the bounds of the integral term are within the bounds of the allowable output range.
        if (m_dMaxIEffort == 0 || m_dMaxIEffort > (dMaxEffort - dMinEffort))
        {
            // Set new integral max output.
            this->SetMaxIntegralEffort(dMaxEffort - dMinEffort);
        }
    }
}

/******************************************************************************
 * @brief Mutator for setting the minimum and maximum allowable values of the
 *      control output from the controller.
 *
 * @param dMinMax - The overall output range of the controller. Must be positive.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
void PIDController::SetOutputLimits(const double dMinMax)
{
    // Set output limits with the same max and min.
    this->SetOutputLimits(-dMinMax, dMinMax);
}

/******************************************************************************
 * @brief Mutator for the maximum rate that the output can change per iteration.
 *
 * @param dOutputRampRate - The maximum ramp rate of the output for the controller.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
void PIDController::SetOutputRampRate(const double dOutputRampRate)
{
    // Assign member variable.
    m_dOutputRampRate = dOutputRampRate;
}

void PIDController::SetOutputFilter(const double dStrength)
{
    // Check if the
}

/******************************************************************************
 * @brief To operate correctly, all PID parameters require the same sign.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-17
 ******************************************************************************/
void PIDController::CheckGainSigns()
{
    // Check if the gains are supposed to be reversed.
    if (m_bReversed)
    {
        // All values should be below zero.
        if (m_dKp > 0)
        {
            // Flip sign sign for proportional gain.
            m_dKi *= -1;
        }
        if (m_dKi > 0)
        {
            // Flip sign for integral gain.
            m_dKi *= -1;
        }
        if (m_dKd > 0)
        {
            // Flip sign for derivative gain.
            m_dKd *= -1;
        }
        if (m_dKff > 0)
        {
            // Flip sign for feedforward gain.
            m_dKff *= -1;
        }
    }
    else
    {
        // All values should be above zero.
        if (m_dKp < 0)
        {
            // Flip sign sign for proportional gain.
            m_dKi *= -1;
        }
        if (m_dKi < 0)
        {
            // Flip sign for integral gain.
            m_dKi *= -1;
        }
        if (m_dKd < 0)
        {
            // Flip sign for derivative gain.
            m_dKd *= -1;
        }
        if (m_dKff < 0)
        {
            // Flip sign for feedforward gain.
            m_dKff *= -1;
        }
    }
}
