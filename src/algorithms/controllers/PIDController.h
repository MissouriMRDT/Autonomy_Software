/******************************************************************************
 * @brief Defines the PIDController class.
 *
 * @file PIDController.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-16
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/******************************************************************************
 * @brief This class encapsulates the fundamental principles of Proportional-Integral-Derivative (PID) control with an additional feedforward component. It provides a
 *      versatile control mechanism for various systems, enabling precise control of processes by adjusting an output in response to the error between a desired setpoint
 *      and a measured process variable.
 *
 * The key features of this class include:
 * - Proportional Control: The proportional gain (Kp) component adjusts the control output in proportion to the current error.
 * - Integral Control: The integral gain (Ki) component accumulates past errors to eliminate steady-state error.
 * - Derivative Control: The derivative gain (Kd) component anticipates future error by considering the rate of change of the error.
 * - Feedforward Control: The feedforward gain (Kff) allows for predictive control, improving response time and minimizing overshoot.
 *                 See https://blog.opticontrols.com/archives/297 for a better explanation on feedforward controller.
 *
 * Additionally, this PID controller offers an optional wraparound feature. When enabled, it can be used for systems with non-linear, circular, or cyclic setpoints,
 * effectively managing control in such scenarios.
 *
 * This class is a valuable tool for control engineering, robotics, and automation, allowing you to fine-tune the control behavior of various processes and systems.
 *
 * For specific details on class attributes and methods, please refer to the class documentation.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-16
 ******************************************************************************/
class PIDController
{
    public:
        /////////////////////////////////////////
        // Declare public enums that are specific to and used withing this class.
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////

        PIDController(const double dKp, const double dKi, const double dKd, const double dKff = 0.0);
        double Calculate(const double dActual, const double dSetpoint);
        double Calculate(const double dActual);
        double Calculate();
        void EnableContinuousInput(const double dMinimumInput, const double dMaximumInput);
        void DisableContinuousInput();
        void Reset();

        /////////////////////////////////////////
        // Setters
        /////////////////////////////////////////

        void SetProportional(const double dKp);
        void SetIntegral(const double dKi);
        void SetDerivative(const double dKd);
        void SetFeedforward(const double dKff);
        void SetPID(const double dKp, const double dKi, const double dKd);
        void SetPID(const double dKp, const double dKi, const double dKd, const double dKff);
        void SetSetpoint(const double dSetpoint);
        void SetMaxSetpointDifference(const double dMaxSetpointDifference);
        void SetMaxIntegralEffort(const double dMaxIEffort);
        void SetOutputLimits(const double dMinEffort, const double dMaxEffort);
        void SetOutputLimits(const double dMaxMin);
        void SetOutputRampRate(const double dOutputRampRate);
        void SetOutputFilter(const double dStrength);
        void SetDirection(const bool bReversed = false);

        /////////////////////////////////////////
        // Getters
        /////////////////////////////////////////

        double GetProportional() const;
        double GetIntegral() const;
        double GetDerivative() const;
        double GetFeedforward() const;
        double GetSetpoint() const;
        double GetMaxSetpointDifference() const;
        double GetMaxIntegralEffort() const;
        double GetOutputRampRate() const;
        double GetOutputFilter() const;
        double GetContinuousInputEnabled() const;
        bool GetReversed() const;

    private:
        /////////////////////////////////////////
        // Declare public methods.
        /////////////////////////////////////////

        void CheckGainSigns();

        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        double m_dKp;                        // Proportional gain.
        double m_dKi;                        // Integral gain.
        double m_dKd;                        // Derivative gain.
        double m_dKff;                       // Feedforward gain.
        double m_dSetpoint;                  // Current control setpoint.
        double m_dErrorSum;                  // Error accumulation.
        double m_dMaxError;                  // Max allowed error.
        double m_dMaxIEffort;                // Max integral calculated term effort.
        double m_dMinEffort;                 // Min output of the PID controller.
        double m_dMaxEffort;                 // Max output of the PID controller.
        double m_dLastActual;                // The previous process variable input.
        double m_dOutputRampRate;            // The max rate of change of the controller output.
        double m_dLastControlOutput;         // The previous control output of the controller.
        double m_dOutputFilter;              // Strength of an exponential rolling sum filter. Used to reduce sharp oscillations.
        double m_dMaxSetpointDifference;     // Limit on how far the setpoint can be from the current position.
        double m_dMinimumContinuousInput;    // The minimum wraparound value of the input for the controller.
        double m_dMaximumContinuousInput;    // The maximum wraparound value of the input for the controller.
        bool m_bControllerIsContinuous;      // Whether of not the controller is working with a circular position range. Good for setpoints that use degrees.
        bool m_bFirstCalculation;            // Whether or not this is the first iteration of the control loop.
        bool m_bReversed;                    // Operating direction of the controller.
};
#endif
