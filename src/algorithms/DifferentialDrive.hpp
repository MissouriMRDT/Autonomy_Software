/******************************************************************************
 * @brief Defines and implements namespaces and functions for algorithms
 *      that pertain to differential drive (tank drive) robots.
 *
 * @file DifferentialDrive.hpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-21
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef DIFFERENTIAL_DRIVE_HPP
#define DIFFERENTIAL_DRIVE_HPP

#include "../AutonomyConstants.h"
#include "../util/NumberOperations.hpp"
#include "controllers/PIDController.h"

/// \cond
#include <array>
#include <math.h>

/// \endcond

/******************************************************************************
 * @brief Namespace containing algorithms related to calculating drive powers,
 *      odometry, trajectories, kinematics, etc of differential drive (tank drive) robots.
 *
 *      Each drive function provides different inverse kinematic relations for a
 *      differential drive robot.
 *
 *      This library uses the NWU axes convention (North-West-Up as external
 *      reference in the world frame). The positive X axis points ahead, the positive
 *      Y axis points to the left, and the positive Z axis points up. Rotations
 *      follow the right-hand rule, so counterclockwise rotation around the Z axis is
 *      positive.
 *
 *      Inputs smaller then 0.02 will be set to 0, and larger values will be scaled
 *      so that the full range is still used. This deadband value can be changed
 *      with SetDeadband().
 *
 *      Referenced from:
 *      https://github.com/wpilibsuite/allwpilib/
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-21
 ******************************************************************************/
namespace diffdrive
{
    /////////////////////////////////////////
    // Declare public enums that are specific to and used within this namespace.
    /////////////////////////////////////////

    // Enum for choosing the differential drive method for certain functions.
    // Enumerator used to specify what method of drive control to use.
    enum DifferentialControlMethod
    {
        eTankDrive,        // Simple controller. Left and right input is directly assigned to each side of the drivetrain.
        eArcadeDrive,      // Typical drive control method for flightsticks. Uses speed and turn input to determine drive powers.
        eCurvatureDrive    // Similar to arcade drive with flightsticks, but the current turning speed of the robot is dampened when moving fast.
    };

    /******************************************************************************
     * @brief This struct is used to store the left and right drive powers for the robot.
     *      Storing these values in a struct allows for easy handling and access to said
     *      variables.
     *
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-13
     ******************************************************************************/
    struct DrivePowers
    {
        public:
            // Define public struct attributes.
            double dLeftDrivePower;
            double dRightDrivePower;
    };

    /******************************************************************************
     * @brief Tank drive inverse kinematics for differential drive robots.
     *
     * @param dLeftSpeed - The left drive power input for the drive. (-1.0 - 1.0)
     * @param dRightSpeed - The right drive power input for the drive. (-1.0 - 1.0)
     * @param bSquareInputs - Decreases the input sensitivity at low input speeds.
     * @return DrivePowers - The result drive powers. [left, right]
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-22
     ******************************************************************************/
    inline DrivePowers CalculateTankDrive(double dLeftSpeed, double dRightSpeed, bool bSquareInputs = false)
    {
        // Limit the input powers.
        dLeftSpeed  = std::clamp(dLeftSpeed, -1.0, 1.0);
        dRightSpeed = std::clamp(dRightSpeed, -1.0, 1.0);

        // Determine if we should square the inputs.
        if (bSquareInputs)
        {
            // Square the inputs but keep the sign.
            dLeftSpeed  = std::copysign(dLeftSpeed * dLeftSpeed, dLeftSpeed);
            dRightSpeed = std::copysign(dRightSpeed * dRightSpeed, dRightSpeed);
        }

        // Return result drive powers.
        return {dLeftSpeed, dRightSpeed};
    }

    /******************************************************************************
     * @brief Arcade drive inverse kinematics for differential drive robots.
     *
     * @param dSpeed - Speed at which the robot should drive forward/backward. Forward is positive. (-1.0 - 1.0)
     * @param dRotation - The rotation rate of the robot. Clockwise is positive. (-1.0 - 1.0)
     * @param bSquareInputs - Decreases the input sensitivity at low input speeds.
     * @return DrivePowers - The result drive powers. [left, right]
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-22
     ******************************************************************************/
    inline DrivePowers CalculateArcadeDrive(double dSpeed, double dRotation, const bool bSquareInputs = false)
    {
        // Limit the input powers.
        dSpeed    = std::clamp(dSpeed, -1.0, 1.0);
        dRotation = std::clamp(dRotation, -1.0, 1.0);

        // Determine if we should square the inputs.
        if (bSquareInputs)
        {
            // Square the inputs but keep the sign.
            dSpeed    = std::copysign(dSpeed * dSpeed, dSpeed);
            dRotation = std::copysign(dRotation * dRotation, dRotation);
        }

        // Find the maximum possible value of throttle and turn along the vector that the speed and rotation is pointing.
        double dGreaterInput = std::max(std::abs(dSpeed), std::abs(dRotation));
        double dLesserInput  = std::min(std::abs(dSpeed), std::abs(dRotation));
        // If the biggest input is zero, then the wheel speeds should be zero.
        if (dGreaterInput == 0.0)
        {
            // Return zero drive power output.
            return {0.0, 0.0};
        }

        // Differential drive inverse kinematics for arcade drive.
        double dLeftSpeed  = dSpeed + dRotation;
        double dRightSpeed = dSpeed - dRotation;
        // Desaturate the input. Normalize to (-1.0, 1.0).
        double dSaturatedInput = (dGreaterInput + dLesserInput) / dGreaterInput;
        dLeftSpeed /= dSaturatedInput;
        dRightSpeed /= dSaturatedInput;

        // Return result drive powers.
        return {dLeftSpeed, dRightSpeed};
    }

    /******************************************************************************
     * @brief Curvature drive inverse kinematics for differential drive robots.
     *      The rotation parameter controls the curvature of the robot's path rather than
     *      it's rate of heading change. This makes the robot more controllable at high speeds.
     *
     * @param dSpeed - Speed at which the robot should drive forward/backwards. Forward is positive. (-1.0, 1.0)
     * @param dRotation - The normalized curvature of the robot. Clockwise is positive. (-1.0, 1.0)
     * @param bAllowTurnInPlace - Whether or not forward input is required to turn. True makes this control exactly like a car.
     * @param bSquareInputs - Decreases the input sensitivity at low input speeds.
     * @return DrivePowers - The result drive powers. [left, right]
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-22
     ******************************************************************************/
    inline DrivePowers CalculateCurvatureDrive(double dSpeed, double dRotation, const bool bAllowTurnInPlace, const bool bSquareInputs = false)
    {
        // Create instance variables.
        double dLeftSpeed  = 0.0;
        double dRightSpeed = 0.0;

        // Limit the input powers.
        dSpeed    = std::clamp(dSpeed, -1.0, 1.0);
        dRotation = std::clamp(dRotation, -1.0, 1.0);

        // Determine if we should square the inputs.
        if (bSquareInputs)
        {
            // Square the inputs but keep the sign.
            dSpeed    = std::copysign(dSpeed * dSpeed, dSpeed);
            dRotation = std::copysign(dRotation * dRotation, dRotation);
        }

        // Check if turn-in-place is allowed.
        if (bAllowTurnInPlace && dSpeed == 0.0)
        {
            // Differential drive inverse kinematics for curvature drive with turn while stopped.
            dLeftSpeed  = dSpeed + dRotation;
            dRightSpeed = dSpeed - dRotation;
        }
        else
        {
            // Differential drive inverse kinematics for curvature drive.
            dLeftSpeed  = dSpeed + std::abs(dSpeed) * dRotation;
            dRightSpeed = dSpeed - std::abs(dSpeed) * dRotation;
        }

        // Desaturate the input. Normalize to (-1.0, 1.0).
        double dMaxMagnitude = std::max(std::abs(dLeftSpeed), std::abs(dRightSpeed));
        if (dMaxMagnitude > 1.0)
        {
            dLeftSpeed /= dMaxMagnitude;
            dRightSpeed /= dMaxMagnitude;
        }

        // Return result drive powers.
        return {dLeftSpeed, dRightSpeed};
    }

    /******************************************************************************
     * @brief This function will calculate the drive powers for a given speed and
     *      absolute heading. The method used to get the drive powers is determined by
     *      the given enumerator (must be arcade or curvature). The turning rate or
     *      curvature is determined by the given PID controller which must be properly
     *      configured.
     *
     * @param dGoalSpeed - The goal speed for the robot.
     * @param dGoalHeading - The goal absolute heading for the robot. (0-360 degrees, CW positive.)
     * @param dActualHeading - The actual current heading of the robot.
     * @param eDriveMethod - The differential drive method to use for navigation.
     * @param PID - A reference to the PID controller to use for hitting the heading setpoint.
     * @return DrivePowers - The resultant drive powers.
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-10-19
     ******************************************************************************/
    inline DrivePowers CalculateMotorPowerFromHeading(double dGoalSpeed,
                                                      double dGoalHeading,
                                                      double dActualHeading,
                                                      DifferentialControlMethod eDriveMethod,
                                                      controllers::PIDController& PID)
    {
        // Create instance variables.
        DrivePowers stOutputPowers;

        // Get control output from PID controller.
        double dTurnOutput = PID.Calculate(dActualHeading, dGoalHeading);
        // Calculate drive powers from inverse kinematics of goal speed and turning adjustment.
        switch (eDriveMethod)
        {
            case eArcadeDrive: stOutputPowers = CalculateArcadeDrive(dGoalSpeed, dTurnOutput, constants::DRIVE_SQUARE_CONTROL_INPUTS); break;
            case eCurvatureDrive:
                stOutputPowers = CalculateCurvatureDrive(dGoalSpeed,
                                                         dTurnOutput,
                                                         constants::DRIVE_CURVATURE_KINEMATICS_ALLOW_TURN_WHILE_STOPPED,
                                                         constants::DRIVE_SQUARE_CONTROL_INPUTS);
                break;
            default:
                stOutputPowers = CalculateCurvatureDrive(dGoalSpeed,
                                                         dGoalHeading,
                                                         constants::DRIVE_CURVATURE_KINEMATICS_ALLOW_TURN_WHILE_STOPPED,
                                                         constants::DRIVE_SQUARE_CONTROL_INPUTS);
                break;
        }

        // Return result powers.
        return stOutputPowers;
    }
}    // namespace diffdrive
#endif
