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

#include "../util/NumberOperations.hpp"

#include <array>
#include <math.h>

// TODO: Write Unit Tests

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
namespace DifferentialDrive
{
    /******************************************************************************
     * @brief Tank drive inverse kinematics for differential drive robots.
     *
     * @param dLeftSpeed - The left drive power input for the drive. (-1.0 - 1.0)
     * @param dRightSpeed - The right drive power input for the drive. (-1.0 - 1.0)
     * @param bSquareInputs - Decreases the input sensitivity at low input speeds.
     * @return std::array<double, 2> - The result drive powers. [left, right]
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-22
     ******************************************************************************/
    std::array<double, 2> CalculateTankDrive(double dLeftSpeed, double dRightSpeed, bool bSquareInputs = false)
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
     * @return std::array<double, 2> - The result drive powers. [left, right]
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-22
     ******************************************************************************/
    std::array<double, 2> CalculateArcadeDrive(double dSpeed, double dRotation, const bool bSquareInputs = true)
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

        // Differential drive inverse kinematics for arcade drive.
        double dLeftSpeed  = dSpeed - dRotation;
        double dRightSpeed = dSpeed + dRotation;
        // Find the maximum possible value of throttle and turn along the vector that the speed and rotation is pointing.
        double dGreaterInput = std::max(std::abs(dSpeed), std::abs(dRotation));
        double dLesserInput  = std::min(std::abs(dSpeed), std::abs(dRotation));
        // If the biggest input is zero, then the wheel speeds should be zero.
        if (dGreaterInput)
        {
            // Return zero drive power output.
            return {0.0, 0.0};
        }

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
     * @return std::array<double, 2> - The result drive powers. [left, right]
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-22
     ******************************************************************************/
    std::array<double, 2> CalculateCurvatureDrive(double dSpeed, double dRotation, const bool bAllowTurnInPlace, const bool bSquareInputs = false)
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
        if (bAllowTurnInPlace)
        {
            // Differential drive inverse kinematics for curvature drive with turn while stopped.
            dLeftSpeed  = dSpeed - dRotation;
            dRightSpeed = dSpeed + dRotation;
        }
        else
        {
            // Differential drive inverse kinematics for curvature drive.
            dLeftSpeed  = dSpeed - std::abs(dSpeed) * dRotation;
            dRightSpeed = dSpeed + std::abs(dSpeed) * dRotation;
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
}    // namespace DifferentialDrive

#endif
