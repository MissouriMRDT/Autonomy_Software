/******************************************************************************
 * @brief Defines and implements namspaces and functions for algorithms
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
     * @param dSpeed - Speed at which the robot should drive forward/backward. (-1.0 - 1.0)
     * @param dRotation - The rotation rate of the robot. Clockwise is positive. (-1.0 - 1.0)
     * @param bSquareInputs - Decreases the input sensitivity at low input speeds.
     * @return std::array<double, 2> - The result drive powers. [left, right]
     *
     * @author clayjay3 (claytonraycowen@gmail.com)
     * @date 2023-09-22
     ******************************************************************************/
    std::array<double, 2> CalculateArcadeDrive(double dSpeed, double dRotation, const bool bSquareInputs = true)
    {
        //
    }
}    // namespace DifferentialDrive

#endif
