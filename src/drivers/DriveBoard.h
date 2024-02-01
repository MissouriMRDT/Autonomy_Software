/******************************************************************************
 * @brief Defines the driver for sending commands to the drive board on
 * 		the Rover.
 *
 * @file DriveBoard.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-18
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#ifndef DRIVEBOARD_H
#define DRIVEBOARD_H

#include "../algorithms/DifferentialDrive.hpp"

/// \cond
#include <array>

/// \endcond

/******************************************************************************
 * @brief This class handles communication with the drive board on the rover by
 *      sending RoveComm packets over the network.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-21
 ******************************************************************************/
class DriveBoard
{
    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        diffdrive::DrivePowers m_stDrivePowers;    // Struct used to store the left and right drive powers of the robot.
        controllers::PIDController* m_pPID;        // The PID controller used for drive towards a heading.

    public:
        /////////////////////////////////////////
        // Declare public enums that are specific to and used within this class.
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////

        DriveBoard();
        ~DriveBoard();
        diffdrive::DrivePowers CalculateMove(const double dGoalSpeed,
                                             const double dGoalHeading,
                                             const double dActualHeading,
                                             const diffdrive::DifferentialControlMethod eKinematicsMethod);
        void SendDrive(double dLeftSpeed, double dRightSpeed);
        void SendStop();

        /////////////////////////////////////////
        // Setters
        /////////////////////////////////////////

        /////////////////////////////////////////
        // Getters
        /////////////////////////////////////////

        diffdrive::DrivePowers GetDrivePowers() const;
};
#endif
