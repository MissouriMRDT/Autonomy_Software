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

#include <array>

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

        float m_fTargetSpeedLeft;
        float m_fTargetSpeedRight;

    public:
        /////////////////////////////////////////
        // Declare public enums that are specific to and used withing this class.
        /////////////////////////////////////////

        // Enumerator used to specify what method of drive control to use.
        enum DifferentialControlMethod
        {
            eArcadeDrive,      // Typical drive control method for flightsticks. Uses speed and turn input to determine drive powers.
            eCurvatureDrive    // Similar to arcade drive with flightsticks, but the current turning speed of the robot is dampened when moving fast.
        };

        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////

        DriveBoard();
        ~DriveBoard();
        std::array<float, 2> CalculateMove(const float fSpeed, const float fAngle, const DifferentialControlMethod eKinematicsMethod);
        void SendDrive(float fLeftSpeed, float fRightSpeed);
        void SendStop();

        /////////////////////////////////////////
        // Setters
        /////////////////////////////////////////

        void SetMaxDrivePower();

        /////////////////////////////////////////
        // Getters
        /////////////////////////////////////////

        float GetMaxDrivePower() const;
};
#endif
