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
        // Declare public methods and member variables.
        /////////////////////////////////////////

        DriveBoard();
        ~DriveBoard();
        std::array<int, 2> CalculateMove(const float fSpeed, const float fAngle);
        void SendDrive(const float fLeftTarget, const float fRightTarget);
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
