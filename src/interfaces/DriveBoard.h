/******************************************************************************
 * @brief Defines the interface for sending commands to the drive board on
 * 		the Rover.
 *
 * @file DriveBoard.h
 * @author Eli Byrd (edbgkk@mst.edu), ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-06-18
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/
#include <vector>

#ifndef DRIVEBOARD_H
#define DRIVEBOARD_H

class DriveBoard
{
    private:
        int m_iTargetSpeedLeft;
        int m_iTargetSpeedRight;

    public:
        DriveBoard();
        ~DriveBoard();

        std::vector<int> CalculateMove(float fSpeed, float fAngle);
        void SendDrive(int iLeftTarget, int iRightTarget);
        void SendStop();
};

#endif    // DRIVEBOARD_H
