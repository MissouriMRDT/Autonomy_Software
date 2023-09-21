/******************************************************************************
 * @brief Implements the interface for sending commands to the drive board on
 * 		the Rover.
 *
 * @file DriveBoard.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-18
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "./DriveBoard.h"

#include "../AutonomyGlobals.h"
#include "../AutonomyLogging.h"
#include "../util/NumberOperations.hpp"

/******************************************************************************
 * @brief Construct a new Drive Board::DriveBoard object.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-18
 ******************************************************************************/
DriveBoard::DriveBoard()
{
    // Initialize member variables.
    m_fTargetSpeedLeft  = 0.0;
    m_fTargetSpeedRight = 0.0;
}

/******************************************************************************
 * @brief Destroy the Drive Board::DriveBoard object.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-18
 ******************************************************************************/
DriveBoard::~DriveBoard()
{
    // Stop drivetrain.
    this->SendStop();
}

/******************************************************************************
 * @brief This method determines drive powers to make the Rover drive towards a
 * 		given heading at a given speed
 *
 * @param fSpeed - The speed to drive at (-1 to 1)
 * @param fAngle - The angle to drive towards.
 * @return std::array<int, 2> - 1D array of length 2 containing two values. (left power, right power)
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-21
 ******************************************************************************/
std::array<int, 2> DriveBoard::CalculateMove(const float fSpeed, const float fAngle)
{
    // Create instance variables.
    double dSpeedLeft;
    double dSpeedRight;

    if (fAngle > 0)
    {
        dSpeedRight = dSpeedRight * (1 - (fAngle / 180.0));
    }
    else if (fAngle < 0)
    {
        dSpeedLeft = dSpeedLeft * (1 + (fAngle / 180.0));
    }

    m_fTargetSpeedLeft  = int(numops::Clamp<double>(dSpeedLeft, constants::MIN_DRIVE_POWER, constants::MAX_DRIVE_POWER));
    m_fTargetSpeedRight = int(numops::Clamp<double>(dSpeedRight, constants::MIN_DRIVE_POWER, constants::MAX_DRIVE_POWER));

    LOG_DEBUG(g_qSharedLogger, "Driving at: ({}, {})", m_fTargetSpeedLeft, m_fTargetSpeedRight);

    return {m_fTargetSpeedLeft, m_fTargetSpeedRight};
}

/******************************************************************************
 * @brief Sets the left and right drive powers of the drive board.
 *
 * @param nLeftTarget - Left drive speed (-1 to 1)
 * @param nRightTarget - Right drive speed (-1 to 1)
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-18
 ******************************************************************************/
void DriveBoard::SendDrive(const float fLeftTarget, const float fRightTarget)
{
    // Update member variables with new target speeds.
    m_fTargetSpeedLeft  = fLeftTarget;
    m_fTargetSpeedRight = fRightTarget;

    // Send drive command over RoveComm to drive board.
}

/******************************************************************************
 * @brief Stop the drivetrain of the Rover.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-06-18
 ******************************************************************************/
void DriveBoard::SendStop()
{
    // Update member variables with new target speeds.
    m_fTargetSpeedLeft  = 0.0;
    m_fTargetSpeedRight = 0.0;

    // Send drive command over RoveComm to drive board.
}
