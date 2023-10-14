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

#include "../AutonomyConstants.h"
#include "../AutonomyLogging.h"
#include "../algorithms/DifferentialDrive.hpp"
#include "../util/NumberOperations.hpp"

/******************************************************************************
 * @brief Construct a new Drive Board::DriveBoard object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-21
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
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-21
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
 * @param fAngle - The angle to drive towards. (0 - 360) 0 is North.
 * @param eKinematicsMethod - The kinematics model to use for differential drive control. Enum within DifferentialDrive.hpp
 * @return std::array<int, 2> - 1D array of length 2 containing two values. (left power, right power)
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-21
 ******************************************************************************/
std::array<float, 2> DriveBoard::CalculateMove(const float fSpeed, const float fAngle, const DifferentialControlMethod eKinematicsMethod)
{
    // Create instance variables.
    diffdrive::DrivePowers stDrivePowers;

    // Check what kinematics model we should use.
    switch (eKinematicsMethod)
    {
        case eArcadeDrive: stDrivePowers = diffdrive::CalculateArcadeDrive(double(fSpeed), double(fAngle), constants::DRIVE_MIN_POWER); break;
        case eCurvatureDrive:
            stDrivePowers = diffdrive::CalculateCurvatureDrive(double(fSpeed),
                                                               double(fAngle),
                                                               constants::DRIVE_CURVATURE_KINEMATICS_ALLOW_TURN_WHILE_STOPPED,
                                                               constants::DRIVE_SQUARE_CONTROL_INPUTS);
            break;
    }

    // Update member variables with new targets speeds. Adjust to match power range.
    m_fTargetSpeedLeft  = double(stDrivePowers.dLeftDrivePower);
    m_fTargetSpeedRight = double(stDrivePowers.dRightDrivePower);

    // Submit logger message.
    LOG_DEBUG(logging::g_qSharedLogger, "Driving at: ({}, {})", m_fTargetSpeedLeft, m_fTargetSpeedRight);

    return {m_fTargetSpeedLeft, m_fTargetSpeedRight};
}

/******************************************************************************
 * @brief Sets the left and right drive powers of the drive board.
 *
 * @param fLeftSpeed - Left drive speed (-1 to 1)
 * @param fRightSpeed - Right drive speed (-1 to 1)
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-21
 ******************************************************************************/
void DriveBoard::SendDrive(float fLeftSpeed, float fRightSpeed)
{
    // Limit input values.
    fLeftSpeed  = std::clamp(fLeftSpeed, -1.0f, 1.0f);
    fRightSpeed = std::clamp(fRightSpeed, -1.0f, 1.0f);

    // Update member variables with new target speeds.
    m_fTargetSpeedLeft  = fLeftSpeed;
    m_fTargetSpeedRight = fRightSpeed;

    // Remap -1.0 - 1.0 range to drive power range defined in constants. This is so that the driveboard/rovecomm can understand our input.
    m_fTargetSpeedLeft  = numops::MapRange(m_fTargetSpeedLeft, -1.0f, 1.0f, constants::DRIVE_MIN_POWER, constants::DRIVE_MAX_POWER);
    m_fTargetSpeedRight = numops::MapRange(m_fTargetSpeedRight, -1.0f, 1.0f, constants::DRIVE_MIN_POWER, constants::DRIVE_MAX_POWER);
    // Limit the power to max and min effort defined in constants.
    m_fTargetSpeedLeft  = std::clamp(m_fTargetSpeedLeft, constants::DRIVE_MIN_EFFORT, constants::DRIVE_MAX_EFFORT);
    m_fTargetSpeedRight = std::clamp(m_fTargetSpeedRight, constants::DRIVE_MIN_EFFORT, constants::DRIVE_MAX_EFFORT);

    // Send drive command over RoveComm to drive board.
    // TODO: Add RoveComm sendpacket.
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
    // TODO: Add RoveComm sendpacket.
}
