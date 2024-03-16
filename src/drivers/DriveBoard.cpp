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
#include "../AutonomyGlobals.h"
#include "../AutonomyLogging.h"

/// \cond
#include <RoveComm/RoveCommManifest.h>

/// \endcond

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
    m_stDrivePowers.dLeftDrivePower  = 0.0;
    m_stDrivePowers.dRightDrivePower = 0.0;

    // Configure PID controller for heading hold function.
    m_pPID = new controllers::PIDController(constants::DRIVE_PID_PROPORTIONAL, constants::DRIVE_PID_INTEGRAL, constants::DRIVE_PID_DERIVATIVE);
    m_pPID->SetMaxSetpointDifference(constants::DRIVE_PID_MAX_ERROR_PER_ITER);
    m_pPID->SetMaxIntegralEffort(constants::DRIVE_PID_MAX_INTEGRAL_TERM);
    m_pPID->SetOutputLimits(constants::DRIVE_PID_MAX_OUTPUT_EFFORT);
    m_pPID->SetOutputRampRate(constants::DRIVE_PID_MAX_RAMP_RATE);
    m_pPID->SetOutputFilter(constants::DRIVE_PID_OUTPUT_FILTER);
    m_pPID->SetDirection(constants::DRIVE_PID_OUTPUT_REVERSED);
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

    // Delete dynamically allocated memory.
    delete m_pPID;

    // Set dangling pointers to null.
    m_pPID = nullptr;
}

/******************************************************************************
 * @brief This method determines drive powers to make the Rover drive towards a
 * 		given heading at a given speed
 *
 * @param dGoalSpeed - The speed to drive at (-1 to 1)
 * @param dGoalHeading - The angle to drive towards. (0 - 360) 0 is North.
 * @param dActualHeading - The real angle that the Rover is current facing.
 * @param eKinematicsMethod - The kinematics model to use for differential drive control. Enum within DifferentialDrive.hpp
 * @return diffdrive::DrivePowers - A struct containing two values. (left power, right power)
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-21
 ******************************************************************************/
diffdrive::DrivePowers DriveBoard::CalculateMove(const double dGoalSpeed,
                                                 const double dGoalHeading,
                                                 const double dActualHeading,
                                                 const diffdrive::DifferentialControlMethod eKinematicsMethod)
{
    // Calculate the drive powers from the current heading, goal heading, and goal speed.
    m_stDrivePowers = diffdrive::CalculateMotorPowerFromHeading(dGoalSpeed, dGoalHeading, dActualHeading, eKinematicsMethod, *m_pPID);

    return m_stDrivePowers;
}

/******************************************************************************
 * @brief Sets the left and right drive powers of the drive board.
 *
 * @param stDrivePowers - A struct containing info about the desired drive powers.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-21
 ******************************************************************************/
void DriveBoard::SendDrive(diffdrive::DrivePowers& stDrivePowers)
{
    // Limit input values.
    double dLeftSpeed  = std::clamp(stDrivePowers.dLeftDrivePower, -1.0, 1.0);
    double dRightSpeed = std::clamp(stDrivePowers.dRightDrivePower, -1.0, 1.0);

    // Update member variables with new target speeds.
    m_stDrivePowers.dLeftDrivePower  = dLeftSpeed;
    m_stDrivePowers.dRightDrivePower = dRightSpeed;

    // Remap -1.0 - 1.0 range to drive power range defined in constants. This is so that the driveboard/rovecomm can understand our input.
    float fDriveBoardLeftPower  = numops::MapRange(float(dLeftSpeed), -1.0f, 1.0f, constants::DRIVE_MIN_POWER, constants::DRIVE_MAX_POWER);
    float fDriveBoardRightPower = numops::MapRange(float(dRightSpeed), -1.0f, 1.0f, constants::DRIVE_MIN_POWER, constants::DRIVE_MAX_POWER);
    // Limit the power to max and min effort defined in constants.
    fDriveBoardLeftPower  = std::clamp(float(dLeftSpeed), constants::DRIVE_MIN_EFFORT, constants::DRIVE_MAX_EFFORT);
    fDriveBoardRightPower = std::clamp(float(dRightSpeed), constants::DRIVE_MIN_EFFORT, constants::DRIVE_MAX_EFFORT);

    // Construct a RoveComm packet with the drive data.
    rovecomm::RoveCommPacket<float> stPacket;
    stPacket.unDataId    = manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID;
    stPacket.unDataCount = manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT;
    stPacket.eDataType   = manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE;
    stPacket.vData.emplace_back(fDriveBoardLeftPower);
    stPacket.vData.emplace_back(fDriveBoardRightPower);
    // Check if we should send packets to the SIM or board.
    const char* cIPAddress = constants::MODE_SIM ? "127.0.0.1" : manifest::Core::IP_ADDRESS.IP_STR.c_str();
    // Send drive command over RoveComm to drive board.
    globals::g_pRoveCommUDPNode->SendUDPPacket(stPacket, cIPAddress, constants::ROVECOMM_OUTGOING_UDP_PORT);

    // Submit logger message.
    LOG_DEBUG(logging::g_qSharedLogger, "Driving at: ({}, {})", m_stDrivePowers.dLeftDrivePower, m_stDrivePowers.dRightDrivePower);
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
    m_stDrivePowers.dLeftDrivePower  = 0.0;
    m_stDrivePowers.dRightDrivePower = 0.0;

    // Construct a RoveComm packet with the drive data.
    rovecomm::RoveCommPacket<float> stPacket;
    stPacket.unDataId    = manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID;
    stPacket.unDataCount = manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT;
    stPacket.eDataType   = manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE;
    stPacket.vData.emplace_back(m_stDrivePowers.dLeftDrivePower);
    stPacket.vData.emplace_back(m_stDrivePowers.dRightDrivePower);
    // Check if we should send packets to the SIM or board.
    const char* cIPAddress = constants::MODE_SIM ? "127.0.0.1" : manifest::Core::IP_ADDRESS.IP_STR.c_str();
    // Send drive command over RoveComm to drive board.
    globals::g_pRoveCommUDPNode->SendUDPPacket(stPacket, cIPAddress, constants::ROVECOMM_OUTGOING_UDP_PORT);
}

/******************************************************************************
 * @brief Accessor for the current drive powers of the robot.
 *
 * @return diffdrive::DrivePowers - A struct containing the left and right drive power of the drivetrain.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-10-20
 ******************************************************************************/
diffdrive::DrivePowers DriveBoard::GetDrivePowers() const
{
    // Return the current drive powers.
    return m_stDrivePowers;
}
