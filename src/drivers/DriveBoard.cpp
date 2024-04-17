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
#include "../AutonomyNetworking.h"

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
    m_fMinDriveEffort                = constants::DRIVE_MIN_EFFORT;
    m_fMaxDriveEffort                = constants::DRIVE_MAX_EFFORT;

    // Configure PID controller for heading hold function.
    m_pPID = new controllers::PIDController(constants::DRIVE_PID_PROPORTIONAL,
                                            constants::DRIVE_PID_INTEGRAL,
                                            constants::DRIVE_PID_DERIVATIVE,
                                            constants::DRIVE_PID_FEEDFORWARD);
    m_pPID->SetMaxSetpointDifference(constants::DRIVE_PID_MAX_ERROR_PER_ITER);
    m_pPID->SetMaxIntegralEffort(constants::DRIVE_PID_MAX_INTEGRAL_TERM);
    m_pPID->SetOutputLimits(1.0);    // Autonomy internally always uses -1.0, 1.0 for turning and drive powers.
    m_pPID->SetOutputRampRate(constants::DRIVE_PID_MAX_RAMP_RATE);
    m_pPID->SetOutputFilter(constants::DRIVE_PID_OUTPUT_FILTER);
    m_pPID->SetMaxSetpointDifference(constants::DRIVE_PID_TOLERANCE);
    m_pPID->SetDirection(constants::DRIVE_PID_OUTPUT_REVERSED);
    m_pPID->EnableContinuousInput(0, 360);

    // Set RoveComm callbacks.
    network::g_pRoveCommUDPNode->AddUDPCallback<float>(SetMaxSpeedCallback, manifest::Autonomy::COMMANDS.find("SETMAXSPEED")->second.DATA_ID);
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
    m_stDrivePowers = diffdrive::CalculateMotorPowerFromHeading(dGoalSpeed,
                                                                dGoalHeading,
                                                                dActualHeading,
                                                                eKinematicsMethod,
                                                                *m_pPID,
                                                                constants::DRIVE_SQUARE_CONTROL_INPUTS,
                                                                constants::DRIVE_CURVATURE_KINEMATICS_ALLOW_TURN_WHILE_STOPPED);

    return m_stDrivePowers;
}

/******************************************************************************
 * @brief Sets the left and right drive powers of the drive board.
 *
 * @param stDrivePowers - A struct containing info about the desired drive powers.
 *              Drive powers are always in between -1.0 and 1.0 no matter what constants
 *              or RoveComm say. the -1.0 to 1.0 range is automatically mapped to the
 *              correct DriveBoard range in this method.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-21
 ******************************************************************************/
void DriveBoard::SendDrive(diffdrive::DrivePowers& stDrivePowers)
{
    // Limit input values.
    double dLeftSpeed  = std::clamp(stDrivePowers.dLeftDrivePower, -1.0, 1.0);
    double dRightSpeed = std::clamp(stDrivePowers.dRightDrivePower, -1.0, 1.0);

    // Remap -1.0 - 1.0 range to drive power range defined in constants. This is so that the driveboard/rovecomm can understand our input.
    float fDriveBoardLeftPower  = numops::MapRange(float(dLeftSpeed), -1.0f, 1.0f, m_fMinDriveEffort, m_fMaxDriveEffort);
    float fDriveBoardRightPower = numops::MapRange(float(dRightSpeed), -1.0f, 1.0f, m_fMinDriveEffort, m_fMaxDriveEffort);
    // Limit the power to max and min effort defined in constants.
    fDriveBoardLeftPower  = std::clamp(float(fDriveBoardLeftPower), constants::DRIVE_MIN_POWER, constants::DRIVE_MAX_POWER);
    fDriveBoardRightPower = std::clamp(float(fDriveBoardRightPower), constants::DRIVE_MIN_POWER, constants::DRIVE_MAX_POWER);

    // Update member variables with new target speeds.
    m_stDrivePowers.dLeftDrivePower  = fDriveBoardLeftPower;
    m_stDrivePowers.dRightDrivePower = fDriveBoardRightPower;

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
    network::g_pRoveCommUDPNode->SendUDPPacket(stPacket, cIPAddress, constants::ROVECOMM_OUTGOING_UDP_PORT);

    // Submit logger message.
    LOG_DEBUG(logging::g_qSharedLogger, "Driving at: ({}, {})", fDriveBoardLeftPower, fDriveBoardRightPower);
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
    network::g_pRoveCommUDPNode->SendUDPPacket(stPacket, cIPAddress, constants::ROVECOMM_OUTGOING_UDP_PORT);

    // Submit logger message.
    LOG_DEBUG(logging::g_qSharedLogger, "Sent stop powers to drivetrain");
}

/******************************************************************************
 * @brief Set the max power limits of the drive.
 *
 * @param fMinDriveEffort - A multiplier from 0-1 for the max power output of the drive.
 *              Multiplier will be applied to constants::DRIVE_MIN_POWER and constants::DRIVE_MAX_POWER.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-15
 ******************************************************************************/
void DriveBoard::SetMaxDriveEffort(const float fMaxDriveEffortMultiplier)
{
    // Acquire write lock for writing to max effort member variables.
    std::unique_lock<std::shared_mutex> lkDriveEffortLock(m_muDriveEffortMutex);

    // Update member variables.
    m_fMinDriveEffort = constants::DRIVE_MIN_POWER * fMaxDriveEffortMultiplier;
    m_fMaxDriveEffort = constants::DRIVE_MAX_POWER * fMaxDriveEffortMultiplier;
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
