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
    m_stDrivePowers.dLeftDrivePower  = 0.0;
    m_stDrivePowers.dRightDrivePower = 0.0;

    // Configure PID controller for heading hold function.
    m_pPID = new PIDController(constants::DRIVE_PID_PROPORTIONAL, constants::DRIVE_PID_INTEGRAL, constants::DRIVE_PID_DERIVATIVE);
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
 * @param dActualHeading -
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

    // Submit logger message.
    LOG_DEBUG(logging::g_qSharedLogger, "Driving at: ({}, {})", m_stDrivePowers.dLeftDrivePower, m_stDrivePowers.dRightDrivePower);

    return m_stDrivePowers;
}

/******************************************************************************
 * @brief Sets the left and right drive powers of the drive board.
 *
 * @param dLeftSpeed - Left drive speed (-1 to 1)
 * @param dRightSpeed - Right drive speed (-1 to 1)
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-21
 ******************************************************************************/
void DriveBoard::SendDrive(double dLeftSpeed, double dRightSpeed)
{
    // Limit input values.
    dLeftSpeed  = std::clamp(dLeftSpeed, -1.0, 1.0);
    dRightSpeed = std::clamp(dRightSpeed, -1.0, 1.0);

    // Update member variables with new target speeds.
    m_stDrivePowers.dLeftDrivePower  = dLeftSpeed;
    m_stDrivePowers.dRightDrivePower = dRightSpeed;

    // TODO: Uncomment once RoveComm is implemented. This is commented to gid rid of unused variable warnings.
    // // Remap -1.0 - 1.0 range to drive power range defined in constants. This is so that the driveboard/rovecomm can understand our input.
    // float fDriveBoardLeftPower  = numops::MapRange(float(dLeftSpeed), -1.0f, 1.0f, constants::DRIVE_MIN_POWER, constants::DRIVE_MAX_POWER);
    // float fDriveBoardRightPower = numops::MapRange(float(dRightSpeed), -1.0f, 1.0f, constants::DRIVE_MIN_POWER, constants::DRIVE_MAX_POWER);
    // // Limit the power to max and min effort defined in constants.
    // fDriveBoardLeftPower  = std::clamp(float(dLeftSpeed), constants::DRIVE_MIN_EFFORT, constants::DRIVE_MAX_EFFORT);
    // fDriveBoardRightPower = std::clamp(float(dRightSpeed), constants::DRIVE_MIN_EFFORT, constants::DRIVE_MAX_EFFORT);

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
    m_stDrivePowers.dLeftDrivePower  = 0.0;
    m_stDrivePowers.dRightDrivePower = 0.0;

    // Send drive command over RoveComm to drive board.
    // TODO: Add RoveComm sendpacket.
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
