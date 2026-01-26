/******************************************************************************
 * @brief Implements the interface for sending commands to the drive board on
 * the Rover.
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
#include "../vision/cameras/ZEDCam.h"

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
    m_fMinDriveEffort                = constants::DRIVE_MIN_POWER;
    m_fMaxDriveEffort                = constants::DRIVE_MAX_POWER;
    m_fDriveEffortMultiplier         = 1.0f;

    // Configure variable drive effort parameters.
    m_pPowerPID = std::make_unique<controllers::PIDController>(constants::DRIVE_PID_PROPORTIONAL,
                                                               constants::DRIVE_PID_INTEGRAL,
                                                               constants::DRIVE_PID_DERIVATIVE,
                                                               constants::DRIVE_PID_FEEDFORWARD);
    m_pPowerPID->SetMaxSetpointDifference(constants::DRIVE_PID_MAX_ERROR);
    m_pPowerPID->SetMaxIntegralEffort(constants::DRIVE_PID_MAX_INTEGRAL_TERM);
    m_pPowerPID->SetOutputLimits(1.0);    // Autonomy internally always uses -1.0, 1.0 for turning and drive powers.
    m_pPowerPID->SetOutputRampRate(constants::DRIVE_PID_MAX_RAMP_RATE);
    m_pPowerPID->SetOutputFilter(constants::DRIVE_PID_OUTPUT_FILTER);
    m_pPowerPID->SetTolerance(constants::DRIVE_PID_TOLERANCE);
    m_pPowerPID->SetDirection(constants::DRIVE_PID_OUTPUT_REVERSED);

    // Configure PID controller for heading hold function.
    m_pSteeringPID = std::make_unique<controllers::PIDController>(constants::DRIVE_PID_PROPORTIONAL,
                                                                  constants::DRIVE_PID_INTEGRAL,
                                                                  constants::DRIVE_PID_DERIVATIVE,
                                                                  constants::DRIVE_PID_FEEDFORWARD);
    m_pSteeringPID->SetMaxSetpointDifference(constants::DRIVE_PID_MAX_ERROR);
    m_pSteeringPID->SetMaxIntegralEffort(constants::DRIVE_PID_MAX_INTEGRAL_TERM);
    m_pSteeringPID->SetOutputLimits(1.0);    // Autonomy internally always uses -1.0, 1.0 for turning and drive powers.
    m_pSteeringPID->SetOutputRampRate(constants::DRIVE_PID_MAX_RAMP_RATE);
    m_pSteeringPID->SetOutputFilter(constants::DRIVE_PID_OUTPUT_FILTER);
    m_pSteeringPID->SetTolerance(constants::DRIVE_PID_TOLERANCE);
    m_pSteeringPID->SetDirection(constants::DRIVE_PID_OUTPUT_REVERSED);
    m_pSteeringPID->EnableContinuousInput(0, 360);

    // Set RoveComm callbacks.
    if (network::g_pRoveCommUDPNode)
    {
        network::g_pRoveCommUDPNode->AddUDPCallback<float>(SetMaxSpeedCallback, manifest::Autonomy::COMMANDS.find("SETMAXSPEED")->second.DATA_ID);
    }
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
 * given heading at a given speed
 *
 * @param dGoalSpeed - The speed to drive at in meters per second.
 * @param dGoalHeading - The angle to drive towards. (0 - 360) 0 is North.
 * @param dActualHeading - The real angle that the Rover is current facing.
 * @param eKinematicsMethod - The kinematics model to use for differential drive control. Enum within DifferentialDrive.hpp
 * @param bAlwaysProgressForward - If true, the rover will always move forward or backward. Point turns will not be allowed.
 * @return diffdrive::DrivePowers - A struct containing two values. (left power, right power)
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-21
 ******************************************************************************/
diffdrive::DrivePowers DriveBoard::CalculateMove(const double dGoalSpeed,
                                                 const double dGoalHeading,
                                                 const double dActualSpeed,
                                                 const double dActualHeading,
                                                 const diffdrive::DifferentialControlMethod eKinematicsMethod,
                                                 const bool bAlwaysProgressForward)
{
    // Calculate the drive powers from the current heading, goal heading, and goal speed.
    diffdrive::DrivePowers stDrivePowers = diffdrive::CalculateMotorPowerFromHeading(dGoalSpeed,
                                                                                     dGoalHeading,
                                                                                     dActualSpeed,
                                                                                     dActualHeading,
                                                                                     eKinematicsMethod,
                                                                                     *m_pPowerPID,
                                                                                     *m_pSteeringPID,
                                                                                     bAlwaysProgressForward,
                                                                                     constants::DRIVE_SQUARE_CONTROL_INPUTS,
                                                                                     constants::DRIVE_CURVATURE_KINEMATICS_ALLOW_TURN_WHILE_STOPPED);

    return stDrivePowers;
}

/******************************************************************************
 * @brief Sets the left and right drive powers of the drive board.
 *
 * @param stDrivePowers - A struct containing info about the desired drive powers.
 * Drive powers are always in between -1.0 and 1.0 no matter what constants
 * or RoveComm say. the -1.0 to 1.0 range is automatically mapped to the
 * correct DriveBoard range in this method.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2023-09-21
 ******************************************************************************/
void DriveBoard::SendDrive(const diffdrive::DrivePowers& stDrivePowers, const bool bEnableVariableDriveEffort)
{
    // Enable or disable variable drive effort.
    if (bEnableVariableDriveEffort)
    {
        float fMultiplier = VariableDriveEffort();
        SetMaxDriveEffort(fMultiplier);
    }

    // Limit input values (-1.0 to 1.0).
    double dLeftInput  = std::clamp(stDrivePowers.dLeftDrivePower, -1.0, 1.0);
    double dRightInput = std::clamp(stDrivePowers.dRightDrivePower, -1.0, 1.0);

    // -------------------------------------------------------------------------
    // Decouple Linear and Angular components to fix low-speed turning.
    // -------------------------------------------------------------------------
    // Separate Linear (Forward/Back) and Angular (Turn) power.
    double dLinearPower  = (dLeftInput + dRightInput) / 2.0;
    double dAngularPower = (dLeftInput - dRightInput) / 2.0;

    // Apply the Speed Multiplier ONLY to the Linear component.
    // This slows down the travel speed but keeps full turning torque available.
    // Use a shared lock to prevent data races when reading the multiplier.
    {
        std::shared_lock<std::shared_mutex> lkDriveEffortLock(m_muDriveEffortMutex);
        dLinearPower *= m_fDriveEffortMultiplier;
    }

    // Reconstruct Left and Right powers.
    double dLeftSpeed  = dLinearPower + dAngularPower;
    double dRightSpeed = dLinearPower - dAngularPower;

    // Desaturate the output to preserve the turning ratio if it exceeds the max.
    // If we commanded (1.0, 0.5) and scaled linear by 0.1, we might get (0.1, 0.05).
    // But if turning adds significant power, we might exceed 1.0.
    double dMaxMagnitude = std::max(std::abs(dLeftSpeed), std::abs(dRightSpeed));
    if (dMaxMagnitude > 1.0)
    {
        dLeftSpeed /= dMaxMagnitude;
        dRightSpeed /= dMaxMagnitude;
    }
    // -------------------------------------------------------------------------

    // If the min and max drive effort have been set to 0, then just send zero powers.
    if (m_fMinDriveEffort != 0.0 || m_fMaxDriveEffort != 0.0)
    {
        // Limit the power to max and min effort defined in constants (Slope Safety).
        m_stDrivePowers.dLeftDrivePower  = std::clamp(float(dLeftSpeed), m_fMinDriveEffort, m_fMaxDriveEffort);
        m_stDrivePowers.dRightDrivePower = std::clamp(float(dRightSpeed), m_fMinDriveEffort, m_fMaxDriveEffort);
    }

    // Construct a RoveComm packet with the drive data.
    rovecomm::RoveCommPacket<float> stPacket;
    stPacket.unDataId    = manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID;
    stPacket.unDataCount = manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT;
    stPacket.eDataType   = manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE;
    stPacket.vData.emplace_back(m_stDrivePowers.dLeftDrivePower);
    stPacket.vData.emplace_back(m_stDrivePowers.dRightDrivePower);
    // Send drive command over RoveComm to drive board.
    if (network::g_pRoveCommUDPNode)
    {
        // Check if we should send packets to the SIM or board.
        const char* cIPAddress = constants::MODE_SIM ? constants::SIM_IP_ADDRESS.c_str() : manifest::Core::IP_ADDRESS.IP_STR.c_str();
        // Send packet.
        network::g_pRoveCommUDPNode->SendUDPPacket(stPacket, cIPAddress, constants::ROVECOMM_OUTGOING_UDP_PORT);
    }
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
    const char* cIPAddress = constants::MODE_SIM ? constants::SIM_IP_ADDRESS.c_str() : manifest::Core::IP_ADDRESS.IP_STR.c_str();
    // Send drive command over RoveComm to drive board.
    if (network::g_pRoveCommUDPNode)
    {
        network::g_pRoveCommUDPNode->SendUDPPacket(stPacket, cIPAddress, constants::ROVECOMM_OUTGOING_UDP_PORT);
    }
    // Submit logger message.
    LOG_DEBUG(logging::g_qSharedLogger, "Sent stop powers to drivetrain");
}

/******************************************************************************
 * @brief This method calculates a multiplier that is applied to
 * SetMaxDriveEffort() to adjust the speed of the rover in relation to the
 * risk of the terrain.
 *
 * @return fMultiplier - A multiplier value between m_fMinDamp and m_fMaxDamp
 *
 * @author Hunter LeRette (hrlnpc@mst.edu), Jordan Hoover (jh69n@mst.edu), Aiden Buter (ab9hm@mst.edu)
 * @date 2026-01-10
 ******************************************************************************/
float DriveBoard::VariableDriveEffort()
{
    // Get pointer to camera.
    std::shared_ptr<ZEDCamera> ExampleZEDCam1 = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam);
    // Declare data structures to store data in.
    sl::SensorsData slSensorData;

    // Put in a request to have our empty sensors data variable filled with the most recent data from the camera.
    std::future<bool> fuCopyStatus = ExampleZEDCam1->RequestSensorsCopy(slSensorData);
    float fMultiplier              = 1;

    // Now we are ready to use the sensors data, let's make sure we have it or wait until we do.
    if (fuCopyStatus.get())
    {
        // Declare roll, pitch, yaw from sensor data
        float fRoll  = fabs(slSensorData.imu.pose.getEulerAngles(false).z);
        float fPitch = fabs(slSensorData.imu.pose.getEulerAngles(false).x);
        float fYaw   = slSensorData.imu.pose.getEulerAngles(false).y;

        // Calculate the risk factor to be applied to the linear polarization equation
        float fTheta = fRoll * (m_fRoll_w) + fPitch * (m_fPitch_w) + fYaw * (m_fYaw_w);

        // Clamp damping based on slope angle: Max damping on flat terrain, Min damping on risky terrain
        if (fTheta <= m_fMinSlope)
            fMultiplier = m_fMaxDamp;
        if (fTheta >= m_fMaxSlope)
            fMultiplier = m_fMinDamp;

        // Calculate multiplier using linear polarization
        const float fK = (m_fMaxDamp - m_fMinDamp) / (m_fMaxSlope - m_fMinSlope);
        float fD       = m_fMaxDamp - fK * (fTheta - m_fMinSlope);

        // Return multiplier
        fMultiplier = std::clamp(fD, m_fMinDamp, m_fMaxDamp);
    }

    return fMultiplier;
}

/******************************************************************************
 * @brief Set the max power limits of the drive.
 *
 * @param fMaxDriveEffortMultiplier - A multiplier from 0-1 for the max power output of the drive.
 * Multiplier will be applied to constants::DRIVE_MIN_POWER and constants::DRIVE_MAX_POWER.
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-15
 ******************************************************************************/
void DriveBoard::SetMaxDriveEffort(const float fMaxDriveEffortMultiplier)
{
    // Clamp the multiplier to the range [0, 1].
    float fClampedMaxDriveEffortMultiplier = std::clamp(fMaxDriveEffortMultiplier, 0.0f, constants::DRIVE_MAX_POWER);

    // Update member variables.
    m_fMinDriveEffort = constants::DRIVE_MIN_POWER * fClampedMaxDriveEffortMultiplier;
    m_fMaxDriveEffort = constants::DRIVE_MAX_POWER * fClampedMaxDriveEffortMultiplier;
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
