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

#include "../algorithms/kinematics/DifferentialDrive.hpp"
#include "../util/threading/Publisher.hpp"

/// \cond
#include "../AutonomyConstants.h"
#include <RoveComm/RoveComm.h>
#include <RoveComm/RoveCommManifest.h>
#include <array>
#include <mutex>
#include <shared_mutex>

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
                                             const diffdrive::DifferentialControlMethod eKinematicsMethod = diffdrive::DifferentialControlMethod::eArcadeDrive,
                                             const bool bDriveBackwards                                   = false,
                                             const bool bAlwaysProgressForward                            = false,
                                             const bool bSquareControlInput                               = false,
                                             const bool bCurvatureDriveAllowTurningWhileStopped           = true);
        void SendDrive(const diffdrive::DrivePowers& stDrivePowers, const bool bEnableVariableDriveEffort = true);
        void SendStop();
        float VariableDriveEffort();

        /////////////////////////////////////////
        // Setters
        /////////////////////////////////////////

        void SetMaxDriveEffort(const float fMaxDriveEffortMultiplier);

        /////////////////////////////////////////
        // Getters
        /////////////////////////////////////////

        diffdrive::DrivePowers GetDrivePowers() const;
        float GetMaxDriveEffort() const;

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        diffdrive::DrivePowers m_stDrivePowers;                // Struct used to store the left and right drive powers of the robot.
        std::unique_ptr<controllers::PIDController> m_pPID;    // The PID controller used for drive towards a heading.
        float m_fDriveEffortMultiplier;                        // The current drive effort multiplier. This is adjusted over RoveComm.
        mutable std::shared_mutex m_muDriveEffortMutex;        // Mutex used for changing the drive efforts.
        const float m_fMinSlope = constants::DRIVE_BOARD_MIN_SLOPE;
        const float m_fMaxSlope = constants::DRIVE_BOARD_MAX_SLOPE;
        const float m_fMinDamp  = constants::DRIVE_BOARD_MIN_DAMP;
        const float m_fMaxDamp  = constants::DRIVE_BOARD_MAX_DAMP;
        const float m_fRoll_w   = constants::DRIVE_BOARD_ROLL_WEIGHT;
        const float m_fPitch_w  = constants::DRIVE_BOARD_PITCH_WEIGHT;
        const float m_fYaw_w    = constants::DRIVE_BOARD_YAW_WEIGHT;

        // Persistent demand for the main camera's sensor data, taken on the first call to
        // VariableDriveEffort(). The camera only retrieves and publishes sensor data while a
        // Subscription is alive; this member holds ours for the lifetime of the DriveBoard.
        std::once_flag m_ocSensorReaderOnce;
        pubsub::Reader<sl::SensorsData> m_rdMainCamSensors;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////
};
#endif
