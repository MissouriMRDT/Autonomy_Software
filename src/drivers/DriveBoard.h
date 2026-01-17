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

/// \cond
#include "../AutonomyConstants.h"
#include <RoveComm/RoveComm.h>
#include <RoveComm/RoveCommManifest.h>
#include <array>
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
                                             const bool bAlwaysProgressForward                            = false);
        void SendDrive(const diffdrive::DrivePowers& stDrivePowers);
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

    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        diffdrive::DrivePowers m_stDrivePowers;                // Struct used to store the left and right drive powers of the robot.
        std::unique_ptr<controllers::PIDController> m_pPID;    // The PID controller used for drive towards a heading.
        float m_fMinDriveEffort;                               // The min power limit of the drive.
        float m_fMaxDriveEffort;                               // The max power limit of the drive.
        float m_fDriveEffortMultiplier;                        // The current drive effort multiplier. This is adjusted over RoveComm.
        std::shared_mutex m_muDriveEffortMutex;                // Mutex used for changing the drive efforts.
        const float m_fMinSlope = constants::DRIVE_BOARD_MIN_SLOPE;
        const float m_fMaxSlope = constants::DRIVE_BOARD_MAX_SLOPE;
        const float m_fMinDamp  = constants::DRIVE_BOARD_MIN_DAMP;
        const float m_fMaxDamp  = constants::DRIVE_BOARD_MAX_DAMP;
        const float m_fRoll_w   = constants::DRIVE_BOARD_ROLL_WEIGHT;
        const float m_fPitch_w  = constants::DRIVE_BOARD_PITCH_WEIGHT;
        const float m_fYaw_w    = constants::DRIVE_BOARD_YAW_WEIGHT;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        /******************************************************************************
         * @brief Callback function that is called whenever RoveComm receives a new SETMAXSPEED packet.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-03-03
         ******************************************************************************/
        const std::function<void(const rovecomm::RoveCommPacket<float>&, const sockaddr_in&)> SetMaxSpeedCallback =
            [this](const rovecomm::RoveCommPacket<float>& stPacket, const sockaddr_in& stdAddr)
        {
            // Not using this.
            (void) stdAddr;

            // Clamp the incoming multiplier to [0.0, 1.0].
            float fClampedMultiplier = std::clamp(std::fabs(stPacket.vData[0]), 0.0f, 1.0f);
            // Update member variable.
            {
                std::unique_lock<std::shared_mutex> lkDriveEffortLock(m_muDriveEffortMutex);
                m_fDriveEffortMultiplier = fClampedMultiplier;
            }

            // Submit logger message.
            LOG_NOTICE(logging::g_qSharedLogger, "Incoming SETMAXSPEED: {}", stPacket.vData[0]);
        };
};
#endif
