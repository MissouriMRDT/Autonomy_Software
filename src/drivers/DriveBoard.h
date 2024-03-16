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

#include "../algorithms/DifferentialDrive.hpp"

/// \cond
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
    private:
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////

        diffdrive::DrivePowers m_stDrivePowers;    // Struct used to store the left and right drive powers of the robot.
        controllers::PIDController* m_pPID;        // The PID controller used for drive towards a heading.
        float m_fMinDriveEffort;                   // The min power limit of the drive. This can be adjusted through RoveComm.
        float m_fMaxDriveEffort;                   // The max power limit of the drive. This can be adjusted through RoveComm.
        std::shared_mutex m_muDriveEffortMutex;    // Mutex used for changing the drive efforts.

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

            // Call class method to update max speed.
            this->SetMaxDriveEffort(stPacket.vData[0]);

            // Submit logger message.
            LOG_DEBUG(logging::g_qSharedLogger, "Incoming SETMAXSPEED: {}", stPacket.vData[0]);
        };

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
                                             const diffdrive::DifferentialControlMethod eKinematicsMethod);
        void SendDrive(diffdrive::DrivePowers& stDrivePowers);
        void SendStop();

        /////////////////////////////////////////
        // Setters
        /////////////////////////////////////////
        void SetMaxDriveEffort(const float fMaxDriveEffortMultiplier);

        /////////////////////////////////////////
        // Getters
        /////////////////////////////////////////

        diffdrive::DrivePowers GetDrivePowers() const;
};
#endif
