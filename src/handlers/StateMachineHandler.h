/******************************************************************************
 * @brief Defines the StateMachineHandler class.
 *
 * @file StateMachineHandler.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef STATEMACHINEHANDLER_H
#define STATEMACHINEHANDLER_H

#include "../states/ApproachingMarkerState.h"
#include "../states/ApproachingObjectState.h"
#include "../states/IdleState.h"
#include "../states/NavigatingState.h"
#include "../states/ReversingState.h"
#include "../states/SearchPatternState.h"
#include "../states/StuckState.h"
#include "../states/VerifyingMarkerState.h"
#include "../states/VerifyingObjectState.h"
#include "../states/VerifyingPositionState.h"
#include "./CameraHandler.h"

/// \cond
#include <RoveComm/RoveComm.h>
#include <RoveComm/RoveCommManifest.h>
#include <atomic>
#include <shared_mutex>
#include <tracy/Tracy.hpp>

/// \endcond

/******************************************************************************
 * @brief The StateMachineHandler class serves as the main state machine for
 *        Autonomy Software. It will handle all state transitions and run the
 *        logic for each state.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
class StateMachineHandler : private AutonomyThread<void>
{
    private:
        /////////////////////////////////////////
        // Declare private class member variables.
        /////////////////////////////////////////
        std::shared_ptr<statemachine::State> m_pCurrentState;
        std::shared_ptr<statemachine::State> m_pPreviousState;
        std::unordered_map<statemachine::States, std::shared_ptr<statemachine::State>> m_umSavedStates;
        std::shared_mutex m_muStateMutex;
        std::shared_mutex m_muEventMutex;
        std::atomic_bool m_bSwitchingStates;
        std::shared_ptr<ZEDCamera> m_pMainCam;

        // Persistent demand for the main camera's sensor data. The camera only calls into the SDK
        // for sensor data while a Subscription is alive, so this handler holds one for its
        // lifetime and then reads the newest snapshot with a lock-free, non-blocking Get().
        pubsub::Subscription m_subMainCamSensors;
        std::shared_ptr<ZEDCamera> m_pRearCam;
        geoops::GPSCoordinate m_stCurrentGPSLocation;
        double m_dZEDHeadingOffset;    // This is the offset that is applied to the ZED's heading to align it with the actual heading of the rover.

        // Kinematic tracking variables for heading recovery.
        double m_dLastRawZEDHeading;
        double m_dLastFusedHeading;
        bool m_bFirstHeadingLoop;

        /////////////////////////////////////////
        // Declare private class methods.
        /////////////////////////////////////////
        std::shared_ptr<statemachine::State> CreateState(statemachine::States eState);
        void ChangeState(statemachine::States eNextState, const bool bSaveCurrentState = false);
        void SaveCurrentState();
        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;

        void AutonomyStartCallback(const rovecomm::RoveCommPacket<uint8_t>& stPacket);
        void AutonomyStopCallback(const rovecomm::RoveCommPacket<uint8_t>& stPacket);
        void ClearWaypointsCallback(const rovecomm::RoveCommPacket<uint8_t>& stPacket);
        void PMSCellVoltageCallback(const rovecomm::RoveCommPacket<float>& stPacket);

    public:
        /////////////////////////////////////////
        // Declare public class methods and variables.
        /////////////////////////////////////////
        StateMachineHandler();
        ~StateMachineHandler();
        void StartStateMachine();
        void StopStateMachine();
        void HandleEvent(statemachine::Event eEvent, const bool bSaveCurrentState = false);
        void ClearSavedStates();
        void ClearSavedState(statemachine::States eState);
        statemachine::States GetCurrentState() const;
        statemachine::States GetPreviousState() const;

        // Smart location retrieving.
        geoops::RoverPose SmartRetrieveRoverPose(bool bIMUHeading = true);
        double SmartRetrieveVelocity();
        double SmartRetrieveAngularVelocity();
        void RealignZEDHeading(const double dNewActualHeading, const double dCurrentZEDHeading);

        using AutonomyThread::GetIPS;
};

#endif    // STATEMACHINEHANDLER_H
