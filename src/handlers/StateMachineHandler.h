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
#include "../util/threading/CommandQueue.hpp"
#include "./CameraHandler.h"

/// \cond
#include <RoveComm/RoveComm.h>
#include <RoveComm/RoveCommManifest.h>
#include <atomic>
#include <thread>
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
        // THREADING MODEL.
        //
        // m_pCurrentState is owned by the state machine thread and touched ONLY by it. Foreign
        // threads (the RoveComm receive threads, main(), the visualization handler) never
        // mutate it; they Post() an event onto m_cmdQueue and the state machine drains that
        // queue at the top of its own loop.
        //
        // This replaces a shared_ptr that the state machine thread dereferenced with no lock
        // at all while RoveComm callbacks reassigned it under m_muStateMutex - a data race on
        // the pointer pair itself (only the control block refcount is atomic), and one an
        // m_bSwitchingStates flag could not fix, because the check and the Run() call were
        // never atomic with respect to the swap.
        //
        // The published state enum stays separately readable, because GetCurrentState() is
        // called from every thread in the system and must not require the state lock.
        std::shared_ptr<statemachine::State> m_pCurrentState;
        std::shared_ptr<statemachine::State> m_pPreviousState;
        std::unordered_map<statemachine::States, std::shared_ptr<statemachine::State>> m_umSavedStates;

        // Control channel in: foreign threads Post() events here and the state machine thread
        // drains them on itself, so all state transitions happen on one thread by construction.
        CommandQueue m_cmdQueue;
        // The current state, published as a plain enum for lock-free reads from any thread.
        std::atomic<statemachine::States> m_aeCurrentState{statemachine::States::eIdle};
        // The previous state, published as a plain enum for the same reason as m_aeCurrentState:
        // GetPreviousState() is called from foreign threads, and dereferencing m_pPreviousState
        // from one raced the state machine thread reassigning it during a transition.
        std::atomic<statemachine::States> m_aePreviousState{statemachine::States::eIdle};
        // Identity of the state machine thread, so HandleEvent() can tell a state
        // transitioning itself (apply inline) from a foreign thread posting an event (queue).
        std::atomic<std::thread::id> m_stStateMachineThreadId{};
        std::shared_ptr<ZEDCamera> m_pMainCam;

        // Persistent demand for the main camera's sensor data. The camera only calls into the SDK
        // for sensor data while a Reader is alive, so this handler holds one for its
        // lifetime and then reads the newest snapshot with a lock-free, non-blocking Get().
        pubsub::Reader<sl::SensorsData> m_rdMainCamSensors;
        // Guards m_umSavedStates and m_pPreviousState. Both are written by the state machine
        // thread during a transition AND by ClearSavedStates(), which is reachable from a
        // RoveComm thread. m_pCurrentState deliberately is NOT in this group - it is owned by
        // the state machine thread alone and needs no lock.
        std::shared_mutex m_muSavedStatesMutex;
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
        void HandleEventOnOwningThread(statemachine::Event eEvent, const bool bSaveCurrentState);
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
