/******************************************************************************
 * @brief Approaching Marker State Implementation for Autonomy State Machine.
 *
 * @file ApproachingMarkerState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "ApproachingMarkerState.h"
#include "../AutonomyConstants.h"
#include "../AutonomyGlobals.h"
#include "../AutonomyNetworking.h"
#include "../util/states/TagDetectionChecker.hpp"

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief This method is called when the state is first started. It is used to
     *        initialize the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void ApproachingMarkerState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Scheduling next run of state logic.");

        // Initialize member variables.
        m_stGoalWaypoint = globals::g_pWaypointHandler->PeekNextWaypoint();
        m_nTargetTagID   = m_stGoalWaypoint.nID;

        // Store the state that got stuck and triggered a MarkerSeen event.
        m_eTriggeringState = globals::g_pStateMachineHandler->GetPreviousState();

        // Get tag detectors.
        m_vTagDetectors = {globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam),
                           globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eFrameLeftCam),
                           globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eFrameRightCam)};
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void ApproachingMarkerState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Exiting state.");
    }

    /******************************************************************************
     * @brief Accessor for the State private member. Returns the state as a string.
     *
     * @return std::string - The current state as a string.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    ApproachingMarkerState::ApproachingMarkerState() : State(States::eApproachingMarker)
    {
        LOG_INFO(logging::g_qConsoleLogger, "Entering State: {}", ToString());

        m_bInitialized  = false;

        m_StuckDetector = statemachine::TimeIntervalBasedStuckDetector(constants::STUCK_CHECK_ATTEMPTS,
                                                                       constants::STUCK_CHECK_INTERVAL,
                                                                       constants::STUCK_CHECK_VEL_THRESH,
                                                                       constants::STUCK_CHECK_ROT_THRESH);
        if (!m_bInitialized)
        {
            Start();
            m_bInitialized = true;
        }
    }

    /******************************************************************************
     * @brief Run the state machine. Returns the next state.
     *
     * @author sam_hajdukiewicz (samanthahajdukiewicz@gmail.com) :3
     * @date 2024-01-17
     ******************************************************************************/
    void ApproachingMarkerState::Run()
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "ApproachingMarkerState: Running state-specific behavior.");

        // Get the current rover pose.
        geoops::RoverPose stCurrentRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();

        // Check Rover radius from marker waypoint.
        geoops::GeoMeasurement stCurrentMeasurement = geoops::CalculateGeoMeasurement(m_stGoalWaypoint.GetGPSCoordinate(), stCurrentRoverPose.GetGPSCoordinate());
        if (stCurrentMeasurement.dDistanceMeters > m_stGoalWaypoint.dRadius)
        {
            globals::g_pStateMachineHandler->HandleEvent(Event::eMarkerUnseen);
            return;
        }

        // Identify target marker.
        tagdetectutils::ArucoTag stBestArucoTag, stBestTorchTag;
        statemachine::IdentifyTargetMarker(m_vTagDetectors, stBestArucoTag, stBestTorchTag, m_nTargetTagID);

        // Check if both tag types are unseen.
        static std::chrono::system_clock::time_point tLastSeenTime = std::chrono::system_clock::now();
        if (stBestArucoTag.nID == -1 && stBestTorchTag.dConfidence == 0.0)
        {
            auto tCurrentTime = std::chrono::system_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(tCurrentTime - tLastSeenTime).count() > constants::APPROACH_MARKER_LOST_GIVE_UP_TIME)
            {
                globals::g_pStateMachineHandler->HandleEvent(Event::eMarkerUnseen);
                return;
            }
        }
        else
        {
            // Reset the last seen time if a tag is detected.
            tLastSeenTime = std::chrono::system_clock::now();
        }

        // Create instance variables.
        double dHeadingSetPoint = 0.0;
        double dDistanceFromTag = 9999;
        // Check if we got a good OpenCV tag.
        if (stBestArucoTag.nID != -1)
        {
            dHeadingSetPoint = numops::InputAngleModulus(stBestArucoTag.dYawAngle + stCurrentRoverPose.GetCompassHeading(), 0.0, 359.9);
            dDistanceFromTag = stBestArucoTag.dStraightLineDistance;
        }
        // Check if we got a good Torch tag.
        else if (stBestTorchTag.dConfidence != 0.0)
        {
            dHeadingSetPoint = numops::InputAngleModulus(stBestTorchTag.dYawAngle + stCurrentRoverPose.GetCompassHeading(), 0.0, 359.9);
            dDistanceFromTag = stBestTorchTag.dStraightLineDistance;
        }

        // Move the rover to the target's estimated position.
        diffdrive::DrivePowers stDrivePowers = globals::g_pDriveBoard->CalculateMove(constants::APPROACH_MARKER_MOTOR_POWER,
                                                                                     dHeadingSetPoint,
                                                                                     stCurrentRoverPose.GetCompassHeading(),
                                                                                     diffdrive::DifferentialControlMethod::eArcadeDrive);
        globals::g_pDriveBoard->SendDrive(stDrivePowers);

        // Check if tag is reached.
        if (dDistanceFromTag < constants::APPROACH_MARKER_VISION_DISTANCE)
        {
            // Submit logger message.
            LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Rover has reached the target marker!");
            // Handle state transition and save the current search pattern state.
            globals::g_pStateMachineHandler->HandleEvent(Event::eReachedMarker);
            // Don't execute the rest of the state.
            return;
        }

        //////////////////////////////////////////
        /* ---  Check if the rover is stuck --- */
        //////////////////////////////////////////

        // Check if stuck.
        if (m_StuckDetector.CheckIfStuck(globals::g_pWaypointHandler->SmartRetrieveVelocity(), globals::g_pWaypointHandler->SmartRetrieveAngularVelocity()))
        {
            // Submit logger message.
            LOG_NOTICE(logging::g_qSharedLogger, "NavigatingState: Rover has become stuck!");
            // Handle state transition and save the current search pattern state.
            globals::g_pStateMachineHandler->HandleEvent(Event::eStuck, true);
            // Don't execute the rest of the state.
            return;
        }

        return;
    }

    /******************************************************************************
     * @brief Trigger an event in the state machine. Returns the next state.
     *
     * @param eEvent - The event to trigger.
     * @return std::shared_ptr<State> - The next state.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    States ApproachingMarkerState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eApproachingMarker;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eReachedMarker:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Handling ReachedMarker event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);
                // Pop old waypoint out of queue.
                globals::g_pWaypointHandler->PopNextWaypoint();
                // Clear saved search pattern state.
                globals::g_pStateMachineHandler->ClearSavedState(States::eSearchPattern);
                // Submit logger message.
                LOG_NOTICE(logging::g_qSharedLogger, "ApproachingMarkerState: Cleared old search pattern state from saved states.");
                // Change states.
                eNextState = States::eIdle;
                break;
            }
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eMarkerUnseen:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Handling MarkerUnseen event.");
                // Change states.
                eNextState = m_eTriggeringState;
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "ApproachingMarkerState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eApproachingMarker)
        {
            LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
