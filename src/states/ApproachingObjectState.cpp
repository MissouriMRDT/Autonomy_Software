/******************************************************************************
 * @brief Approaching Object State Implementation for Autonomy State Machine.
 *
 * @file ApproachingObjectState.cpp
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "ApproachingObjectState.h"
#include "../AutonomyConstants.h"
#include "../AutonomyGlobals.h"
#include "../AutonomyNetworking.h"
#include "../util/states/ObjectDetectionChecker.hpp"

/******************************************************************************
 * @brief Namespace containing all state machine related classes.
 *
 * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
 * @date 2024-01-17
 ******************************************************************************/
namespace statemachine
{
    /******************************************************************************
     * @brief This method is called when the state is first started. It is used to
     * initialize the state.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void ApproachingObjectState::Start()
    {
        // Schedule the next run of the state's logic.
        LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Scheduling next run of state logic.");

        // Initialize target parameters.
        m_stGoalWaypoint   = globals::g_pWaypointHandler->PeekNextWaypoint();
        m_eTriggeringState = globals::g_pStateMachineHandler->GetPreviousState();

        // Schedule the next run of the state's logic.
        LOG_INFO(logging::g_qSharedLogger,
                 "ApproachingObjectState: Started. Goal Waypoint -> Lat: {:.6f}, Lon: {:.6f}, Radius: {:.2f}m. Previous State: {}",
                 m_stGoalWaypoint.GetGPSCoordinate().dLatitude,
                 m_stGoalWaypoint.GetGPSCoordinate().dLongitude,
                 m_stGoalWaypoint.dRadius,
                 StateToString(m_eTriggeringState));

        // Fetch detectors.
        m_vObjectDetectors = {globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eHeadMainCam),
                              globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eRearCam)};

        LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Fetched {} object detectors for tracking.", m_vObjectDetectors.size());

        // Reset all persistent tracking variables for a clean slate.
        m_dHeadingSetPoint                  = 0.0;
        m_bDriveBackwards                   = false;
        m_bHasLastGeolocatedPosition        = false;
        m_bHasSeenTarget                    = false;
        m_bAlreadyPrintedLost               = false;
        m_bAlreadyPrintedVisualLostFallback = false;
        m_tmLastSeenTime                    = std::chrono::system_clock::now();
        m_tmLastLogTime                     = std::chrono::system_clock::now();
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     * the state.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void ApproachingObjectState::Exit()
    {
        // Clean up the state before exiting.
        LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Exiting state.");
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    ApproachingObjectState::ApproachingObjectState() : State(States::eApproachingObject)
    {
        LOG_INFO(logging::g_qConsoleLogger, "Entering State: {}", ToString());

        m_bInitialized  = false;

        m_StuckDetector = statemachine::TimeIntervalBasedStuckDetector(constants::APPROACH_OBJECT_STUCK_CHECK_ATTEMPTS, constants::APPROACH_OBJECT_STUCK_CHECK_INTERVAL);

        if (!m_bInitialized)
        {
            Start();
            m_bInitialized = true;
        }
    }

    /******************************************************************************
     * @brief Run the state machine. Returns the next state.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void ApproachingObjectState::Run()
    {
        /******************************************************************************
         * STATE LOGIC FLOW:
         * 1. Geofence Check: Verify the rover is within the goal waypoint's radius.
         * 2. Target Identification: Find the closest target across all cameras.
         * 3. High Confidence Check: If object is highly confident, STOP and verify.
         * 4. Navigation Decision Tree: Track low-confidence hits or fallbacks.
         * 5. Execution: Send drive commands and run stuck detection.
         ******************************************************************************/

        LOG_DEBUG(logging::g_qSharedLogger, "ApproachingObjectState: Running state-specific behavior.");

        // Get the current rover pose and add to plot.
        geoops::RoverPose stCurrentRoverPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();

        // 1. Boundary Check: Verify rover radius from object waypoint.
        geoops::GeoMeasurement stCurrentMeasurement = geoops::CalculateGeoMeasurement(m_stGoalWaypoint.GetGPSCoordinate(), stCurrentRoverPose.GetGPSCoordinate());
        if (stCurrentMeasurement.dDistanceMeters > m_stGoalWaypoint.dRadius + 5)
        {
            LOG_WARNING(logging::g_qSharedLogger,
                        "ApproachingObjectState: Rover broke geofence! Radius threshold is {} m, current distance is {:.2f} m. Triggering ObjectUnseen.",
                        m_stGoalWaypoint.dRadius,
                        stCurrentMeasurement.dDistanceMeters);
            globals::g_pStateMachineHandler->HandleEvent(Event::eObjectUnseen);
            return;
        }

        // 2. Identify target object.
        objectdetectutils::Object stBestObject;
        statemachine::IdentifyTargetObject(m_vObjectDetectors, stBestObject, m_stGoalWaypoint.eType);

        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();
        double dSecondsSinceLastSeen                        = std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - m_tmLastSeenTime).count() / 1000.0;

        // Clear stale session data if re-entered after a long time.
        if (dSecondsSinceLastSeen > constants::APPROACH_OBJECT_LOST_GIVE_UP_TIME + 5.0)
        {
            m_bHasLastGeolocatedPosition = false;
        }

        // 3. Object Detected -> IMMEDIATE STOP AND VERIFY (No distance check!)
        if (stBestObject.dConfidence > 0.0)
        {
            LOG_NOTICE(logging::g_qSharedLogger, "ApproachingObjectState: OBJECT DETECTED ({:.2f}%). Halting to verify!", stBestObject.dConfidence * 100.0);

            globals::g_pDriveBoard->SendStop();
            globals::g_pStateMachineHandler->HandleEvent(Event::eReachedObject, true);
            return;
        }

        // 4. Object unseen fallback logic
        if (dSecondsSinceLastSeen > constants::APPROACH_OBJECT_LOST_GIVE_UP_TIME)
        {
            LOG_WARNING(logging::g_qSharedLogger,
                        "ApproachingObjectState: Object has been unseen for {:.2f}s (Threshold: {:.2f}s). Giving up and triggering ObjectUnseen.",
                        dSecondsSinceLastSeen,
                        constants::APPROACH_OBJECT_LOST_GIVE_UP_TIME);
            globals::g_pStateMachineHandler->HandleEvent(Event::eObjectUnseen);
            return;
        }

        if (!m_bAlreadyPrintedLost)
        {
            m_bAlreadyPrintedLost = true;
            LOG_WARNING(logging::g_qSharedLogger, "ApproachingObjectState: Tracking lost! No valid objects detected across any camera.");
        }

        // Fallback 1: Drive to last known geolocated position if available.
        if (m_bHasLastGeolocatedPosition)
        {
            geoops::GeoMeasurement stLastMeasurement =
                geoops::CalculateGeoMeasurement(stCurrentRoverPose.GetUTMCoordinate(), m_stLastGeolocatedPosition.GetUTMCoordinate());
            m_dHeadingSetPoint = stLastMeasurement.dStartRelativeBearing;

            if (!m_bAlreadyPrintedVisualLostFallback)
            {
                LOG_NOTICE(logging::g_qSharedLogger,
                           "ApproachingObjectState: [FALLBACK 1] Visual lost. Driving to last known geolocated UTM: [{:.2f}E, {:.2f}N]. Target Bearing: {:.2f} degrees",
                           m_stLastGeolocatedPosition.GetUTMCoordinate().dEasting,
                           m_stLastGeolocatedPosition.GetUTMCoordinate().dNorthing,
                           m_dHeadingSetPoint);
                m_bAlreadyPrintedVisualLostFallback = true;
            }
        }
        // Fallback 2: Coast along last known heading.
        else if (m_bHasSeenTarget)
        {
            if (!m_bAlreadyPrintedVisualLostFallback)
            {
                LOG_NOTICE(logging::g_qSharedLogger,
                           "ApproachingObjectState: [FALLBACK 2] Visual lost & no geolocation history. Coasting along last known heading. Target Bearing: {:.2f} degrees",
                           m_dHeadingSetPoint);
                m_bAlreadyPrintedVisualLostFallback = true;
            }
        }
        // Wait in place: We have never seen the object.
        else
        {
            globals::g_pDriveBoard->SendStop();
            return;
        }

        // 5. Execute drive command based on the best available information and run stuck detection.
        diffdrive::DrivePowers stDrivePowers = globals::g_pDriveBoard->CalculateMove(constants::APPROACH_OBJECT_MOTOR_POWER,
                                                                                     m_dHeadingSetPoint,
                                                                                     stCurrentRoverPose.GetCompassHeading(),
                                                                                     diffdrive::DifferentialControlMethod::eArcadeDrive,
                                                                                     m_bDriveBackwards,    // Reverse control flag.
                                                                                     false,
                                                                                     false,
                                                                                     false);
        globals::g_pDriveBoard->SendDrive(stDrivePowers);

        // Check if the rover is stuck.
        if (constants::APPROACH_OBJECT_ENABLE_STUCK_DETECT &&
            m_StuckDetector.CheckIfStuck(globals::g_pStateMachineHandler->SmartRetrieveVelocity(),
                                         globals::g_pStateMachineHandler->SmartRetrieveAngularVelocity(),
                                         constants::APPROACH_OBJECT_STUCK_CHECK_VEL_THRESH * globals::g_pDriveBoard->GetMaxDriveEffort(),
                                         constants::APPROACH_OBJECT_STUCK_CHECK_ROT_THRESH))
        {
            LOG_WARNING(logging::g_qSharedLogger,
                        "ApproachingObjectState: Rover has become stuck! Triggering Stuck event. Curr Vel: {:.2f}, Ang Vel: {:.2f}",
                        globals::g_pStateMachineHandler->SmartRetrieveVelocity(),
                        globals::g_pStateMachineHandler->SmartRetrieveAngularVelocity());
            globals::g_pStateMachineHandler->HandleEvent(Event::eStuck, true);
            return;
        }
    }

    /******************************************************************************
     * @brief Trigger an event in the state machine. Returns the next state.
     *
     * @param eEvent - The event to trigger.
     * @return std::shared_ptr<State> - The next state.
     *
     * @author Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    States ApproachingObjectState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eApproachingObject;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eReachedObject:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Handling ReachedObject event.");

                if (constants::APPROACH_OBJECT_VERIFY_POSITION)
                {
                    eNextState = States::eVerifyingObject;
                }
                else
                {
                    globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);
                    globals::g_pWaypointHandler->PopNextWaypoint();
                    globals::g_pStateMachineHandler->ClearSavedStates();
                    LOG_NOTICE(logging::g_qSharedLogger, "ApproachingObjectState: Cleared old search pattern state and approaching object state from saved states.");
                    eNextState = States::eIdle;
                }
                break;
            }
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Handling Start event.");
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eObjectUnseen:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Handling ObjectUnseen event.");
                // Change states.
                eNextState = m_eTriggeringState;
                break;
            }
            case Event::eStuck:
            {
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Stuck event.");
                eNextState = States::eStuck;
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eOff);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "ApproachingObjectState: Handling unknown event ({}), defaulting to Idle.", static_cast<int>(eEvent));
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eApproachingMarker)
        {
            LOG_INFO(logging::g_qSharedLogger, "ApproachingObjectState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state.
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
