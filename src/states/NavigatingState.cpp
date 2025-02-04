/******************************************************************************
 * @brief Navigating State Implementation for Autonomy State Machine.
 *
 * @file NavigatingState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "NavigatingState.h"
#include "../AutonomyGlobals.h"
#include "../AutonomyNetworking.h"

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
    void NavigatingState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Scheduling next run of state logic.");

        // Initialize member variables.
        m_bFetchNewWaypoint          = true;
        m_nMaxDataPoints             = 100;

        m_dStuckCheckLastPosition[0] = 0;
        m_dStuckCheckLastPosition[1] = 0;

        m_vRoverXPosition.reserve(m_nMaxDataPoints);
        m_vRoverYPosition.reserve(m_nMaxDataPoints);

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
    void NavigatingState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Exiting state.");
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    NavigatingState::NavigatingState() : State(States::eNavigating)
    {
        // Submit logger message.
        LOG_INFO(logging::g_qConsoleLogger, "Entering State: {}", ToString());

        // Initialize member variables.
        m_bInitialized  = false;
        m_StuckDetector = statemachine::TimeIntervalBasedStuckDetector(constants::STUCK_CHECK_ATTEMPTS,
                                                                       constants::STUCK_CHECK_INTERVAL,
                                                                       constants::STUCK_CHECK_VEL_THRESH,
                                                                       constants::STUCK_CHECK_ROT_THRESH);
        // Start state.
        if (!m_bInitialized)
        {
            Start();
            m_bInitialized = true;
        }
    }

    /******************************************************************************
     * @brief Run the state machine. Returns the next state.
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void NavigatingState::Run()
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Running state-specific behavior.");

        /*
            The overall flow of this state is as follows.
            1. Navigate to goal waypoint.
            1. Is there a tag -> MarkerSeen
            2. Is there an object -> ObjectSeen
            3. Is there an obstacle -> TBD
            4. Is the rover stuck -> Stuck
        */

        ///////////////////////////////////////
        /* --- Navigate to goal waypoint --- */
        ///////////////////////////////////////

        // Check if we should get a new goal waypoint and that the waypoint handler has one for us.
        if (m_bFetchNewWaypoint && globals::g_pWaypointHandler->GetWaypointCount() > 0)
        {
            // Trigger new waypoint event.
            globals::g_pStateMachineHandler->HandleEvent(Event::eNewWaypoint, true);
            return;
        }

        // Get Current rover pose.
        geoops::RoverPose stCurrentRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();
        // Calculate distance and bearing from goal waypoint.
        geoops::GeoMeasurement stGoalWaypointMeasurement = geoops::CalculateGeoMeasurement(stCurrentRoverPose.GetUTMCoordinate(), m_stGoalWaypoint.GetUTMCoordinate());

        // Only print out every so often.
        static bool bAlreadyPrinted = false;
        if ((std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() % 5) == 0 && !bAlreadyPrinted)
        {
            // Get raw Navboard GPS position.
            geoops::GPSCoordinate stCurrentGPSPosition = globals::g_pNavigationBoard->GetGPSData();
            // Calculate error between pose and GPS.
            geoops::GeoMeasurement stErrorMeasurement = geoops::CalculateGeoMeasurement(stCurrentRoverPose.GetGPSCoordinate(), stCurrentGPSPosition);

            // Assemble the error metrics into a single string. We are going to include the distance and bearing to the goal waypoint and
            // the error between the rover pose and the GPS position. The rover pose could be from VIO or GNSS fusion, or just GPS.
            std::string szErrorMetrics =
                "--------[ Navigating Error Report ]--------\nDistance to Goal Waypoint: " + std::to_string(stGoalWaypointMeasurement.dDistanceMeters) + " meters\n" +
                "Bearing to Goal Waypoint: " + std::to_string(stGoalWaypointMeasurement.dStartRelativeBearing) + " degrees\n" +
                "GPS/VIO Position Error (UTM for easy reading):\n" + std::to_string(stErrorMeasurement.dDistanceMeters) + " (distance) " +
                std::to_string(stErrorMeasurement.dStartRelativeBearing) + " (bearing)";
            // Submit the error metrics to the logger.
            LOG_INFO(logging::g_qSharedLogger, "{}", szErrorMetrics);

            // Set toggle.
            bAlreadyPrinted = true;
        }
        else if ((std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() % 5) != 0 && bAlreadyPrinted)
        {
            // Reset toggle.
            bAlreadyPrinted = false;
        }

        // Check if we are at the goal waypoint.
        if (stGoalWaypointMeasurement.dDistanceMeters > constants::NAVIGATING_REACHED_GOAL_RADIUS)
        {
            // Calculate drive move/powers.
            diffdrive::DrivePowers stDriveSpeeds = globals::g_pDriveBoard->CalculateMove(constants::DRIVE_MAX_POWER,
                                                                                         stGoalWaypointMeasurement.dStartRelativeBearing,
                                                                                         stCurrentRoverPose.GetCompassHeading(),
                                                                                         diffdrive::DifferentialControlMethod::eArcadeDrive);
            // Send drive powers over RoveComm.
            globals::g_pDriveBoard->SendDrive(stDriveSpeeds);
        }
        else
        {
            // Stop drive.
            globals::g_pDriveBoard->SendStop();

            // Check waypoint type.
            switch (m_stGoalWaypoint.eType)
            {
                // Goal waypoint is navigation.
                case geoops::WaypointType::eNavigationWaypoint:
                {
                    // We are at the goal, signal event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedGpsCoordinate, false);
                    break;
                }
                // Goal waypoint is marker.
                case geoops::WaypointType::eTagWaypoint:
                {
                    // We are at the goal, signal event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedMarker, false);
                    break;
                }
                // Goal waypoint is object.
                case geoops::WaypointType::eObjectWaypoint:
                {
                    // We are at the goal, signal event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedObject, false);
                    break;
                }
                // Goal waypoint is object.
                case geoops::WaypointType::eMalletWaypoint:
                {
                    // We are at the goal, signal event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedObject, false);
                    break;
                }
                // Goal waypoint is object.
                case geoops::WaypointType::eWaterBottleWaypoint:
                {
                    // We are at the goal, signal event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedObject, false);
                    break;
                }
                default: break;
            }
        }

        /////////////////////////
        /* --- Detect Tags --- */
        /////////////////////////

        // In order to even care about any tags we see, the goal waypoint needs to be of type MARKER and we need to be within the search radius of the MARKER waypoint.
        if (m_stGoalWaypoint.eType == geoops::WaypointType::eTagWaypoint && stGoalWaypointMeasurement.dDistanceMeters <= m_stGoalWaypoint.dRadius)
        {
            // Get a list of the currently detected tags, and their stats.
            std::vector<arucotag::ArucoTag> vDetectedArucoTags;
            std::vector<tensorflowtag::TensorflowTag> vDetectedTensorflowTags;
            tagdetectutils::LoadDetectedTags(vDetectedArucoTags, vDetectedTensorflowTags, m_vTagDetectors, false);

            // Check if we have detected any tags.
            if (vDetectedArucoTags.size() || vDetectedTensorflowTags.size())
            {
                // Check if any of the tags have a detection counter or confidence greater than the threshold.
                if (std::any_of(vDetectedArucoTags.begin(),
                                vDetectedArucoTags.end(),
                                [this](arucotag::ArucoTag& stTag)
                                {
                                    // If the Tag ID given by the user in the waypoint is less than 0, then we don't care about the ID.
                                    if (m_stGoalWaypoint.nID < 0)
                                    {
                                        return stTag.nHits >= constants::APPROACH_MARKER_DETECT_ATTEMPTS_LIMIT;
                                    }
                                    else
                                    {
                                        return (stTag.nID == m_stGoalWaypoint.nID && stTag.nHits >= constants::APPROACH_MARKER_DETECT_ATTEMPTS_LIMIT);
                                    }
                                }) ||
                    std::any_of(vDetectedTensorflowTags.begin(),
                                vDetectedTensorflowTags.end(),
                                [](tensorflowtag::TensorflowTag& stTag) { return stTag.dConfidence >= constants::APPROACH_MARKER_TF_CONFIDENCE_THRESHOLD; }))
                {
                    // Submit logger message.
                    LOG_NOTICE(logging::g_qSharedLogger, "NavigatingState: Marker seen!");
                    // Handle state transition.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eMarkerSeen);
                    return;
                }
            }
        }

        ////////////////////////////
        /* --- Detect Objects --- */
        ////////////////////////////

        // TODO: Add object detection to SearchPattern state

        //////////////////////////////
        /* --- Detect Obstacles --- */
        //////////////////////////////

        // TODO: Add obstacle detection to SearchPattern state

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
    States NavigatingState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eNavigating;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eNoWaypoint:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling No Waypoint event.");
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            case Event::eReachedGpsCoordinate:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Reached GPS Coordinate event.");
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eVerifyingPosition;
                break;
            }
            case Event::eReachedMarker:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Reached Marker Waypoint event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eSearchPattern;
                break;
            }
            case Event::eReachedObject:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Reached Object Waypoint event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eSearchPattern;
                break;
            }
            case Event::eNewWaypoint:
            {
                // Check if the next goal waypoint equals the current one.
                if (m_stGoalWaypoint == globals::g_pWaypointHandler->PeekNextWaypoint())
                {
                    // Submit logger message.
                    LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Reusing current Waypoint.");
                }
                else
                {
                    // Submit logger message.
                    LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling New Waypoint event.");
                    // Get and store new goal waypoint.
                    m_stGoalWaypoint = globals::g_pWaypointHandler->PeekNextWaypoint();
                }

                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Set toggle.
                m_bFetchNewWaypoint = false;
                break;
            }
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Abort event.");
                // Stop drive.
                globals::g_pDriveBoard->SendStop();
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Set toggle.
                m_bFetchNewWaypoint = true;
                // Change states.
                eNextState = States::eIdle;
                break;
            }
            case Event::eMarkerSeen:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling MarkerSeen event.");
                // Change states.
                eNextState = States::eApproachingMarker;
                break;
            }
            case Event::eObstacleAvoidance:
            {
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Obstacle Avoidance event.");
                eNextState = States::eAvoidance;
                break;
            }
            case Event::eReverse:
            {
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Reverse event.");
                eNextState = States::eReversing;
                break;
            }
            case Event::eStuck:
            {
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling Stuck event.");
                eNextState = States::eStuck;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "NavigatingState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eNavigating)
        {
            LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
