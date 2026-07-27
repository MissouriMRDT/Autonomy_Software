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

/// \cond
#include <tracy/Tracy.hpp>

/// \endcond

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
     * initialize the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void ApproachingMarkerState::Start()
    {
        // Schedule the next run of the state's logic.
        LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Scheduling next run of state logic.");

        // Initialize target parameters.
        m_stGoalWaypoint   = globals::g_pWaypointHandler->PeekNextWaypoint();
        m_eTriggeringState = globals::g_pStateMachineHandler->GetPreviousState();

        // Schedule the next run of the state's logic.
        LOG_INFO(logging::g_qSharedLogger,
                 "ApproachingMarkerState: Started. Goal Waypoint -> Lat: {:.6f}, Lon: {:.6f}, Radius: {:.2f}m, Target ID: {}. Previous State: {}",
                 m_stGoalWaypoint.GetGPSCoordinate().dLatitude,
                 m_stGoalWaypoint.GetGPSCoordinate().dLongitude,
                 m_stGoalWaypoint.dRadius,
                 m_stGoalWaypoint.nID,
                 StateToString(m_eTriggeringState));

        // Fetch detectors.
        m_vTagDetectors = {globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam),
                           globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eRearCam)};

        LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Fetched {} tag detectors for tracking.", m_vTagDetectors.size());

        // Reset all persistent tracking variables for a clean slate.
        m_dHeadingSetPoint                  = 0.0;
        m_dDistanceFromTag                  = 0.0;
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
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void ApproachingMarkerState::Exit()
    {
        // Clean up the state before exiting.
        LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Exiting state.");
    }

    /******************************************************************************
     * @brief Accessor for the State private member. Returns the state as a string.
     *
     * @author Eli Byrd (edbgkk@mst.edu), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    ApproachingMarkerState::ApproachingMarkerState() : State(States::eApproachingMarker)
    {
        LOG_INFO(logging::g_qConsoleLogger, "Entering State: {}", ToString());

        m_bInitialized  = false;

        m_StuckDetector = statemachine::TimeIntervalBasedStuckDetector(constants::APPROACH_MARKER_STUCK_CHECK_ATTEMPTS, constants::APPROACH_MARKER_STUCK_CHECK_INTERVAL);

        if (!m_bInitialized)
        {
            Start();
            m_bInitialized = true;
        }
    }

    /******************************************************************************
     * @brief Run the state machine. Returns the next state.
     *
     * @author sam_hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void ApproachingMarkerState::Run()
    {
        ZoneScopedC(tracy::Color::Ivory1);

        /******************************************************************************
         * STATE LOGIC FLOW:
         * 1. Geofence Check: Verify the rover is within the goal waypoint's radius.
         * 2. Target Identification: Find the closest target across all cameras using screen area %.
         * 3. Navigation Decision Tree:
         * -> Target Unseen: Coast on last heading -> Drive to last GPS -> Timeout & Exit.
         * -> Target Detected: Toggle Forward/Reverse based on camera (Front vs Rear).
         * - Good Depth (> 0.0m): Calculate absolute GPS heading and save to history.
         * - Bad Depth (== 0.0m): Fallback to pure relative vision heading (Yaw).
         * 4. Execution: Send drive commands, check proximity for success, and run stuck detection.
         ******************************************************************************/

        LOG_DEBUG(logging::g_qSharedLogger, "ApproachingMarkerState: Running state-specific behavior.");

        // Get the current rover pose and add to plot.
        geoops::RoverPose stCurrentRoverPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();

        // 1. Boundary Check: Verify rover radius from marker waypoint.
        geoops::GeoMeasurement stCurrentMeasurement = geoops::CalculateGeoMeasurement(m_stGoalWaypoint.GetGPSCoordinate(), stCurrentRoverPose.GetGPSCoordinate());
        if (stCurrentMeasurement.dDistanceMeters > m_stGoalWaypoint.dRadius + 5)
        {
            LOG_WARNING(logging::g_qSharedLogger,
                        "ApproachingMarkerState: Rover broke geofence! Radius threshold is {} m, current distance is {:.2f} m. Triggering MarkerUnseen.",
                        m_stGoalWaypoint.dRadius,
                        stCurrentMeasurement.dDistanceMeters);
            globals::g_pStateMachineHandler->HandleEvent(Event::eMarkerUnseen);
            return;
        }

        // 2. Identify target marker.
        tagdetectutils::ArucoTag stBestArucoTag;
        tagdetectutils::ArucoTag stBestTorchTag;    // Used as placeholder for ML torch tag structure
        statemachine::IdentifyTargetMarker(m_vTagDetectors, stBestArucoTag, stBestTorchTag, m_stGoalWaypoint.nID);

        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();
        double dSecondsSinceLastSeen                        = std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - m_tmLastSeenTime).count() / 1000.0;
        bool bMarkerDetected                                = (stBestArucoTag.nID != -1 || stBestTorchTag.dConfidence != 0.0);

        // Clear stale session data if re-entered after a long time.
        if (dSecondsSinceLastSeen > constants::APPROACH_MARKER_LOST_GIVE_UP_TIME + 5.0)
        {
            m_bHasLastGeolocatedPosition = false;
        }

        std::string szCameraOrigin   = "Unknown/None";
        std::string szTagModelSource = "None";
        double dCurrentTagYaw        = 0.0;
        double dCurrentTagConf       = 0.0;

        // 3. Update tracking states.
        if (!bMarkerDetected)
        {
            // Marker unseen.
            if (dSecondsSinceLastSeen > constants::APPROACH_MARKER_LOST_GIVE_UP_TIME)
            {
                LOG_WARNING(logging::g_qSharedLogger,
                            "ApproachingMarkerState: Tag has been unseen for {:.2f}s (Threshold: {:.2f}s). Giving up and triggering MarkerUnseen.",
                            dSecondsSinceLastSeen,
                            constants::APPROACH_MARKER_LOST_GIVE_UP_TIME);
                globals::g_pStateMachineHandler->HandleEvent(Event::eMarkerUnseen);
                return;
            }

            if (!m_bAlreadyPrintedLost)
            {
                m_bAlreadyPrintedLost = true;
                LOG_WARNING(logging::g_qSharedLogger, "ApproachingMarkerState: Tracking lost! No valid markers/tags detected across any camera.");
            }

            // Fallback 1: Drive to last known geolocated position if available.
            if (m_bHasLastGeolocatedPosition)
            {
                geoops::GeoMeasurement stLastMeasurement =
                    geoops::CalculateGeoMeasurement(stCurrentRoverPose.GetUTMCoordinate(), m_stLastGeolocatedPosition.GetUTMCoordinate());
                m_dHeadingSetPoint = stLastMeasurement.dStartRelativeBearing;
                m_dDistanceFromTag = stLastMeasurement.dDistanceMeters;

                if (!m_bAlreadyPrintedVisualLostFallback)
                {
                    LOG_NOTICE(logging::g_qSharedLogger,
                               "ApproachingMarkerState: [FALLBACK 1] Visual lost. Driving to last known geolocated UTM: [{:.2f}E, {:.2f}N]. Current Dist: {:.2f}m, "
                               "Target Bearing: {:.2f} degrees, Reverse: {}",
                               m_stLastGeolocatedPosition.GetUTMCoordinate().dEasting,
                               m_stLastGeolocatedPosition.GetUTMCoordinate().dNorthing,
                               m_dDistanceFromTag,
                               m_dHeadingSetPoint,
                               m_bDriveBackwards);
                    m_bAlreadyPrintedVisualLostFallback = true;
                }
            }
            // Fallback 2: Coast along last known heading (Handles 0.0 distance depth dropouts).
            else if (m_bHasSeenTarget)
            {
                if (!m_bAlreadyPrintedVisualLostFallback)
                {
                    LOG_NOTICE(
                        logging::g_qSharedLogger,
                        "ApproachingMarkerState: [FALLBACK 2] Visual lost & no geolocation history. Coasting along last known heading. Target Bearing: {:.2f} degrees, "
                        "Reverse: {}",
                        m_dHeadingSetPoint,
                        m_bDriveBackwards);
                    m_bAlreadyPrintedVisualLostFallback = true;
                }
            }
            // Wait in place: We have never seen the marker.
            else
            {
                globals::g_pDriveBoard->SendStop();
                return;
            }
        }
        else
        {
            // Marker detected.
            if (m_bAlreadyPrintedLost)
            {
                LOG_NOTICE(logging::g_qSharedLogger, "ApproachingMarkerState: Tracking regained! Target marker re-acquired.");
            }

            // Reset temporal trackers.
            m_tmLastSeenTime                            = tmCurrentTime;
            m_bAlreadyPrintedLost                       = false;
            m_bAlreadyPrintedVisualLostFallback         = false;
            m_bHasSeenTarget                            = true;

            std::shared_ptr<TagDetector> pFrontDetector = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam);
            std::shared_ptr<TagDetector> pRearDetector  = globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eRearCam);

            // Handle specific tag logic tracking.
            if (stBestArucoTag.nID != -1)
            {
                szTagModelSource = "OpenCV ArUco (ID: " + std::to_string(stBestArucoTag.nID) + ")";
                dCurrentTagYaw   = stBestArucoTag.dYawAngle;
                dCurrentTagConf  = stBestArucoTag.dConfidence;

                if (pFrontDetector != nullptr && stBestArucoTag.szDetectorUUID == pFrontDetector->GetThreadUUID())
                {
                    m_bDriveBackwards = false;
                    szCameraOrigin    = "Head Main Camera";
                }
                else if (pRearDetector != nullptr && stBestArucoTag.szDetectorUUID == pRearDetector->GetThreadUUID())
                {
                    m_bDriveBackwards = true;
                    szCameraOrigin    = "Rear Camera";
                }
                else
                {
                    szCameraOrigin = "Unknown Camera";
                }

                m_dDistanceFromTag = stBestArucoTag.dStraightLineDistance;

                // Require distance to be greater than 0.0 to use Geolocation absolute tracking.
                if (stBestArucoTag.stGeolocatedPosition.eType != geoops::WaypointType::eUNKNOWN && m_dDistanceFromTag > 0.0)
                {
                    geoops::GeoMeasurement stTagMeasurement =
                        geoops::CalculateGeoMeasurement(stCurrentRoverPose.GetUTMCoordinate(), stBestArucoTag.stGeolocatedPosition.GetUTMCoordinate());
                    m_dHeadingSetPoint           = stTagMeasurement.dStartRelativeBearing;
                    m_stLastGeolocatedPosition   = stBestArucoTag.stGeolocatedPosition;
                    m_bHasLastGeolocatedPosition = true;
                }
                else
                {
                    // Fallback to pure relative tracking (Current Heading + Yaw Angle).
                    double dVisionYawOffset = stBestArucoTag.dYawAngle;
                    if (m_bDriveBackwards)
                    {
                        dVisionYawOffset += 180.0;
                    }
                    m_dHeadingSetPoint = numops::InputAngleModulus(dVisionYawOffset + stCurrentRoverPose.GetCompassHeading(), 0.0, 360.0);
                }
            }
            else if (stBestTorchTag.dConfidence != 0.0)
            {
                szTagModelSource = "ML Torch Model";
                dCurrentTagYaw   = stBestTorchTag.dYawAngle;
                dCurrentTagConf  = stBestTorchTag.dConfidence;

                if (pFrontDetector != nullptr && stBestTorchTag.szDetectorUUID == pFrontDetector->GetThreadUUID())
                {
                    m_bDriveBackwards = false;
                    szCameraOrigin    = "Head Main Camera";
                }
                else if (pRearDetector != nullptr && stBestTorchTag.szDetectorUUID == pRearDetector->GetThreadUUID())
                {
                    m_bDriveBackwards = true;
                    szCameraOrigin    = "Rear Camera";
                }
                else
                {
                    szCameraOrigin = "Unknown Camera";
                }

                m_dDistanceFromTag = stBestTorchTag.dStraightLineDistance;

                // Require distance to be greater than 0.0 to use Geolocation absolute tracking.
                if (stBestTorchTag.stGeolocatedPosition.eType != geoops::WaypointType::eUNKNOWN && m_dDistanceFromTag > 0.0)
                {
                    geoops::GeoMeasurement stTagMeasurement =
                        geoops::CalculateGeoMeasurement(stCurrentRoverPose.GetUTMCoordinate(), stBestTorchTag.stGeolocatedPosition.GetUTMCoordinate());
                    m_dHeadingSetPoint           = stTagMeasurement.dStartRelativeBearing;
                    m_stLastGeolocatedPosition   = stBestTorchTag.stGeolocatedPosition;
                    m_bHasLastGeolocatedPosition = true;
                }
                else
                {
                    // Fallback to pure relative tracking (Current Heading + Yaw Angle).
                    double dVisionYawOffset = stBestTorchTag.dYawAngle;
                    if (m_bDriveBackwards)
                    {
                        dVisionYawOffset += 180.0;
                    }
                    m_dHeadingSetPoint = numops::InputAngleModulus(dVisionYawOffset + stCurrentRoverPose.GetCompassHeading(), 0.0, 360.0);
                }
            }
        }

        // 4. Execute Move Commands.
        diffdrive::DrivePowers stDrivePowers = globals::g_pDriveBoard->CalculateMove(constants::APPROACH_MARKER_MOTOR_POWER,
                                                                                     m_dHeadingSetPoint,
                                                                                     stCurrentRoverPose.GetCompassHeading(),
                                                                                     diffdrive::DifferentialControlMethod::eArcadeDrive,
                                                                                     m_bDriveBackwards,    // Reverse control flag.
                                                                                     false,
                                                                                     false,
                                                                                     false);
        globals::g_pDriveBoard->SendDrive(stDrivePowers);

        // 5. Output periodic logging (1Hz).
        if (std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - m_tmLastLogTime).count() >= 1)
        {
            m_tmLastLogTime = tmCurrentTime;

            if (bMarkerDetected)
            {
                std::string szTrackingType = (m_bHasLastGeolocatedPosition && m_dDistanceFromTag > 0.0) ? "Absolute (GPS Geolocation)" : "Relative (Vision Yaw Angle)";

                LOG_NOTICE(logging::g_qSharedLogger,
                           "ApproachingMarkerState Status:\n"
                           "  >> Detection   : Source: {}, Camera: {}\n"
                           "  >> Target      : Conf: {:.2f}%, Dist: {:.2f}m, Yaw: {:.2f} degrees\n"
                           "  >> Tracking    : Mode: {}, Reverse: {}\n"
                           "  >> Navigation  : Curr Hdg: {:.2f} degrees, Tgt Hdg: {:.2f} degrees",
                           szTagModelSource,
                           szCameraOrigin,
                           dCurrentTagConf * 100.0,
                           m_dDistanceFromTag,
                           dCurrentTagYaw,
                           szTrackingType,
                           m_bDriveBackwards ? "TRUE" : "FALSE",
                           stCurrentRoverPose.GetCompassHeading(),
                           m_dHeadingSetPoint);
            }
            else
            {
                LOG_NOTICE(logging::g_qSharedLogger,
                           "ApproachingMarkerState Status: Marker not visible. Executing fallback logic. Target Hdg: {:.2f} degrees, Reverse: {}, Est. Dist: {:.2f}m",
                           m_dHeadingSetPoint,
                           m_bDriveBackwards,
                           m_dDistanceFromTag);
            }
        }

        // 6. Check if target is reached.
        if (m_dDistanceFromTag != 0.0 && m_dDistanceFromTag < constants::APPROACH_MARKER_PROXIMITY_THRESHOLD)
        {
            LOG_NOTICE(logging::g_qSharedLogger,
                       "ApproachingMarkerState: SUCCESS! Rover has reached the target marker! (Distance: {:.2f}m < Threshold: {:.2f}m)",
                       m_dDistanceFromTag,
                       constants::APPROACH_MARKER_PROXIMITY_THRESHOLD);

            globals::g_pStateMachineHandler->HandleEvent(Event::eReachedMarker, true);
            return;
        }

        // 7. Check if the rover is stuck.
        if (constants::APPROACH_MARKER_ENABLE_STUCK_DETECT &&
            m_StuckDetector.CheckIfStuck(globals::g_pStateMachineHandler->SmartRetrieveVelocity(),
                                         globals::g_pStateMachineHandler->SmartRetrieveAngularVelocity(),
                                         constants::APPROACH_MARKER_STUCK_CHECK_VEL_THRESH * globals::g_pDriveBoard->GetMaxDriveEffort(),
                                         constants::APPROACH_MARKER_STUCK_CHECK_ROT_THRESH))
        {
            LOG_WARNING(logging::g_qSharedLogger,
                        "ApproachingMarkerState: Rover has become stuck! Triggering Stuck event. Curr Vel: {:.2f}, Ang Vel: {:.2f}",
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

                if (constants::APPROACH_MARKER_VERIFY_POSITION)
                {
                    eNextState = States::eVerifyingMarker;
                }
                else
                {
                    globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);
                    globals::g_pWaypointHandler->PopNextWaypoint();
                    globals::g_pStateMachineHandler->ClearSavedStates();
                    LOG_NOTICE(logging::g_qSharedLogger, "ApproachingMarkerState: Cleared old search pattern state and approaching marker state from saved states.");
                    eNextState = States::eIdle;
                }
                break;
            }
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Handling Start event.");
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
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eOff);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "ApproachingMarkerState: Handling unknown event ({}), defaulting to Idle.", static_cast<int>(eEvent));
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eApproachingMarker)
        {
            LOG_INFO(logging::g_qSharedLogger, "ApproachingMarkerState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state.
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
