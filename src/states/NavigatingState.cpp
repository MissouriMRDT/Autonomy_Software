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
#include "../util/states/ObjectDetectionChecker.hpp"
#include "../util/states/TagDetectionChecker.hpp"
#include "../util/vision/ObjectDetectionUtility.hpp"

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
     * @author Eli Byrd (edbgkk@mst.edu), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void NavigatingState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Scheduling next run of state logic.");

        // Initialize member variables.
        m_bWasStuck             = false;
        m_dStuckDistanceToGoal  = 0;
        m_bFetchNewWaypoint     = true;
        m_vTagDetectors         = {globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam),
                                   globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eRearCam)};
        m_vObjectDetectors      = {globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eHeadMainCam),
                                   globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eRearCam)};
        m_tmLastAvoidanceUpdate = std::chrono::system_clock::time_point::min();
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void NavigatingState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Exiting state.");

        // Wipe all virtual obstacles from local memory.
        m_vActiveVirtualObstacles.clear();

        // Wipe the global handlers so the next state/waypoint starts with a clean slate.
        globals::g_pWaypointHandler->ClearObstacles();
        globals::g_pGeoPlanner->ClearGeoCache();
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    NavigatingState::NavigatingState() : State(States::eNavigating)
    {
        // Submit logger message.
        LOG_INFO(logging::g_qConsoleLogger, "Entering State: {}", ToString());

        // Initialize member variables.
        m_bInitialized       = false;
        m_StuckDetector      = statemachine::TimeIntervalBasedStuckDetector(constants::NAVIGATING_STUCK_CHECK_ATTEMPTS,
                                                                            constants::NAVIGATING_STUCK_CHECK_INTERVAL,
                                                                            constants::NAVIGATING_STUCK_CHECK_VEL_THRESH,
                                                                            constants::NAVIGATING_STUCK_CHECK_ROT_THRESH);
        m_pStanleyController = std::make_unique<controllers::PredictiveStanleyController>(constants::STANLEY_CROSSTRACK_CONTROL_GAIN,
                                                                                          constants::STANLEY_ANGULAR_VELOCITY_LIMIT,
                                                                                          constants::STANLEY_PREDICTION_HORIZON,
                                                                                          constants::STANLEY_PREDICTION_TIME_STEP);

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
     * @author Eli Byrd (edbgkk@mst.edu), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void NavigatingState::Run()
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Running state-specific behavior.");

        // Check if we should get a new goal waypoint and that the waypoint handler has one for us.
        if (m_bFetchNewWaypoint && globals::g_pWaypointHandler->GetWaypointCount() > 0)
        {
            // Trigger new waypoint event.
            globals::g_pStateMachineHandler->HandleEvent(Event::eNewWaypoint);
            return;
        }

        // Get Current rover pose.
        geoops::RoverPose stCurrentRoverPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();

        // If navigating was previously stuck, then re-path plan stuck area
        if (m_bWasStuck)
        {
            // Change the path to reflect the new obstacle
            geoops::UTMCoordinate stLastStuckPoint = ModifyPathAfterStuckState();

            // Change Control Gain to turn around the obstacle and the distance from goal in which Control Gain can be changed back to normal
            m_dStuckDistanceToGoal = geoops::CalculateGeoMeasurement(stLastStuckPoint, m_stGoalWaypoint.GetUTMCoordinate()).dDistanceMeters;
            m_pStanleyController->SetControlGain(1);

            // Update visualizer and stanley
            globals::g_pWaypointHandler->StorePath("GeoPlannerPath", m_vPathCoordinates);
            m_pStanleyController->SetReferencePath(m_vPathCoordinates);

            m_StuckDetector.ResetStuckChecks();
            m_bWasStuck = false;
        }

        // Calculate distance and bearing from goal waypoint.
        geoops::GeoMeasurement stGoalWaypointMeasurement = geoops::CalculateGeoMeasurement(stCurrentRoverPose.GetUTMCoordinate(), m_stGoalWaypoint.GetUTMCoordinate());

        // Change the Control Gain back to normal once we are past the stuck obstacle
        if (m_dStuckDistanceToGoal != 0 && m_dStuckDistanceToGoal > stGoalWaypointMeasurement.dDistanceMeters)
        {
            m_pStanleyController->SetControlGain(constants::STANLEY_CROSSTRACK_CONTROL_GAIN);
            m_dStuckDistanceToGoal = 0;
        }

        // Only print out every so often.
        static bool bAlreadyPrinted = false;
        if ((std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() % 5) == 0 && !bAlreadyPrinted)
        {
            // Assemble the error metrics into a single string. We are going to include the distance and bearing to the goal waypoint and
            // the error between the rover pose and the GPS position. The rover pose could be from VIO or GNSS fusion, or just GPS.
            std::string szErrorMetrics =
                "--------[ Navigating Error Report ]--------\nDistance to Goal Waypoint: " + std::to_string(stGoalWaypointMeasurement.dDistanceMeters) + " meters\n" +
                "Bearing to Goal Waypoint: " + std::to_string(stGoalWaypointMeasurement.dStartRelativeBearing) + " degrees\n";
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

        /*
            The overall flow of this state is as follows.
            1. Is there a tag -> MarkerSeen
            2. Is there an object -> ObjectSeen
            3. Is there an obstacle -> TBD
            4. Navigate to goal waypoint.
            5. Is the rover stuck -> Stuck
        */

        /////////////////////////
        /* --- Detect Tags --- */
        /////////////////////////

        // In order to even care about any tags we see, the goal waypoint needs to be of type MARKER and we need to be within the search radius of the MARKER waypoint.
        if (m_stGoalWaypoint.eType == geoops::WaypointType::eTagWaypoint && stGoalWaypointMeasurement.dDistanceMeters <= m_stGoalWaypoint.dRadius)
        {
            // Create instance variables.
            tagdetectutils::ArucoTag stBestArucoTag, stBestTorchTag;
            // Identify target marker.
            statemachine::IdentifyTargetMarker(m_vTagDetectors, stBestArucoTag, stBestTorchTag, m_stGoalWaypoint.nID);
            // Check if either tag type is seen.
            if (stBestArucoTag.nID != -1 || stBestTorchTag.dConfidence != 0.0)
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Rover has seen a target marker!");
                // Handle state transition and save the current search pattern state.
                globals::g_pStateMachineHandler->HandleEvent(Event::eMarkerSeen, true);
                // Don't execute the rest of the state.
                return;
            }
        }

        ////////////////////////////
        /* --- Detect Objects --- */
        ////////////////////////////

        // In order to even care about any objects we see, the goal waypoint needs to be of an object type and we need to be within the search radius of the object
        // waypoint.
        if ((m_stGoalWaypoint.eType == geoops::WaypointType::eObjectWaypoint || m_stGoalWaypoint.eType == geoops::WaypointType::eMalletWaypoint ||
             m_stGoalWaypoint.eType == geoops::WaypointType::eWaterBottleWaypoint || m_stGoalWaypoint.eType == geoops::WaypointType::eRockPickWaypoint) &&
            stGoalWaypointMeasurement.dDistanceMeters <= m_stGoalWaypoint.dRadius)
        {
            // Create instance variables.
            objectdetectutils::Object stBestTorchObject;
            // Identify target object.
            statemachine::IdentifyTargetObject(m_vObjectDetectors, stBestTorchObject, m_stGoalWaypoint.eType);
            // Check if either tag type is seen.
            if (stBestTorchObject.dConfidence != 0.0)
            {
                // Submit logger message.
                LOG_NOTICE(logging::g_qSharedLogger, "NavigatingState: Rover has seen a target object!");

                // Handle state transition and save the current search pattern state.
                globals::g_pStateMachineHandler->HandleEvent(Event::eObjectSeen, true);
                // Don't execute the rest of the state.
                return;
            }
        }

        //////////////////////////////
        /* --- Detect Obstacles --- */
        //////////////////////////////

        // Implement Time To Live for detected obstacles.
        std::chrono::system_clock::time_point tmNow = std::chrono::system_clock::now();
        bool bObstaclesPruned                       = false;

        std::vector<VirtualObstacle>::iterator it   = m_vActiveVirtualObstacles.begin();

        // Iterate through detected active obstacles.
        while (it != m_vActiveVirtualObstacles.end())
        {
            // If it has existed longer than the limit, remove it.
            if (std::chrono::duration_cast<std::chrono::seconds>(tmNow - it->tmTimeDetected).count() >= constants::NAVIGATING_TIME_TO_LIVE_LIMIT)
            {
                it               = m_vActiveVirtualObstacles.erase(it);
                bObstaclesPruned = true;
            }
            else
            {
                ++it;
            }
        }

        // Check if we have removed any obstacles.
        if (bObstaclesPruned)
        {
            // Remove them from the waypoint handler.
            globals::g_pWaypointHandler->ClearObstacles();

            // Loop through the active obstacles.
            for (size_t i = 0; i < m_vActiveVirtualObstacles.size(); ++i)
            {
                globals::g_pWaypointHandler->AddObstacle(m_vActiveVirtualObstacles[i].stWaypoint);
            }
            globals::g_pGeoPlanner->ClearGeoCache();
        }

        // TODO: CHANGE 1000 to a constant
        // Check when the last time we found an obstacle was.
        if (std::chrono::duration_cast<std::chrono::milliseconds>(tmNow - m_tmLastAvoidanceUpdate).count() >= 1000)
        {
            // Update the last avoidance.
            m_tmLastAvoidanceUpdate = std::chrono::system_clock::now();

            // Open the main ZED.
            std::shared_ptr<ZEDCamera> pZED = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam);
            if (pZED)
            {
                // Request a copy of the pointcloud.
                cv::Mat cvPointCloud;
                std::future<bool> fuCloudStatus = pZED->RequestPointCloudCopy(cvPointCloud);

                // Add a very short timeout to the future so the state machine never hangs if the ZED disconnects.
                if (fuCloudStatus.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready && fuCloudStatus.get() && !cvPointCloud.empty())
                {
                    // Look for obstacles in the ZED.
                    std::vector<geoops::UTMCoordinate> vNewObstacles = objectdetectutils::ExtractObstaclesFromZED(cvPointCloud,
                                                                                                                  stCurrentRoverPose,
                                                                                                                  constants::NAVIGATING_POINTCLOUD_SUBSAMPLES,
                                                                                                                  constants::NAVIGATING_GRID_CELL_SIZE_METERS,
                                                                                                                  constants::NAVIGATING_OBSTACLE_VARIANCE_THRESHOLD);

                    // Check if new obstacles populated.
                    if (!vNewObstacles.empty())
                    {
                        for (size_t i = 0; i < vNewObstacles.size(); ++i)
                        {
                            // Create new obstacle waypoint.
                            geoops::Waypoint stObsWaypoint(vNewObstacles[i], geoops::WaypointType::eObstacleWaypoint, constants::NAVIGATING_OBSTACLE_RADIUS);

                            // Create variables for tracking the new obstacle.
                            VirtualObstacle stTrackedObs;
                            stTrackedObs.stWaypoint     = stObsWaypoint;
                            stTrackedObs.tmTimeDetected = tmNow;
                            m_vActiveVirtualObstacles.push_back(stTrackedObs);

                            globals::g_pWaypointHandler->AddObstacle(stObsWaypoint);
                        }

                        // First, check if the path is empty.
                        if (m_vPathCoordinates.empty())
                        {
                            LOG_WARNING(logging::g_qSharedLogger, "NavigatingState: Cannot splice detour, global path is empty!");
                            return;
                        }

                        // Create variables to splice the new path.
                        size_t nCurrentIndex                    = m_pStanleyController->GetReferencePathTargetIndex();
                        size_t nRejoinIndex                     = std::min(nCurrentIndex + 15, m_vPathCoordinates.size() - 1);
                        geoops::UTMCoordinate stLocalRejoinGoal = m_vPathCoordinates[nRejoinIndex].GetUTMCoordinate();

                        globals::g_pGeoPlanner->ClearGeoCache();

                        // TODO: CREATE CONSTANTS AND MAKE COMMENTS
                        // Generate local detour natively using the existing GeoPlanner.
                        std::vector<geoops::Waypoint> vDetour =
                            globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler, stCurrentRoverPose.GetUTMCoordinate(), stLocalRejoinGoal, 2.0, 5.0, 5.0);

                        if (!vDetour.empty())
                        {
                            m_vPathCoordinates.erase(m_vPathCoordinates.begin() + nCurrentIndex, m_vPathCoordinates.begin() + nRejoinIndex);
                            m_vPathCoordinates.insert(m_vPathCoordinates.begin() + nCurrentIndex, vDetour.begin(), vDetour.end());

                            globals::g_pWaypointHandler->StorePath("GeoPlannerPath", m_vPathCoordinates);
                            m_pStanleyController->SetReferencePath(m_vPathCoordinates);
                        }
                        else
                        {
                            LOG_WARNING(logging::g_qSharedLogger, "NavigatingState: GeoPlanner failed to map a safe detour! Rover is trapped.");
                            m_dHeadingBeforeStuck = stCurrentRoverPose.GetCompassHeading();
                            globals::g_pStateMachineHandler->HandleEvent(Event::eStuck, true);
                            m_bWasStuck = true;
                            return;
                        }

                        // Reset the cooldown timer so the rover has time to drive the new path.
                        m_tmLastAvoidanceUpdate = std::chrono::system_clock::now();
                    }
                }
            }
        }

        ///////////////////////////////////////
        /* --- Navigate to goal waypoint --- */
        ///////////////////////////////////////
        // Check if we are at the goal waypoint.
        if (stGoalWaypointMeasurement.dDistanceMeters > constants::NAVIGATING_REACHED_GOAL_RADIUS)
        {
            // NOTE: Optional - Uncomment the above code and comment out the below code to use stanley control to navigate to the goal waypoint.
            // Use stanley to calculate drive move/powers.
            controllers::PredictiveStanleyController::DriveVector stDriveVector = m_pStanleyController->Calculate(stCurrentRoverPose);
            // Calculate move from goal heading and desired speed.
            diffdrive::DrivePowers stDriveSpeeds = globals::g_pDriveBoard->CalculateMove(stDriveVector.dVelocity,
                                                                                         stDriveVector.dThetaHeading,
                                                                                         stCurrentRoverPose.GetCompassHeading(),
                                                                                         diffdrive::DifferentialControlMethod::eArcadeDrive);
            // diffdrive::DrivePowers stDriveSpeeds = globals::g_pDriveBoard->CalculateMove(constants::NAVIGATING_MOTOR_POWER,
            //                                                                              stGoalWaypointMeasurement.dStartRelativeBearing,
            //                                                                              stCurrentRoverPose.GetCompassHeading(),
            //                                                                              diffdrive::DifferentialControlMethod::eArcadeDrive);
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
                    // Continuously navigate to the next waypoint if our current waypoint ID is set to -99.
                    if (globals::g_pWaypointHandler->GetWaypointCount() > 1 &&
                        m_stGoalWaypoint.nID == static_cast<int>(manifest::Autonomy::AUTONOMYWAYPOINTTYPES::CONTINUOUSNAVIGATE))
                    {
                        // Submit logger message.
                        LOG_NOTICE(logging::g_qSharedLogger, "NavigatingState: The current waypoint ID is {}. Continuing to next waypoint...", m_stGoalWaypoint.nID);
                        // Pop the next waypoint.
                        globals::g_pWaypointHandler->PopNextWaypoint();
                        // Trigger new waypoint event.
                        globals::g_pStateMachineHandler->HandleEvent(Event::eNewWaypoint, true);
                    }
                    else
                    {
                        // We are at the goal, signal event.
                        globals::g_pStateMachineHandler->HandleEvent(Event::eReachedGpsCoordinate, false);
                    }
                    return;
                }
                // Goal waypoint is marker.
                case geoops::WaypointType::eTagWaypoint:
                {
                    // We are at the goal, signal event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedMarker, false);
                    return;
                }
                // Goal waypoint is object.
                case geoops::WaypointType::eObjectWaypoint:
                {
                    // We are at the goal, signal event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedObject, false);
                    return;
                }
                // Goal waypoint is mallet.
                case geoops::WaypointType::eMalletWaypoint:
                {
                    // We are at the goal, signal event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedObject, false);
                    return;
                }
                // Goal waypoint is water bottle.
                case geoops::WaypointType::eWaterBottleWaypoint:
                {
                    // We are at the goal, signal event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedObject, false);
                    return;
                }
                // Goal waypoint is rock pick.
                case geoops::WaypointType::eRockPickWaypoint:
                {
                    // We are at the goal, signal event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eReachedObject, false);
                    return;
                }
                default:
                {
                    // This waypoint type is not supported.
                    LOG_ERROR(logging::g_qSharedLogger, "NavigatingState: Unknown waypoint type!");
                    // Handle event.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eAbort, true);
                    // Don't execute the rest of the state.
                    return;
                }
            }
        }

        //////////////////////////////////////////
        /* ---  Check if the rover is stuck --- */
        //////////////////////////////////////////

        // Check if stuck.
        if (constants::NAVIGATING_ENABLE_STUCK_DETECT &&
            m_StuckDetector.CheckIfStuck(globals::g_pStateMachineHandler->SmartRetrieveVelocity(), globals::g_pStateMachineHandler->SmartRetrieveAngularVelocity()))
        {
            // Submit logger message.
            LOG_NOTICE(logging::g_qSharedLogger, "NavigatingState: Rover has become stuck!");
            // Save heading
            m_dHeadingBeforeStuck = stCurrentRoverPose.GetCompassHeading();
            // Handle state transition and save the current search pattern state.
            globals::g_pStateMachineHandler->HandleEvent(Event::eStuck, true);
            m_bWasStuck = true;
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
                // Check constants to see if we should go into verifying position or just trigger reached marker.
                if (constants::NAVIGATING_VERIFY_POSITION)
                {
                    // Send multimedia command to update state display.
                    globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                    // Change state.
                    eNextState = States::eVerifyingPosition;
                }
                else
                {
                    // Send multimedia command to update state display.
                    globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eReachedGoal);
                    // Pop the next waypoint.
                    globals::g_pWaypointHandler->PopNextWaypoint();
                    // Change state.
                    eNextState = States::eIdle;
                }
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
                    // Plan a new path using the GeoPlanner.
                    m_vPathCoordinates = globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler,
                                                                          globals::g_pStateMachineHandler->SmartRetrieveRoverPose().GetUTMCoordinate(),
                                                                          m_stGoalWaypoint.GetUTMCoordinate());
                    // Add the path to the waypoint handler for reference by other states or handlers.
                    globals::g_pWaypointHandler->StorePath("GeoPlannerPath", m_vPathCoordinates);
                    // Set the path of the stanley controller.
                    m_pStanleyController->SetReferencePath(m_vPathCoordinates);

                    // Get all obstacles from the obstacle handler.
                    std::vector<geoops::Waypoint> vObstacles = globals::g_pWaypointHandler->GetAllObstacles();

                    // Check if the path is empty. If it is, go to idle state.
                    if (m_vPathCoordinates.empty())
                    {
                        LOG_WARNING(logging::g_qSharedLogger, "NavigatingState: Planned path is empty! Transitioning to Idle State.");
                        eNextState = States::eIdle;
                    }
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
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eOff);
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
            case Event::eObjectSeen:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "NavigatingState: Handling ObjectSeen event.");
                // Change states.
                eNextState = States::eApproachingObject;
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

    /******************************************************************************
     * @brief Modify path after stuck state to avoid the stuck obstacle
     *
     * 1) Filter out any points that are a specific distance away from the obstacle
     * 2) Connect the rover position to the path to the first node of the vector by path-planning. If the rover is inside the obstacle, then first path plan it to
     *      the close edge of the obstacle.
     * 3) Iteratively go through the points of the path vector. If the point is inside the obstacle remove it. If the next point isn't being removed then
     *      connect the "hole" in the path by path-planning
     *
     *
     * @author Sam Nolte (samnolte0302@gmail.com), Sam Hajdukiewicz (samanthahajdukiewicz@gmail.com)
     * @date 2026-04-13
     ******************************************************************************/
    geoops::UTMCoordinate NavigatingState::ModifyPathAfterStuckState()
    {
        geoops::RoverPose stCurrentRoverPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();
        geoops::UTMCoordinate furthestPoint;

        // Get the obstacle's origin and radians from rover to obstacle.
        int nObstacleIndex                       = globals::g_pWaypointHandler->GetObstaclesCount();
        geoops::UTMCoordinate stObstaclePosition = globals::g_pWaypointHandler->RetrieveObstacleAtIndex(nObstacleIndex - 1).GetUTMCoordinate();
        LOG_INFO(logging::g_qSharedLogger, "Retrieved Obstacle: ({}, {})", stObstaclePosition.dEasting, stObstaclePosition.dNorthing);

        // Inflate the obstacle so Stanley will avoid it.
        globals::g_pWaypointHandler->AddObstacle(stObstaclePosition, constants::STUCK_OBSTACLE_RADIUS * 2);

        // Get the point on the obstacle's border which the rover came from.
        double dHeadingRad = (90.0 - m_dHeadingBeforeStuck) * M_PI / 180.0;
        if (dHeadingRad < 0)
        {
            dHeadingRad += 2 * M_PI;
        }

        // Grab starting coordinate and goal coordinate.
        geoops::UTMCoordinate stStartCoordinate = stCurrentRoverPose.GetUTMCoordinate();
        geoops::UTMCoordinate stGoalCoordinate  = stObstaclePosition;

        // Get goal coordinate eating and northing.
        stGoalCoordinate.dEasting -= std::cos(dHeadingRad) * constants::STUCK_OBSTACLE_RADIUS;
        stGoalCoordinate.dNorthing -= std::sin(dHeadingRad) * constants::STUCK_OBSTACLE_RADIUS;

        // Create variables for splicing the path.
        std::vector<geoops::Waypoint> vSplicePathCoordinates;
        std::vector<geoops::Waypoint>::iterator it = m_vPathCoordinates.begin();
        int nPointsAdded                           = 0;
        int nPointsRemoved                         = 0;

        // Unload affected LiDAR tile save data
        globals::g_pGeoPlanner->UnloadLiDARTiles(stObstaclePosition.dEasting - constants::STUCK_OBSTACLE_RADIUS,
                                                 stObstaclePosition.dEasting + constants::STUCK_OBSTACLE_RADIUS,
                                                 stObstaclePosition.dNorthing - constants::STUCK_OBSTACLE_RADIUS,
                                                 stObstaclePosition.dNorthing + constants::STUCK_OBSTACLE_RADIUS);

        // Find the node closest to the rover's current position to determine what has already been passed.
        std::vector<geoops::Waypoint>::iterator itClosest = m_vPathCoordinates.begin();
        double dMinDistSq                                 = std::numeric_limits<double>::max();

        // Loop through the waypoints.
        for (std::vector<geoops::Waypoint>::iterator itSearch = m_vPathCoordinates.begin(); itSearch != m_vPathCoordinates.end(); ++itSearch)
        {
            // Get easting and northing coordinates and determine the distance squared.
            double dx      = itSearch->GetUTMCoordinate().dEasting - stCurrentRoverPose.GetUTMCoordinate().dEasting;
            double dy      = itSearch->GetUTMCoordinate().dNorthing - stCurrentRoverPose.GetUTMCoordinate().dNorthing;
            double dDistSq = dx * dx + dy * dy;

            // If our distance is too small, update the minimum value and closest iterator point.
            if (dDistSq < dMinDistSq)
            {
                dMinDistSq = dDistSq;
                itClosest  = itSearch;
            }
        }

        // Delete all points in the path prior to the closest point, as they are behind the rover.
        if (itClosest != m_vPathCoordinates.begin())
        {
            nPointsRemoved += std::distance(m_vPathCoordinates.begin(), itClosest);
            it = m_vPathCoordinates.erase(m_vPathCoordinates.begin(), itClosest);
        }

        // Else, set it to the beginning of the path.
        else
        {
            it = m_vPathCoordinates.begin();
        }

        // If rover is in the obstacle path it out first and connect it to previous path
        double dx = stCurrentRoverPose.GetUTMCoordinate().dEasting - stObstaclePosition.dEasting;
        double dy = stCurrentRoverPose.GetUTMCoordinate().dNorthing - stObstaclePosition.dNorthing;
        if (sqrt(dx * dx + dy * dy) <= constants::STUCK_OBSTACLE_RADIUS)
        {
            geoops::UTMCoordinate stFirstNodeOfOriginalPath = m_vPathCoordinates.front().GetUTMCoordinate();

            // Splice in a new path from rover's current location to outside of the obstacle
            vSplicePathCoordinates = globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler, stStartCoordinate, stGoalCoordinate);
            it                     = m_vPathCoordinates.insert(m_vPathCoordinates.begin(), vSplicePathCoordinates.begin(), vSplicePathCoordinates.end());
            it += vSplicePathCoordinates.size();
            nPointsAdded += vSplicePathCoordinates.size();

            // Splice in a new path from outside of the obstacle to the end of the previous path
            stStartCoordinate      = (vSplicePathCoordinates.size() >= 2) ? std::prev(vSplicePathCoordinates.end())->GetUTMCoordinate() : stStartCoordinate;
            vSplicePathCoordinates = globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler, stStartCoordinate, stFirstNodeOfOriginalPath);
            if (vSplicePathCoordinates.size() >= 3)
            {
                it = m_vPathCoordinates.insert(it, std::next(vSplicePathCoordinates.begin()), std::prev(vSplicePathCoordinates.end()));
                nPointsAdded += vSplicePathCoordinates.size() - 2;
            }
        }

        // Rover is not inside obstacle, so just connect it to previous path.
        else
        {
            // Splice in a new path from rover's current location to the end of the previous path
            stGoalCoordinate       = m_vPathCoordinates.front().GetUTMCoordinate();
            vSplicePathCoordinates = globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler, stStartCoordinate, stGoalCoordinate);
            if (vSplicePathCoordinates.size() >= 2)
            {
                it = m_vPathCoordinates.insert(m_vPathCoordinates.begin(), vSplicePathCoordinates.begin(), std::prev(vSplicePathCoordinates.end()));
                nPointsAdded += vSplicePathCoordinates.size() - 1;
            }
        }

        // Remove all points that are in stuck zone.
        bool bLastDeleted = false;
        while (it != std::prev(m_vPathCoordinates.end()))
        {
            double dDifferenceX = it->GetUTMCoordinate().dEasting - stObstaclePosition.dEasting;
            double dDifferenceY = it->GetUTMCoordinate().dNorthing - stObstaclePosition.dNorthing;

            // If path coord is inside stuck zone, then remove it.
            if (sqrt(dDifferenceX * dDifferenceX + dDifferenceY * dDifferenceY) <= constants::STUCK_OBSTACLE_RADIUS)
            {
                bLastDeleted = true;
                it           = m_vPathCoordinates.erase(it);
                ++nPointsRemoved;
            }
            // If the previous node was deleted, then connect the dots correctly by splicing a new path in between.
            else if (bLastDeleted)
            {
                //  Plan a new path to the next remaining path node.
                stStartCoordinate      = (it != m_vPathCoordinates.begin()) ? std::prev(it)->GetUTMCoordinate() : stCurrentRoverPose.GetUTMCoordinate();
                stGoalCoordinate       = it->GetUTMCoordinate();
                vSplicePathCoordinates = globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler, stStartCoordinate, stGoalCoordinate);
                if (vSplicePathCoordinates.size() >= 3)
                {
                    it           = m_vPathCoordinates.insert(it, std::next(vSplicePathCoordinates.begin()), std::prev(vSplicePathCoordinates.end()));
                    bLastDeleted = false;
                    it += vSplicePathCoordinates.size() - 1;
                    nPointsAdded += vSplicePathCoordinates.size() - 2;
                }
            }
            else
            {
                ++it;
            }
        }
        if (bLastDeleted)
        {
            // Plan a new path to the next remaining path node
            stStartCoordinate      = std::prev(it)->GetUTMCoordinate();
            stGoalCoordinate       = it->GetUTMCoordinate();
            vSplicePathCoordinates = globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler, stStartCoordinate, stGoalCoordinate);
            m_vPathCoordinates.insert(it, std::next(vSplicePathCoordinates.begin()), std::prev(vSplicePathCoordinates.end()));
            nPointsAdded += vSplicePathCoordinates.size() - 2;
        }

        // Pop inflated obstacle
        globals::g_pWaypointHandler->DeleteObstacle(nObstacleIndex);

        LOG_INFO(logging::g_qSharedLogger, "Stuck state modified rover path: {} nodes added, {} nodes removed", nPointsAdded, nPointsRemoved);

        return stGoalCoordinate;
    }
}    // namespace statemachine
