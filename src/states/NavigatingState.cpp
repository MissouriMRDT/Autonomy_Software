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
        m_bFetchNewWaypoint = true;
        m_vTagDetectors     = {globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam),
                               globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eRearCam)};
        m_vObjectDetectors  = {globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eHeadMainCam),
                               globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eRearCam)};

        // Create rover path layers.
        m_pRoverPathPlot->CreatePathLayer("NavPath", "--b");
        m_pRoverPathPlot->CreatePathLayer("RoverPath", "-k");
        m_pRoverPathPlot->CreatePathLayer("GeoPath", "-m");
        m_pRoverPathPlot->CreateDotLayer("StanleyTargetIndex", "or");
        m_pRoverPathPlot->CreateDotLayer("ObstaclesLocation", "o");
        m_pRoverPathPlot->CreateDotLayer("DetectedTags", "green");
        m_pRoverPathPlot->CreateDotLayer("DetectedObjects", "red");
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
        m_bInitialized       = false;
        m_StuckDetector      = statemachine::TimeIntervalBasedStuckDetector(constants::STUCK_CHECK_ATTEMPTS,
                                                                            constants::STUCK_CHECK_INTERVAL,
                                                                            constants::STUCK_CHECK_VEL_THRESH,
                                                                            constants::STUCK_CHECK_ROT_THRESH);
        m_pRoverPathPlot     = std::make_unique<logging::graphing::PathTracer>("NavigatingRoverPath");
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
     * @author Eli Byrd (edbgkk@mst.edu)
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
        // Calculate distance and bearing from goal waypoint.
        geoops::GeoMeasurement stGoalWaypointMeasurement = geoops::CalculateGeoMeasurement(stCurrentRoverPose.GetUTMCoordinate(), m_stGoalWaypoint.GetUTMCoordinate());
        // Add the current rover pose to the path plot.
        m_pRoverPathPlot->AddPathPoint(stCurrentRoverPose.GetUTMCoordinate(), "RoverPath", 1);

        // Place a dot on the stanley target index.
        geoops::Waypoint stStanleyTargetCoordinate =
            m_pStanleyController->GetReferencePath().at(static_cast<size_t>(m_pStanleyController->GetReferencePathTargetIndex()));
        m_pRoverPathPlot->ClearLayer("StanleyTargetIndex");
        m_pRoverPathPlot->AddDot(stStanleyTargetCoordinate.GetUTMCoordinate(), "StanleyTargetIndex", 1);

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

                // Check if the OpenCV tag has a good absolute position.
                if (stBestArucoTag.nID != -1 && stBestArucoTag.stGeolocatedPosition.eType == geoops::WaypointType::eTagWaypoint)
                {
                    // Add the tag to the path plot.
                    m_pRoverPathPlot->AddDot(stBestArucoTag.stGeolocatedPosition.GetUTMCoordinate(), "DetectedTags");
                }
                // Check if the torch tag has a good absolute position.
                if (stBestTorchTag.dConfidence != 0.0 && stBestTorchTag.stGeolocatedPosition.eType == geoops::WaypointType::eTagWaypoint)
                {
                    // Add the tag to the path plot.
                    m_pRoverPathPlot->AddDot(stBestTorchTag.stGeolocatedPosition.GetUTMCoordinate(), "DetectedTags");
                }

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

                // Check if the object has a good absolute position.
                if (stBestTorchObject.stGeolocatedPosition.eType == geoops::WaypointType::eObjectWaypoint)
                {
                    // Add the object to the path plot.
                    m_pRoverPathPlot->AddDot(stBestTorchObject.stGeolocatedPosition.GetUTMCoordinate(), "DetectedObjects");
                }

                // Handle state transition and save the current search pattern state.
                globals::g_pStateMachineHandler->HandleEvent(Event::eObjectSeen, true);
                // Don't execute the rest of the state.
                return;
            }
        }

        //////////////////////////////
        /* --- Detect Obstacles --- */
        //////////////////////////////

        std::shared_ptr<ZEDCamera> pZED = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam);
        if (pZED)
        {
            cv::Mat cvPointCloud;
            sl::Plane slFloorPlane;

            // Request point cloud and floor plane.
            std::future<bool> fuCloudStatus = pZED->RequestPointCloudCopy(cvPointCloud);
            std::future<bool> fuPlaneStatus = pZED->RequestFloorPlaneCopy(slFloorPlane);

            if (fuCloudStatus.get() && fuPlaneStatus.get() && !cvPointCloud.empty())
            {
                std::vector<geoops::UTMCoordinate> vNewObstacles = objectdetectutils::ExtractObstaclesFromZED(cvPointCloud, stCurrentRoverPose);

                if (!vNewObstacles.empty())
                {
                    LOG_DEBUG(logging::g_qSharedLogger, "NavigatingState: Extracted {} virtual obstacles from ZED.", vNewObstacles.size());

                    // Add the new points to the WaypointHandler's global obstacle list
                    for (const geoops::UTMCoordinate& stPoint : vNewObstacles)
                    {
                        // Assign a 0.5m radius.
                        globals::g_pWaypointHandler->AddObstacle(stPoint, 0.5);
                    }

                    // Dynamic local avoidance splicing using GeoPlanner.
                    size_t nCurrentIndex                    = m_pStanleyController->GetReferencePathTargetIndex();
                    size_t nRejoinIndex                     = std::min(nCurrentIndex + 15, m_vPathCoordinates.size() - 1);
                    geoops::UTMCoordinate stLocalRejoinGoal = m_vPathCoordinates[nRejoinIndex].GetUTMCoordinate();

                    // Clear the GeoPlanner's cache so it is forced to look at the new obstacles.
                    globals::g_pGeoPlanner->ClearGeoCache();

                    // Plan the detour.
                    std::vector<geoops::Waypoint> vDetour = globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler,
                                                                                             stCurrentRoverPose.GetUTMCoordinate(),
                                                                                             stLocalRejoinGoal,
                                                                                             2.0,       // Search Radius
                                                                                             5.0,       // Max Search Time
                                                                                             false);    // Plot Path

                    if (!vDetour.empty())
                    {
                        // Splice the detour into the global path.
                        m_vPathCoordinates.erase(m_vPathCoordinates.begin() + nCurrentIndex, m_vPathCoordinates.begin() + nRejoinIndex);
                        m_vPathCoordinates.insert(m_vPathCoordinates.begin() + nCurrentIndex, vDetour.begin(), vDetour.end());

                        // Pass the updated path back to the controller.
                        m_pStanleyController->SetReferencePath(m_vPathCoordinates);
                    }
                    else
                    {
                        globals::g_pStateMachineHandler->HandleEvent(Event::eStuck, true);
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
                    // Clear the old path plot and add the new path.
                    m_pRoverPathPlot->ClearLayer("NavPath");
                    // Add starting point and goal point to path plot.
                    m_pRoverPathPlot->AddPathPoint(globals::g_pStateMachineHandler->SmartRetrieveRoverPose().GetUTMCoordinate(), "NavPath", 0);
                    m_pRoverPathPlot->AddPathPoint(m_stGoalWaypoint, "NavPath", 0);

                    // Update our plot with the new path.
                    m_pRoverPathPlot->ClearLayer("GeoPath");
                    // Plan a new path using the GeoPlanner.
                    std::vector<geoops::Waypoint> m_vPathCoordinates =
                        globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler,
                                                         globals::g_pStateMachineHandler->SmartRetrieveRoverPose().GetUTMCoordinate(),
                                                         m_stGoalWaypoint.GetUTMCoordinate());
                    // Add the path to the waypoint handler for reference by other states or handlers.
                    globals::g_pWaypointHandler->StorePath("GeoPlannerPath", m_vPathCoordinates);
                    // Add the new path to the plot.
                    m_pRoverPathPlot->AddPathPoints(m_vPathCoordinates, "GeoPath", 0);
                    // Set the path of the stanley controller.
                    m_pStanleyController->SetReferencePath(m_vPathCoordinates);

                    // Get all obstacles from the obstacle handler.
                    std::vector<geoops::Waypoint> vObstacles = globals::g_pWaypointHandler->GetAllObstacles();
                    m_pRoverPathPlot->ClearLayer("ObstaclesLocation");
                    m_pRoverPathPlot->AddDots(vObstacles, "ObstaclesLocation", 0);

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
}    // namespace statemachine
