/******************************************************************************
 * @brief Search Pattern State Implementation for Autonomy State Machine.
 *
 * @file SearchPatternState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "SearchPatternState.h"
#include "../AutonomyGlobals.h"
#include "../algorithms/SearchPattern.hpp"
#include "../algorithms/kinematics/DifferentialDrive.hpp"
#include "../interfaces/State.hpp"
#include "../util/states/ObjectDetectionChecker.hpp"
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
    void SearchPatternState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Scheduling next run of state logic.");

        // Initialize member variables.
        m_eCurrentSearchPatternType = SearchPatternType::eSpiral;
        m_nSearchPathIdx            = 0;
        m_stSearchPatternCenter     = globals::g_pWaypointHandler->PeekNextWaypoint();

        // Get the current rover pose.
        geoops::RoverPose stCurrentRoverPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();

        // Calculate the search path.
        m_vSearchPath = searchpattern::CalculateSpiralPatternWaypoints(m_stSearchPatternCenter.GetGPSCoordinate(),
                                                                       constants::SEARCH_ANGULAR_STEP_DEGREES,
                                                                       m_stSearchPatternCenter.dRadius,
                                                                       stCurrentRoverPose.GetCompassHeading(),
                                                                       constants::SEARCH_SPIRAL_SPACING);

        m_vSearchPath = GeoPlanSearchPattern(m_vSearchPath);

        // Add the search and rover path layers to the plot.
        m_pRoverPathPlot->CreatePathLayer("SpiralSearchPattern", "-o");
        m_pRoverPathPlot->CreatePathLayer("ReverseSpiralSearchPattern", "-o");
        m_pRoverPathPlot->CreateDotLayer("SnakeSearchPattern", "-g");
        m_pRoverPathPlot->CreateDotLayer("VerticalZigZagSearchPattern", "yellow");
        m_pRoverPathPlot->CreateDotLayer("DetectedTags", "blue");
        m_pRoverPathPlot->CreateDotLayer("DetectedObjects", "purple");
        m_pRoverPathPlot->CreateDotLayer("PurePursuitTargetIndex", "or");
        m_pRoverPathPlot->CreatePathLayer("RoverPath", "-k");
        // Plot the search path on the rover path.
        m_pRoverPathPlot->AddPathPoints(m_vSearchPath, "SpiralSearchPattern", 0);
        // Plot the search path in the visualizer.
        globals::g_pWaypointHandler->StorePath("GeoPlannerPath", m_vSearchPath);

        // Set the path of the pure pursuit controller.
        m_pPursuitController->SetReferencePath(m_vSearchPath);

        m_vTagDetectors    = {globals::g_pTagDetectionHandler->GetTagDetector(TagDetectionHandler::TagDetectors::eHeadMainCam)};
        m_vObjectDetectors = {globals::g_pObjectDetectionHandler->GetObjectDetector(ObjectDetectionHandler::ObjectDetectors::eHeadMainCam)};
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void SearchPatternState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Exiting state.");

        // Stop drive.
        globals::g_pDriveBoard->SendStop();
    }

    std::vector<geoops::Waypoint> SearchPatternState::GeoPlanSearchPattern(const std::vector<geoops::Waypoint>& skeltonPath)
    {
        LOG_NOTICE(logging::g_qSharedLogger, "Starting GeoPlanSearchPattern Path length: {}", skeltonPath.size());
        std::vector<geoops::Waypoint> m_vSearchPath;
        for (long unsigned int i = 0; i < skeltonPath.size() - 1; i++)
        {
            std::vector<geoops::Waypoint> newPoints =
                globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler, skeltonPath[i].GetUTMCoordinate(), skeltonPath[i + 1].GetUTMCoordinate(), 2.0, 240.0, false);
            m_vSearchPath.insert(m_vSearchPath.end(), newPoints.begin(), newPoints.end());
        }

        return m_vSearchPath;
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    SearchPatternState::SearchPatternState() : State(States::eSearchPattern)
    {
        // Submit logger message.
        LOG_INFO(logging::g_qConsoleLogger, "Entering State: {}", ToString());

        // Initialize member variables.
        m_bInitialized       = false;
        m_StuckDetector      = statemachine::TimeIntervalBasedStuckDetector(constants::STUCK_CHECK_ATTEMPTS,
                                                                       constants::STUCK_CHECK_INTERVAL,
                                                                       constants::STUCK_CHECK_VEL_THRESH,
                                                                       constants::STUCK_CHECK_ROT_THRESH);
        m_pRoverPathPlot     = std::make_unique<logging::graphing::PathTracer>("SearchPatternRoverPath");
        m_pPursuitController = std::make_unique<controllers::PurePursuitController>();

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
     * @author Jason Pittman (jspencerpittman@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void SearchPatternState::Run()
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "SearchPatternState: Running state-specific behavior.");

        // Get the current rover pose.
        geoops::RoverPose stCurrentRoverPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();

        // Add the current rover pose to the path plot.
        m_pRoverPathPlot->AddPathPoint(stCurrentRoverPose.GetUTMCoordinate(), "RoverPath");

        // Place a dot on the pure pursuit target index.
        // geoops::Waypoint stPurePursuitTargetCoordinate =
        //     m_pPurePursuitController->GetReferencePath().at(static_cast<size_t>(m_pPurePursuitController->GetReferencePathTargetIndex()));
        // m_pRoverPathPlot->ClearLayer("PurePursuitTargetIndex");
        // m_pRoverPathPlot->AddDot(stPurePursuitTargetCoordinate.GetUTMCoordinate(), "PurePursuitTargetIndex", 1);

        /*
            The overall flow of this state is as follows.
            1. Is there a tag -> MarkerSeen
            2. Is there an object -> ObjectSeen
            3. Is there an obstacle -> TBD
            4. Is the rover stuck -> Stuck
            5. Is the search pattern complete -> Abort
            6. Follow the search pattern.
        */

        /////////////////////////
        /* --- Detect Tags --- */
        /////////////////////////

        // In order to even care about any tags we see, the goal waypoint needs to be of type MARKER and we need to be within the search radius of the MARKER waypoint.
        if (m_stSearchPatternCenter.eType == geoops::WaypointType::eTagWaypoint)
        {
            // Create instance variables.
            tagdetectutils::ArucoTag stBestArucoTag, stBestTorchTag;
            // Identify target marker.
            statemachine::IdentifyTargetMarker(m_vTagDetectors, stBestArucoTag, stBestTorchTag, m_stSearchPatternCenter.nID);
            // Check if either tag type is seen.
            if (stBestArucoTag.nID != -1 || stBestTorchTag.dConfidence != 0.0)
            {
                // Submit logger message.
                LOG_NOTICE(logging::g_qSharedLogger, "SearchPatternState: Rover has seen a target marker!");

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
        if (m_stSearchPatternCenter.eType == geoops::WaypointType::eObjectWaypoint || m_stSearchPatternCenter.eType == geoops::WaypointType::eMalletWaypoint ||
            m_stSearchPatternCenter.eType == geoops::WaypointType::eWaterBottleWaypoint || m_stSearchPatternCenter.eType == geoops::WaypointType::eRockPickWaypoint)
        {
            // Create instance variables.
            objectdetectutils::Object stBestTorchObject;
            // Identify target object.
            statemachine::IdentifyTargetObject(m_vObjectDetectors, stBestTorchObject, m_stSearchPatternCenter.eType);
            // Check if either tag type is seen.
            if (stBestTorchObject.dConfidence != 0.0)
            {
                // Submit logger message.
                LOG_NOTICE(logging::g_qSharedLogger, "SearchPatternState: Rover has seen a target object!");

                // Check if the torch tag has a good absolute position.
                if (stBestTorchObject.dConfidence != 0.0 && stBestTorchObject.stGeolocatedPosition.eType == geoops::WaypointType::eObjectWaypoint)
                {
                    // Add the tag to the path plot.
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

        // TODO: Add obstacle detection to SearchPattern state

        //////////////////////////////////////////
        /* ---  Check if the rover is stuck --- */
        //////////////////////////////////////////

        // Check if stuck.
        if (constants::SEARCH_ENABLE_STUCK_DETECT &&
            m_StuckDetector.CheckIfStuck(globals::g_pStateMachineHandler->SmartRetrieveVelocity(), globals::g_pStateMachineHandler->SmartRetrieveAngularVelocity()))
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger, "SearchPattern: Rover has become stuck!");
            // Increment search path index so we skip the waypoint where we got stuck when reentering searchpattern.
            m_nSearchPathIdx += 1;
            // Check path index is within bounds.
            if (m_nSearchPathIdx >= int(m_vSearchPath.size()))
            {
                m_nSearchPathIdx = m_vSearchPath.size() - 1;
            }
            // Handle state transition and save the current search pattern state.
            globals::g_pStateMachineHandler->HandleEvent(Event::eStuck, true);
            // Don't execute the rest of the state.
            return;
        }

        ///////////////////////////////////
        /* --- Follow Search Pattern --- */
        ///////////////////////////////////

        // Check if the search path is empty.
        if (m_vSearchPath.empty())
        {
            // Submit logger message.
            LOG_WARNING(logging::g_qSharedLogger, "SearchPatternState: Search path is empty, aborting search.");
            // Handle state transition.
            globals::g_pStateMachineHandler->HandleEvent(Event::eAbort);
            return;
        }

        // Have we reached the current waypoint?
        geoops::GPSCoordinate stCurrTargetGPS    = m_vSearchPath[m_nSearchPathIdx].GetGPSCoordinate();
        geoops::GeoMeasurement stCurrRelToTarget = geoops::CalculateGeoMeasurement(stCurrentRoverPose.GetGPSCoordinate(), stCurrTargetGPS);
        bool bReachedTarget                      = stCurrRelToTarget.dDistanceMeters <= constants::SEARCH_WAYPOINT_PROXIMITY;

        // If the entire search pattern has been completed without seeing tags or objects, try different search pattern.
        if (bReachedTarget && m_nSearchPathIdx >= int(m_vSearchPath.size() - 1))
        {
            globals::g_pStateMachineHandler->HandleEvent(Event::eSearchFailed);
            return;
        }
        // Move on to the next waypoint in the search path.
        else if (bReachedTarget)
        {
            ++m_nSearchPathIdx;
            stCurrTargetGPS   = m_vSearchPath[m_nSearchPathIdx].GetGPSCoordinate();
            stCurrRelToTarget = geoops::CalculateGeoMeasurement(stCurrentRoverPose.GetGPSCoordinate(), stCurrTargetGPS);
        }
        // NOTE: Optional - Uncomment the above code and comment out the below code to use pure pursuit control to navigate to the goal waypoint.
        // Use pure pursuit to calculate drive move/powers.
        controllers::PurePursuitController::DriveVector stDriveVector = m_pPursuitController->Calculate(stCurrentRoverPose, constants::SEARCH_MOTOR_POWER);
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
    States SearchPatternState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState = States::eSearchPattern;
        // Get the current rover pose.
        geoops::RoverPose stCurrentRoverPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();
        bool bCompleteStateExit              = true;

        switch (eEvent)
        {
            case Event::eMarkerSeen:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Handling MarkerSeen event.");
                // Change states.
                eNextState = States::eApproachingMarker;
                break;
            }
            case Event::eObjectSeen:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Handling ObjectSeen event.");
                // Change state.
                eNextState = States::eApproachingObject;
                break;
            }
            case Event::eStart:
            {
                // Submit logger message
                LOG_NOTICE(logging::g_qSharedLogger, "SearchPatternState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eSearchFailed:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Handling SearchFailed event.");
                // Stop drive.
                globals::g_pDriveBoard->SendStop();

                // Regenerate a new search pattern.
                switch (m_eCurrentSearchPatternType)
                {
                    // Check which pattern to do next.
                    case SearchPatternType::eSpiral:
                    {
                        // Submit logger message.
                        LOG_NOTICE(logging::g_qSharedLogger, "SearchPatternState: Spiral search pattern failed, trying reverse spiral...");
                        // Generate vertical reverse spiral pattern.

                        // m_vSearchPath = searchpattern::CalculateSpiralPatternWaypoints(m_stSearchPatternCenter.GetGPSCoordinate(),
                        //                                                                -constants::SEARCH_ANGULAR_STEP_DEGREES,
                        //                                                                m_stSearchPatternCenter.dRadius,
                        //                                                                stCurrentRoverPose.GetCompassHeading(),
                        //                                                                constants::SEARCH_SPIRAL_SPACING);
                        // Reset index counter.
                        m_nSearchPathIdx = 0;
                        // Update current search pattern
                        m_eCurrentSearchPatternType = SearchPatternType::END;

                        m_vSearchPath               = GeoPlanSearchPattern(m_vSearchPath);
                        std::reverse(m_vSearchPath.begin(), m_vSearchPath.end());

                        // Add the search and rover path layers to the plot.
                        m_pRoverPathPlot->AddPathPoints(m_vSearchPath, "ReverseSpiralSearchPattern", 0);
                        // Plot the search path in the visualizer.
                        globals::g_pWaypointHandler->StorePath("GeoPlannerPath", m_vSearchPath);
                        // Set the path of the pure pursuit controller.
                        m_pPursuitController->SetReferencePath(m_vSearchPath);
                        break;
                    }
                    case SearchPatternType::END:
                    {
                        // Submit logger message.
                        LOG_WARNING(logging::g_qSharedLogger, "SearchPatternState: All patterns failed to find anything, giving up...");
                        // Pop old waypoint out of queue.
                        globals::g_pWaypointHandler->PopNextWaypoint();
                        // Change states.
                        eNextState = States::eIdle;
                        break;
                    }
                    default:
                    {
                        // Change states.
                        eNextState = States::eIdle;
                        break;
                    }
                }
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eOff);
                // Stop drive.
                globals::g_pDriveBoard->SendStop();
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            case Event::eStuck:
            {
                LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Handling Stuck event.");
                eNextState = States::eStuck;
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "SearchPatternState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eSearchPattern)
        {
            LOG_INFO(logging::g_qSharedLogger, "SearchPatternState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }
}    // namespace statemachine
