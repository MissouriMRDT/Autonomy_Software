/******************************************************************************
 * @brief Stuck State Implementation for Autonomy State Machine.
 *
 * @file StuckState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2024-03-03
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "StuckState.h"
#include "../AutonomyConstants.h"
#include "../AutonomyGlobals.h"
#include "../algorithms/kinematics/DifferentialDrive.hpp"
#include "../handlers/WaypointHandler.h"
#include "../util/GeospatialOperations.hpp"

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
     * @author Eli Byrd (edbgkk@mst.edu), OcelotEmpire (hobbz.pi@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void StuckState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "StuckState: Scheduling next run of state logic.");

        // Store the postion and heading where the rover got stuck.
        geoops::RoverPose stStartRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();
        m_stObstaclePosition               = stStartRoverPose.GetGPSCoordinate();
        m_dObstacleHeading                 = stStartRoverPose.GetCompassHeading();

        // Check if this is the first time we are entering StuckState
        if (!m_bInitialized)
        {
            m_stOriginalPosition = m_stObstaclePosition;
            m_dOriginalHeading   = m_dObstacleHeading;
            m_eStuckLeg          = StuckLeg::eUnsticking;
            m_bReachedGoal       = false;

            // Store the state that got stuck and triggered a stuck event.
            m_eTriggeringState = globals::g_pStateMachineHandler->GetPreviousState();

            // Mark state as initialized.
            m_bInitialized = true;
        }

        m_bIsCurrentlyAligning = false;
        m_eAttemptType         = AttemptType::eReverseCurrentHeading;

        // Get state start time.
        m_tmStuckStartTime = std::chrono::system_clock::now();

        // Stop drivetrain.
        globals::g_pDriveBoard->SendStop();
    }

    /******************************************************************************
     * @brief This method is called when the state is exited. It is used to clean up
     *        the state.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    void StuckState::Exit()
    {
        // Clean up the state before exiting
        LOG_INFO(logging::g_qSharedLogger, "StuckState: Exiting state.");
    }

    /******************************************************************************
     * @brief Construct a new State object.
     *
     *
     * @author Eli Byrd (edbgkk@mst.edu)
     * @date 2024-01-17
     ******************************************************************************/
    StuckState::StuckState() : State(States::eStuck)
    {
        LOG_INFO(logging::g_qConsoleLogger, "Entering State: {}", this->ToString());

        m_bInitialized = false;
        Start();
    }

    /******************************************************************************
     * @brief Run the state machine. Returns the next state.
     *
     *
     * @author OcelotEmpire (hobbz.pi@gmail.com)
     * @date 2025-05-26
     ******************************************************************************/
    void StuckState::Run()
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "StuckState: Running state-specific behavior.");

        // Store the current postion and heading.
        geoops::RoverPose stCurrentRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();
        // Get current time.
        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();

        // First, get unstuck. Then, place waypoints to the right of the obstacle. If we get sent back into
        // StuckState, get unstuck and place new waypoints to the left of the obstacle. If those waypoints
        // are reached, ReachedWaypointState will clear all StuckStates. If neither route works, navigate
        // back to the original point and go into IdleState and wait for Basestation to decide what to do.
        switch (m_eStuckLeg)
        {
            case StuckLeg::eUnsticking:
            {
                // Check if we're still stuck.
                if (SamePosition(m_stObstaclePosition, stCurrentRoverPose.GetGPSCoordinate()))
                {
                    TryUnsticking();
                }
                else
                {
                    // Submit logger message.
                    LOG_NOTICE(logging::g_qSharedLogger,
                               "StuckState: Rover has successfully unstuckith itself! A total of {} seconds was wasted being stuck.",
                               std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - m_tmStuckStartTime).count());
                    LOG_INFO(logging::g_qSharedLogger, "StuckState: Attempting to navigate to the right of the obstacle.");

                    // Store the postion where the rover first got unstuck.
                    m_stHomePosition = stCurrentRoverPose.GetGPSCoordinate();

                    // Generate routes for later.
                    GeneratePaths();
                    // Add right path to front of waypoint queue inreverse order.
                    for (auto stdIt = m_vRightPath.rbegin(); stdIt != m_vRightPath.rend(); ++stdIt)
                    {
                        globals::g_pWaypointHandler->PushWaypoint(*stdIt);
                    }
                    // Continue to next StuckState leg.
                    m_eStuckLeg = StuckLeg::eGoingRight;
                    // Return to NavigatingState, but save this StuckState to come back to.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eUnstuck, true);
                }
                break;
            }
            case StuckLeg::eGoingRight:
            {
                // We have gotten stuck while going right. Get unstuck and try to take the left route.
                if (SamePosition(m_stObstaclePosition, stCurrentRoverPose.GetGPSCoordinate()))
                {
                    TryUnsticking();
                }
                else
                {
                    // Submit logger message.
                    LOG_NOTICE(logging::g_qSharedLogger,
                               "StuckState: Rover has successfully unstuckith itself! A total of {} seconds was wasted being stuck.",
                               std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - m_tmStuckStartTime).count());
                    LOG_INFO(logging::g_qSharedLogger, "StuckState: Attempting to navigate to the left of the obstacle.");

                    // Check if there are waypoints left over from m_vRightPath.
                    geoops::Waypoint stGoalWaypoint = globals::g_pWaypointHandler->PeekNextWaypoint();
                    // Find position of stGoalWaypoint in m_vRightPath.
                    const auto stdGoalIt = std::find(m_vRightPath.begin(), m_vRightPath.end(), stGoalWaypoint);
                    // Remove any points from m_vRightPath
                    for (auto stdIt = stdGoalIt; stdIt != m_vRightPath.end(); ++stdIt)
                    {
                        globals::g_pWaypointHandler->PopNextWaypoint();
                    }

                    // If we were in the middle of m_vRightPath, we want to back track down the path we came from.
                    if (stdGoalIt != m_vRightPath.end())
                    {
                        // Since we are backtracking, we add the portion of m_vRightPath that we have already traversed to the
                        // beginning of m_vLeftPath in reverse order.
                        for (auto stdIt = m_vRightPath.begin(); stdIt != stdGoalIt; ++stdIt)
                        {
                            m_vLeftPath.insert(m_vLeftPath.begin(), *stdIt);
                        }
                    }
                    // We have reached a StuckState goal, there will be no StuckState waypoints in the waypoint queue.
                    // If this is the case, we can return to the previous state.
                    else
                    {
                        // Submit logger message.
                        LOG_NOTICE(logging::g_qSharedLogger, "StuckState: Rover unstuck. Resuming execution of previous state.");
                        // Flag that we have reached a StuckState goal waypoint.
                        m_bReachedGoal = true;
                        // Trigger unstuck event and clear this state.
                        globals::g_pStateMachineHandler->HandleEvent(Event::eUnstuck, false);
                        // Don't run rest of state.
                        return;
                    }

                    // Add left path to front of waypoint queue inreverse order.
                    for (auto stdIt = m_vLeftPath.rbegin(); stdIt != m_vLeftPath.rend(); ++stdIt)
                    {
                        globals::g_pWaypointHandler->PushWaypoint(*stdIt);
                    }
                    // Continue to next StuckState leg.
                    m_eStuckLeg = StuckLeg::eGoingLeft;
                    // Return to NavigatingState, but save this StuckState to come back to.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eUnstuck, true);
                }
                break;
            }
            case StuckLeg::eGoingLeft:
            {
                // We have gotten stuck while going left. Get unstuck and try to return to m_stHomePosition.
                if (SamePosition(m_stObstaclePosition, stCurrentRoverPose.GetGPSCoordinate()))
                {
                    TryUnsticking();
                }
                else
                {
                    // Submit logger message.
                    LOG_NOTICE(logging::g_qSharedLogger,
                               "StuckState: Rover has successfully unstuckith itself! A total of {} seconds was wasted being stuck.",
                               std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - m_tmStuckStartTime).count());
                    LOG_INFO(logging::g_qSharedLogger, "StuckState: Attempting to navigate back to the first place Rover got unstuck.");

                    // Check if there are waypoints left over from m_vLeftPath.
                    geoops::Waypoint stGoalWaypoint = globals::g_pWaypointHandler->PeekNextWaypoint();
                    // Find position of stGoalWaypoint in m_vLeftPath.
                    const auto stdGoalIt = std::find(m_vLeftPath.begin(), m_vLeftPath.end(), stGoalWaypoint);
                    // Remove any points from m_vLeftPath
                    for (auto stdIt = stdGoalIt; stdIt != m_vLeftPath.end(); ++stdIt)
                    {
                        globals::g_pWaypointHandler->PopNextWaypoint();
                    }

                    // If we were in the middle of m_vLeftPath, we want to back track down the path we came from.
                    if (stdGoalIt != m_vLeftPath.end())
                    {
                        // Since we are backtracking, we add the portion of m_vLeftPath that we have already traversed to the
                        // beginning of the waypoint queue in reverse order.
                        for (auto stdIt = m_vLeftPath.begin(); stdIt != stdGoalIt; ++stdIt)
                        {
                            globals::g_pWaypointHandler->PushWaypoint(*stdIt);
                        }
                    }
                    // We have reached a StuckState goal, there will be no StuckState waypoints in the waypoint queue.
                    // If this is the case, we can return to the previous state.
                    else
                    {
                        // Submit logger message.
                        LOG_NOTICE(logging::g_qSharedLogger, "StuckState: Rover unstuck. Resuming execution of previous state.");
                        // Flag that we have reached a StuckState goal waypoint.
                        m_bReachedGoal = true;
                        // Trigger unstuck event and clear this state.
                        globals::g_pStateMachineHandler->HandleEvent(Event::eUnstuck, false);
                        // Don't run rest of state.
                        return;
                    }

                    // Continue to next StuckState leg.
                    m_eStuckLeg = StuckLeg::eReturning;
                    // Return to NavigatingState, but save this StuckState to come back to.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eUnstuck, true);
                }
                break;
            }
            case StuckLeg::eReturning:
            {
                // It's kind of over at this point. Try to get unstuck one last time then go into IdleState.
                if (SamePosition(m_stObstaclePosition, stCurrentRoverPose.GetGPSCoordinate()))
                {
                    TryUnsticking();
                }
                else
                {
                    // Submit logger message.
                    LOG_NOTICE(logging::g_qSharedLogger,
                               "StuckState: Rover has successfully unstuckith itself! A total of {} seconds was wasted being stuck.",
                               std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - m_tmStuckStartTime).count());
                    // Notify Basestation.
                    LOG_NOTICE(logging::g_qSharedLogger, "Couldn't find a route around the obstacle. Waiting for a new command...");
                    // Return to IdleState.
                    globals::g_pStateMachineHandler->HandleEvent(Event::eAbort, false);
                }
                break;
            }
            default:
            {
                // Submit logger message.
                LOG_ERROR(logging::g_qSharedLogger, "StuckState: Unknown StuckState leg!");
                // Return to IdleState.
                globals::g_pStateMachineHandler->HandleEvent(Event::eAbort, false);
                break;
            }
        }
    }

    /******************************************************************************
     * @brief Try to get unstuck by reversing in various directions.
     *
     * @author Eli Byrd (edbgkk@mst.edu), Jason Pittman (jspencerpittman@gmail.com), clayjay3 (claytonraycowen@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void StuckState::TryUnsticking()
    {
        // Store the current postion and heading.
        geoops::RoverPose stCurrentRoverPose = globals::g_pWaypointHandler->SmartRetrieveRoverPose();
        // Get current time.
        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();

        // Perform unstuck logic.
        switch (m_eAttemptType)
        {
            // On the first attempt we use the rover's original heading so alignment would already be completed.
            case AttemptType::eReverseCurrentHeading:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "StuckState: Maintaining current heading and reversing...");
                // Update stuck type enum for if we are still stuck after reversing.
                m_eAttemptType = AttemptType::eReverseLeft;
                // Handle reversing event. Save current state.
                globals::g_pStateMachineHandler->HandleEvent(Event::eReverse, true);
                break;
            }
                // On the second attempt align the rover constants::STUCK_ALIGN_DEGREES degrees to the right of the original heading instead.
            case AttemptType::eReverseLeft:
            {
                // Check if we are already realigning.
                if (!m_bIsCurrentlyAligning)
                {
                    // Submit logger message.
                    LOG_INFO(logging::g_qSharedLogger, "StuckState: Aligning rover heading {} degrees clockwise...", constants::STUCK_ALIGN_DEGREES);
                    // Set aligning toggle.
                    m_bIsCurrentlyAligning = true;
                    // Update start heading.
                    m_dObstacleHeading = stCurrentRoverPose.GetCompassHeading();
                    // Update start time.
                    m_tmAlignStartTime = std::chrono::system_clock::now();
                }
                else
                {
                    // Calculate time elapsed since realignment was started.
                    double dTimeElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - m_tmAlignStartTime).count() / 1000.0;
                    // Calculate the goal realignment heading.
                    double dGoalHeading = numops::InputAngleModulus<double>(m_dObstacleHeading + constants::STUCK_ALIGN_DEGREES, 0, 360);
                    // Calculate total rotation degrees so far.
                    double dRealignmentDegrees = numops::AngularDifference<double>(stCurrentRoverPose.GetCompassHeading(), dGoalHeading);

                    // Align drivetrain to a certain heading with 0 forward/reverse power.
                    diffdrive::DrivePowers stTurnPowers = globals::g_pDriveBoard->CalculateMove(0.0,
                                                                                                dGoalHeading,
                                                                                                stCurrentRoverPose.GetCompassHeading(),
                                                                                                diffdrive::DifferentialControlMethod::eArcadeDrive);
                    // Send drive powers.
                    globals::g_pDriveBoard->SendDrive(stTurnPowers);

                    // Check if we have successfully realigned.
                    if (dRealignmentDegrees <= constants::STUCK_ALIGN_TOLERANCE)
                    {
                        // Submit logger message.
                        LOG_INFO(logging::g_qSharedLogger, "StuckState: Realignment complete! Reversing...");
                        // Update stuck type enum for if we are still stuck after reversing.
                        m_eAttemptType = AttemptType::eReverseRight;
                        // Reset currently aligning toggle.
                        m_bIsCurrentlyAligning = false;
                        // Handle reversing event.
                        globals::g_pStateMachineHandler->HandleEvent(Event::eReverse, true);
                    }
                    // If not aligned yet, check if we hit the timeout.
                    else if (dTimeElapsed >= constants::STUCK_HEADING_ALIGN_TIMEOUT)
                    {
                        // Submit logger message.
                        LOG_NOTICE(logging::g_qSharedLogger,
                                   "StuckState: Rotated/Realigned {} degrees in {} seconds before timeout was reached. Rover is still stuck...",
                                   constants::STUCK_ALIGN_DEGREES - dRealignmentDegrees,
                                   dTimeElapsed);
                        // Update stuck type enum for if we are still stuck after reversing.
                        m_eAttemptType = AttemptType::eReverseRight;
                        // Reset currently aligning toggle.
                        m_bIsCurrentlyAligning = false;
                        // Handle reversing event.
                        globals::g_pStateMachineHandler->HandleEvent(Event::eReverse, true);
                    }
                }
                break;
            }
            // For the third do it constants::STUCK_ALIGN_DEGREES degrees to the left of the original heading.
            case AttemptType::eReverseRight:
            {
                // Check if we are already realigning.
                if (!m_bIsCurrentlyAligning)
                {
                    // Submit logger message.
                    LOG_INFO(logging::g_qSharedLogger, "StuckState: Aligning rover heading {} degrees counter-clockwise...", constants::STUCK_ALIGN_DEGREES);
                    // Set aligning toggle.
                    m_bIsCurrentlyAligning = true;
                    // Update start heading.
                    m_dObstacleHeading = stCurrentRoverPose.GetCompassHeading();
                    // Update start time.
                    m_tmAlignStartTime = std::chrono::system_clock::now();
                }
                else
                {
                    // Calculate time elapsed since realignment was started.
                    double dTimeElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - m_tmAlignStartTime).count() / 1000.0;
                    // Calculate the goal realignment heading.
                    double dGoalHeading = numops::InputAngleModulus<double>(m_dObstacleHeading - constants::STUCK_ALIGN_DEGREES, 0, 360);
                    // Calculate total rotation degrees so far.
                    double dRealignmentDegrees = numops::AngularDifference<double>(stCurrentRoverPose.GetCompassHeading(), dGoalHeading);

                    // Align drivetrain to a certain heading with 0 forward/reverse power.
                    diffdrive::DrivePowers stTurnPowers = globals::g_pDriveBoard->CalculateMove(0.0,
                                                                                                dGoalHeading,
                                                                                                stCurrentRoverPose.GetCompassHeading(),
                                                                                                diffdrive::DifferentialControlMethod::eArcadeDrive);
                    // Send drive powers.
                    globals::g_pDriveBoard->SendDrive(stTurnPowers);

                    // Check if we have successfully realigned.
                    if (dRealignmentDegrees <= constants::STUCK_ALIGN_TOLERANCE)
                    {
                        // Submit logger message.
                        LOG_INFO(logging::g_qSharedLogger, "StuckState: Realignment complete! Reversing...");
                        // Update stuck type enum for if we are still stuck after reversing.
                        m_eAttemptType = AttemptType::eGiveUp;
                        // Reset currently aligning toggle.
                        m_bIsCurrentlyAligning = false;
                        // Handle reversing event.
                        globals::g_pStateMachineHandler->HandleEvent(Event::eReverse, true);
                    }
                    // If not aligned yet, check if we hit the timeout.
                    else if (dTimeElapsed >= constants::STUCK_HEADING_ALIGN_TIMEOUT)
                    {
                        // Submit logger message.
                        LOG_NOTICE(logging::g_qSharedLogger,
                                   "StuckState: Rotated/Realigned {} degrees in {} seconds before timeout was reached. Rover is still stuck...",
                                   constants::STUCK_ALIGN_DEGREES - dRealignmentDegrees,
                                   dTimeElapsed);
                        // Update stuck type enum for if we are still stuck after reversing.
                        m_eAttemptType = AttemptType::eGiveUp;
                        // Reset currently aligning toggle.
                        m_bIsCurrentlyAligning = false;
                        // Handle reversing event.
                        globals::g_pStateMachineHandler->HandleEvent(Event::eReverse, true);
                    }
                }
                break;
            }
            case AttemptType::eGiveUp:
            {
                // Submit logger message.
                LOG_WARNING(logging::g_qSharedLogger, "StuckState: After multiple attempts, autonomy was unable to get the rover unstuck. Giving Up...");
                // Return to idle.
                globals::g_pStateMachineHandler->HandleEvent(Event::eAbort);
                break;
            }
            default:
            {
                // Submit logger message.
                LOG_ERROR(logging::g_qSharedLogger, "StuckState: Unknown attempt type!");
                // Return to idle.
                globals::g_pStateMachineHandler->HandleEvent(Event::eAbort);
                break;
            }
        }
    }

    /******************************************************************************
     * @brief Generate a left and a right path around the obstacle and store them.
     *
     *
     * @author OcelotEmpire (hobbz.pi@gmail.com)
     * @date 2025-05-26
     ******************************************************************************/
    void StuckState::GeneratePaths()
    {
        // X is easting, Y is altitude, Z is northing. (Minecraft coordinates)
        std::vector<numops::CoordinatePoint<double>> vWaypointOffsetsFromObstacle{
            {.tX = 4, .tZ = -4},     // eRightNav
            {.tX = 4, .tZ = 4},      // eRightGoal
            {.tX = -4, .tZ = -4},    // eLeftNav
            {.tX = -4, .tZ = 4},     // eLeftGoal
        };
        // Rotate the relative offsets to the rover's frame.
        numops::CoordinateFrameRotate3D(vWaypointOffsetsFromObstacle, 0, m_dOriginalHeading, 0);
        // Convert to navigation waypoints
        std::vector<geoops::Waypoint> vWaypoints;
        vWaypoints.reserve(vWaypointOffsetsFromObstacle.size());
        int nID = -100;
        for (const numops::CoordinatePoint<double>& stOffset : vWaypointOffsetsFromObstacle)
        {
            geoops::UTMCoordinate stWaypointPos = geoops::ConvertGPSToUTM(m_stOriginalPosition);
            stWaypointPos.dEasting += stOffset.tX;
            stWaypointPos.dNorthing += stOffset.tZ;
            vWaypoints.emplace_back(stWaypointPos, geoops::WaypointType::eNavigationWaypoint, 0, nID--);
        }
        // Set IDs of the waypoints so that NavigatingState can parse them correctly.
        // TODO: Make this not hard coded!!!
        assert(vWaypoints.size() == 4);
        // Create and save paths from generated waypoints.
        m_vRightPath = {vWaypoints[0], vWaypoints[1]};
        m_vLeftPath  = {vWaypoints[2], vWaypoints[3]};
    }

    /******************************************************************************
     * @brief Trigger an event in the state machine. Returns the next state.
     *
     * @param eEvent - The event to trigger.
     * @return std::shared_ptr<State> - The next state.
     *
     * @author Eli Byrd (edbgkk@mst.edu), clayjay3 (claytonraycowen@gmail.com), OcelotEmpire (hobbz.pi@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    States StuckState::TriggerEvent(Event eEvent)
    {
        // Create instance variables.
        States eNextState       = States::eStuck;
        bool bCompleteStateExit = true;

        switch (eEvent)
        {
            case Event::eStart:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "StuckState: Handling Start event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                break;
            }
            case Event::eAbort:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "StuckState: Handling Abort event.");
                // Send multimedia command to update state display.
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eAutonomy);
                // Change state.
                eNextState = States::eIdle;
                break;
            }
            case Event::eReverse:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "StuckState: Handling Reverse event.");
                // Change state.
                eNextState = States::eReversing;
                break;
            }
            case Event::eUnstuck:
            {
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "StuckState: Handling Unstuck event.");
                if (m_bReachedGoal)
                {
                    // Resume the state that originally triggered StuckState.
                    eNextState = m_eTriggeringState;
                }
                else
                {
                    eNextState = States::eNavigating;
                }
                break;
            }
            default:
            {
                LOG_WARNING(logging::g_qSharedLogger, "StuckState: Handling unknown event.");
                eNextState = States::eIdle;
                break;
            }
        }

        if (eNextState != States::eStuck)
        {
            LOG_INFO(logging::g_qSharedLogger, "StuckState: Transitioning to {} State.", StateToString(eNextState));

            // Exit the current state
            if (bCompleteStateExit)
            {
                Exit();
            }
        }

        return eNextState;
    }

    /******************************************************************************
     * @brief Checks if the rover is approximately in the same position.
     *
     * @note The threshold that defines how far away we need to be from the original point to be considered
     *  a different position is constants::STUCK_SAME_POINT_PROXIMITY.
     *
     * @param stLastPosition - Original position the rover was located.
     * @param stCurrPosition - Current position the rover is located.
     * @return true - The rover is in the same position.
     * @return false - The rover is in a different position.
     *
     * @author Jason Pittman (jspencerpittman@gmail.com)
     * @date 2024-02-14
     ******************************************************************************/
    bool StuckState::SamePosition(const geoops::GPSCoordinate& stOriginalPosition, const geoops::GPSCoordinate& stCurrPosition)
    {
        double dDistance = geoops::CalculateGeoMeasurement(stOriginalPosition, stCurrPosition).dDistanceMeters;
        return dDistance <= constants::STUCK_SAME_POINT_PROXIMITY;
    }

    // TODO: Make this not hard coded!!!
    bool StuckState::IsStuckWaypoint(const int nID)
    {
        return nID <= -100 && nID > -104;
    }

    // TODO: Make this not hard coded!!!
    bool StuckState::IsStuckWaypointGoal(const int nID)
    {
        return nID == -101 || nID == -103;
    }

}    // namespace statemachine
