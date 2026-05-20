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
    void StuckState::Start()
    {
        // Schedule the next run of the state's logic
        LOG_INFO(logging::g_qSharedLogger, "StuckState: Scheduling next run of state logic.");

        // Initialize member variables.
        m_dOriginalHeading     = 0;
        m_bIsCurrentlyAligning = false;
        m_eAttemptType         = AttemptType::eReverseCurrentHeading;

        // Store the state that got stuck and triggered a stuck event.
        m_eTriggeringState = globals::g_pStateMachineHandler->GetPreviousState();

        // Store the postion and heading where the rover get stuck.
        geoops::RoverPose stStartRoverPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();
        m_stOriginalPosition               = stStartRoverPose.GetGPSCoordinate();
        m_dOriginalHeading                 = stStartRoverPose.GetCompassHeading();
        // Get state start time.
        m_tmStuckStartTime = std::chrono::system_clock::now();

        // Stop drivetrain.
        globals::g_pDriveBoard->SendStop();

        // Declare area in front of rover as an obstacle
        DeclareObstacle();
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
        LOG_INFO(logging::g_qConsoleLogger, "Entering State: {}", ToString());

        m_bInitialized = false;

        if (!m_bInitialized)
        {
            Start();
            m_bInitialized = true;
        }
    }

    /******************************************************************************
     * @brief Run the state machine. Returns the next state.
     *
     * @author Eli Byrd (edbgkk@mst.edu), Jason Pittman (jspencerpittman@gmail.com), clayjay3 (claytonraycowen@gmail.com), Sam Nolte (samnolte0302@gmail.com)
     * @date 2024-01-17
     ******************************************************************************/
    void StuckState::Run()
    {
        // Submit logger message.
        LOG_DEBUG(logging::g_qSharedLogger, "StuckState: Running state-specific behavior.");

        // Store the current postion and heading.
        geoops::RoverPose stCurrentRoverPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();
        // Get current time.
        std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();

        // Check if we are unstuck from our starting spot.
        if (!this->SamePosition(m_stOriginalPosition, stCurrentRoverPose.GetGPSCoordinate()))
        {
            // Submit logger message.
            LOG_NOTICE(logging::g_qSharedLogger,
                       "StuckState: Rover has successfully unstuckith itself! A total of {} seconds was wasted being stuck.",
                       std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - m_tmStuckStartTime).count());
            // Handing unstuck event. Destroy this unstuck state.
            globals::g_pStateMachineHandler->HandleEvent(Event::eUnstuck, false);
        }
        else
        {
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
                        m_dOriginalHeading = stCurrentRoverPose.GetCompassHeading();
                        // Update start time.
                        m_tmAlignStartTime = std::chrono::system_clock::now();
                    }
                    else
                    {
                        // Calculate time elapsed since realignment was started.
                        double dTimeElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - m_tmAlignStartTime).count() / 1000.0;
                        // Calculate the goal realignment heading.
                        double dGoalHeading = numops::InputAngleModulus<double>(m_dOriginalHeading + constants::STUCK_ALIGN_DEGREES, 0, 360);
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
                        m_dOriginalHeading = stCurrentRoverPose.GetCompassHeading();
                        // Update start time.
                        m_tmAlignStartTime = std::chrono::system_clock::now();
                    }
                    else
                    {
                        // Calculate time elapsed since realignment was started.
                        double dTimeElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(tmCurrentTime - m_tmAlignStartTime).count() / 1000.0;
                        // Calculate the goal realignment heading.
                        double dGoalHeading = numops::InputAngleModulus<double>(m_dOriginalHeading - constants::STUCK_ALIGN_DEGREES, 0, 360);
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
    }

    /******************************************************************************
     * @brief Trigger an event in the state machine. Returns the next state.
     *
     * @param eEvent - The event to trigger.
     * @return std::shared_ptr<State> - The next state.
     *
     * @author Eli Byrd (edbgkk@mst.edu), clayjay3 (claytonraycowen@gmail.com)
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
                globals::g_pMultimediaBoard->SendLightingState(MultimediaBoard::MultimediaBoardLightingState::eOff);
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
                // Modify referenced rover path to reflect new obstacle
                ModifyPath();
                // Submit logger message.
                LOG_INFO(logging::g_qSharedLogger, "StuckState: Handling Unstuck event.");
                // Change state back to the state that originally got stuck.
                eNextState = m_eTriggeringState;
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

    /******************************************************************************
     * @brief Adds the area in front of the rover as an obstacle by modifying the area's trav-score in LiDAR data
     *
     * @note Uses constants::STUCK_OBSTACLE_RADIUS for the declared obstacle's radius and constants::STUCK_OBSTACLE_DISTANCE for distance in front of the rover that is
     * declared an obstacle
     *
     * @author Sam Nolte (samnolte0302@gmail.com)
     * @date 2026-01-27
     ******************************************************************************/
    void StuckState::DeclareObstacle()
    {
        // Convert from compass degrees to unit circle radians.
        double dRadians = (90.0 - m_dOriginalHeading) * M_PI / 180.0;
        if (dRadians < 0)
        {
            dRadians += 2 * M_PI;
        }

        // Get the obstacle's origin.
        geoops::UTMCoordinate stObstaclePosition = globals::g_pStateMachineHandler->SmartRetrieveRoverPose().GetUTMCoordinate();
        stObstaclePosition.dEasting += std::cos(dRadians) * constants::STUCK_OBSTACLE_DISTANCE;
        stObstaclePosition.dNorthing += std::sin(dRadians) * constants::STUCK_OBSTACLE_DISTANCE;

        // Add to waypoint handler obstacle list
        globals::g_pWaypointHandler->AddObstacle(stObstaclePosition, constants::STUCK_OBSTACLE_RADIUS);
    }

    /******************************************************************************
     * @brief Modify stored path to reflect new obstacle and store it for use in returning state
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
    void StuckState::ModifyPath()
    {
        // Get the rover's pose AFTER stuck state has ran
        geoops::RoverPose stCurrentRoverPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();

        // Get the obstacle's origin
        int nObstacleIndex                       = globals::g_pWaypointHandler->GetObstaclesCount();
        geoops::UTMCoordinate stObstaclePosition = globals::g_pWaypointHandler->RetrieveObstacleAtIndex(nObstacleIndex - 1).GetUTMCoordinate();

        // Get saved rover path
        std::vector<geoops::Waypoint> vRefPath          = globals::g_pWaypointHandler->RetrievePath("stuckPath");
        std::vector<geoops::Waypoint> vRevSearchRefPath = (m_eTriggeringState == States::eSearchPattern) ? vRefPath : std::vector<geoops::Waypoint>();

        if (vRefPath.empty())
        {
            LOG_WARNING(logging::g_qSharedLogger, "Stuck state received empty path to modify!");
            globals::g_pWaypointHandler->StorePath("unstuckPath", vRefPath);
            if (m_eTriggeringState == States::eSearchPattern)
            {
                globals::g_pWaypointHandler->StorePath("RevSpiralPath", vRevSearchRefPath);
            }
            return;
        }

        // Get the point on the obstacle's border which the rover came from.
        double dHeadingRad = (90.0 - m_dOriginalHeading) * M_PI / 180.0;
        if (dHeadingRad < 0)
        {
            dHeadingRad += 2 * M_PI;
        }

        // Grab starting coordinate and goal (edge of obstacle behind rover) coordinate.
        geoops::UTMCoordinate stStartCoordinate = stCurrentRoverPose.GetUTMCoordinate();
        geoops::UTMCoordinate stGoalCoordinate  = stObstaclePosition;

        // Get goal coordinate easting and northing.
        stGoalCoordinate.dEasting -= std::cos(dHeadingRad) * constants::STUCK_OBSTACLE_RADIUS;
        stGoalCoordinate.dNorthing -= std::sin(dHeadingRad) * constants::STUCK_OBSTACLE_RADIUS;

        std::vector<geoops::Waypoint> vSplicePathCoordinates;
        std::vector<geoops::Waypoint>::iterator it = vRefPath.begin();

        int nPointsAdded                           = 0;
        int nPointsRemoved                         = 0;

        // TODO: closest point is not necessarily current point in path (thinking of seach state drifting)
        // Find the node closest to the rover's current position to determine what has already been passed.
        std::vector<geoops::Waypoint>::iterator itClosest = vRefPath.begin();
        double dMinDistSq                                 = std::numeric_limits<double>::max();
        for (std::vector<geoops::Waypoint>::iterator itSearch = vRefPath.begin(); itSearch != vRefPath.end(); ++itSearch)
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
        if (itClosest != vRefPath.begin())
        {
            nPointsRemoved += std::distance(vRefPath.begin(), itClosest);
            it = vRefPath.erase(vRefPath.begin(), itClosest);
        }

        LOG_INFO(logging::g_qSharedLogger, "Stuck state path filter removed {} elements: ", nPointsRemoved);
        nPointsRemoved = 0;

        // If rover is in the obstacle, then path it out first and connect it to previous path
        double dx = stCurrentRoverPose.GetUTMCoordinate().dEasting - stObstaclePosition.dEasting;
        double dy = stCurrentRoverPose.GetUTMCoordinate().dNorthing - stObstaclePosition.dNorthing;
        if (dx * dx + dy * dy <= constants::STUCK_OBSTACLE_RADIUS * constants::STUCK_OBSTACLE_RADIUS)
        {
            geoops::UTMCoordinate stFirstNodeOfOriginalPath = vRefPath.front().GetUTMCoordinate();

            // Splice in a new path from rover's current location to outside of the obstacle
            vSplicePathCoordinates = globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler, stStartCoordinate, stGoalCoordinate);
            it                     = vRefPath.insert(vRefPath.begin(), vSplicePathCoordinates.begin(), vSplicePathCoordinates.end());
            it += vSplicePathCoordinates.size();
            nPointsAdded += vSplicePathCoordinates.size();

            // Splice in a new path from outside of the obstacle to the end of the previous path
            stStartCoordinate      = (vSplicePathCoordinates.size() >= 2) ? std::prev(vSplicePathCoordinates.end())->GetUTMCoordinate() : stStartCoordinate;
            vSplicePathCoordinates = globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler, stStartCoordinate, stFirstNodeOfOriginalPath);
            if (vSplicePathCoordinates.size() >= 3)
            {
                it = vRefPath.insert(it, std::next(vSplicePathCoordinates.begin()), std::prev(vSplicePathCoordinates.end()));
                nPointsAdded += vSplicePathCoordinates.size() - 2;
            }
            LOG_INFO(logging::g_qSharedLogger, "Stuck State was within obstacle: {} nodes added, {} nodes removed", nPointsAdded, nPointsRemoved);
        }

        // Else if rover is not inside obstacle, then just connect it to previous path
        else
        {
            // Splice in a new path from rover's current location to the end of the previous path
            stGoalCoordinate       = vRefPath.front().GetUTMCoordinate();
            vSplicePathCoordinates = globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler, stStartCoordinate, stGoalCoordinate);
            if (vSplicePathCoordinates.size() >= 2)
            {
                it = vRefPath.insert(vRefPath.begin(), vSplicePathCoordinates.begin(), std::prev(vSplicePathCoordinates.end()));
                nPointsAdded += vSplicePathCoordinates.size() - 1;
            }
            LOG_INFO(logging::g_qSharedLogger, "Stuck State was not within obstacle: {} nodes added, {} nodes removed", nPointsAdded, nPointsRemoved);
        }

        // Remove all points that are in stuck zone and re path-plan deleted path segments.
        SplicePath(vRefPath, stObstaclePosition);
        if (m_eTriggeringState == States::eSearchPattern)
        {
            SplicePath(vRevSearchRefPath, stObstaclePosition);
        }

        // Save path for use in returning state
        globals::g_pWaypointHandler->StorePath("unstuckPath", vRefPath);
        if (m_eTriggeringState == States::eSearchPattern)
        {
            globals::g_pWaypointHandler->StorePath("RevSpiralPath", vRevSearchRefPath);
        }
    }

    /******************************************************************************
     * @brief Removes all points within stuck area in path and re-path plans all deleted paths segments
     *
     *
     * @author Sam Nolte (samnolte0302@gmail.com)
     * @date 2026-05-19
     ******************************************************************************/
    void StuckState::SplicePath(std::vector<geoops::Waypoint>& vPath, geoops::UTMCoordinate stObstaclePosition)
    {
        if (vPath.size() < 2)
        {
            return;
        }

        geoops::RoverPose stCurrentRoverPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();
        geoops::UTMCoordinate stStartCoordinate;
        geoops::UTMCoordinate stGoalCoordinate;
        std::vector<geoops::Waypoint> vSplicePathCoordinates;
        std::vector<geoops::Waypoint>::iterator it = vPath.begin();

        int nPointsAdded                           = 0;
        int nPointsRemoved                         = 0;

        bool bLastDeleted                          = false;
        while (it != std::prev(vPath.end()))
        {
            double dDifferenceX = it->GetUTMCoordinate().dEasting - stObstaclePosition.dEasting;
            double dDifferenceY = it->GetUTMCoordinate().dNorthing - stObstaclePosition.dNorthing;

            // If path coord is inside stuck zone, then remove it.
            if (dDifferenceX * dDifferenceX + dDifferenceY * dDifferenceY <= constants::STUCK_OBSTACLE_RADIUS * constants::STUCK_OBSTACLE_RADIUS)
            {
                bLastDeleted = true;
                it           = vPath.erase(it);
                ++nPointsRemoved;
            }
            // If the previous node was deleted, then connect the dots correctly by splicing a new path in between.
            else if (bLastDeleted)
            {
                // Plan a new path to the next remaining path node.
                stStartCoordinate      = (it != vPath.begin()) ? std::prev(it)->GetUTMCoordinate() : stCurrentRoverPose.GetUTMCoordinate();
                stGoalCoordinate       = it->GetUTMCoordinate();
                vSplicePathCoordinates = globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler, stStartCoordinate, stGoalCoordinate);
                if (vSplicePathCoordinates.size() >= 3)
                {
                    bLastDeleted = false;
                    it           = vPath.insert(it, std::next(vSplicePathCoordinates.begin()), std::prev(vSplicePathCoordinates.end()));
                    it += vSplicePathCoordinates.size() - 1;
                    nPointsAdded += vSplicePathCoordinates.size() - 2;
                }
            }
            else
            {
                ++it;
            }
        }
        // If last node is deleted and while loop ends then still connect the path to goal
        if (bLastDeleted)
        {
            // Plan a new path to the next remaining path node
            stStartCoordinate      = std::prev(it)->GetUTMCoordinate();
            stGoalCoordinate       = it->GetUTMCoordinate();
            vSplicePathCoordinates = globals::g_pGeoPlanner->PlanPath(globals::g_pLiDARHandler, stStartCoordinate, stGoalCoordinate);
            vPath.insert(it, std::next(vSplicePathCoordinates.begin()), std::prev(vSplicePathCoordinates.end()));
            nPointsAdded += vSplicePathCoordinates.size() - 2;
        }

        LOG_INFO(logging::g_qSharedLogger, "Stuck State Splice modified path: {} nodes added, {} nodes removed", nPointsAdded, nPointsRemoved);
    }
}    // namespace statemachine
