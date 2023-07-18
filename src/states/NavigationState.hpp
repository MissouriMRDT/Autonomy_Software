/******************************************************************************
 * @brief 
 * 
 * @file NavigationState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 * 
 * @copyright Copyright MRDT 2023 - All Rights Reserved
******************************************************************************/

/******************************************************************************
 * @brief Navigation State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct NavigationState : sc::simple_state<NavigationState, StateMachine>
{
        NavigationState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Navigation"; }

        typedef mpl::list<sc::custom_reaction<Navigation_NewWaypointTransition>,
                          sc::custom_reaction<Navigation_AbortTransition>,
                          sc::custom_reaction<Navigation_StuckTransition>,
                          sc::custom_reaction<Navigation_ReachedGPSTransition>,
                          sc::custom_reaction<Navigation_SeenTagTransition>,
                          sc::custom_reaction<Navigation_ObstacleAvoidanceTransition>>
            reactions;

        sc::result react(const Navigation_NewWaypointTransition& event) { return transit<NavigationState>(); }

        sc::result react(const Navigation_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Navigation_StuckTransition& event) { return transit<StuckState>(); }

        sc::result react(const Navigation_ReachedGPSTransition& event)
        {
            // If - GPS Only
            //      return transit<IdleState>();

            // If - ArUco Search
            //      return transit<SearchPatternState>();

            return transit<IdleState>();
        }

        sc::result react(const Navigation_SeenTagTransition& event)
        {
            // If - Marker
            //      return transit<ApproachingMarkerState>();

            // If - Gate
            //      return transit<ApproachingGateState>();

            return transit<IdleState>();
        }

        sc::result react(const Navigation_ObstacleAvoidanceTransition& event) { return transit<AvoidanceState>(); }
};

struct Navigation_NewWaypointTransition : sc::event<Navigation_NewWaypointTransition>
{
        Navigation_NewWaypointTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Navigation (New Waypoint)"; }
};

struct Navigation_AbortTransition : sc::event<Navigation_AbortTransition>
{
        Navigation_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Navigation (Abort)"; }
};

struct Navigation_StuckTransition : sc::event<Navigation_StuckTransition>
{
        Navigation_StuckTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Navigation (Stuck)"; }
};

struct Navigation_ReachedGPSTransition : sc::event<Navigation_ReachedGPSTransition>
{
        Navigation_ReachedGPSTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Navigation (Reached GPS)"; }
};

struct Navigation_SeenTagTransition : sc::event<Navigation_SeenTagTransition>
{
        Navigation_SeenTagTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Navigation (Seen Tag)"; }
};

struct Navigation_ObstacleAvoidanceTransition : sc::event<Navigation_ObstacleAvoidanceTransition>
{
        Navigation_ObstacleAvoidanceTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Navigation (Obstacle Avoidance)"; }
};