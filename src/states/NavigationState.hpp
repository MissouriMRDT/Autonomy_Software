/******************************************************************************
 * @brief Navigation State Implementation for Autonomy State Machine
 *
 * @file NavigationState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Navigation State Handler
 *
 *        Primarily the Navigation State Handler, handles the navigation of
 *        the Rover from point to point. It is the state which Autonomy
 *        stays in most of the time.
 *
 *        It also listens for state events that pertain to the Navigation
 *        State and calls the approprate transition handler to transition
 *        states as needed.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct NavigationState : sc::simple_state<NavigationState, StateMachine>
{
        NavigationState() { LOG_INFO(g_qSharedLogger, "In State: Navigation"); }

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

            return transit<AbortState>();
        }

        sc::result react(const Navigation_SeenTagTransition& event)
        {
            // If - Marker
            //      return transit<ApproachingMarkerState>();

            // If - Gate
            //      return transit<ApproachingGateState>();

            return transit<AbortState>();
        }

        sc::result react(const Navigation_ObstacleAvoidanceTransition& event) { return transit<AvoidanceState>(); }
};

/******************************************************************************
 * @brief Navigation State - Transition to New Waypoint
 *
 *        When the state machine reaches the 'New Waypoint' transition handler,
 *        Autonomy will stop navigating to the current position and restart
 *        by navigating to the new position it recieved.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Navigation_NewWaypointTransition : sc::event<Navigation_NewWaypointTransition>
{
        Navigation_NewWaypointTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Navigation (New Waypoint)"); }
};

/******************************************************************************
 * @brief Navigation State - Transition to Abort
 *
 *        When the state machine reaches the 'Abort' transition handler,
 *        Autonomy will stop all processes and transition to the Abort State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Navigation_AbortTransition : sc::event<Navigation_AbortTransition>
{
        Navigation_AbortTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Navigation (Abort)"); }
};

/******************************************************************************
 * @brief Navigation State - Transition to Stuck
 *
 *        When the state machine reaches the 'Stuck' transition handler,
 *        Autonomy will navigate to the Stuck State and attenpt a series
 *        of algorithms to become unstuck.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Navigation_StuckTransition : sc::event<Navigation_StuckTransition>
{
        Navigation_StuckTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Navigation (Stuck)"); }
};

/******************************************************************************
 * @brief Navigation State - Transition to Reached GPS
 *
 *        When the state machine reaches the 'Reached GPS' transition handler,
 *        Autonomy will either stop change the LED Panel and navigate back to
 *        Idle because this was a GPS-Only leg of the competition. Or, it will
 *        proceed into Seach Pattern because we need to now search for an
 *        ArUco Tag within a 20 meter radius.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Navigation_ReachedGPSTransition : sc::event<Navigation_ReachedGPSTransition>
{
        Navigation_ReachedGPSTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Navigation (Reached GPS)"); }
};

/******************************************************************************
 * @brief Navigation State - Transition to Seen Tag
 *
 *        When the state machine reaches the 'Seen Tag' transition handler,
 *        Autonomy will navigate into to appropraite approach state based on
 *        the ID of the tag it recognized.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Navigation_SeenTagTransition : sc::event<Navigation_SeenTagTransition>
{
        Navigation_SeenTagTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Navigation (Seen Tag)"); }
};

/******************************************************************************
 * @brief Navigation State - Transition to Obstacle Avoidance
 *
 *        When the state machine reaches the 'Obstacle Avoidance' transition
 *        handler, Autonomy will switch to Obstacle Avoidance to change its
 *        route to go around whatever obstacle was detected.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Navigation_ObstacleAvoidanceTransition : sc::event<Navigation_ObstacleAvoidanceTransition>
{
        Navigation_ObstacleAvoidanceTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Navigation (Obstacle Avoidance)"); }
};
