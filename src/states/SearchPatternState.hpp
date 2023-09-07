/******************************************************************************
 * @brief Search Pattern State Implementation for Autonomy State Machine
 *
 * @file SearchPatternState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../AutonomyGlobals.h"
#include "../AutonomyLogging.h"

/******************************************************************************
 * @brief Search Pattern State Handler
 *
 *        Primarily the Search Pattern State Handler, handles the search
 *        algorithmn to attept and find a tag after reaching the GPS Point
 *        for a Gate or Marker Leg.
 *
 *        It also listens for state events that pertain to the Search
 *        Pattern State and calls the approprate transition handler to
 *        transition states as needed.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct SearchPatternState : sc::simple_state<SearchPatternState, StateMachine>
{
        SearchPatternState() { LOG_INFO(g_qSharedLogger, "In State: Search Pattern"); }

        typedef mpl::list<sc::custom_reaction<SeachPattern_GateSeenTransition>,
                          sc::custom_reaction<SeachPattern_ObstacleAvoidanceTransition>,
                          sc::custom_reaction<SeachPattern_MarkerSeenTransition>,
                          sc::custom_reaction<SeachPattern_AbortTransition>,
                          sc::custom_reaction<SeachPattern_StuckTransition>>
            reactions;

        sc::result react(const SeachPattern_GateSeenTransition& event)
        {
            (void) event;    // Will be removed in new implementation of State Machine that doesn't require boost.
            return transit<ApproachingGateState>();
        }

        sc::result react(const SeachPattern_ObstacleAvoidanceTransition& event)
        {
            (void) event;    // Will be removed in new implementation of State Machine that doesn't require boost.
            return transit<AvoidanceState>();
        }

        sc::result react(const SeachPattern_MarkerSeenTransition& event)
        {
            (void) event;    // Will be removed in new implementation of State Machine that doesn't require boost.
            return transit<ApproachingMarkerState>();
        }

        sc::result react(const SeachPattern_AbortTransition& event)
        {
            (void) event;    // Will be removed in new implementation of State Machine that doesn't require boost.
            return transit<AbortState>();
        }

        sc::result react(const SeachPattern_StuckTransition& event)
        {
            (void) event;    // Will be removed in new implementation of State Machine that doesn't require boost.
            return transit<StuckState>();
        }
};

/******************************************************************************
 * @brief Search Pattern State - Transition to Gate Seen
 *
 *        When the state machine reaches the 'Gate Seen' transition handler,
 *        Autonomy will transition into Approaching Gate and attempt to drive
 *        between the posts and stop within the required distance.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct SeachPattern_GateSeenTransition : sc::event<SeachPattern_GateSeenTransition>
{
        SeachPattern_GateSeenTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Search Pattern (Gate Seen)"); }
};

/******************************************************************************
 * @brief Search Pattern State - Transition to Obstacle Avoidance
 *
 *        When the state machine reaches the 'Obstacle Avoidance' transition
 *        handler, Autonomy will switch to Obstacle Avoidance to change its
 *        route to go around whatever obstacle was detected.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct SeachPattern_ObstacleAvoidanceTransition : sc::event<SeachPattern_ObstacleAvoidanceTransition>
{
        SeachPattern_ObstacleAvoidanceTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Search Pattern (Obstacle Avoidance)"); }
};

/******************************************************************************
 * @brief Search Pattern State - Transition to Marker Seen
 *
 *        When the state machine reaches the 'Marker Seen' transition handler,
 *        Autonomy will transition into Approaching Marker and attempt to stop
 *        within the required distance of the post.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct SeachPattern_MarkerSeenTransition : sc::event<SeachPattern_MarkerSeenTransition>
{
        SeachPattern_MarkerSeenTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Search Pattern (Marker Seen)"); }
};

/******************************************************************************
 * @brief Search Pattern State - Transition to Abort
 *
 *        When the state machine reaches the 'Abort' transition handler,
 *        Autonomy will stop all processes and transition to the Abort State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct SeachPattern_AbortTransition : sc::event<SeachPattern_AbortTransition>
{
        SeachPattern_AbortTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Search Pattern (Abort)"); }
};

/******************************************************************************
 * @brief Search Pattern State - Transition to Stuck
 *
 *        When the state machine reaches the 'Stuck' transition handler,
 *        Autonomy will navigate to the Stuck State and attenpt a series
 *        of algorithms to become unstuck.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct SeachPattern_StuckTransition : sc::event<SeachPattern_StuckTransition>
{
        SeachPattern_StuckTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Search Pattern (Stuck)"); }
};
