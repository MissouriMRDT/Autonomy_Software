/******************************************************************************
 * @brief Obstacle Avoidance State Implementation for Autonomy State Machine
 *
 * @file AvoidanceState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../AutonomyGlobals.h"

/******************************************************************************
 * @brief Obstacle Avoidance State Handler
 *
 *        Primarily the Obstacle Avoidance State Handler, handles the
 *        detection and navigation of the Rover around obstacles.
 *
 *        It also listens for state events that pertain to the Obstacle
 *        Avoidance State and calls the approprate transition handler to
 *        transition states as needed.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct AvoidanceState : sc::simple_state<AvoidanceState, StateMachine>
{
        AvoidanceState() { LOG_INFO(g_qSharedLogger, "In State: Avoidance"); }

        typedef mpl::
            list<sc::custom_reaction<Avoidance_EndAvoidanceTransition>, sc::custom_reaction<Avoidance_AbortTransition>, sc::custom_reaction<Avoidance_StuckTransition>>
                reactions;

        sc::result react(const Avoidance_EndAvoidanceTransition& event)
        {
            // If - Search Pattern
            //      return transit<SearchPatternState>();

            // If - Navigation
            //      return transit<NavigationState>();

            return transit<AbortState>();
        }

        sc::result react(const Avoidance_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Avoidance_StuckTransition& event) { return transit<StuckState>(); }
};

/******************************************************************************
 * @brief Obstacle Avoidance - Transition to End Avoidance
 *
 *        When the state machine reaches the 'End Avoidance' transition
 *        handler, Autonomy will transition into the state it was
 *        previously in.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Avoidance_EndAvoidanceTransition : sc::event<Avoidance_EndAvoidanceTransition>
{
        Avoidance_EndAvoidanceTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Avoidance (End Avoidance)"); }
};

/******************************************************************************
 * @brief Obstacle Avoidance State - Transition to Abort
 *
 *        When the state machine reaches the 'Abort' transition handler,
 *        Autonomy will stop all processes and transition to the Abort State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Avoidance_AbortTransition : sc::event<Avoidance_AbortTransition>
{
        Avoidance_AbortTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Avoidance (Abort)"); }
};

/******************************************************************************
 * @brief Obstacle Avoidance State - Transition to Stuck
 *
 *        When the state machine reaches the 'Stuck' transition handler,
 *        Autonomy will navigate to the Stuck State and attenpt a series
 *        of algorithms to become unstuck.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Avoidance_StuckTransition : sc::event<Avoidance_StuckTransition>
{
        Avoidance_StuckTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Avoidance (Stuck)"); }
};
