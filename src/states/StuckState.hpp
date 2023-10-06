/******************************************************************************
 * @brief Stuck State Implementation for Autonomy State Machine
 *
 * @file StuckState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../AutonomyGlobals.h"
#include "../AutonomyLogging.h"

/******************************************************************************
 * @brief Stuck State Handler
 *
 *        Primarily the Stuck State Handler, handles the process of attempting
 *        to get the Rover unstuck when we detect that it has become stuck.
 *
 *        It also listens for state events that pertain to the Stuck
 *        State and calls the appropriate transition handler to transition
 *        states as needed.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct StuckState : sc::simple_state<StuckState, StateMachine>
{
        StuckState() { LOG_INFO(logging::g_qSharedLogger, "In State: Stuck"); }

        typedef mpl::list<sc::custom_reaction<Stuck_AbortTransition>, sc::custom_reaction<Stuck_ReverseTransition>> reactions;

        sc::result react(const Stuck_AbortTransition& event)
        {
            (void) event;    // Will be removed in new implementation of State Machine that doesn't require boost.
            return transit<AbortState>();
        }

        sc::result react(const Stuck_ReverseTransition& event)
        {
            (void) event;    // Will be removed in new implementation of State Machine that doesn't require boost.
            return transit<ReverseState>();
        }
};

/******************************************************************************
 * @brief Stuck State - Transition to Abort
 *
 *        When the state machine reaches the 'Abort' transition handler,
 *        Autonomy will stop all processes and transition to the Abort State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Stuck_AbortTransition : sc::event<Stuck_AbortTransition>
{
        Stuck_AbortTransition() { LOG_INFO(logging::g_qSharedLogger, "In Transition: Stuck (Abort)"); }
};

/******************************************************************************
 * @brief Stuck State - Transition to Reverse
 *
 *        When the state machine reaches the 'Reverse' transition handler,
 *        Autonomy will attempt to become unstuck by navigating in reverse.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Stuck_ReverseTransition : sc::event<Stuck_ReverseTransition>
{
        Stuck_ReverseTransition() { LOG_INFO(logging::g_qSharedLogger, "In Transition: Stuck (Reverse)"); }
};
