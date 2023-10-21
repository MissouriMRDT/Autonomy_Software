/******************************************************************************
 * @brief Idle State Implementation for Autonomy State Machine
 *
 * @file IdleState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../AutonomyGlobals.h"
#include "../AutonomyLogging.h"

/******************************************************************************
 * @brief Idle State Handler
 *
 *        Primarily the Idle State Handler, handles the routing that
 *        occurs at the beginning of each leg. This is also the state
 *        which Autonomy will sit in if moving.
 *
 *        It also listens for state events that pertain to the Idle State
 *        and calls the appropriate transition handler to transition states
 *        as needed.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct IdleState : sc::simple_state<IdleState, StateMachine>
{
        IdleState() { LOG_INFO(logging::g_qSharedLogger, "In State: Idle"); }

        typedef mpl::list<sc::custom_reaction<Idle_AbortTransition>, sc::custom_reaction<Idle_NavigatingTransition>, sc::custom_reaction<Idle_ReverseTransition>>
            reactions;

        sc::result react(const Idle_AbortTransition& event)
        {
            (void) event;    // Will be removed in new implementation of State Machine that doesn't require boost.
            return transit<AbortState>();
        }

        sc::result react(const Idle_NavigatingTransition& event)
        {
            (void) event;    // Will be removed in new implementation of State Machine that doesn't require boost.
            return transit<NavigationState>();
        }

        sc::result react(const Idle_ReverseTransition& event)
        {
            (void) event;    // Will be removed in new implementation of State Machine that doesn't require boost.
            return transit<ReverseState>();
        }
};

/******************************************************************************
 * @brief Idle State - Transition to Abort
 *
 *        When the state machine reaches the 'Abort' transition handler,
 *        Autonomy will stop all processes and transition to the Abort State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Idle_AbortTransition : sc::event<Idle_AbortTransition>
{
        Idle_AbortTransition() { LOG_INFO(logging::g_qSharedLogger, "In Transition: Idle (Abort)"); }
};

/******************************************************************************
 * @brief Idle State - Transition to Navigating
 *
 *        When the state machine reaches the 'Navigating' transition handler,
 *        Autonomy will begin navigating it's way towards the specified GPS
 *        coordinates while looking for Markers or Gates if in the appropriate
 *        leg of the Competition.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Idle_NavigatingTransition : sc::event<Idle_NavigatingTransition>
{
        Idle_NavigatingTransition() { LOG_INFO(logging::g_qSharedLogger, "In Transition: Idle (Navigating)"); }
};

/******************************************************************************
 * @brief Idle State - Transition to Reverse
 *
 *        When the state machine reaches the 'Reverse' transition handler,
 *        Autonomy will use the reverse state to navigate away from the
 *        current marker or gate that it is currently at.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Idle_ReverseTransition : sc::event<Idle_ReverseTransition>
{
        Idle_ReverseTransition() { LOG_INFO(logging::g_qSharedLogger, "In Transition: Idle (Reverse)"); }
};
