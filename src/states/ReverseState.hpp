/******************************************************************************
 * @brief Reverse State Implementation for Autonomy State Machine
 *
 * @file ReverseState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../AutonomyGlobals.h"
#include "../AutonomyLogging.h"

/******************************************************************************
 * @brief Reverse State Handler
 *
 *        Primarily the Reverse State Handler, handles the navigation of
 *        the Rover away from a gate or marker. However, in the event the
 *        Rover becomes stuck, one of the methods we can use to attept to
 *        become unstuck, is to navigate in reverse.
 *
 *        It also listens for state events that pertain to the Reverse
 *        State and calls the approprate transition handler to transition
 *        states as needed.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct ReverseState : sc::simple_state<ReverseState, StateMachine>
{
        ReverseState() { LOG_INFO(g_qSharedLogger, "In State: Reverse"); }

        typedef mpl::list<sc::custom_reaction<Reverse_ContinueTransition>, sc::custom_reaction<Reverse_AbortTransition>, sc::custom_reaction<Reverse_StuckTransition>>
            reactions;

        sc::result react(const Reverse_ContinueTransition& event)
        {
            // If - Idle
            //      return transit<IdleState>();

            // If - Navigate
            //      return transit<NavigateState>();

            // If - Search Pattern
            //      return transit<SearchPatternState>();

            // If - Avoidance
            //      return transit<AvoidanceState>();

            // If - Approaching Marker
            //      return transit<ApproachingMarkerState>();

            // If - Approaching Gate
            //      return transit<ApproachingGateState>();

            return transit<AbortState>();
        }

        sc::result react(const Reverse_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Reverse_StuckTransition& event) { return transit<StuckState>(); }
};

/******************************************************************************
 * @brief Reverse State - Transition to Continue
 *
 *        When the state machine reaches the 'Continue' transition handler,
 *        Autonomy will proceed to the state it was in prior to transitioning
 *        to Reverse.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Reverse_ContinueTransition : sc::event<Reverse_ContinueTransition>
{
        Reverse_ContinueTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Reverse (Continue)"); }
};

/******************************************************************************
 * @brief Reverse State - Transition to Abort
 *
 *        When the state machine reaches the 'Abort' transition handler,
 *        Autonomy will stop all processes and transition to the Abort State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Reverse_AbortTransition : sc::event<Reverse_AbortTransition>
{
        Reverse_AbortTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Reverse (Abort)"); }
};

/******************************************************************************
 * @brief Reverse State - Transition to Stuck
 *
 *        When the state machine reaches the 'Stuck' transition handler,
 *        Autonomy will navigate to the Stuck State and attenpt a series
 *        of algorithms to become unstuck.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Reverse_StuckTransition : sc::event<Reverse_StuckTransition>
{
        Reverse_StuckTransition() { LOG_INFO(g_qSharedLogger, "In Transition: Reverse (Stuck)"); }
};
