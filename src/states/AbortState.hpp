/******************************************************************************
 * @brief Abort State Implementation for Autonomy State Machine
 *
 * @file AbortState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../AutonomyGlobals.h"
#include "../AutonomyLogging.h"

/******************************************************************************
 * @brief Abort State Handler
 *
 *        Primarily the Abort State Handler, handles the stopping of any
 *        navigation commands and exiting of the application in the event of
 *        a catastrophic system failure during the Autonomy Mission.
 *
 *        However, it also listens for state events that pertain to the Abort
 *        State and calls the appropriate transition handler to transition
 *        states as needed.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct AbortState : sc::simple_state<AbortState, StateMachine>
{
        AbortState() { LOG_INFO(logging::g_qSharedLogger, "In State: Abort"); }

        typedef mpl::list<sc::custom_reaction<Abort_RestartTransition>, sc::custom_reaction<Abort_ExitTransition>> reactions;

        sc::result react(const Abort_RestartTransition& event)
        {
            (void) event;    // Will be removed in new implementation of State Machine that doesn't require boost.
            return transit<IdleState>();
        }

        sc::result react(const Abort_ExitTransition& event)
        {
            (void) event;    // Will be removed in new implementation of State Machine that doesn't require boost.
            return transit<IdleState>();
        }
};

/******************************************************************************
 * @brief Abort State - Transition to Restart
 *
 *        When the state machine reaches the 'Restart' transition
 *        handler, Autonomy will stop all navigation processes and attempt
 *        to reboot the State Machine.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Abort_RestartTransition : sc::event<Abort_RestartTransition>
{
        Abort_RestartTransition() { LOG_INFO(logging::g_qSharedLogger, "In Transition: Abort (Restart)"); }
};

/******************************************************************************
 * @brief Abort State - Transition to Exit
 *
 *        When the state machine reaches the 'Exit' transition handler,
 *        Autonomy will stop all processes and cleanly exit the application.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Abort_ExitTransition : sc::event<Abort_ExitTransition>
{
        Abort_ExitTransition() { LOG_INFO(logging::g_qSharedLogger, "In Transition: Abort (Exit)"); }
};
