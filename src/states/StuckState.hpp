/******************************************************************************
 * @brief 
 * 
 * @file StuckState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 * 
 * @copyright Copyright MRDT 2023 - All Rights Reserved
******************************************************************************/

/******************************************************************************
 * @brief Stuck State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct StuckState : sc::simple_state<StuckState, StateMachine>
{
        StuckState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Stuck"; }

        typedef mpl::list<sc::custom_reaction<Stuck_AbortTransition>, sc::custom_reaction<Stuck_ReverseTransition>> reactions;

        sc::result react(const Stuck_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Stuck_ReverseTransition& event) { return transit<ReverseState>(); }
};

struct Stuck_AbortTransition : sc::event<Stuck_AbortTransition>
{
        Stuck_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Stuck (Abort)"; }
};

struct Stuck_ReverseTransition : sc::event<Stuck_ReverseTransition>
{
        Stuck_ReverseTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Stuck (Reverse)"; }
};