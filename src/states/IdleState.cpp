/******************************************************************************
 * @brief
 *
 * @file IdleState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-0716
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "./StateMachine.hpp"

struct IdleState : sc::simple_state<IdleState, StateMachine>
{
        IdleState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Idle\n"; }

        typedef mpl::list<sc::custom_reaction<Idle_AbortTransition>,
                          sc::custom_reaction<Idle_TakeoffTransition>,
                          sc::custom_reaction<Idle_NavigatingTransition>,
                          sc::custom_reaction<Idle_LandingTransition>,
                          sc::custom_reaction<Idle_ReverseTransition>>
            reactions;

        sc::result react(const Idle_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Idle_TakeoffTransition& event) { return transit<TakeoffState>(); }

        sc::result react(const Idle_NavigatingTransition& event) { return transit<NavigationState>(); }

        sc::result react(const Idle_LandingTransition& event) { return transit<LandingState>(); }

        sc::result react(const Idle_ReverseTransition& event) { return transit<ReverseState>(); }
};

struct Idle_AbortTransition : sc::event<Idle_AbortTransition>
{
        Idle_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Idle (Abort)\n"; }
};

struct Idle_TakeoffTransition : sc::event<Idle_TakeoffTransition>
{
        Idle_TakeoffTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Idle (Takeoff)\n"; }
};

struct Idle_NavigatingTransition : sc::event<Idle_NavigatingTransition>
{
        Idle_NavigatingTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Idle (Navigating)\n"; }
};

struct Idle_LandingTransition : sc::event<Idle_LandingTransition>
{
        Idle_LandingTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Idle (Landing)\n"; }
};

struct Idle_ReverseTransition : sc::event<Idle_NavigatingTransition>
{
        Idle_ReverseTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Idle (Reverse)\n"; }
};
