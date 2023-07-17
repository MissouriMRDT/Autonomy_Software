/******************************************************************************
 * @brief
 *
 * @file LandingState.cpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-0716
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "./StateMachine.hpp"

struct LandingState : sc::simple_state<LandingState, StateMachine>
{
        LandingState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Landing\n"; }

        typedef mpl::list<sc::custom_reaction<Landing_NoResponseTransition>,
                          sc::custom_reaction<Landing_DecendTransition>,
                          sc::custom_reaction<Landing_AbortTransition>,
                          sc::custom_reaction<Landing_LandedTransition>>
            reactions;

        sc::result react(const Landing_NoResponseTransition& event) { return transit<CrashedState>(); }

        sc::result react(const Landing_DecendTransition& event) { return transit<DecendState>(); }

        sc::result react(const Landing_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Landing_LandedTransition& event) { return transit<NavigationState>(); }
};

struct Landing_NoResponseTransition : sc::event<Landing_NoResponseTransition>
{
        Landing_NoResponseTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Landing (No Response))\n"; }
};

struct Landing_DecendTransition : sc::event<Landing_DecendTransition>
{
        Landing_DecendTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Landing (Decend)\n"; }
};

struct Landing_AbortTransition : sc::event<Landing_AbortTransition>
{
        Landing_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Landing (Abort)\n"; }
};

struct Landing_LandedTransition : sc::event<Landing_LandedTransition>
{
        Landing_LandedTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Landing (Landed)\n"; }
};
