/******************************************************************************
 * @brief
 *
 * @file AvoidanceState.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include "../Autonomy_Globals.h"

/******************************************************************************
 * @brief Obstacle Avoidance State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct AvoidanceState : sc::simple_state<AvoidanceState, StateMachine>
{
        AvoidanceState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Avoidance"; }

        typedef mpl::
            list<sc::custom_reaction<Avoidance_EndAvoidanceTransition>, sc::custom_reaction<Avoidance_AbortTransition>, sc::custom_reaction<Avoidance_StuckTransition>>
                reactions;

        sc::result react(const Avoidance_EndAvoidanceTransition& event)
        {
            // If - Search Pattern
            //      return transit<SearchPatternState>();

            // If - Navigation
            //      return transit<NavigationState>();

            return transit<IdleState>();
        }

        sc::result react(const Avoidance_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Avoidance_StuckTransition& event) { return transit<StuckState>(); }
};

struct Avoidance_EndAvoidanceTransition : sc::event<Avoidance_EndAvoidanceTransition>
{
        Avoidance_EndAvoidanceTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Avoidance (End Avoidance)"; }
};

struct Avoidance_AbortTransition : sc::event<Avoidance_AbortTransition>
{
        Avoidance_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Avoidance (Abort)"; }
};

struct Avoidance_StuckTransition : sc::event<Avoidance_StuckTransition>
{
        Avoidance_StuckTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Avoidance (Stuck)"; }
};
