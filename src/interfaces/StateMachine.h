/******************************************************************************
 * @brief
 *
 * @file StateMachine.h
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-0716
 *
 * @copyright Copyright MRDT 2023 - All Rights Reserved
 ******************************************************************************/

#include <iostream>

#include "../Autonomy_Globals.h"

#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/transition.hpp>

#include <boost/mpl/list.hpp>

namespace sc  = boost::statechart;
namespace mpl = boost::mpl;

#ifndef STATEMACHINE_H
#define STATEMACHINE_H

// Forward Declared Shared States
struct IdleState;
struct NavigationState;
struct SearchPatternState;
struct AvoidanceState;
struct ApproachingMarkerState;
struct ApproachingGateState;
struct AbortState;

// Forward Declared Rover States
struct ReverseState;
struct StuckState;

// Forward Declared Drone States
struct CrashedState;
struct DecendState;
struct LandingState;
struct ClimbState;
struct TakeoffState;

// Forward Declared State Machine
struct StateMachine : sc::state_machine<StateMachine, IdleState>
{
        StateMachine() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "Starting State Machine..."; }
};

// Idle Transitions
struct Idle_AbortTransition;
struct Idle_TakeoffTransition;
struct Idle_NavigatingTransition;
struct Idle_LandingTransition;
struct Idle_ReverseTransition;

// Navigation Transitions
struct Navigation_NewWaypointTransition;
struct Navigation_AbortTransition;
struct Navigation_StuckTransition;
struct Navigation_ReachedGPSTransition;
struct Navigation_SeenTagTransition;
struct Navigation_ObstacleAvoidanceTransition;

// Seach Pattern Transitions
struct SeachPattern_GateSeenTransition;
struct SeachPattern_ObstacleAvoidanceTransition;
struct SeachPattern_MarkerSeenTransition;
struct SeachPattern_AbortTransition;
struct SeachPattern_StuckTransition;
struct SeachPattern_AdjustAltitudeTransition;

// Avoidance Transitions
struct Avoidance_EndAvoidanceTransition;
struct Avoidance_AbortTransition;
struct Avoidance_StuckTransition;
struct Avoidance_AdjustAltitudeTransition;

// Approaching Marker Transitions
struct ApproachingMarker_MarkerLostTransition;
struct ApproachingMarker_AbortTransition;
struct ApproachingMarker_ReachedMarkerTransition;

// Approaching Gate Transitions
struct ApproachingGate_MarkerLostTransition;
struct ApproachingGate_AbortTransition;
struct ApproachingGate_ReachedMarkerTransition;

// Abort Transitions
struct Abort_RestartTransition;
struct Abort_ExitTransition;

// Reverse Transitions
struct Reverse_ContinueTransition;
struct Reverse_AbortTransition;
struct Reverse_StuckTransition;

// Stuck Transitions
struct Stuck_AbortTransition;
struct Stuck_ReverseTransition;

// Crashed Transitions
struct Crashed_ExitTransition;
struct Crashed_AbortTransition;

// Decend Transitions
struct Decend_NoResponseTransition;
struct Decend_AbortTransition;
struct Decend_ReachedAltitudeTransition;

// Landed Transitions
struct Landing_NoResponseTransition;
struct Landing_DecendTransition;
struct Landing_AbortTransition;
struct Landing_LandedTransition;

// Climb Transitions
struct Climb_NoResponseTransition;
struct Climb_AbortTransition;
struct Climb_ReachedAltitudeTransition;

// Takeoff Transitions
struct Takeoff_NoResponseTransition;
struct Takeoff_ClimbTransition;
struct Takeoff_AbortTransition;
struct Takeoff_ReachedAltitudeTransition;

struct AbortState : sc::simple_state<AbortState, StateMachine>
{
        AbortState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Abort"; }

        typedef mpl::list<sc::custom_reaction<Abort_RestartTransition>, sc::custom_reaction<Abort_ExitTransition>> reactions;

        sc::result react(const Abort_RestartTransition& event) { return transit<IdleState>(); }

        sc::result react(const Abort_ExitTransition& event) { return transit<IdleState>(); }
};

struct Abort_RestartTransition : sc::event<Abort_RestartTransition>
{
        Abort_RestartTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Abort (Restart)"; }
};

struct Abort_ExitTransition : sc::event<Abort_ExitTransition>
{
        Abort_ExitTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Abort (Exit)"; }
};

struct ApproachingGateState : sc::simple_state<ApproachingGateState, StateMachine>
{
        ApproachingGateState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Approaching Gate"; }

        typedef mpl::list<sc::custom_reaction<ApproachingGate_MarkerLostTransition>,
                          sc::custom_reaction<ApproachingGate_AbortTransition>,
                          sc::custom_reaction<ApproachingGate_ReachedMarkerTransition>>
            reactions;

        sc::result react(const ApproachingGate_MarkerLostTransition& event) { return transit<SearchPatternState>(); }

        sc::result react(const ApproachingGate_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const ApproachingGate_ReachedMarkerTransition& event) { return transit<IdleState>(); }
};

struct ApproachingGate_MarkerLostTransition : sc::event<ApproachingGate_MarkerLostTransition>
{
        ApproachingGate_MarkerLostTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Approaching Gate (Marker Lost)"; }
};

struct ApproachingGate_AbortTransition : sc::event<ApproachingGate_AbortTransition>
{
        ApproachingGate_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Approaching Gate (Abort)"; }
};

struct ApproachingGate_ReachedMarkerTransition : sc::event<ApproachingGate_ReachedMarkerTransition>
{
        ApproachingGate_ReachedMarkerTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Approaching Gate (Reached Marker)"; }
};

struct ApproachingMarkerState : sc::simple_state<ApproachingMarkerState, StateMachine>
{
        ApproachingMarkerState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Approaching Marker"; }

        typedef mpl::list<sc::custom_reaction<ApproachingMarker_MarkerLostTransition>,
                          sc::custom_reaction<ApproachingMarker_AbortTransition>,
                          sc::custom_reaction<ApproachingMarker_ReachedMarkerTransition>>
            reactions;

        sc::result react(const ApproachingMarker_MarkerLostTransition& event) { return transit<SearchPatternState>(); }

        sc::result react(const ApproachingMarker_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const ApproachingMarker_ReachedMarkerTransition& event) { return transit<IdleState>(); }
};

struct ApproachingMarker_MarkerLostTransition : sc::event<ApproachingMarker_MarkerLostTransition>
{
        ApproachingMarker_MarkerLostTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Approaching Marker (Marker Lost)"; }
};

struct ApproachingMarker_AbortTransition : sc::event<ApproachingMarker_AbortTransition>
{
        ApproachingMarker_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Approaching Marker (Abort)"; }
};

struct ApproachingMarker_ReachedMarkerTransition : sc::event<ApproachingMarker_ReachedMarkerTransition>
{
        ApproachingMarker_ReachedMarkerTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Approaching Marker (Reached Marker)"; }
};

struct AvoidanceState : sc::simple_state<AvoidanceState, StateMachine>
{
        AvoidanceState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Avoidance"; }

        typedef mpl::list<sc::custom_reaction<Avoidance_EndAvoidanceTransition>,
                          sc::custom_reaction<Avoidance_AbortTransition>,
                          sc::custom_reaction<Avoidance_StuckTransition>,
                          sc::custom_reaction<Avoidance_AdjustAltitudeTransition>>
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

        sc::result react(const Avoidance_AdjustAltitudeTransition& event)
        {
            // If - Climb
            //      return transit<ClimbState>();

            // If - Decend
            //      return transit<DecendState>();

            return transit<IdleState>();
        }
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

struct Avoidance_AdjustAltitudeTransition : sc::event<Avoidance_AdjustAltitudeTransition>
{
        Avoidance_AdjustAltitudeTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Avoidance (Adjust Altitude)"; }
};

struct ClimbState : sc::simple_state<ClimbState, StateMachine>
{
        ClimbState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Climb"; }

        typedef mpl::
            list<sc::custom_reaction<Climb_NoResponseTransition>, sc::custom_reaction<Climb_AbortTransition>, sc::custom_reaction<Climb_ReachedAltitudeTransition>>
                reactions;

        sc::result react(const Climb_NoResponseTransition& event) { return transit<CrashedState>(); }

        sc::result react(const Climb_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Climb_ReachedAltitudeTransition& event)
        {
            // If - Takeoff
            //      return transit<TakeoffState>();

            // If - Navigate
            //      return transit<NavigateState>();

            // If - Search Pattern
            //      return transit<SearchPatternState>();

            // If - Avoidance
            //      return transit<AvoidanceState>();

            return transit<IdleState>();
        }
};

struct Climb_NoResponseTransition : sc::event<Climb_NoResponseTransition>
{
        Climb_NoResponseTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Climb (No Response)"; }
};

struct Climb_AbortTransition : sc::event<Climb_AbortTransition>
{
        Climb_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Climb (Abort)"; }
};

struct Climb_ReachedAltitudeTransition : sc::event<Climb_ReachedAltitudeTransition>
{
        Climb_ReachedAltitudeTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Climb (Reached Altitude)"; }
};

struct CrashedState : sc::simple_state<CrashedState, StateMachine>
{
        CrashedState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Crashed"; }

        typedef mpl::list<sc::custom_reaction<Crashed_ExitTransition>, sc::custom_reaction<Crashed_AbortTransition>> reactions;

        sc::result react(const Crashed_ExitTransition& event) { return transit<IdleState>(); }

        sc::result react(const Crashed_AbortTransition& event) { return transit<AbortState>(); }
};

struct Crashed_ExitTransition : sc::event<Crashed_ExitTransition>
{
        Crashed_ExitTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Crashed (Exit)"; }
};

struct Crashed_AbortTransition : sc::event<Crashed_AbortTransition>
{
        Crashed_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Crashed (Abort)"; }
};

struct DecendState : sc::simple_state<DecendState, StateMachine>
{
        DecendState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Decend"; }

        typedef mpl::
            list<sc::custom_reaction<Decend_NoResponseTransition>, sc::custom_reaction<Decend_AbortTransition>, sc::custom_reaction<Decend_ReachedAltitudeTransition>>
                reactions;

        sc::result react(const Decend_NoResponseTransition& event) { return transit<CrashedState>(); }

        sc::result react(const Decend_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Decend_ReachedAltitudeTransition& event)
        {
            // If - Takeoff
            //      return transit<TakeoffState>();

            // If - Navigate
            //      return transit<NavigateState>();

            // If - Search Pattern
            //      return transit<SearchPatternState>();

            // If - Avoidance
            //      return transit<AvoidanceState>();

            return transit<IdleState>();
        }
};

struct Decend_NoResponseTransition : sc::event<Decend_NoResponseTransition>
{
        Decend_NoResponseTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Decend (No Response)"; }
};

struct Decend_AbortTransition : sc::event<Decend_AbortTransition>
{
        Decend_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Decend (Abort)"; }
};

struct Decend_ReachedAltitudeTransition : sc::event<Decend_ReachedAltitudeTransition>
{
        Decend_ReachedAltitudeTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Decend (Reached Altitude)"; }
};

struct IdleState : sc::simple_state<IdleState, StateMachine>
{
        IdleState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Idle"; }

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
        Idle_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Idle (Abort)"; }
};

struct Idle_TakeoffTransition : sc::event<Idle_TakeoffTransition>
{
        Idle_TakeoffTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Idle (Takeoff)"; }
};

struct Idle_NavigatingTransition : sc::event<Idle_NavigatingTransition>
{
        Idle_NavigatingTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Idle (Navigating)"; }
};

struct Idle_LandingTransition : sc::event<Idle_LandingTransition>
{
        Idle_LandingTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Idle (Landing)"; }
};

struct Idle_ReverseTransition : sc::event<Idle_NavigatingTransition>
{
        Idle_ReverseTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Idle (Reverse)"; }
};

struct LandingState : sc::simple_state<LandingState, StateMachine>
{
        LandingState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Landing"; }

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
        Landing_NoResponseTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Landing (No Response))"; }
};

struct Landing_DecendTransition : sc::event<Landing_DecendTransition>
{
        Landing_DecendTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Landing (Decend)"; }
};

struct Landing_AbortTransition : sc::event<Landing_AbortTransition>
{
        Landing_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Landing (Abort)"; }
};

struct Landing_LandedTransition : sc::event<Landing_LandedTransition>
{
        Landing_LandedTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Landing (Landed)"; }
};

struct NavigationState : sc::simple_state<NavigationState, StateMachine>
{
        NavigationState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Navigation"; }

        typedef mpl::list<sc::custom_reaction<Navigation_NewWaypointTransition>,
                          sc::custom_reaction<Navigation_AbortTransition>,
                          sc::custom_reaction<Navigation_StuckTransition>,
                          sc::custom_reaction<Navigation_ReachedGPSTransition>,
                          sc::custom_reaction<Navigation_SeenTagTransition>,
                          sc::custom_reaction<Navigation_ObstacleAvoidanceTransition>>
            reactions;

        sc::result react(const Navigation_NewWaypointTransition& event) { return transit<NavigationState>(); }

        sc::result react(const Navigation_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Navigation_StuckTransition& event) { return transit<StuckState>(); }

        sc::result react(const Navigation_ReachedGPSTransition& event)
        {
            // If - GPS Only
            //      return transit<IdleState>();

            // If - ArUco Search
            //      return transit<SearchPatternState>();

            return transit<IdleState>();
        }

        sc::result react(const Navigation_SeenTagTransition& event)
        {
            // If - Marker
            //      return transit<ApproachingMarkerState>();

            // If - Gate
            //      return transit<ApproachingGateState>();

            return transit<IdleState>();
        }

        sc::result react(const Navigation_ObstacleAvoidanceTransition& event) { return transit<AvoidanceState>(); }
};

struct Navigation_NewWaypointTransition : sc::event<Navigation_NewWaypointTransition>
{
        Navigation_NewWaypointTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Navigation (New Waypoint)"; }
};

struct Navigation_AbortTransition : sc::event<Navigation_AbortTransition>
{
        Navigation_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Navigation (Abort)"; }
};

struct Navigation_StuckTransition : sc::event<Navigation_StuckTransition>
{
        Navigation_StuckTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Navigation (Stuck)"; }
};

struct Navigation_ReachedGPSTransition : sc::event<Navigation_ReachedGPSTransition>
{
        Navigation_ReachedGPSTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Navigation (Reached GPS)"; }
};

struct Navigation_SeenTagTransition : sc::event<Navigation_SeenTagTransition>
{
        Navigation_SeenTagTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Navigation (Seen Tag)"; }
};

struct Navigation_ObstacleAvoidanceTransition : sc::event<Navigation_ObstacleAvoidanceTransition>
{
        Navigation_ObstacleAvoidanceTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Navigation (Obstacle Avoidance)"; }
};

struct ReverseState : sc::simple_state<ReverseState, StateMachine>
{
        ReverseState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Reverse"; }

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

            return transit<IdleState>();
        }

        sc::result react(const Reverse_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Reverse_StuckTransition& event) { return transit<StuckState>(); }
};

struct Reverse_ContinueTransition : sc::event<Reverse_ContinueTransition>
{
        Reverse_ContinueTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Reverse (Continue)"; }
};

struct Reverse_AbortTransition : sc::event<Reverse_AbortTransition>
{
        Reverse_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Reverse (Abort)"; }
};

struct Reverse_StuckTransition : sc::event<Reverse_StuckTransition>
{
        Reverse_StuckTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Reverse (Stuck)"; }
};

struct SearchPatternState : sc::simple_state<SearchPatternState, StateMachine>
{
        SearchPatternState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Search Pattern"; }

        typedef mpl::list<sc::custom_reaction<SeachPattern_GateSeenTransition>,
                          sc::custom_reaction<SeachPattern_ObstacleAvoidanceTransition>,
                          sc::custom_reaction<SeachPattern_MarkerSeenTransition>,
                          sc::custom_reaction<SeachPattern_AbortTransition>,
                          sc::custom_reaction<SeachPattern_StuckTransition>>
            reactions;

        sc::result react(const SeachPattern_GateSeenTransition& event) { return transit<ApproachingGateState>(); }

        sc::result react(const SeachPattern_ObstacleAvoidanceTransition& event) { return transit<AvoidanceState>(); }

        sc::result react(const SeachPattern_MarkerSeenTransition& event) { return transit<ApproachingMarkerState>(); }

        sc::result react(const SeachPattern_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const SeachPattern_StuckTransition& event) { return transit<StuckState>(); }

        sc::result react(const SeachPattern_AdjustAltitudeTransition& event)
        {
            // If - Climb
            //      return transit<ClimbState>();

            // If - Decend
            //      return transit<DecendState>();

            return transit<IdleState>();
        }
};

struct SeachPattern_GateSeenTransition : sc::event<SeachPattern_GateSeenTransition>
{
        SeachPattern_GateSeenTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Search Pattern (Gate Seen)"; }
};

struct SeachPattern_ObstacleAvoidanceTransition : sc::event<SeachPattern_ObstacleAvoidanceTransition>
{
        SeachPattern_ObstacleAvoidanceTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Search Pattern (Obstacle Avoidance)"; }
};

struct SeachPattern_MarkerSeenTransition : sc::event<SeachPattern_MarkerSeenTransition>
{
        SeachPattern_MarkerSeenTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Search Pattern (Marker Seen)"; }
};

struct SeachPattern_AbortTransition : sc::event<SeachPattern_AbortTransition>
{
        SeachPattern_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Search Pattern (Abort)"; }
};

struct SeachPattern_StuckTransition : sc::event<SeachPattern_StuckTransition>
{
        SeachPattern_StuckTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Search Pattern (Stuck)"; }
};

struct SeachPattern_AdjustAltitudeTransition : sc::event<SeachPattern_AdjustAltitudeTransition>
{
        SeachPattern_AdjustAltitudeTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Search Pattern (Adjust Altitude)"; }
};

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

struct TakeoffState : sc::simple_state<TakeoffState, StateMachine>
{
        TakeoffState() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In State: Takeoff"; }

        typedef mpl::list<sc::custom_reaction<Takeoff_NoResponseTransition>,
                          sc::custom_reaction<Takeoff_ClimbTransition>,
                          sc::custom_reaction<Takeoff_AbortTransition>,
                          sc::custom_reaction<Takeoff_ReachedAltitudeTransition>>
            reactions;

        sc::result react(const Takeoff_NoResponseTransition& event) { return transit<CrashedState>(); }

        sc::result react(const Takeoff_ClimbTransition& event) { return transit<DecendState>(); }

        sc::result react(const Takeoff_AbortTransition& event) { return transit<AbortState>(); }

        sc::result react(const Takeoff_ReachedAltitudeTransition& event) { return transit<NavigationState>(); }
};

struct Takeoff_NoResponseTransition : sc::event<Takeoff_NoResponseTransition>
{
        Takeoff_NoResponseTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Takeoff (No Response)"; }
};

struct Takeoff_ClimbTransition : sc::event<Takeoff_ClimbTransition>
{
        Takeoff_ClimbTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Takeoff (Climb)"; }
};

struct Takeoff_AbortTransition : sc::event<Takeoff_AbortTransition>
{
        Takeoff_AbortTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Takeoff (Abort)"; }
};

struct Takeoff_ReachedAltitudeTransition : sc::event<Takeoff_ReachedAltitudeTransition>
{
        Takeoff_ReachedAltitudeTransition() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "In Transition: Takeoff (Reached Altitude)"; }
};

#endif
