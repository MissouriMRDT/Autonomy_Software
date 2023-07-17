/******************************************************************************
 * @brief
 *
 * @file StateMachine.hpp
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
        StateMachine() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "Starting State Machine...\n"; }
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

#endif
