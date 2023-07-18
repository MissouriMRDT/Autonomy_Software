/******************************************************************************
 * @brief
 *
 * @file StateMachine.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
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

/******************************************************************************
 * @brief Forward Declared References to States
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct IdleState;
struct NavigationState;
struct SearchPatternState;
struct AvoidanceState;
struct ApproachingMarkerState;
struct ApproachingGateState;
struct AbortState;
struct ReverseState;
struct StuckState;

/******************************************************************************
 * @brief State Machine
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct StateMachine : sc::state_machine<StateMachine, IdleState>
{
        StateMachine() { PLOGI_(AutonomyLogger::AL_ConsoleLogger) << "Starting State Machine..."; }
};

/******************************************************************************
 * @brief Idle State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct Idle_AbortTransition;
struct Idle_NavigatingTransition;
struct Idle_ReverseTransition;

#include "../states/IdleState.hpp"

/******************************************************************************
 * @brief Navigation State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct Navigation_NewWaypointTransition;
struct Navigation_AbortTransition;
struct Navigation_StuckTransition;
struct Navigation_ReachedGPSTransition;
struct Navigation_SeenTagTransition;
struct Navigation_ObstacleAvoidanceTransition;

#include "../states/NavigationState.hpp"

/******************************************************************************
 * @brief Search Pattern State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct SeachPattern_GateSeenTransition;
struct SeachPattern_ObstacleAvoidanceTransition;
struct SeachPattern_MarkerSeenTransition;
struct SeachPattern_AbortTransition;
struct SeachPattern_StuckTransition;

#include "../states/SearchPatternState.hpp"

/******************************************************************************
 * @brief Obstacle Avoidance State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct Avoidance_EndAvoidanceTransition;
struct Avoidance_AbortTransition;
struct Avoidance_StuckTransition;

#include "../states/AvoidanceState.hpp"

/******************************************************************************
 * @brief Approaching Marker State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct ApproachingMarker_MarkerLostTransition;
struct ApproachingMarker_AbortTransition;
struct ApproachingMarker_ReachedMarkerTransition;

#include "../states/ApproachingMarkerState.hpp"

/******************************************************************************
 * @brief Approaching Gate State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct ApproachingGate_MarkerLostTransition;
struct ApproachingGate_AbortTransition;
struct ApproachingGate_ReachedMarkerTransition;

#include "../states/ApproachingGateState.hpp"

/******************************************************************************
 * @brief Abort State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct Abort_RestartTransition;
struct Abort_ExitTransition;

#include "../states/AbortState.hpp"

/******************************************************************************
 * @brief Reverse State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct Reverse_ContinueTransition;
struct Reverse_AbortTransition;
struct Reverse_StuckTransition;

#include "../states/ReverseState.hpp"

/******************************************************************************
 * @brief Stuck State
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-17
 ******************************************************************************/
struct Stuck_AbortTransition;
struct Stuck_ReverseTransition;

#include "../states/StuckState.hpp"
