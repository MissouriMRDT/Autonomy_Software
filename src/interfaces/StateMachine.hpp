/******************************************************************************
 * @brief State Machine Implementation for Autonomy Software
 *
 * @file StateMachine.hpp
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 *
 * @copyright Copyright Mars Rover Design Team 2023 - All Rights Reserved
 ******************************************************************************/

#include "../AutonomyGlobals.h"
#include "../AutonomyLogging.h"

// Forward References
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
 * @brief State Machine Object
 *
 *        The State Machine is what controls the operation of the our Rover
 *        Autonomously. Though the state handlers below and attached
 *        transitions the Rover navigates though many algorithms to
 *        complete the Autonomous task.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct StateMachine : sc::state_machine<StateMachine, IdleState>
{
        StateMachine() { LOG_INFO(g_qSharedLogger, "Starting State Machine..."); }
};

/******************************************************************************
 * @brief Idle State Handler and Transition Forward Declarations
 *
 *        Adds the state handler and transitions for the Idle State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Idle_AbortTransition;
struct Idle_NavigatingTransition;
struct Idle_ReverseTransition;

#include "../states/IdleState.hpp"

/******************************************************************************
 * @brief Navigation State Handler and Transition Forward Declarations
 *
 *        Adds the state handler and transitions for the Navigation State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Navigation_NewWaypointTransition;
struct Navigation_AbortTransition;
struct Navigation_StuckTransition;
struct Navigation_ReachedGPSTransition;
struct Navigation_SeenTagTransition;
struct Navigation_ObstacleAvoidanceTransition;

#include "../states/NavigationState.hpp"

/******************************************************************************
 * @brief Search Pattern State Handler and Transition Forward Declarations
 *
 *        Adds the state handler and transitions for the Search Pattern State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct SeachPattern_GateSeenTransition;
struct SeachPattern_ObstacleAvoidanceTransition;
struct SeachPattern_MarkerSeenTransition;
struct SeachPattern_AbortTransition;
struct SeachPattern_StuckTransition;

#include "../states/SearchPatternState.hpp"

/******************************************************************************
 * @brief Obstacle Avoidance State Handler and Transition Forward Declarations
 *
 *        Adds the state handler and transitions for the Obstacle
 *        Avoidance State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Avoidance_EndAvoidanceTransition;
struct Avoidance_AbortTransition;
struct Avoidance_StuckTransition;

#include "../states/AvoidanceState.hpp"

/******************************************************************************
 * @brief Approaching Marker State Handler and Transition Forward Declarations
 *
 *        Adds the state handler and transitions for the Approaching
 *        Marker State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct ApproachingMarker_MarkerLostTransition;
struct ApproachingMarker_AbortTransition;
struct ApproachingMarker_ReachedMarkerTransition;

#include "../states/ApproachingMarkerState.hpp"

/******************************************************************************
 * @brief Approaching Gate State Handler and Transition Forward Declarations
 *
 *        Adds the state handler and transitions for the Approaching
 *        Gate State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct ApproachingGate_MarkerLostTransition;
struct ApproachingGate_AbortTransition;
struct ApproachingGate_ReachedMarkerTransition;

#include "../states/ApproachingGateState.hpp"

/******************************************************************************
 * @brief Abort State Handler and Transition Forward Declarations
 *
 *        Adds the state handler and transitions for the Abort State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Abort_RestartTransition;
struct Abort_ExitTransition;

#include "../states/AbortState.hpp"

/******************************************************************************
 * @brief Reverse State Handler and Transition Forward Declarations
 *
 *        Adds the state handler and transitions for the Reverse State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Reverse_ContinueTransition;
struct Reverse_AbortTransition;
struct Reverse_StuckTransition;

#include "../states/ReverseState.hpp"

/******************************************************************************
 * @brief Stuck State Handler and Transition Forward Declarations
 *
 *        Adds the state handler and transitions for the Stuck State.
 *
 *
 * @author Eli Byrd (edbgkk@mst.edu)
 * @date 2023-07-31
 ******************************************************************************/
struct Stuck_AbortTransition;
struct Stuck_ReverseTransition;

#include "../states/StuckState.hpp"
