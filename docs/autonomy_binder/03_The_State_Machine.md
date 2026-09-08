# The State Machine

The State Machine forms the decision-making core of the Autonomy Software. It dictates rover behavior by evaluating sensory perception, navigation waypoints, failsafes, and mission timers in a continuous loop.

---

## 1. Architectural Philosophy

The software employs a state-based architecture where navigation tasks are decoupled into explicit, self-contained states derived from `statemachine::State` (`src/interfaces/State.hpp`).

### Design Principles
- **Strict Behavioral Isolation**: Each state executes only the logic relevant to its immediate objective. For example, `eApproachingMarker` focuses strictly on visual tracking and approach heuristics without running the global A* path planner.
- **Event-Driven Transitions**: State changes occur when events are dispatched via `globals::g_pStateMachineHandler->HandleEvent(eEvent, bSaveCurrentState)`.
- **State Preservation and Recall**: When a temporary disruption occurs (such as getting stuck or needing a reversing maneuver), the calling state can be preserved in `m_umSavedStates` via `bSaveCurrentState = true`. Once recovery finishes, the previous state is recalled without losing internal progress (such as an in-progress search spiral).
- **Dedicated Execution Thread**: The state machine runs on an `AutonomyThread` in `StateMachineHandler.cpp`, decoupled from camera capture and network I/O. Its tick rate is throttled by `constants::STATEMACHINE_MAX_IPS` (typically 60 Hz).

---

## 2. Enumerated State Definitions

The autonomy system defines 10 discrete states (`statemachine::States`):

| State Enum | Name | Primary Objective and Behavior |
| :--- | :--- | :--- |
| `eIdle` | **Idle** | Default standby state. Drives are halted (`SendStop()`), and lighting is set to off or teleop. The system listens for RoveComm start commands (`eStart`) or waypoint assignments. |
| `eNavigating` | **Navigating** | Global transit state. The rover queries `GeoPlanner` / A* to navigate through waypoints toward the target coordinate while steering with predictive or PID controllers. |
| `eSearchPattern` | **Search Pattern** | Executed when the rover arrives at the vicinity of a marker or object but cannot visually identify it. Drives systematic geometric patterns (Spiral, ZigZag, Snake) around the waypoint coordinate. |
| `eApproachingMarker` | **Approaching Marker** | Visual servoing state. Activates once an ArUco tag is detected. Uses trigonometric pose estimation from `TagDetectionUtilty.hpp` to drive directly toward the tag face. |
| `eApproachingObject` | **Approaching Object** | Object-tracking servoing state. Activates once a target prop (mallet, bottle, rock pick) is detected by YOLO. Tracks the bounding box and approaches the geolocated coordinate. |
| `eVerifyingPosition` | **Verifying Position** | Stop-and-sample state for GNSS-only waypoints. The rover remains stationary for `NAVIGATING_VERIFY_SAMPLE_TIME` seconds to average GPS coordinates and confirm arrival within tolerance. |
| `eVerifyingMarker` | **Verifying Marker** | Stationary confirmation state for AR tags. The rover stops in front of the tag for `APPROACH_MARKER_VERIFY_TIME` seconds, confirming tag visibility before declaring completion. |
| `eVerifyingObject` | **Verifying Object** | Stationary confirmation state for ground objects. The rover halts and samples the YOLO detector across `APPROACH_OBJECT_VERIFY_TIME` seconds, verifying a minimum detection hit-rate. |
| `eReversing` | **Reversing** | Fallback driving state. Drives backward for `constants::REVERSE_DISTANCE` at `constants::REVERSE_MOTOR_POWER` to back away from an obstruction or overshoot. |
| `eStuck` | **Stuck** | Multi-phase recovery state. Triggered when motors are commanded but position/heading do not change. Executes sequential reversing and realignment attempts. |

---

## 3. Enumerated Events

State transitions are driven by discrete triggers defined in `statemachine::Event`:

```cpp
enum class Event
{
    eStart,                 // Operator commanded autonomy to begin
    eReachedGpsCoordinate,  // Rover entered goal radius of target coordinate
    eReachedMarker,         // Rover closed distance to ArUco marker within proximity threshold
    eReachedObject,         // Rover closed distance to target object within proximity threshold
    eMarkerSeen,            // Target ArUco tag detected with sufficient confidence/age
    eObjectSeen,            // Target ground object detected by YOLO model
    eMarkerUnseen,          // Visual lock on ArUco tag lost beyond timeout buffer
    eObjectUnseen,          // Visual lock on object lost beyond timeout buffer
    eVerifyingComplete,     // Verification window succeeded (confirmed target)
    eVerifyingFailed,       // Verification window failed (false positive or lost sight)
    eAbort,                 // Operator commanded immediate emergency stop
    eRestart,               // Command to reset and restart mission
    eNoWaypoint,            // Waypoint queue is empty
    eNewWaypoint,           // New waypoint added to queue
    eReverse,               // Triggered to back out of a deadlock
    eReverseComplete,       // Reversing distance has been achieved
    eSearchFailed,          // Search pattern exhausted without detecting target
    eStuck,                 // Motion sensors confirm drive stall
    eUnstuck                // Motion confirmed; rover free to resume prior state
};
```

---

## 4. State Transition Matrix

The table below details typical state transitions, their triggering events, and the resulting target state:

| Current State | Event Trigger | Next State | Context / Action |
| :--- | :--- | :--- | :--- |
| `eIdle` | `eStart` | `eNavigating` | Operator begins mission; first waypoint popped from queue. |
| `eNavigating` | `eReachedGpsCoordinate` (Nav Leg) | `eVerifyingPosition` | Reached GNSS waypoint within `NAVIGATING_REACHED_GOAL_RADIUS`. |
| `eNavigating` | `eReachedGpsCoordinate` (Marker/Obj Leg) | `eSearchPattern` | Arrived at vicinity coordinates without visual detection. |
| `eNavigating` | `eMarkerSeen` | `eApproachingMarker` | Tag detected en route; global path planning aborted for visual approach. |
| `eNavigating` | `eObjectSeen` | `eApproachingObject` | Object detected en route; visual approach begins. |
| `eNavigating` | `eNoWaypoint` | `eIdle` | Mission queue completed. |
| `eSearchPattern` | `eMarkerSeen` | `eApproachingMarker` | Tag spotted during search spiral; switches to visual servoing. |
| `eSearchPattern` | `eObjectSeen` | `eApproachingObject` | Object spotted during search spiral; switches to visual approach. |
| `eSearchPattern` | `eSearchFailed` | `eIdle` / Next Leg | Search area exhausted without detection; logs warning and proceeds. |
| `eApproachingMarker` | `eReachedMarker` | `eVerifyingMarker` | Rover within `APPROACH_MARKER_PROXIMITY_THRESHOLD` (e.g., 2.0 m). |
| `eApproachingMarker` | `eMarkerUnseen` | `eSearchPattern` | Tag tracking lost for longer than buffer time; resumes search pattern. |
| `eApproachingObject` | `eReachedObject` | `eVerifyingObject` | Rover within `APPROACH_OBJECT_PROXIMITY_THRESHOLD`. |
| `eApproachingObject` | `eObjectUnseen` | `eSearchPattern` | Object lost from view; returns to localized search. |
| `eVerifyingMarker` | `eVerifyingComplete` | `eNavigating` / `eIdle` | Goal confirmed; flashes green LED, signals basestation, loads next leg. |
| `eVerifyingMarker` | `eVerifyingFailed` | `eSearchPattern` | Tag could not be re-verified; falls back to search pattern. |
| `eVerifyingObject` | `eVerifyingComplete` | `eNavigating` / `eIdle` | Object confirmed; flashes green LED, signals basestation, loads next leg. |
| `eVerifyingPosition` | `eVerifyingComplete` | `eNavigating` / `eIdle` | Position confirmed within GPS error radius; loads next leg. |
| *Any Moving State* | `eStuck` | `eStuck` | Motion detector confirmed motor stall; state preserved for recovery. |
| `eStuck` | `eUnstuck` | *Previous State* | Rover escaped stall; recalled state resumes previous task. |
| *Any State* | `eAbort` | `eIdle` | Emergency abort commanded; motors stopped immediately. |

---

## 5. Recovery and Failsafe Subsystems

### Stuck State Recovery Machine (`StuckState.cpp`)
Stuck detection is handled by `TimeIntervalBasedStuckDetector` (`src/util/states/StuckDetection.hpp`). If linear velocity is below `constants::STUCK_CHECK_VEL_THRESH` and angular velocity is below `constants::STUCK_CHECK_ROT_THRESH` while motors are commanding power for multiple consecutive intervals, `Event::eStuck` is dispatched.

`StuckState` executes a four-tier recovery routine:
1. **`AttemptType::eReverseCurrentHeading`**: The rover maintains its current heading, preserves its current state in `m_umSavedStates`, and dispatches `Event::eReverse`. The rover backs up by `REVERSE_DISTANCE`.
2. **`AttemptType::eReverseLeft`**: If still stationary after reversing, the rover point-turns to `m_dOriginalHeading + constants::STUCK_ALIGN_DEGREES` and dispatches a second reversing attempt.
3. **`AttemptType::eReverseRight`**: If still stuck, the rover point-turns to `m_dOriginalHeading - constants::STUCK_ALIGN_DEGREES` and reverses a third time.
4. **`AttemptType::eGiveUp`**: If all three directional reversals fail to move the rover beyond `constants::STUCK_SAME_POINT_PROXIMITY`:
   - It calls `DeclareObstacle()` to inject a virtual obstacle directly in front of the rover.
   - It executes `SplicePath()` to excise the blocked region from the active waypoint route.
   - It logs a critical failure and returns to `eIdle` or aborts the current leg.

### Battery Protection Failsafe
The state machine monitors battery metrics via RoveComm PMS telemetry. If `BATTERY_CHECKS_ENABLED` is true and any cell drops below `constants::BATTERY_MINIMUM_CELL_VOLTAGE` (default 3.2V), the state machine forcefully dispatches `Event::eAbort` to transition to `eIdle` and halt motor output, preventing battery degradation.

### Heading and Odometry Dynamic Realignment
In `StateMachineHandler::SmartRetrieveRoverPose()`, the system monitors the drift between the ZED visual-inertial odometry and absolute GPS/magnetometer heading.
- When the rover drives forward at speeds exceeding `constants::ZED_REALIGN_VEL_THRESH` with angular rates below `constants::ZED_REALIGN_ROT_THRESH`, or while resting in `eIdle`, `RealignZEDHeading()` computes the offset:
  $$\text{Offset} = \text{Heading}_{\text{Actual}} - \text{Heading}_{\text{Raw ZED}}$$
- During high-rate point-turns or evasive maneuvers where magnetic interference spikes, the system uses the high-frequency ZED IMU fused with this calibrated offset, avoiding erratic steering from compass distortion.
