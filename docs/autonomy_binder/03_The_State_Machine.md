# The State Machine

The State Machine is the "Brain" of the autonomy software. It operates on a continuous loop, determining what the rover should be doing at any given moment based on sensor input, detected objects, and the current mission objectives.

## Philosophy
The autonomy system uses a **Hierarchical State Machine** approach. The design philosophy centers on breaking complex navigation tasks into smaller, manageable, and highly specific states.
- By strictly limiting the robot's actions within a specific state, we prevent the software from getting stuck in confusing loops (e.g., trying to avoid an obstacle while simultaneously trying to align to an AR tag).
- It runs on its own dedicated thread within `StateMachineHandler`, capping its update rate (configurable via `STATEMACHINE_MAX_IPS`, usually 60 iterations per second) to preserve CPU cycles for perception algorithms.

## State Definitions
The autonomy software defines the following discrete states (found in `src/interfaces/State.hpp`):

- **`eIdle`**: The default standby state. The rover does nothing but process incoming sensor data. It enters this state on startup, when the mission finishes, or if critical errors occur (like low battery voltage).
- **`eNavigating`**: The primary driving state. The rover calculates a global path to its next target UTM waypoint and follows it while actively avoiding obstacles.
- **`eSearchPattern`**: Triggered when the rover reaches a target location but does not immediately see the expected objective (like an AR tag). The rover drives in predefined patterns (Spiral, ZigZag, or Snake) until the object is detected.
- **`eApproachingMarker`**: Triggered when an AR tag is detected. The rover abandons global navigation and uses visual servoing/tracking to drive directly toward the detected tag.
- **`eApproachingObject`**: Triggered when a target object (like a bottle or mallet) is detected. Similar to `eApproachingMarker`, but tailored for Neural Network object detections.
- **`eVerifyingPosition`**: A temporary stopping state where the rover sits still for a configured time (`NAVIGATING_VERIFY_SAMPLE_TIME`) to collect and average GPS points to ensure it actually reached its destination.
- **`eVerifyingMarker` / `eVerifyingObject`**: The rover stops in front of the detected marker/object to ensure a stable, high-confidence detection before officially declaring the objective complete.
- **`eReversing`**: A fallback state used when the rover detects it has overshot a target, gotten too close to an obstacle, or as a component of the Stuck state recovery.
- **`eStuck`**: A critical failsafe state triggered when the rover determines it is providing motor power but not making any geographical progress.

## Transitions
Transitions between states are handled by discrete `Event` triggers (e.g., `eReachedGpsCoordinate`, `eMarkerSeen`, `eObjectUnseen`, `eStuckDetected`).

Examples of how the state machine transitions:
- `eNavigating` $\rightarrow$ (Event: `eMarkerSeen`) $\rightarrow$ `eApproachingMarker`
- `eApproachingMarker` $\rightarrow$ (Event: `eMarkerUnseen` - lost tracking) $\rightarrow$ `eSearchPattern`
- `eNavigating` $\rightarrow$ (Event: `eReachedGpsCoordinate`) $\rightarrow$ `eVerifyingPosition`
- *Any Moving State* $\rightarrow$ (Event: `eStuckDetected`) $\rightarrow$ `eStuck`

The math behind these transitions is often tied to constants defined in `AutonomyConstants.cpp`. For example, `eNavigating` triggers `eReachedGpsCoordinate` when the distance to the goal is less than `NAVIGATING_REACHED_GOAL_RADIUS`.

## Failsafes & Fallbacks
- **Low Battery Shutdown**: The state machine continually monitors battery levels. If the cell voltage drops below `BATTERY_MINIMUM_CELL_VOLTAGE` (e.g., 3.2V), it forcefully transitions to `eIdle` to protect the hardware.
- **Stuck Recovery (`eStuck`)**: If the rover detects it's stuck, it enters a recovery routine defined by `AttemptType`. This typically involves reversing, turning slightly, and attempting to drive forward again.
- **Vision Loss**: If tracking of a tag or object is lost while approaching, the state machine will fall back to `eSearchPattern` centered around the last known position rather than blindly driving forward.
- **ZED / GPS Desync**: If the error between the ZED Camera's visual odometry and the absolute GPS position exceeds `STATEMACHINE_ZED_REALIGN_THRESHOLD` (e.g., 0.5 meters), the system will force a realignment to prevent the robot from driving into imaginary obstacles or away from the goal.
