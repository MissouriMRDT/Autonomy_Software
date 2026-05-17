# Drive Board Driver

The `DriveBoard` class acts as the bridge between the autonomy software's kinematics calculations and the physical motor controllers on the rover.

## Primary Responsibilities
1. **Kinematics Calculation**: It takes in high-level commands (e.g., a target speed and a target heading) and translates them into left and right track power percentages using Differential Drive inverse kinematics.
2. **Network Transmission**: It formats these left and right track powers into a UDP RoveComm packet (`DRIVELEFTRIGHT`) and sends it over the ethernet network to the physical Core/Drive board microcontroller.
3. **Safety and Multipliers**: It handles hardware-level constraints, such as limiting the maximum drive effort on steep slopes based on inclinometer data.

## Algorithm Explanation

### 1. Kinematics (`CalculateMove`)
The State Machine or Path Planner typically provides a `GoalSpeed` and a `GoalHeading`. The `DriveBoard` calculates the necessary wheel speeds using the `DifferentialDrive` namespace.

- A PID controller first calculates a turn/rotation effort based on the error between the `GoalHeading` and the `ActualHeading`.
- This rotation effort is passed into either an **Arcade Drive** or **Curvature Drive** inverse kinematics model.
- *Arcade Drive* simply adds the turn effort to one side and subtracts it from the other.
- *Curvature Drive* uses the turn effort to dictate the curvature of the robot's arc, which makes driving at high speeds significantly more stable.

### 2. Variable Drive Effort (`VariableDriveEffort`)
The rover uses an inclinometer to determine its current pitch and roll on the terrain. The `DriveBoard` actively monitors these values (received via RoveComm from the Core board) to calculate a damping multiplier.
- If the rover is on a slope steeper than `DRIVE_BOARD_MAX_SLOPE`, the multiplier drops to `DRIVE_BOARD_MIN_DAMP` (e.g., 50%).
- This safety mechanism forces the rover to drive slower on treacherous hills, preventing it from flipping over backward or barrel-rolling.

### 3. RoveComm Callbacks (`SetMaxSpeedCallback`)
The driver registers a callback on the UDP RoveComm node to listen for a `SETMAXSPEED` packet. This allows an external operator (using a basestation GUI) to dynamically adjust the global `DriveEffortMultiplier` from 0.0 to 1.0, effectively acting as a master throttle for the entire autonomy system.

## Inputs and Outputs
- **Inputs**:
  - `dGoalSpeed`: Requested forward/backward speed (-1.0 to 1.0).
  - `dGoalHeading`: The compass angle the rover should point towards.
  - `dActualHeading`: The current compass angle of the rover.
  - `RoveComm`: Inclinometer pitch/roll packets.
- **Outputs**:
  - `DRIVELEFTRIGHT`: A RoveComm UDP packet sent to the `Core` board containing two floats representing the left and right motor powers.

## Usage in State Machine
Virtually all moving states (e.g., `NavigatingState`, `ApproachingMarkerState`, `SearchPatternState`) will call `globals::g_pDriveBoard->CalculateMove(...)` to generate the correct kinematics, and then immediately call `globals::g_pDriveBoard->SendDrive(...)` to dispatch the command to the physical motors.
