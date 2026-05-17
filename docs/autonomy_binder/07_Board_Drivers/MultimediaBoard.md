# Multimedia Board Driver

The `MultimediaBoard` acts as the primary communication link from the autonomy software to the humans observing the robot. It is responsible for controlling LED light strips, screens, or other visual indicators on the rover.

## Primary Responsibilities
1. **Visual Status Indications**: It translates the internal status of the autonomy software into distinct LED colors or lighting states so that judges, operators, or bystanders can instantly understand what the robot is doing.
2. **Custom RGB Commands**: It provides an interface for sending explicit Hex or integer RGB values directly to the LED controllers.

## Algorithm Explanation

### 1. State-Based Lighting (`SendLightingState`)
The core functionality of this driver revolves around the `MultimediaBoardLightingState` enum. Rather than manually typing RGB codes throughout the state machine, developers pass this enum to `SendLightingState()`, which internally maps the state to specific colors and creates the RoveComm packets.

- `eOff`: Sends `[0, 0, 0]` to turn the LED panels off.
- `eTeleOp`: Sends the standard blue color `[0, 0, 255]` to indicate the rover is being manually driven.
- `eAutonomy`: Sends the standard red color `[255, 0, 0]` to clearly indicate that the rover is operating autonomously (a critical safety indicator during competitions).
- `eReachedGoal`: Sends a flashing green command `[0, 255, 0]` to signal that the rover has successfully completed a navigation waypoint or found an objective.

### 2. Network Transmission
The class constructs a `LEDRGB` RoveComm packet with an array of uint8 values (Red, Green, Blue). Because lighting commands aren't as high-priority or high-frequency as motor commands, these packets are sent over UDP and only when the state actively changes to avoid flooding the network.

## Inputs and Outputs
- **Inputs**:
  - `MultimediaBoardLightingState` enums triggered by the `StateMachineHandler`.
  - Custom `RGB` struct objects.
- **Outputs**:
  - `LEDRGB`: A RoveComm UDP packet sent to the physical multimedia/core board.

## Usage in State Machine
When the autonomy software transitions from `eIdle` to `eNavigating`, it calls `SendLightingState(MultimediaBoardLightingState::eAutonomy)`. When the `eVerifyingPosition` state succeeds, it might call `eReachedGoal`. If a catastrophic failure occurs or the software is shut down, it transitions back to `eOff` or `eTeleOp`.
