# Control and Actuation Subsystem

The Control and Actuation subsystem is responsible for translating high-level navigation goals (like a target heading or speed) into physical motor movements.

## Algorithm Explanation

The autonomy software uses **Differential Drive** kinematics to steer the rover, as the rover operates like a tank (skid-steer) rather than a car (Ackermann steering).

1. **Heading Control (PID Controller)**:
   - To point the rover in the correct direction, we use a **Proportional-Integral-Derivative (PID)** controller (`DRIVE_PID_*` constants).
   - The error is calculated as the difference between the actual heading (from the compass/IMU) and the goal heading (the angle to the next waypoint).
   - The PID controller calculates a rotational effort output (from -1.0 to 1.0) to minimize this error.

2. **Inverse Kinematics (`DifferentialDrive.hpp`)**:
   - We utilize two primary inverse kinematic drive models:
     - **Arcade Drive**: Maps a forward speed and a rotation rate directly to left and right wheel speeds. If the rover is commanded to drive forward and turn right, the left wheels spin faster than the right wheels.
     - **Curvature Drive**: Controls the *curvature* of the robot's path rather than its rate of heading change. This makes the robot significantly more controllable and stable at high speeds compared to Arcade Drive. It optionally allows turning in place when the forward speed is near zero.
   - The system also includes an option to "Square Control Inputs" (`DRIVE_SQUARE_CONTROL_INPUTS`), which decreases sensitivity at low speeds, resulting in smoother fine-tuning when aligning to an object.

3. **Motor Commands**:
   - The final left and right speed constraints (-1.0 to 1.0) are sent to the `DriveBoard` driver.
   - The driver constructs a `RoveComm` UDP packet containing these values and sends it to the physical drive board microcontrollers over the ethernet network.

## Inputs and Outputs

- **Inputs**:
  - Goal Heading (from Path Planner or Vision servoing).
  - Goal Speed.
  - Actual Heading (from IMU/NavBoard).
- **Outputs**:
  - Left and Right drive powers (Normalized floats from -1.0 to 1.0).

## Known Limitations

- **Skid-Steer Friction**: Because the rover uses skid-steering, turning relies on the wheels slipping laterally against the ground. High friction surfaces (like asphalt or thick carpet) can cause the PID controller to stall or oscillate if the integral term (`DRIVE_PID_INTEGRAL`) is not tuned high enough to overcome static friction.
- **Actuator Latency**: There is a network latency between sending the UDP packet and the motor physically moving. Aggressive derivative (`DRIVE_PID_DERIVATIVE`) tuning can lead to jittering due to this delay.
