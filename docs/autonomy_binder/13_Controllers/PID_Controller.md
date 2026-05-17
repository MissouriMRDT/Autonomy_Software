# PID Controller

The `PIDController` class (`src/algorithms/controllers/PIDController.h`) is a fundamental building block of the autonomy software's control logic. It encapsulates the principles of Proportional-Integral-Derivative control, with an added Feedforward component.

## Primary Uses
The most common use of the PID Controller in the autonomy software is for **Heading Control**.
When the rover needs to point towards a specific waypoint or an AR Tag, the `DifferentialDrive` kinematics use this PID controller to calculate exactly how much rotational effort (from -1.0 to 1.0) is needed to turn the rover based on the error between its current compass heading and the goal heading.

## Algorithm Breakdown

The PID output is the sum of up to four components:

1. **Proportional (P)**:
   - `Output = Error * Kp`
   - Adjusts the output directly in proportion to the current error. If the rover is very far off-target, it turns hard. If it's close, it turns gently.
2. **Integral (I)**:
   - `Output = Accumulated_Error * Ki`
   - Accumulates past errors over time. This is critical for overcoming static friction on the rover's wheels (especially on grass or carpet), as a tiny proportional error might not generate enough motor power to actually move the chassis.
3. **Derivative (D)**:
   - `Output = Rate_Of_Change * Kd`
   - Anticipates future error by measuring how fast the error is closing. This "dampens" the turn as the rover nears the target, preventing it from wildly overshooting.
4. **Feedforward (FF)**:
   - `Output = Target * Kff`
   - Used for predictive control, providing a baseline output effort based purely on the setpoint rather than the error.

## Key Features & Configurations

The `PIDController` class provides several features necessary for real-world robotics:

- **Continuous Input Wraparound**: By calling `EnableContinuousInput(0.0, 360.0)`, the controller understands that an actual heading of `350` and a goal heading of `10` only has an error of `20` degrees, not `340` degrees.
- **Integral Windup Protection**: `SetMaxIntegralEffort()` caps the maximum influence the I-term can have, preventing the rover from violently spinning out of control if it gets physically stuck for a few seconds.
- **Output Ramp Rate**: `SetOutputRampRate()` prevents the controller from instantly spiking from 0.0 to 1.0 effort, which could blow a fuse on the motor controllers. It artificially forces the output to ramp up smoothly.
- **Output Filtering**: `SetOutputFilter()` applies a low-pass filter to the output to smooth out high-frequency noise from the IMU.

## Tuning via Constants
The PID gains for the main drive system are located in `AutonomyConstants.cpp`:
- `DRIVE_PID_PROPORTIONAL`
- `DRIVE_PID_INTEGRAL`
- `DRIVE_PID_DERIVATIVE`

*(Refer to the `Configuration and Tuning / Autonomy Constants` page for a guide on how to tune these values).*
