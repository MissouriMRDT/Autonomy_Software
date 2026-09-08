# PID Controller

The `PIDController` class (`src/algorithms/controllers/PIDController.h`) implements a Proportional-Integral-Derivative controller with Feedforward support, anti-windup limits, continuous input wraparound, output slew rate limiting, and output low-pass filtering.

---

## 1. Primary Use Cases

The primary application in Autonomy Software is **Heading and Steering Control**:
- When turning the rover toward a goal waypoint or orienting the chassis toward an ArUco marker, the difference between goal heading and current heading is evaluated as an error signal.
- The PID controller outputs a normalized rotational effort $u \in [-1.0, 1.0]$ passed to the differential drive kinematics.

---

## 2. Mathematical Formulation

At discrete timestep $k$ with time delta $\Delta t = t_k - t_{k-1}$, the control signal $u(k)$ is computed as:

$$u(k) = u_P(k) + u_I(k) + u_D(k) + u_{FF}(k)$$

### Component Breakdown
1. **Proportional Term ($u_P$)**:
   $$u_P(k) = K_p \cdot e(k)$$
   Provides immediate corrective action proportional to instantaneous error $e(k) = r(k) - y(k)$ (where $r$ is the setpoint and $y$ is the process variable).
2. **Integral Term ($u_I$)**:
   $$u_I(k) = u_I(k-1) + K_i \cdot e(k) \cdot \Delta t$$
   Accumulates steady-state error over time. This term is critical for overcoming static ground friction in skid-steer systems, where small proportional errors fail to produce enough torque to initiate turning.
3. **Derivative Term ($u_D$)**:
   $$u_D(k) = K_d \cdot \frac{e(k) - e(k-1)}{\Delta t}$$
   Measures error rate of change to provide damping as the error approaches zero, counteracting overshoot and oscillation.
4. **Feedforward Term ($u_{FF}$)**:
   $$u_{FF}(k) = K_{ff} \cdot r(k)$$
   Provides baseline output effort driven directly by the setpoint value rather than the error signal.

---

## 3. Specialized Robotics Features

The `PIDController` class includes several features designed for physical ground robots:

### Continuous Input Wraparound
Compass headings wrap from $360^\circ$ to $0^\circ$. Without handling, navigating from $355^\circ$ to $5^\circ$ would compute an error of $-350^\circ$, causing a full counter-clockwise rotation instead of a $10^\circ$ clockwise turn.
- Calling `EnableContinuousInput(0.0, 360.0)` automatically detects the shortest angular distance across the boundary.

### Integral Windup Prevention
If the rover is physically obstructed, the integral term can accumulate unbounded error, causing massive overshoot or violent motor spin once the obstacle clears.
- `SetMaxIntegralEffort(double dMaxEffort)` clamps the maximum contribution of $u_I$:
  $$|u_I(k)| \le \text{constants::DRIVE\_PID\_MAX\_INTEGRAL\_TERM}$$

### Output Slew Rate Limiting (Ramp Rate)
Instantaneous step changes from $0.0$ to $1.0$ effort can strip motor gearbox teeth or trigger overcurrent cutoffs.
- `SetOutputRampRate(double dMaxRatePerSecond)` limits the rate of change of the output:
  $$|u(k) - u(k-1)| \le \text{constants::DRIVE\_PID\_MAX\_RAMP\_RATE} \cdot \Delta t$$

### Output Low-Pass Filter
Noisy IMU data can cause high-frequency derivative chatter.
- `SetOutputFilter(double dFilterAlpha)` applies an exponential moving average to smooth output signals before passing them to motor drivers:
  $$u_{\text{filtered}}(k) = \alpha \cdot u(k) + (1 - \alpha) \cdot u_{\text{filtered}}(k-1)$$

---

## 4. Tuning Parameters in `AutonomyConstants.cpp`

| Constant Name | Type | Purpose | Tuning Directive |
| :--- | :--- | :--- | :--- |
| `DRIVE_PID_PROPORTIONAL` | `double` | $K_p$ gain | Increase for faster heading response; decrease if the rover oscillates around the setpoint. |
| `DRIVE_PID_INTEGRAL` | `double` | $K_i$ gain | Increase if the rover stalls before finishing a turn; decrease if slow hunting oscillations occur. |
| `DRIVE_PID_DERIVATIVE` | `double` | $K_d$ gain | Increase to damp overshoot; decrease if high-frequency jitter occurs due to network/actuation delay. |
| `DRIVE_PID_FEEDFORWARD` | `double` | $K_{ff}$ gain | Baseline effort scaling; typically 0.0 for pure heading tracking. |
| `DRIVE_PID_MAX_INTEGRAL_TERM` | `double` | Ceiling on $u_I$ | Clamps integral effort to prevent windup during extended stalls. |
| `DRIVE_PID_MAX_RAMP_RATE` | `double` | Output slew limit | Caps maximum acceleration of commanded effort per second. |
| `DRIVE_PID_OUTPUT_FILTER` | `double` | Filter factor $\alpha$ | Controls output smoothing against IMU noise. |
| `DRIVE_PID_TOLERANCE` | `double` | Deadband tolerance | Error threshold within which the controller declares alignment achieved. |
