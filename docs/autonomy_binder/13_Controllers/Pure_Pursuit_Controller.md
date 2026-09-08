# Pure Pursuit Controller

The `PurePursuitController` class (`src/algorithms/controllers/PurePursuitController.h`) provides an alternative geometric path-tracking controller that calculates the steering angle required to pursue a lookahead waypoint located a set distance down the reference path.

---

## 1. Overview and Theory

Pure Pursuit is a widely established geometric tracking algorithm. Unlike PID heading control (which aims at the nearest node) or Stanley control (which evaluates perpendicular cross-track error to the nearest line segment), Pure Pursuit fits a circular arc between the rover's current pose and a dynamic "lookahead" point on the reference trajectory.

### Core Principles
- **Lookahead Anchor**: The controller searches ahead along the path for a target coordinate positioned at a distance $L_d$ (the lookahead distance) from the rover.
- **Curvature Fitting**: It calculates the radius of curvature $R$ of the circular arc connecting the center of the rover to the lookahead point.
- **Steering Setpoint**: The heading setpoint points tangent to this circular arc, smoothing sharp corners and naturally anticipating upcoming bends in the path.

---

## 2. Mathematical Implementation

1. **Closest Waypoint Search**:
   The controller first determines the nearest waypoint index to the rover's current UTM position using Euclidean distance:
   $$i_{\text{closest}} = \arg\min_i \sqrt{(E_{\text{rover}} - E_i)^2 + (N_{\text{rover}} - N_i)^2}$$
2. **Lookahead Waypoint Selection**:
   Starting from $i_{\text{closest}}$, the algorithm iterates forward along the path segments until it locates the first point where distance from the rover exceeds the lookahead threshold:
   $$\text{dist}(\text{Rover}, \text{Waypoint}_j) \ge L_d$$
   If discrete waypoint spacing is large, it interpolates along the segment between waypoints to locate the exact intersection with the circle of radius $L_d$ centered at the rover.
3. **Heading Calculation**:
   The target heading $\theta_{\text{target}}$ is the bearing from the rover's current UTM position to the lookahead coordinate $(E_{\text{lookahead}}, N_{\text{lookahead}})$:
   $$\theta_{\text{target}} = \text{atan2}(E_{\text{lookahead}} - E_{\text{rover}}, N_{\text{lookahead}} - N_{\text{rover}}) \times \frac{180}{\pi}$$
   (adjusted to standard clockwise compass degrees where North is $0^\circ$).
4. **End-of-Path Deceleration and Stop**:
   When approaching the terminal path segment ($i \ge \text{size} - 2$), the controller projects the rover onto the final segment vector. If the normalized projection reaches or exceeds $1.0$, or if the Euclidean distance to the final node is $< 0.5$ meters, the controller commands zero velocity to cleanly halt the rover.

---

## 3. Configuration Parameters

The controller exposes constructor arguments and runtime mutators:

| Parameter | Default | Purpose and Impact |
| :--- | :--- | :--- |
| `dLookaheadDistance` | `2.0` meters | Distance along the path where the target point is selected. Increasing $L_d$ results in smoother trajectories but cuts corners. Decreasing $L_d$ tracks the path tighter but can induce lateral oscillations. |
| `nLookaheadIndex` | `5` | Fallback index offset when distance-based lookahead search reaches path limits. |
| `dMaxSpeed` | `constants::NAVIGATING_MOTOR_POWER` | Maximum linear velocity ceiling commanded by the controller. |

---

## 4. Output: `DriveVector`

The controller returns a `PurePursuitController::DriveVector` struct:

```cpp
struct DriveVector
{
    double dThetaHeading;  // Target absolute compass heading setpoint (degrees)
    double dVelocity;      // Target forward velocity (-1.0 to 1.0)
};
```

---

## 5. Usage Example

```cpp
// Instantiate with a 2.5 meter lookahead distance
controllers::PurePursuitController controller(2.5, 5);

// Set reference path from GeoPlanner
controller.SetReferencePath(vGeoPlannerPath);

// Execute within state machine loop
geoops::RoverPose stPose = globals::g_pStateMachineHandler->SmartRetrieveRoverPose();
controllers::PurePursuitController::DriveVector stVector = controller.Calculate(stPose, constants::NAVIGATING_MOTOR_POWER);

// Pass setpoints to DriveBoard kinematics
globals::g_pDriveBoard->CalculateMove(stVector.dVelocity, stVector.dThetaHeading, stPose.GetCompassHeading());
globals::g_pDriveBoard->SendDrive();
```
