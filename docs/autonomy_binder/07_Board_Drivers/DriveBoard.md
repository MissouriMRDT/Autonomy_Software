# Drive Board Driver

The `DriveBoard` class (`src/drivers/DriveBoard.h` & `DriveBoard.cpp`) converts high-level speed and steering requests into physical motor powers and transmits them over RoveComm to the Core board microcontroller.

---

## 1. Primary Responsibilities

1. **Kinematics Processing**: Accepts linear speed and heading requests and calculates left and right track power percentages using Differential Drive inverse kinematics.
2. **Network Transmission**: Formats track powers into RoveComm UDP `DRIVELEFTRIGHT` packets and transmits them to the Core microcontroller.
3. **Terrain Slope Damping**: Intercepts pitch and roll telemetry from the rover's inclinometer to attenuate motor power on steep inclines, preventing tip-overs.
4. **Master Throttle Regulation**: Listens for Basestation `SETMAXSPEED` commands, scaling output powers across $[0.0, 1.0]$.
5. **Emergency Stop Command**: Provides `SendStop()` to immediately command $0.0$ power across both tracks.

---

## 2. Kinematics Pipeline (`CalculateMove`)

```cpp
void DriveBoard::CalculateMove(double dSpeed, double dGoalHeading, double dActualHeading);
```

The calculation follows three sequential steps:
1. **Heading Error and PID Effort**:
   Computes the angular delta:
   $$\theta_{\text{error}} = \theta_{\text{goal}} - \theta_{\text{actual}}$$
   The internal PID controller (`DRIVE_PID_*`) calculates a normalized rotational turn effort:
   $$\omega = \text{PID.Calculate}(\theta_{\text{error}}) \in [-1.0, 1.0]$$
2. **Differential Drive Inverse Kinematics**:
   Depending on configuration, the forward speed $v$ and turn effort $\omega$ are evaluated using:
   - **Arcade Drive**:
     $$\text{Left} = v + \omega, \quad \text{Right} = v - \omega$$
   - **Curvature Drive**:
     Scales turning sensitivity inversely with forward velocity to prevent dynamic rollovers at high speeds. Point-turning is permitted when forward speed is near zero.
   - Powers are normalized so neither track exceeds $\pm 1.0$, with optional input squaring (`DRIVE_SQUARE_CONTROL_INPUTS`).
3. **Terrain Damping Multiplier**:
   Multiplies raw track powers by `VariableDriveEffort()` and the global `m_dMaxDriveEffort` multiplier:
   $$P_{\text{final}} = P_{\text{raw}} \cdot \text{Damp}_{\text{slope}} \cdot \text{Multiplier}_{\text{throttle}}$$
   Final outputs are clamped to `constants::DRIVE_MAX_SAFE_POWER`.

---

## 3. Inclinometer Safety Damping (`VariableDriveEffort`)

The driver registers a RoveComm callback listening for `manifest::Core::TELEMETRY["INCLINOMETERDATA"]`:
- Extracts chassis `Pitch` and `Roll` in degrees.
- Computes effective slope angle $\phi$:
  $$\phi = w_{\text{roll}} \cdot |\text{Roll}| + w_{\text{pitch}} \cdot |\text{Pitch}|$$
  where weights are defined by `constants::DRIVE_BOARD_ROLL_WEIGHT` and `constants::DRIVE_BOARD_PITCH_WEIGHT`.
- If $\phi \le \text{constants::DRIVE\_BOARD\_MIN\_SLOPE}$ ($10^\circ$), damping factor is $1.0$.
- If $\phi \ge \text{constants::DRIVE\_BOARD\_MAX\_SLOPE}$ ($30^\circ$), damping factor clamps to `constants::DRIVE_BOARD_MIN_DAMP` ($0.50$).
- Between these limits, linear interpolation smoothly decreases drive power.

---

## 4. Public Interface Summary

```cpp
// Kinematics and Movement
void CalculateMove(double dSpeed, double dGoalHeading, double dActualHeading);
void SendDrive();
void SendStop();

// Power Inspection & Setters
diffdrive::DrivePowers GetDrivePowers() const;
void SetMaxDriveEffort(const double dMaxDriveEffort);
double GetMaxDriveEffort() const;

// Differential Drive Mode Selection
void SetDifferentialControlMethod(diffdrive::DifferentialControlMethod eMethod);
```
