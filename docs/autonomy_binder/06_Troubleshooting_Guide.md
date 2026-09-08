# Autonomy Troubleshooting & Field Diagnostics Guide

This document provides systematic diagnostics and rapid-resolution procedures for common hardware, networking, algorithmic, and software anomalies encountered during field testing and competition operations.

---

## 1. Live Terminal Diagnostic Hotkeys

The autonomy main loop runs in non-canonical terminal mode (`SetNonCanonicalTerminalMode()`), enabling real-time keyboard commands without requiring a trailing newline. When investigating system behavior via SSH, press the following single keys:

| Key | Diagnostic Action | Information Output |
| :---: | :--- | :--- |
| **`h`** | Print Help Menu | Displays the catalog of available terminal diagnostic keys. |
| **`f`** | Thread Performance Telemetry | Dumps instantaneous execution frequencies (FPS) across all 11 worker threads (Main, Cameras, Detectors, State Machine, RoveComm UDP/TCP, Visualizer). |
| **`p`** | Rover Spatial Pose | Dumps current UTM coordinates (Easting, Northing, Altitude) and compass heading from `SmartRetrieveRoverPose()`. |
| **`s`** | ZED Inertial Telemetry | Dumps raw linear acceleration (X, Y, Z), angular velocity (X, Y, Z), and Euler gyro orientation from the forward ZED camera. |
| **`d`** | Drive Actuation State | Dumps real-time normalized motor power commands (Left Power, Right Power) currently sent to the drive board. |
| **`t`** | ArUco Marker Telemetry | Dumps total detected tags, Best OpenCV tag ID, distance (m), and yaw angle, alongside Best Torch tag metrics. |
| **`m`** | Object Detection Telemetry | Dumps total identified mission objects, best detected class (Mallet, Water Bottle, Rock Pick), distance, and yaw angle. |
| **`q`** | Graceful System Shutdown | Signals all threads to halt, stops motor output, flushes logs, generates `visualization.html`, and exports `spatial_map.ply`. |

---

## 2. Common Symptom-Based Solutions

### 1. Drivetrain Jitters or Oscillates While Driving
- **Underlying Cause**: Closed-loop heading PID controller is over-tuned, or the Navigation Board IMU is subject to electromagnetic interference from motor power cables.
- **Diagnostic Action**: Press `d` to observe drive power fluctuations. If powers rapidly oscillate between positive and negative extremes, the heading controller is unstable.
- **Resolution**:
  1. Decrease `DRIVE_PID_PROPORTIONAL` ($K_p$) by 20% to 30% in `AutonomyConstants.cpp`.
  2. Increase `DRIVE_PID_DERIVATIVE` ($K_d$) slightly to dampen overshoot.
  3. Increase `DRIVE_PID_OUTPUT_FILTER` to smooth high-frequency actuation chatter.
  4. Press `p` and verify that the compass heading is stable while driving in a straight line.

### 2. Rover Stalls When Attempting Point Turns
- **Underlying Cause**: Static ground friction exceeds motor breakaway torque, or integral anti-windup clamping is restricting power build-up.
- **Diagnostic Action**: Observe if drive powers plateau at low values without overcoming carpet or soft soil resistance.
- **Resolution**:
  1. Increase `DRIVE_PID_INTEGRAL` ($K_i$) slightly to accelerate error integration.
  2. Increase `DRIVE_PID_MAX_INTEGRAL_TERM` in `AutonomyConstants.cpp` to allow higher sustained current during stationary turning.
  3. Verify battery voltage is above 3.5V per cell to prevent voltage sag brownouts under heavy steering load.

### 3. "RoveComm UDP/TCP Node Failed to Initialize"
- **Underlying Cause**: Socket port 11000 is already bound by a stale background instance, a failed prior run, or an unclosed Python utility.
- **Diagnostic Action**: Inspect terminal logs for `RoveComm did not initialize properly! UDPNode Status: 0, TCPNode Status: 0`.
- **Resolution**:
  1. Check for processes occupying the port:
     ```bash
     sudo lsof -i :11000
     sudo lsof -i :11001
     ```
  2. Terminate the blocking process:
     ```bash
     sudo kill -9 <PID>
     ```
  3. Restart `./Autonomy_Software`.

### 4. ZED Camera Connection Failures or Framerate Dropping to Zero
- **Underlying Cause**: USB 3.0 power fluctuations, kernel USB autosuspend, or loose USB-C locking screws.
- **Diagnostic Action**: Run `lsusb` in the terminal. Each connected ZED 2i camera should present two separate USB endpoints (one for video, one for internal sensors/IMU):
  ```text
  Bus 002 Device 004: ID 2b03:f880 STEREOLABS ZED 2i
  Bus 001 Device 007: ID 2b03:f881 STEREOLABS ZED 2i MCU
  ```
- **Resolution**:
  1. If endpoints are missing, reseat the USB-C locking connector on the Jetson carrier board.
  2. Restart the Jetson USB subsystem or reset the camera device:
     ```bash
     sudo systemctl restart udev
     ```
  3. Press `f` in the autonomy terminal to verify camera FPS recovers to 30 FPS.

### 5. Autonomy Unexpectedly Enters `IdleState`
- **Underlying Cause**: Battery failsafe triggered, or all recovery attempts in `StuckState` were exhausted.
- **Diagnostic Action**: Inspect `console_output.log` or press `f` to check current state.
- **Resolution**:
  1. Look for log message: `Battery cell voltage below minimum!`. If present, swap the main LiPo battery.
  2. Look for log message: `Exhausted all recovery attempts in StuckState!`. If present, the rover was trapped by insurmountable terrain; inspect physical rover clearance and obstacle map.

### 6. Visual Odometry Drift and Discontinuous Jumps
- **Underlying Cause**: Stereoscopic feature loss in featureless terrain (e.g., flat sand or monochrome asphalt) causing the ZED SDK visual odometry to diverge from GNSS coordinates.
- **Diagnostic Action**: Check if `STATEMACHINE_ZED_REALIGN_THRESHOLD` warnings appear repeatedly in the log.
- **Resolution**:
  1. Press `p` to inspect current rover pose coordinates.
  2. If visual odometry has drifted, press `q` to halt cleanly and restart autonomy software to force visual odometry to re-anchor to GNSS UTM coordinates.
  3. Ensure camera lenses are clean of dust, mud, and water droplets.

### 7. LiDAR Database Opening Failure
- **Underlying Cause**: The DuckDB spatial database file is missing, corrupt, or has incorrect file permissions.
- **Diagnostic Action**: Look for `Failed to open LiDAR database.` at launch.
- **Resolution**:
  1. Verify the file path configured in `constants::LIDAR_HANDLER_DB_PATH`.
  2. Check that the `.db` file exists and has read permissions:
     ```bash
     ls -la data/LiDAR/
     ```
  3. Ensure DuckDB shared libraries are correctly located in the system library path.
