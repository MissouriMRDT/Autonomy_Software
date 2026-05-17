# Troubleshooting Guide

This guide is designed to help you quickly diagnose and fix common issues that occur on the competition field or during testing.

## Symptom-Based Fixes

### 1. Robot jitters or oscillates back and forth while driving.
- **Likely Cause:** The PID controller for heading is tuned too aggressively.
- **Fix:** Decrease `DRIVE_PID_PROPORTIONAL` or `DRIVE_PID_DERIVATIVE` in `AutonomyConstants.cpp`. Alternatively, check if the NavBoard IMU is experiencing high magnetic interference causing heading spikes.

### 2. Robot does not turn when trying to align, just stalls.
- **Likely Cause:** The friction on the ground is too high for the current PID integral term, or the battery voltage is low causing motor droop.
- **Fix:** Increase `DRIVE_PID_INTEGRAL` slightly to allow the controller to build up enough power to overcome static friction. Ensure the battery is fully charged.

### 3. Autonomy suddenly enters `eIdle` and stops driving.
- **Likely Cause:** A failsafe was triggered.
- **Fix:**
  1. Check the console/Quill logs for "Battery cell voltage below minimum". If true, replace the battery.
  2. Check if the State Machine detected a `eStuck` event and exhausted all `AttemptType` recovery maneuvers.

### 4. Robot drives into obstacles that are clearly visible on camera.
- **Likely Cause:** Coordinate frame desync, or ZED spatial mapping failure.
- **Fix:**
  1. Check if `STATEMACHINE_ZED_REALIGN_THRESHOLD` is triggering constantly in the logs. If so, restart the autonomy software to re-initialize the ZED positional tracking.
  2. Ensure the LiDAR or ZED depth streams are actually publishing data to the `GeoPlanner`.

### 5. Dev Container fails to build or crashes on startup.
- **Likely Cause:** Docker cache corruption or missing GPU drivers.
- **Fix:**
  1. Run `Dev Containers: Rebuild Container without Cache` in VSCode.
  2. Ensure the NVIDIA Container Toolkit is installed on the host machine if you are trying to utilize the GPU.

### 6. "RoveComm UDP/TCP Node Failed to Bind" error.
- **Likely Cause:** Another instance of Autonomy, a Python script, or another RoveComm tool is already running and holding the socket port open.
- **Fix:** Use `lsof -i :11000` (or `11001` if in SIM mode) to find the process ID holding the port, and use `kill -9 <PID>` to kill it.
