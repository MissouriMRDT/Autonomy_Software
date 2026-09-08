# Autonomy Pre-Flight & Operations Checklist

This checklist defines the required engineering procedures for configuring, verifying, compiling, and operating the autonomy software during field trials and official University Rover Challenge (URC) competition runs.

---

## 1. On-Rover Jetson Setup & Verification

### Codebase and Toolchain Verification
- [ ] **Verify Git Branch**:
  - `git status` (Confirm working directory is clean and checked out to the designated deployment branch).
- [ ] **Fetch Latest Commits**:
  - `git fetch origin && git pull`
- [ ] **Verify Build System & Compiler Version**:
  - `gcc --version` (Must strictly report GCC 10.x per CMake configuration rules).
- [ ] **Verify Configuration Constants** in `src/AutonomyConstants.cpp`:
  - `BUILD_SIM_MODE` is set to `OFF` in `CMakeLists.txt`.
  - `LIDAR_HANDLER_DB_PATH` points to the valid DuckDB database tile file.
  - `TAGDETECT_TORCH_MODEL` points to the verified TorchScript ArUco weights (`data/Models/best_tag.pt`).
  - `OBJECTDETECT_TORCH_MODEL` points to the verified TorchScript mission object weights (`data/Models/best_object.pt`).
  - Extrinsic offsets match the current physical camera and GPS mast placements.

### Compilation
- [ ] **Clean Build Configuration**:
  - `rm -rf build && mkdir build`
  - `cmake -B build -DCMAKE_BUILD_TYPE=Release`
  - Ensure CMake configuration prints `--   [ ]: Sim Mode: Disabled` and generates `Autonomy_Software` (not `Autonomy_Software_Sim`).
- [ ] **Compile**:
  - `make -C build -j$(nproc)`
  - Confirm binary links without warnings or missing shared library errors.

### Bench Sensor Verification (Prior to Mounting)
- [ ] **Check USB Device Tree**:
  - Run `lsusb` in the terminal.
  - Verify that each connected ZED 2i camera populates two distinct devices:
    - Camera Video Interface (`ID 2b03:f880`)
    - Sensor / IMU Microcontroller (`ID 2b03:f881`)
- [ ] **Execute Standalone Smoke Test**:
  - Launch executable: `./build/Autonomy_Software`
  - Press `f`: Confirm camera capture rates and detector threads achieve steady 30 FPS.
  - Press `s`: Confirm linear acceleration and gyro angular rates update dynamically when moving the camera.
  - Press `t` and `m`: Confirm tag and object detector models are loaded onto the CUDA device without memory exhaustion.
  - Press `q`: Confirm clean shutdown, database closure, and export of `visualization.html`.
- [ ] **Network Interface Isolation**:
  - Put the Jetson internal Wi-Fi adapter into Airplane Mode / Disabled state to prevent wireless interference with the 900 MHz and 5.8 GHz competition radio links.

---

## 2. Basestation Operations & Deployment

### Remote Session Launch
- [ ] **Establish Secure Shell Session**:
  - Open terminal on the basestation computer: `ssh pigeon@192.168.3.100` (Default password: `nandgate`).
- [ ] **Launch Autonomy Process**:
  - `cd ~/Documents/GitHub/Autonomy_Software`
  - `./build/Autonomy_Software`
- [ ] **Verify RoveComm Telemetry**:
  - Confirm log message reports: `RoveComm UDP and TCP nodes successfully initialized.`
  - Verify heartbeat packets populate in the Basestation GUI telemetry dashboard.

---

## 3. Mission Waypoint Injection & Leg Types

Waypoints are queued into the autonomy system via the Basestation GUI or through direct RoveComm packets. Ensure the appropriate leg type and parameters are injected:

### Leg Type Configurations

1. **GNSS Position Leg (`ADDPOSITIONLEG`)**:
   - **Command ID**: `11002`
   - **Payload**: `[Latitude, Longitude, WaypointID]`
   - Used for raw coordinate transit legs. The rover navigates until it enters `constants::NAVIGATING_REACHED_GOAL_RADIUS` (typically 1.5 m).

2. **ArUco Marker Leg (`ADDMARKERLEG`)**:
   - **Command ID**: `11003`
   - **Payload**: `[Latitude, Longitude, MarkerID, SearchRadius]`
   - **MarkerID Parameter**:
     - `0`, `1`, `2`, or `3`: Directs the rover to search specifically for that numerical tag ID.
     - `-1` (`manifest::Autonomy::AUTONOMYWAYPOINTTYPES::ANY`): Rover accepts any detected ArUco marker.
   - **SearchRadius**: Clamped between 0.0 m and 40.0 m. Specifies the Archimedean spiral search envelope upon reaching the GNSS post location.

3. **Mission Object Leg (`ADDOBJECTLEG`)**:
   - **Command ID**: `11004`
   - **Payload**: `[Latitude, Longitude, ObjectID, SearchRadius]`
   - **ObjectID Parameter**:
     - `-2` (`manifest::Autonomy::AUTONOMYWAYPOINTTYPES::MALLET`): Orange mallet target.
     - `-3` (`manifest::Autonomy::AUTONOMYWAYPOINTTYPES::WATERBOTTLE`): Water bottle target.
     - `-4` (`manifest::Autonomy::AUTONOMYWAYPOINTTYPES::ROCKPICK`): Geologist rock hammer target.
   - **SearchRadius**: Clamped between 0.0 m and 40.0 m.

---

## 4. Autonomous Mission Execution Workflow

- [ ] **Confirm Waypoint Queue Status**:
  - Check terminal output: Confirm the incoming leg appears with correct coordinates, ID, and search radius.
- [ ] **Initiate Autonomous Navigation**:
  - On the Basestation GUI, click **Start Autonomy** (dispatches `eStartAutonomy` packet).
  - Verify rover state machine transitions from `eIdle` to `eNavigating`.
- [ ] **Monitor Multimedia Board Status Lights**:
  - **Off**: Autonomy is stopped / idle.
  - **Solid Red**: Autonomy is actively driving.
  - **Flashing Green**: Rover has successfully navigated to the waypoint, detected the target tag, or completed object approach.
  - **Solid Blue**: Teleoperation override active.
- [ ] **Emergency Abort Protocol**:
  - In the event of an imminent collision or boundary violation, click **Abort** on the Basestation GUI or press the physical wireless E-Stop.
  - The state machine immediately clears motor commands via `DriveBoard::SendStop()` and returns to `eIdle`.

---

## 5. Post-Mission Data Archival

- [ ] **Terminate Autonomy Session**:
  - Press `q` in the active SSH terminal.
- [ ] **Archive Run Artifacts**:
  - Navigate to `logs/<YYYY-MM-DD_HH-MM-SS>/`.
  - Verify generation of:
    - `console_output.log` and `console_output.csv`
    - `visualization.html` (Open locally in browser to inspect traversed route and 3D point cloud)
    - `spatial_map.ply` (ZED stereoscopic mesh)
    - Recorded video streams (`MainCam_Raw.mp4`, `MainCam_TagOverlay.mp4`, `MainCam_ObjectOverlay.mp4`)
- [ ] **Log Trajectory Playback**:
  - Run `python3 tools/logging/log_playback.py logs/<timestamp>/console_output.csv` to review controller tracking fidelity and telemetry timelines.
