# Quick Start and Operations Manual

This guide covers the exact steps required to set up, build, test, and run the Autonomy Software, whether developing inside the Dev Container, simulating with Unreal Engine RoveSoSimulator, or deploying directly to physical rover hardware.

---

## 1. Setting up the Dev Container Environment

The Autonomy Software project uses Docker and Visual Studio Code Dev Containers to ensure consistent toolchains and dependencies across development machines and the onboard Jetson computers.

### Prerequisites
- Visual Studio Code with the `Dev Containers` extension (`ms-vscode-remote.remote-containers`).
- Docker Engine (Linux) or Docker Desktop (Windows/macOS).
- NVIDIA Container Toolkit (required on Linux/Jetson hosts for GPU pass-through, CUDA, and TensorRT acceleration).

### Building and Attaching
1. Clone the repository recursively to populate submodules (including RoveComm):
   ```bash
   git clone --recursive https://github.com/MissouriMRDT/Autonomy_Software.git
   ```
2. Open the repository root in VSCode:
   ```bash
   code Autonomy_Software
   ```
3. When prompted in the lower-right notification, select **Reopen in Container**.
   - If the prompt does not appear, open the Command Palette (`Ctrl + Shift + P` or `Cmd + Shift + P`), type `Dev Containers: Rebuild and Reopen in Container`, and press Enter.
4. Docker pulls the preconfigured image and initializes the workspace. First-time initialization may require several minutes to complete.

### Container Environment Details
The container environment provides:
- **Compilers and Toolchain**: GCC 10.0 (strictly enforced by `CMakeLists.txt`), CMake 3.24.3+, LLD linker (`-fuse-ld=lld`), C++20 standard, CUDA 12 / 11 matching the ZED SDK.
- **Machine Learning**: LibTorch (PyTorch C++ frontend) with CUDA acceleration for YOLO object and tag detection.
- **Computer Vision**: OpenCV 4.x with CUDA modules, Stereolabs ZED SDK 4.x.
- **Geospatial and Scientific**: GeographicLib (geodesic and UTM projections), Eigen3, Point Cloud Library (PCL), OpenMPI, OpenMP.
- **Storage and Networking**: DuckDB (USGS LiDAR point cloud queries), RoveComm (in-house UDP/TCP protocol), LibDataChannel (WebRTC for simulator camera feeds), nlohmann-json.
- **Logging**: Quill asynchronous logging engine.

---

## 2. Building the Code

The build system is orchestrated using CMake.

> [!IMPORTANT]
> GCC 10.0 is strictly enforced by `CMakeLists.txt` via `gcc -dumpversion`. Building with other GCC versions will result in a fatal configuration error.

### Standard Build (Release Mode)
Always use Release mode on the rover and during competitive runs for maximum compiler optimization (`-O3` equivalent) and loop vectorization:

```bash
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

To limit memory usage during parallel compilation, CMake Unity builds are enabled by default (`CMAKE_UNITY_BUILD ON` with batch size 8). You can adjust parallel compile jobs via:
```bash
make -j4
```

### Simulation Mode Build
When testing without physical rover hardware, compile with `BUILD_SIM_MODE=ON`:

```bash
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SIM_MODE=ON ..
make -j$(nproc)
```

In simulation mode:
- The output binary is named `Autonomy_Software_Sim`.
- The software connects to Unreal Engine RoveSoSimulator via WebRTC (LibDataChannel) and local RoveComm endpoints rather than physical hardware boards and cameras.

### Unit and Integration Tests Build
To compile the Google Test suites:

```bash
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTS_MODE=ON ..
make -j$(nproc)
ctest --output-on-failure
```

Optional test suites for LiDAR and GeoPlanner require USGS database files and can be enabled with `-DENABLE_LIDAR_GEO_UTESTS=ON`.

### Cleaning the Build Directory
If CMake cache issues occur after switching branches or updating submodules:

```bash
rm -rf build/*
cmake -DCMAKE_BUILD_TYPE=Release -B build/
make -C build/ -j$(nproc)
```

---

## 3. Running Autonomy

Executables are written to the `build/` directory.

### Running on Physical Hardware
```bash
cd build
./Autonomy_Software
```

On launch, the software performs the following initialization sequence:
1. Initializes the Quill asynchronous loggers and prints the ASCII software header.
2. Binds RoveComm UDP and TCP sockets (UDP port from manifest, TCP interface IP from constants).
3. Instantiates board drivers (`DriveBoard`, `MultimediaBoard`, `NavigationBoard`).
4. Instantiates handlers (`WaypointHandler`, `LiDARHandler`, `CameraHandler`, `TagDetectionHandler`, `ObjectDetectionHandler`, `StateMachineHandler`).
5. Loads the USGS DuckDB database from `constants::LIDAR_HANDLER_DB_PATH`.
6. Instantiates `GeoPlanner` and starts `VisualizationHandler` on port 8080.
7. Starts camera background threads and begins detector pipelines.
8. Enters the main execution loop in `main.cpp`.

### Running in Simulation
```bash
cd build
./Autonomy_Software_Sim
```
Ensure that the Unreal RoveSoSimulator environment is executing before launching the simulation binary.

---

## 4. Interactive Terminal Controls

While `Autonomy_Software` is executing, the terminal is configured into non-canonical mode, enabling single-keypress diagnostic commands without pressing Enter:

| Key | Function | Output Description |
| :--- | :--- | :--- |
| `h` / `H` | **Help** | Prints the terminal hotkey command summary. |
| `f` / `F` | **FPS / IPS Stats** | Prints exact iterations-per-second for all camera, detector, networking, and state machine threads. |
| `p` / `P` | **Rover Pose** | Prints current Easting, Northing, Altitude (UTM), and fused compass heading. |
| `s` / `S` | **Sensor Data** | Queries the ZED IMU and prints linear acceleration (X, Y, Z), angular velocity, and Euler orientation. |
| `d` / `D` | **Drive Powers** | Prints current left and right drive power commands (-1.0 to 1.0). |
| `t` / `T` | **Tag Detections** | Dumps total detected ArUco markers, best OpenCV tag ID/distance/yaw, and best Torch tag detection. |
| `m` / `M` | **Object Detections** | Dumps total detected props (mallets, bottles, rock picks) with bounding box dimensions, confidence, and classes. |
| `q` / `Q` | **Graceful Quit** | Triggers clean shutdown: stops drives, signals state machine abort, saves 3D visualization, joins threads, and exits. |

---

## 5. Log Management

Text logging is powered by the **Quill** asynchronous engine, ensuring logging operations never block time-critical control threads.

- **Storage Location**: Logs are written to timestamped directories under `logs/` (e.g., `logs/YYYY-MM-DD_HH-MM-SS/`).
- **Sinks Configured**:
  - **Console Sink**: Colorized terminal output for immediate visibility.
  - **Rotating File Sink**: Written to disk with automatic size rotation.
  - **RoveComm Sink**: Broadcasts log records over UDP to the Basestation GUI console.
- **Log Levels**: Default levels are defined in `AutonomyConstants.cpp` (`CONSOLE_DEFAULT_LEVEL`, `FILE_DEFAULT_LEVEL`, `ROVECOMM_DEFAULT_LEVEL`) and can be modified at runtime via RoveComm `SETLOGGINGLEVELS` packets.

---

## 6. Quick Commands Reference

| Action | Command |
| :--- | :--- |
| **Clean Build Tree** | `rm -rf build/*` |
| **Configure Release** | `cmake -DCMAKE_BUILD_TYPE=Release -B build/` |
| **Compile All Targets** | `make -C build/ -j$(nproc)` |
| **Run Onboard** | `./build/Autonomy_Software` |
| **Run Simulation** | `./build/Autonomy_Software_Sim` |
| **Run Unit Tests** | `cd build && ctest --output-on-failure` |
