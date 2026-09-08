# Architecture Overview

This section describes the high-level software architecture of the Autonomy Software, detailing data pathways, inter-thread synchronization, external hardware communication, and coordinate transformations across navigation frames.

---

## 1. System Data Flow

The software operates as a modular, multithreaded architecture organized around a centralized state machine. The primary data pipeline proceeds through five stages:

```
[Sensors & Hardware]
      |
      v (RoveComm UDP / USB / CUDA)
[Drivers & Handlers] (NavBoard, DriveBoard, CameraHandler, LiDARHandler)
      |
      v (Thread-safe Queues / Shared Memory)
[Perception & Geolocation] (TagDetector, ObjectDetector, GeolocateBox)
      |
      v (UTM Waypoints & Sensor Fusion)
[State Machine] (StateMachineHandler -> Active State)
      |
      v (Goal Coordinates & Costmap Lookups)
[Path Planning] (GeoPlanner / A* / Search Patterns)
      |
      v (Target Waypoints & Speed Profile)
[Controllers & Kinematics] (Predictive Stanley / Pure Pursuit / PID / DifferentialDrive)
      |
      v (RoveComm UDP)
[Core & Motor Controllers]
```

### Data Pipeline Breakdown
1. **Sensors to Drivers and Handlers**:
   - Navigation telemetry (GPS latitude, longitude, altitude, and IMU compass heading) arrives over UDP via RoveComm from the physical Navigation Board.
   - Stereoscopic camera frames (RGB color matrices and 32-bit floating point depth/point-cloud matrices) are captured asynchronously on GPU streams via the Stereolabs ZED SDK (`ZEDCam.cpp`) or virtual WebRTC pipes (`SIMZEDCam.cpp`).
   - Spatial elevation and obstacle data are retrieved through DuckDB spatial queries against pre-indexed USGS LAS 1.4 point cloud databases (`LiDARHandler.cpp`).
2. **Handlers to Perception Pipelines**:
   - Image frames are passed to `TagDetectionHandler` and `ObjectDetectionHandler`.
   - `TagDetector` runs OpenCV ArUco dictionary matching (`DICT_4X4_50`) and an optional LibTorch YOLO model in parallel.
   - `ObjectDetector` runs custom LibTorch YOLO models (`.pt` TorchScript) with CUDA acceleration, followed by non-maximum suppression (NMS) and OpenCV CSRT/KCF bounding-box tracking.
   - When an object or marker is detected in pixel coordinates, `GeolocateBox()` queries the corresponding 3D camera point cloud (`CV_32FC4`), applies statistical depth filtering (20th percentile surface extraction), and projects the vector into global UTM space using the fused rover pose.
3. **Perception and Pose to State Machine**:
   - `StateMachineHandler` executes at a controlled frequency (`STATEMACHINE_MAX_IPS`, typically 60 Hz).
   - The handler runs `SmartRetrieveRoverPose()`, fusing low-frequency GPS heading and position with high-frequency ZED IMU angular rates and dynamic drift correction.
   - The active state evaluates target proximity, visual locks, stuck criteria, and mission timeouts to determine whether to transition states.
4. **State Machine to Path Planning**:
   - During `eNavigating`, the state machine passes the fused rover UTM position and the target destination to `GeoPlanner`.
   - `GeoPlanner` calculates a 2.5D costmap using USGS terrain data from `LiDARHandler`, applies obstacle inflation, and computes a collision-free path using Kinematically Constrained Weighted A*.
   - During `eSearchPattern`, the system mathematically constructs expanding spiral, zigzag, or snake patterns around the search radius.
5. **Path Planning to Kinematics and Motor Actuation**:
   - The planned path is passed to either the `PredictiveStanleyController`, `PurePursuitController`, or the heading PID controller.
   - The controller outputs a steering angle and linear velocity setpoint.
   - Kinematics routines in `DifferentialDrive.hpp` (Arcade Drive or Curvature Drive) translate linear and angular requests into normalized left and right track power commands (-1.0 to 1.0).
   - Inclinometer slope damping is applied based on rover pitch and roll.
   - The resulting powers are packaged into a RoveComm UDP `DRIVELEFTRIGHT` packet and transmitted to the Core motor controller board.

---

## 2. Concurrency and Threading Model

To avoid latency in control loops, compute-intensive processes are separated into dedicated asynchronous worker threads:

### Thread Architecture
- **`AutonomyThread<T>` Interface**: Fundamental base class for long-running worker loops (`CameraHandler`, `TagDetector`, `ObjectDetector`, `StateMachineHandler`, `VisualizationHandler`, `RoveCommUDP`). Each instance runs an independent OS thread, manages thread states (`eStarting`, `eRunning`, `eStopping`, `eStopped`), enforces iteration-per-second (IPS) rate limits, and safely cleans up in its destructor.
- **Thread Pools (`BS::thread_pool`)**: Each `AutonomyThread` instantiates an internal Barak Shoshany thread pool. This is used for bursting tasks without thread allocation overhead, such as dispatching asynchronous frame copies across multiple subscriber buffers (`RequestFrameCopy()`) or evaluating parallelized search loops.
- **Mutex Strategies**:
  - `std::shared_mutex` is used throughout the codebase for read-heavy resources (such as `WaypointHandler` lists and `LiDARHandler` database connections). Multiple consumers can acquire shared read locks (`std::shared_lock`) simultaneously, while mutations require an exclusive unique lock (`std::unique_lock`).
  - Atomic flags (`std::atomic<bool>`) are utilized for fast state toggles and lifecycle management without mutex lock contention.

---

## 3. Communication Architecture (RoveComm)

Communication between the Jetson computing platform and distributed rover subsystems uses the MRDT RoveComm protocol.

### Transport Protocols
- **UDP (User Datagram Protocol)**:
  - Used for real-time, high-rate, loss-tolerant telemetry.
  - Examples: motor power commands (`DRIVELEFTRIGHT`), GPS telemetry (`GPSLATLON`), IMU heading packets (`IMUDATA`), lighting controls (`STATEDISPLAY`, `LEDRGB`), and console log streaming.
  - Sockets do not block; dropped packets are superseded by subsequent iterations.
- **TCP (Transmission Control Protocol)**:
  - Used for state-critical, ordered, guaranteed commands.
  - Examples: mission waypoint injection (`ADDPOSITIONLEG`, `ADDMARKERLEG`, `ADDOBJECTLEG`), queue clears (`CLEARWAYPOINTS`), and logging level reconfigurations (`SETLOGGINGLEVELS`).

### Manifest Binding
All communication relies on `RoveCommManifest.h`. Packet headers define:
- `unDataId`: Unique 16-bit identifier for the command or telemetry stream.
- `unDataCount`: Number of elements contained in the payload array.
- `eDataType`: Primitive type (`UINT8_T`, `INT32_T`, `FLOAT_T`, `DOUBLE_T`).
Payload bytes are converted to and from network byte order using endianness helper macros (`htonll`, `ntohll`).

---

## 4. Coordinate Reference Frames

Navigational calculations span three distinct reference frames. Maintaining mathematical consistency across these frames is essential for accurate geolocation and tracking.

### 1. Global / World Reference Frame (UTM / NWU)
- **Coordinate System**: Universal Transverse Mercator (UTM).
- **Units**: Meters along a 2D Cartesian grid, with separate altitude in meters above sea level.
- **Orientation Standard**: North-West-Up (NWU):
  - **+X**: North
  - **+Y**: West
  - **+Z**: Up
- **Compass Heading**: Measured clockwise from true North ($0^\circ = \text{North}$, $90^\circ = \text{East}$, $180^\circ = \text{South}$, $270^\circ = \text{West}$).

### 2. Rover / Kinematics Body Frame
- **Origin**: Physical center of rotation of the rover chassis on the ground plane.
- **Axes Convention**:
  - **+X**: Forward (longitudinal direction of travel)
  - **+Y**: Left (lateral port side)
  - **+Z**: Up (vertical dorsal direction)
- **Rotations**: Follow standard right-hand rule about the +Z axis (counter-clockwise yaw is positive).

### 3. Camera / Perception Optical Frame
- **Camera Standard**: Stereolabs ZED SDK native coordinate system: `sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP`.
- **Axes Convention**:
  - **+X**: Right (across sensor face)
  - **+Y**: Up (vertical sensor plane)
  - **+Z**: Forward (optical axis into the scene)
- **Frame Transformation**:
  When an object is detected at 3D camera coordinates $(X_c, Y_c, Z_c)$, its position in global UTM coordinates $(E_o, N_o, U_o)$ is computed relative to camera pose $(E_c, N_c, U_c)$ and compass heading $\theta$ (converted to standard Cartesian angle $\alpha = 90^\circ - \theta$):
  $$E_o = E_c + \left(Z_c \cos \alpha + X_c \sin \alpha\right)$$
  $$N_o = N_c + \left(Z_c \sin \alpha - X_c \cos \alpha\right)$$
  $$U_o = U_c + Y_c$$
