# Architecture Overview

This section explains the high-level architecture of the autonomy software, including how data flows through the system, how the software modules communicate, and the coordinate frames used for navigation and perception.

## Data Flow

The Autonomy Software operates using a centralized architecture where a State Machine dictates the high-level actions of the rover based on processed sensor data and algorithms.

The typical flow of data during operation is:

1. **Sensors $\rightarrow$ Handlers**:
   - Physical hardware sensors (Cameras, LiDAR, GPS, IMU) capture raw data.
   - This data is read by **Board Drivers** (e.g., `NavigationBoard`, `MultimediaBoard`) or directly by **Hardware Handlers** (e.g., `CameraHandler`, `LiDARHandler`).
2. **Handlers $\rightarrow$ Perception Algorithms**:
   - Raw data is passed to perception modules via handlers (e.g., `TagDetectionHandler`, `ObjectDetectionHandler`).
   - Image frames and point clouds are processed using computer vision (OpenCV, ZED SDK) and deep learning models (YOLO via PyTorch/TensorFlow) to identify obstacles, AR tags, and objectives.
   - Using the `Geolocate` utility, pixel-coordinates of detected objects are transformed into global coordinate waypoints.
3. **Perception $\rightarrow$ State Machine**:
   - The `StateMachineHandler` continually polls the perception handlers and navigation drivers for the latest data (e.g., current GPS position, heading, detected tags, obstacles).
   - Based on this data, the State Machine evaluates transition conditions to determine the current state (e.g., `Navigating`, `Searching`, `Aligning`, etc.).
4. **State Machine $\rightarrow$ Path Planning**:
   - Based on the active state, a goal waypoint is determined.
   - The `GeoPlanner` (using A* or other algorithms) calculates an optimal path to the goal while avoiding known obstacles.
5. **Path Planning $\rightarrow$ Kinematics $\rightarrow$ Motors**:
   - The path is broken down into heading and speed setpoints.
   - These setpoints are passed through kinematics controllers (e.g., PID controllers and `DifferentialDrive` inverse kinematics like Arcade or Curvature drive) to determine the exact wheel speeds.
   - The final motor commands are sent to the `DriveBoard` driver, which transmits them to the physical motor controllers over the network.

## IPC (Inter-Process Communication) and Threading

The autonomy software heavily utilizes multithreading to ensure that blocking operations (like fetching camera frames or running neural network inferences) do not slow down the main state machine loop.

### Threading (`AutonomyThread` & `ThreadPool`)
- **`AutonomyThread`**: Many core components inherit from a custom `AutonomyThread` interface. This allows classes to run continuous background loops (e.g., retrieving frames from the ZED camera or continuously polling sensors) at independent tick rates.
- **`BS::thread_pool`**: Used for parallelizing tasks, such as queueing multiple frame copies simultaneously to increase throughput.

### Networking (`RoveComm`)
Communication between the autonomy software (running on the Jetson/main computer) and the microcontrollers/boards (Drive Board, Nav Board) is handled by a custom networking protocol called **RoveComm**.
- **UDP (User Datagram Protocol)**: Used for high-frequency, loss-tolerant data, such as sending continuous motor speed commands to the `DriveBoard` or receiving continuous GPS/IMU updates from the `NavigationBoard`.
- **TCP (Transmission Control Protocol)**: Used for critical data that must arrive reliably, such as configuration parameters or critical state changes.

## Coordinate Frames

Understanding the coordinate frames is critical for debugging navigation and perception issues. The software primarily uses three reference frames:

### 1. Global / World Frame (UTM & GPS)
- All significant waypoints, rover positions, and obstacle maps are maintained in **UTM Coordinates** (Universal Transverse Mercator), which provides a 2D Cartesian grid in meters.
- **GPS Coordinates** (Latitude/Longitude) are converted to UTM upon arrival from the Navigation Board to simplify Euclidean math.
- The world frame uses the **NWU (North-West-Up)** convention:
  - **+X**: North
  - **+Y**: West
  - **+Z**: Up

### 2. Rover / Kinematics Frame
- Used when calculating drive commands and relative movements (e.g., `DifferentialDrive.hpp`).
- The center of the rover is the origin.
- Follows the right-hand rule standard:
  - **+X**: Forward (Ahead of the rover)
  - **+Y**: Left
  - **+Z**: Up
  - **Heading**: Measured clockwise, where North is 0 degrees. Counter-clockwise rotation around the Z-axis is mathematically positive.

### 3. Camera / Perception Frame
- Used natively by the ZED SDK and OpenCV for point clouds and image matrices.
- The ZED SDK uses a **Left-Handed, Y-Up** coordinate system:
  - **+X**: Right
  - **+Y**: Up
  - **+Z**: Forward
- **Geolocation Transformation**: When an object is detected in the camera frame, the `GeolocateBox` function converts the `(X, Y, Z)` camera coordinates into the Global UTM frame by rotating the vector based on the rover's current absolute heading and translating it by the rover's current UTM position.
