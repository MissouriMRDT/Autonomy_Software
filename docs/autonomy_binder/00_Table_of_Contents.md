# Autonomy Binder Table of Contents

Welcome to the Autonomy Binder! This is the centralized reference for all things autonomy.

## Navigation

- [01. Quick Start and Operations](01_Quick_Start_and_Ops.md)
- [02. Architecture Overview](02_Architecture_Overview.md)
- [03. The State Machine](03_The_State_Machine.md)
- [URC Autonomy Rules](14_URC_Rules/URC_Autonomy_Rules.md)

### Subsystems Deep Dive
- [Perception](04_Subsystems_Deep_Dive/Perception.md)
- [Path Planning](04_Subsystems_Deep_Dive/Path_Planning.md)
- [Control and Actuation](04_Subsystems_Deep_Dive/Control_and_Actuation.md)

### Controllers
- [PID Controller](13_Controllers/PID_Controller.md)
- [Predictive Stanley Controller](13_Controllers/Predictive_Stanley_Controller.md)

### Vision
- [Cameras](12_Vision/Cameras.md)
- [ArUco Tag Detection](12_Vision/ArUco_Tag_Detection.md)
- [Object Detection](12_Vision/Object_Detection.md)
- [Vision Utilities](12_Vision/Vision_Utilities.md)

### Handlers
- [State Machine Handler](08_Handlers/StateMachineHandler.md)
- [Camera Handler](08_Handlers/CameraHandler.md)
- [Tag Detection Handler](08_Handlers/TagDetectionHandler.md)
- [Object Detection Handler](08_Handlers/ObjectDetectionHandler.md)
- [LiDAR Handler](08_Handlers/LiDARHandler.md)
- [Waypoint Handler](08_Handlers/WaypointHandler.md)

### Board Drivers
- [Drive Board](07_Board_Drivers/DriveBoard.md)
- [Navigation Board](07_Board_Drivers/NavigationBoard.md)
- [Multimedia Board](07_Board_Drivers/MultimediaBoard.md)

### Threading & Infrastructure
- [Autonomy Thread Interface](09_Threading/AutonomyThread.md)
- [Thread Pools](09_Threading/ThreadPool.md)

### Networking
- [RoveComm Protocol](10_Networking/RoveComm.md)

### Logs, Data, and Visualization
- [Log Files & Quill](11_Logs_and_Data/Log_Files.md)
- [Camera Feeds & Recording](11_Logs_and_Data/Camera_Feeds_and_Recording.md)
- [Path Plots & Matplot++](11_Logs_and_Data/Path_Plots.md)
- [3D Interactive Visualization](11_Logs_and_Data/Visualization.md)

### Configuration and Tuning
- [Autonomy Constants](05_Configuration_and_Tuning/Autonomy_Constants.md)
- [CMake Options](05_Configuration_and_Tuning/CMake_Options.md)

### Troubleshooting & Checklists
- [Troubleshooting Guide](06_Troubleshooting_Guide.md)
- [Pre-Flight Checklist](15_Checklists/Autonomy_Checklist.md)

## Cheat Sheet

| Action | Command |
| :--- | :--- |
| **Clean Build Directory** | `rm -rf build/*` |
| **Standard Build Config** | `cmake -DCMAKE_BUILD_TYPE=Release -B build/` |
| **Compile** | `make -C build/ -j$(nproc)` |
| **Run Autonomy** | `./build/Autonomy_Software` |
| **Run Tests** | `cd build && ctest` |
