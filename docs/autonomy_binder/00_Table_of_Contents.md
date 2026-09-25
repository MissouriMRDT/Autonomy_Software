# Autonomy Binder Table of Contents

Welcome to the Autonomy Binder! This is the centralized engineering reference and operations manual for the Mars Rover Design Team autonomy software.

---

## Binder Navigation

### General Architecture & Operations
- [01. Quick Start and Operations](01_Quick_Start_and_Ops.md)
- [02. Architecture Overview](02_Architecture_Overview.md)
- [03. The State Machine](03_The_State_Machine.md)
- [URC Autonomy Rules](14_URC_Rules/URC_Autonomy_Rules.md)

### Subsystems Deep Dive
- [Perception Subsystem](04_Subsystems_Deep_Dive/Perception.md)
- [Path Planning Subsystem](04_Subsystems_Deep_Dive/Path_Planning.md)
- [Control and Actuation Subsystem](04_Subsystems_Deep_Dive/Control_and_Actuation.md)

### Controllers
- [PID Controller](13_Controllers/PID_Controller.md)
- [Predictive Stanley Controller](13_Controllers/Predictive_Stanley_Controller.md)
- [Pure Pursuit Controller](13_Controllers/Pure_Pursuit_Controller.md)

### Vision Pipeline
- [Cameras & Hardware Drivers](12_Vision/Cameras.md)
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
- [Recording Handler](08_Handlers/RecordingHandler.md)

### Board Drivers
- [Drive Board Driver](07_Board_Drivers/DriveBoard.md)
- [Navigation Board Driver](07_Board_Drivers/NavigationBoard.md)
- [Multimedia Board Driver](07_Board_Drivers/MultimediaBoard.md)

### Threading & Infrastructure
- [Autonomy Thread Interface](09_Threading/AutonomyThread.md)
- [Thread Pools](09_Threading/ThreadPool.md)

### Networking
- [RoveComm Protocol](10_Networking/RoveComm.md)

### Logs, Data, and Visualization
- [Log Files & Quill Logging Engine](11_Logs_and_Data/Log_Files.md)
- [Camera Feeds & Video Recording](11_Logs_and_Data/Camera_Feeds_and_Recording.md)
- [Path Plots, Analytics, and Post-Mortem Graphing](11_Logs_and_Data/Path_Plots.md)
- [3D Interactive Visualization & The Visualization Engine](11_Logs_and_Data/Visualization.md)

### Configuration and Tuning
- [Autonomy Constants Reference & Tuning Guide](05_Configuration_and_Tuning/Autonomy_Constants.md)
- [CMake Build Configuration & Options](05_Configuration_and_Tuning/CMake_Options.md)

### Troubleshooting & Checklists
- [Troubleshooting & Field Diagnostics Guide](06_Troubleshooting_Guide.md)
- [Autonomy Pre-Flight & Operations Checklist](15_Checklists/Autonomy_Checklist.md)

---

## External Resources & Team Portals

| Resource | URL | Description |
| :--- | :--- | :--- |
| **MRDT Documentation Portal** | [docs.themrdt.org](https://docs.themrdt.org/) | Central organization wiki, systems specifications, and team-wide documentation. |
| **Software Documentation Repo** | [MissouriMRDT/RoveSoDocs](https://github.com/MissouriMRDT/RoveSoDocs) | Source repository for team documentation and technical binders. |
| **MRDT GitLab Organization** | [gitlab.themrdt.org/MissouriMRDT](https://gitlab.themrdt.org/MissouriMRDT) | Team GitLab instance hosting internal repositories, CI pipelines, and large datasets. |
| **USGS LiDAR Data Repository** | [MissouriMRDT/USGS_Data](https://gitlab.themrdt.org/MissouriMRDT/USGS_Data) | Primary storage for preprocessed USGS 3DEP LiDAR point clouds and DuckDB spatial databases. |
| **Autonomy Installation Guide** | [INSTALL.md](https://github.com/MissouriMRDT/Autonomy_Software/blob/development/INSTALL.md) | Step-by-step native and container installation guide for development and Jetson deployment. |
| **Contributing Guidelines** | [CONTRIBUTING.md](https://github.com/MissouriMRDT/Autonomy_Software/blob/development/CONTRIBUTING.md) | Branching guidelines, coding conventions, Git workflow, and pull request requirements. |
| **Web Visualizer Suite** | [visualizer.themrdt.org](https://visualizer.themrdt.org/) | Hosted web-based mission planning and data inspection suite. |
| **Autonomy Task Visualizer** | [visualizer.themrdt.org/autonomy-task/](https://visualizer.themrdt.org/autonomy-task/) | Web tool for planning, simulating, and reviewing autonomy waypoints and navigation trajectories. |
| **LiDAR Visualizer Tool** | [visualizer.themrdt.org/lidar-tool/](https://visualizer.themrdt.org/lidar-tool/) | Web tool for 3D point cloud inspection, terrain cross-sections, and slope visualization. |

---

## Operational Cheat Sheet

| Operation | Command |
| :--- | :--- |
| **Clean Build Directory** | `rm -rf build && mkdir build` |
| **CMake Configuration (Release)** | `cmake -B build -DCMAKE_BUILD_TYPE=Release` |
| **Compile Executable** | `make -C build -j$(nproc)` |
| **Run Autonomy Software** | `./build/Autonomy_Software` |
| **Run Simulation Mode** | `cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SIM_MODE=ON && make -C build -j$(nproc)` |
| **Execute Test Suites** | `cmake -B build -DBUILD_TESTS_MODE=ON && make -C build -j$(nproc) && ctest --test-dir build --output-on-failure` |
| **Interactive Hotkeys (In Terminal)** | `h` (Help), `f` (FPS), `p` (Pose), `s` (Sensors), `d` (Drive Powers), `t` (Tags), `m` (Objects), `q` (Quit) |
| **Compile Binder to Single Markdown / PDF** | `bash tools/compile_binder_pandoc.sh` or `bash tools/compile_binder_pdf.sh` |
