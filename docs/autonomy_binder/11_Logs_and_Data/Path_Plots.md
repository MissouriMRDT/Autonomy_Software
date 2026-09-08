# Path Plots, Analytics, and Post-Mortem Graphing

Trajectory analysis, path planning verification, and control-loop diagnostics are supported by dedicated offline playback tooling and runtime logging analytics. This document details how log data is parsed, visualized, and evaluated to assess rover navigation performance.

---

## 1. The `log_playback.py` Visualization Suite

Located at `tools/logging/log_playback.py`, this Python analysis script parses the tab-delimited CSV log produced by Quill during an autonomy session (`logs/<timestamp>/console_output.csv`) and generates animated, synchronized multi-panel plots using `matplotlib`.

### Script Capabilities and Parsed Telemetry

The script uses regular expressions to extract structured metrics from the unstructured and semi-structured Quill log messages:

1. **3D Positional Trajectory**:
   - **GPS Position**: Extracted from `GPS Data: (<lat> lat, <lon> lon, <alt> alt)`.
   - **Fused Rover Pose**: Extracted from `Rover Pose: <lat> (lat), <lon> (lon), <alt> (alt), <deg> (degrees), GNSS/VIO FUSED? = <bool>`.
   - Plotted as dual 3D trajectory subplots (`Axes3D`) comparing raw GPS against the visual-inertial fused pose.

2. **Heading and Compass Alignment**:
   - **Compass Data**: Extracted from `Incoming Compass Data: <heading>`.
   - Plotted alongside the fused pose heading to detect local magnetic anomalies, declination drift, or IMU yaw lag.

3. **GNSS Accuracy and Fix Quality**:
   - Extracted from `Incoming Accuracy Data: (2D: <val>, 3D: <val>, Compass: <val>, FIX_TYPE: <fix>)`.
   - Tracks 2D horizontal accuracy, 3D spatial accuracy, and fix status across the run, highlighting GPS degradation under satellite occlusion.

4. **Thread Framerate Monitors**:
   - Extracted from periodic `Threads FPS` messages.
   - Tracks the performance of 11 concurrent threads simultaneously:
     - `main_process_fps`
     - `main_cam_fps`
     - `left_cam_fps`
     - `right_cam_fps`
     - `ground_cam_fps`
     - `main_detector_fps`
     - `left_detector_fps`
     - `right_detector_fps`
     - `state_machine_fps`
     - `rovecomm_udp_fps`
     - `rovecomm_tcp_fps`

5. **Drivetrain Power and State Overlay**:
   - Extracted from `Driving at: (<left_power>, <right_power>)` and `Current State: <state_name>`.
   - Displays real-time dynamic bar graphs of left and right motor efforts (scaled between -1.0 and +1.0) correlated directly with active state machine phases (`Navigating`, `ApproachingMarker`, `Stuck`, etc.).

6. **Waypoint Queue Transitions**:
   - Extracts waypoint additions and queue resets to demarcate leg boundaries visually along the timeline.

### Usage

Run the playback script from within the autonomy workspace:

```bash
python3 tools/logging/log_playback.py path/to/logs/2026-09-08_15-30-00/console_output.csv
```

---

## 2. Real-Time Path Tracking via `VisualizationHandler`

While `log_playback.py` operates offline after a run, real-time spatial trajectories and planned paths are maintained dynamically by the `VisualizationHandler`:

- **Path History (`m_vPathHistory`)**:
  - Accumulates `DisplayPoint` structs containing Easting, Northing, and Altitude relative to the local session origin `m_stOriginUTM`.
  - Captures the traversal score and the active rover state at each recorded point.
  - Rendered as a persistent trail in the Three.js 3D web interface.

- **Planned Path (`m_vPlannedPath`)**:
  - Queried at 1 Hz from `GeoPlanner::GetPlannedPath()`.
  - Transferred over HTTP JSON endpoints (`/api/planned_path`) to display upcoming A* trajectory splines and search pattern geometries.

---

## 3. Matplot++ Library Integration

For standalone benchmarking, algorithm evaluation, and C++ plotting routines, the build environment provides pre-compiled packages for **Matplot++** (located in `tools/package-builders/matplotplusplus/`). 

Matplot++ provides a C++ syntax mirroring MATLAB plotting functions, allowing developers to:
- Export costmap heatmaps and elevation contours directly from DuckDB point queries.
- Plot A* search trees, open sets, and closed sets during path planning algorithm tuning.
- Save high-resolution vector plots (`.svg` or `.png`) for technical design reports and competition review documentation.
