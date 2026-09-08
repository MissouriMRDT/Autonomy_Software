# 3D Interactive Visualization & The Visualization Engine

The autonomy software incorporates a real-time, interactive 3D digital twin engine hosted directly on the rover's Jetson processor. Managed by the `VisualizationHandler`, this subsystem aggregates sensor fusion telemetry, spatial LiDAR point clouds, planned path splines, and neural network detections into a live WebGL 3D scene accessible from any device on the rover's local network.

---

## 1. System Architecture

To avoid burdening the embedded compute platform with heavy native desktop UI dependencies (such as Qt or X11 OpenGL contexts), visualization is implemented via an asynchronous HTTP and WebGL architecture.

The `VisualizationHandler` inherits from `AutonomyThread<void>` and executes at 20 Hz (`SetMainThreadIPSLimit(20)`). It encapsulates a lightweight HTTP server (`SimpleWebServer`) that serves web assets and JSON telemetry endpoints while maintaining thread-safe internal state buffers.

```
+-------------------------------------------------------------------------------+
| Handlers & Subsystems                                                         |
|   LiDARHandler  -->  Spatial 2.5D Point Cloud (DuckDB)                        |
|   GeoPlanner    -->  Planned A* Splines & Search Geometries                   |
|   StateMachine  -->  Rover Pose (GPS + ZED IMU), Active State, Waypoints      |
|   Vision        -->  ArUco Tag & Object Detections (Mallet, Bottle, Pick)     |
+-------------------------------------------------------------------------------+
                                      |
                                      v (Thread-safe mutexes)
+-------------------------------------------------------------------------------+
| VisualizationHandler (Runs at 20 Hz on dedicated AutonomyThread)              |
|   - Anchors local origin (m_stOriginUTM) on first valid GPS coordinate        |
|   - Transforms UTM/Global coordinates into Origin-Relative (fX, fY, fZ)       |
|   - Manages DisplayPoint, DisplayWaypoint, and DisplayDetection buffers       |
+-------------------------------------------------------------------------------+
                                      |
                                      v
+-------------------------------------------------------------------------------+
| SimpleWebServer (Port 8080)                                                   |
|   Static Assets: /lib/three.js, /lib/orbit.js, /detections/*.png              |
|   Data Endpoints: /api/telemetry, /api/map, /api/planned_path,                |
|                   /api/waypoints, /api/detections, /api/detection_list        |
+-------------------------------------------------------------------------------+
                                      |
                                      v (HTTP / JSON)
+-------------------------------------------------------------------------------+
| Web Client (Laptop Browser, Basestation GUI, or visualizer.themrdt.org)       |
|   - Three.js WebGL Scene with OrbitControls                                   |
|   - Interactive camera panning, rotation, and elevation cross-sections        |
+-------------------------------------------------------------------------------+
```

---

## 2. Local Coordinate Origin Anchoring

Global UTM coordinates contain large Easting and Northing values (e.g., Easting ~ 500,000 m, Northing ~ 4,200,000 m). Directly feeding these values into single-precision 32-bit floating-point WebGL buffers introduces severe floating-point jitter and vertex distortion.

To eliminate this precision loss, `VisualizationHandler` initializes a session origin (`m_stOriginUTM`) upon receiving the first valid GPS coordinate from `StateMachineHandler::SmartRetrieveRoverPose()`:

```cpp
if (!m_bOriginSet)
{
    if (std::abs(stRoverUTM.dEasting) > 1.0 || std::abs(stRoverUTM.dNorthing) > 1.0)
    {
        m_stOriginUTM = stRoverUTM;
        m_bOriginSet  = true;
    }
}
```

All spatial vectors streamed over the web API are projected into this local tangential frame:
$$\Delta X = \text{Easting} - \text{Origin}_{\text{Easting}}$$
$$\Delta Z = \text{Northing} - \text{Origin}_{\text{Northing}}$$
$$\Delta Y = \text{Altitude} - \text{Origin}_{\text{Altitude}}$$

This yields millimeter-level visualization precision centered at `(0, 0, 0)`.

---

## 3. Core Data Structures

The handler packages spatial elements into compact C++ structs protected by dedicated mutexes:

```cpp
// Historical trajectory breadcrumb
struct DisplayPoint
{
    float fX, fY, fZ;    // Coordinates relative to m_stOriginUTM
    float fScore;        // Terrain traversal score from costmap
    int nState;          // Active robot state machine state
};

// Target waypoint or navigation beacon
struct DisplayWaypoint
{
    float fX, fY, fZ;    // Coordinates relative to m_stOriginUTM
    int nType;           // Waypoint type enum
};

// Persistent vision detection
struct DisplayDetection
{
    float fX, fY, fZ;    // Coordinates relative to m_stOriginUTM
    int nType;           // 10 = ArUco Tag, 11 = Mallet, 12 = Water Bottle, 13 = Rock Pick
};
```

---

## 4. HTTP API Endpoints

The internal `SimpleWebServer` exposes endpoints on port 8080 (configurable via `constants::VISUALIZER_WEBSERVER_PORT`):

| Endpoint | Method | Response Format | Purpose |
| :--- | :---: | :---: | :--- |
| `/` | `GET` | HTML (`text/html`) | Serves the embedded Three.js 3D web application interface. |
| `/lib/three.js` | `GET` | JavaScript | Serves the bundled Three.js library. |
| `/lib/orbit.js` | `GET` | JavaScript | Serves the Three.js OrbitControls camera manipulation library. |
| `/api/telemetry` | `GET` | JSON | Returns current rover pose (relative position, heading, pitch, roll, active state). |
| `/api/map` | `GET` | JSON | Queries `LiDARHandler` for spatial terrain points within a radius of the rover and returns point positions with traversal costs. |
| `/api/planned_path` | `GET` | JSON | Returns upcoming waypoint coordinates and the active A* trajectory spline. |
| `/api/waypoints` | `GET` | JSON | Returns all waypoints currently queued in `WaypointHandler`. |
| `/api/detections` | `GET` | JSON | Returns 3D positions and type tags of all confirmed visual detections. |
| `/api/detection_list`| `GET` | JSON | Returns a list of filenames for detection snapshot images captured on disk. |
| `/detections/<file>`| `GET` | PNG Image | Serves static detection snapshot images recorded during the run. |

---

## 5. Web Client Features

The frontend application renders the digital twin with the following layers:
- **Rover Model & Coordinate Frame**: Indicates current position and orientation in real-time.
- **Path History Ribbon**: Color-coded line tracing where the rover has driven, shaded by the traversal cost of the terrain beneath it.
- **Planned Path Spline**: Cyan path vector showing the route generated by `GeoPlanner`.
- **LiDAR Point Cloud**: Colored terrain scatter plot reflecting relative elevation and slope hazards.
- **Waypoints & Markers**: Cylindrical beacons showing goal locations, labeled by ID and search radius tolerances.
- **Detection Markers**: Specialized 3D glyphs highlighting confirmed objects (green for ArUco tags, orange for mallets, blue for water bottles, brown for rock picks).

---

## 6. Shutdown Environment Exports

When `src/main.cpp` executes its shutdown sequence (upon receiving `SIGINT` or user hotkey `Q`), persistent exports are saved into `logs/<timestamp>/`:

1. **Self-Contained HTML Export (`visualization.html`)**:
   - `pVisualizationHandler->SaveVisualization()` computes the bounding envelope of the entire run, queries all corresponding LiDAR tiles from the DuckDB database, and bakes the Three.js viewer, telemetry history, and 3D terrain points into a standalone `.html` file.
   - This file can be opened offline in any standard web browser without network connectivity or external web servers.

2. **ZED Spatial Mapping PLY Export (`spatial_map.ply`)**:
   - If ZED spatial mapping was active (`pMainCam->GetSpatialMappingState() == sl::SPATIAL_MAPPING_STATE::OK`), the main routine asynchronously requests the fused 3D mesh:
     ```cpp
     std::future<sl::Mesh> fuSpatialMap;
     pMainCam->ExtractSpatialMapAsync(fuSpatialMap);
     sl::Mesh slSpatialMap = fuSpatialMap.get();
     slSpatialMap.save(szFilePath.c_str(), sl::MESH_FILE_FORMAT::PLY);
     ```
   - The resulting `.ply` mesh can be loaded into CloudCompare, MeshLab, or Blender for detailed geometric inspection of terrain obstacles.
