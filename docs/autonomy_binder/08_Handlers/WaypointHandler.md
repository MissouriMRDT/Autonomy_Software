# Waypoint Handler

The `WaypointHandler` (`src/handlers/WaypointHandler.h` & `WaypointHandler.cpp`) is a thread-safe registry that stores, sequences, and manages mission waypoints, intermediate planned paths, and permanent obstacle coordinates.

---

## 1. Primary Responsibilities

1. **Mission Queue Management**: Maintains the sequential queue of target waypoints (`std::vector<geoops::Waypoint>`) commanding autonomous rover movement.
2. **RoveComm Command Ingestion**: Registers asynchronous callbacks listening for Basestation mission commands (`ADDPOSITIONLEG`, `ADDMARKERLEG`, `ADDOBJECTLEG`, `CLEARWAYPOINTS`).
3. **Trajectory and Path Storage**: Provides key-value path caching (`StorePath()`, `RetrievePath()`) allowing `GeoPlanner` to save generated A* routes.
4. **Obstacle Memory**: Stores global coordinates of declared obstacles (`m_vPermanentObstacles`), ensuring the path planner retains obstacle awareness across mission legs.

---

## 2. RoveComm Network Callbacks

The handler intercepts incoming RoveComm packets transmitted by the Basestation GUI:

### A. Position Leg (`ADDPOSITIONLEG`)
- **Data Payload**: `[Latitude, Longitude, LegID]`
- **Action**: Constructs a `geoops::Waypoint` with type `geoops::WaypointType::eNavigationWaypoint` and appends it to the queue.

### B. Marker Leg (`ADDMARKERLEG`)
- **Data Payload**: `[Latitude, Longitude, MarkerID, SearchRadius]`
- **Action**: Clamps the search radius between 0 and 40 meters, assigns type `geoops::WaypointType::eTagWaypoint`, and appends the marker waypoint with the specified ArUco ID.

### C. Object Leg (`ADDOBJECTLEG`)
- **Data Payload**: `[Latitude, Longitude, ObjectID, SearchRadius]`
- **Action**: Parses `ObjectID` using `manifest::Autonomy::AUTONOMYWAYPOINTTYPES`:
  - `MALLET` $\rightarrow$ `geoops::WaypointType::eMalletWaypoint`
  - `WATERBOTTLE` $\rightarrow$ `geoops::WaypointType::eWaterBottleWaypoint`
  - `ROCKPICK` $\rightarrow$ `geoops::WaypointType::eRockPickWaypoint`
  - Clamps the search radius between 0 and 40 meters and appends the object waypoint.

### D. Clear Queue (`CLEARWAYPOINTS`)
- **Action**: Clears the waypoint queue and signals the state machine if an active navigation leg is running.

---

## 3. Thread Safety and Concurrency

- Internal vectors are protected by reader-writer locks using `std::shared_mutex`:
  - `m_muWaypointsMutex` protects the mission queue.
  - `m_muPathMutex` protects stored A* paths.
  - `m_muObstaclesMutex` protects declared obstacle coordinates.
- Multiple threads (such as `GeoPlanner`, `VisualizationHandler`, and `NavigatingState`) can read waypoints simultaneously using `std::shared_lock`, while incoming RoveComm callbacks acquire exclusive `std::unique_lock`.

---

## 4. Public Interface Summary

```cpp
// Queue Manipulation
void AddWaypoint(const geoops::Waypoint& stWaypoint);
geoops::Waypoint PeekNextWaypoint();
geoops::Waypoint PopNextWaypoint();
void ClearWaypoints();
int GetWaypointCount();
const std::vector<geoops::Waypoint> GetAllWaypoints();

// Path Storage (GeoPlanner)
void StorePath(const std::string& szPathName, const std::vector<geoops::Waypoint>& vWaypointPath);
const std::vector<geoops::Waypoint> RetrievePath(const std::string& szPathName);

// Obstacle Management
void AddObstacle(const geoops::Waypoint& stObstacle);
const std::vector<geoops::Waypoint> GetAllObstacles();
```
