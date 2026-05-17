# Waypoint Handler

The `WaypointHandler` is a globally accessible, thread-safe database for managing the coordinates the rover needs to navigate to, as well as coordinates the rover needs to avoid.

## Primary Responsibilities
1. **Mission Queueing**: Stores an ordered list (`std::vector<geoops::Waypoint>`) of the primary objectives/waypoints the rover must visit during a mission.
2. **Path Storage**: Allows the `GeoPlanner` to store the generated A* path (the intermediate steps between the rover and the primary waypoint).
3. **Obstacle Memory**: Stores global coordinates of dynamically detected obstacles so the path planner remembers them even after they leave the camera's field of view.

## Architecture & Threading
- **Global Access**: Instantiated in `AutonomyGlobals.cpp` as `globals::g_pWaypointHandler`, it acts as a central hub.
- **Thread Safety**: Uses `std::shared_mutex` to allow the GUI (via RoveComm TCP) to append new waypoints to the mission list simultaneously while the `NavigatingState` is reading the list to determine where to drive next.

## Usage
- The Basestation sends a TCP packet with a list of Lat/Lon coordinates.
- A callback intercepts this, converts them to `geoops::Waypoint` (UTM), and calls `globals::g_pWaypointHandler->AddWaypoint()`.
- The `NavigatingState` calls `PeekNextWaypoint()` to get the current goal. Once reached, it calls `PopNextWaypoint()` to remove it from the queue and move on to the next objective.
