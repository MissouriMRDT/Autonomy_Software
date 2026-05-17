# LiDAR Handler

The `LiDARHandler` manages runtime queries against LiDAR point cloud data, allowing the autonomy software to understand spatial obstacles in real-time.

## Primary Responsibilities
1. **Database Interfacing**: Connects to a preloaded SQLite database containing massive point cloud datasets (often formatted from USGS LAS 1.4).
2. **Spatial Lookups**: Provides the `GeoPlanner` with real-time access to nearby obstacles by querying points within a specific radius of a given `(Easting, Northing)` UTM coordinate.
3. **Data Filtering**: Filters raw LiDAR points by classification (e.g., ignoring ground points, focusing on trees, rocks, or buildings).

## Architecture & Threading
- **SQLite Optimization**: Uses SQLite's spatial/R-Tree capabilities to perform highly optimized bounding-box queries on millions of points without consuming excessive RAM.
- **Thread Safety**: Uses `std::shared_mutex` to allow multiple subsystems (e.g., the A* planner and the Visualization server) to read points simultaneously while preventing corruption if the database is dynamically updated.

## Usage
The path planner uses this handler to inflate obstacles around the rover's current or projected path.
```cpp
std::vector<LiDARHandler::PointRow> vObstacles = globals::g_pLiDARHandler->GetPointsInRadius(stRoverUTM, dSearchRadius);
```
