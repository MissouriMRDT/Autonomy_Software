# LiDAR Handler

The `LiDARHandler` (`src/handlers/LiDARHandler.h` & `LiDARHandler.cpp`) manages runtime spatial queries against preprocessed LiDAR point cloud databases, providing real-time terrain topology and obstacle metrics to `GeoPlanner` and `VisualizationHandler`.

---

## 1. Primary Responsibilities

1. **DuckDB Database Interfacing**: Connects to pre-built DuckDB database files (`constants::LIDAR_HANDLER_DB_PATH`) containing millions of geospatial points derived from USGS 3DEP LAS 1.4 point clouds.
2. **Radial Spatial Lookups**: Executes fast spatial queries to extract terrain points within a specified radius $R$ of an Easting/Northing coordinate.
3. **Multi-Property Filtering**: Supports filtering points by classification (e.g., ground vs vegetation vs structures), surface normal vectors, terrain slope, roughness, and curvature.
4. **Traversal Metrics Provisioning**: Provides precomputed traversal scores to the `GeoPlanner` costmap generator.

---

## 2. Point Data Architecture

Points are stored and returned in the `LiDARHandler::PointRow` structure:

```cpp
struct PointRow
{
    int nID;                         // Unique point identifier
    double dEasting;                 // UTM Easting coordinate (meters)
    double dNorthing;                // UTM Northing coordinate (meters)
    double dAltitude;                // Altitude above sea level (meters)
    std::string szZone;              // UTM Zone designator
    std::string szClassification;    // Point classification (ground, rock, etc.)
    double dNormalX;                 // X component of local surface normal vector
    double dNormalY;                 // Y component of local surface normal vector
    double dNormalZ;                 // Z component of local surface normal vector
    double dSlope;                   // Surface slope (degrees)
    double dRoughness;               // Local terrain roughness metric
    double dCurvature;               // Surface curvature metric
    double dTraversalScore;          // Composite score [0.0 = impassable, 1.0 = smooth]
};
```

### Filtering via `PointFilter`
Queries can be conditioned using `LiDARHandler::PointFilter`, specifying min/max bounds on slope, roughness, curvature, and normal vectors to isolate specific terrain hazards.

---

## 3. Database Engine: DuckDB

The handler leverages **DuckDB** rather than traditional relational engines:
- **Columnar Execution Engine**: Optimized for analytical vectorized queries on large numerical datasets.
- **Embedded Operation**: Runs in-process without requiring background server daemons.
- **Thread Safety**: Uses `std::shared_mutex` to allow concurrent read queries across `GeoPlanner` and `VisualizationHandler` threads.

---

## 4. Usage Example

```cpp
// Opening the database during initialization (in main.cpp)
globals::g_pLiDARHandler = new LiDARHandler();
if (!globals::g_pLiDARHandler->OpenDB(constants::LIDAR_HANDLER_DB_PATH))
{
    LOG_ERROR(logging::g_qSharedLogger, "Failed to open LiDAR DuckDB database.");
}

// Querying terrain within a 15-meter radius of the rover
std::vector<LiDARHandler::PointRow> vNearbyPoints;
vNearbyPoints = globals::g_pLiDARHandler->GetPointsInRadius(stRoverUTM, 15.0);

// Filtering for steep obstacles (slope > 25 degrees)
LiDARHandler::PointFilter stFilter;
stFilter.dEasting  = stRoverUTM.dEasting;
stFilter.dNorthing = stRoverUTM.dNorthing;
stFilter.dRadius   = 20.0;
stFilter.dSlope    = LiDARHandler::PointFilter::Range<double>{25.0, 90.0};

std::vector<LiDARHandler::PointRow> vSteepObstacles;
vSteepObstacles = globals::g_pLiDARHandler->GetPointsWithFilter(stFilter);
```

---

## 5. LiDAR Data Sources & Web Inspection Tools

### A. USGS LiDAR Point Cloud Storage Repository
The spatial elevation and terrain point clouds queried by `LiDARHandler` are sourced from the USGS 3D Elevation Program (3DEP) and processed into indexed DuckDB databases. Raw LAS/LAZ point cloud tiles, pre-generated DuckDB database artifacts, and ingestion scripts are hosted on the team's GitLab server:
- **USGS LiDAR Dataset Repository**: [MissouriMRDT/USGS_Data](https://gitlab.themrdt.org/MissouriMRDT/USGS_Data)
- **MRDT GitLab Organization**: [MissouriMRDT GitLab](https://gitlab.themrdt.org/MissouriMRDT)

Developers running simulations or offline tests requiring local terrain maps should acquire the appropriate regional `.duckdb` tiles from `USGS_Data` and place them at the path configured in `constants::LIDAR_HANDLER_DB_PATH` (`data/LiDAR/` by default).

### B. Online LiDAR Visualizer Tool
Terrain point clouds, cross-sectional elevation profiles, and traversability slopes can be visualized interactively in the web browser without launching local DuckDB instances:
- **Interactive LiDAR Visualizer**: [visualizer.themrdt.org/lidar-tool/](https://visualizer.themrdt.org/lidar-tool/)

This tool supports inspecting 3D colored point distributions, evaluating elevation gradients, and testing traversability threshold configurations across competition terrains.

