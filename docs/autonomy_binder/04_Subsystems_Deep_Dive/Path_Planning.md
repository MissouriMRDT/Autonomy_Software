# Path Planning Subsystem

The Path Planning subsystem determines collision-free, kinematically viable trajectories from the rover's current global position to target waypoints across complex terrain.

---

## 1. Algorithmic Architecture

Path planning is orchestrated through two primary components: the **`GeoPlanner`** (for global terrain traversal) and **`SearchPattern`** (for localized target search).

```
[Target Destination] (from WaypointHandler)
        |
        v
[GeoPlanner::PlanPath()]
        |
        +---> [LiDARHandler Query] (DuckDB spatial lookup within corridor padding)
        |
        +---> [2.5D Costmap Generation] (Elevation, Slope, Roughness, Curvature)
        |
        +---> [Obstacle Dilation Pass] (nDilationPasses, dSafeTravScoreThreshold)
        |
        +---> [Kinematically Constrained Weighted A* Search]
        |
        v
[Path Post-Processing] (SplicePath, Waypoint Tolerance Pruning)
        |
        v
[Ordered Waypoint Path] (std::vector<geoops::Waypoint>)
```

---

## 2. Global Terrain Planning: `GeoPlanner`

The `GeoPlanner` (`src/algorithms/planners/GeoPlanner.cpp`) is a specialized geospatial path planner designed for rough natural environments:

### A. 2.5D Costmap Generation
Rather than assuming a flat 2D plane with binary open/closed cells, `GeoPlanner` constructs a continuous 2.5D costmap using preprocessed USGS LiDAR data from `LiDARHandler` (sourced from the team's [USGS_Data repository](https://gitlab.themrdt.org/MissouriMRDT/USGS_Data)):

- **Terrain Metrics**: Each spatial cell evaluates local surface normal vectors ($N_x, N_y, N_z$), slope gradient, surface roughness, and curvature.
- **Traversal Score**: A composite traversal score ($0.0 = \text{impassable cliff/boulder}$, $1.0 = \text{flat open ground}$) is assigned to each cell. Cells with scores below `dMinTravScore` are marked non-traversable.
- **Obstacle Dilation**: To prevent the rover chassis from clipping edges, non-traversable cells undergo multiple morphological dilation passes (`nDilationPasses`, default 2), expanding obstacles by an inflation margin.

### B. Weighted A* with Kinematic Constraints
- **Search Grid Resolution**: Evaluated on discrete grid tiles (`dGridResolution = 0.5` meters, `dTileSize = 50.0` meters).
- **Corridor Padding**: To keep compute times bounded, DuckDB queries only pull points within a corridor (`dCorridorPadding = 100.0` meters) along the direct line between start and goal.
- **Cost Function**:
  $$f(n) = g(n) + w_h \cdot h(n) + \beta \cdot \text{Cost}_{\text{terrain}}(n)$$
  - $g(n)$: Distance traveled from start.
  - $h(n)$: Euclidean distance heuristic to the goal, scaled by heuristic weight ($w_h = 1.5$) for faster convergence.
  - $\text{Cost}_{\text{terrain}}(n)$: Non-linear penalty term scaled by `dPenaltyScalingFactor` and `dPenaltyPower` to actively penalize rough ground even when traversable.
  - $\beta$ (`dBetaBias`): Tuning weight balancing shortest path distance against terrain smoothness.
- **Kinematic Constraints**: The planner checks turning angle delta between sequential nodes, penalizing sharp turns that exceed the skid-steer chassis's lateral turning dynamics.

### C. Tile Management and Caching
To maintain high runtime performance:

- `GeoPlanner` caches evaluated grid tiles in memory.
- When traversing long distances, distant tiles can be cleared using `UnloadLiDARTiles()` or `ClearGeoCache()`.

> [!TIP] Route Pre-Planning & Inspection
> Mission routes, waypoint sequences, and A* navigation splines can be validated and previewed using the hosted [Autonomy Task Visualizer](https://visualizer.themrdt.org/autonomy-task/). Underlying point cloud terrain tiles and slope hazards can be inspected in 3D using the [LiDAR Tool](https://visualizer.themrdt.org/lidar-tool/), both part of the hosted [MRDT Visualizer Suite](https://visualizer.themrdt.org/).

---

## 3. Localized Search Patterns (`SearchPattern.hpp`)

When the rover reaches the vicinity coordinate of an ArUco post or ground object but does not detect it, the state machine enters `eSearchPattern`. `SearchPattern` mathematically constructs structured search paths:

1. **Two-Phase Archimedean Spiral (`CalculateSpiralPatternWaypoints`)**:
   - **Heading Initialization**: The starting angle is aligned with the rover's current compass heading:
     $$\theta_0 = -\text{Heading}_{\text{degrees}} \times \frac{\pi}{180}$$
   - **Phase 1: Outward Spiral (Expansion)**:
     Generates an expanding Archimedean spiral around origin $(E_0, N_0)$:
     $$r(\theta) = \frac{d_{\text{spacing}}}{2\pi} \cdot (\theta - \theta_0)$$
     $$E(\theta) = E_0 + d_{\text{windup}} \cos \theta, \quad N(\theta) = N_0 + d_{\text{windup}} \sin \theta$$
     Angular step size is governed by `constants::SEARCH_ANGULAR_STEP_DEGREES` (typically $15.0^\circ$), with radial arm separation controlled by `constants::SEARCH_SPIRAL_SPACING` (typically $2.0$ m). Outward generation continues until reaching the designated search radius $R$.
   - **Phase 2: Inward Spiral (Return Sweep)**:
     Upon reaching the outer boundary $R$, the algorithm immediately generates an inward spiral winding back toward the origin until $r \ge 0.5$ m and radial spacing wind-up reaches $0.0$:
     $$d_{\text{windup}} \leftarrow d_{\text{windup}} - d_{\text{spacing}}$$
     This inward sweep provides a continuous second-chance search pass and guides the rover back to the vicinity center without leaving it stranded at the outer perimeter.
   - **Dual-Path Splitting in `SearchPatternState`**:
     After filtering red-zone terrain and passing through `GeoPlanSearchPattern()`, the planned trajectory is split into two halves:
     - **Forward Spiral (`vFirstHalf`)**: Stored in `WaypointHandler` as `"GeoPlannerPath"`, assigned to `PurePursuitController`.
     - **Reverse Return Spiral (`vSecondHalf`)**: Cached in `WaypointHandler` as `"GeoPlannerPathReverse"`.
     If the outward leg completes without acquiring the target, the state machine transitions `m_eCurrentSearchPatternType` to `SearchPatternType::END`, retrieves `"GeoPlannerPathReverse"`, promotes it to `"GeoPlannerPath"`, and navigates back to center.
   - **Completion Safeguard**:
     To prevent false search pattern completion (which can occur if the rover's start position passes within the completion radius of the origin early in the maneuver), `bReachedFinalTarget` is guarded by target index verification:
     $$\text{TargetIndex} > \text{size}(v_{\text{SearchPath}}) - 4$$
     Only when the lookahead tracker has actively traversed through to the final segments of the path is `eSearchFailed` permitted to trigger.
2. **ZigZag / Lawnmower Pattern**:
   - Generates alternating parallel transects spaced by `constants::SEARCH_ZIGZAG_SPACING`.
   - Used in directional terrain features (e.g., canyon floors or ridgelines).
3. **Snake Pattern**:
   - Curved sinusoidal sweep pattern controlled by `constants::SEARCH_SNAKE_SLITHERS`.

---

## 4. Path Splicing and Dynamic Recovery (`StuckState.cpp`)

If the rover encounters an unmapped obstruction or becomes stuck during transit:

- **Obstacle Injection (`DeclareObstacle`)**:
  When `StuckState::Start()` initiates, it computes an obstacle position projected `constants::STUCK_OBSTACLE_DISTANCE` (default 1.0 m) ahead along the rover's current heading:
  $$E_{\text{obs}} = E_{\text{rover}} + d_{\text{obs}} \cos(\theta), \quad N_{\text{obs}} = N_{\text{rover}} + d_{\text{obs}} \sin(\theta)$$
  This obstacle is permanently recorded in `WaypointHandler` with radius `constants::STUCK_OBSTACLE_RADIUS` (default 2.0 m).
- **Recovery Maneuvers**:
  The rover executes staged directional reversals (`eReverseCurrentHeading`, `eReverseLeft`, `eReverseRight`). Once displacement from the stuck origin exceeds `constants::STUCK_SAME_POINT_PROXIMITY` (default 0.5 m), the state machine dispatches `Event::eUnstuck`, invoking `ModifyPath()`.
- **Dynamic Path Splicing (`SplicePath`)**:
  1. **Direct Cache Modification**: Splicing directly modifies `"GeoPlannerPath"` in place (and also splices `"GeoPlannerPathReverse"` if recovering during `SearchPatternState`), eliminating legacy intermediate path keys (`"stuckPath"`, `"unstuckPath"`, `"RevSpiralPath"`).
  2. **Boundary Safeguards**:
     - Verifies `GetObstaclesCount() > 0` before querying obstacle records.
     - Retrieves the most recently added obstacle at index `GetObstaclesCount() - 1`.
     - Strictly preserves the final destination waypoint: `it != std::prev(vPath.end())` prevents goal point excision.
  3. **Node Removal and GeoPlanner Re-route**:
     - Waypoint nodes falling within $(E - E_{\text{obs}})^2 + (N - N_{\text{obs}})^2 \le R_{\text{obs}}^2$ are excised via `vPath.erase()`.
     - When leaving the obstacle zone, `GeoPlanner::PlanPath()` generates a connecting detour between the last valid waypoint before the obstacle and the first valid waypoint beyond it.
     - **Head-Deletion Handling**: If the very first node of the path is within the obstacle radius, `stStartCoordinate` automatically falls back to `stCurrentRoverPose.GetUTMCoordinate()`.
     - Iterator advancement correctly skips over newly inserted detour nodes (`vSplicePathCoordinates.size() - 2`), preventing duplicate processing or iterator invalidation.

---

## 5. Inputs, Outputs, and Known Constraints

### Inputs
- Start UTM Coordinate (`geoops::UTMCoordinate`).
- Goal Waypoint Coordinate (`geoops::Waypoint`).
- USGS LAS DuckDB runtime handler pointer (`LiDARHandler*`).
- Search radius and corridor boundaries.

### Outputs
- An ordered `std::vector<geoops::Waypoint>` representing the sequential navigation points.

### Operational Constraints
- **Search Timeouts**: Global path calculation is bounded by `dMaxSearchTimeSeconds` (default 120.0 seconds). If terrain geometry creates an impenetrable barrier, the planner aborts rather than freezing the application.
- **Grid Resolution vs Compute**: Reducing `dGridResolution` below 0.25 meters dramatically increases open-set node evaluations. A resolution of 0.5 meters provides optimal balance between path fidelity and real-time responsiveness.
