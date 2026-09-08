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
Rather than assuming a flat 2D plane with binary open/closed cells, `GeoPlanner` constructs a continuous 2.5D costmap using preprocessed USGS LiDAR data from `LiDARHandler`:
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

---

## 3. Localized Search Patterns (`SearchPattern.hpp`)

When the rover reaches the vicinity coordinate of an ArUco post or ground object but does not detect it, the state machine enters `eSearchPattern`. `SearchPattern` mathematically constructs structured search paths:

1. **Archimedean Spiral**:
   - Generates an expanding spiral around the origin coordinate $(E_0, N_0)$:
     $$r(\theta) = \frac{d_{\text{spacing}}}{2\pi} \cdot \theta$$
     $$E(\theta) = E_0 + r(\theta) \cos \theta, \quad N(\theta) = N_0 + r(\theta) \sin \theta$$
   - Angular step is controlled by `constants::SEARCH_ANGULAR_STEP_DEGREES`, with spacing set by `constants::SEARCH_SPIRAL_SPACING`.
   - Ensures exhaustive visual coverage of the vicinity radius without leaving blind spots.
2. **ZigZag / Lawnmower Pattern**:
   - Generates alternating parallel transects spaced by `constants::SEARCH_ZIGZAG_SPACING`.
   - Used in directional terrain features (e.g., canyon floors or ridgelines).
3. **Snake Pattern**:
   - Curved sinusoidal sweep pattern controlled by `constants::SEARCH_SNAKE_SLITHERS`.

---

## 4. Path Splicing and Dynamic Recovery

If the rover encounters an unmapped obstruction or becomes stuck during transit:
- `StuckState::DeclareObstacle()` calculates the obstacle coordinate in front of the rover.
- `StuckState::SplicePath()` iterates through the active waypoint vector and excises all intermediate waypoints falling within `constants::STUCK_OBSTACLE_RADIUS` of the declared blockage.
- The path planner then splices a new connecting segment from the rover's current position around the obstacle to the nearest downstream clear node.

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
