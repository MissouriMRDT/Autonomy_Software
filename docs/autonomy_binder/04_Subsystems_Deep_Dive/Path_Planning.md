# Path Planning Subsystem

The Path Planning subsystem determines how the rover navigates from its current location to a desired destination while avoiding physical obstacles.

## Algorithm Explanation

The autonomy software uses a layered approach to path planning, driven primarily by the `GeoPlanner` and `AStar` classes.

1. **Global Planning (A* Algorithm)**:
   - We utilize the **A* (A-Star)** search algorithm (`src/algorithms/planners/AStar.cpp`).
   - The world is mapped as a 2D grid in UTM coordinates.
   - When the `NavigatingState` requests a path to a waypoint, the A* algorithm calculates the shortest path from the rover's current UTM position to the goal UTM position.
   - Known obstacles (received from the `LiDARHandler` or ZED spatial mapping) are inflated by an `AvoidanceRadius` to ensure the rover doesn't clip the edges of rocks or walls.
   - The algorithm evaluates nodes based on the cost to reach them (G-cost) and the estimated distance to the goal (H-cost/Heuristic).

2. **Waypoint Management**:
   - The resulting path is a list of sequential `geoops::Waypoint` objects.
   - The rover navigates to the first waypoint in the list. Once it is within the `NAVIGATING_REACHED_GOAL_RADIUS`, the waypoint is popped off the list, and the rover begins driving toward the next one.

3. **Search Patterns (`SearchPattern.hpp`)**:
   - If the rover reaches the final waypoint but hasn't detected its target, it employs localized search patterns.
   - It mathematically generates a localized series of waypoints forming a Spiral, ZigZag, or Snake pattern around the current location to systematically scan the area.

## Inputs and Outputs

- **Inputs**:
  - Current Rover UTM Coordinate.
  - Goal Waypoint UTM Coordinate.
  - List of obstacle UTM coordinates (from LiDAR/Vision).
- **Outputs**:
  - An ordered `std::vector<geoops::Waypoint>` representing the path to follow.

## Known Limitations

- **Grid Resolution**: The A* algorithm rounds coordinates to `ASTAR_NODE_SIZE`. If this value is too large, the rover may fail to navigate narrow gaps between obstacles. If it's too small, the algorithm becomes computationally expensive and slow.
- **Dynamic Obstacles**: The standard A* implementation calculates the path once. If a moving obstacle steps in front of the rover, the path must be dynamically recalculated, which can cause momentary stuttering.
- **Local Minima**: If the goal is completely surrounded by obstacles (e.g., inside a U-shaped trap), the planner may fail to find a valid path and abort the navigation sequence.
