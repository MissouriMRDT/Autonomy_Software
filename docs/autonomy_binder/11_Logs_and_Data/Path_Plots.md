# Path Plots & Graphing

To visualize the mathematical logic of the `GeoPlanner` and the A* pathfinding algorithm, the autonomy software utilizes `matplot++`, a C++ graphics library designed to emulate MATLAB's plotting capabilities.

## The `PathTracer` Class

The `PathTracer` (`src/util/logging/PathTracer.hpp`) is a utility class designed to make generating 2D and 3D graphs of the rover's trajectory easy.

### Common Uses
1. **A* Path Verification**: When the `GeoPlanner` calculates a route to a waypoint, it can dump the raw UTM coordinates of the obstacles and the resulting path into a `PathTracer`. The tracer generates a 2D scatter plot showing the starting point, the destination, the inflated obstacle bounds, and the exact line the A* algorithm chose.
2. **GPS Odometry Tracking**: Over a long run, the `PathTracer` can be fed the current GPS/UTM location every few seconds. At the end of the run, it generates a graph showing the actual physical path the rover took, allowing developers to see if the rover oscillated heavily or deviated significantly from the planned path.

## Output Generation

The plots are generated as images (typically `.png` or `.svg`) and saved to the current run's timestamped folder inside the `logs/` directory.

Because generating a high-resolution graph can block the thread for up to a second, these plots are usually only saved:
- When a path is initially calculated (before the rover starts moving).
- When the autonomy software shuts down during the cleanup phase.
