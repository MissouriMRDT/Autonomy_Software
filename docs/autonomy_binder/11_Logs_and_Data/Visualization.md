# 3D Interactive Visualization

While 2D plots and video files are great for post-mortem debugging, the autonomy software also features a real-time, interactive 3D visualization engine.

## The `VisualizationHandler`

The `VisualizationHandler` maintains a persistent "World State" by accumulating data from the `LiDARHandler`, the `GeoPlanner`, and the `NavigationBoard`.

It keeps track of:
- The historical trajectory (where the rover has been).
- The current planned path (where the rover intends to go).
- The accumulated obstacle map (point clouds from ZED Spatial Mapping or LiDAR).

## The `SimpleWebServer`

To make this data visible without requiring heavy graphical libraries (like Qt or OpenGL) running natively on the Jetson, the `VisualizationHandler` spins up a `SimpleWebServer`.

- **HTTP Interface**: It hosts a lightweight web server directly on the Jetson (default port 8080).
- **Web UI**: Any device on the network (like a Basestation laptop or a developer's phone) can navigate to `http://<Jetson-IP>:8080` in a standard web browser.
- **Three.js / WebGL**: The server pushes the point cloud data and trajectory coordinates to the browser, where Javascript libraries render it into an interactive 3D scene. The user can pan, zoom, and rotate around the digital twin of the rover's environment in real-time.

## Saving the Environment

When the autonomy software is gracefully shut down (e.g., pressing `Q` in the terminal), the `VisualizationHandler` performs a final dump of the environment.

1. **HTML Export**: It bundles the final state of the 3D web viewer into a standalone `.html` file (`visualization.html`) inside the `logs/` directory. You can double-click this file later to explore the 3D map offline.
2. **PLY Export**: It extracts the spatial map mesh directly from the ZED SDK and saves it as a `.ply` file, which can be opened in professional 3D software like Blender or Meshlab.
