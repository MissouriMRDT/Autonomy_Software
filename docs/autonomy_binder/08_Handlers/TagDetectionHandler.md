# Tag Detection Handler

The `TagDetectionHandler` manages all instances of AR Tag (Fiducial Marker) detectors running across different camera feeds.

## Primary Responsibilities
1. **Detector Aggregation**: It initializes a `TagDetector` object for each camera (e.g., Main Camera, Rear Camera).
2. **Algorithm Fusion**: It configures each detector to use both traditional OpenCV ArUco marker detection AND custom-trained YOLO tag detection (via PyTorch/TensorFlow).
3. **Debug Overlays**: Provides a method (`RequestDetectionOverlayFrame`) to get an image matrix with bounding boxes and distances drawn over the tags, which is useful for the web UI or debugging.

## Architecture & Threading
- **Multithreading**: Each `TagDetector` spawned by this handler inherits from `AutonomyThread` and runs continuously in the background. It fetches the latest frame from the `CameraHandler`, runs the ArUco and YOLO models, and caches the results.
- **Recording**: Like the `CameraHandler`, this handler can spin up a `RecordingHandler` to explicitly record frames *with* the detection bounding boxes drawn on them, saving them to the `logs/` directory.

## Usage
The `ApproachingMarkerState` constantly queries this handler to get the current pixel location, estimated distance, and yaw angle of the target tag to feed into the drive PID controller.
