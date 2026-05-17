# ArUco Tag Detection

During competitions, the rover must autonomously locate and drive toward fiducial markers (AR Tags) scattered across the terrain.

## The `TagDetector`

The `TagDetector` class (`src/vision/aruco/TagDetector.cpp`) runs a continuous loop on an `AutonomyThread`, processing camera frames to find these markers.

### Detection Pipeline

1. **ArUco Library**: The primary method for detection is OpenCV's `cv::aruco`. The algorithm scans the image for high-contrast square shapes and attempts to decode the internal bit-pattern using a predefined dictionary (e.g., `DICT_4X4_50`).
2. **YOLO Fallback (Torch/TFLite)**: At extreme distances or under heavy glare/occlusion, OpenCV's strict geometric requirements fail. To combat this, `TagDetector` optionally layers a custom YOLO neural network over the image. If YOLO finds a bounding box with high confidence that looks like an AR tag, but OpenCV failed to decode it, we can still track the blob and drive toward it until we get close enough for OpenCV to confirm the ID.
3. **Tracking**: If the rover turns quickly, the camera blurs, and both algorithms might drop the detection for a few frames. We use OpenCV CSRT/KCF Trackers (via `BoundingBoxTracking.h`) to predict where the tag moved during those blind spots.

## Pose Estimation

Simply knowing a tag is "in the image" isn't enough; the rover needs to know exactly how far away it is and its angle relative to the chassis.

`TagDetectionUtility.hpp` provides the `EstimatePoseFromCameraFrame()` method.
If we know:
- The physical size of the printed AR tag (`ARUCO_TAG_SIDE_LENGTH`).
- The Field of View (FOV) of the camera lens.
- The pixel resolution of the frame.

We can use basic trigonometry (or OpenCV's `solvePnP` / Camera Intrinsics) to mathematically estimate the straight-line distance to the tag and its yaw angle (heading offset). The `ApproachingMarkerState` uses these outputs directly to feed the `DriveBoard` PID controller.
