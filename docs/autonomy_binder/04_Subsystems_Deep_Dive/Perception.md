# Perception Subsystem

The Perception subsystem acts as the "eyes" of the autonomy software. It processes visual and spatial data to understand the environment, find target objectives, and locate obstacles.

## Algorithm Explanation

The autonomy software uses a combination of deep learning and traditional computer vision algorithms, primarily utilizing the **Stereolabs ZED SDK**, **OpenCV**, and **PyTorch/TensorFlow**.

1. **Object Detection (`ObjectDetector`)**:
   - We use custom-trained **YOLO (You Only Look Once)** models (v5/v8) to identify complex objects like mallets, bottles, or specific competition props.
   - Inference is run via PyTorch or TensorFlow depending on the hardware architecture.
   - Bounding box tracking (using CSRT or KCF trackers via `BoundingBoxTracking.h`) is used to maintain locks on objects between expensive neural network inferences.

2. **AR Tag Detection (`TagDetector`)**:
   - Uses OpenCV's **ArUco** library for fast, traditional detection of fiducial markers.
   - Optionally layers a YOLO PyTorch model on top to identify tags that might be partially occluded or too distant for standard ArUco corner refinement to catch.
   - Uses `TagDetectionUtility` to estimate the 6D pose (distance and yaw) of the tag relative to the camera.

3. **Geolocation (`GeolocateBox`)**:
   - Once a 2D bounding box is found in the image frame, we extract the corresponding 3D data from the ZED camera's depth/point cloud matrix (`CV_32FC4`).
   - The system samples a neighborhood (e.g., 5x5 pixels) around the object center to find the average X, Y, and Z coordinates in the camera's frame.
   - Using the rover's current absolute heading and UTM GPS position, it rotates and translates this vector to calculate the absolute global UTM coordinate of the detected object/tag.

## Inputs and Outputs

- **Inputs**:
  - Raw RGB image frames from the ZED Camera or Basic Webcams.
  - 3D Point Cloud / Depth frames from the ZED SDK.
  - Current Rover Pose (UTM coordinate and compass heading).
- **Outputs**:
  - `geoops::Waypoint` objects containing the exact global locations (UTM Easting/Northing) of detected tags or objects.
  - Distance and Yaw angle metrics to feed into the state machine for approach states.

## Known Limitations

- **Lighting & Glare**: Traditional ArUco detection heavily relies on contrast. Direct sunlight glare on a printed tag or extreme shadows can cause the corner refinement algorithm to fail.
- **Motion Blur**: Fast turns can blur the camera feed, causing frame drops in the YOLO model and ArUco detectors. This is why `MAX_TURN_SPEED` is capped in the drive kinematics.
- **Depth Range**: The ZED camera's depth accuracy degrades exponentially at long distances (typically > 15-20 meters). Geolocated coordinates for distant objects will have a higher margin of error.
