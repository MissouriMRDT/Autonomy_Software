# Vision Utilities

The `src/util/vision/` directory contains several crucial helper classes that bridge the gap between raw math and high-level autonomy logic.

## `YOLOModel.hpp`
This utility abstracts away the incredibly dense PyTorch (`libtorch`) and TensorFlow Lite C++ APIs.
- It allows the autonomy software to load a `.torchscript` or `.tflite` model file with a single line of code.
- It handles transferring image matrices from standard CPU RAM to the GPU memory space.
- It contains the dense tensor-parsing math required to translate YOLOv5 and YOLOv8 output structures into standard `cv::Rect` bounding boxes.

## `BoundingBoxTracking`
Neural network inference is computationally expensive. Running YOLO at 30 FPS on a Jetson Orin will max out the GPU and starve the A* planner.

To fix this, we use the `MultiTracker` class.
- We run the YOLO inference at a lower frequency (e.g., 5-10 Hz).
- When YOLO finds a bounding box, we initialize an OpenCV tracker (e.g., `CSRT` or `KCF`) on that box.
- For the frames *between* YOLO inferences, the tracker analyzes the pixels inside the box and estimates where they shifted to. Trackers use classic computer vision techniques (like correlation filters) and are astronomically faster than running a neural network, allowing the system to maintain a "lock" on an object at 60 FPS while saving battery and thermal headroom.

## `Geolocate`
Covered in the Object Detection section, `GeolocateBox()` is the math powerhouse that converts a 2D `(x, y)` camera pixel into a 3D `(Easting, Northing, Altitude)` global UTM waypoint. It handles the linear algebra matrix rotations required to convert from the ZED's Left-Handed Y-Up coordinate frame to the standard North-West-Up world frame based on the rover's IMU heading.
