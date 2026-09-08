# Vision Utilities

The `src/util/vision/` directory contains specialized mathematical, image-processing, and neural-network helper utilities supporting the computer vision subsystems.

---

## 1. `YOLOModel.hpp`

This utility encapsulates the LibTorch C++ API, providing high-level loading, tensor conversion, and inference execution for YOLO models (`.pt` TorchScript).

### Key Features
- **Device Management**: Automatically selects between CUDA hardware acceleration (`HardwareDevices::eCUDA`) and host CPU execution (`HardwareDevices::eCPU`).
- **Tensor Formatting**: Converts OpenCV image matrices (`cv::Mat`) to normalized floating-point PyTorch tensors with shape $[1, 3, H, W]$, handling color space transformation (BGR to RGB) and memory alignment.
- **Output Parsing**: Translates multi-dimensional output tensors into structured `yolomodel::Detection` objects containing class indices, confidence scores, and `cv::Rect` bounding boxes.
- **Non-Maximum Suppression**: Wraps OpenCV's `cv::dnn::NMSBoxes` to eliminate redundant bounding boxes based on IoU overlap.

---

## 2. `BoundingBoxTracking.h` & `BoundingBoxTracking.cpp`

Neural network inference on high-resolution frames requires significant GPU cycles. To maintain high tracking rates while keeping compute loads manageable, the system employs **OpenCV Multi-Object Tracking**.

### Pipeline
1. When YOLO detects an object, a tracker instance (KCF or CSRT) is initialized on the detected bounding box.
2. On subsequent camera frames, the tracker follows the visual features within the bounding box without executing full neural network inference.
3. Trackers are maintained until:
   - The object leaves the camera field of view.
   - Tracking is lost for longer than `constants::BBOX_TRACKER_LOST_TIMEOUT`.
   - Continuous tracking exceeds `constants::BBOX_TRACKER_MAX_TRACK_TIME`, forcing a neural network re-evaluation.
4. When a new neural network inference completes, overlapping tracker boxes are reconciled using Intersection-over-Union (IoU) matching (`BBOX_TRACKER_IOU_MATCH_THRESHOLD`).

---

## 3. `Geolocate.hpp`

Provides the `geoloc::GeolocateBox()` function, which bridges the 2D optical frame and the 3D UTM global frame:
- **Neighborhood Depth Sampling**: Evaluates an $N \times N$ pixel window around a detected object centroid within the ZED camera's `CV_32FC4` point cloud.
- **20th Percentile Depth Isolation**: Filters background terrain points to measure the distance to the front surface of the object.
- **Monocular Ground Plane Raycast Fallback**: If depth data is missing (due to glare or occlusion), it executes a pinhole geometric raycast using known camera mounting height and pitch angle.
- **UTM Frame Projection**: Rotates the camera-relative vector by the rover's compass heading and adds the camera's current UTM position to produce a `geoops::Waypoint`.

---

## 4. `TagDetectionUtilty.hpp` & `ObjectDetectionUtility.hpp`

- **`TagDetectionUtilty.hpp`**:
  - Provides `EstimatePoseFromCameraFrame()`, which computes straight-line distance and optical yaw angle from tag pixel dimensions, camera resolution, and horizontal field of view.
  - Generates debug visualization overlays with corner outlines, marker IDs, and distance text.
- **`ObjectDetectionUtility.hpp`**:
  - Provides drawing and debug overlay routines for YOLO detections.
  - Implements helper routines for isolating specific target classes (mallet, rock pick, water bottle) from raw multi-class detection vectors.

---

## 5. `ImageOperations.hpp` & `FetchContainers.hpp`

- **`ImageOperations.hpp`**:
  - Provides fast image manipulation routines, including letterboxing (preserving aspect ratio during resizing to 640x640), matrix cropping, colorspace conversions, and CPU-to-GPU matrix transfers (`cv::cuda::GpuMat`).
- **`FetchContainers.hpp`**:
  - Defines thread-safe template wrappers (`containers::FrameFetchContainer<T>`) pairing image matrices with `std::promise<bool>` and `std::future<bool>`.
  - Enables asynchronous frame retrieval pipelines across threads without blocking capture loops.
