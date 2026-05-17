# Object Detection

The `ObjectDetector` class (`src/vision/objects/ObjectDetector.cpp`) is responsible for finding non-tag competition props, such as mallets, water bottles, and rocks.

## The YOLO Pipeline

Unlike AR Tags, which have predictable high-contrast borders, everyday objects require deep learning to identify. We utilize the **YOLO (You Only Look Once)** architecture (v5 or v8).

1. **Preprocessing**: The raw camera frame is resized to match the model's expected input dimensions (e.g., 640x640) and normalized.
2. **Inference**: The `YOLOModel` wrapper passes the image tensor through the PyTorch (CUDA) or TensorFlow Lite (EdgeTPU) model.
3. **Postprocessing**: The raw output tensor contains thousands of overlapping bounding box predictions. We use **NMS (Non-Maximum Suppression)** to filter out boxes that fall below the `OBJECTDETECT_MAINCAM_TORCH_CONFIDENCE` threshold and to merge overlapping boxes that are predicting the same physical object.

## Geolocation

Unlike AR tags, we cannot easily use trigonometry to estimate the distance to a mallet because a mallet's size varies depending on the angle we view it from.

Instead, the `ObjectDetector` utilizes the ZED Camera's depth map.
1. Once a 2D bounding box (pixels) is found in the RGB frame, we find the center pixel `(x, y)`.
2. We pass this pixel to `GeolocateBox()` inside `src/util/vision/Geolocate.hpp`.
3. The function looks up that exact pixel in the ZED's 3D Point Cloud matrix. It samples a 5x5 neighborhood around the pixel to filter out noise, averaging the depth values.
4. Using the rover's current GPS position and heading, it transforms that localized depth point into an absolute global UTM coordinate.
5. The State Machine then treats this physical object exactly like a standard GPS waypoint, calculating an A* path right to it.
