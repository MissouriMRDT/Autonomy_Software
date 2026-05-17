# Object Detection Handler

The `ObjectDetectionHandler` is functionally similar to the `TagDetectionHandler` but is focused entirely on identifying non-AR objects (like mallets, bottles, rocks, or competition props).

## Primary Responsibilities
1. **Model Management**: Loads the YOLO `.torchscript` or TensorFlow models specified in `AutonomyConstants.cpp` into the GPU/TPU memory.
2. **Detector Aggregation**: Maintains an `ObjectDetector` instance for each configured camera.
3. **Bounding Box Tracking**: Because running a full neural network inference on every frame is too computationally expensive, this handler utilizes OpenCV trackers (e.g., CSRT or KCF) to track the bounding boxes of detected objects between full YOLO inferences.

## Architecture & Threading
- **Background Inference**: Each `ObjectDetector` runs on its own background thread. This is critical because a PyTorch inference might take 30-100ms. If this ran on the main thread, the entire state machine and drive controls would stutter.
- **Asynchronous Output**: The State Machine simply requests the *latest cached detection* from this handler, ensuring the main loop never blocks waiting for an image to process.

## Usage
Used heavily in the `ApproachingObjectState` and `VerifyingObjectState` to steer the rover toward mallets or bottles using visual servoing.
