# Camera Feeds & Video Recording

During testing and competitions, it is incredibly valuable to see exactly what the cameras were seeing—especially what the Neural Networks thought they were seeing. To accomplish this without bogging down the main control loops, the autonomy software utilizes the `RecordingHandler`.

## Architecture

The `RecordingHandler` inherits from `AutonomyThread` and runs continuously in the background. It is instantiated inside the `CameraHandler`, `ObjectDetectionHandler`, and `TagDetectionHandler`.

When `StartRecording()` is called, the handler automatically enumerates all the cameras (or detectors) available to its parent handler and creates an OpenCV `cv::VideoWriter` for each one.

## Output Locations

When the autonomy software runs, it creates a new folder in the `logs/` directory named with the current timestamp (e.g., `logs/2023-11-20_14-30-00/`).

Inside this folder, the `RecordingHandler` saves the video files encoded in `H264` (the `.mp4` container).

## Recording Modes

Depending on which handler spawns the `RecordingHandler`, it operates in one of three modes:

1. **`eCameraHandler`**: Records the raw, unadulterated RGB camera feeds from the ZED cameras and basic webcams.
2. **`eTagDetectionHandler`**: Asks the `TagDetector` to generate an "Overlay Frame." This frame has the original image, but with OpenCV drawing ArUco bounding boxes, target crosshairs, and text indicating the estimated yaw and distance to the tag.
3. **`eObjectDetectionHandler`**: Similar to above, but records the YOLO bounding boxes and confidence scores drawn over the objects (mallets, bottles).

## Why is it a separate handler?
Encoding an image into `H264` is an incredibly CPU/GPU intensive process. If the `ObjectDetector` thread had to encode its own output video after running a PyTorch inference, the framerate would plummet.

By having the `RecordingHandler` sit on its own thread, it can asynchronously grab the latest processed frame from the detectors and encode it at its own pace (typically throttled by `RECORDER_FPS` in the constants), ensuring the real-time autonomy logic remains unblocked.
