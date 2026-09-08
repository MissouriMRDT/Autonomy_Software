# Recording Handler

The `RecordingHandler` (`src/handlers/RecordingHandler.h` & `RecordingHandler.cpp`) manages the asynchronous recording of camera video feeds and computer vision detection overlays directly to disk.

---

## 1. Primary Responsibilities

1. **Stream Recording**: Connects to active camera and detector streams and encodes frames into video files using OpenCV `cv::VideoWriter`.
2. **Asynchronous Encoding**: Runs on an independent `AutonomyThread<void>` to prevent video encoding overhead from degrading computer vision and control loop framerates.
3. **Multi-Mode Operation**: Supports three distinct operational modes depending on which handler instantiates it.
4. **Selective Recording Toggles**: Reads individual recording enable flags from `AutonomyConstants.cpp` to conserve disk space and CPU resources.

---

## 2. Operational Modes (`RecordingMode`)

```cpp
enum class RecordingMode
{
    eCameraHandler,            // Records raw, unmodified RGB camera feeds
    eTagDetectionHandler,      // Records video feeds with ArUco and YOLO tag detection overlays
    eObjectDetectionHandler    // Records video feeds with YOLO prop detection overlays
};
```

### Modes Explained
- **`eCameraHandler`**: Instantiated inside `CameraHandler`. Directly captures raw frames from `ZEDCamera` and `BasicCamera` streams for ground-truth review and simulation replay.
- **`eTagDetectionHandler`**: Instantiated inside `TagDetectionHandler`. Queries `RequestDetectionOverlayFrame()` from each `TagDetector` to capture video showing detected ArUco marker corners, decoded IDs, and estimated distances.
- **`eObjectDetectionHandler`**: Instantiated inside `ObjectDetectionHandler`. Captures video showing YOLO object bounding boxes, class labels (Mallet, Bottle, Rock Pick), and confidence scores.

---

## 3. Concurrency and Output Formatting

- **Thread Independence**: Inherits from `AutonomyThread<void>`, executing `ThreadedContinuousCode()` at an iteration rate throttled by `constants::RECORDER_FPS` (typically 15 to 30 FPS).
- **Asynchronous Frame Pulling**: Pushes requests to cameras and detectors via futures (`std::future<bool>`), awaiting data transfer in the background without blocking the capture pipeline.
- **File Container and Codec**: Video streams are encoded in H.264 / MP4 format and saved within the timestamped mission directory inside `logs/` (e.g., `logs/YYYY-MM-DD_HH-MM-SS/`).

---

## 4. Configuration Parameters in `AutonomyConstants.cpp`

| Constant Name | Type | Purpose |
| :--- | :--- | :--- |
| `RECORDER_FPS` | `int` | Framerate ceiling for disk video encoding. |
| `ZED_MAINCAM_ENABLE_RECORDING` | `bool` | Enables raw recording of the main mast ZED camera. |
| `ZED_REARCAM_ENABLE_RECORDING` | `bool` | Enables raw recording of the rear ZED camera. |
| `TAGDETECT_MAINCAM_ENABLE_RECORDING` | `bool` | Enables recording of ArUco detection overlays on main camera. |
| `TAGDETECT_REARCAM_ENABLE_RECORDING` | `bool` | Enables recording of ArUco detection overlays on rear camera. |
| `OBJECTDETECT_MAINCAM_ENABLE_RECORDING` | `bool` | Enables recording of YOLO prop detection overlays on main camera. |
| `OBJECTDETECT_REARCAM_ENABLE_RECORDING` | `bool` | Enables recording of YOLO prop detection overlays on rear camera. |
