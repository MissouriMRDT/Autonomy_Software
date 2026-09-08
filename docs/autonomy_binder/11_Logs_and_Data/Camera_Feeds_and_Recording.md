# Camera Feeds & Video Recording

During testing operations and competition runs, reviewing raw camera perspectives alongside real-time neural network and detector inferences is essential for diagnostic analysis. Video recording is managed by the `RecordingHandler`, an asynchronous recording subsystem that runs independently of sensor capture and machine vision inference pipelines.

---

## 1. Asynchronous Recording Architecture

H.264 video compression and disk I/O are computationally heavy operations. If detector threads or camera acquisition threads encoded and wrote video frames synchronously, perception loop frequencies would drop significantly.

To isolate the critical path:
1. `RecordingHandler` runs in a dedicated thread derived from `AutonomyThread<void>`.
2. Dedicated instances are spawned by three parent handlers: `CameraHandler`, `TagDetectionHandler`, and `ObjectDetectionHandler`.
3. In each recording loop iteration, the `RecordingHandler` requests frames asynchronously from its parent cameras or detectors using non-blocking futures (`std::future<bool>`), leaving the parent processing pipelines completely unhindered.
4. Frames are fed into OpenCV `cv::VideoWriter` pipelines initialized with four-character code `H264` or `mp4v` targeting `.mp4` container files.

```
+-------------------------+     +-------------------------------+     +---------------------------------+
| CameraHandler           |     | TagDetectionHandler           |     | ObjectDetectionHandler          |
| (Raw Frame Acquisition) |     | (ArUco Detection + Overlays)  |     | (YOLO Model + Overlays)         |
+-------------------------+     +-------------------------------+     +---------------------------------+
             |                                  |                                       |
             v                                  v                                       v
+-------------------------+     +-------------------------------+     +---------------------------------+
| RecordingHandler        |     | RecordingHandler              |     | RecordingHandler                |
| Mode: eCameraHandler    |     | Mode: eTagDetectionHandler    |     | Mode: eObjectDetectionHandler   |
| (Raw Video Streams)     |     | (Tag Overlay Video)           |     | (Object Overlay Video)          |
+-------------------------+     +-------------------------------+     +---------------------------------+
             \                                  |                                      /
              \---------------------------------+-------------------------------------/
                                                |
                                                v
                               +---------------------------------+
                               | Disk Output Directory:          |
                               | logs/<timestamp>/               |
                               | *.mp4 encoded at RECORDER_FPS   |
                               +---------------------------------+
```

---

## 2. Recording Modes and Stream Types

The `RecordingHandler::RecordingMode` enum configures the nature of the frames captured:

### 1. `eCameraHandler`
- **Source**: Directly captures raw RGB frames from initialized cameras (`ZEDCamera` instances or `BasicCam` USB devices).
- **Content**: Clean, unprocessed video without bounding boxes, telemetry overlays, or artificial markers.
- **Use Case**: Post-session sensor calibration, stereo disparity ground-truth evaluation, and photogrammetry reconstruction.

### 2. `eTagDetectionHandler`
- **Source**: Calls `TagDetector::RequestOverlayFrameCopyAsync()`.
- **Content**: The original camera frame augmented with OpenCV visualization overlays:
  - Green/red bounding quad outlines tracing detected ArUco markers.
  - Marker ID numeric tags drawn above the tag center.
  - 3D pose coordinate axes projecting outward from the marker center.
  - Distance (meters) and yaw offset angle (degrees) rendered as diagnostic text.
- **Use Case**: Verifying tag detection range, tracker stability during rover movement, and corner refinement accuracy.

### 3. `eObjectDetectionHandler`
- **Source**: Calls `ObjectDetector::RequestOverlayFrameCopyAsync()`.
- **Content**: The original camera frame augmented with LibTorch YOLO inference outputs:
  - Bounding boxes color-coded by detected object class (Mallet, Water Bottle, Rock Pick).
  - Class name label and confidence percentage score.
  - 20th-percentile geolocated distance and relative bearing angles.
- **Use Case**: Evaluating model false positive rates, tracking continuity, occlusion handling, and non-maximum suppression (NMS) thresholds in outdoor sunlight conditions.

---

## 3. Output Storage Structure

At startup, `AutonomyLogging::InitializeLoggers()` establishes a unified run folder based on the session timestamp:
```
logs/
+-- 2026-09-08_15-30-00/
    |-- console_output.log
    |-- console_output.csv
    |-- visualization.html
    |-- spatial_map.ply
    |-- MainCam_Raw.mp4
    |-- RearCam_Raw.mp4
    |-- MainCam_TagOverlay.mp4
    |-- RearCam_TagOverlay.mp4
    |-- MainCam_ObjectOverlay.mp4
    +-- RearCam_ObjectOverlay.mp4
```

Videos are written at the resolution established by `constants::ZED_MAINCAM_RESOLUTIONX` and `constants::ZED_MAINCAM_RESOLUTIONY` (typically 1280x720) and throttled to `constants::RECORDER_FPS`.

---

## 4. Configuration Constants

Recording behavior is selectively controlled in `src/AutonomyConstants.cpp`:

| Constant Name | Type | Default | Description |
| :--- | :---: | :---: | :--- |
| `RECORDER_FPS` | `int` | `15` | Target framerate for video encoding. Lower values conserve GPU encoder capacity and disk bandwidth. |
| `ZED_MAINCAM_ENABLE_RECORDING` | `bool` | `true` | Toggles raw video recording on the forward ZED camera. |
| `ZED_REARCAM_ENABLE_RECORDING` | `bool` | `true` | Toggles raw video recording on the rear ZED camera. |
| `TAGDETECT_MAINCAM_ENABLE_RECORDING` | `bool` | `true` | Toggles overlay recording on the forward ArUco tag detector. |
| `TAGDETECT_REARCAM_ENABLE_RECORDING` | `bool` | `false` | Toggles overlay recording on the rear ArUco tag detector. |
| `OBJECTDETECT_MAINCAM_ENABLE_RECORDING` | `bool` | `true` | Toggles overlay recording on the forward YOLO object detector. |
| `OBJECTDETECT_REARCAM_ENABLE_RECORDING` | `bool` | `false` | Toggles overlay recording on the rear YOLO object detector. |

---

## 5. Lifecycle Management

1. **Initialization**: Handlers create their internal `RecordingHandler` instances upon construction.
2. **Activation**: In `src/main.cpp`, recording is formally enabled after hardware verification:
   ```cpp
   globals::g_pCameraHandler->StartRecording();
   globals::g_pTagDetectionHandler->StartRecording();
   globals::g_pObjectDetectionHandler->StartRecording();
   ```
3. **Shutdown**: When the main loop exits (via signal or `Q` key), the parent handler stop calls signal the `RecordingHandler` thread to finish writing remaining frames, close the `cv::VideoWriter` streams cleanly, and finalize file containers on disk.
