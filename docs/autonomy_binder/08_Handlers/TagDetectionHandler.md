# Tag Detection Handler

The `TagDetectionHandler` (`src/handlers/TagDetectionHandler.h` & `TagDetectionHandler.cpp`) orchestrates all ArUco marker detection pipelines across active camera feeds.

---

## 1. Primary Responsibilities

1. **Detector Lifecycle Management**: Instantiates and initializes `TagDetector` worker threads for assigned cameras (`eHeadMainCam`, `eRearCam`).
2. **Dual-Model Fusion**: Configures detectors to run classical OpenCV ArUco decoding in parallel with LibTorch YOLO marker candidate detection.
3. **Debug Overlay Streaming**: Generates annotated image frames (`RequestDetectionOverlayFrame()`) containing marker bounding boxes, coordinate axes, IDs, and estimated distances for transmission to the Basestation GUI or WebRTC stream.
4. **Synchronized Video Recording**: Houses an internal `RecordingHandler` configured in `RecordingType::eTagDetectionHandler` mode to save annotated detection feeds to disk.

---

## 2. Managed Detectors

The handler provides access to detectors via the `TagDetectors` enumeration:
- **`TagDetectors::eHeadMainCam`**: Primary detector analyzing frames from the forward mast camera.
- **`TagDetectors::eRearCam`**: Secondary detector monitoring the rear camera feed when enabled.

---

## 3. Concurrency and Integration

- Each `TagDetector` runs as an independent `AutonomyThread<void>`.
- Frame pulling from `CameraHandler` is asynchronous.
- Detections are cached thread-safely in `tagdetectutils::ArucoTag` structs with creation timestamps, enabling `TagDetectionChecker` to evaluate lifetime persistence before triggering state machine transitions.

---

## 4. Usage Example

```cpp
// Initialization in main.cpp
globals::g_pTagDetectionHandler = new TagDetectionHandler();
globals::g_pTagDetectionHandler->StartAllDetectors();
globals::g_pTagDetectionHandler->StartRecording();

// Querying detection overlay for UI streaming:
cv::Mat cvAnnotatedFrame = globals::g_pTagDetectionHandler->RequestDetectionOverlayFrame(
    TagDetectionHandler::TagDetectors::eHeadMainCam
);
```
