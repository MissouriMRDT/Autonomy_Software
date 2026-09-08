# Object Detection Handler

The `ObjectDetectionHandler` (`src/handlers/ObjectDetectionHandler.h` & `ObjectDetectionHandler.cpp`) orchestrates all deep-learning prop and obstacle detection pipelines across active camera feeds.

---

## 1. Primary Responsibilities

1. **Model Loading and Management**: Loads custom LibTorch YOLO models (`OBJECTDETECT_TORCH_MODEL`) onto GPU memory via CUDA.
2. **Detector Lifecycle Management**: Instantiates and initializes `ObjectDetector` instances for assigned cameras (`eHeadMainCam`, `eRearCam`).
3. **Bounding Box Tracking Integration**: Coordinates OpenCV CSRT/KCF multi-object trackers between neural network inferences to reduce compute load.
4. **Debug Overlay Streaming**: Generates annotated frames (`RequestDetectionOverlayFrame()`) displaying bounding boxes, class labels, and confidence scores.
5. **Video Recording**: Houses an internal `RecordingHandler` configured in `RecordingType::eObjectDetectionHandler` mode to record annotated detection video.

---

## 2. Managed Detectors

Access to detector instances is provided via the `ObjectDetectors` enumeration:
- **`ObjectDetectors::eHeadMainCam`**: Primary detector analyzing the forward camera stream.
- **`ObjectDetectors::eRearCam`**: Secondary detector analyzing the rear camera stream when enabled.

---

## 3. Concurrency and Integration

- Each `ObjectDetector` executes on its own `AutonomyThread<void>`.
- Inferences run asynchronously at rates up to `constants::OBJECTDETECT_MAINCAM_MAX_FPS`.
- Detected props are cached thread-safely in `objectdetectutils::Object` structs, which `ObjectDetectionChecker` evaluates against requested leg types (Mallet, Water Bottle, Rock Pick).

---

## 4. Usage Example

```cpp
// Initialization in main.cpp
globals::g_pObjectDetectionHandler = new ObjectDetectionHandler();
globals::g_pObjectDetectionHandler->StartAllDetectors();
globals::g_pObjectDetectionHandler->StartRecording();

// Querying detection overlay for UI streaming:
cv::Mat cvAnnotatedFrame = globals::g_pObjectDetectionHandler->RequestDetectionOverlayFrame(
    ObjectDetectionHandler::ObjectDetectors::eHeadMainCam
);
```
