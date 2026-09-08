# Camera Handler

The `CameraHandler` class (`src/handlers/CameraHandler.h` & `CameraHandler.cpp`) manages camera hardware lifecycle, initializes camera worker threads, provides centralized access to video feeds, and controls asynchronous video recording.

---

## 1. Primary Responsibilities

1. **Hardware Detection and Configuration**: Detects, instantiates, and starts camera objects based on compilation flags (`BUILD_SIM_MODE`) and configuration settings (`MODE_REAR_ZED`).
2. **Global Feed Registry**: Exposes thread-safe getters (`GetZED()`, `GetBasicCam()`) accessible via `globals::g_pCameraHandler` to allow detector handlers to retrieve shared pointers to active camera streams.
3. **Simulation Abstraction**: Automatically instantiates `SIMZEDCam` (WebRTC / LibDataChannel) when `BUILD_SIM_MODE` is enabled, or physical `ZEDCam` (ZED SDK 4.x) when running on physical hardware.
4. **Recording Coordination**: Spawns an internal `RecordingHandler` instance to write raw camera feeds to disk without blocking computer vision processing.

---

## 2. Managed Cameras

The handler manages cameras designated by `ZEDCamName` and `BasicCamName` enumerations:

- **`ZEDCamName::eHeadMainCam`**: The forward-facing ZED 2i stereoscopic camera mounted on the rover mast. Used as the primary feed for ArUco tag detection, YOLO object detection, visual odometry, and 3D geolocation.
- **`ZEDCamName::eRearCam`**: An optional rear-facing ZED stereoscopic camera (enabled when `constants::MODE_REAR_ZED` is true). Used for reversing maneuvers and rear situational awareness.
- **`BasicCamName`**: Extensible interface for standard V4L2 USB cameras or virtual simulation webcams (`BasicCam` / `SIMBasicCam`).

---

## 3. Concurrency and Integration

- Each camera managed by `CameraHandler` inherits from `AutonomyThread<void>`.
- Frame acquisition runs on an independent background thread at the hardware framerate (30 or 60 FPS).
- Downstream modules request data using future-based asynchronous methods (`RequestFrameCopy()`, `RequestDepthCopy()`, `RequestPointCloudCopy()`, `RequestSensorsCopy()`), preventing downstream inference latency from stalling hardware capture.

---

## 4. Usage Example

```cpp
// Startup sequence (in main.cpp)
globals::g_pCameraHandler = new CameraHandler();
globals::g_pCameraHandler->StartAllCameras();
globals::g_pCameraHandler->StartRecording();

// Accessing cameras in downstream modules:
std::shared_ptr<ZEDCamera> pMainCam = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam);

// Asynchronously request color frame and point cloud
cv::Mat cvFrame;
cv::Mat cvPointCloud;
std::future<bool> fuFrame = pMainCam->RequestFrameCopy(cvFrame);
std::future<bool> fuCloud = pMainCam->RequestPointCloudCopy(cvPointCloud);

if (fuFrame.get() && fuCloud.get())
{
    // Execute computer vision and 3D geolocation
}
```
