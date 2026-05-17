# Camera Handler

The `CameraHandler` is a centralized manager for all video feeds used by the autonomy software, including ZED stereoscopic cameras and basic USB webcams.

## Primary Responsibilities
1. **Device Initialization**: Automatically detects and initializes the physical cameras (e.g., ZED Front, ZED Rear) or hooks into virtual cameras (if `BUILD_SIM_MODE` is enabled).
2. **Centralized Access**: Acts as a global registry (`globals::g_pCameraHandler`) where other subsystems (like Object Detection or Tag Detection) can request pointers to specific camera streams.
3. **Recording Management**: Internally spawns a `RecordingHandler` thread to save raw RGB or depth frames directly to the disk for later debugging and simulation replay.

## Architecture & Threading
- **Object Aggregation**: It stores `std::shared_ptr<ZEDCamera>` and `std::shared_ptr<BasicCamera>`.
- **`AutonomyThread` Implementation**: Each camera instantiated by this handler runs on its own background thread (inheriting from `AutonomyThread<void>`). The camera continuously polls the physical hardware for new frames as fast as the hardware allows, decoupling the frame acquisition from the slower neural network inferences.
- **Thread Pool Utilization**: To prevent bottlenecks when copying massive image matrices (`cv::Mat`), the cameras utilize a thread pool to dispatch frame copy requests asynchronously.

## Usage
When the program starts, `StartAllCameras()` is called.
Later, if a detector needs an image:
```cpp
std::shared_ptr<ZEDCamera> pMainCam = globals::g_pCameraHandler->GetZED(CameraHandler::ZEDCamName::eHeadMainCam);
cv::Mat cvCurrentFrame = pMainCam->RequestFrameCopy();
```
