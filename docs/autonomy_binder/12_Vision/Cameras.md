# Cameras

The Autonomy Software relies on the `CameraHandler` to manage physical and virtual video streams. The vision system supports **Stereolabs ZED Stereoscopic Cameras** and **Basic USB Webcams**, with dedicated simulation interfaces for offline testing.

---

## 1. ZED Stereoscopic Camera Interface (`ZEDCamera.hpp` & `ZEDCam.cpp`)

The primary optical sensors mounted on the rover are Stereolabs ZED 2i cameras (Head Main Camera and optional Rear Camera). The `ZEDCam` class wraps the native Stereolabs C++ SDK (`sl::Camera`).

### Capabilities and Outputs
1. **High-Definition RGB Frames**: Standard color imagery captured at configurable resolutions (`HD720`, `HD1080`) and framerates (typically 30 or 60 FPS).
2. **Dense Depth Maps**: `CV_32FC4` or half-precision floating point matrices where each pixel corresponds to an $(X, Y, Z)$ coordinate relative to the optical center in meters.
3. **Spatial Mapping**: Continuous 3D voxel mesh reconstruction of the terrain (`sl::Mesh`), exported as `.ply` files upon shutdown if enabled.
4. **Visual-Inertial Positional Tracking**: Tracks 6-DoF chassis movement using fused visual odometry and internal IMU measurements.
5. **Sensor Telemetry**: Real-time extraction of linear acceleration, angular velocity, and magnetic heading via `sl::SensorsData`.

### Hardware vs Simulation Architecture
- **Hardware Mode (`ZEDCam.cpp`)**: Communicates with physical ZED 2i cameras via USB 3.0. Supports zero-copy GPU memory sharing via `cv::cuda::GpuMat` to feed CUDA-based PyTorch/YOLO inference without round-tripping to CPU RAM.
- **Simulation Mode (`SIMZEDCam.cpp`)**: When `BUILD_SIM_MODE` is enabled, `CameraHandler` instantiates `SIMZEDCam` instead of `ZEDCam`. It connects to Unreal Engine RoveSoSimulator via WebRTC video tracks (LibDataChannel) and decodes H.264 video streams into RGB and depth matrices, matching physical camera APIs.

---

## 2. Asynchronous Frame Retrieval Architecture

To prevent high-latency operations (such as deep learning inference or GUI streaming) from blocking the high-frequency camera capture loop, all frame retrieval methods are asynchronous and return `std::future<bool>`:

```cpp
// 1. Request an RGB color frame into a local buffer
cv::Mat cvColorFrame;
std::future<bool> fuFrameReady = pMainCam->RequestFrameCopy(cvColorFrame);

// 2. Request a 3D Point Cloud matrix
cv::Mat cvPointCloud;
std::future<bool> fuPointcloudReady = pMainCam->RequestPointCloudCopy(cvPointCloud);

// 3. Request IMU sensor telemetry
sl::SensorsData slSensors;
std::future<bool> fuSensorsReady = pMainCam->RequestSensorsCopy(slSensors);

// 4. Await completion before reading buffers
if (fuFrameReady.get() && fuPointcloudReady.get())
{
    // Process cvColorFrame and cvPointCloud safely
}
```

Behind the scenes:
- `m_qFrameCopySchedule` queues incoming subscriber requests.
- An internal thread pool (`BS::thread_pool`) processes the queue, copying data into destination buffers in parallel.

---

## 3. Basic Camera Interface (`BasicCamera.hpp` & `BasicCam.cpp`)

For non-stereoscopic tasks (such as inspecting ground clearance, verifying robotic arm end-effectors, or streaming auxiliary web feeds), the software uses `BasicCam`:
- Wraps OpenCV's `cv::VideoCapture` for standard V4L2 USB cameras on Linux.
- In simulation mode, `SIMBasicCam` receives virtual feeds via WebRTC channels.
- Employs identical asynchronous `RequestFrameCopy()` semantics, ensuring consistent consumer APIs across all camera types.

---

## 4. Key Configuration Parameters in `AutonomyConstants.cpp`

| Constant Name | Default | Purpose |
| :--- | :--- | :--- |
| `ZED_MAINCAM_RESOLUTIONX` / `Y` | `1280` / `720` | Native camera resolution (`HD720`). |
| `ZED_MAINCAM_FPS` | `30` | Capture framerate target. |
| `ZED_COORD_SYSTEM` | `LEFT_HANDED_Y_UP` | Native coordinate convention (+X Right, +Y Up, +Z Forward). |
| `ZED_DEPTH_MODE` | `ULTRA` / `NEURAL` | Depth reconstruction algorithm. `NEURAL` is higher accuracy; `ULTRA` consumes less GPU power. |
| `ZED_MAINCAM_USE_GPU_MAT` | `false` | When true, buffers are maintained in CUDA memory (`cv::cuda::GpuMat`). |
| `ZED_MAINCAM_FRAME_RETRIEVAL_THREADS` | `5` | Thread pool worker count for servicing parallel frame copy requests. |
| `ZED_MAINCAM_SERIAL` | `0` | Hardware serial number to differentiate head and rear cameras on USB bus. |
