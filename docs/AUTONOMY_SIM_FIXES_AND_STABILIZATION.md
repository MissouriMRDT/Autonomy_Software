# Autonomy Software Simulator Repair & Stabilization Technical Guide

**Document Version:** 1.0.0  
**Target Repository:** `Autonomy_Software`  
**Target System:** `Autonomy_Software_Sim` (Docker Container / Linux x86_64)  
**Simulation Platform:** Unreal Engine 5.6.1 (`RoveSoSimulator`) via Pixel Streaming (WebRTC)  
**Target Hardware Context:** Modern NVIDIA GPUs (RTX 40 / 50 Series, Blackwell / Ada Lovelace) & Multi-core CPUs  

---

## Table of Contents

1. [Executive Summary & System Architecture](#1-executive-summary--system-architecture)
2. [The Progression of Failures (Chronological Journey)](#2-the-progression-of-failures-chronological-journey)
3. [Deep Dive: Root Cause Analysis & Solutions](#3-deep-dive-root-cause-analysis--solutions)
   - [3.1 WebRTC / FFmpeg NAL Unit & Corrupted Macroblock Spam](#31-webrtc--ffmpeg-nal-unit--corrupted-macroblock-spam)
   - [3.2 WebRTC Signaling Keepalive & Connection Drop](#32-webrtc-signaling-keepalive--connection-drop)
   - [3.3 Terminal Input Blocking & Failure to Enter Idle State](#33-terminal-input-blocking--failure-to-enter-idle-state)
   - [3.4 PyTorch / LibTorch CPU Inference Use-After-Free (SIGSEGV)](#34-pytorch--libtorch-cpu-inference-use-after-free-sigsegv)
   - [3.5 Libx264 VideoWriter Thread Proliferation & Exhaustion](#35-libx264-videowriter-thread-proliferation--exhaustion)
   - [3.6 Asynchronous Stack Buffer Lifetime Race in Detection Checkers](#36-asynchronous-stack-buffer-lifetime-race-in-detection-checkers)
   - [3.7 Navigation & Path Planning Failure on Collocated Waypoints](#37-navigation--path-planning-failure-on-collocated-waypoints)
   - [3.8 Stanley Controller Heading Modulus & Discontinuity](#38-stanley-controller-heading-modulus--discontinuity)
   - [3.9 Premature Marker Verification Abort in VerifyingMarkerState](#39-premature-marker-verification-abort-in-verifyingmarkerstate)
   - [3.10 Vector Mutex Violations & Race Conditions in Tag/Object Detectors](#310-vector-mutex-violations--race-conditions-in-tagobject-detectors)
   - [3.11 ZED Point Cloud Retrieval Abandonment & Asynchronous Memory Race](#311-zed-point-cloud-retrieval-abandonment--asynchronous-memory-race)
   - [3.12 Null Pointer Dereferences in State Machine Checkers](#312-null-pointer-dereferences-in-state-machine-checkers)
   - [3.13 WebRTC Image Scaling Context Mismatch & Libswscale Crash (SIGSEGV)](#313-webrtc-image-scaling-context-mismatch--libswscale-crash-sigsegv)
4. [Comprehensive File-by-File Change Log](#4-comprehensive-file-by-file-change-log)
5. [Diagnostics & Debugging Methodology](#5-diagnostics--debugging-methodology)
6. [Verification & Mission Testing Results](#6-verification--mission-testing-results)
7. [Operational Guide & Future Maintenance](#7-operational-guide--future-maintenance)

---

## 1. Executive Summary & System Architecture

The Missouri S&T Mars Rover Design Team (MRDT) Autonomy Software is an autonomous navigation, computer vision, and state-machine-driven robotics platform. The software connects to either real rover hardware (Jetson Orin, physical ZED 2i stereocameras, LiDAR, CAN bus, RoveComm) or a photorealistic simulation environment powered by Unreal Engine 5 (`RoveSoSimulator`).

```
+-----------------------------------------------------------------------------+
|                                Unreal Engine 5                              |
|                              (RoveSoSimulator)                              |
|                                                                             |
|   +---------------------+   +---------------------+   +-----------------+   |
|   | Head Main RGB Cam   |   | Front ZED RGB+Depth |   | Rear ZED RGB+D  |   |
|   +----------+----------+   +----------+----------+   +--------+--------+   |
+--------------|-------------------------|-----------------------|------------+
               | WebRTC                  | WebRTC                | WebRTC
               | H.264 Streams           | H.264 Streams         | H.264 Streams
               v                         v                       v
+-----------------------------------------------------------------------------+
|                          Autonomy_Software_Sim                              |
|                           (Docker Container)                                |
|                                                                             |
|   [ WebRTC Stream Receivers -> FFmpeg H.264 Decoders -> cv::Mat BGR/Depth ]|
|                                     |                                       |
|               +---------------------+---------------------+                 |
|               |                                           |                 |
|               v                                           v                 |
|       [ TagDetectors ]                            [ ObjectDetectors ]       |
|    (OpenCV ArUco + YOLOv8)                     (YOLOv8 TorchScript Detect)  |
|               |                                           |                 |
|               +---------------------+---------------------+                 |
|                                     |                                       |
|                                     v                                       |
|                      [ Geolocation & Point Cloud ]                          |
|                       (3D Box & Marker Localizer)                           |
|                                     |                                       |
|                                     v                                       |
|                      [ Hierarchical State Machine ]                         |
|   (Idle -> Navigating -> SearchPattern -> ApproachingMarker -> Verifying)   |
|                                     |                                       |
|                                     v                                       |
|                    [ Path Planner & Stanley Controller ]                    |
|                                     |                                       |
|                                     v                                       |
|                [ Drive Commands via RoveComm UDP Packets ]                  |
+-----------------------------------------------------------------------------+
```

When running `Autonomy_Software_Sim` on updated development systems (specifically equipped with NVIDIA 40-series and 50-series Blackwell GPUs, such as the RTX 5080), the simulator was completely unusable:
1. The console was inundated hundreds of times per second with H.264 slice logs and macroblock decoding errors.
2. The terminal became unresponsive, refusing to enter the `IdleState` or accept input from the simulation GUI.
3. Upon receiving inputs or camera frames, the application violently crashed with `SIGSEGV (Address boundary error)` or `SIGABRT (free(): invalid pointer)`.
4. Once driving was restored, the rover would reach the target marker, enter `VerifyingMarkerState`, and either prematurely abort verification (bouncing back to approach) or crash with a segmentation fault.

Through systematic root-cause investigation using Linux core dump analysis (GDB), thread stack tracing, memory inspection, and concurrency auditing, twelve distinct architectural, concurrency, and algorithmic bugs were uncovered and resolved.

---

## 2. The Progression of Failures (Chronological Journey)

### Stage 1: The FFmpeg Log Flood & WebRTC Hang
* **Symptoms:** Upon starting `./Autonomy_Software_Sim`, the terminal printed thousands of lines per second of:
  ```text
  [h264 @ 0x56281cfefd80] nal_unit_type: 1(Coded slice of a non-IDR picture), nal_ref_idc: 3
  [h264 @ 0x55742ec28380] corrupted macroblock 48 32 (total_coeff=-1)
  [h264 @ 0x55742ec28380] error while decoding MB 48 32
  [h264 @ 0x55742ec4fb80] no frame!
  ```
* **Consequence:** Terminal I/O was saturated. The main thread stalled in `read(STDIN_FILENO)` and failed to process RoveComm incoming packets. The state machine never transitioned out of initialization.

### Stage 2: The PyTorch Forward-Pass Segfault (`core.222043`)
* **Symptoms:** After subduing the log flood, clicking "Start Autonomy" or receiving the first video frame triggered an immediate crash:
  ```text
  Thread 14 "Autonomy_Softwa" received signal SIGSEGV, Segmentation fault.
  [Switching to Thread 0x7f43377fe640 (LWP 222071)]
  0x00007f433f4a3e20 in dnnl::impl::cpu::x64::brgemm_convolution_fwd_t...
  ```
* **Consequence:** Total application crash within oneDNN/MKL CPU convolution kernels during YOLO inference.

### Stage 3: The VideoWriter Thread Explosion
* **Symptoms:** Inspecting GDB showed 333 threads running inside the process, hundreds of which were named `libx264` or `ffmpeg`.
* **Consequence:** Thread exhaustion, OS context-switch thrashing, and high latency.

### Stage 4: Glibc Memory Corruption (`free(): invalid pointer`)
* **Symptoms:** Intermittent abort during state transitions:
  ```text
  free(): invalid pointer
  SIGABRT: Aborted
  ```
* **Consequence:** Detection checkers spawned asynchronous requests pointing to stack memory that expired before the background thread pool could copy the data.

### Stage 5: Navigation Stall at Goal
* **Symptoms:** Rover received the navigation waypoint, but the path planner returned an empty path because the rover was already within 1 meter of the coordinate, causing it to abort immediately back to `IdleState`.

### Stage 6: Marker Verification Logic Inversion & Abort
* **Symptoms:** The rover drove to the marker, stopped directly in front of ArUco marker ID 1 (distance 0.25m), and entered `VerifyingMarkerState`. It immediately triggered `eVerifyingFailed` and reversed back into `ApproachingMarkerState`, repeating infinitely.

### Stage 7: Concurrency Data Race & Use-After-Free Crash
* **Symptoms:** During the approach-to-verify bounce, the process crashed with:
  ```text
  SIGSEGV: Address boundary error (fault addr 0x56281...)
  ```
* **Consequence:** `TagDetector` was copying vector data to the state machine while concurrently clearing and updating the vector in another thread with zero mutex synchronization, combined with asynchronous point cloud buffer writes racing against `GeolocateBox`.

---

## 3. Deep Dive: Root Cause Analysis & Solutions

---

### 3.1 WebRTC / FFmpeg NAL Unit & Corrupted Macroblock Spam

#### Root Cause
In `src/vision/cameras/sim/WebRTC.cpp`, `InitializeH264Decoder()` explicitly configured the FFmpeg library logging level to debug:
```cpp
av_log_set_level(AV_LOG_DEBUG);
```
Unreal Engine's Pixel Streaming sends H.264 video slices over WebRTC RTP packets. In H.264, every non-IDR P-frame slice generates a debug message in FFmpeg's decoder (`nal_unit_type: 1`). Because the simulator sends 30–60 frames per second across 3 separate camera streams (Head Main, Front ZED RGB+Depth, Rear ZED RGB+Depth), FFmpeg was printing over 500 lines per second directly to standard output.

Furthermore, before the WebRTC data channel has received an IDR intra-frame (keyframe), P-frames cannot be decoded, leading to `corrupted macroblock` warnings printed on every frame.

#### Solution
1. Changed `av_log_set_level(AV_LOG_DEBUG)` to `av_log_set_level(AV_LOG_ERROR)` in `WebRTC::InitializeH264Decoder()`.
2. Rate-limited `send_packet` warnings in `WebRTC::DecodeH264BytesToCVMat()` to once every 3 seconds:
```cpp
// Rate-limit the warning to once every 3 seconds so terminal is not spammed before keyframe arrives.
static auto tmLastWarn = std::chrono::steady_clock::now();
auto tmNow = std::chrono::steady_clock::now();
if (std::chrono::duration_cast<std::chrono::seconds>(tmNow - tmLastWarn).count() >= 3)
{
    LOG_WARNING(logging::g_qSharedLogger, "WebRTC camera {} FFMPEG send_packet failed. Error: {} {}", m_szStreamerID, nReturnCode, aErrorBuffer);
    tmLastWarn = tmNow;
}
```

---

### 3.2 WebRTC Signaling Keepalive & Connection Drop

#### Root Cause
Unreal Engine's Pixel Streaming Signalling Server periodically transmits WebSocket ping packets (`{"type": "ping"}`) to connected clients. If the client does not answer with a pong message (`{"type": "pong"}`), the signalling server terminates the WebSocket connection after a timeout, causing camera streams to abruptly disconnect and enter an unrecoverable reconnect loop.

#### Solution
In `src/vision/cameras/sim/WebRTC.cpp`, added ping/pong response handling to `ConnectToSignallingServer()`:
```cpp
else if (szType == "ping")
{
    // Reply to signalling server keepalive ping with pong to maintain connection.
    nlohmann::json jsnPong;
    jsnPong["type"] = "pong";
    if (jsnMessage.contains("time"))
    {
        jsnPong["time"] = jsnMessage["time"];
    }
    m_pWebSocket->send(jsnPong.dump());
    LOG_DEBUG(logging::g_qSharedLogger, "WebRTC camera {} Replied to 'ping' with 'pong'.", m_szStreamerID);
}
```
Additionally, added a 2-second timeout to `WebRTC::CloseConnection()` to eliminate shutdown deadlocks.

---

### 3.3 Terminal Input Blocking & Failure to Enter Idle State

#### Root Cause
In `src/main.cpp`, the main execution loop attempted to read keyboard characters from `STDIN_FILENO` using a blocking or non-blocking `read()` without checking if standard input was connected to a real interactive TTY:
```cpp
char chInputChar;
int nBytesRead = read(STDIN_FILENO, &chInputChar, 1);
```
When running inside Docker containers or background terminal processes where STDIN is disconnected or redirected (`/dev/null`), `read()` returns `-1` with `errno = EBADF` or `EIO`. The logging code immediately printed warning messages repeatedly, consuming 100% CPU on Core 0 and preventing state machine ticks.

#### Solution
In `src/main.cpp`:
1. Wrapped STDIN reading with `isatty(STDIN_FILENO)` checks.
2. Verified that `nBytesRead > 0` before examining `chInputChar`.
3. Ignored non-blocking errors (`EAGAIN`, `EWOULDBLOCK`):
```cpp
if (isatty(STDIN_FILENO))
{
    int nBytesRead = read(STDIN_FILENO, &chInputChar, 1);
    if (nBytesRead > 0)
    {
        // Process character command...
    }
}
```

---

### 3.4 PyTorch / LibTorch CPU Inference Use-After-Free (SIGSEGV)

#### Root Cause (Core Dump `core.222043` Deep Dive)
This was the most critical crash in the vision pipeline.

GDB stack trace from the 5.7 GB core dump:
```text
#0  0x00007f433f4a3e20 in dnnl::impl::cpu::x64::brgemm_convolution_fwd_t<...>::ker_trans(...) ()
    from /workspaces/Autonomy_Software/external/libtorch/lib/libtorch_cpu.so
#1  0x00007f433e5c9162 in torch::jit::Module::forward(...)
#2  0x000055742139e8a0 in yolomodel::pytorch::PyTorchInterpreter::Inference(...)
```

Faulting assembly instruction:
```text
vbroadcastss 0x18(%r11), %ymm0
```
Register `%r11` pointed to unmapped virtual memory (`0x7f4314000000`), generating an immediate MMU hardware page fault (`SIGSEGV: Address boundary error`).

**The Mechanism of the Bug:**
In `src/util/vision/YOLOModel.hpp`, `PreprocessImage()` contained:
```cpp
torch::Tensor trTensorImage = torch::from_blob(cvResizedImage.data, 
                                                {1, cvResizedImage.rows, cvResizedImage.cols, 3}, 
                                                torch::kFloat);
trTensorImage = trTensorImage.permute({0, 3, 1, 2});
trTensorImage = trTensorImage.to(trDevice);
return trTensorImage;
```

1. **`torch::from_blob` does NOT allocate memory.** It creates a tensor that aliases the memory buffer of `cvResizedImage.data`.
2. The user was running on an NVIDIA RTX 5080 GPU (Blackwell architecture, compute capability `sm_120`). The precompiled LibTorch binary in the Docker container only had CUDA kernels up to `sm_86` (Ampere). When `trModel.to(at::kCUDA)` was attempted, CUDA threw `CUDA error: no kernel image is available for execution on the device`.
3. The codebase caught the exception and fell back to CPU execution:
   ```cpp
   m_trDevice = torch::kCPU;
   ```
4. When `trDevice` is `torch::kCPU`, calling `.to(trDevice)` on a CPU tensor is an identity operation—it returns the **exact same borrowed-memory tensor** without copying.
5. As soon as `PreprocessImage()` returned, `cv::Mat cvResizedImage` went out of scope on the stack. Its destructor called `fastFree()`, releasing the backing image buffer to the OS/glibc memory allocator.
6. Milliseconds later, `m_trModel.forward({trTensorImage})` executed inside the oneDNN library on CPU. The AVX-512 convolution kernel attempted to read the image data that had already been freed. As soon as glibc reclaimed or unmapped the page, `vbroadcastss` hit an unmapped address and crashed with `SIGSEGV`.

#### Solution
1. In `src/util/vision/YOLOModel.hpp`, added an explicit `.clone()` to `PreprocessImage()` to force deep memory allocation and ownership:
```cpp
// Force an owned copy of memory. from_blob only borrows memory!
// When running on CPU, .to(CPU) is a NO-OP and doesn't copy, so when
// cvResizedImage goes out of scope, the tensor pointed to freed memory.
trTensorImage = trTensorImage.permute({0, 3, 1, 2}).clone();
```
2. Enforced inference thread-safety and disabled gradient tracking on inference:
```cpp
torch::NoGradGuard trNoGrad;
std::lock_guard<std::mutex> lgLock(m_muInferenceMutex);
```

---

### 3.5 Libx264 VideoWriter Thread Proliferation & Exhaustion

#### Root Cause
`RecordingHandler` is responsible for saving camera and detector feeds to disk. In `src/handlers/RecordingHandler.cpp`:
```cpp
void RecordingHandler::UpdateVideoWriters()
{
    for (size_t i = 0; i < m_vVideoWriters.size(); ++i)
    {
        if (!m_vVideoWriters[i].isOpened() && m_vRecordingToggles[i])
        {
            cv::Size cvFrameSize = m_vCameras[i]->GetPropResolution();
            m_vVideoWriters[i].open(szFileName, cv::VideoWriter::fourcc('X', '2', '6', '4'), 30, cvFrameSize);
        }
    }
}
```
During startup, cameras and detectors are not yet connected, so `GetPropResolution()` returns `cv::Size(0, 0)`.
OpenCV's `cv::VideoWriter::open()` with fourcc `X264` attempts to initialize an FFmpeg encoder with a 0x0 resolution, which fails and returns `false`.
However, because `m_vRecordingToggles[i]` remained `true`, on the **very next frame iteration** (at 60 Hz), `!isOpened()` was evaluated again and `open()` was invoked again!
Each call to `cv::VideoWriter::open()` constructs internal FFmpeg encoder contexts that spawn worker threads. Within 5 seconds, over **300 threads** were spawned, exhausting OS thread handles and saturating the CPU.

#### Solution
In `src/handlers/RecordingHandler.cpp`:
1. Added explicit resolution bounds checking before attempting to open writers:
```cpp
if (cvFrameSize.width <= 0 || cvFrameSize.height <= 0)
{
    continue;
}
```
2. If `open()` fails, the recording toggle for that camera is disabled (`m_vRecordingToggles[i] = false`), preventing infinite retry loops.

---

### 3.6 Asynchronous Stack Buffer Lifetime Race in Detection Checkers

#### Root Cause
In `src/util/states/TagDetectionChecker.hpp` and `ObjectDetectionChecker.hpp`:
```cpp
void LoadDetectedTags(...)
{
    std::vector<std::vector<tagdetectutils::ArucoTag>> vDetectedArucoTagBuffers(siNumTagDetectors);
    std::vector<std::future<bool>> vDetectedArucoTagsFuture;

    for (size_t siIdx = 0; siIdx < siNumTagDetectors; ++siIdx)
    {
        vDetectedArucoTagsFuture.emplace_back(
            vTagDetectors[siIdx]->RequestDetectedArucoTags(vDetectedArucoTagBuffers[siIdx])
        );
    }
    // ...
}
```
`vDetectedArucoTagBuffers` is allocated on the stack of `LoadDetectedTags()`.
`RequestDetectedArucoTags()` queues a pointer to `vDetectedArucoTagBuffers[siIdx]` into `TagDetector`'s internal queue.
In earlier code, a timeout was used:
```cpp
if (vDetectedArucoTagsFuture[i].wait_for(std::chrono::milliseconds(100)) == std::future_status::ready)
```
When CPU inference took longer than 100ms, the `wait_for()` timed out. `LoadDetectedTags()` exited, and `vDetectedArucoTagBuffers` was **destroyed on the stack**.
Moments later, the worker thread in `TagDetector::PooledLinearCode()` processed the queue and wrote data to `*stContainer.pData` (now an invalid pointer to destroyed stack memory), causing glibc heap corruption and `free(): invalid pointer (SIGABRT)`.

#### Solution
1. Replaced `wait_for()` timeouts with strict `.get()` synchronization on every spawned future, ensuring `LoadDetectedTags()` can never return while background threads hold pointers to its stack buffers.
2. Added a tracking vector `vSpawnedFuture` so that only valid futures are waited on, avoiding indices misalignments.

---

### 3.7 Navigation & Path Planning Failure on Collocated Waypoints

#### Root Cause
In `src/states/NavigatingState.cpp`:
When a waypoint is submitted that is within 1 meter of the rover's current position, `GeoPlanner::PlanPath()` detects that the start and end UTM coordinates are inside the same grid cell or within collision tolerance, and returns an **empty path** (`m_vPathCoordinates.empty() == true`).

`NavigatingState` handled an empty path with:
```cpp
if (m_vPathCoordinates.empty())
{
    LOG_WARNING(logging::g_qSharedLogger, "NavigatingState: Planned path is empty! Transitioning to Idle State.");
    eNextState = States::eIdle;
}
```
This caused the rover to immediately cancel autonomy and switch to `IdleState` before ever starting the search pattern or detecting tags!

#### Solution
In `src/states/NavigatingState.cpp`:
Before falling back to `IdleState`, check whether the rover is already within `constants::NAVIGATING_REACHED_GOAL_RADIUS` of the goal waypoint. If so, treat it as an immediate arrival:
```cpp
if (m_vPathCoordinates.empty())
{
    geoops::GeoMeasurement stMeasurement = geoops::CalculateGeoMeasurement(stCurrentPose.GetUTMCoordinate(), m_stGoalWaypoint.GetUTMCoordinate());
    if (stMeasurement.dDistanceMeters <= constants::NAVIGATING_REACHED_GOAL_RADIUS)
    {
        LOG_NOTICE(logging::g_qSharedLogger,
                   "NavigatingState: Rover is already at goal waypoint position ({:.2f}m). Transitioning to arrival handler...",
                   stMeasurement.dDistanceMeters);
        if (m_stGoalWaypoint.eType == geoops::WaypointType::eTagWaypoint ||
            m_stGoalWaypoint.eType == geoops::WaypointType::eObjectWaypoint)
        {
            eNextState = States::eSearchPattern;
        }
        else
        {
            globals::g_pWaypointHandler->PopNextWaypoint();
            eNextState = States::eIdle;
        }
    }
}
```

---

### 3.8 Stanley Controller Heading Modulus & Discontinuity

#### Root Cause
In `src/algorithms/controllers/PredictiveStanleyController.cpp`, calculate cross-track error and heading error:
```cpp
double dHeadingError = dPathHeading - dCurrentHeading;
```
When crossing 0° / 360° (e.g. current heading 359° and path heading 2°), `dHeadingError` evaluated to `-357°` instead of `+3°`. The steering controller commanded maximum steering angle, causing aggressive oscillations and steering saturation.

#### Solution
In `PredictiveStanleyController.cpp`:
Applied modular angle difference wrapping using `numops::AngularDifference()`:
```cpp
dHeadingError = numops::AngularDifference(dCurrentHeading, dPathHeading);
```
Ensured all heading angles remain normalized within $[-180^\circ, +180^\circ]$.

---

### 3.9 Premature Marker Verification Abort in VerifyingMarkerState

#### Root Cause
When the rover approached within 2 meters of the marker, it transitioned from `ApproachingMarkerState` to `VerifyingMarkerState`.

In `src/states/VerifyingMarkerState.cpp`:
```cpp
tagdetectutils::ArucoTag stBestArucoTag = statemachine::IdentifyTargetMarker(m_vArucoTags, m_stTargetMarker.nID);
tagdetectutils::ArucoTag stBestTorchTag = statemachine::IdentifyTargetMarker(m_vTorchTags, m_stTargetMarker.nID);

if (stBestArucoTag.nID == -1 && stBestTorchTag.nID == -1)
{
    // Neither detected
}
else
{
    if (stBestArucoTag.nID != -1 && stBestArucoTag.dStraightLineDistance > constants::APPROACH_MARKER_PROXIMITY_THRESHOLD)
    {
        globals::g_pStateMachineHandler->HandleEvent(Event::eVerifyingFailed);
        return;
    }
    else if (stBestTorchTag.dConfidence > 0.0 && stBestTorchTag.dStraightLineDistance > constants::APPROACH_MARKER_PROXIMITY_THRESHOLD)
    {
        globals::g_pStateMachineHandler->HandleEvent(Event::eVerifyingFailed);
        return;
    }
}
```
Notice the `else if` branch!
1. The OpenCV ArUco detector positively detected marker ID 1 at a distance of **0.25 meters** ($\le 2.0$m threshold).
2. Concurrently, the auxiliary Torch detector (YOLO) detected an unlocalized bounding box or a false-positive background detection with a distance of `0.0` or `> 2.0` meters.
3. Because the `else if` evaluated `stBestTorchTag.dStraightLineDistance > 2.0`, it triggered `eVerifyingFailed` **even though the ground-truth ArUco marker was directly in front of the rover!**
4. This triggered a state transition back to `ApproachingMarkerState`, which immediately saw the marker and transitioned back to `VerifyingMarkerState`, looping infinitely.

#### Solution
In `src/states/VerifyingMarkerState.cpp`:
Prioritized OpenCV ArUco detections over auxiliary Torch detections. If an ArUco tag is detected within range, the auxiliary Torch check is bypassed entirely:
```cpp
// Check the tags distance. If ArUco is detected, prioritize it over auxiliary Torch detection.
if (stBestArucoTag.nID != -1)
{
    if (stBestArucoTag.dStraightLineDistance > constants::APPROACH_MARKER_PROXIMITY_THRESHOLD)
    {
        LOG_INFO(logging::g_qSharedLogger,
                 "VerifyingMarkerState: ArUco tag detected but too far away ({:.2f}m > {:.2f}m). Triggering verify failed event.",
                 stBestArucoTag.dStraightLineDistance,
                 constants::APPROACH_MARKER_PROXIMITY_THRESHOLD);
        globals::g_pStateMachineHandler->HandleEvent(Event::eVerifyingFailed);
        return;
    }
}
else if (stBestTorchTag.dConfidence > 0.0)
{
    if (stBestTorchTag.dStraightLineDistance > constants::APPROACH_MARKER_PROXIMITY_THRESHOLD)
    {
        LOG_INFO(logging::g_qSharedLogger,
                 "VerifyingMarkerState: Torch tag detected but too far away ({:.2f}m > {:.2f}m). Triggering verify failed event.",
                 stBestTorchTag.dStraightLineDistance,
                 constants::APPROACH_MARKER_PROXIMITY_THRESHOLD);
        globals::g_pStateMachineHandler->HandleEvent(Event::eVerifyingFailed);
        return;
    }
}
```

---

### 3.10 Vector Mutex Violations & Race Conditions in Tag/Object Detectors

#### Root Cause
In `src/vision/aruco/TagDetector.cpp`:
```cpp
void TagDetector::PooledLinearCode()
{
    std::unique_lock<std::shared_mutex> lkArucoTagQueue(m_muArucoDataCopyMutex);
    if (!m_qDetectedArucoTagCopySchedule.empty())
    {
        containers::DataFetchContainer<std::vector<tagdetectutils::ArucoTag>> stContainer = m_qDetectedArucoTagCopySchedule.front();
        m_qDetectedArucoTagCopySchedule.pop();
        
        lkArucoTagQueue.unlock(); // <--- MUTEX UNLOCKED PREMATURELY!

        *stContainer.pData = m_vDetectedArucoTags; // <--- VECTOR COPY OCCURS WITHOUT LOCK!

        stContainer.pCopiedDataStatus->set_value(true);
    }
}
```
Meanwhile, in `TagDetector::UpdateDetectedTags()` (executing in the continuous detection thread):
```cpp
void TagDetector::UpdateDetectedTags(std::vector<tagdetectutils::ArucoTag>& vNewlyDetectedTags)
{
    // <--- NO MUTEX WAS LOCKED HERE AT ALL!
    m_vDetectedArucoTags.clear();
    for (auto& tag : vNewlyDetectedTags)
    {
        m_vDetectedArucoTags.emplace_back(tag);
    }
}
```
When the rapid state transitions occurred during marker approach and verification, `TagDetectionChecker` called `RequestDetectedArucoTags()` at 30 Hz.
At the exact microsecond that `PooledLinearCode()` was performing a vector copy constructor (`*stContainer.pData = m_vDetectedArucoTags`), `UpdateDetectedTags()` called `clear()` or `emplace_back()`.
`std::vector` reallocation deallocated the underlying memory array. The copy constructor dereferenced the dangling pointer -> `SIGSEGV: Address boundary error`.

The identical bug existed in `src/vision/objects/ObjectDetector.cpp` with `m_vDetectedObjects`.

#### Solution
1. In `TagDetector::PooledLinearCode()`, kept `lkArucoTagQueue` held until *after* `*stContainer.pData = m_vDetectedArucoTags;` completed:
```cpp
// Copy the detected tags to the target location while holding the lock
*stContainer.pData = m_vDetectedArucoTags;

// Release lock.
lkArucoTagQueue.unlock();
```
2. In `TagDetector::UpdateDetectedTags()`, acquired an exclusive lock `std::unique_lock<std::shared_mutex> lkAruco(m_muArucoDataCopyMutex);` before modifying `m_vDetectedArucoTags`.
3. In `TagDetector::ThreadedContinuousCode()`, protected `DrawDetections()` with a shared read lock `std::shared_lock<std::shared_mutex> lkAruco(m_muArucoDataCopyMutex);`.
4. Applied the exact same locking discipline to `ObjectDetector.cpp` for `m_vDetectedObjects`.

---

### 3.11 ZED Point Cloud Retrieval Abandonment & Asynchronous Memory Race

#### Root Cause
In `TagDetector::ThreadedContinuousCode()`:
```cpp
bool bCloudReady = !bRequestingPointCloud;
bool bFrameReady = false;
int nPollAttempts = 0;

while (...)
{
    if (!bCloudReady && fuPointCloudCopyStatus.wait_for(std::chrono::milliseconds(5)) == std::future_status::ready)
        bCloudReady = true;
    if (!bFrameReady && fuRegularFrameCopyStatus.wait_for(std::chrono::milliseconds(5)) == std::future_status::ready)
        bFrameReady = true;
    if (bCloudReady && bFrameReady)
        break;
    if (bFrameReady && nPollAttempts >= 2) // <--- BREAKS AFTER ONLY 10 MS!
        break;
    nPollAttempts++;
}

// ...
if (!bCloudReady || !fuPointCloudCopyStatus.get()) // <--- SHORT CIRCUITS!
    LOG_WARNING(...);
```
1. In the simulator, `SIMZEDCam::CalculatePointCloud()` takes 12–20 ms to compute 3D coordinates for 921,600 pixels using OpenMP.
2. The check `if (bFrameReady && nPollAttempts >= 2)` broke out of the polling loop after only **10 milliseconds**, at which point `bCloudReady` was still `false`.
3. Because `!bCloudReady` was `true`, the C++ logical OR operator `||` **short-circuited**, and `fuPointCloudCopyStatus.get()` was **never called**.
4. The `std::future` went out of scope. But `SIMZEDCam`'s thread pool was still actively copying data into `m_cvPointCloud`!
5. Downstream in `TagDetector`, `UpdateDetectedTags()` saw `if (!m_cvPointCloud.empty())` and called:
   ```cpp
   geoloc::GeolocateBox(m_cvPointCloud, ...);
   ```
   `GeolocateBox` accessed `m_cvPointCloud.at<cv::Vec4f>(y, x)` while the background thread was simultaneously writing to it.
6. In the next frame iteration, `TagDetector` called `RequestPointCloudCopy(m_cvPointCloud)` again, queuing a second concurrent writer to the same memory buffer. This resulted in internal OpenCV reference count corruption and segmentation faults.

#### Solution
1. Increased max polling window from 30ms (`nPollAttempts < 6`) to 100ms (`nPollAttempts < 20`).
2. Removed the premature 10ms early break (`nPollAttempts >= 2`).
3. Explicitly awaited `fuPointCloudCopyStatus` to guarantee that background writers are finished before any consumer code touches `m_cvPointCloud`:
```cpp
bool bPointCloudSuccess = false;
if (bRequestingPointCloud)
{
    if (bCloudReady)
    {
        bPointCloudSuccess = fuPointCloudCopyStatus.get();
    }
    else if (fuPointCloudCopyStatus.valid())
    {
        fuPointCloudCopyStatus.wait();
        bPointCloudSuccess = fuPointCloudCopyStatus.get();
    }

    if (!bPointCloudSuccess)
    {
        LOG_WARNING(logging::g_qSharedLogger, "TagDetector unable to get point cloud from ZEDCam!");
        m_cvPointCloud.release();
    }
}
```
4. If point cloud retrieval fails, `m_cvPointCloud.release()` is called so downstream code safely skips depth processing instead of reading corrupt/stale matrices.
5. Replicated identical logic in `ObjectDetector.cpp`.

---

### 3.12 Null Pointer Dereferences in State Machine Checkers

#### Root Cause
In `TagDetectionChecker.hpp` and `ObjectDetectionChecker.hpp`:
```cpp
for (size_t siIdx = 0; siIdx < siNumTagDetectors; ++siIdx)
{
    if (vTagDetectors[siIdx]->GetIsReady()) // <--- Unchecked pointer dereference!
    {
        // ...
    }
}
```
If any camera in `m_vTagDetectors` or `m_vObjectDetectors` failed initialization or was omitted from the configuration (e.g. simulated rover without rear camera or secondary detector), the vector element contained `nullptr`. Calling `->GetIsReady()` dereferenced a null pointer and crashed with `SIGSEGV`.

#### Solution
Added explicit null checks before invoking methods:
```cpp
if (vTagDetectors[siIdx] != nullptr && vTagDetectors[siIdx]->GetIsReady())
```
and:
```cpp
if (vObjectDetectors[siIdx] != nullptr && vObjectDetectors[siIdx]->GetIsReady())
```

---

### 3.13 WebRTC Image Scaling Context Mismatch & Libswscale Crash (SIGSEGV)

#### Root Cause (Core Dump `49388` Analysis)
During extended navigation in `NavigatingState`, the simulator crashed after 30–60 seconds with:
```text
fish: Job 1, './Autonomy_Software_Sim' terminated by signal SIGSEGV (Address boundary error)
```
Inspection with `systemd-coredump` / `coredumpctl` revealed:
```text
PID: 49388, TID: 49575
Signal: 11 (SEGV) si_code: SEGV_MAPERR
Stack trace of thread 15548:
#0  0x00007f7914b14c13 n/a (/usr/local/lib/libswscale.so.8.3.100 + 0x36c13)
```
Disassembly at offset `0x36c13` inside `libswscale.so`:
```text
0x0000000000036c13: movzbl 0x4(%r10), %eax
```
This fault occurred during YUV420P-to-RGB conversion where `%r10` attempted to read beyond the mapped memory page.

**The Mechanics of the Failure:**
1. **Static SwsContext:** In `WebRTC::DecodeH264BytesToCVMat()`, `m_pSWSContext` was allocated with `sws_getContext()` only on the very first decoded frame (`if (m_pSWSContext == nullptr)`).
2. **Dynamic WebRTC Video Changes:** During active simulation driving, Unreal Engine's Pixel Streaming NVENC encoder dynamically adjusts bitrate, resolution, or macroblock slice structures.
3. When resolution or stride changed (or when corrupted macroblock slices arrived), `m_pSWSContext` was **not updated**. `sws_scale()` attempted to read the old context's dimensions (e.g. 1280x720) from an incoming frame buffer of different dimensions (e.g. 640x360), immediately reading past the allocated memory boundary (`SEGV_MAPERR`).
4. **Packet Pointer Dangling:** `m_pPacket->data` borrowed the pointer from `vH264EncodedBytes`. When `vH264EncodedBytes` was deallocated on callback exit, `m_pPacket` was left pointing to dangling memory.
5. **Missing Frame/Plane Validation:** If a frame had missing planes, corrupt flags (`AV_FRAME_FLAG_CORRUPT`), or linesize $\le 0$, it was passed directly into `sws_scale()`.

#### Solution
1. In `src/vision/cameras/sim/WebRTC.cpp`, replaced one-time `sws_getContext()` with dynamic `sws_getCachedContext()`:
```cpp
m_pSWSContext = sws_getCachedContext(m_pSWSContext,
                                     m_pFrame->width,
                                     m_pFrame->height,
                                     static_cast<AVPixelFormat>(m_pFrame->format),
                                     m_pFrame->width,
                                     m_pFrame->height,
                                     eOutputPixelFormat,
                                     SWS_FAST_BILINEAR,
                                     nullptr,
                                     nullptr,
                                     nullptr);
```
2. Validated all decoded frame properties (`width > 0`, `height > 0`, `format >= 0`, and `!(m_pFrame->flags & AV_FRAME_FLAG_CORRUPT)`).
3. Verified all required image planes and positive strides using `av_pix_fmt_desc_get()` before scaling.
4. Immediately cleared `m_pPacket->data = nullptr` and `m_pPacket->size = 0` after `avcodec_send_packet()`.
5. Passed `m_pFrame->linesize` steps to OpenCV matrix plane constructors in YUV mode.

---

## 4. Comprehensive File-by-File Change Log

| File Path | Description of Changes |
| :--- | :--- |
| `src/vision/cameras/sim/WebRTC.cpp` | • Changed FFmpeg decode log level from `AV_LOG_DEBUG` to `AV_LOG_ERROR` to eliminate console saturation.<br>• Added WebSocket `"ping"`/`"pong"` keepalive handling to maintain connection to Unreal Signalling Server.<br>• Rate-limited H.264 `send_packet` warnings to once every 3 seconds.<br>• Added 2-second timeout to `CloseConnection()` to avoid deadlocks.<br>• **Cached SwsContext dynamically via `sws_getCachedContext` to prevent mid-run `libswscale` memory boundary segfaults.**<br>• **Added frame and plane validation checks (`av_pix_fmt_desc_get`, `AV_FRAME_FLAG_CORRUPT`).**<br>• **Cleared `m_pPacket` data pointers immediately after submission to prevent dangling references.** |
| `src/vision/cameras/sim/WebRTC.h` | • Included `<libavutil/pixdesc.h>` for pixel format descriptor validation. |
| `src/vision/cameras/sim/SIMZEDCam.cpp` | • Initialized `m_cvFrame` with 3 channels (`CV_8UC3`) to match BGR decoding.<br>• Added BGRA conversion in `PooledLinearCode` for consumers requesting 4 channels.<br>• Added empty check on `m_cvDepthImage` before estimating depth measure.<br>• Updated `GetCameraIsOpen()` with null pointer verification. |
| `src/main.cpp` | • Wrapped `STDIN_FILENO` reading with `isatty()` checks to prevent infinite error loops in non-interactive containers.<br>• Ignored non-blocking read codes (`EAGAIN`, `EWOULDBLOCK`). |
| `src/util/vision/YOLOModel.hpp` | • Added `.clone()` to `PreprocessImage()` tensor output, preventing use-after-free on CPU fallback.<br>• Added `torch::NoGradGuard` and `m_muInferenceMutex` around model forward pass. |
| `src/handlers/RecordingHandler.cpp` | • Added frame resolution bounds check (`width > 0 && height > 0`) before opening `cv::VideoWriter`.<br>• Disabled failed recording toggles to prevent 300+ thread explosion. |
| `src/states/NavigatingState.cpp` | • Handled empty path edge case: if rover is already within goal radius, immediately transition to arrival / search pattern instead of aborting to Idle. |
| `src/algorithms/controllers/PredictiveStanleyController.cpp` | • Applied `numops::AngularDifference()` to heading error calculations to eliminate 0°/360° discontinuities. |
| `src/states/VerifyingMarkerState.cpp` | • Prioritized OpenCV ArUco detections over auxiliary Torch detections.<br>• Prevented distant/false-positive Torch detections from triggering premature `eVerifyingFailed` events. |
| `src/vision/aruco/TagDetector.cpp` | • Acquired `m_muArucoDataCopyMutex` across `m_vDetectedArucoTags` vector copies in `PooledLinearCode()`.<br>• Acquired exclusive lock in `UpdateDetectedTags()` and shared lock in `DrawDetections()`.<br>• Increased point cloud polling timeout to 100ms and removed premature 10ms break.<br>• Properly awaited `fuPointCloudCopyStatus` to prevent data race on `m_cvPointCloud`. |
| `src/vision/objects/ObjectDetector.cpp` | • Synchronized `m_vDetectedObjects` vector access with `m_muArucoDataCopyMutex` across all threads.<br>• Fixed point cloud polling timeout and future awaiting to match `TagDetector`. |
| `src/util/states/TagDetectionChecker.hpp` | • Replaced `wait_for` timeouts with `.get()` on valid detector futures to prevent stack memory corruption.<br>• Added `vTagDetectors[siIdx] != nullptr` safety check. |
| `src/util/states/ObjectDetectionChecker.hpp` | • Replaced `wait_for` timeouts with `.get()` on valid detector futures.<br>• Added `vObjectDetectors[siIdx] != nullptr` safety check. |

---

## 5. Diagnostics & Debugging Methodology

### GDB Core Dump Inspection
Linux core dumps were enabled by setting `ulimit -c unlimited`. When the application segfaulted, the kernel generated `core.<PID>`.
The core file was inspected inside the Docker environment using:
```bash
gdb ./build/Autonomy_Software_Sim ./core.222043
```
Key GDB commands used during investigation:
- `bt full`: Show complete backtrace with all local variables and function parameters.
- `info threads`: List all active threads and their execution states.
- `thread apply all bt`: Generate backtraces across all 333 threads (revealed VideoWriter thread leak).
- `disassemble`: Inspect the exact assembly instruction causing the hardware fault (`vbroadcastss 0x18(%r11), %ymm0`).
- `info registers`: Inspect register state (`%r11 = 0x7f4314000000`, outside mapped memory).

### Concurrency Race Auditing
A static analysis review was conducted on all shared member variables:
1. Traced the lifecycle of every `std::future<bool>` returned from `Request*` methods.
2. Verified whether futures alias stack memory or heap memory.
3. Verified that every `std::vector` read and write operation is guarded by a shared/unique mutex lock.

---

## 6. Verification & Mission Testing Results

Following the implementation of all changes, `Autonomy_Software_Sim` was recompiled using CMake unity builds:
```bash
docker exec -w /workspaces/Autonomy_Software 369a60e72a89 cmake --build build -j$(nproc)
```

### Test Scenarios Executed:

1. **Clean Initialization & WebRTC Handshake:**
   - Autonomy software launched without FFmpeg NAL spam.
   - Front ZED, Rear ZED, and Head Main cameras established WebRTC peer connections and completed ICE exchange (`Completed`).
   - WebServer initialized on `http://0.0.0.0:3284`.
   - Rover entered `IdleState` at a steady 3 Hz tick rate (`IdleState: Stopped drive.`).

2. **Simulation Button Input & Autonomy Activation:**
   - Sent "Start Autonomy" command from RoveSoSimulator GUI.
   - State machine transitioned cleanly from `IdleState` to `NavigatingState`.

3. **Autonomous Navigation & Search Pattern:**
   - Rover planned and executed path coordinates using the Predictive Stanley Controller.
   - Upon arriving at waypoint, transitioned into `SearchPatternState` and commenced spiral search.

4. **Marker Approach & Verification:**
   - Head Main Camera OpenCV ArUco detector identified Marker ID 1.
   - Rover transitioned into `ApproachingMarkerState`, steering toward the target while tracking 3D distance.
   - At 0.25 meters, rover transitioned into `VerifyingMarkerState`.
   - ArUco tag was verified continuously for the full 5.0 seconds (`APPROACH_MARKER_VERIFY_TIME`) without premature aborts or segfaults.
   - Rover transitioned to `IdleState` with mission objective successfully achieved.

---

## 7. Operational Guide & Future Maintenance

### Recommended Best Practices for Developers:
1. **Never borrow memory with `torch::from_blob` without `.clone()`** if the underlying buffer is stack-allocated or liable to be mutated/freed before inference completes.
2. **Never allow worker thread pool tasks to write to stack memory** unless the spawning thread is guaranteed to block until the task is complete.
3. **Always guard vector reads and writes** with standard mutexes or `std::shared_mutex` across multi-threaded thread pool implementations.
4. **Always verify hardware architecture compatibility:** Modern GPUs (e.g. RTX 40/50 series) require PyTorch/CUDA wheels built with compatible PTX or SASS architectures; ensure graceful CPU fallback paths maintain data ownership.
5. **Always rate-limit network and decoding warnings** in continuous media streaming pipelines to prevent I/O blocking and terminal deadlocks.
