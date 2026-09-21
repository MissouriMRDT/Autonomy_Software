# Design Specification: Autonomy TUI Dashboard Enhancements

**Date**: 2026-09-21  
**Status**: Proposed  
**Branch**: `feature/autonomy-tui-dashboard`  

---

## 1. Overview & Objectives

This specification outlines the enhancements to the Autonomy Terminal User Interface (TUI) dashboard to provide richer geospatial awareness, complete sensor and vision pipeline diagnostics, live GPU telemetry and thermals via dynamic NVML loading, and a powerful, interactive log viewer with raw console output, mouse navigation, scroll fixes, and real-time search.

---

## 2. Component Architecture & Detailed Design

### 2.1 Telemetry Tab: Dual Coordinates & Sensor Diagnostics

#### Dual UTM & GPS Display
- `TuiTelemetrySnapshot` will be expanded with:
  - `int nUTMZone`: UTM zone integer (e.g., `15`).
  - `bool bUTMNorth`: Hemisphere indicator (`true` for Northern).
  - `double dLatitude`, `double dLongitude`: GPS decimal degrees.
  - `bool bHasGPSFix`: Boolean flag indicating valid satellite fix.
- In `src/main.cpp`, `stPose.GetUTMCoordinate()` provides UTM zone, easting, northing, altitude. `geoops::ConvertUTMToGPS(stPose.GetUTMCoordinate())` calculates latitude and longitude via GeographicLib.
- `TelemetryView.h` will render both formats simultaneously in the *NAV / ROVER POSE* pane:
  ```text
  UTM: Zone 15N | E: 598234.12 m | N: 4198234.56 m | Alt: 312.4 m
  GPS: 37.954231° N, 91.773124° W (Fix: 3D GNSS)
  ```

#### Vision & Sensor Health Status
- `TuiTelemetrySnapshot` will track:
  - `bool bMainCamOpen`, `bool bRearCamOpen`: Queried via `ZEDCam::GetCameraIsOpen()`.
  - `bool bTagDetectorReady`, `bool bObjectDetectorReady`: Queried via `TagDetector::GetIsReady()` and `ObjectDetector::GetIsReady()`.
- `TelemetryView.h` will display clear visual status badges in *VISION & SENSORS*:
  - Front ZED: `[ONLINE / 30 FPS]` (Green) or `[OFFLINE / 0 FPS]` (Red)
  - Rear ZED: `[ONLINE / 30 FPS]` (Green) or `[OFFLINE / 0 FPS]` (Red)
  - Tag Detector (ArUco): `[ACTIVE]` (Green) or `[INITIALIZING]` (Yellow) or `[OFFLINE]` (Red)
  - Object Detector (YOLO): `[ACTIVE]` (Green) or `[INITIALIZING]` (Yellow) or `[OFFLINE]` (Red)

---

### 2.2 Hardware Tab: Dynamic NVML Telemetry & Virtualization Diagnostics

#### NVML Dynamic Loading
- To avoid hard compile-time or link-time dependencies on NVIDIA proprietary drivers (preserving cross-compilation and non-GPU system execution), `SystemMetricsCollector` will dynamically load `libnvidia-ml.so.1` (or `libnvidia-ml.so`) at runtime via `dlopen`/`dlsym`.
- Loaded function pointers:
  - `nvmlInit_v2`, `nvmlShutdown`
  - `nvmlDeviceGetHandleByIndex_v2`
  - `nvmlDeviceGetName`
  - `nvmlDeviceGetTemperature`
  - `nvmlDeviceGetUtilizationRates`
  - `nvmlDeviceGetMemoryInfo`
- Retrieved telemetry:
  - **GPU Model**: e.g., `NVIDIA GeForce RTX 5070 Ti` (replaces hardcoded Jetson string).
  - **GPU Temp**: Real-time degrees Celsius (e.g., `45°C`).
  - **GPU Engine Load**: Real-time compute utilization percentage (e.g., `3%`).
  - **GPU VRAM**: Used vs. Total (e.g., `2.66 / 16.30 GB`).
- **Fallback**: On Jetson Orin / Xavier hardware without NVML, falls back to Tegra sysfs `/sys/devices/gpu.0/load` and `/sys/class/thermal/thermal_zone*`.
- **WSL2 / Docker CPU Temp**: In hypervisor virtual machines where ACPI thermal zones are not passed to Linux guest kernels, CPU and Board temperatures will clearly display `N/A (VM)` instead of blank/missing.

---

### 2.3 Live Logs: Raw Console Output, Mouse Controls, Scroll Fix, and Search Filter

#### Subtabs & Raw Console View
- The Live Logs tab header will feature 6 selectable subtabs:
  `[0: Console / Raw]  [1: All]  [2: Debug]  [3: Info]  [4: Warn]  [5: Error]`
- **Subtab 0: Console / Raw**: Displays the ASCII software banner followed by the exact unparsed log stream (`szFormatted` from `MRDTTuiSink`) matching standard Autonomy console output.
- **Subtabs 1–5**: Filtered structured log view by Quill severity level.

#### Mouse-Clickable Navigation & Wheel Scrolling
- **Mouse Clicks**: Subtab buttons and the search box respond to `Mouse::Left` click events in `CatchEvent`, allowing users to switch subtabs or focus the search box with a single click.
- **Mouse Wheel**:
  - `Mouse::WheelUp`: Automatically pauses auto-scroll and increments `m_nLogScrollOffset`.
  - `Mouse::WheelDown`: Decrements `m_nLogScrollOffset`; resumes auto-scroll once at the bottom (`offset <= 0`).

#### Scroll Fix
- In `TuiManager.cpp`, pressing `↑`, `PageUp`, or scrolling up will automatically disable `m_bLogAutoScroll` and scroll backward into historical buffer entries.
- In `LogView.h`, the historical render slice will correctly clamp between `nStart` and `nEnd` (`nTotal - nScrollOffset`), ensuring view stability without stretching to the live tail.

#### Real-Time Log Search
- Pressing `/` or clicking the search box activates search mode.
- Users can type a search query, which filters log messages in real-time using case-insensitive substring matching across timestamps, logger names, and message bodies.
- Displays match count: `Search: [ query ] (14 matches) | [ESC to Clear]`.

---

## 3. Verification Plan

1. **Compilation & Unity Build**:
   - Verify build with `make -j8 Autonomy_Software_Sim` with `CMAKE_UNITY_BUILD ON`.
2. **Automated Unit Tests**:
   - Run Google Test suite (`Autonomy_Software_Sim_UnitTests`) to ensure buffer, filtering, and metric queries pass.
3. **Interactive Verification**:
   - Launch `./Autonomy_Software_Sim --tui` in devcontainer.
   - Verify Dual Coordinates (UTM Zone 15N + GPS Lat/Lon) in Telemetry tab.
   - Verify Vision & Sensors display online/active status badges.
   - Verify Hardware tab shows GPU model, 45°C GPU temp, GPU utilization, and VRAM.
   - Verify Live Logs:
     - Switching between `Console / Raw` and severity subtabs via mouse click and keys.
     - Smooth scrolling up and down via arrow keys, PageUp/PageDown, and mouse wheel.
     - Search filtering via `/` key and mouse click.
