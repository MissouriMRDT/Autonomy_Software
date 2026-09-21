# Autonomy TUI Dashboard Enhancements Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Enhance the Autonomy TUI dashboard with dual UTM/GPS coordinates, camera and detector status indicators, live GPU metrics via dynamic NVML loading, a raw console output subtab, mouse-clickable subtab switching, wheel-based log scrolling, and real-time log search.

**Architecture:** Extend `TuiTelemetrySnapshot` and `main.cpp` telemetry gathering with `ConvertUTMToGPS` and camera/detector status queries. Implement dynamic `dlopen` loading of NVML in `SystemMetricsCollector` to capture GPU temperatures, model names, and utilization without hard compile-time dependencies. Overhaul `LogView.h` and `TuiManager.cpp` with interactive subtabs, fixed window slicing for historical scrolling, mouse event dispatch, and case-insensitive search filtering.

**Tech Stack:** C++17, FTXUI v5.0.0, GeographicLib (`GeospatialOperations.hpp`), NVML (`libnvidia-ml.so.1`), Quill v10, Google Test, Docker/WSL2 with NVIDIA container toolkit.

## Global Constraints

- Global `CMAKE_UNITY_BUILD ON` must remain enabled for all Autonomy software targets.
- FTXUI targets must remain isolated with `UNITY_BUILD OFF`.
- Standard non-TUI mode (`./Autonomy_Software_Sim`) must remain 100% backward compatible.
- Dynamic NVML loading must never crash or prevent startup on non-NVIDIA or Jetson hardware (graceful fallback).

---

### Task 1: Dual Coordinates & Vision/Sensor Status in Telemetry View

**Files:**
- Modify: `src/util/tui/TuiTelemetrySnapshot.h`
- Modify: `src/main.cpp:350-400`
- Modify: `src/util/tui/views/TelemetryView.h:55-105`

**Interfaces:**
- Consumes: `geoops::ConvertUTMToGPS(geoops::UTMCoordinate)` from `src/util/GeospatialOperations.hpp`, `ZEDCam::GetCameraIsOpen()` from `src/vision/cameras/ZEDCam.h`, `TagDetector::GetIsReady()` from `src/vision/tags/TagDetector.h`, `ObjectDetector::GetIsReady()` from `src/vision/objects/ObjectDetector.h`.
- Produces: `TuiTelemetrySnapshot` with `nUTMZone`, `bUTMNorth`, `dLatitude`, `dLongitude`, `bHasGPSFix`, `bMainCamOpen`, `bRearCamOpen`, `bTagDetectorReady`, `bObjectDetectorReady`.

- [ ] **Step 1: Update `TuiTelemetrySnapshot.h`**
  Add coordinate and health fields to `TuiTelemetrySnapshot`:
  ```cpp
  int nUTMZone = 0;
  bool bUTMNorth = true;
  double dLatitude = 0.0;
  double dLongitude = 0.0;
  bool bHasGPSFix = false;
  bool bMainCamOpen = false;
  bool bRearCamOpen = false;
  bool bTagDetectorReady = false;
  bool bObjectDetectorReady = false;
  ```

- [ ] **Step 2: Update `main.cpp` telemetry gathering**
  In the telemetry update block of `main.cpp`:
  - Query `geoops::UTMCoordinate utm = stPose.GetUTMCoordinate();`
  - Populate `nUTMZone` and `bUTMNorth`.
  - Calculate `geoops::GPSCoordinate gps = geoops::ConvertUTMToGPS(utm);` and populate `dLatitude`, `dLongitude`, `bHasGPSFix`.
  - Check camera openness via `pMainCam->GetCameraIsOpen()` and `pRearCam->GetCameraIsOpen()`.
  - Check detector status via `pTagDetector->GetIsReady()` and `pObjectDetector->GetIsReady()`.

- [ ] **Step 3: Update `TelemetryView.h` to render dual coordinates and sensor badges**
  - Render dual coordinate rows:
    `UTM: Zone 15N | E: 598234.12 m | N: 4198234.56 m | Alt: 312.4 m`
    `GPS: 37.954231° N,  91.773124° W`
  - Render status badges in *VISION & SENSORS*:
    `Front ZED: [ONLINE / 30 FPS]` or `[OFFLINE / 0 FPS]`
    `Rear ZED:  [ONLINE / 30 FPS]` or `[OFFLINE / 0 FPS]`
    `Tag Detector (ArUco):   [ACTIVE]` or `[OFFLINE]`
    `Object Detector (YOLO): [ACTIVE]` or `[OFFLINE]`

- [ ] **Step 4: Build and verify**
  Run `docker exec -w /workspaces/Autonomy_Software/build 7853a94c186e make -j8 Autonomy_Software_Sim` and verify clean compilation.

- [ ] **Step 5: Commit changes**
  `git commit -m "feat(tui): add dual UTM/GPS coordinates and sensor health indicators"`

---

### Task 2: Dynamic NVML Telemetry & Virtualization Diagnostics in Hardware View

**Files:**
- Modify: `src/util/tui/SystemMetrics.h:20-55`
- Modify: `src/util/tui/SystemMetrics.cpp:150-222`
- Modify: `src/util/tui/views/HardwareView.h:25-95`
- Test: `tests/Unit/src/util/TuiTests.cc`

**Interfaces:**
- Consumes: `libnvidia-ml.so.1` (via `dlopen`/`dlsym`).
- Produces: `HardwareStats` with `fGpuTempCelsius`, `fGpuUsagePercent`, `szGpuModel`, `fGpuMemUsedGB`, `fGpuMemTotalGB`, `bIsVirtualMachine`.

- [ ] **Step 1: Write unit test for NVML & Hypervisor query**
  In `tests/Unit/src/util/TuiTests.cc`, add a test validating that `SystemMetricsCollector::Query()` returns valid `HardwareStats` without crashing, handles GPU queries safely, and checks memory/load metrics.

- [ ] **Step 2: Update `SystemMetrics.h` with NVML state & GPU fields**
  Add GPU model, VRAM metrics, and function pointer types for NVML functions (`nvmlInit_v2`, `nvmlDeviceGetHandleByIndex_v2`, `nvmlDeviceGetName`, `nvmlDeviceGetTemperature`, `nvmlDeviceGetUtilizationRates`, `nvmlDeviceGetMemoryInfo`, `nvmlShutdown`).

- [ ] **Step 3: Implement dynamic NVML loading in `SystemMetrics.cpp`**
  - In `UpdateGpu()`, attempt to load `libnvidia-ml.so.1` or `libnvidia-ml.so`.
  - If successful, query device handle 0, retrieve model name, GPU load %, GPU temperature, and VRAM used/total.
  - Fallback to Tegra Jetson sysfs (`/sys/devices/gpu.0/load`) if NVML is unavailable.
  - In `UpdateThermals()`, if CPU temp is 0.0°C and running under virtualization (detect via `/proc/cpuinfo` hypervisor flag or `/sys/class/dmi`), mark `bIsVirtualMachine = true`.

- [ ] **Step 4: Update `HardwareView.h`**
  - Render GPU model name: `Architecture / Model: <szGpuModel>` (e.g. `NVIDIA GeForce RTX 5070 Ti`).
  - Render live GPU temperature in the top banner: `GPU TEMP: 45°C`.
  - Render CPU and Board temp as `N/A (VM)` if virtualized.
  - Render GPU Engine Load gauge and VRAM usage meter in the Accelerator box.

- [ ] **Step 5: Run unit tests and build**
  Run `docker exec -w /workspaces/Autonomy_Software/build 7853a94c186e ctest -R TuiTests --output-on-failure`.

- [ ] **Step 6: Commit changes**
  `git commit -m "feat(tui): integrate dynamic NVML GPU telemetry and VM diagnostics"`

---

### Task 3: Live Logs Console/Raw View, Subtab Navigation & Scroll Fix

**Files:**
- Modify: `src/util/tui/views/LogView.h:40-155`
- Modify: `src/util/tui/TuiManager.h:35-65`
- Modify: `src/util/tui/TuiManager.cpp:200-300`

**Interfaces:**
- Consumes: `TuiLogBuffer::GetSnapshot()` and `TuiLogEntry::szFormatted` from `src/util/tui/TuiLogSink.h`.
- Produces: Clickable subtabs (`0: Console / Raw`, `1: All`, `2: Debug`, `3: Info`, `4: Warn`, `5: Error`), mouse wheel scrolling, and fixed viewport slicing.

- [ ] **Step 1: Overhaul `LogView.h` historical viewport slicing**
  Fix the bug in line 127 of `LogView.h`:
  - Compute `int nEnd = std::clamp(nTotal - nScrollOffset, 0, nTotal);`
  - Compute `int nStart = std::max(0, nEnd - nWindowSize);`
  - Loop strictly from `nStart` to `(bAutoScroll ? nTotal : nEnd)`.

- [ ] **Step 2: Add Console / Raw view mode in `LogView.h`**
  - When subtab index is `0` (`Console / Raw`):
    - Render ASCII banner (`data/ASCII/v25.txt`) at the top of the stream.
    - Render raw unparsed `entry.szFormatted` strings with terminal syntax styling matching standard non-TUI output.
  - When subtab index is `1..5`:
    - Render structured color-coded entries filtered by Quill log level.

- [ ] **Step 3: Implement mouse-clickable subtab badges and wheel scroll**
  - In `LogView.h`, render subtabs with clickable hit targets or FTXUI interactive components.
  - In `TuiManager.cpp` `CatchEvent`:
    - Handle `Event::Special({3})` / `SIGINT`.
    - Handle `event.is_mouse()`:
      - `Mouse::WheelUp`: Set `m_bLogAutoScroll = false`, increment `m_nLogScrollOffset += 3`.
      - `Mouse::WheelDown`: Decrement `m_nLogScrollOffset -= 3`. If `<= 0`, set offset to 0 and resume `m_bLogAutoScroll = true`.
      - `Mouse::Left` click on subtab header row: switch subtab index (`0..5`).
    - Handle `ArrowUp` / `PageUp`: Immediately pause auto-scroll and scroll backward.
    - Handle `ArrowDown` / `PageDown`: Scroll forward; if at bottom, resume auto-scroll.

- [ ] **Step 4: Build and test**
  Run `docker exec -w /workspaces/Autonomy_Software/build 7853a94c186e make -j8 Autonomy_Software_Sim`.

- [ ] **Step 5: Commit changes**
  `git commit -m "feat(tui): add raw console log view, fix scroll logic, and enable mouse navigation"`

---

### Task 4: Interactive Real-Time Log Search Filter

**Files:**
- Modify: `src/util/tui/TuiManager.h:40-70`
- Modify: `src/util/tui/TuiManager.cpp:200-330`
- Modify: `src/util/tui/views/LogView.h:60-155`

**Interfaces:**
- Consumes: `m_szLogSearchQuery` and `m_bSearchInputActive` from `TuiManager`.
- Produces: Case-insensitive search filtering across message content, logger names, and timestamps, with live match counts.

- [ ] **Step 1: Add search state to `TuiManager`**
  - Add `std::string m_szSearchQuery;` and `bool m_bSearchMode = false;`.
  - Provide getters/setters and pass to `RenderLogView`.

- [ ] **Step 2: Add search key and mouse event handling**
  - In `TuiManager.cpp` `CatchEvent`:
    - When user presses `/` (or clicks search box): activate search input mode (`m_bSearchMode = true`).
    - In search mode:
      - Printable characters are appended to `m_szSearchQuery`.
      - `Backspace`: Removes last character.
      - `Escape` or `Enter`: Exits search input mode.
      - `Ctrl+U` or `c`: Clears search query.

- [ ] **Step 3: Filter log entries in `LogView.h`**
  - In `RenderLogView`:
    - If `!szSearchQuery.empty()`, filter `vFiltered` using case-insensitive substring search across `entry.szMessage`, `entry.szLoggerName`, and `entry.szTimestamp`.
    - Render search bar in the control header:
      `SEARCH: [ query ] (N matches) | [/ to Focus | ESC to Clear]`

- [ ] **Step 4: Build and test**
  Run `docker exec -w /workspaces/Autonomy_Software/build 7853a94c186e make -j8 Autonomy_Software_Sim`.

- [ ] **Step 5: Commit changes**
  `git commit -m "feat(tui): add interactive real-time log search filtering"`

---

### Task 5: Comprehensive Automated Testing & End-to-End Verification

**Files:**
- Modify: `tests/Unit/src/util/TuiTests.cc`
- Create: `scripts/test_tui_features.py`

**Interfaces:**
- Validates all new features in unit tests and interactive container PTY execution.

- [ ] **Step 1: Expand Google Test unit test coverage**
  Add unit tests to `tests/Unit/src/util/TuiTests.cc`:
  - `TuiTestsTest.DualCoordinatesConversion`: Test `ConvertUTMToGPS` with UTM Zone 15N coordinates.
  - `TuiTestsTest.DynamicNVMLQuery`: Test `SystemMetricsCollector` returns valid memory, CPU, and handles NVML gracefully.
  - `TuiTestsTest.LogSearchFilter`: Test log filtering with case-insensitive search queries.

- [ ] **Step 2: Run test suite**
  Execute `docker exec -w /workspaces/Autonomy_Software/build 7853a94c186e ./Autonomy_Software_Sim_UnitTests --gtest_filter='TuiTestsTest.*'` and ensure all tests pass.

- [ ] **Step 3: End-to-end interactive verification script**
  Run automated PTY verification script testing:
  - Startup in < 0.05s.
  - Navigation between tabs 1, 2, 3.
  - Subtab switching and search query filtering.
  - Clean shutdown via `q` and `Ctrl+C`.

- [ ] **Step 4: Commit final test updates**
  `git commit -m "test(tui): add comprehensive unit and regression tests for TUI enhancements"`
