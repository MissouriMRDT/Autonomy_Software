# Autonomy TUI Dashboard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement an interactive, modular Terminal User Interface (TUI) inside `Autonomy_Software` using FTXUI, providing a 3-tab dashboard (Telemetry, btop-style Hardware Monitor, and Interactive Log Stream) activated via `--tui`.

**Architecture:** FTXUI is integrated via CMake `FetchContent` as a C++20 dependency. When `--tui` is specified, Quill logging swaps `ConsoleSink` for an in-memory thread-safe circular ring buffer (`TuiLogSink`), and `TuiManager` spawns a dedicated rendering thread (10–15 Hz) that visualizes an atomic snapshot (`TuiTelemetrySnapshot`) without blocking any autonomy threads. On exit, RAII guards restore the primary terminal screen and canonical settings.

**Tech Stack:** C++20, GCC 10, FTXUI v5.0.0, Quill Asynchronous Logger, POSIX Linux / Jetson JetPack.

## Global Constraints
- Target standard: C++20 (`-std=c++20`), GCC 10.
- 100% backward compatibility: Default `./Autonomy_Software` (without `--tui`) must remain untouched, streaming raw Quill logs to `stdout`.
- Zero performance impact on vision or autonomy: TUI rendering must run on its own thread and never block the main loop, camera capture, or network packets.
- Robust terminal safety: RAII guard must restore cursor visibility, reset termios, and exit the alternate screen buffer (`\033[?1049l`) on all exit paths.

---

### Task 1: CMake Dependency Integration for FTXUI

**Files:**
- Modify: `CMakeLists.txt:300-360`
- Test: Build configuration check

**Interfaces:**
- Consumes: External Git repository `https://github.com/ArthurSonzogni/FTXUI.git` tag `v5.0.0`
- Produces: CMake targets `ftxui::screen`, `ftxui::dom`, `ftxui::component` linked into `Autonomy_Software` and test targets

- [x] **Step 1: Add FTXUI FetchContent to CMakeLists.txt**

Add `FetchContent` block in `CMakeLists.txt`:
```cmake
## Fetch FTXUI for Terminal User Interface
include(FetchContent)
FetchContent_Declare(
    ftxui
    GIT_REPOSITORY https://github.com/ArthurSonzogni/FTXUI.git
    GIT_TAG        v5.0.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(ftxui)
```
And link `ftxui::screen`, `ftxui::dom`, `ftxui::component` into `target_link_libraries(${PROJECT_NAME} PRIVATE ...)` and `${EXE_NAME}`.

- [x] **Step 2: Verify CMake configuration**

Run: `cmake -B build -S .`
Expected: CMake downloads FTXUI and configures targets successfully.

- [x] **Step 3: Commit**

```bash
git add CMakeLists.txt
git commit -m "build: integrate FTXUI v5.0.0 via CMake FetchContent"
```

---

### Task 2: Data Model & Custom Quill Log Sink (`TuiLogSink`)

**Files:**
- Create: `src/util/tui/TuiTelemetrySnapshot.h`
- Create: `src/util/tui/TuiLogSink.h`
- Create: `src/util/tui/TuiLogSink.cpp`
- Test: `test/TuiRingBufferTest.cpp`

**Interfaces:**
- Consumes: `quill::Sink`, `quill::LogLevel`
- Produces: `TuiTelemetrySnapshot` struct, `TuiLogBuffer` ring buffer class, and `TuiLogSink` class

- [x] **Step 1: Write the failing unit test for `TuiLogBuffer`**

Create `test/TuiRingBufferTest.cpp`:
```cpp
#include <gtest/gtest.h>
#include "../src/util/tui/TuiLogSink.h"

TEST(TuiRingBufferTest, PushesAndWrapsEntries)
{
    tui::TuiLogBuffer qBuffer(5);
    for (int i = 0; i < 10; ++i)
    {
        qBuffer.Push("12:00:0" + std::to_string(i), quill::LogLevel::Info, "Message " + std::to_string(i));
    }
    EXPECT_EQ(qBuffer.GetTotalCount(), 10);
    auto vSnapshot = qBuffer.GetSnapshot(quill::LogLevel::Info);
    EXPECT_EQ(vSnapshot.size(), 5);
    EXPECT_EQ(vSnapshot.back().szMessage, "Message 9");
}
```

- [x] **Step 2: Run test to verify it fails**

Run: `cmake --build build --target Autonomy_Tests && ./build/Autonomy_Tests --gtest_filter=TuiRingBufferTest.*`
Expected: FAIL compilation with missing header `TuiLogSink.h`.

- [x] **Step 3: Implement `TuiTelemetrySnapshot.h` and `TuiLogSink`**

Write `src/util/tui/TuiTelemetrySnapshot.h` with state machine, pose, drive, detectors, and hardware metric fields.
Write `src/util/tui/TuiLogSink.h` and `src/util/tui/TuiLogSink.cpp` with thread-safe `TuiLogBuffer` and custom `quill::Sink`.

- [x] **Step 4: Run test to verify it passes**

Run: `cmake --build build --target Autonomy_Tests && ./build/Autonomy_Tests --gtest_filter=TuiRingBufferTest.*`
Expected: PASS.

- [x] **Step 5: Commit**

```bash
git add src/util/tui/ test/TuiRingBufferTest.cpp
git commit -m "feat(tui): add TuiTelemetrySnapshot and TuiLogSink ring buffer"
```

---

### Task 3: System Hardware Metrics Collector (`SystemMetrics`)

**Files:**
- Create: `src/util/tui/SystemMetrics.h`
- Create: `src/util/tui/SystemMetrics.cpp`
- Test: `test/SystemMetricsTest.cpp`

**Interfaces:**
- Consumes: `/proc/stat`, `/proc/meminfo`, sysfs thermal zones, NVML / Jetson sysfs
- Produces: `tui::SystemMetrics::Query()` returning CPU per-core %, RAM %, GPU %, and temperatures

- [x] **Step 1: Write failing unit test for `SystemMetrics`**

Create `test/SystemMetricsTest.cpp`:
```cpp
#include <gtest/gtest.h>
#include "../src/util/tui/SystemMetrics.h"

TEST(SystemMetricsTest, QueriesMetricsWithoutThrowing)
{
    tui::SystemMetricsCollector qCollector;
    tui::HardwareStats stStats = qCollector.Query();
    EXPECT_GE(stStats.fCpuTotalUsage, 0.0f);
    EXPECT_LE(stStats.fCpuTotalUsage, 100.0f);
    EXPECT_GE(stStats.fRamUsedGB, 0.0f);
}
```

- [x] **Step 2: Run test to verify it fails**

Run: `cmake --build build --target Autonomy_Tests && ./build/Autonomy_Tests --gtest_filter=SystemMetricsTest.*`
Expected: FAIL with undefined `SystemMetrics.h`.

- [x] **Step 3: Implement `SystemMetrics.h` and `SystemMetrics.cpp`**

Implement robust Linux/Jetson metric parsing with graceful fallback:
- Reads `/proc/stat` delta to compute exact per-core and total CPU percentages.
- Reads `/proc/meminfo` for `MemTotal`, `MemAvailable`, `SwapTotal`, `SwapFree`.
- Reads thermal zones from `/sys/class/thermal/thermal_zone*/temp`.
- Reads Jetson GPU load from `/sys/devices/gpu.0/load` (or fallback to NVML if desktop NVIDIA GPU).
- Never throws exceptions if running in restricted environments.

- [x] **Step 4: Run test to verify it passes**

Run: `cmake --build build --target Autonomy_Tests && ./build/Autonomy_Tests --gtest_filter=SystemMetricsTest.*`
Expected: PASS.

- [x] **Step 5: Commit**

```bash
git add src/util/tui/SystemMetrics.* test/SystemMetricsTest.cpp
git commit -m "feat(tui): implement SystemMetrics hardware collector"
```

---

### Task 4: FTXUI Rendering Engine & 3-Tab Views (`TuiManager`)

**Files:**
- Create: `src/util/tui/TuiManager.h`
- Create: `src/util/tui/TuiManager.cpp`
- Create: `src/util/tui/views/TelemetryView.h`
- Create: `src/util/tui/views/HardwareView.h`
- Create: `src/util/tui/views/LogView.h`

**Interfaces:**
- Consumes: `TuiTelemetrySnapshot`, `TuiLogBuffer`, `ftxui::ScreenInteractive`
- Produces: Full-screen FTXUI TUI lifecycle with Tab 1 (Telemetry Grid), Tab 2 (Hardware), Tab 3 (Logs), terminal RAII guard, and arrow-key event handling

- [x] **Step 1: Implement Views and `TuiManager`**

- `TelemetryView`: 4-pane modular grid rendering Pose, Vision, GeoPlanner, Network, and Status Header.
- `HardwareView`: Per-core horizontal gauges (`C0`–`CN`), RAM bar, GPU load, thermals.
- `LogView`: Scrollable log view with level filters and Spacebar auto-scroll toggle.
- `TuiManager`: Coordinates `ftxui::ScreenInteractive::FitComponent()`, alternate screen buffer escape sequences, mouse tracking, and tab navigation (`←`/`→` or `1`/`2`/`3`).

- [x] **Step 2: Build verification**

Run: `cmake --build build --target Autonomy_Software_Sim`
Expected: Builds cleanly with FTXUI components.

- [x] **Step 3: Commit**

```bash
git add src/util/tui/
git commit -m "feat(tui): implement TuiManager and 3-tab FTXUI rendering engine"
```

---

### Task 5: Main Program Integration & CLI Switch (`--tui`)

**Files:**
- Modify: `src/main.cpp:110-310`
- Modify: `src/AutonomyLogging.h:80-110`
- Modify: `src/AutonomyLogging.cpp:55-150`

**Interfaces:**
- Consumes: `argc`, `argv`, `--tui` CLI flag
- Produces: Optional TUI lifecycle activation without impacting default execution

- [x] **Step 1: Update `AutonomyLogging` to support `TuiLogSink`**

Add `logging::EnableTuiLoggingMode(std::shared_ptr<tui::TuiLogBuffer> pBuffer)` which directs Quill records to the TUI buffer in place of `ConsoleSink`.

- [x] **Step 2: Update `main.cpp`**

1. Parse `--tui` argument:
   ```cpp
   bool bEnableTUI = false;
   for (int i = 1; i < argc; ++i) {
       if (std::string(argv[i]) == "--tui" || std::string(argv[i]) == "-tui") bEnableTUI = true;
   }
   ```
2. If `bEnableTUI`:
   - Initialize `TuiLogBuffer` and pass to `logging::EnableTuiLoggingMode`.
   - Start `TuiManager` on a dedicated worker thread.
   - In main periodic while-loop, populate `TuiTelemetrySnapshot` at 10 Hz from `globals::g_pStateMachineHandler`, `globals::g_pWaypointHandler`, `globals::g_pCameraHandler`, etc.
   - Forward `q` or `SIGINT` to trigger `TuiManager::Stop()`.
3. If not `bEnableTUI`:
   - Execute standard unchanged console loop.

- [x] **Step 3: Build and verify compilation**

Run: `cmake --build build --target Autonomy_Software_Sim`
Expected: Compile and link success.

- [x] **Step 4: Commit**

```bash
git add src/main.cpp src/AutonomyLogging.*
git commit -m "feat(tui): wire --tui CLI flag, Quill sink hook, and telemetry sync in main.cpp"
```

---

### Task 6: End-to-End System Verification

**Files:**
- Test verification script / execution

- [x] **Step 1: Test default mode regression**
  Run `./build/Autonomy_Software_Sim` (without `--tui`).
  Verify output matches existing stdout log stream.

- [x] **Step 2: Test TUI launch & tab switching**
  Run `./build/Autonomy_Software_Sim --tui`.
  Verify alternate screen opens, Tab 1 renders 4-pane grid, Tab 2 renders CPU/GPU bars, Tab 3 renders live logs. Switch tabs with `←`/`→`.

- [x] **Step 3: Test log pause & filtering**
  In Tab 3, press `Space` to pause log stream, scroll with `PgUp`/`PgDn`, press `Space` to resume.

- [x] **Step 4: Test terminal resize**
  Resize window between 80×24 and full screen; verify responsive reflow.

- [x] **Step 5: Test clean exit**
  Press `q`; verify terminal prompt restores cleanly with no corruption.

- [x] **Step 6: Commit and update walkthrough**
```bash
git add docs/
git commit -m "docs: complete verification for Autonomy TUI Dashboard"
```
