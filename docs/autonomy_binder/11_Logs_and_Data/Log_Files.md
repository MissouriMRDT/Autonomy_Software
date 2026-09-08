# Log Files & The Quill Logging Engine

The autonomy software utilizes **Quill**, a low-latency, asynchronous C++ logging library, to manage system telemetry, diagnostics, and debugging outputs across all threads without degrading control-loop execution speed.

---

## 1. Asynchronous Logging Architecture

Traditional synchronous logging libraries format and write log messages directly within the calling thread, introducing unpredictable latency spikes (ranging from microseconds to milliseconds) caused by terminal I/O or filesystem buffer writes. In real-time robotics operating at high frequencies (such as state machine and motor control loops), such delays can cause jitter or missed deadlines.

Quill solves this by decoupling log generation from I/O operations:
1. **Frontend**: When a worker thread invokes a logging macro (e.g., `LOG_INFO`), Quill serializes the arguments into a lock-free, bounded Single-Producer Single-Consumer (SPSC) queue dedicated to that thread. This operation takes mere nanoseconds.
2. **Backend**: A dedicated background logging worker thread (`quill::Backend::start()`) continuously drains these queues, executes the `fmt`-based string formatting, and dispatches the formatted text to the configured sinks.

```
+-------------------------------------------------------------+
| Calling Threads (StateMachine, Perception, Drivers, etc.)   |
|   |                   |                      |              |
|   v                   v                      v              |
| [Thread SPSC]      [Thread SPSC]          [Thread SPSC]     |
+-------------------------------------------------------------+
                            |
                            v (Drained asynchronously)
+-------------------------------------------------------------+
| Quill Backend Thread                                        |
|   - Formats log records using PatternFormatter              |
|   - Filters messages against active log levels              |
|   - Manages backtrace buffers                               |
+-------------------------------------------------------------+
           |                         |
           v                         v
+---------------------+   +---------------------+
|   MRDTConsoleSink   |   | MRDTRotatingFileSink|
|   (Colorized stdout)|   | (.log and .csv)     |
+---------------------+   +---------------------+
```

---

## 2. Configured Sinks and Loggers

In `src/AutonomyLogging.cpp`, initialization creates custom sinks and distinct logger instances:

### Sinks

1. **`MRDTConsoleSink`**:
   - Subclasses `quill::ConsoleSink`.
   - Directs formatted output to standard output (`stdout`).
   - Automatically detects color support (`quill::ConsoleSinkConfig::ColourMode::Automatic`) and applies ANSI color sequences assigned to each log level (`szDebugColor`, `szInfoColor`, `szWarningColor`, `szErrorColor`, `szCriticalColor`).
   - Evaluates `logging::g_eConsoleLogLevel` on every write to selectively output messages that meet or exceed the active console threshold.

2. **`MRDTRotatingFileSink`**:
   - Subclasses `quill::RotatingFileSink`.
   - Configured with open mode `'a'` (append).
   - Generates two synchronized files inside `logs/<YYYY-MM-DD_HH-MM-SS>/`:
     - `console_output.log`: Formatted text log for human reading.
     - `console_output.csv`: Tab-delimited CSV log for automated parsing and analytics.
   - Evaluates `logging::g_eFileLogLevel` on every write to record messages that meet or exceed the active file threshold.

### Logger Instances

- **`logging::g_qFileLogger`**: Routes output to both file sinks (`qLogFileSink` and `qCSVFileSink`).
- **`logging::g_qConsoleLogger`**: Routes output exclusively to `qConsoleSink`.
- **`logging::g_qSharedLogger`**: Routes output simultaneously to `qLogFileSink`, `qCSVFileSink`, and `qConsoleSink`. This is the primary logger used across the autonomy codebase.

---

## 3. Formatting Patterns

Quill uses pattern strings to structure each log line:

| Stream | Pattern String |
| :--- | :--- |
| **Log File (`.log`)** | `%(time) %(log_level) [%(thread_id)] [%(file_name):%(line_number)] %(message)` |
| **CSV File (`.csv`)** | `%(time),\t%(log_level),\t[%(thread_id)],\t[%(file_name):%(line_number)],\t"%(message)"` |
| **Console (`stdout`)** | `%(time) %(log_level:9) [%(thread_id)] [%(file_name):%(line_number)] %(message)` |
| **Timestamp Format** | `%Y-%m-%d %H:%M:%S.%Qms` (Year-Month-Day Hour:Minute:Second.Milliseconds) |

Example log file line:
```text
2026-09-08 14:32:05.124 INFO [140234512] [StateMachineHandler.cpp:88] Rover state changed from Idle to Navigating
```

---

## 4. Logging Levels and Priorities

Quill log levels are ordered by increasing severity. When a log level is set, only messages equal to or higher in priority are emitted:

| Priority | Quill Log Level | Typical Code Usage |
| :---: | :--- | :--- |
| 1 | `quill::LogLevel::TraceL3` | Unused in general operation. |
| 2 | `quill::LogLevel::TraceL2` | Deep memory or byte-level debug tracing. |
| 3 | `quill::LogLevel::TraceL1` | High-frequency raw sensor data (e.g., every individual LiDAR point or pixel coordinate). |
| 4 | `quill::LogLevel::Debug` | Intermediate algorithm values, Stanley control vectors, raw GPS/IMU readings. |
| 5 | `quill::LogLevel::Info` | State machine transitions, waypoint completions, camera initialization confirmations. |
| 6 | `quill::LogLevel::Notice` | Main loop interactive diagnostic dumps (hotkeys `h`, `f`, `p`, `s`, `d`, `t`, `m`). |
| 7 | `quill::LogLevel::Warning` | Non-fatal conditions: tracking bounding box lost, GPS data aging past threshold, out-of-bounds parameters clamped. |
| 8 | `quill::LogLevel::Error` | Recoverable component failure: camera frame retrieval failure, LiDAR DB query failure, obstacle blockages. |
| 9 | `quill::LogLevel::Critical` | Fatal errors causing program shutdown: RoveComm socket binding failure, low battery failsafe trigger. |
| 10 | `quill::LogLevel::Backtrace` | Reserved for backtrace buffer dumps. |

---

## 5. Backtrace Ring Buffer

All three loggers configure a backtrace ring buffer:
```cpp
g_qFileLogger->init_backtrace(10, quill::LogLevel::Critical);
g_qConsoleLogger->init_backtrace(10, quill::LogLevel::Critical);
g_qSharedLogger->init_backtrace(10, quill::LogLevel::Critical);
```

When active, Quill keeps a rolling buffer of the last 10 log messages across all levels (even if their priority was lower than the active logging filter). If an event triggers a `CRITICAL` log, Quill automatically flushes the entire backtrace ring buffer to the sinks. This ensures that the events and states directly preceding a crash are captured in the log without requiring verbose logging during normal execution.

---

## 6. Dynamic Runtime Log Level Adjustment

Operators can adjust the logging severity thresholds on the fly without restarting the autonomy software. The autonomy system registers a RoveComm UDP callback for the `SETLOGGINGLEVELS` command (`manifest::Autonomy::COMMANDS.find("SETLOGGINGLEVELS")->second.DATA_ID`):

```cpp
const std::function<void(const rovecomm::RoveCommPacket<uint8_t>&, const sockaddr_in&)> SetLoggingLevelsCallback =
    [](const rovecomm::RoveCommPacket<uint8_t>& stPacket, const sockaddr_in& stdAddr)
{
    (void) stdAddr;

    const int nMinConsoleLevel = static_cast<int>(constants::CONSOLE_MIN_LEVEL);
    const int nMinFileLevel    = static_cast<int>(constants::FILE_MIN_LEVEL);

    const int nRequestedConsoleLevel = stPacket.vData[0];
    const int nRequestedFileLevel    = stPacket.vData[1];

    bool bConsoleLevelChangePermitted = nRequestedConsoleLevel >= nMinConsoleLevel;
    bool bFileLevelChangePermitted    = nRequestedFileLevel >= nMinFileLevel;

    logging::g_eConsoleLogLevel = bConsoleLevelChangePermitted ? static_cast<quill::LogLevel>(stPacket.vData[0]) : logging::g_eConsoleLogLevel;
    logging::g_eFileLogLevel    = bFileLevelChangePermitted ? static_cast<quill::LogLevel>(stPacket.vData[1]) : logging::g_eFileLogLevel;

    LOG_INFO(logging::g_qSharedLogger, "Incoming SETLOGGINGLEVELS: [Console: {}, File: {}]", stPacket.vData[0], stPacket.vData[1]);
};
```

### Safety Rules:
- The requested console log level cannot be set lower than `constants::CONSOLE_MIN_LEVEL`.
- The requested file log level cannot be set lower than `constants::FILE_MIN_LEVEL`.
- If an invalid level is sent, the previous logging level is retained.

---

## 7. Logging in C++ Source Code

To emit log statements in any module, include `AutonomyLogging.h` and use Quill's format macros:

```cpp
#include "AutonomyLogging.h"

// Info log with variable formatting
LOG_INFO(logging::g_qSharedLogger, "Rover arrived at waypoint {}. Distance to target: {:.2f} m", nWaypointIndex, dDistance);

// Warning log with contextual details
LOG_WARNING(logging::g_qSharedLogger, "NavBoard GPS age exceeds limit: {:.3f} s > {:.3f} s", dAge, constants::NAVBOARD_MAX_GPS_DATA_AGE);

// Error log
LOG_ERROR(logging::g_qSharedLogger, "Camera retrieval timed out on camera index {}", nCamIndex);

// Critical log (will trigger backtrace dump and program abort if fatal)
LOG_CRITICAL(logging::g_qSharedLogger, "Failed to bind RoveComm UDP socket to port {}", nPort);
```
