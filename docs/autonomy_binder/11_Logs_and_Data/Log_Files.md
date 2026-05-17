# Log Files & The Quill Logger

The autonomy software uses **Quill**, a blazing-fast, asynchronous, low-latency C++ logging library, to handle all of its text-based logging.

## Log Streams (Sinks)

The logger is configured to output to three different "sinks" simultaneously:

1. **Console Sink**: Prints formatted, color-coded logs directly to the standard output (your terminal/VSCode).
2. **Rotating File Sink**: Writes logs to a `.log` file inside the `logs/<timestamp>` directory. When the file reaches a certain size, it automatically "rotates" to a new file to prevent the disk from filling up endlessly.
3. **RoveComm Sink**: Intercepts log strings and packages them into `ROVECOMM_LOG` UDP packets, broadcasting them over the network. This allows operators using the Basestation GUI to see the autonomy logs in real-time without needing an SSH session.

## Log Levels

Quill supports several severity levels. In `AutonomyConstants.cpp`, we define the minimum level and default level for each of the three sinks independently.

| Level | Purpose |
| :--- | :--- |
| `TRACE` (L1/L2/L3) | Extremely verbose output (e.g., printing the raw X/Y coordinate of a pixel in every frame). Usually disabled. |
| `DEBUG` | Useful state transitions, intermediate PID calculations, or verbose math. |
| `INFO` | Standard operational events ("State changed to Navigating", "ZED Camera Initialized"). |
| `WARNING` | Non-fatal anomalies ("NavBoard data is out of date", "Failed to lock bounding box tracker"). |
| `ERROR` | A subsystem failed or crashed, but the autonomy loop can theoretically recover or fall back. |
| `CRITICAL` | A fatal error occurred (e.g., RoveComm completely failed to bind to a port). The application will likely terminate. |

*Note: During development, the Console level is usually set to `DEBUG`. During a competition, it is highly recommended to set the Console and RoveComm sinks to `INFO` or `WARNING` to reduce network spam.*

## Usage in Code

To log a message anywhere in the codebase, use the Quill macros along with `fmt`-style formatting:

```cpp
// Includes formatting variables into the string automatically
LOG_INFO(logging::g_qSharedLogger, "Rover arrived at waypoint. Distance to goal: {} meters", dDistance);

LOG_ERROR(logging::g_qSharedLogger, "Failed to connect to camera on port {}", nPort);
```
