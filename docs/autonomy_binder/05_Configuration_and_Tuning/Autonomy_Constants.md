# Autonomy Constants Reference & Tuning Guide

This document serves as the exhaustive engineering reference for configurable constants within the autonomy software. All constants are declared in `src/AutonomyConstants.h` and defined in `src/AutonomyConstants.cpp`. Modifying any value requires recompilation.

---

## 1. General & System Constants

| Constant Name | Type | Typical Value | Engineering Purpose & Tuning Effect |
| :--- | :---: | :---: | :--- |
| `MODE_SIM` | `bool` | `false` | When true, activates simulated camera WebRTC pipelines and simulator network sockets. Set to false for hardware deployment on the physical rover. |
| `SIM_IP_ADDRESS` | `std::string` | `"127.0.0.1"` | IP address for connecting to the Unreal Engine RoveSoSimulator instance. |
| `SIM_WEBSOCKET_PORT` | `uint` | `8080` | WebRTC signaling port for simulator pixel streaming. |
| `SIM_WEBRTC_QP` | `uint` | `20` | Quantization parameter for WebRTC video decompression. Lower values yield higher image fidelity. |
| `BATTERY_MINIMUM_CELL_VOLTAGE` | `double` | `3.2` | Minimum allowable LiPo cell voltage (V). If battery voltage falls below this threshold and checks are enabled, the state machine transitions to `IdleState`. |
| `BATTERY_CHECKS_ENABLED` | `bool` | `true` | Enables or disables battery monitoring failsafes. Set to false in lab environments lacking PMS telemetry. |
| `LOGGING_OUTPUT_PATH_ABSOLUTE` | `std::string` | `"../logs/"` | Base directory on the filesystem where session log folders and recordings are written. |
| `CONSOLE_MIN_LEVEL` | `quill::LogLevel` | `Debug` | Absolute minimum permissible log level for the console sink. Restricts `SETLOGGINGLEVELS` changes from muting vital diagnostics. |
| `FILE_MIN_LEVEL` | `quill::LogLevel` | `Debug` | Absolute minimum permissible log level for file sinks (`.log` and `.csv`). |
| `CONSOLE_DEFAULT_LEVEL` | `quill::LogLevel` | `Notice` | Initial console verbosity at program launch. Recommended `Notice` or `Info` for competition to prevent terminal saturation. |
| `FILE_DEFAULT_LEVEL` | `quill::LogLevel` | `Debug` | Initial file verbosity at launch. Captures full diagnostic details to disk. |
| `ROVECOMM_OUTGOING_UDP_PORT` | `int` | `11000` | Target UDP port for outgoing telemetry packets dispatched across the rover network. |
| `ROVECOMM_OUTGOING_TCP_PORT` | `int` | `11000` | Target TCP port for reliable packet transmission. |
| `ROVECOMM_TCP_INTERFACE_IP` | `std::string` | `"0.0.0.0"` | Network interface IP bound by the local RoveComm TCP listener socket. |

---

## 2. Drive & Kinematics Constants

| Constant Name | Type | Typical Value | Engineering Purpose & Tuning Effect |
| :--- | :---: | :---: | :--- |
| `DRIVE_MAX_POWER` | `float` | `1.0` | Absolute software ceiling for motor effort scalar. |
| `DRIVE_MIN_POWER` | `float` | `-1.0` | Absolute software floor for motor effort scalar. |
| `DRIVE_MAX_SAFE_POWER` | `float` | `0.7` | Global safety clamp applied across all autonomous states to limit peak speeds during testing. |
| `DRIVE_PID_PROPORTIONAL` | `double` | `0.008` | Proportional gain $K_p$ for closed-loop heading correction. Increases responsiveness to angular heading errors. |
| `DRIVE_PID_INTEGRAL` | `double` | `0.0001` | Integral gain $K_i$ for steady-state heading error accumulation to overcome surface scrubbing friction. |
| `DRIVE_PID_DERIVATIVE` | `double` | `0.001` | Derivative gain $K_d$ to dampen angular velocity and mitigate overshoot when approaching the setpoint heading. |
| `DRIVE_PID_FEEDFORWARD` | `double` | `0.0` | Feedforward gain $K_{ff}$ for heading control. |
| `DRIVE_PID_MAX_ERROR` | `double` | `180.0` | Maximum angular error (degrees) fed into the PID controller calculation. |
| `DRIVE_PID_MAX_INTEGRAL_TERM` | `double` | `0.2` | Anti-windup clamping threshold on the accumulated integral term. |
| `DRIVE_PID_MAX_RAMP_RATE` | `double` | `0.05` | Slew rate limiter restricting maximum change in PID output per second to prevent aggressive motor current spikes. |
| `DRIVE_PID_OUTPUT_FILTER` | `double` | `0.1` | Low-pass filter smoothing coefficient applied to the controller output. |
| `DRIVE_PID_TOLERANCE` | `double` | `1.5` | Heading error tolerance band (degrees) within which heading error is treated as zero. |
| `DRIVE_PID_OUTPUT_REVERSED` | `bool` | `false` | Reverses polarity of PID controller output if motor cabling is inverted. |
| `DRIVE_SQUARE_CONTROL_INPUTS` | `bool` | `false` | Applies parabolic scaling ($x \cdot |x|$) to throttle commands to enhance fine control at low velocities. |
| `DRIVE_CURVATURE_KINEMATICS_ALLOW_TURN_WHILE_STOPPED` | `bool` | `true` | Allows zero-radius point turns when forward throttle is zero. |

---

## 3. Inclinometer Damping Multipliers

Dynamically down-scales motor throttle as terrain slope steepens to prevent high-speed rollover incidents:

| Constant Name | Type | Typical Value | Engineering Purpose & Tuning Effect |
| :--- | :---: | :---: | :--- |
| `DRIVE_BOARD_MIN_SLOPE` | `float` | `10.0` | Slope angle (degrees) below which no damping is applied ($D = 1.0$). |
| `DRIVE_BOARD_MAX_SLOPE` | `float` | `30.0` | Slope angle (degrees) at which damping reaches maximum severity ($D = D_{\text{min}}$). |
| `DRIVE_BOARD_MIN_DAMP` | `float` | `0.4` | Minimum motor power multiplier (40% throttle) permitted at or beyond `MAX_SLOPE`. |
| `DRIVE_BOARD_MAX_DAMP` | `float` | `1.0` | Maximum motor power multiplier (100% throttle) applied when terrain is flat. |
| `DRIVE_BOARD_ROLL_WEIGHT` | `float` | `0.6` | Weight assigned to roll axis tilt. Weighted higher because lateral rollovers occur at lower angles than pitch rollovers. |
| `DRIVE_BOARD_PITCH_WEIGHT` | `float` | `0.4` | Weight assigned to pitch axis tilt. |
| `DRIVE_BOARD_YAW_WEIGHT` | `float` | `0.0` | Weight assigned to yaw axis tilt (typically zero). |

---

## 4. Video Recording Handler Constants

| Constant Name | Type | Typical Value | Engineering Purpose & Tuning Effect |
| :--- | :---: | :---: | :--- |
| `RECORDER_FPS` | `int` | `15` | Framerate limit for encoding `.mp4` video files to disk. |
| `ZED_MAINCAM_ENABLE_RECORDING` | `bool` | `true` | Toggles raw video recording from the forward ZED camera. |
| `ZED_REARCAM_ENABLE_RECORDING` | `bool` | `true` | Toggles raw video recording from the rear ZED camera. |
| `TAGDETECT_MAINCAM_ENABLE_RECORDING` | `bool` | `true` | Toggles ArUco overlay frame recording from the forward detector. |
| `TAGDETECT_REARCAM_ENABLE_RECORDING` | `bool` | `false` | Toggles ArUco overlay frame recording from the rear detector. |
| `OBJECTDETECT_MAINCAM_ENABLE_RECORDING` | `bool` | `true` | Toggles YOLO overlay frame recording from the forward detector. |
| `OBJECTDETECT_REARCAM_ENABLE_RECORDING` | `bool` | `false` | Toggles YOLO overlay frame recording from the rear detector. |

---

## 5. Camera & Perception Hardware Constants

### ZED Camera SDK Parameters
- `ZED_BASE_RESOLUTION`: `sl::RESOLUTION::HD720` (1280x720).
- `ZED_MEASURE_UNITS`: `sl::UNIT::METER`.
- `ZED_COORD_SYSTEM`: `sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP`.
- `ZED_DEPTH_MODE`: `sl::DEPTH_MODE::NEURAL` (High accuracy neural stereo matching).
- `ZED_DEFAULT_MINIMUM_DISTANCE`: `0.3f` (Clamps depth below 30 cm to prevent lens distortion artifacts).
- `ZED_DEFAULT_MAXIMUM_DISTANCE`: `25.0f` (Maximum usable range in meters).
- `ZED_DEFAULT_FLOOR_PLANE_ERROR`: `0.15f` (Floor plane detection tolerance in meters).
- `ZED_DEPTH_STABILIZATION`: `1` (Enables temporal smoothing of depth point clouds).

### Physical Extrinsic Offsets
- Forward ZED Camera:
  - `ZED_MAINCAM_EASTING_OFFSET`: `0.0` m
  - `ZED_MAINCAM_NORTHING_OFFSET`: `0.35` m (Camera mounted 35 cm forward of chassis center)
  - `ZED_MAINCAM_ALTITUDE_OFFSET`: `0.65` m (Camera mounted 65 cm above ground level)
  - Quaternion rotation offsets: `X = 0.0, Y = 0.0, Z = 0.0, W = 1.0`
- Rear ZED Camera:
  - `MODE_REAR_ZED`: `true`
  - `ZED_REARCAM_NORTHING_OFFSET`: `-0.35` m (Camera mounted 35 cm behind chassis center)
  - `ZED_REARCAM_ALTITUDE_OFFSET`: `0.65` m
  - Quaternion rotation offsets: `X = 0.0, Y = 1.0, Z = 0.0, W = 0.0` (180 degree yaw rotation)

---

## 6. Vision Detection & Tracking Constants

| Constant Name | Type | Typical Value | Engineering Purpose & Tuning Effect |
| :--- | :---: | :---: | :--- |
| `BBOX_MIN_LIFETIME_THRESHOLD` | `double` | `0.3` | Minimum duration (seconds) a detection must persist before confirmation. Filters single-frame visual noise. |
| `BBOX_MIN_SCREEN_PERCENTAGE` | `double` | `0.0005` | Minimum screen area fraction required to track an object bounding box. |
| `BBOX_TRACKER_LOST_TIMEOUT` | `double` | `1.0` | Maximum time (seconds) a lost tracker will extrapolate position before deregistration. |
| `BBOX_TRACKER_MAX_TRACK_TIME` | `double` | `30.0` | Maximum lifespan (seconds) of a continuous bounding box track before mandatory re-detection. |
| `BBOX_TRACKER_IOU_MATCH_THRESHOLD` | `double` | `0.3` | Intersection-over-Union threshold for associating new neural inferences with active trackers. |
| `TAGDETECT_TORCH_MODEL` | `std::string` | `"data/Models/best_tag.pt"` | TorchScript weight path for YOLO ArUco detection model. |
| `OBJECTDETECT_TORCH_MODEL` | `std::string` | `"data/Models/best_object.pt"` | TorchScript weight path for YOLO competition object model. |
| `TAGDETECT_MAINCAM_TORCH_CONFIDENCE` | `float` | `0.55` | Confidence score cutoff for ArUco tag neural detections. |
| `TAGDETECT_MAINCAM_TORCH_NMS_THRESH` | `float` | `0.45` | Non-Maximum Suppression IoU threshold for tag bounding boxes. |
| `OBJECTDETECT_MAINCAM_TORCH_CONFIDENCE` | `float` | `0.60` | Confidence cutoff for Mallet, Water Bottle, and Rock Pick detections. |
| `OBJECTDETECT_MAINCAM_TORCH_NMS_THRESH` | `float` | `0.45` | Non-Maximum Suppression IoU threshold for object bounding boxes. |
| `ARUCO_TAG_SIDE_LENGTH` | `float` | `0.20` | Physical edge length of competition ArUco tags (meters). Set to 0.20 m per URC rules. |

---

## 7. State Machine Execution Constants

| Constant Name | Type | Typical Value | Engineering Purpose & Tuning Effect |
| :--- | :---: | :---: | :--- |
| `STATEMACHINE_MAX_IPS` | `int` | `60` | Loop execution rate ceiling (Hz) for the core state machine thread. |
| `STATEMACHINE_ZED_REALIGN_THRESHOLD` | `double` | `1.5` | Discrepancy (meters) between visual odometry and GPS before triggering visual realignment. |
| `NAVIGATING_MOTOR_POWER` | `double` | `0.6` | Base motor power scalar while in `NavigatingState`. |
| `NAVIGATING_REACHED_GOAL_RADIUS` | `double` | `1.5` | Arrival tolerance radius (meters) around a target navigation waypoint. |
| `NAVIGATING_VERIFY_POSITION` | `bool` | `true` | When true, stops rover at waypoint and averages GPS samples to verify arrival. |
| `NAVIGATING_VERIFY_SAMPLE_TIME` | `double` | `2.0` | Duration (seconds) rover samples GPS to confirm arrival at waypoint. |
| `NAVIGATING_SLOWDOWN_WITHIN_WAYPOINT_RADIUS` | `bool` | `true` | Toggles linear speed deceleration as rover closes within waypoint arrival radius. |
| `APPROACH_MARKER_MOTOR_POWER` | `double` | `0.35` | Motor power scalar while actively homing in on an ArUco post. |
| `APPROACH_MARKER_PROXIMITY_THRESHOLD` | `double` | `1.0` | Target standoff distance (meters) for completing marker approach phase. |
| `APPROACH_MARKER_LOST_GIVE_UP_TIME` | `double` | `5.0` | Maximum time (seconds) marker can remain lost before falling back to search patterns. |
| `APPROACH_OBJECT_MOTOR_POWER` | `double` | `0.30` | Motor power scalar while closing distance to a mission object. |
| `APPROACH_OBJECT_PROXIMITY_THRESHOLD` | `double` | `0.8` | Target standoff distance (meters) for completing object approach phase. |
| `APPROACH_OBJECT_REQUIRED_TIME_HIT_RATE` | `double` | `0.5` | Required fraction of detection frames needed to maintain active homing state. |
| `SEARCH_MOTOR_POWER` | `double` | `0.40` | Motor power scalar while tracing spiral or snake search patterns. |
| `SEARCH_ANGULAR_STEP_DEGREES` | `double` | `15.0` | Angular step size (degrees) for computing Archimedean spiral search trajectory waypoints. |
| `SEARCH_SPIRAL_SPACING` | `double` | `2.0` | Radial distance (meters) between concentric arms of the spiral pattern. |
| `SEARCH_ZIGZAG_SPACING` | `double` | `3.0` | Track separation distance (meters) for zigzag search geometry. |
| `REVERSE_MOTOR_POWER` | `double` | `-0.35` | Motor effort scalar applied during `ReversingState`. |
| `REVERSE_DISTANCE` | `double` | `1.5` | Total linear distance (meters) traversed backward during recovery maneuvers. |
| `REVERSE_TIMEOUT_PER_METER` | `double` | `4.0` | Time allowance (seconds/meter) before reversing maneuver aborts due to stall. |
| `STUCK_SAME_POINT_PROXIMITY` | `double` | `0.5` | Spatial radius (meters) within which the rover is flagged as stuck if progress halts. |
| `STUCK_HEADING_ALIGN_TIMEOUT` | `double` | `8.0` | Maximum time (seconds) allotted to turn toward recovery headings in `StuckState`. |

---

## 8. Path Planning & Controller Constants

| Constant Name | Type | Typical Value | Engineering Purpose & Tuning Effect |
| :--- | :---: | :---: | :--- |
| `GEOPLANNER_TILE_SIZE` | `double` | `50.0` | Edge length (meters) of spatial tiles cached from DuckDB terrain database. |
| `ASTAR_AVOIDANCE_MULTIPLIER` | `double` | `2.5` | Multiplier inflating obstacle boundaries in the 2.5D costmap during A* search. |
| `ASTAR_MAX_SEARCH_GRID` | `double` | `150.0` | Maximum dimension (meters) of local search window to cap computational complexity. |
| `ASTAR_MAX_SEARCH_TIME` | `double` | `0.5` | Maximum execution time (seconds) before A* yields best available partial path. |
| `ASTAR_NODE_SIZE` | `double` | `0.25` | Spatial grid cell resolution (meters) for A* nodes. |
| `STANLEY_CROSSTRACK_CONTROL_GAIN` | `double` | `0.8` | Gain coefficient $k$ scaling lateral deviation correction in the Stanley controller. |
| `STANLEY_WHEELBASE` | `double` | `1.2` | Effective kinematic wheelbase length (meters) between front and rear axle centers. |
| `STANLEY_ANGULAR_VELOCITY_LIMIT` | `double` | `1.5` | Maximum permissible yaw angular rate (rad/s) computed by the Stanley controller. |
| `STANLEY_PREDICTION_HORIZON` | `int` | `5` | Lookahead steps $N$ simulated by `UnicycleModel` forward projection. |
| `STANLEY_PREDICTION_TIME_STEP` | `double` | `0.1` | Integration time step $dt$ (seconds) for kinematic unicycle trajectory simulation. |
| `STANLEY_MIN_STABLE_SPEED` | `double` | `0.15` | Minimum velocity threshold (m/s) in Stanley denominator to prevent division by zero. |
| `CLOSE_RANGE_PENALTY` | `double` | `0.5` | Speed damping scalar applied by Pure Pursuit when within close range of path terminators. |

---

## 9. Navigation Board Driver Constants

| Constant Name | Type | Typical Value | Engineering Purpose & Tuning Effect |
| :--- | :---: | :---: | :--- |
| `NAVBOARD_MAX_GPS_DATA_AGE` | `double` | `2.0` | Maximum acceptable age (seconds) of GPS packets before data is marked stale. |
| `NAVBOARD_MAX_COMPASS_DATA_AGE` | `double` | `1.0` | Maximum acceptable age (seconds) of compass packets before data is marked stale. |
| `NAVBOARD_EASTING_OFFSET` | `double` | `0.0` | GPS antenna physical mounting offset in Easting axis relative to rover center. |
| `NAVBOARD_NORTHING_OFFSET` | `double` | `-0.20` | GPS antenna mounting offset in Northing axis (meters). |
| `NAVBOARD_ALTITUDE_OFFSET` | `double` | `0.85` | GPS antenna mounting offset in Altitude axis above ground plane (meters). |
