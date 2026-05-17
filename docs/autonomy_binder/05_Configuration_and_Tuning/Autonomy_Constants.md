# Autonomy Constants Tuning Guide

This is the master guide for tunable parameters within the autonomy software. All constants are defined in `src/AutonomyConstants.cpp`. Changing these values requires a recompile.

Because this file is extensive, it is broken down into logical sections matching the structure of the source code.

## 1. General & Logging Constants

| Constant Name | What it does / Why it exists | Tuning Guide (If / Then) |
| :--- | :--- | :--- |
| `MODE_SIM` | Toggles RoveComm and Cameras to use local simulator data instead of physical hardware. | **If true**: Connects to localhost Webots/Unreal Engine. **If false**: Connects to physical rover hardware. |
| `BATTERY_MINIMUM_CELL_VOLTAGE` | The minimum battery cell voltage (default 3.2V) before autonomy forcefully enters the Idle state to prevent lipo damage. | **If increased**: Shuts down earlier, saving battery health but reducing runtime. **If decreased**: Risky; might damage the battery. |
| `BATTERY_CHECKS_ENABLED` | Toggles whether autonomy actually monitors the PMS currents for shutdown. | Leave **true** on the physical rover, **false** in testing environments without PMS. |
| `CONSOLE_DEFAULT_LEVEL` <br> `FILE_DEFAULT_LEVEL` <br> `ROVECOMM_DEFAULT_LEVEL` | Sets the default Quill logging verbosity (e.g., `Info`, `Debug`, `TraceL3`) for different streams. | **If increased to Trace**: Generates massive log files, useful for deep debugging. **If decreased to Info/Warning**: Cleaner output for competition. |

## 2. Drive & Kinematics Constants

| Constant Name | What it does / Why it exists | Tuning Guide (If / Then) |
| :--- | :--- | :--- |
| `DRIVE_MAX_SAFE_POWER` | An absolute cap (0.0 to 1.0) on motor effort across all states. Used as a hard safety feature. | **If increased**: Rover drives faster overall. **If decreased**: Limits top speed, making the rover safer in confined testing areas. |
| `DRIVE_PID_PROPORTIONAL` | The 'P' term for heading correction. Applies turn power directly proportional to the heading error. | **If increased**: Snappier turns, but may oscillate/overshoot the target heading. **If decreased**: Sluggish turning, might not overcome static friction. |
| `DRIVE_PID_INTEGRAL` | The 'I' term. Accumulates error over time to help the rover overcome friction or stalling. | **If increased**: Helps push past friction on carpet/grass, but too high causes massive oscillation. **If decreased**: Rover might stall on small heading errors. |
| `DRIVE_PID_DERIVATIVE` | The 'D' term. Dampens the turning speed as the error approaches zero to prevent overshoot. | **If increased**: Prevents overshooting, but too high causes jittering due to network/actuator latency. |
| `DRIVE_PID_MAX_INTEGRAL_TERM` | The maximum effort (0.0 to 1.0) the Integral term is allowed to contribute. Prevents "integral windup". | **If increased**: Allows the I-term to push harder against massive resistance. |
| `DRIVE_SQUARE_CONTROL_INPUTS` | Toggles squaring the inputs in the differential drive inverse kinematics. | **If true**: Makes fine inputs (at low speeds) smoother, but the rover feels less immediately responsive. |
| `DRIVE_CURVATURE_KINEMATICS_ALLOW_TURN_WHILE_STOPPED` | Allows the curvature drive model to point-turn when forward speed is zero. | Leave **true** to allow the rover to spin in place like a tank. |

## 3. Drive Board Multipliers (Inclinometer Constraints)

These constants dynamically scale back motor effort when the rover detects it is driving on steep terrain.

| Constant Name | What it does / Why it exists | Tuning Guide (If / Then) |
| :--- | :--- | :--- |
| `DRIVE_BOARD_MIN_SLOPE` | The minimum slope (degrees) before the damping multiplier activates. | **If decreased**: Starts slowing the rover down on shallower hills. |
| `DRIVE_BOARD_MAX_SLOPE` | The maximum slope (degrees) where damping reaches its absolute maximum effect. | **If increased**: Allows the rover to drive at full power up steeper inclines before max damping hits. |
| `DRIVE_BOARD_MIN_DAMP` | The minimum speed multiplier (e.g., 0.5 = 50% max speed) applied when at the max slope. | **If decreased**: Rover drives even slower on steep hills to prevent tipping. |
| `DRIVE_BOARD_ROLL_WEIGHT` <br> `DRIVE_BOARD_PITCH_WEIGHT` | The percentage of importance given to the Roll (side-to-side) vs Pitch (front-to-back) axes. | Roll is usually weighted higher (e.g., 0.6) because rovers are more susceptible to barrel-rolling than back-flipping. |

## 4. Vision & Perception Constants

### ZED Camera (Main & Rear)
| Constant Name | What it does / Why it exists | Tuning Guide (If / Then) |
| :--- | :--- | :--- |
| `ZED_MAINCAM_RESOLUTION` | Sets the internal processing resolution for the ZED SDK (e.g., `HD720`, `HD1080`). | **If increased**: Better long-range detection, drastically lower FPS. **If decreased**: Fast FPS, but poor long-range visibility. |
| `ZED_MAINCAM_FPS` | The requested hardware framerate for the camera. | Keep at 30 or 60. Note: SDK processing will likely bottleneck this anyway. |
| `ZED_MAINCAM_DEPTH_MODE` | The algorithm used for stereoscopic depth calculation (e.g., `ULTRA`, `NEURAL`). | `NEURAL` is highly accurate but requires heavy GPU usage. `ULTRA` is a good fallback. |
| `ZED_MAINCAM_USE_HALF_PRECISION_DEPTH` | Uses 16-bit floats instead of 32-bit for the depth map matrix. | **If true**: Uses half the RAM and bandwidth, highly recommended for Jetson architectures. |
| `ZED_MAINCAM_EASTING_OFFSET` (etc.) | Positional offset of the camera relative to the center of the rover. | Crucial for accurate `Geolocate` math. Update if the camera is physically moved on the chassis. |

### Detectors (ArUco & YOLO)
| Constant Name | What it does / Why it exists | Tuning Guide (If / Then) |
| :--- | :--- | :--- |
| `BBOX_MIN_LIFETIME_THRESHOLD` | Seconds an object must be tracked before it is considered a valid, "real" detection. | **If increased**: Eliminates false positives, but delays state machine reactions. **If decreased**: Fast reaction, but might track random visual noise. |
| `TAGDETECT_MAINCAM_ENABLE_TORCH` | Enables the YOLO PyTorch model to run alongside traditional ArUco. | **If true**: Better detection of blurry/far tags. Requires GPU. |
| `TAGDETECT_MAINCAM_TORCH_CONFIDENCE` | Minimum probability score (0.0-1.0) for YOLO to accept an AR tag detection. | **If increased**: Strict detections only. **If decreased**: More detections, but more false positives. |
| `OBJECTDETECT_MAINCAM_TORCH_CONFIDENCE` | Minimum probability score for YOLO to accept an object (mallet/bottle) detection. | Same as above. |
| `TAGDETECT_MAINCAM_USE_ARUCO3_DETECTION` | Enables newer OpenCV ArUco 3 strategies. | Usually leave `true` for performance unless using very old OpenCV versions. |

## 5. State Machine Constants

| Constant Name | What it does / Why it exists | Tuning Guide (If / Then) |
| :--- | :--- | :--- |
| `STATEMACHINE_MAX_IPS` | The maximum iterations per second for the State Machine logic loop. | Keep around 60. Too high wastes CPU polling sensors; too low causes sluggish behavior. |
| `STATEMACHINE_ZED_REALIGN_THRESHOLD` | The error threshold (meters) between visual odometry and GPS before forcing a camera realign. | **If decreased**: Realigns constantly, causing stuttering. **If increased**: Allows the robot to drift further before correcting its internal map. |
| `APPROACH_MARKER_PROXIMITY_THRESHOLD` | How close (meters) the rover must be to the tag to consider the approach complete. | **If increased**: Stops further away from the tag. |
| `SEARCH_ANGULAR_STEP_DEGREES` | The angle increment for generating spiral search patterns. | **If increased**: Generates a sparser, wider spiral. **If decreased**: Generates a tighter, denser spiral that takes longer to drive. |
| `NAVIGATING_MOTOR_POWER` | Speed used when actively navigating toward a waypoint. | Usually set higher (e.g., 90% of max safe power) for fast traversal. |
| `NAVIGATING_REACHED_GOAL_RADIUS` | The radius (in meters) around a waypoint that counts as a "success". | **If increased**: Waypoints are cleared earlier (sloppier pathing). **If decreased**: Rover tries to hit the exact spot (might circle infinitely). |
| `NAVIGATING_VERIFY_POSITION` | Toggles whether the rover stops at the end of a nav sequence to average GPS data and confirm it actually arrived. | **If true**: More accurate final positioning, but takes `NAVIGATING_VERIFY_SAMPLE_TIME` seconds longer to finish. |
| `STUCK_CHECK_ROT_THRESH` <br> `STUCK_CHECK_VEL_THRESH` | The minimum angular/linear velocities required for the rover to be considered "moving". | If the rover is commanded to move but velocity stays below these thresholds for a set time, `eStuck` state triggers. |
| `REVERSE_MOTOR_POWER` <br> `REVERSE_DISTANCE` | How fast and how far the rover drives backward during an `eReversing` state sequence. | Tune based on how aggressively the rover tends to overshoot goals. |

## 6. Algorithm Constants (A* & GeoPlanner)

| Constant Name | What it does / Why it exists | Tuning Guide (If / Then) |
| :--- | :--- | :--- |
| `ASTAR_NODE_SIZE` | The physical size (in meters) of a single grid square in the A* world map. | **If increased**: Pathfinding is computationally much faster, but the rover can't plan through narrow gaps. **If decreased**: High resolution planning, but massively spikes CPU usage on long routes. |
| `ASTAR_AVOIDANCE_MULTIPLIER` | Multiplier for marking extra nodes around detected objects as impassable obstacles. | **If increased**: Rover gives obstacles a much wider berth. **If decreased**: Rover cuts close to rocks/walls, risking physical collision. |
| `ASTAR_MAX_SEARCH_GRID` | Maximum size of the search grid in meters. | Keeps the algorithm from expanding to infinity if the goal is completely blocked. |
| `STANLEY_CROSSTRACK_CONTROL_GAIN` | (If using Stanley Controller) Determines how reactive the rover is to cross-track errors. | **If increased**: Rover aggressively snaps back to the path line. **If decreased**: Rover smoothly/lazily drifts back to the path. |

## 7. Driver Constants

| Constant Name | What it does / Why it exists | Tuning Guide (If / Then) |
| :--- | :--- | :--- |
| `NAVBOARD_MAX_GPS_DATA_AGE` | The maximum age (seconds) of GPS data before the system starts throwing warnings. | If this throws, your RoveComm connection to the Navigation Board is likely dropping packets. |
| `NAVBOARD_EASTING_OFFSET` (etc.) | Positional offset of the GPS antenna relative to the center of the rover. | Crucial for exact global positioning. Update if the antenna is physically moved on the chassis. |
