# Autonomy Constants Tuning Guide

This is the master guide for tunable parameters within the autonomy software. All constants are defined in `src/AutonomyConstants.cpp`. Changing these values requires a recompile.

## Drive & PID Constants

| Name | Location | What it does / Why it exists | Tuning Guide (If / Then) |
| :--- | :--- | :--- | :--- |
| `DRIVE_MAX_SAFE_POWER` | `AutonomyConstants.cpp` | An absolute cap (0.0 to 1.0) on motor effort. Used as a safety feature. | **If increased**: Rover drives faster. **If decreased**: Limits top speed, making the rover safer in confined testing areas. |
| `DRIVE_PID_PROPORTIONAL` | `AutonomyConstants.cpp` | The 'P' term. Applies turn power directly proportional to the heading error. | **If increased**: Snappier turns, but may overshoot the target heading. **If decreased**: Sluggish turning, might not overcome static friction. |
| `DRIVE_PID_INTEGRAL` | `AutonomyConstants.cpp` | The 'I' term. Accumulates error over time to overcome friction/stalling. | **If increased**: Helps push past friction on carpet/grass, but too high causes massive oscillation. |
| `DRIVE_PID_DERIVATIVE` | `AutonomyConstants.cpp` | The 'D' term. Dampens the turning speed as the error approaches zero. | **If increased**: Prevents overshooting, but too high causes jittering due to latency. |

## State Machine Constants

| Name | Location | What it does / Why it exists | Tuning Guide (If / Then) |
| :--- | :--- | :--- | :--- |
| `NAVIGATING_REACHED_GOAL_RADIUS` | `AutonomyConstants.cpp` | The radius (in meters) around a waypoint that counts as a "success". | **If increased**: Waypoints are cleared earlier (sloppier pathing). **If decreased**: Rover tries to hit the exact spot (might circle infinitely). |
| `SEARCH_ANGULAR_STEP_DEGREES` | `AutonomyConstants.cpp` | The angle increment for generating spiral search patterns. | **If increased**: A sparser, wider spiral. **If decreased**: A tighter, denser spiral that takes longer to drive. |
| `SEARCH_MOTOR_POWER` | `AutonomyConstants.cpp` | Speed used when driving a search pattern. | Keep lower than navigation speed to allow cameras time to process images. |

## Vision & Tracking Constants

| Name | Location | What it does / Why it exists | Tuning Guide (If / Then) |
| :--- | :--- | :--- | :--- |
| `BBOX_MIN_LIFETIME_THRESHOLD` | `AutonomyConstants.cpp` | Seconds an object must be tracked before it is considered "real". | **If increased**: Fewer false positives, but slower reaction times. **If decreased**: Fast reaction, but might track random noise. |
| `OBJECTDETECT_MAINCAM_TORCH_CONFIDENCE` | `AutonomyConstants.cpp` | Minimum probability score (0.0-1.0) for YOLO to accept a detection. | **If increased**: Strict detections only. **If decreased**: More detections, but more false positives. |
| `TAGDETECT_MAINCAM_USE_ARUCO3_DETECTION` | `AutonomyConstants.cpp` | Enables newer OpenCV ArUco 3 strategies. | Usually leave `true` for performance. |
| `ZED_MAINCAM_USE_HALF_PRECISION_DEPTH` | `AutonomyConstants.cpp` | Uses 16-bit floats instead of 32-bit for depth map generation. | **If true**: Uses less memory and bandwidth, highly recommended for Jetson. |

*Note: Always rebuild in Release mode (`-DCMAKE_BUILD_TYPE=Release`) after tweaking constants for competition runs.*
