# ArUco Tag Detection

During competitive missions, the rover must autonomously locate, identify, and drive toward fiducial markers (AR Tags) mounted on target posts across the course.

---

## 1. The `TagDetector` Pipeline (`TagDetector.cpp`)

The `TagDetector` class inherits from `AutonomyThread<void>` and executes continuous image processing on incoming frames from its assigned camera:

```
[Camera Frame Input]
        |
        +-----------------------------------+
        |                                   |
        v                                   v
[OpenCV ArUco Detection]            [LibTorch YOLO Tag Fallback]
 - Dictionary: DICT_4X4_50           - Model: TAGDETECT_TORCH_MODEL (.pt)
 - Sub-pixel Corner Refinement       - Bounding Box & Confidence Score
 - Decode Marker ID & Corners        - Distant / Glare Detection
        |                                   |
        +-----------------+-----------------+
                          |
                          v
               [Bounding Box Tracking]
                - KCF / CSRT Tracker Updates
                - BBOX_MIN_LIFETIME_THRESHOLD Filter
                - BBOX_MIN_SCREEN_PERCENTAGE Filter
                          |
                          v
             [TagDetectionUtility::EstimatePose]
              - Trigonometric Distance Calculation
              - Optical Yaw Angle Extraction
                          |
                          v
           [tagdetectutils::ArucoTag Struct]
```

---

## 2. Detection Methods

### 1. Classical OpenCV ArUco
- **Dictionary**: `cv::aruco::DICT_4X4_50` matching official URC specifications.
- **Corner Refinement**: Uses `cv::aruco::CORNER_REFINE_SUBPIX` to pinpoint tag corner vertices at sub-pixel accuracy.
- **Inverted Marker Detection**: Toggled by `constants::TAGDETECT_MAINCAM_DETECT_INVERTED_MARKER` to detect markers in harsh shadow or backlighting.

### 2. LibTorch YOLO Fallback
When distance exceeds 10 meters, dust occludes corners, or direct sunlight washes out the tag face, classical ArUco fails to detect the geometric square.
- A custom YOLO neural network trained on marker silhouettes runs via LibTorch (`yolomodel::pytorch::PyTorchInterpreter`).
- If YOLO detects a tag bounding box with confidence $\ge \text{constants::TAGDETECT\_MAINCAM\_TORCH\_CONFIDENCE}$, the rover begins approaching the candidate blob using visual servoing until close enough for OpenCV to decode the exact integer ID.

### 3. Temporal Validation and Tracking
Visual noise and random terrain patterns can produce instantaneous false positive detections.
- Before a tag is marked valid by `TagDetectionChecker::IdentifyTargetMarker()`, its bounding box must occupy at least `constants::BBOX_MIN_SCREEN_PERCENTAGE` of the camera image and persist for at least `constants::BBOX_MIN_LIFETIME_THRESHOLD` (typically 0.5 seconds).
- Active locks are tracked between neural network inferences using OpenCV KCF or CSRT trackers.

---

## 3. Pose Estimation (`TagDetectionUtilty.hpp`)

Knowing a tag exists in frame is insufficient; the control system requires the straight-line distance and the horizontal yaw angle between the camera optical axis and the marker:

1. **Tag Corner Geometry**:
   The physical width of the marker is known: `constants::ARUCO_TAG_SIDE_LENGTH` (default 0.20 meters).
2. **Trigonometric Distance Calculation**:
   Given horizontal camera field of view $\text{FOV}_h$, image width $W$, and pixel width of the detected marker $w_{\text{px}}$:
   $$\text{Apparent Width} = \frac{w_{\text{px}}}{W}$$
   $$d_{\text{straight}} = \frac{\text{ARUCO\_TAG\_SIDE\_LENGTH}}{2 \cdot \tan\left(\frac{\text{FOV}_h \cdot \text{Apparent Width}}{2}\right)}$$
3. **Yaw Offset Angle**:
   Given tag bounding box center $x_{\text{center}}$:
   $$\text{Offset Ratio} = \frac{x_{\text{center}} - \frac{W}{2}}{\frac{W}{2}}$$
   $$\theta_{\text{yaw}} = \text{Offset Ratio} \times \frac{\text{FOV}_h}{2}$$
   - If $\theta_{\text{yaw}} > 0$, the tag is to the right of the optical axis.
   - If $\theta_{\text{yaw}} < 0$, the tag is to the left of the optical axis.

---

## 4. Usage in State Machine

Inside `ApproachingMarkerState`:
- Visual servoing feeds $\theta_{\text{yaw}}$ into the heading PID controller, commanding point-turns or curved approaches to center the tag in the frame.
- Forward speed is modulated based on $d_{\text{straight}}$.
- When $d_{\text{straight}} \le \text{constants::APPROACH\_MARKER\_PROXIMITY\_THRESHOLD}$ (e.g., 2.0 meters), the state machine triggers `Event::eReachedMarker` to transition to `eVerifyingMarker`.
