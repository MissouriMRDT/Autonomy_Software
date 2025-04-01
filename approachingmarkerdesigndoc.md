/*
# ApproachingMarkerState Design Requirements and Implementation Outline

## 1. Overview
The `ApproachingMarkerState` is a state in the autonomy state machine responsible for guiding the rover towards a detected marker using our OpenCV, TensorFlow, and
PyTorch computer vision systems. This document outlines the design requirements, features, edge cases, state events, and transitions for the `ApproachingMarkerState`.

---

## 2. Features

### 2.1 Marker Detection
- Use multiple vision-based detection systems (OpenCV, TensorFlow, and PyTorch) to identify markers.
- Prioritize OpenCV for detection due to its reliability, but fall back to PyTorch if OpenCV fails. TensorFlow should be used last, as the predictions from the
  EdgeTPU and tflite models are not as reliable.

### 2.2 Marker Filtering and Selection
- For Aruco markers, filter based on ID and closest distance to the rover.
- For PyTorch markers, filter based on detection time and distance from the rover.
- For TensorFlow markers, filter based on detection time and distance from the rover.

### 2.3 Driving Towards the Marker
There are two ways we can do this...
- Via the Camera's Point Cloud:
    Once a marker is detected and selected, we could reference the camera's point cloud to determine the relative position of the marker from the rover. Then, since we
    know the rover's UTM position, we could calculate the marker's UTM position. This would allow us to drive towards the marker using the rover's GPS and our normal
    differential drive control. (same way that navigating state works). Doing it this way would also allow us to use the tags UTM position as a goal for A*, so path
planning with obstacle avoidance would work nicely. THE ISSUE: At far distance the camera's point cloud is not very accurate. So, we would need to be within a certain
distance of the marker before we could use this method.

- Via the direct RGB image:
    Once a marker is detected and selected, we could use the FOV of the camera and the marker's X pixel position in the iamge to calculate the relative heading to the
marker. Then, we could use the marker's Y pixel position in the image to calculate the distance to the marker. This would allow us to drive towards the marker using the
rover's differential drive control. This method would work at any distance, but the accuracy of the heading and distance would be less than the first method. THE ISSUE:
The marker's Y pixel position in the image is not very accurate at judging distance. We could try to use the depth image distance. We would still run into the issue of
the camera's point cloud not being very accurate at far distances, but our heading would at least always be accurate.

- Once we are within 2m of the marker, we should stop the rover and flash the LED panel green to indicate that we have reached the marker.

### 2.4 Logging and Feedback
- Log detection time, identified markers, and rover movement details.
- Provide periodic updates on the rover's distance from the marker.

---

## 3. Edge Cases

### 3.1 Marker Not Detected
- If no marker is detected after a predefined time has passed, trigger the `eMarkerUnseen` event to abort the state.

### 3.2 Multiple Markers Detected
- Select the closest marker as the target.
- Ensure the selected marker meets time seen requirements.

### 3.3 Marker Lost During Approach
- If the marker is lost during the approach, keep driving towards the last known heading and distance.
- If the marker is not detected after a certain amount of time, trigger the `eMarkerUnseen` event.

### 3.4 Vision System Failures
- Handle cases where both OpenCV, PyTorch, and TensorFlow fail to detect markers.
- Use the last known heading and distance to continue the approach if possible.

---

## 4. State Events and Transitions

### 4.1 Events
- **`eStart`**: In this state, start will not necessarily do anything, but should still exist as a defined event.
- **`eReachedMarker`**: Triggered when the rover reaches the marker.
- **`eMarkerUnseen`**: Triggered when the marker cannot be detected or is lost.
- **`eAbort`**: Triggered to abort the state and return to idle.

### 4.2 Transitions
- From `eStart`:
  1) Print log message indicating the state start has been triggered.
  2) Send autonomy lighting state (red)
- From `eReachedMarker`:
  1) Print log message indicating the marker has been reached.
  2) Send autonomy lighting state (green)
  3) Transition to `Idle` state.
- From `eMarkerUnseen`:
    1) Print log message indicating the marker is unseen.
    2) Send autonomy lighting state (red)
    3) Transition to `Idle` state.
- From `eAbort`:
  - Transition to `Idle` state.

---

## 5. Implementation Outline

### 5.1 Constructor and Start()
- Reset any local timers.
- Load the tag detectors.

### 5.2 Run()
1) Get the rover's current pose from the WaypointHandler.
2) Request the a copy of the aruco, torch, and tensorflow tags from the detectors.
3) Identify the target marker using the `IdentifyTargetArucoMarker`, `IdentifyTargetTorchMarker`, and `IdentifyTargetTensorflowMarker` methods.
    NOTE: This method should be called in the order of OpenCV, PyTorch, and TensorFlow. It should also do the checking for the total detection time and other stuff that we want.
4) If a target marker is identified, calculate the heading and distance to the marker.
5) Determine if the rover is close enough to the marker to stop. If so, trigger the `eReachedMarker` event.
6) Otherwise, continue to drive towards the marker using the calculated heading and distance.
7) If the marker is not detected after a certain amount of time, trigger the `eMarkerUnseen` event.
