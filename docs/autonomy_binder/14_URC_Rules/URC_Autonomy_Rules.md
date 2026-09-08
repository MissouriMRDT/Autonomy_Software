# URC Autonomous Navigation Mission Rules

This section contains the official University Rover Challenge (URC) specifications and rules for the Autonomous Navigation Mission, providing a reference for tuning state machine constraints, tolerance radiuses, and detection thresholds.

---

## 1. General Mission Constraints

- **Course Duration and Length**: The total mission window is **30 minutes**, with a cumulative course distance up to **2.0 km**.
- **Mission Turnaround**: Teams may be required to begin the Autonomous Navigation Mission as soon as 10 minutes following the completion of the Equipment Servicing Mission, operating from the same Command and Control (C2) station.
- **Reference Start Gate**: Teams are provided with a reference GNSS coordinate at a designated start gate. Differential GNSS (RTK) is permitted and strongly recommended for base station and rover ground-truth positioning.

---

## 2. Mission Target Specifications (7 Total Targets)

The course features **7 distinct targets** distributed across desert terrain. Teams are permitted to attempt and clear targets in any operational sequence.

### A. GNSS-Only Locations (2 Targets)
- **Description**: Target coordinates provided as pure GNSS coordinates with no physical visual markers or posts at the site.
- **Objective**: Autonomously navigate to the designated coordinate and come to a complete stop.
- **Success Criteria**: The rover must come to a complete halt within **3.0 meters** of the target GNSS coordinate.
- **Search Radius**: Target coordinates are provided with high precision. No expanded search pattern is required.

### B. AR Tag Posts (2 Targets)
- **Description**: Vertical posts equipped with three-sided visual markers displaying black and white ArUco tags.
- **Physical Specifications**:
  - Markers have 20 cm x 20 cm faces mounted 0.5 to 1.5 meters above ground level.
  - ArUco dictionary: **`DICT_4X4_50`**.
  - Marker grid: 4x4 data cells with a 1-cell wide white border. Each individual cell measures **2.5 cm**.
  - All three sides of the post display identical marker IDs to ensure 360-degree visibility.
- **Objective**: Navigate to vicinity coordinates, locate the post using optical cameras, and approach.
- **Success Criteria**: The rover must autonomously halt within **2.0 meters** of the marker post.
- **GNSS Offset and Search Radiuses**:
  - **Post 1**: Provided vicinity GNSS coordinate is **5 to 10 meters** from the physical post.
  - **Post 2**: Provided vicinity GNSS coordinate is **10 to 20 meters** from the physical post.
  - Software must trigger `eSearchPattern` upon entering the vicinity to locate the marker visually.

### C. Ground Objects (3 Targets)
- **Description**: Loose objects placed on terrain requiring autonomous optical identification. Objects may be located near obstacles (such as rock gardens or berms) requiring autonomous obstacle avoidance.
- **Target Prop Specifications**:
  1. An orange rubber mallet.
  2. A rock pick hammer.
  3. A standard 1-liter wide-mouthed plastic water bottle (approximately 21.5 cm height, 9.0 cm diameter, unconstrained color/labeling).
- **Objective**: Detect the target prop visually and highlight it on the operator C2 display. Physical contact or manipulation is not required.
- **Success Criteria**:
  - The rover must come to a complete stop at any distance with the target object in clear optical view.
  - The C2 operator display must autonomously and distinctly highlight exactly one bounding box around the target object to verify computer vision recognition to the judging panel.
- **GNSS Offset and Search Radiuses**:
  - **Objects 1 and 2**: Provided vicinity coordinates have an error offset of **< 3.0 meters**.
  - **Object 3**: Provided vicinity coordinates have an error offset of **< 10.0 meters**.

---

## 3. Communication, Telemetry, and Lighting

### Visual Status Indicators
The rover must carry an externally visible status LED assembly (controlled via the `MultimediaBoard` driver) meeting the following operational conventions:
- **Solid Red**: Autonomous mode active (state machine executing).
- **Solid Blue**: Manual teleoperation active (operator joystick override).
- **Flashing Green**: Objective arrival confirmed (state machine completed target verification).

### Arrival Confirmation Protocol
Upon reaching a target location, the rover must autonomously:
1. Cease all drive motor commands (`SendStop()`).
2. Activate the flashing green LED indicator.
3. Broadcast telemetry to the C2 display over RoveComm, triggering an unambiguous visual prompt for judges.

---

## 4. Operational Protocols: Aborts, Returns, and Reprogramming

### Aborts and Location Returns
Operators may command an autonomous abort at any point:
- The rover may autonomously return to any previously visited target location or reference coordinate.
- The rover must halt within **5.0 meters** of the prior target coordinate.
- **Autonomous Return Penalty**: 0% penalty.
- **Teleoperated Return Penalty**: If operators manually pilot the rover back to a previously visited target, a **20% penalty** is assessed against the maximum points available for that target. Teleoperation must follow the most direct navigable route back without scouting unvisited areas.

### Reprogramming Conditions
Operators may only transmit waypoint coordinates, modify configuration parameters, or alter software logic under strict mission conditions:
- While halted following a successful target arrival confirmation.
- While halted following an abort return to a previously visited location.

No manual waypoint input or parameter changes are permitted while the rover is in motion or stopped at unverified vicinity coordinates.

---

## 5. Technical Summary Cheat Sheet

| Target Type | Required Stop Distance | Initial Coordinate Offset (Search Radius) | Visual Specification | State Machine Event Trigger |
| :--- | :--- | :--- | :--- | :--- |
| **GNSS 1 & 2** | $\le$ 3.0 meters | 0 meters (exact) | None | `eReachedGpsCoordinate` |
| **AR Post 1** | $\le$ 2.0 meters | 5 to 10 meters | `DICT_4X4_50`, 20x20 cm | `eMarkerSeen` $\rightarrow$ `eReachedMarker` |
| **AR Post 2** | $\le$ 2.0 meters | 10 to 20 meters | `DICT_4X4_50`, 20x20 cm | `eMarkerSeen` $\rightarrow$ `eReachedMarker` |
| **Object 1 & 2** | Any (optical lock) | $\le$ 3.0 meters | Mallet / Rock Pick | `eObjectSeen` $\rightarrow$ `eReachedObject` |
| **Object 3** | Any (optical lock) | $\le$ 10.0 meters | 1-Liter Water Bottle | `eObjectSeen` $\rightarrow$ `eReachedObject` |
