# URC Autonomous Navigation Mission Rules

This section contains the official University Rover Challenge (URC) rules and specifications for the Autonomous Navigation Mission, as well as a quick-reference summary for constraints like search radiuses and object types. Understanding these rules is critical for tuning the State Machine constraints and search pattern sizes.

---

## 1. General Mission Constraints

- **Time and Distance**: The total time on the course will be **30 minutes**, and the cumulative distance shall be no greater than **2 km**.
- **Course Start**: Teams may be required to begin this mission as soon as 10 minutes after the completion of the Equipment Servicing Mission, operating from the same C2 (Command and Control) station on an adjacent course.
- **Reference Coordinate**: Teams will be provided with a high-accuracy coordinate at a start gate as a reference. (Teams are strongly encouraged to implement differential GNSS / RTK for higher accuracy).

## 2. Mission Objectives (7 Total Targets)

The rover is required to autonomously traverse to **7 different targets** scattered across the course. Teams may visit locations in any order.

### A. GNSS-Only Locations (2 Targets)
- **Description**: Highly accurate coordinates provided without any physical or visual markers to assist the computer vision.
- **Goal**: Navigate to the exact coordinates and stop.
- **Requirement**: Stopping within **3 meters** of the GNSS location is considered successful.
- **Search Range**: Coordinates are highly accurate; no extended search radius is expected.

### B. AR Tag Posts (2 Targets)
- **Description**: Posts marked with 3-sided visual markers displaying black and white ArUco tags.
- **Marker Specs**:
  - The posts have 20 x 20 cm faces, elevated 0.5 – 1.5m off the ground.
  - The ArUco dictionary used is exactly **`4x4_50`**.
  - Each tag has a white border 1 cell in width. Cells are exactly **2.5 cm** across.
  - The same tag is displayed on each of the 3 sides so it is visible from any direction.
- **Goal**: Navigate to the provided vicinity coordinates, find the post visually, and approach it.
- **Requirement**: Stopping within **2 meters** of the actual post is considered successful.
- **Search Range**:
  - The provided GNSS coordinate for the **First Post** will be **5 – 10 meters** away from the actual post.
  - The provided GNSS coordinate for the **Second Post** will be **10 – 20 meters** away from the actual post.

### C. Ground Objects (3 Targets)
- **Description**: Objects placed on the ground that require autonomous image detection. Some objects may be surrounded by obstacles (e.g., boulder fields) requiring autonomous avoidance.
- **Target Objects**:
  1. An orange rubber mallet.
  2. A rock pick hammer.
  3. A standard 1-liter wide-mouthed plastic water bottle (approx. 21.5 cm tall, 9 cm diameter, unspecified color/markings).
- **Goal**: Autonomously detect and highlight the object on the C2 display. The rover is *not* required to physically interact with the objects.
- **Requirement**: The rover must completely stop at any distance, so long as the object is detected. The C2 station display must clearly (and autonomously) highlight or designate the object (only 1 object may be highlighted at a time) so the judge knows the computer actually recognized it.
- **Search Range**:
  - Objects 1 and 2: The provided GNSS coordinates will have an accuracy of **<3 meters**.
  - Object 3: The provided GNSS coordinates will have an accuracy of **<10 meters**.

---

## 3. Communication & Indicators

### LED Indicators
There must be an LED indicator on the back of the rover, visible in bright daylight (e.g., an LED array or high-power LED). The autonomy software (`MultimediaBoard` driver) must control these exactly:
- **Red**: Autonomous operation.
- **Blue**: Teleoperation (Manually driving).
- **Flashing Green**: Successful arrival at a target.

### Arrival Signaling
The rover’s on-board State Machine must autonomously decide when it has reached a location. Upon reaching the location, the rover must:
1. Come to a complete stop.
2. Signal using the LED indicator (Flashing Green).
3. Display a large, obvious message or signal on the operator’s display at the C2 station for the judge to observe.

---

## 4. Aborts, Reprogramming, and Teleoperation

### Aborts & Returns
Operators may, at any point, send a signal to the rover to abort the current attempt and autonomously return to any previous GNSS coordinate, post, or object.
- The rover must stop within **5 meters** of the previous location.
- **Autonomous Return**: No point penalty.
- **Teleoperation Return**: Operators may manually drive back to any previously visited location, but will incur a **20% penalty** on the points available for that location. The teleoperation must take the most direct reasonable route back and *may not* go scouting.

### Reprogramming Constraints
Teams are permitted to do programming (e.g., entering GNSS waypoints, making changes to controls and algorithms) **ONLY** under the following conditions:
- While stopped after a successful arrival at a target.
- While stopped after returning to a previous location during an abort.

*Crucially, teams may not drive the rover during reprogramming. GNSS points given in the vicinity of posts or objects do not count as targets themselves, so no programming is allowed at those "vicinity" points unless they are arrived at during an aborted attempt on another target.*

---

## Technical Summary Cheat Sheet

| Target Type | Required Stop Distance | Initial GNSS Accuracy (Search Radius) | Detection Specs |
| :--- | :--- | :--- | :--- |
| **GNSS-Only 1 & 2** | <= 3 meters | Highly Accurate (Exact target) | None |
| **AR Post 1** | <= 2 meters | 5 to 10 meters off target | `DICT_4X4_50`, 2.5cm cells |
| **AR Post 2** | <= 2 meters | 10 to 20 meters off target | `DICT_4X4_50`, 2.5cm cells |
| **Object 1 & 2** | Any distance | < 3 meters off target | Visual Bounding Box (Mallet/Hammer) |
| **Object 3** | Any distance | < 10 meters off target | Visual Bounding Box (Water Bottle) |

*Note: Object targets require the rover to fully stop and display exactly 1 bounding box/highlight on the basestation GUI.*
