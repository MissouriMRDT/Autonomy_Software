# URC 2027 Autonomous Navigation Mission Rules

This section contains the official University Rover Challenge (URC) 2027 specifications and requirements for the Autonomy Mission, serving as the definitive baseline for tuning state machine behaviors, navigation tolerances, detection algorithms, and recovery strategies.

Official Competition Reference: [URC Requirements & Guidelines](https://urc.marssociety.org/home/requirements-guidelines)  
Local Source Rulebook: `docs/autonomy_binder/University Rover Challenge Rules 2027.pdf`

---

## 1. General Mission Overview & Operational Parameters

The URC 2027 Autonomy Mission represents a major evolution from prior years, expanding from 30 to **40 minutes** total course time and dividing the mission into two distinct 50-point sub-missions totaling **100 points**:

- **Course Duration**: **40 minutes** total time on course (Section 1.e.i).
- **Sub-Mission Architecture**:
  1. **Astronaut Assistance Sub-Mission** (50 points maximum)
  2. **Autonomous Route-Finding Sub-Mission** (50 points maximum)
- **Execution Order**: Teams may attempt the two sub-missions in any sequence (Section 1.e.i).
- **Operating Environment**: Desert terrain at the Mars Desert Research Station (MDRS) near Hanksville, Utah. The route-finding terrain spans a state-owned square mile bounded approximately by **(38.411°N, -110.786°W)** and **(38.425°N, -110.768°W)** (Section 1.e.xiii).
- **Coordinate Datum**: All coordinates are distributed in the **WGS 84** datum in latitude/longitude format (Section 3.d.v).
- **Mission Turnaround**: Teams may be scheduled to start the Equipment Servicing Mission as soon as 10 minutes following the Autonomy Mission (or vice-versa), operating from the same Command and Control (C2) station (Section 1.a).

---

## 2. Status Indicators & In-Run Reprogramming

### A. Rear LED Status Indicator (Section 1.e.ii)
The rover must feature an externally visible LED array or high-power LED indicator mounted on the rear of the chassis, clearly distinguishable in direct sunlight:

- **Solid Red**: Autonomous mode active (state machine executing).
- **Solid Blue**: Manual teleoperation active (operator joystick/teleop override).
- **Flashing Green**: Successful arrival at a target location or completion of a task.

### B. In-Run Reprogramming Policy (Section 1.e.iii)
A critical rule modernization allows operators to reprogram the rover during an active run:

- While the rover is **stopped at any time**, operators may perform any programming, including entering GNSS points, waypoints, or keep-out/stay-out zones, and tuning control algorithms or parameters.
- Operators **may not drive** the rover while performing programming.

---

## 3. Sub-Mission 1: Astronaut Assistance (50 Points Total)

A designated team member acts as an "astronaut in the field" whom the rover must assist through visual, auditory, and manipulation tasks (Section 1.e.iv - 1.e.x).

### Task Breakdown & Scoring

| Task ID | Task Name | Description & Success Criteria | Points |
| :--- | :--- | :--- | :---: |
| **1.e.iv** | **EVA Suit System** | The team must provide an EVA suit for the astronaut. The suit does not need to be flight-rated for Mars (no pressurization or oxygen tanks needed), but **must include an onboard camera and microphone** streamable to and monitored by the C2 station operators. Helmets must be easily removable for heat safety. | **5 pts** |
| **1.e.v** | **Drive to Astronaut** | The rover must autonomously navigate from the starting area to a provided GNSS coordinate where the astronaut is waiting. Success is achieved by autonomously coming to a complete stop within **3.0 meters** of the GNSS location. | **5 pts** |
| **1.e.vi** | **Follow! Command** | The astronaut gives a command to follow and walks toward a destination designated during setup. The rover must autonomously follow the walking astronaut and stop within **3.0 meters** when the astronaut halts.<br><br>Scoring scales by command complexity:<br>• **Device-based**: Command transmitted via handheld device carried by astronaut $\rightarrow$ **5 pts** (1/3 value)<br>• **Visual Sign**: Astronaut presents a physical sign displaying an AR tag, written words, or pictures $\rightarrow$ **5 pts** (1/3 value)<br>• **Audio Speech**: Voice recognition of spoken word/phrase (e.g., *"follow"*) $\rightarrow$ **10 pts** (2/3 value)<br>• **Visual Gesture**: Vision model recognizes a quiet **beckoning gesture** made by the astronaut $\rightarrow$ **15 pts** (Full value) | **15 pts** max |
| **1.e.vii** | **Stay! Command** | The astronaut commands the rover to stay in place while the astronaut walks $>20$ meters away. The rover must remain completely stationary until commanded again. | **5 pts** |
| **1.e.viii** | **Fetch! Tool Pick-Up** | The astronaut commands the rover to fetch a tool. The rover must **autonomously locate and pick up a rock pick hammer** from the ground using its robotic manipulator. (Teleoperated pick-up is permitted for recovery but awards 0 points). | **10 pts** |
| **1.e.ix** | **Come! Command** | The astronaut commands the rover to drive to the astronaut's new location. The rover must navigate and stop within **3.0 meters** of the astronaut. | **5 pts** |
| **1.e.x** | **Give! Tool Hand-Off** | On command, the rover must autonomously place the rock pick hammer onto the ground or drop it safely. | **5 pts** |

### Aborts & Exiting Autonomous Mode (Section 1.e.xi)
- **Autonomous Recovery (0% penalty)**: The rover may autonomously abort, stop, or return to the astronaut with zero point penalty. The command may be re-issued.
- **C2 Signal Abort (20% penalty)**: C2 operators may send an electronic signal commanding the rover to stop or return to the astronaut, incurring a **20% penalty** on that specific task.
- **Teleoperated Return (50% penalty)**: Operators may manually teleoperate the rover back to any previously visited location, incurring a **50% penalty** on that specific task.
- **Penalty Cap**: Exiting autonomous mode penalties are capped at **50%** per task. Subsequent aborts or teleoperation on that task consume mission time but incur no additional point deductions.

---

## 4. Sub-Mission 2: Autonomous Route-Finding (50 Points Total)

In this sub-mission, the rover is deployed in complex desert badlands and hills to navigate challenging topological routes without real-time human guidance (Section 1.e.xii - 1.e.xviii).

### Course Architecture & Targets

1. **Mission Start Location**:
   - Located on flat, accessible terrain.
   - Operators receive GNSS coordinates for the start gate and may manually teleoperate the rover to this location.

2. **Hilly Target Locations (2 Targets, 25 Points Each)**:
   - **Target 1 (Navigable Hill Ascent - Section 1.e.xv)**: Situated atop a hill. The location is selected such that not all approach vectors are traversable; the rover's planning pipeline (`GeoPlanner`) must evaluate terrain slope and contour to find an achievable ascent route.
   - **Target 2 (Non-Line-of-Sight Behind Hill - Section 1.e.xvi)**: Intentionally positioned behind the hill, completely **severing radio line-of-sight communications** with the C2 station. The rover must navigate completely autonomously without operator telemetry or remote abort links.

3. **Target Visual Identification Markers**:
   - Both targets are marked with **3-sided AR marker posts**:
     - **Post Dimensions**: 20 cm $\times$ 20 cm faces mounted 0.5 to 1.5 meters above ground level.
     - **Fiducial Tag Library**: ArUco dictionary **`DICT_4X4_50`**.
     - **Cell Geometry**: 4x4 data cells with a 1-cell wide white border (cells are **2.5 cm** across).
     - Identical tags appear on all three sides for 360-degree detection coverage.

4. **Success Criteria & Scoring (Section 1.e.xvii)**:
   - **25 points** per target reached.
   - The rover must autonomously stop within **1.0 meter** of the target location (stricter tolerance than the 3.0 m astronaut radius).
   - Must signal arrival via flashing green LED and telemetry.
   - Partial points are awarded for successfully completing portions of the route.

5. **Route-Finding Aborts & Penalties (Section 1.e.xviii)**:
   - **Autonomous Return (20% penalty)**: Operators transmit a command for the rover to autonomously retrace its steps or return to the mapping start point, assessing a **20% penalty** on the attempted target.
   - **Teleoperation (50% penalty)**: Operators teleoperate back to a previously visited location, assessing a **50% penalty** on that target. Teleoperation is strictly forbidden in areas not yet autonomously explored.
   - Mode penalties are capped at **50%** per target.

---

## 5. Aerial Drone Integration (Sections 1.e.xiv & 2.b)

URC 2027 permits and incentivizes the integration of a reconnaissance drone:

- **Reconnaissance Window**: A drone may be flown for aerial scouting during the Astronaut Assistance sub-mission to survey the route-finding terrain and map hills.
- **Landing Requirement**: The drone must return and land at the designated landing pad before the rover departs the route-finding start location (Section 1.e.xiv).
- **Technical Restrictions**:
  - Rotary-wing aircraft only (hover capable); fixed-wing and lighter-than-air craft prohibited.
  - Maximum take-off mass: **5.0 kg** (11 lbs).
  - Must carry an **inert dummy mass equal to battery weight** to simulate Mars atmospheric lift deficits (Section 2.b.v).
  - FAA compliance required: Remote ID broadcast, FAA TRUST certification for pilots, visual line-of-sight spotter in field, ceiling $\le 400$ ft AGL.

---

## 6. Physical Interventions & Equipment Regulations

### A. Team Interventions (Section 3.e)
- Any physical contact with the rover in the field constitutes an intervention.
- **Penalty**: **20% deduction** of the total points scored in the mission per intervention. Penalties are additive (e.g., 2 interventions = 40% penalty; final score is 60% of points earned).
- The 40-minute mission clock continues running during interventions.
- Only C2 operators may request an intervention; team members acting as "runners" in the field cannot re-enter the C2 station to operate during that mission.

### B. Rover Physical Constraints (Section 2.a)
- **Deployed Mass Limit**: Maximum **50.0 kg** (rounded down to nearest whole kg). Exceeding 50 kg incurs a **5% penalty per kilogram over 50 kg**.
- **Transport Envelope**: Rover must fit inside a **1.2 m $\times$ 1.2 m $\times$ 1.2 m** volume during pre-mission weigh-in without disassembly (wheels and antennas may fold). Failure to fit incurs a **40% penalty**.
- **Emergency Stop (E-Stop)**: A prominent red push-button emergency stop must be externally mounted to instantly sever all battery power.

---

## 7. Autonomous Features in Other Missions

### Equipment Servicing Mission - Autonomous Typing (Section 1.d.ii)
In addition to the Autonomy Mission, the Equipment Servicing Mission includes a dedicated autonomous scoring task:

- Operators are given a 3 to 6-letter launch key before the mission.
- The rover must autonomously position its robotic manipulator and type this launch key onto a physical keyboard.
- Operators must declare autonomous mode to judges and remain hands-off the controls.

---

## 8. State Machine & Pipeline Requirements Matrix

| Mission Phase / Task | Detection Modality | State Machine State | Tolerance / Threshold | Scoring Weight |
| :--- | :--- | :--- | :--- | :---: |
| **Astronaut Rendezvous** | Absolute GNSS coordinate | `eNavigating` | $\le$ 3.0 m radius stop | 5 pts |
| **Follow! Astronaut** | Computer vision gesture / Speech audio / Visual sign | `eApproachingMarker` / Custom Follow State | $\le$ 3.0 m following stop | 15 pts max |
| **Stay! In Place** | Zero velocity command hold | `eIdle` / `ePaused` | Complete standstill | 5 pts |
| **Fetch! Hammer** | YOLOv8s object detection (`RockPick`) | `eApproachingObject` + Manipulator Planner | Autonomous grasp & lift | 10 pts |
| **Come! To Astronaut** | Person detection / Relative beacon | `eNavigating` | $\le$ 3.0 m radius stop | 5 pts |
| **Give! Release Tool** | Manipulator release trigger | End-effector open | Autonomous drop/place | 5 pts |
| **Route Finding: Hill Target** | USGS DEM + 2.5D A* (`GeoPlanner`) + ArUco (`DICT_4X4_50`) | `eNavigating` $\rightarrow$ `eApproachingMarker` | $\le$ 1.0 m radius stop | 25 pts |
| **Route Finding: Non-LOS Target** | Pure offline autonomous navigation (no C2 link) | `eNavigating` $\rightarrow$ `eApproachingMarker` | $\le$ 1.0 m radius stop | 25 pts |
