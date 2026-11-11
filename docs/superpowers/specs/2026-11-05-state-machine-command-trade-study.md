# Engineering Trade Study: Top-Level States vs. Dedicated Sub-Mission Controller for Astronaut Assistance

> **Document Status**: Engineering Trade Study & Architectural Evaluation  
> **Author**: Autonomy Lead  
> **Target Milestone**: Software Critical Design Review (CDR) Preparation  
> **Subsystems Investigated**: `StateMachineHandler`, `State.hpp`, `WaypointHandler`, `AstronautMissionController`, RoveComm Telemetry, Catch2 Testing  
> **Primary References**: 
> * [`src/interfaces/State.hpp`](file:///c:/Users/ltkli/Documents/GitHub/Autonomy_Software/src/interfaces/State.hpp)
> * [`src/handlers/StateMachineHandler.cpp`](file:///c:/Users/ltkli/Documents/GitHub/Autonomy_Software/src/handlers/StateMachineHandler.cpp)
> * [`src/handlers/WaypointHandler.h`](file:///c:/Users/ltkli/Documents/GitHub/Autonomy_Software/src/handlers/WaypointHandler.h)
> * [`docs/superpowers/specs/2026-11-05-state-machine-architecture-design.md`](file:///c:/Users/ltkli/Documents/GitHub/Autonomy_Software/docs/superpowers/specs/2026-11-05-state-machine-architecture-design.md)
> * URC 2027 Rules Section 1.e.xviii & Official URC Q&A #6, #7, #16

---

## 1. Executive Summary & Problem Formulation

The University Rover Challenge (URC) 2027 introduced the **Astronaut Assistance Sub-Mission (50 Points)**. The rover must interact with a suited human in the field across five distinct sequential tasks:
1. `Follow!` (15 pts): Maintain a 2.0 to 2.5 m trailing distance while following a walking astronaut.
2. `Stay!` (5 pts): Halt and remain stationary within 3 to 5 m of the astronaut for 5.0 seconds.
3. `Fetch!` (10 pts): Turn, locate a dropped tool (rock pick) via vision, drive to it, and grasp it.
4. `Come!` (10 pts): Return to the astronaut and halt within 2.5 m.
5. `Give!` (10 pts): Extend the manipulator/winch and release the tool to the astronaut.

When translating these five tasks into software architecture, our team faced a core structural decision:

* **Approach A (Hierarchical Sub-Mission Controller):** Represent the entire interaction as a single top-level state (`statemachine::States::eAstronautAssistance`). Delegate task sequencing, visual servoing, and button decoding internally to an isolated `AstronautMissionController`.
* **Approach B (Flat Top-Level States):** Expand the global `statemachine::States` enum to include each command as a first-class state (`eAstronautFollow`, `eAstronautStay`, `eAstronautFetch`, `eAstronautCome`, `eAstronautGive`), letting `StateMachineHandler` manage every transition directly.
* **Approach C (Polymorphic Flat States with Base Mixin):** Expose individual top-level states, but have them inherit from a shared `AstronautStateBase` class that manages suit telemetry, standoff braking, and downward camera handles.

This trade study evaluates all three approaches across code complexity, state explosion, waypoint queue semantics, telemetry visibility, hardware lifecycle management, and testability.

---

## 2. Architectural Comparison of the Approaches

```
APPROACH A: HIERARCHICAL SUB-MISSION CONTROLLER
[ Global State Machine (9 States) ]
    eIdle <---> eNavigating <---> eApproachingMarker <---> eVerifyingMarker
                     |
                     | Reached Astronaut Waypoint
                     v
             +-------------------------------------------------------+
             | eAstronautAssistance (Single Top-Level State)         |
             |                                                       |
             |   [AstronautMissionController: Internal Sub-FSM]     |
             |   Sub_Follow -> Sub_Stay -> Sub_Fetch -> Sub_Come     |
             |                                         |             |
             |                                         v             |
             |                                    Sub_Give           |
             +-------------------------------------------------------+
                     |
                     | eAstronautMissionComplete
                     v
                 eNavigating (Proceeds to Target 1 Hill)


APPROACH B: FLAT TOP-LEVEL STATES (FLAT FSM)
[ Global State Machine (14 States) ]
    eIdle <---> eNavigating <---> eApproachingMarker <---> eVerifyingMarker
                     |
                     | Reached Astronaut Waypoint
                     v
             eAstronautFollow
                     | Button / Gesture
                     v
             eAstronautStay
                     | Button / Gesture
                     v
             eAstronautFetch  <--- (What if rover gets stuck here?)
                     | Tool Grasped
                     v
             eAstronautCome
                     | Arrived
                     v
             eAstronautGive
                     | Released
                     v
    eNavigating (Proceeds to Target 1 Hill)
```

---

## 3. Criterion-by-Criterion Trade Study

### Criterion 1: Waypoint Queue & Navigation Semantics (`WaypointHandler`)

In our codebase, [`WaypointHandler`](file:///c:/Users/ltkli/Documents/GitHub/Autonomy_Software/src/handlers/WaypointHandler.h) maintains the mission itinerary as a FIFO queue of `geoops::Waypoint` objects. Each waypoint carries a geographic UTM/GPS coordinate and a `WaypointType` enum:

```cpp
enum class WaypointType {
    eNavigationWaypoint,
    eTagWaypoint,
    eObjectWaypoint,
    ...
};
```

#### How the Rover Advances Waypoints
In `NavigatingState.cpp`, when the rover arrives at a goal coordinate:
1. It reads the current waypoint from `globals::g_pWaypointHandler->PeekNextWaypoint()`.
2. It executes the leg.
3. Upon completing verification (`eVerifyingComplete`), it calls `globals::g_pWaypointHandler->PopNextWaypoint()` and transitions back to `eNavigating`.

#### Impact on Approach A (Hierarchical)
* **Clean 1:1 Mapping:** The base station queues a single waypoint for the astronaut:
  `Waypoint(Lat, Lon, WaypointType::eAstronautWaypoint)`.
* When the rover drives to that staging coordinate, `NavigatingState` detects `eAstronautWaypoint` and transitions to `eAstronautAssistance`.
* All five tasks occur locally at that staging coordinate (within a 20-meter radius).
* When the fifth task (`Give!`) finishes, the controller signals `eAstronautMissionComplete`.
* The state machine pops that single astronaut waypoint and immediately routes to Target 1 on the hill.

#### Impact on Approach B (Flat Top-Level States)
* **Semantic Breakdown in `WaypointHandler`:**
  * Does the operator have to insert **five identical waypoints** into the queue for the same geographic location? If so, why would the rover "navigate" to the exact spot it is already standing on between `Stay` and `Fetch`?
  * If the operator does *not* insert five waypoints, when does `PopNextWaypoint()` get called?
  * If `eAstronautFollow` does not pop the waypoint, but `eAstronautGive` does, then the state machine violates its core contract. Four of the five states would bypass the standard waypoint lifecycle, requiring custom conditional checks inside `StateMachineHandler::ChangeState`.

> **Verdict on Waypoint Semantics:** **Approach A is vastly superior.** It preserves the core invariant that one queue entry equals one geographic destination.

---

### Criterion 2: State Explosion & Global Failsafe Routing

In [`src/handlers/StateMachineHandler.cpp`](file:///c:/Users/ltkli/Documents/GitHub/Autonomy_Software/src/handlers/StateMachineHandler.cpp#L285-L302), every event is processed through:
```cpp
statemachine::States eNextState = m_pCurrentState->TriggerEvent(eEvent);
ChangeState(eNextState, bSaveCurrentState);
```

#### Global Event Combinatorics
Autonomy defines several global, high-priority events that can fire at any second:
* `Event::eAbort`: Operator commands emergency stop.
* `Event::eReverse`: LiDAR or ultrasonic sensors detect a cliff edge or negative obstacle.
* `Event::eStuck`: Wheel tachometers detect zero progress under motor power.
* `PMSCellVoltageCallback`: Battery cell drops below 3.0 V, forcing an immediate transition to `eIdle`.

In a flat state machine (Approach B), increasing the state count from 9 to 14 states significantly expands the transition matrix:

| Metric | Approach A (Hierarchical) | Approach B (Flat 14 States) |
| :--- | :--- | :--- |
| **Top-Level States in Enum** | 9 | 14 |
| **Global Transition Branches to Maintain** | 9 states $\times$ 4 global events = 36 branches | 14 states $\times$ 4 global events = 56 branches |
| **Boilerplate Files (`.h` / `.cpp`)** | 2 files (`AstronautAssistanceState` + `Controller`) | 10 files (5 distinct `.h` and 5 distinct `.cpp`) |
| **`StateMachineHandler::CreateState` Cases** | 9 switch cases | 14 switch cases |

#### State Recall and Unwinding (`m_umSavedStates`)
In `StateMachineHandler.cpp:136-152`, when the rover encounters an obstacle or gets stuck, it saves the current state into `m_umSavedStates` via `bSaveCurrentState = true`, transitions to `eReversing` or `eStuck`, and recalls the saved state once the maneuver completes.

* **Under Approach A:** If the rover gets stuck in loose dirt during `Fetch!`, `StateMachineHandler` saves `eAstronautAssistance`. When unstuck, it restores `eAstronautAssistance`. The internal `AstronautMissionController` is still pointing to `Sub_ExecutingFetch`. The tool recovery picks up right where it stalled.
* **Under Approach B:** If the rover gets stuck during `eAstronautFetch`, `StateMachineHandler` saves `eAstronautFetch`. When unstuck, it must re-instantiate `eAstronautFetch`. But `eAstronautFetch` must now reconstruct its camera handles, re-verify tool distance, and confirm that the gripper did not partially close. Managing this across five separate state classes introduces five separate state-restoration edge cases.

> **Verdict on State Complexity:** **Approach A wins.** It shields the top-level 60 Hz event loop from transient human interaction states that are only active for 15 minutes of the competition.

---

### Criterion 3: Hardware Lifecycle & Resource Contention (RAII)

The physical rover contains specialized hardware peripherals that are used **only** during the astronaut sub-mission:
1. **Autohead Downward Camera (`eAutoheadDownwardCam`):** Used during `Fetch!` visual servoing.
2. **Autohead Winch & Gripper Smoco Motor Controller:** Controlled via RoveComm packets (Data IDs 3020 to 3023) for lowering, snapping, and raising the tool.
3. **Spacesuit 900 MHz GNSS Telemetry Socket:** Used to track the walking human.

During the rest of the 45-minute competition (Route Finding across steep desert hills), these peripherals should be powered down, closed, or homed to save compute and battery power.

#### In Approach A (Hierarchical):
Resource management follows clean RAII (Resource Acquisition Is Initialization):
* **On State Entry (`AstronautAssistanceState::Start()`):**
  * Opens the downward camera video device (`/dev/v4l/by-id/...`).
  * Binds the telemetry UDP socket to listen for suit coordinates.
  * Sends a homing packet to the winch motor controller.
* **On State Exit (`AstronautAssistanceState::Exit()`):**
  * Commands the winch to the fully stowed top limit.
  * Ensures the gripper fingers are locked.
  * Closes the downward camera feed.
  * Releases the suit telemetry socket.

If an emergency stop (`eAbort`) occurs at any point during Follow, Stay, Fetch, Come, or Give, `StateMachineHandler` calls `m_pCurrentState->Exit()`. **One function** cleanly stows the arm, powers down the downward camera, and secures the rover.

#### In Approach B (Flat Top-Level States):
Hardware ownership is fractured across five classes:
* Who opens the downward camera? If opened in `eAstronautFetch::Start()`, what happens if the camera driver fails to initialize? Does `eAstronautStay` pre-initialize it?
* Who stows the arm if the operator hits Abort during `eAstronautGive`?
* If `eAstronautFollow` opens the suit telemetry socket, but transitions to `eAstronautStay`, does `eAstronautStay` close the socket and `eAstronautCome` reopen it? Or must the socket live in global memory (`AutonomyGlobals.h`), polluting the global namespace?

> **Verdict on Hardware Lifecycle:** **Approach A is substantially safer.** Fracturing hardware setup and teardown across five separate state classes creates resource leak risks on physical hardware.

---

### Criterion 4: Telemetry, Observability & Base Station Visibility

This is the most critical area where Approach B presents a valid advantage.

Look at lines 159 through 165 of [`src/handlers/StateMachineHandler.cpp`](file:///c:/Users/ltkli/Documents/GitHub/Autonomy_Software/src/handlers/StateMachineHandler.cpp#L159-L165):
```cpp
rovecomm::RoveCommPacket<uint8_t> stPacket;
stPacket.unDataId    = manifest::Autonomy::TELEMETRY.find("CURRENTSTATE")->second.DATA_ID;
stPacket.unDataCount = manifest::Autonomy::TELEMETRY.find("CURRENTSTATE")->second.DATA_COUNT;
stPacket.eDataType   = manifest::Autonomy::TELEMETRY.find("CURRENTSTATE")->second.DATA_TYPE;
stPacket.vData.emplace_back(static_cast<uint8_t>(this->GetCurrentState()));
network::g_pRoveCommUDPNode->SendUDPPacket(stPacket, "0.0.0.0", constants::ROVECOMM_OUTGOING_UDP_PORT);
```

#### Under Approach B (Flat Top-Level States):
* **Native Base Station Visibility:** Every time the rover transitions from `Follow` to `Stay`, `Fetch`, `Come`, or `Give`, `StateMachineHandler` automatically sends a `CURRENTSTATE` packet over RoveComm.
* The Autonomy GUI, the web visualizer, and the Multimedia Board LED display update immediately without a single line of new networking code.
* Operators in the base station command trailer see exact text: `"Executing Fetch"` or `"Executing Give"`.

#### Under Approach A (Hierarchical):
* **The "Black Box" Problem:** To `StateMachineHandler`, the rover remains in `eAstronautAssistance` (e.g., State ID 2) for the entire 15-minute mission.
* Unless extra code is written, base station operators and drive judges have zero visibility into whether the rover is currently following, searching for the pick, or waiting for a button press.

#### How to Solve This in Approach A (The 5-Line Ponytail Fix):
The observability drawback in Approach A is completely eliminated by having `AstronautMissionController` emit a dedicated RoveComm sub-state telemetry packet whenever its internal step changes:

```cpp
void AstronautMissionController::TransitionTo(AstronautSubState eNewSubState) {
    m_eCurrentSubState = eNewSubState;
    
    // Broadcast sub-state telemetry (Data ID 11022: ASTRONAUT_SUBSTATE)
    rovecomm::RoveCommPacket<uint8_t> stPacket;
    stPacket.unDataId = manifest::Autonomy::TELEMETRY.find("ASTRONAUT_SUBSTATE")->second.DATA_ID;
    stPacket.vData    = {static_cast<uint8_t>(m_eCurrentSubState)};
    network::g_pRoveCommUDPNode->SendUDPPacket(stPacket, "0.0.0.0", constants::ROVECOMM_OUTGOING_UDP_PORT);
}
```

> **Verdict on Telemetry:** **Approach B is natively simpler for the GUI, but Approach A solves it completely with one small packet emission.**

---

### Criterion 5: Developer Ergonomics, Testing & Student Onboarding

Our team principles emphasize that building confident student engineers is our primary goal. How does each approach affect student developers who must write, test, and debug this code?

#### Testing Approach A (Hierarchical):
* **Headless Unit Testing in Catch2:** `AstronautMissionController` is a self-contained C++ class. A student can write an automated unit test without launching the full autonomy engine:
  ```cpp
  TEST_CASE("AstronautMissionController: Step Sequence Verification") {
      MockSensors sensors;
      AstronautMissionController controller(&sensors);
      
      // Inject Follow Button
      controller.HandleCommand(AstronautCommand::eFollow);
      CHECK(controller.GetSubState() == AstronautSubState::Sub_Following);
      
      // Simulate 2.5m Standoff
      sensors.SetSuitDistance(2.4);
      controller.Update();
      CHECK(controller.GetSubState() == AstronautSubState::Sub_AwaitingStay);
  }
  ```
* Students can test gesture recognition timing, button debouncing, and winch state logic entirely on their laptops in seconds without mocking `LiDARHandler`, `GeoPlanner`, or `StateMachineHandler`.

#### Testing Approach B (Flat Top-Level States):
* Testing requires instantiating `StateMachineHandler`, which starts background threads, binds UDP network ports, and queries camera singletons.
* To test if `Fetch!` transitions to `Come!`, the developer must mock the entire global state machine and its waypoint queue.

> **Verdict on Testing & Ergonomics:** **Approach A is vastly easier to test and debug.** It allows a subteam pair to work in an isolated sandbox without risking breaking global navigation.

---

### Criterion 6: Task Re-ordering & Competition Penalties (The Rules Reality)

What happens if an unexpected event occurs on the competition course in Hanksville?
* *Scenario 1:* The rover fails to grasp the rock pick during `Fetch!` after two attempts. The team decides to take a 5-minute penalty and skip the pick task, moving directly to `Come!`.
* *Scenario 2:* The astronaut presses the `Stay!` button while the rover is mid-transit during `Follow!`.

#### Under Approach A (Hierarchical):
* `AstronautMissionController` maintains its own internal transition rules.
* If a skip command or preemption packet arrives from the RIM module, the controller handles it locally. It can easily jump from `Sub_ExecutingFetch` to `Sub_ExecutingCome`.
* The global state machine never knows or cares that an internal step was skipped.

#### Under Approach B (Flat Top-Level States):
* Skipping a task requires adding cross-cutting transition edges between non-adjacent states in `StateMachineHandler`.
* `eAstronautFetch` must now have explicit transition branches not only to `eAstronautCome`, but back to `eAstronautStay`, to `eIdle`, to `eReversing`, and to `eNavigating`. The transition graph becomes a tangled web.

---

## 4. Summary Scorecard

| Architectural Dimension | Approach A: Sub-Mission Controller | Approach B: Flat Top-Level States | Approach C: Flat States + Base Class |
| :--- | :---: | :---: | :---: |
| **Waypoint Queue Consistency** | **5 / 5** (1 leg = 1 destination) | **1 / 5** (Breaks queue semantics) | **1 / 5** (Breaks queue semantics) |
| **Global State Machine Cleanliness** | **5 / 5** (Zero state bloat) | **2 / 5** (Expands to 14 states) | **2 / 5** (Expands to 14 states) |
| **Hardware RAII (Winch / Downward Cam)** | **5 / 5** (Single Start/Exit scope) | **1 / 5** (Fractured across 5 classes) | **3 / 5** (Shared base destructor) |
| **Native Base Station Visibility** | **3 / 5** (Requires sub-state packet) | **5 / 5** (Native `CURRENTSTATE` enum) | **5 / 5** (Native `CURRENTSTATE` enum) |
| **Headless Unit Testability (Catch2)** | **5 / 5** (Testable in isolation) | **2 / 5** (Coupled to global handler) | **2 / 5** (Coupled to global handler) |
| **Code Boilerplate & Maintenance** | **5 / 5** (2 files: state + controller) | **1 / 5** (10 separate class files) | **2 / 5** (11 separate class files) |
| **Failsafe Unwinding (`eStuck` / `eReverse`)** | **5 / 5** (Clean recursive recall) | **2 / 5** (5 restoration edge cases) | **3 / 5** (Base class restoration) |
| **TOTAL SCORE (out of 35)** | **33 / 35** | **14 / 35** | **18 / 35** |

---

## 5. Final Recommendation & Implementation Path

### The Verdict: Approach A (Sub-Mission Controller) is the Correct Architecture

The trade study provides conclusive evidence that **Approach A** is the superior engineering choice:
1. It maintains the integrity of `WaypointHandler`, treating the astronaut staging area as a single geographic destination.
2. It encapsulates specialized hardware (winch, gripper, downward camera) within a single RAII lifecycle, eliminating hardware lockups on abort.
3. It keeps the global navigation state machine lightweight (9 clean states instead of 14 bloated states).
4. It enables rapid student testing in Catch2 without running the full autonomy binary.

### How to Address the Only Legitimate Drawback (Base Station Visibility)
The only drawback of Approach A is the loss of granular `CURRENTSTATE` telemetry. We solve this with ponytail simplicity:
1. Add a single sub-state telemetry packet: `manifest::Autonomy::TELEMETRY::ASTRONAUT_SUBSTATE` (Data ID 11022).
2. Have `AstronautMissionController` emit this byte whenever its internal phase advances.
3. In the base station visualizer GUI, if `CURRENTSTATE == eAstronautAssistance`, display the string corresponding to `ASTRONAUT_SUBSTATE`.