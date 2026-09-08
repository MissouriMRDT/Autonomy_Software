# State Machine Handler

The `StateMachineHandler` (`src/handlers/StateMachineHandler.h` & `StateMachineHandler.cpp`) manages active state execution, evaluates event-driven transitions, coordinates recovery fallbacks, and executes sensor fusion for the Autonomy Software.

---

## 1. Primary Responsibilities

1. **State Lifecycle Execution**: Instantiates, runs, and terminates concrete `statemachine::State` objects derived from `src/interfaces/State.hpp`.
2. **Event Dispatch and Transitions**: Handles incoming `statemachine::Event` triggers, halts motor outputs for safety, queries the active state for the subsequent state, and manages transitions.
3. **State Preservation and Recall**: Maintains `m_umSavedStates` (`std::unordered_map<statemachine::States, std::shared_ptr<statemachine::State>>`). When an interruption occurs (such as getting stuck or initiating a reverse maneuver), the active state can be saved and restored once recovery completes.
4. **Sensor Fusion Engine**: Implements `SmartRetrieveRoverPose()`, combining low-rate absolute GPS/magnetometer heading from `NavigationBoard` with high-rate visual-inertial odometry from `ZEDCamera`.
5. **RoveComm Telemetry**: Broadcasts `manifest::Autonomy::TELEMETRY["CURRENTSTATE"]` UDP packets on every state change to keep the Basestation GUI synchronized.

---

## 2. Event Handling and State Transition Sequence

When a state transition is commanded via `HandleEvent(statemachine::Event eEvent, bool bSaveCurrentState)`:

```
[Event Triggered] (HandleEvent called)
        |
        v
[Lock Event Mutex] (m_muEventMutex)
        |
        v
[Safety Stop Command] (globals::g_pDriveBoard->SendStop())
        |
        v
[Query Active State] (eNextState = m_pCurrentState->TriggerEvent(eEvent))
        |
        v
[ChangeState(eNextState, bSaveCurrentState)]
        |
        +---> [Lock State Mutex] (m_muStateMutex)
        +---> [Set Switching Flag] (m_bSwitchingStates = true)
        +---> [Save Current State if requested] (m_umSavedStates[state] = m_pCurrentState)
        +---> [Check Saved States Map]
        |        |
        |        +--> Found: Restore preserved state object
        |        +--> Not Found: Create fresh state via CreateState()
        |
        +---> [Clear Switching Flag] (m_bSwitchingStates = false)
        +---> [Broadcast Telemetry] (RoveComm CURRENTSTATE packet)
```

---

## 3. Sensor Fusion: `SmartRetrieveRoverPose()`

Compass magnetometers on electric rovers are susceptible to magnetic interference caused by high motor currents. Conversely, pure visual odometry drifts over time. `SmartRetrieveRoverPose()` fuses both sources:

1. **Direct GPS and Magnetometer Ingestion**: Reads latitude, longitude, and compass heading from `globals::g_pNavigationBoard`.
2. **Dynamic Realignment Conditions**:
   When the rover is in `eIdle`, or when driving straight at steady speed ($|v| > \text{constants::ZED\_REALIGN\_VEL\_THRESH}$ and $|\omega| < \text{constants::ZED\_REALIGN\_ROT\_THRESH}$):
   - Queries ZED IMU Euler yaw via `m_pMainCam->RequestSensorsCopy()`.
   - Computes offset:
     $$\text{Offset} = \text{Heading}_{\text{GPS}} - \text{Heading}_{\text{Raw ZED}}$$
3. **Fused Heading Output**:
   During turns or high motor throttle, the system applies the calibrated offset to high-frequency ZED IMU readings:
   $$\text{Heading}_{\text{Fused}} = (\text{Heading}_{\text{Raw ZED}} + \text{Offset}) \pmod{360}$$
   This eliminates heading jumps caused by magnetic spikes from drive motors.

---

## 4. Concurrency and Thread Safety

- **`AutonomyThread` Base**: Runs continuously in `ThreadedContinuousCode()` at a rate capped by `constants::STATEMACHINE_MAX_IPS` (default 60 Hz).
- **Double Mutex Protection**:
  - `m_muEventMutex`: Serializes event processing so multiple asynchronous events (e.g., vision detection and GPS arrival) do not race.
  - `m_muStateMutex`: Prevents state logic from executing while `m_pCurrentState` pointers are being swapped.
- **Atomic State Guard**: `m_bSwitchingStates` is set to true during transitions, preventing `ThreadedContinuousCode()` from executing methods on half-constructed state objects.

---

## 5. Public Interface Summary

```cpp
// Lifecycle
void StartStateMachine();
void StopStateMachine();

// Transitions
void HandleEvent(statemachine::Event eEvent, const bool bSaveCurrentState = false);

// State Inspection & Cache
statemachine::States GetCurrentState() const;
statemachine::States GetPreviousState() const;
void ClearSavedStates();
void ClearSavedState(statemachine::States eState);

// Pose & Telemetry Fusion
geoops::RoverPose SmartRetrieveRoverPose(bool bIMUHeading = true);
double SmartRetrieveVelocity();
double SmartRetrieveAngularVelocity();
void RealignZEDHeading(const double dNewActualHeading, const double dCurrentZEDHeading);
```
