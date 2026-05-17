# State Machine Handler

The `StateMachineHandler` is the core engine of the autonomy software. It dictates the high-level logic and orchestrates the transitions between different states (e.g., `NavigatingState`, `SearchPatternState`, etc.).

## Primary Responsibilities
1. **State Execution**: It holds the currently active `State` object (from `src/states/`) and repeatedly calls its `Run()` method.
2. **Transition Management**: It processes `Event` triggers (like `eMarkerSeen` or `eReachedGpsCoordinate`) and safely swaps out the `m_pCurrentState` for the appropriate next state.
3. **Data Polling & Failsafes**: Before running the current state, it polls the `NavigationBoard` for the latest GPS location, checks the battery voltage, and verifies that the ZED camera odometry hasn't drifted too far from the absolute GPS coordinate.

## Architecture & Threading
- **`AutonomyThread`**: The handler inherits from `AutonomyThread` and runs continuously in the background at a rate capped by `STATEMACHINE_MAX_IPS` (usually 60 Hz).
- **Polymorphism**: The handler maintains a map of pre-instantiated state objects (`std::unordered_map<statemachine::States, std::shared_ptr<statemachine::State>>`). This prevents dynamic memory allocation overhead when switching states. All states inherit from a base `State` interface, allowing the handler to blindly call `m_pCurrentState->Run()` without needing to know *which* state is active.
- **Thread Safety**: Uses `std::shared_mutex` (`m_muStateMutex`) to ensure that one thread cannot request a state change while the main thread is in the middle of executing the current state's logic.

## Usage
Called in `main.cpp`:
```cpp
globals::g_pStateMachineHandler->StartStateMachine();
```
Whenever an objective is met inside a state (e.g., `NavigatingState` calculates it is within the goal radius), that state requests a transition:
```cpp
globals::g_pStateMachineHandler->HandleEvent(statemachine::Event::eReachedGpsCoordinate);
```
