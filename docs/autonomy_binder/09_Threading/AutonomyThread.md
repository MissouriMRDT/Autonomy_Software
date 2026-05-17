# AutonomyThread Interface

The `AutonomyThread` (`src/interfaces/AutonomyThread.hpp`) is a critical C++ template interface used extensively across the Autonomy Software. It provides a standardized, abstracted way to run background loops and manage thread states without forcing developers to manually manage `std::thread`, mutexes, and `std::atomic` stop flags in every class.

## Primary Purpose
Most handlers and detectors in the autonomy software (e.g., `CameraHandler`, `TagDetector`, `StateMachineHandler`) need to run a continuous loop (polling hardware, running neural networks, checking state conditions). If these loops ran on the main application thread, the software would block and freeze.

By inheriting from `AutonomyThread<void>`, a class instantly gains the ability to:
1. Run a continuous background loop safely.
2. Track its iterations per second (IPS).
3. Gracefully start, stop, and join the thread.
4. Spawns internal thread pools for hyper-parallelized sub-tasks.

## Core Concepts & Implementation

### 1. `ThreadedContinuousCode()`
This is a pure virtual method. Any class inheriting from `AutonomyThread` must implement this method.
- When you call `Start()` on the object, a new OS thread is spawned.
- This new thread runs a `while(!m_bStopThreads)` loop.
- Inside that loop, it repeatedly calls your `ThreadedContinuousCode()` implementation.

### 2. IPS (Iterations Per Second) Limiting
By default, the continuous loop will run as fast as the CPU allows. However, this often wastes CPU cycles (e.g., polling a 30 FPS camera at 1000 Hz).
- You can call `SetMainThreadIPSLimit(int nMaxIterationsPerSecond)`.
- The interface automatically measures the execution time of your `ThreadedContinuousCode()`. If it finishes faster than the required time slice (e.g., 33ms for 30 FPS), it puts the thread to sleep for the remaining time, freeing up CPU resources for other modules.
- You can check the actual runtime speed using `GetIPS().GetExactIPS()`.

### 3. Thread States (`AutonomyThreadState`)
The interface safely manages the lifecycle of the thread using atomics and condition variables, exposing the following states:
- `eStopped`: Thread is not running.
- `eStarting`: Thread has been commanded to start but the loop hasn't begun.
- `eRunning`: The main loop is actively calling `ThreadedContinuousCode()`.
- `eStopping`: A stop has been requested, waiting for the current iteration to finish.

### 4. Safe Stopping & Joining
- `RequestStop()`: Sets the atomic boolean `m_bStopThreads = true`. The loop will exit *after* the current `ThreadedContinuousCode()` iteration finishes.
- `Join()`: Blocks the calling thread until the loop completely exits.
- **Destructor Safety**: The `~AutonomyThread()` destructor automatically calls `RequestStop()` and `Join()`. This ensures that if the main program crashes or exits, background threads aren't orphaned, preventing nasty race conditions or segfaults during teardown.

## Example Usage

```cpp
class MySensor : public AutonomyThread<void>
{
public:
    MySensor()
    {
        // Limit this thread to poll the sensor at 10 Hz
        SetMainThreadIPSLimit(10);
    }

protected:
    void ThreadedContinuousCode() override
    {
        // This will be called repeatedly in the background!
        ReadPhysicalHardware();
        ProcessData();
    }
};

// In main.cpp
MySensor sensor;
sensor.Start(); // Spawns the thread, begins calling ThreadedContinuousCode()
// ... do other work ...
sensor.RequestStop();
sensor.Join();
```
