# AutonomyThread Interface

The `AutonomyThread` template interface (`src/interfaces/AutonomyThread.hpp`) is the foundational concurrency abstraction across the Autonomy Software. It provides standardized thread lifecycle management, rate-limiting, performance instrumentation, and integrated thread-pooling without exposing raw OS thread primitives to derived classes.

---

## 1. Architectural Purpose

Most autonomy modules (cameras, neural network detectors, the state machine, network listeners) require dedicated background loops. Executing these on the main thread would cause catastrophic latency spikes.

Inheriting from `AutonomyThread<T>` provides:
1. **Managed Background Execution**: Safely spawns and oversees an independent OS thread executing `ThreadedContinuousCode()`.
2. **Deterministic IPS Limiting**: Regulates loop execution frequency via high-precision sleep calculations.
3. **Execution Metrics**: Instruments real-time iterations-per-second (`IPS`) tracking.
4. **Internal Thread Pooling**: Embeds a `BS::thread_pool` for executing parallelized burst tasks (`PooledLinearCode()`).
5. **Thread Prioritization**: Wraps thread scheduling priorities (`AutonomyThreadPriority`) from lowest to highest.
6. **Destructor Safety**: Enforces clean thread joining upon destruction, preventing segmentation faults from orphaned threads during application shutdown.

---

## 2. Core Methods and Overrides

### A. Pure Virtual Worker Methods
- **`virtual void ThreadedContinuousCode() = 0`**: The payload of the continuous loop. Runs inside a `while(!m_bStopThreads)` loop on the background thread.
- **`virtual void PooledLinearCode() = 0`**: The payload executed by tasks dispatched to the embedded thread pool.

### B. Lifecycle and Control
- **`Start()`**: Spawns the worker thread, transitions state to `AutonomyThreadState::eRunning`, and begins loop execution.
- **`RequestStop()`**: Atomically sets `m_bStopThreads = true` and updates state to `eStopping`. The current iteration will complete before the thread terminates.
- **`Join()`**: Blocks the calling thread until the worker thread has completely exited.
- **`SetMainThreadIPSLimit(int nMaxIterationsPerSecond)`**: Configures the rate ceiling. After each iteration of `ThreadedContinuousCode()`, the elapsed time is measured; if execution finished ahead of the timestep, the thread sleeps for the remainder of the slice.
- **`GetIPS().GetExactIPS()`**: Returns the moving-average iterations per second.

---

## 3. Thread Priority System (`AutonomyThreadPriority`)

The interface abstracts thread scheduling priority:

```cpp
enum class AutonomyThreadPriority
{
    eLowest  = BS::pr::lowest,   // Scheduled less frequently; yields to other tasks
    eLow     = BS::pr::low,
    eNormal  = BS::pr::normal,    // Default priority
    eHigh    = BS::pr::high,     // Prioritized under system load
    eHighest = BS::pr::highest   // Highest scheduling priority
};
```

---

## 4. Usage Example

```cpp
// Template parameter defines the return type of pooled tasks (void if unused)
class SensorWatcher : public AutonomyThread<void>
{
public:
    SensorWatcher()
    {
        // Regulate continuous polling to 20 Hz
        SetMainThreadIPSLimit(20);
    }

protected:
    void ThreadedContinuousCode() override
    {
        // Runs continuously on worker thread
        ReadHardware();
        ProcessTelemetry();
    }

    void PooledLinearCode() override
    {
        // Executed by thread pool workers when dispatched
    }
};

// Application usage (in main.cpp)
SensorWatcher watcher;
watcher.Start();

// Shutdown sequence
watcher.RequestStop();
watcher.Join();
```
