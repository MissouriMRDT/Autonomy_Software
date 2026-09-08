# Thread Pools

While `AutonomyThread` oversees single persistent background worker loops, the Autonomy Software also requires mechanisms to execute parallelized burst workloads across multiple CPU cores without thread allocation latency.

---

## 1. Engine: `BS::thread_pool`

The codebase utilizes the **Barak Shoshany C++ Thread Pool library** (`BS::thread_pool`), included under `external/threadpool/include/BS_thread_pool.hpp`.

### Benefits Over Dynamic Thread Creation
- **Pre-Allocated Worker Threads**: Spawns worker threads once during object initialization.
- **Zero OS Thread Creation Overhead**: Tasks are submitted into a concurrent priority queue, and idle workers immediately claim and execute them.
- **Priority Scheduling**: Tasks can be assigned priorities (`BS::pr::lowest` to `BS::pr::highest`) to ensure time-critical computations bypass routine jobs.

---

## 2. Integrated Pool Methods in `AutonomyThread`

Every class derived from `AutonomyThread<T>` contains an embedded `BS::thread_pool`. It exposes several protected methods for dispatching parallel work:

### A. Batch Execution
- **`RunPool(int nTasks, int nThreads, AutonomyThreadPriority ePriority)`**: Submits `nTasks` to execute `PooledLinearCode()`. It returns a vector of `std::future<T>`, allowing the caller to collect return values via `GetPoolResults()`.
- **`RunDetachedPool(int nTasks, int nThreads, AutonomyThreadPriority ePriority)`**: Submits `nTasks` as fire-and-forget executions, bypassing future synchronization for minimal latency.

### B. Dynamic Task Submission
- **`SubmitTaskToPool(Func&& task, Args&&... args)`**: Queues an arbitrary lambda or function pointer to the pool, returning an `std::future` representing its eventual completion.
- **`SubmitDetachedTaskToPool(Func&& task, Args&&... args)`**: Queues an arbitrary function without allocating a future object.

### C. Loop Parallelization (`ParallelizeLoop`)
Splits large iterative loops across available CPU cores:

```cpp
// Distributes 10,000 iterations across 4 worker threads
this->ParallelizeLoop(4, 10000, [this](const int nStart, const int nEnd) {
    for (int i = nStart; i < nEnd; ++i)
    {
        ProcessDataPoint(i);
    }
});
```

---

## 3. Real-World Application: Multi-Subscriber Frame Copying

The primary consumer of thread pooling in the software is camera buffer distribution (`ZEDCam.cpp` and `BasicCam.cpp`):
1. A physical camera frame is captured on the camera capture thread.
2. Multiple consumer threads (`TagDetector`, `ObjectDetector`, `SimpleWebServer` video streamer) require independent copies of the frame matrix.
3. If the camera thread copied frames sequentially, a slow consumer would block subsequent hardware frame grabs.
4. Instead, the camera pushes a copy task for each active subscriber into its thread pool. Workers execute the matrix copies simultaneously in parallel, allowing the hardware capture loop to immediately fetch the next frame.
