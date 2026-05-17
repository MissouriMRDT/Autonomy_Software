# Thread Pools

While `AutonomyThread` handles single continuous background loops, the autonomy software also frequently needs to execute highly parallelized "burst" workloads. For example:
- A single camera frame arrives, and we need to copy it into 5 different memory buffers for 5 different detector threads.
- A search pattern needs to quickly calculate 100 different ray-casts against a LiDAR point cloud.

To handle these scenarios, the codebase heavily utilizes **Thread Pools**.

## The Implementation: `BS::thread_pool`
Instead of manually spawning `std::thread` objects every time a parallel task is needed (which incurs massive OS overhead for thread creation/destruction), we use the lightweight, modern C++ library `BS::thread_pool` (Barak Shoshany).

A Thread Pool creates a fixed number of threads *once* at startup. These threads sit idle until a "task" is pushed to the pool's queue. The threads wake up, execute the tasks as fast as possible in parallel, and go back to sleep.

## Thread Pools inside `AutonomyThread`

The `AutonomyThread` interface doesn't just manage one main loop thread; it actually instantiates a `BS::thread_pool` internally specifically for parallelizing tasks *within* your class.

### 1. `PooledLinearCode()`
This is the second pure virtual method in the interface (alongside `ThreadedContinuousCode()`). It is designed to be the payload for parallel execution.

### 2. `RunPool()` vs `RunDetachedPool()`
Inside your continuous loop, you can call these protected methods to dispatch work to your internal pool:
- **`RunPool(int nTasks, int nThreads)`**: Queues `nTasks` to run `PooledLinearCode()`. It returns `std::future` objects so you can retrieve results (via `GetPoolResults()`).
- **`RunDetachedPool(int nTasks, int nThreads)`**: Queues the tasks but ignores the return values. This is slightly faster because it doesn't bother managing `std::future` objects. It is a "fire and forget" method.

### 3. Loop Parallelization (`ParallelizeLoop`)
If you have a massive `for` loop that is chewing up CPU time, you can use the `ParallelizeLoop` utility provided by `AutonomyThread`:

```cpp
// Instead of this:
for(int i = 0; i < 10000; i++) {
    ProcessPixel(i);
}

// Do this:
this->ParallelizeLoop(4, 10000, [this](const int start, const int end) {
    for(int i = start; i < end; i++) {
        ProcessPixel(i);
    }
});
```
This automatically divides the 10,000 iterations into 4 chunks and processes them simultaneously across 4 threads.

## Example: Camera Frame Copying
The most prominent use of Thread Pools in the codebase is in `ZEDCam.cpp` and `BasicCam.cpp`.
When a camera reads a frame from the hardware, it needs to provide that frame to the Object Detector, the Tag Detector, and the UI Streamer.

If one of those detectors is lagging, a simple mutex lock would cause the camera thread to freeze, dropping frames. Instead, the Camera class queues a `PooledLinearCode` task for *every active subscriber*. The internal `BS::thread_pool` wakes up and copies the OpenCV matrix into the subscriber's buffer simultaneously. This ensures the camera is instantly ready to pull the next frame from the hardware without blocking.

*Note: You configure the size of these pools in `AutonomyConstants.cpp` using variables like `ZED_MAINCAM_FRAME_RETRIEVAL_THREADS`.*
