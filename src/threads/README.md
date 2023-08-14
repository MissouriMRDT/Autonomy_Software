\dir src/threads

# Threads Directory

The **Threads** directory is dedicated to storing C++ files related to multithreading in the rover project. This directory focuses on managing the main classes of states, vision, and other modules that require their own dedicated threads for parallel execution.

## Directory Structure

The **Threads** directory stores files representing the main classes of different modules, each of which will spawn its own thread for parallel execution. The structure within the directory will typically resemble the organization of the corresponding modules, such as states, vision, and other relevant components.

## Usage

Here's a general guideline for organizing files within the **Threads** directory:

- Create a separate file for each main class that requires its own dedicated thread.
- Use descriptive names for the files that reflect the purpose or functionality of each class.
- Inherit the `AutonomyThread` abstract class in the interfaces folder, to ensure the class you want to thread will integrate with the rest of the codebase.
- Ensure that the files are properly documented with comments explaining the class's role, its interaction with other modules, and any relevant details.
- Implement the necessary thread management mechanisms within each class, adhering to best practices for multithreading.

### Detailed Walkthrough

#### Step 0: Do I really need this to run in a new thread?
Before continuing, ask yourself if the code you've written will actually benefit from being ran outside of the main program:
- **_Is your code for a new subsystem that hasn't aleady been implemented?_** 
- **_Will your code actually benefit from running in a seperate thread; is the code parallelizable or does it wait/depend on other external code running first?_**
- **_What kind of system or program resources does my code access?_**

All these question are extremely valid when you are writing code for a new system that you think could benefit from multithreading. Sharing memory between threads is not generally something that you want to do, so plan carefully when designing your class. Have the main class do all of the resource locking by creating getters and/or setters for each piece of data that you want to be accesible from outside of your threaded code. Since the `AutonomyThread` interface was designed so that only a single special method is ran in a different thread, resource handling should be relative simple if you are familiar with the different methods of mutexes and locking in C++. It will also be useful to know how to correctly use atomics in C++, as these let you freely share certain resources between threads without needed to manually lock and unlock them.

**NOTE: Look in the `examples/threading` directory to see example implementations of what this guide discusses.**

#### Step 1: Create the class and inherit the interface.
Create a new file in the `threads` directory and name it appropriately. For example, if there is a main class responsible for managing the states of the rover, create a file named `StateMachineThread.cpp`. Similarly, if there is a main class handling vision processing, name the file `VisionThread.py`. Then, create a class with the same name as the file and inherit the `AutonomyThread` interface and define the template type like shown below:
```
#include "../../src/interfaces/AutonomyThread.hpp"

/******************************************************************************
 * @brief This class inherits the AutonomyThread interface so that it can inherit
 * and implement its base methods. This allows the programmer to easily thread code
 * put in a specific function and it also gives them the ability to create a thread pool
 * of subroutines and parallelize loops.
 *
 *
 * @author ClayJay3 (claytonraycowen@gmail.com)
 * @date 2023-07-27
 ******************************************************************************/
class VisionThread : public AutonomyThread<void>    // <-- The '<void>' part lets you define what can be returned from the thread pool.
``` 

#### Step 2: Implement the inherited methods.
The `AutonomyThread` base class contains some [virtual](https://www.geeksforgeeks.org/virtual-function-cpp/) and [pure virtual](https://www.geeksforgeeks.org/pure-virtual-functions-and-abstract-classes/#) methods that your new child class should inherit.

Remember to integrate the thread-related classes with other components of the project to ensure proper coordination and synchronization between threads.

### Mutexes and Locks
When working with different mutex types in C++20, it's important to understand how locks can be acquired to ensure proper synchronization and avoid potential issues like deadlocks. Here's a detailed explanation of the types of locks that can be acquired for each mutex type:

### 1. `std::mutex`

- **Lock Type:** Exclusive Lock
- **Acquisition:** A thread can acquire an exclusive lock on a `std::mutex` using the `std::unique_lock` class or by directly calling the `lock()` member function on the mutex.

- **Use Case:** Use an exclusive lock when you need to ensure that only one thread can access a critical section of code or a shared resource at a time.

### 2. `std::timed_mutex`

- **Lock Type:** Exclusive Lock with Timeout
- **Acquisition:** A thread can attempt to acquire an exclusive lock on a `std::timed_mutex` using the `std::unique_lock` class and providing a timeout value when calling the `try_lock_for()` or `try_lock_until()` member functions.

- **Use Case:** Use an exclusive lock with timeout when you want to prevent indefinite waiting for a lock and need to decide on an alternative action if the lock cannot be acquired within a specified time.

### 3. `std::recursive_mutex`

- **Lock Type:** Exclusive Lock (Recursive)
- **Acquisition:** Similar to a `std::mutex`, a thread can acquire an exclusive lock on a `std::recursive_mutex` using the `std::unique_lock` class or by directly calling the `lock()` member function on the mutex. The same thread can acquire the lock multiple times without causing a deadlock.

- **Use Case:** Use a recursive lock when you have a nested structure of function calls that require the same mutex protection, and you want to avoid deadlocks caused by recursive locking.

### 4. `std::recursive_timed_mutex`

- **Lock Type:** Exclusive Lock (Recursive) with Timeout
- **Acquisition:** Similar to a `std::timed_mutex`, a thread can attempt to acquire an exclusive lock on a `std::recursive_timed_mutex` using the `std::unique_lock` class and specifying a timeout value with `try_lock_for()` or `try_lock_until()`.

- **Use Case:** Use a recursive lock with timeout when you need both the ability to recursively lock and a mechanism to avoid indefinite waiting for the lock.

### 5. `std::shared_mutex`

- **Lock Type:** Shared Lock (Read Lock), Exclusive Lock (Write Lock)
- **Acquisition:** A thread can acquire a shared lock (read lock) on a `std::shared_mutex` using the `std::shared_lock` class. To acquire an exclusive lock (write lock), use the `std::unique_lock` class or call the `lock()` member function directly.

- **Use Case:** Use a shared lock when you have a resource that is frequently read by multiple threads. Use an exclusive lock when you need to modify the resource to ensure exclusive access.

### 6. `std::shared_timed_mutex`

- **Lock Type:** Shared Lock (Read Lock), Exclusive Lock (Write Lock) with Timeout
- **Acquisition:** Similar to `std::shared_mutex`, a thread can acquire shared and exclusive locks using `std::shared_lock` and `std::unique_lock`, respectively. Additionally, you can use the `try_lock_shared_for()` and `try_lock_shared_until()` functions to attempt to acquire a shared lock with a timeout.

- **Use Case:** Use a shared timed lock when you need to read a resource concurrently but want to avoid indefinite waiting. Use an exclusive timed lock when you need to write with a timeout.

Always choose the appropriate type of lock based on your application's requirements and concurrency patterns to ensure efficient and safe synchronization of shared resources.

## Final Considerations

When working with multithreading, keep in mind the following considerations:

- Pay attention to thread safety and avoid race conditions by properly synchronizing access to shared resources.
- Take advantage of thread-safe data structures and synchronization primitives provided by the chosen programming language or libraries.
- Implement error handling and graceful shutdown mechanisms to handle exceptions and ensure proper termination of threads.
- Consider performance implications, such as potential bottlenecks or resource contention, when designing the multithreading architecture.

The `AutonomyThread` interface will attempt to make the process of running the code for your subsystem eas by handling most of the shared memory and scheduling. Be diligent in testing and debugging the multithreaded code to ensure the desired behavior and avoid potential issues like deadlocks or data corruption.

Happy coding and threading!