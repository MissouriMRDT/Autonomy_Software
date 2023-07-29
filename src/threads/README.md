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

For example, if there is a main class responsible for managing the states of the rover, create a file named `StateMachineThread.cpp`. Similarly, if there is a main class handling vision processing, name the file as `VisionThread.py`.

Make sure to update this README file whenever new files or modules requiring dedicated threads are added to the **Threads** directory, providing a brief summary of each class's purpose and functionality.

Remember to integrate the thread-related classes with other components of the project to ensure proper coordination and synchronization between threads.

## Considerations

When working with multithreading, keep in mind the following considerations:

- Pay attention to thread safety and avoid race conditions by properly synchronizing access to shared resources.
- Take advantage of thread-safe data structures and synchronization primitives provided by the chosen programming language or libraries.
- Implement error handling and graceful shutdown mechanisms to handle exceptions and ensure proper termination of threads.
- Consider performance implications, such as potential bottlenecks or resource contention, when designing the multithreading architecture.

The `AutonomyThread` interface will attempt to make the process of running the code for your subsystem eas by handling most of the shared memory and scheduling. Be diligent in testing and debugging the multithreaded code to ensure the desired behavior and avoid potential issues like deadlocks or data corruption.

Happy coding and threading!