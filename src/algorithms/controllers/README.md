\dir src/algorithms/controllers

# Controllers Directory

The **Controllers** directory is dedicated to storing C++ files that define and implement advanced machine and motion controllers. Specifically, it contains files related to PID (Proportional-Integral-Derivative), LQR (Linear Quadratic Regulator), MPC (Model Predictive Controller), and the Stanley Controller.

## Purpose

The **Controllers** directory focuses on the logic and computational aspects of the project, providing implementations for a range of control algorithms and data structures.

## Guidelines

To maintain a well-organized directory, please adhere to the following guidelines:

1. Each C++ file should contain one or more implementations of control algorithms or data structures.
2. Use clear and descriptive names for the files, reflecting the purpose and algorithm implemented.
3. Ensure that the files are properly documented with comments explaining the algorithm's logic, input/output specifications, and any relevant details.
4. Include a README file in any subdirectories within the **Controllers** directory to provide additional information if necessary.

## Usage

In the **Controllers** directory, you can expect to find files related to the following categories:

1. **PID Controllers**: Files implementing the Proportional-Integral-Derivative controller, used for control loops in various applications.
2. **LQR Controllers**: Files that contain Linear Quadratic Regulator implementations, suitable for control system design and optimization.
3. **MPC Controllers**: Implementations of Model Predictive Controllers for advanced control in dynamic systems.
4. **Stanley Controller**: Files dedicated to the Stanley Controller for path tracking in autonomous vehicles.

Each controller should have a `.h` and a `.cpp` file, not `.hpp`.

Happy coding and control system development!
