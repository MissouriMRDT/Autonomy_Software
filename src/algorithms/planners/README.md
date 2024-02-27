\dir src/algorithms/planners

# Planners Directory

The **Planners** directory, located at `src/algorithms/planners`, is dedicated to storing C++ files that define and implement various path planning algorithms. This includes implementations for popular algorithms like A* (ASTAR), Dijkstra's algorithm, RRT (Rapidly-Exploring Random Trees), and more.

## Purpose

The **Planners** directory focuses on providing efficient and robust solutions for path planning in diverse scenarios. It encapsulates the logic and functionality of algorithms that guide autonomous systems, robots, or other entities through complex environments.

## Guidelines

To maintain a well-organized and comprehensible directory, please adhere to the following guidelines:

1. Each C++ header file should contain the declaration and implementation of a specific path planning algorithm.
2. Use clear and descriptive names for the files, reflecting the purpose and algorithm implemented (e.g., `AStar.h, AStar.cpp`).
3. Ensure that the header files are properly documented with comments explaining the algorithm's logic, input/output specifications, and any relevant details.
4. If a particular algorithm requires additional supporting files or data structures, consider organizing them into subdirectories within the **Planners** directory.
5. Each class should be placed in the planners namespace to improve organization and readability project wide.
6. The src/util/planners directory contains util classes and structs that can be used to help store or manipulate path data.
7. Include a README file in the **Planners** directory and any subdirectories if additional information or guidance is necessary.

## Usage

In the **Planners** directory, you can expect to find header files related to a variety of path planning algorithms, including but not limited to:

1. **A* (ASTAR)**: Header file implementing the A* algorithm, a widely used pathfinding algorithm known for its optimality and efficiency.
2. **Dijkstra's Algorithm**: Implementation of Dijkstra's algorithm for finding the shortest path in a graph.
3. **RRT (Rapidly-Exploring Random Trees)**: Header file providing the RRT algorithm for rapidly exploring and constructing feasible paths.
4. [Add more algorithms as needed]

Each algorithm should have its own dedicated `.hpp` or `.h, .cpp` file, and you are encouraged to document and comment your code thoroughly.

Happy coding and path planning development!
