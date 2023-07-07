<details><summary></summary>
<p>
~Doxygen flag/marks~
  
\dir src/states
</p>
</details>

# States Directory

The **States** directory is intended to store C++ files related to the implementation of state machines in the rover project. This directory focuses on managing the different states and transitions within the project's logic.

## Guidelines

The following are the guidelines for organizing files within the **States** directory:

1. Each C++ file should represent a specific state machine or a set of related states and transitions.
2. Use descriptive names for the files that reflect the purpose or functionality of the state machine.
3. Ensure that the files are properly documented with comments explaining the states, transitions, and overall behavior of the state machine.
4. Include a README file in any subdirectories within the **States** directory to provide additional information if necessary.

## Usage

Here are some common types of files you might find in the **States** directory:

1. **Main state machine**: Files that define and manage the main state machine of the rover, encompassing the high-level behavior and control flow.
2. **Sub-state machines**: Files that implement sub-state machines or nested state machines for specific functionalities or modules of the rover.
3. **State definitions**: Files that define the different states and their associated properties or behaviors in the rover project.
4. **Transition handlers**: Files that handle the transitions between states, including the conditions, actions, and event triggers.
5. **Event handlers**: Files that manage the events or triggers that cause state transitions, such as user inputs, sensor readings, or timer-based events.

Feel free to create subdirectories within the **States** directory to further categorize the files based on the specific functionality or module they relate to.

Remember to update this README file whenever new state machines or states are added to the directory, providing a brief summary of each file's purpose and functionality.

Ensure that the state machine files are properly integrated with other components of the project to ensure cohesive control and behavior of the rover.

Happy coding and state management!