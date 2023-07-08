<details><summary></summary>
<p>
~Doxygen flag/marks~
  
\dir src/interfaces
</p>
</details>

# Interfaces Directory

The **Interfaces** directory is intended to store C++ files related to the physical systems, boards, microcontrollers, or sensors used in the rover project. This directory focuses on the communication and interaction with these external interfaces.

## Guidelines

The following are the guidelines for organizing files within the **Interfaces** directory:

1. Each C++ file should represent a specific interface or device used in the rover project.
2. Use descriptive names for the files that reflect the purpose or functionality of the interface.
3. Ensure that the files are properly documented with comments explaining the interface's usage, input/output specifications, and any relevant details.
4. Include a README file in any subdirectories within the **Interfaces** directory to provide additional information if necessary.

## Usage

Here are some common types of files you might find in the **Interfaces** directory:

1. **Motor control**: Files that interface with motor controllers or drivers, providing control signals to the rover's motors.
2. **Sensors**: Files that handle interactions with various sensors like proximity sensors, temperature sensors, or GPS modules.
3. **Microcontrollers**: Files that communicate with microcontrollers on the rover, such as Arduino or Raspberry Pi boards, to exchange data or control signals.
4. **Communication protocols**: Files that implement communication protocols like UART, I2C, SPI, or CAN bus to interface with external devices or modules.
5. **Camera interfaces**: Files that manage image acquisition, processing, or streaming from cameras or vision systems used in the rover project. (Other than USB cameras)
6. **Power management**: Files that control power distribution, battery monitoring, or charging systems on the rover.

Feel free to create subdirectories within the **Interfaces** directory to further categorize the files based on the type of interface or functionality.

Remember to update this README file whenever new interfaces or devices are added to the directory, providing a brief summary of each file's purpose and functionality.

Ensure that the interface files are properly integrated with other components of the project to ensure seamless communication and control between the rover and its physical systems.

Happy coding and hardware interfacing!