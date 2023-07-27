\dir src/interfaces

# Interfaces Directory

The **Interfaces** directory within the **src** directory is dedicated to storing C++ abstract classes or interfaces that define the contract for other classes in the Rover project. This directory focuses on providing a set of common methods and properties that classes can inherit and implement for the sake of organization and consistency.

## Directory Structure

The **Interfaces** directory stores C++ files representing the abstract classes or interfaces that define the contract for specific functionalities or modules within the project. Each file within this directory corresponds to a specific interface or abstract class.

## Usage

Here's a general guideline for organizing files within the **Interfaces** directory:

- Create a separate file for each abstract class or interface, defining the common methods and properties that other classes will inherit.
- Use descriptive names for the files that reflect the purpose or functionality of each interface or abstract class.
- Ensure that the files are properly documented with comments explaining the contract, expected behavior, and any relevant details.
- Think hard about what common methods you want to define and the different ways they could be used across the codebase. (maximize reusability)

For example, if there is an abstract class defining the common behavior for threads in the **Threads** directory, create a file named `AutonomyThread.h` within the **interfaces** directory. Similarly, if there is an interface defining the contract for state classes, name the file as `State.h`.

Make sure to update this README file whenever new interface or abstract class files are added to the **Interfaces** directory, providing a brief summary of each interface's purpose and the classes that inherit from it.

Remember to have the classes within other directories inherit these interfaces or abstract classes to enforce adherence to the defined contract and ensure a consistent and structured implementation across the project.

<<<<<<< HEAD
## Considerations
=======
1. **Motor control**: Files that interface with motor controllers or drivers, providing control signals to the Rover's motors.
2. **Sensors**: Files that handle interactions with various sensors like proximity sensors, temperature sensors, or GPS modules.
3. **Microcontrollers**: Files that communicate with microcontrollers on the Rover, such as Arduino or Raspberry Pi boards, to exchange data or control signals.
4. **Communication protocols**: Files that implement communication protocols like UART, I2C, SPI, or CAN bus to interface with external devices or modules.
5. **Camera interfaces**: Files that manage image acquisition, processing, or streaming from cameras or vision systems used in the Rover project. (Other than USB cameras)
6. **Power management**: Files that control power distribution, battery monitoring, or charging systems on the Rover.
>>>>>>> origin/development

When working with interfaces and abstract classes, keep the following considerations in mind:

- Ensure that the interface or abstract class provides a clear contract and defines the expected behavior that inheriting classes must implement.
- Design the interfaces to be flexible and extensible, allowing for different implementations to satisfy the contract.
- Use inheritance and polymorphism to achieve code reuse and promote modularity in the project.
- Regularly review and update the interfaces or abstract classes as the project evolves to accommodate new requirements or changes in functionality.

<<<<<<< HEAD
By utilizing interfaces and abstract classes, you can establish a consistent structure, encourage code modularity, and facilitate the implementation of specific functionalities across the rover project.
=======
Ensure that the interface files are properly integrated with other components of the project to ensure seamless communication and control between the Rover and its physical systems.
>>>>>>> origin/development

Happy coding and designing interfaces!