\dir src

# SRC Directory

The **src** directory is the main source code directory of the Rover project. It contains several subdirectories that organize the different components and functionalities of the project.

## Directory Structure

The **src** directory is structured as follows:

- **algorithms**: This directory contains the C++ files related to the algorithms used in the Rover project. Refer to the [algorithms/README.md](algorithms/README.md) for more details on organizing files within this directory.

- **drivers**: This directory contains the C++ files related to physical systems, boards, microcontrollers, or sensors used in the Rover project. Refer to the [drivers/README.md](drivers/README.md) for more details on organizing files within this directory.

- **interfaces**: This directory stores files representing the abstract classes or interfaces that define the contract for specific functionalities or modules within the project. Refer to the [interfaces/README.md](interfaces/README.md) for more details on organizing files within this directory.

- **states**: This directory contains the C++ files related to the state machine code of the Rover project. Refer to the [states/README.md](states/README.md) for more details on organizing files within this directory.

- **threads**: This directory stores files representing the main classes of different modules, each of which will spawn its own thread for parallel execution. Refer to the [threads/README.md](threads/README.md) for more details on organizing files within this directory.

- **util**: This directory contains utility scripts and helper functions used in the Rover project. Refer to the [util/README.md](util/README.md) for more details on organizing files within this directory.

- **vision**: This directory contains the C++ files related to vision processing or computer vision algorithms used in the Rover project. Refer to the [vision/README.md](vision/README.md) for more details on organizing files within this directory.

Each subdirectory within **src** may have its own specific file organization and guidelines as described in their respective README files.

## Usage

The **src** directory is where the core source code of the Rover project resides. It contains the implementation of algorithms, interfaces, states, utilities, and vision-related functionality.

Developers should regularly update and maintain the code within this directory, following the guidelines provided in the subdirectory README files.

Make sure to adhere to good software development practices, such as modularization, code readability, documentation, and version control, to ensure the maintainability and scalability of the Rover project.

Happy coding and developing within the **src** directory!