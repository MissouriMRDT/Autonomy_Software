\page md_DEBUG Development Guide

# Debugging with CMake and Visual Studio Code

Debugging a C++ application in Visual Studio Code can be made seamless and efficient using the CMakeTools extension from Microsoft, which was automatically installed into the devcontainer from the Visual Studio Code Extensions Marketplace. This extension integrates CMake with Visual Studio Code, enabling developers to easily build and debug C++ applications directly within the VSCode environment.

## Setting up the Development Environment

1. Open this project in a devcontainer: First, install Microsoft's _Dev Containers_ extension from the marketplace. Then hit `CTRL + SHIFT + P` sequence
to open the editor commands, and select the `Dev Container: Rebuild Container` option. 

3. Select the CMake Kit: Once you open the project, you'll be prompted to select the CMake Kit for your project. You can do this either from the bottom status bar or during the initial setup of the development container a prompt will automatically show up. The CMake Kit represents the C++ toolchain used for building the project (compiler, architecture, etc.). Choose the appropriate kit for your project.

| ![](data/README_Resources/images/kit_selection_first_container_start.png) | 
|:--:| 
| *When the devcontainer is first started CMAKE Tools will ask you to select a kit. The compiler located at /usr/bin/g++ and /usr/bin/gcc will always be the safest choice.* |
| ![](data/README_Resources/images/kit_selection_toolbar.png) |  
| *During subsequent startups, you can easily change the kit using the bottom toolbar.* |

4. Configure and build the project: If the CMake cache needs to be generated or updated, the extension will configure the project automatically. This process may take a few seconds, depending on your hardware.

| ![](data/README_Resources/images/toolbar_build_run.png) | 
|:--:| 
| *Use the buttons in the toolbar to build and run the Autonomy_Software application.* |

## Debugging the C++ Application

Now that you have set up the development environment, you are ready to debug your C++ application.

1. Set Breakpoints: Place breakpoints in your C++ source code where you want to pause the program's execution to inspect variables, analyze the flow, or 
diagnose issues.

| ![](data/README_Resources/images/breakpoint_selection.png) | 
|:--:| 
| *To set breakpoints directly within the vscode editor, toggle on the red dot next to the line numbers.* |

2. Compile for Debugging: In order to properly debug our application, we must compile the program with special flags set. We can let CMAKE do this for us by simply selecting the debug configuration from the bottom toolbar.

| ![](data/README_Resources/images/toolbar_cmake_debug_config.png) | 
|:--:| 
| *Use the toolbar to change between Release and Debug configurations. Release runs faster but can't be debugged, so whenever you're done debugging switch back to the Release config.* |

3. Start Debugging: Click the debug icon in the toolbar to automatically build and start the program.

| ![](data/README_Resources/images/toolbar_debug_button.png) | 
|:--:| 
| *Click the debug button to enter debug mode in VSCode.* |

4. Debugging Controls: Use the debugging controls (e.g., step into, step over, continue, etc.) in the Debug toolbar to navigate through your code while inspecting variables, stack traces, and more.
![](data/README_Resources/images/vscode_debug_mode.png)

- Inspect Variables: In the "Variables" view, you can inspect the current values of variables in your code during debugging.
- Analyze Call Stack: The "Call Stack" view provides a stack trace, helping you understand the flow of execution and identify the function calls that led to the current point in your code.
- Set Watchpoints: You can set watchpoints on specific variables to break execution when their values change.
- Debug Console: Use the Debug Console to execute custom expressions and commands during debugging.

With CMakeTools and Visual Studio Code, you can efficiently build and debug your C++ applications within the familiar and powerful VSCode environment. The seamless integration between CMake and Visual Studio Code makes it easier for C++ developers to focus on writing code and quickly diagnose and fix issues. Happy debugging!
