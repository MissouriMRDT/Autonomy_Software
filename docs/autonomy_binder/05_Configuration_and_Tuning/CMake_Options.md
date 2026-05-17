# CMake Options

The autonomy software uses CMake for its build system. The way the code is compiled heavily affects its performance and capabilities.

## Build Types

You set the build type during the initial configuration step:
`cmake -DCMAKE_BUILD_TYPE=<Type> ..`

- **Debug**: Compiles the code with debug symbols (`-g`) and removes optimizations (`-O0`). *Never use this on the real robot during competition*, as the framerates for perception algorithms will be catastrophically slow. Use only for running GDB locally.
- **Release**: Compiles the code with maximum optimizations (`-O3`). *Always use this for competition and general testing.* It strips debug symbols and runs significantly faster.

## Custom Flags

These flags are defined in the `CMakeLists.txt` and can be toggled via command line (`-DFLAG_NAME=ON/OFF`).

- **`BUILD_SIM_MODE`**:
  - *Default:* `OFF`
  - *Purpose:* Compiles the software to interface with a simulator (like Webots or Unreal Engine via RoveSoSimulator) instead of physical hardware. It changes the output executable name to `Autonomy_Software_Sim` and alters the RoveComm port defaults.
- **`BUILD_TESTS_MODE`**:
  - *Default:* `OFF`
  - *Purpose:* Compiles Google Test unit and integration tests into separate executables.
- **`BUILD_CODE_COVERAGE`**:
  - *Default:* `OFF`
  - *Purpose:* Compiles with `gcov` flags to track test coverage. Used primarily by CI/CD pipelines.
- **`BUILD_EXAMPLES_MODE`**:
  - *Default:* `OFF`
  - *Purpose:* Compiles standalone example scripts (e.g., test scripts for specific sensors or logic).

## Hardware Flags

- **NVIDIA Jetson / ARM64**:
  - If CMake detects it is running on `aarch64` (e.g., an NVIDIA Jetson), it automatically adds the `-DJETSON_STYLE` definition. This is used in the code to conditionally compile specific optimizations or hardware pathways (like using different TensorRT models).
- **CUDA Toolkits**:
  - CMake automatically searches for `CUDA::toolkit`, `PCL` (Point Cloud Library), and `Torch` (PyTorch). If building natively, ensure these are in your path. (The Dev Container handles this automatically).
