# CMake Build Configuration & Options

The autonomy software utilizes **CMake** (version 3.24.3 or newer) as its meta-build system. The build configuration enforces modern C++20 language standards, compiler constraints, and aggressive compile-time optimizations to support high-performance embedded execution on the NVIDIA Jetson platform.

---

## 1. Compiler and Toolchain Constraints

### Mandatory GCC 10 Enforcement
The top-level `CMakeLists.txt` executes a pre-configuration check verifying the host compiler version:
```cmake
execute_process(
    COMMAND gcc -dumpversion
    OUTPUT_VARIABLE GCC_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT GCC_VERSION VERSION_EQUAL "10.0")
    message(FATAL_ERROR "Only GCC 10.0 is allowed! Detected GCC version ${GCC_VERSION}.")
endif()
```
*Rationale*: Strict binary compatibility with NVIDIA JetPack 5.x, CUDA 11.4/12.x, and LibTorch runtime libraries requires GCC 10. Attempting to build with GCC 9 or GCC 11+ produces ABI incompatibilities or internal compiler segmentation faults during TorchScript and TensorRT template expansions.

### C++ and CUDA Standards
- **`CMAKE_CXX_STANDARD 20`**: Enables modern C++20 features (concepts, ranges, `std::span`, designated initializers, three-way comparisons, and coroutine primitives).
- **`CMAKE_CUDA_STANDARD 20`**: Aligns device CUDA compilation with host C++20 standards.

---

## 2. Performance & Compilation Optimizations

1. **LLD Linker Integration**:
   ```cmake
   if(NOT MSVC)
       add_link_options("-fuse-ld=lld")
   endif()
   ```
   Replaces the default GNU `ld` or `gold` linkers with LLVM's `lld`. This reduces final binary link times by up to 70% and substantially curtails peak RAM consumption on RAM-constrained Jetson systems.

2. **CMake Unity Builds**:
   ```cmake
   set(CMAKE_UNITY_BUILD ON)
   set(CMAKE_UNITY_BUILD_BATCH_SIZE 8)
   ```
   Aggregates up to 8 translation units into unified compilation files. This dramatically reduces redundant header parsing overhead (particularly heavy headers like OpenCV, LibTorch, and ZED SDK), speeding up full builds and minimizing compiler process swapping.

---

## 3. Build Types

Specify the build type during CMake generation:
```bash
cmake -B build -DCMAKE_BUILD_TYPE=<Type>
```

- **`Release`** (Mandatory for Testing & Competition):
  - Sets optimization level `-O3`.
  - Strips debug symbol tables and enables aggressive inlining and vectorization.
  - Required to hit targeted vision framerates (30 FPS) and maintain sub-millisecond state machine iterations.
- **`Debug`** (Local Developer Diagnostics Only):
  - Sets optimization level `-O0` and includes `-g` debug symbols.
  - Useful for debugging segmentation faults with GDB or Valgrind.
  - *Warning*: Do not run on the physical rover in competition; perception pipelines will drop below 5 FPS.
- **`RelWithDebInfo`**:
  - Compiles with `-O2 -g`. Provides optimized execution while preserving stack traces for core dump analysis.

---

## 4. Configurable CMake Options

Options are toggled via `-D<OPTION>=ON|OFF` during configuration:

| Option | Default | Output Executable | Description |
| :--- | :---: | :---: | :--- |
| **`BUILD_SIM_MODE`** | `OFF` | `Autonomy_Software_Sim` (when ON) | Defines `__AUTONOMY_SIM_MODE__=1`. Switches sensor streams to consume WebRTC pixel streaming and local loopback sockets from Unreal Engine RoveSoSimulator. |
| **`BUILD_TESTS_MODE`** | `OFF` | `tests/*` | Enables `CTest` and builds GoogleTest unit and integration test suites in `tests/`. |
| **`ENABLE_LIDAR_GEO_UTESTS`** | `OFF` | N/A | Enables specialized LiDAR database query and GeoPlanner unit tests that require physical USGS LiDAR data tiles. |
| **`BUILD_CODE_COVERAGE`** | `OFF` | N/A | Injects GCC profiling flags (`-O0 -g -fprofile-arcs -ftest-coverage --coverage`) to generate `gcov`/`lcov` coverage reports in CI pipelines. |
| **`BUILD_COVERAGE_WATCH`** | `OFF` | N/A | Enables real-time code coverage file-watching mode for development workflows. |
| **`BUILD_VERBOSE_MODE`** | `OFF` | N/A | Generates verbose Makefiles displaying all raw compiler and linker commands during compilation. |
| **`BUILD_EXAMPLES_MODE`** | `OFF` | `examples/*` | Compiles standalone hardware verification examples for isolated subsystem testing. |
| **`LINK_SHARED_ZED`** | `ON` | N/A | Links dynamically against the ZED SDK shared libraries (`sl::Camera`). Set to OFF if using custom static ZED builds. |
| **`RC_CROSS_COMPILE`** | `OFF` | N/A | Cross-compiles the RoveComm communication library for both Linux and Windows environments. |
| **`LIST_ALL_VARS`** | `OFF` | N/A | Dumps all active internal CMake cache variables and include paths to the console during configuration. |

---

## 5. Subsystem Dependencies

The CMake build automatically locates and links the following system packages:

- **CUDA Toolkit** (`CUDA::cudart`, `CUDA::curand`): Accelerates deep learning inferences and stereoscopic processing.
- **LibTorch** (`Torch`): PyTorch C++ front-end for executing YOLO TorchScript models.
- **OpenCV** (`OpenCV`): Computer vision algorithms, image manipulation, ArUco fiducials, and video encoding.
- **ZED SDK** (`sl::Camera`): Stereolabs depth estimation, positional tracking, and spatial mapping.
- **GeographicLib** (`GeographicLib::GeographicLib`): High-accuracy ellipsoidal geodesy and UTM coordinate conversions.
- **DuckDB** (`duckdb`): Embedded analytical spatial SQL engine for querying 2.5D USGS elevation maps.
- **Quill** (`quill::quill`): Low-latency asynchronous multi-threaded logging engine.
- **BS::thread_pool**: Header-only thread pool for concurrent camera and frame dispatching.
- **RoveComm**: Missouri MRDT telemetry transport layer.
