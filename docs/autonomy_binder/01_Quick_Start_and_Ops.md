# Quick Start and Operations Manual

This guide covers the exact steps to go from a closed laptop to a moving robot. It contains everything you need to know to get the Autonomy Software running, whether you are developing locally, testing in a simulation, or deploying on the physical hardware.

---

## 1. Setting up the Dev Container Environment

The Autonomy Software project relies heavily on **Docker** and **Visual Studio Code Dev Containers**. This ensures that every developer and the robot itself run the exact same environment with all dependencies (CMake, GCC, OpenCV, PyTorch, ZED SDK, ROS, etc.) pre-installed.

### Prerequisites
- **Visual Studio Code** with the `Dev Containers` extension installed.
- **Docker** (Docker Desktop for Windows/Mac, Docker Engine for Linux).
- **NVIDIA Container Toolkit** (Optional, but highly recommended if you have an NVIDIA GPU for hardware-accelerated processing).

### Building and Attaching
1. Clone the repository recursively:
   ```bash
   git clone --recursive https://github.com/MissouriMRDT/Autonomy_Software.git
   ```
2. Open the cloned folder in VSCode.
3. You should see a prompt in the bottom right corner asking to **"Reopen in Container"**. Click it.
   - If you don't see the prompt, press `CTRL + SHIFT + P`, type `Dev Containers: Rebuild and Reopen in Container`, and hit enter.
4. The first time you do this, Docker will pull the image and build the container. This can take several minutes. Once the logs in the terminal stop and the terminal prompt appears, your container is ready.

### What is inside the Dev Container?
The container uses an Ubuntu base (Jammy or JetPack depending on the architecture) and includes:
- **Compilers and Build Tools**: CMake 3.30+, GCC 10.
- **Machine Learning**: PyTorch, TensorFlow Lite, libedgetpu.
- **Computer Vision**: OpenCV 4.11+, ZED SDK 4.1.
- **Utilities**: Quill (logging), Google Test, Eigen, GeographicLib.
- **Extensions**: Custom VSCode extensions defined in `.devcontainer/devcontainer.json` for formatting, debugging, and Doxygen documentation generation.

---

## 2. Building the Code

Our build system is managed by **CMake**.

### Standard Build (Release Mode)
For competition and real-world deployment, you should always build in Release mode for optimal performance.

1. Create and navigate to the build directory:
   ```bash
   mkdir -p build && cd build
   ```
2. Configure with CMake:
   ```bash
   cmake -DCMAKE_BUILD_TYPE=Release ..
   ```
3. Compile the code:
   ```bash
   make -j$(nproc)
   ```
   > **Pro Tip**: In VSCode, you can configure the number of parallel jobs used by CMake to prevent locking up your machine. Edit `.devcontainer/devcontainer.json` and set `"cmake.parallelJobs": N` (where `N` is no more than half your CPU cores).

### Simulation Mode
If you are testing logic without the physical robot, you can build with simulation features enabled (currently configured for Webots).

```bash
mkdir -p build && cd build
cmake -DBUILD_SIM_MODE=ON ..
make -j$(nproc)
```

### Cleaning a Corrupted Build
If things get weird (e.g., strange linker errors after pulling new code), clear the build cache:
```bash
cd build
rm -rf *
cmake ..
make -j$(nproc)
```

---

## 3. Running Autonomy

Once built, the executable will be located in the `build/` directory (or wherever you configured CMake to output).

### Running on Physical Hardware
To run the main autonomy loop on the actual rover:
```bash
cd build
./Autonomy_Software
```

### Running in Simulation
If you built with `BUILD_SIM_MODE=ON`, the executable name is changed:
```bash
cd build
./Autonomy_Software_Sim
```

### Running Tests
To verify everything is working correctly, run the unit and integration tests (must be built with `-DBUILD_TESTS_MODE=ON`):
```bash
cd build
cmake -DBUILD_TESTS_MODE=ON ..
make -j$(nproc)
ctest --output-on-failure
```

---

## 4. Log Management

Logs are critical for understanding what the robot was thinking when it made a decision. We use the **Quill** logging library for fast, low-latency logging.

- **Location**: Logs are typically saved in the `logs/` directory at the root of the workspace (or wherever configured in `AutonomyLogging.cpp`/`config.yaml`).
- **Viewing Logs**: You can view the `.log` files in VSCode directly. Use tools like `grep` or the VSCode search to find specific state transitions or error messages.
- **Log Levels**: Ensure your build or runtime configuration is set to the appropriate log level (e.g., `DEBUG` for verbose output during testing, `INFO` or `WARNING` for competition).

---

## 5. Quick Commands Reference (Cheat Sheet)

| Action | Command |
| :--- | :--- |
| **Clean Build Directory** | `rm -rf build/*` |
| **Standard Build Config** | `cmake -DCMAKE_BUILD_TYPE=Release -B build/` |
| **Compile** | `make -C build/ -j$(nproc)` |
| **Run Autonomy** | `./build/Autonomy_Software` |
| **Run Tests** | `cd build && ctest` |
