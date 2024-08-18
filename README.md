\mainpage

<div align="center">
  <a href="https://github.com/missourimrdt/autonomy_software">
    <img width="125" src="https://i.postimg.cc/XYtdp84Z/logo.png" alt="Mars Rover Design Team Logo">
  </a>
  <h1>Autonomy Software C++</h1>
  <p><b>Autonomous Traversal and Object Detection for Rovers in C++</b></p>
  <div>
    <a href="https://github.com/MissouriMRDT/Autonomy_Software/actions/workflows/codeql.yml">
      <img src="https://img.shields.io/github/actions/workflow/status/missourimrdt/autonomy_software/codeql.yml?branch=development&label=CodeQL&style=flat-round" alt="codeql-ci" />
    </a>
    <a href="https://github.com/MissouriMRDT/Autonomy_Software/actions/workflows/tests.yml">
      <img src="https://img.shields.io/github/actions/workflow/status/missourimrdt/autonomy_software/tests.yml?branch=development&label=Unit%20Tests&style=flat-round" alt="tests-ci" />
    </a>
    <a href="https://github.com/MissouriMRDT/Autonomy_Software/actions/workflows/doxygen_generate.yml">
      <img src="https://img.shields.io/github/actions/workflow/status/missourimrdt/autonomy_software/doxygen_generate.yml?branch=development&label=Docs&style=flat-round" alt="docs-ci" />
    </a>
    <a href="https://github.com/MissouriMRDT/Autonomy_Software/actions/workflows/clang_check.yml">
      <img src="https://img.shields.io/github/actions/workflow/status/missourimrdt/autonomy_software/clang_check.yml?branch=development&label=Clang&style=flat-round" alt="clang-ci" />
    </a>
    <a href="https://github.com/MissouriMRDT/Autonomy_Software/actions/workflows/valgrind.yml">
      <img src="https://img.shields.io/github/actions/workflow/status/missourimrdt/autonomy_software/valgrind.yml?branch=development&label=Valgrind&style=flat-round" alt="valgrind-ci" />
    </a>
  </div>

  <div>
    <a href="https://codecov.io/gh/MissouriMRDT/Autonomy_Software" > 
        <img src="https://codecov.io/gh/MissouriMRDT/Autonomy_Software/branch/topic%2Fcode-coverage/graph/badge.svg?token=AZVPRPE5A8" alt="codecov-ci" /> 
    </a>
    <a href="https://app.codacy.com/gh/missourimrdt/autonomy_software/dashboard?utm_source=gh&utm_medium=referral&utm_content=&utm_campaign=Badge_grade">
      <img src="https://img.shields.io/codacy/grade/cd387bc34658475d98bff84db3ad5287?logo=codacy&style=flat-round" alt="codacy-ci" />
    </a>
    <a href="https://www.codefactor.io/repository/github/missourimrdt/autonomy_software">
      <img src="https://img.shields.io/codefactor/grade/github/missourimrdt/autonomy_software?logo=codefactor&style=flat-round" alt="codefactor-ci" />
    </a>
  </div>

  <div>
    <a href="https://github.com/MissouriMRDT/Autonomy_Software/pkgs/container/autonomy-jammy">
      <img src="https://img.shields.io/badge/Ubuntu_Jammy-latest-orange" alt="jammy-pkg" />
    </a>
    <a href="https://github.com/MissouriMRDT/Autonomy_Software/pkgs/container/autonomy-jetpack">
      <img src="https://img.shields.io/badge/NVIDIA_JetPack_6-latest-orange" alt="jetpack-pkg" />
    </a>
  </div>
  <div>
    <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">
      <img src="https://img.shields.io/badge/license-GPLv3-blue.svg?style=flat-round" alt="license" />
    </a>
    <a href="https://en.cppreference.com/w/cpp/20">
      <img src="https://img.shields.io/badge/language-C%2B%2B20-blue.svg?style=flat-round" alt="language" />
    </a>
  </div>
</div>

---

## 🚀 Introduction

Welcome to the **Autonomy Software** repository of the [Mars Rover Design Team (MRDT)](https://marsrover.mst.edu) at [Missouri University of Science and Technology (Missouri S&T)](https://mst.edu)! This repository contains the source code, documentation, and other resources for developing autonomy software for our Mars Rover. We aim to compete in the University Rover Challenge (URC) by showcasing advanced autonomous capabilities and robust navigation algorithms.

---

## 🗂️ Codebase Layout

Here's the overall layout of the Autonomy Software codebase:

![Autonomy_Software Codebase Structure](https://missourimrdt.github.io/Autonomy_Software/main_8cpp__incl.png)

### Directory Structure:

- `algorithms/`: Implements core autonomous navigation and perception logic.
- `drivers/`: Interfaces with the Rover’s hardware systems such as the drive board and navigation board.
- `handlers/`: Manages various functional components such as camera, object detection, and waypoint handling.
- `interfaces/`: Manages communication with external hardware components.
- `states/`: Defines state machine behavior for efficient task execution.
- `util/`: Utility scripts and helper functions for development and debugging.
- `vision/`: Processes visual data for navigation and object recognition.
- `tests/`: Ensures code correctness and reliability.
- `tools/`: Miscellaneous development tools and utilities.
- `external/`: External libraries and dependencies.
- `examples/`: Sample code demonstrating specific functionalities.
- `docs/`: Comprehensive documentation for developers and contributors.
- `data/`: Contains various datasets and configurations for testing and analysis.

---

## 📖 API Documentation

Our documentation is automatically generated via [Doxygen](https://doxygen.nl/) on each commit to the `development` branch. Ensure you use the correct documentation templates to keep the documentation consistent.

**📚 [Click here for full API documentation.](https://missourimrdt.github.io/Autonomy_Software/)**

---

## 🛠️ Getting Started

Before diving into the installation, ensure that your system meets the following prerequisites.

### Prerequisites

To build and run the Autonomy Software, you can either set up the necessary dependencies manually on your local system or use our pre-configured Docker environment where everything is preinstalled.

#### Option 1: Using the Preinstalled Docker Environment (Recommended)

The easiest way to get started is by using our Docker environment. This environment comes with all the required software and dependencies preinstalled, including:

- **NVIDIA CUDA Toolkit 12.2**
- **CMake 3.30.2**
- **GCC 10**
- **OpenCV 4.10.0**
- **PyTorch 2.2.2**
- **TensorFlow 2.15.0**
- **ZED SDK 4.1**
- **Quill 7.0.0-pre**
- **Google Test 1.15.2**
- **Abseil 20230802.1**
- **GeographicLib 2.3**
- **Git LFS**
- **And more...**

This Docker environment is designed to work seamlessly with NVIDIA GPUs and supports both AMD64 and ARM64 architectures.

#### Option 2: Setting Up Manually on Your Local System

If you prefer to set up the environment manually, make sure your system has the following hardware and software dependencies installed:

- **Hardware Requirements**:
  - NVIDIA Graphics Card with CUDA support (required for running GPU-accelerated vision processing)
  - Minimum CUDA version: 12.2 (compatible with the NVIDIA graphics card for GPU computation)
  - NVIDIA Jetson Devices (JetPack SDK required for ARM64 platforms)

- **Operating Systems**:
  - Ubuntu 22.04 or higher (for AMD64 platforms)
  - NVIDIA Jetson Devices with JetPack SDK (required for ARM64 platforms)

- **Required Software**:
  - CMake 3.24.3 or higher
  - GCC 10 or higher (for compiling C++ code)
  - Git 2.25 or higher
  - Git LFS (Large File Storage) 2.13.0 or higher (for handling large files)
  - Python 3.8 or higher (for scripting and utility purposes)
  - Doxygen (for generating documentation)
  - OpenCV 4.10.0 or higher (required for vision processing)
  - CUDA Toolkit 12.2 or higher (or NVIDIA graphics card support)
  - NVIDIA JetPack SDK (required for Jetson platforms)
  - ZED SDK 4.1 or higher (for stereolabs cameras)
  - PyTorch 2.2.2 or higher (for machine learning tasks)
  - TensorFlow 2.15.0 or higher (for deep learning tasks)
  - Quill 6.1.2 or higher (for logging)
  - Google Test 1.15.2 (for testing)
  - Abseil 20230802.1 (for additional utilities)
  - GeographicLib 2.3 (for geospatial computations)

Now that your system is set up, you can proceed with the installation and build instructions provided in the [INSTALL.md](INSTALL.md) file.

---

## 🔧 Build Modes

The Autonomy Software project includes various build modes to cater to different development, testing, and deployment needs.

### Simulation Mode

Simulation mode enables the software to be compiled with simulation-specific features. This mode is disabled by default and is currently configured for integration in the Webots Robot Simulator.

To enable **Simulation Mode**:

```bash
mkdir -p <Autonomy Install Location>/build
cd <Autonomy Install Location>/build
cmake -DBUILD_SIM_MODE=ON ..
make
./Autonomy_Software_Sim
```

### Code Coverage Mode

Code coverage mode compiles the software with flags that enable coverage reporting. This mode is disabled by default.

To enable **Code Coverage Mode**:

```bash
mkdir -p <Autonomy Install Location>/build
cd <Autonomy Install Location>/build
cmake -DBUILD_CODE_COVERAGE=ON ..
make
./Autonomy_Software
```

### Verbose Makefile Mode

Verbose mode provides detailed output during the build process, useful for debugging build issues. This mode is disabled by default.

To enable **Verbose Makefile Mode**:

```bash
mkdir -p <Autonomy Install Location>/build
cd <Autonomy Install Location>/build
cmake -DBUILD_VERBOSE_MODE=ON ..
make
```

### Install Mode

Install mode enables the packaging of the executable for installation. This mode is disabled by default.

To enable **Install Mode** and package the executable:

```bash
mkdir -p <Autonomy Install Location>/build
cd <Autonomy Install Location>/build
cmake -DBUILD_INSTALL_MODE=ON ..
make
sudo make install
```

### Tests Mode

Tests mode enables the compilation and execution of unit and integration tests. This mode is disabled by default.

To enable **Tests Mode** and run the tests:

```bash
mkdir -p <Autonomy Install Location>/build
cd <Autonomy Install Location>/build
cmake -DBUILD_TESTS_MODE=ON ..
make
ctest
```

If tests are available, they will be compiled into separate executables for unit and integration tests. These tests can be run individually or using CTest.

---

## 🗒️ Third-Party Libraries

This project makes use of several open-source libraries and tools that help us achieve our autonomy goals. We would like to acknowledge and thank the developers of the following libraries:

- **OpenCV** - Computer vision library for real-time image and video processing.  
  [OpenCV GitHub](https://github.com/opencv/opencv)  
  License: [BSD-3-Clause License](https://opencv.org/license/)

- **PyTorch** - A deep learning framework for building and training neural networks.  
  [PyTorch GitHub](https://github.com/pytorch/pytorch)  
  License: [BSD-3-Clause License](https://github.com/pytorch/pytorch/blob/main/LICENSE)

- **TensorFlow Lite** - Lightweight version of TensorFlow for running machine learning models on mobile and embedded devices.  
  [TensorFlow Lite](https://www.tensorflow.org/lite/guide)  
  License: [Apache License 2.0](https://www.tensorflow.org/license)

- **Libedgetpu** - Library for accessing Google’s Edge TPU for hardware-accelerated machine learning inference on embedded devices.  
  [Libedgetpu](https://coral.ai/docs/reference/cpp/)  
  License: [Apache License 2.0](https://github.com/google-coral/libedgetpu/blob/master/LICENSE)

- **ZED SDK** - Software development kit for Stereolabs ZED cameras.  
  [Stereolabs ZED SDK](https://www.stereolabs.com/developers/)  
  License: [ZED SDK License](https://www.stereolabs.com/legal/sdk-eula/)

- **Quill** - Fast, low-latency logging library.  
  [Quill GitHub](https://github.com/odygrd/quill)  
  License: [MIT License](https://github.com/odygrd/quill/blob/master/LICENSE)

- **Google Test** - C++ testing framework used for unit tests.  
  [Google Test GitHub](https://github.com/google/googletest)  
  License: [BSD-3-Clause License](https://github.com/google/googletest/blob/main/LICENSE)

- **Abseil** - Collection of C++ libraries designed for performance and ease of use.  
  [Abseil GitHub](https://github.com/abseil/abseil-cpp)  
  License: [Apache License 2.0](https://github.com/abseil/abseil-cpp/blob/master/LICENSE)

- **GeographicLib** - C++ library for geodesic and geographic computations.  
  [GeographicLib GitHub](https://github.com/geographiclib/geographiclib)  
  License: [MIT License](https://github.com/geographiclib/geographiclib/blob/main/LICENSE.txt)

- **Eigen** - A high-performance C++ library for linear algebra, matrix, and vector operations.  
  [Eigen GitHub](https://gitlab.com/libeigen/eigen)  
  License: [MPL2 License](https://gitlab.com/libeigen/eigen/-/blob/master/COPYING.MPL2)

- **CMake** - Cross-platform tool for managing the build process of software.  
  [CMake GitHub](https://github.com/Kitware/CMake)  
  License: [BSD-3-Clause License](https://cmake.org/licensing/)

- **CUDA Toolkit** - Parallel computing platform and programming model by NVIDIA.  
  [NVIDIA CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit)  
  License: [NVIDIA CUDA EULA](https://docs.nvidia.com/cuda/eula/index.html)

- **Git LFS** - Git extension for versioning large files.  
  [Git LFS GitHub](https://github.com/git-lfs/git-lfs)  
  License: [MIT License](https://github.com/git-lfs/git-lfs/blob/main/LICENSE.md)

- **Doxygen** - Tool for generating documentation from annotated C++ sources.  
  [Doxygen GitHub](https://github.com/doxygen/doxygen)  
  License: [GPL-2.0 License](https://github.com/doxygen/doxygen/blob/master/LICENSE)

---

## 📦 Submodule Libraries

This project also uses the following submodules, which are included as part of the repository:

- **Thread Pool** - A lightweight and efficient C++17 thread pool implementation.  
  [Thread Pool GitHub](https://github.com/bshoshany/thread-pool)  
  License: [MIT License](https://github.com/bshoshany/thread-pool/blob/master/LICENSE)

- **RoveComm_CPP** - A C++ implementation of the RoveComm communication protocol for MRDT systems.  
  [RoveComm_CPP GitHub](https://github.com/MissouriMRDT/RoveComm_CPP)  
  License: [MIT License](https://github.com/MissouriMRDT/RoveComm_CPP/blob/master/LICENSE)

---

## 🙌 A Special Thanks to Our Contributors

The success of the **Mars Rover Design Team's Autonomy Software** wouldn’t be possible without the dedication, passion, and hard work of our amazing contributors. Every commit, every idea, and every effort brings us one step closer to reaching new heights in the University Rover Challenge.

**Thank you to:**

- The **Software Team** for their commitment to building robust and cutting-edge autonomy algorithms.
- The **Mechanical, Electrical, and Science Subteams** for providing the rover with the hardware and instrumentation it needs to achieve autonomy.
- The **R&D Subteam** for pushing the boundaries of our capabilities with innovation and forward-thinking.
- The **Public Relations Subteam** for keeping our community informed and excited about our journey.

We also extend our gratitude to the countless alumni, mentors, and supporters who have provided invaluable guidance and resources over the years.

Every contributor, from newcomers to veterans, plays a critical role in the continued success of the team. Your hard work is what makes **Mars Rover Design Team** the exceptional organization that it is today.

**Let’s keep roving hard!** 🚀
