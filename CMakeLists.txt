####################################################################################################################
##											Autonomy Software												   	  ##
##											v24.02.00 Build 001												 	  ##
##										  Mars Rover Design Team												  ##
##								   Copyright 2023 - All Rights Reserved										 	  ##
####################################################################################################################


## Set CMake Minimum Version
cmake_minimum_required(VERSION 3.24.3)

## C++ Version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

## Project Name and Software Version Number
project(Autonomy_Software VERSION 24.02.00 LANGUAGES CXX CUDA)

## Allows use of "FindCUDA" function
cmake_policy(SET CMP0146 OLD)

## Include Required CMake Packages
if(NOT CPack_CMake_INCLUDED)
    include(CPack)                                                                  # CMake Packaging Library
endif()
if(NOT CTest_CMake_INCLUDED)
    include(CTest)                                                                  # CMake Testing Framework
endif()
include(FetchContent)                                                               # CMake Dependency Framework

## Enable or Disble Simulation Mode
option(BUILD_SIM_MODE "Enable Simulation Mode" OFF)
if(BUILD_SIM_MODE)
    message("Sim Mode: Enabled")
    add_compile_definitions(__AUTONOMY_SIM_MODE__=1)
    set(EXE_NAME "${PROJECT_NAME}_Sim")
else()
    message("Sim Mode: Disabled")
    add_compile_definitions(__AUTONOMY_SIM_MODE__=0)
    set(EXE_NAME "${PROJECT_NAME}")
endif()

## Define Project Name and Version Number for CPack
set(CPACK_PROJECT_NAME ${EXE_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_VENDOR "Mars Rover Design Team")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/data/CPACK/logo.ico")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_SYSTEM_NAME}")
set(CPACK_GENERATOR "STGZ")

# Configure BSThreadPool
add_compile_definitions(BS_THREAD_POOL_ENABLE_PAUSE=1)
add_compile_definitions(BS_THREAD_POOL_ENABLE_PRIORITY=1)
add_compile_definitions(BS_THREAD_POOL_ENABLE_WAIT_DEADLOCK_CHECK=1)

## Find RoveComm
include_directories(external/rovecomm/src)
add_subdirectory(external/rovecomm)
add_compile_definitions(__ROVECOMM_LIBRARY_MODE__=1)

## Determine if shared or static libraries will be used.
option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)
if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

## Determine if all variables should be listed. Nice for finding available vars when using find_package.
option(LIST_ALL_VARS "Print all CMAKE variables." OFF)

## Find Threads
find_package(Threads REQUIRED)

## Find Quill
find_package(quill REQUIRED)

## Find Google Test
find_package(GTest CONFIG REQUIRED)
include(GoogleTest)
add_library(GTest::GTest INTERFACE IMPORTED)
target_link_libraries(GTest::GTest INTERFACE gtest_main)

## Find Eigen3.
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

## Find OpenCV.
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

## Find Geographic Lib.
find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})

## Find ZEDSDK. Add as system package to supress library warnings.
find_package(ZED 4 REQUIRED)
include_directories(SYSTEM ${ZED_INCLUDE_DIRS})

## Find CUDA. Must match ZEDSDK version. Add as system package to supress library warnings.
find_package(CUDAToolkit ${ZED_CUDA_VERSION} REQUIRED)
include_directories(SYSTEM ${CUDAToolkit_INCLUDE_DIRS})

## Find PyTorch.
set (CAFFE2_USE_CUDNN "1")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

## Find flatbuffers. This is a dependency of tensorflow.
find_package(Flatbuffers REQUIRED)

## Find Abseil. This is a dependency of tensorflow.
find_package(absl REQUIRED)
set(ABSL_LIBRARY_DIRS absl::algorithm 
                    absl::base 
                    absl::debugging 
                    absl::flat_hash_map 
                    absl::flags 
                    absl::memory 
                    absl::meta 
                    absl::numeric 
                    absl::random_random 
                    absl::strings 
                    absl::synchronization 
                    absl::time 
                    absl::utility
    )

## Find TensorflowLite.
find_package(TENSORFLOWLITE REQUIRED PATHS "external/tensorflow/")
include_directories(SYSTEM ${TENSORFLOWLITE_INCLUDE_DIRS})

## Find Libedgetpu. This is used for interfacing with all Coral devices.
find_package(LIBEDGETPU REQUIRED PATHS "external/tensorflow/")
include_directories(SYSTEM ${LIBEDGETPU_INCLUDE_DIRS})

## Define libraries for ZED. Optionally static or dynamic.
if (LINK_SHARED_ZED)
    set(ZED_LIBS ${ZED_LIBRARIES} CUDA::toolkit)
else()
    set(ZED_LIBS ${ZED_LIBRARIES_STATIC_RELEASE} CUDA::toolkit)
endif()

## Add special flag if compiling on aarch64.
if (CMAKE_SYSTEM_PROCESSOR MATCHES aarch64)
    add_definitions(-DJETSON_STYLE)
endif()

## Check if all variables should be listed.
if (LIST_ALL_VARS)
    get_cmake_property(_variableNames VARIABLES)
    list (SORT _variableNames)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
endif()

## Enable to CTest and Google Test Frameworks
enable_testing()

## Search Project Directories for CPP Files
file(GLOB_RECURSE SRC			  	    CONFIGURE_DEPENDS  "src/*.cpp")
file(GLOB_RECURSE Examples_SRC		  	CONFIGURE_DEPENDS  "examples/*/*.cpp")
file(GLOB_RECURSE External_SRC		  	CONFIGURE_DEPENDS  "external/src/*.cpp")
file(GLOB_RECURSE Tools_SRC			 	CONFIGURE_DEPENDS  "tools/*.cpp")

## Create Executable File
add_executable(${EXE_NAME}  ${External_SRC}
						    ${SRC}
						    ${Examples_SRC}
						    ${Tools_SRC}
	)

## Set Compile Options for Autonomy Software.
if(MSVC) # True when compiler is Microsoft Visual C++/simulation of Visual C++ CL.
	target_compile_options(${EXE_NAME} PRIVATE /W4 /WX)
else()
	target_compile_options(${EXE_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

## Link Libraries to Executable
target_link_libraries(${EXE_NAME} PRIVATE   Threads::Threads
												Eigen3::Eigen
                                                RoveComm_CPP
                                                quill::quill
												${OpenCV_LIBS}
                                                ${GeographicLib_LIBRARIES}
                                                ${ZED_LIBS}
                                                ${TORCH_LIBRARIES}
                                                flatbuffers::flatbuffers
                                                ${ABSL_LIBRARY_DIRS}
                                                ${TENSORFLOWLITE_LIBS}
                                                ${LIBEDGETPU_LIBS}
    )

## Package Executable
install(TARGETS ${EXE_NAME} RUNTIME_DEPENDENCIES DIRECTORIES RoveComm_CPP ${OpenCV_LIBS} ${GeographicLib_LIBRARIES} ${ZED_LIBS} quill::quill RUNTIME DESTINATION bin)

## Unit/Integration Tests
file(GLOB_RECURSE UnitTests_SRC         CONFIGURE_DEPENDS  "tests/Unit/*.cc")
file(GLOB_RECURSE IntegrationTests_SRC  CONFIGURE_DEPENDS  "tests/Integration/*.cc")
file(GLOB_RECURSE Algorithms_SRC        CONFIGURE_DEPENDS  "src/algorithms/*.cpp")
file(GLOB         Network_SRC			CONFIGURE_DEPENDS  "src/AutonomyNetworking.cpp")
file(GLOB         Logging_SRC			CONFIGURE_DEPENDS  "src/AutonomyLogging.cpp")

list(LENGTH UnitTests_SRC           UnitTests_LEN)
list(LENGTH IntegrationTests_SRC    IntegrationTests_LEN)

if (UnitTests_LEN GREATER 0)
    add_executable(${EXE_NAME}_UnitTests ${UnitTests_SRC} ${Algorithms_SRC} ${Network_SRC} ${Logging_SRC})
    target_link_libraries(${EXE_NAME}_UnitTests GTest::gtest GTest::gtest_main RoveComm_CPP quill::quill ${OpenCV_LIBS} ${GeographicLib_LIBRARIES})
    add_test(Unit_Tests ${EXE_NAME}_UnitTests)
else()
    message("No Unit Tests!")
endif()

if (IntegrationTests_LEN GREATER 0)
    add_executable(${EXE_NAME}_IntegrationTests ${IntegrationTests_SRC} ${Algorithms_SRC} ${Network_SRC} ${Logging_SRC})
    target_link_libraries(${EXE_NAME}_IntegrationTests GTest::gtest GTest::gtest_main RoveComm_CPP quill::quill ${OpenCV_LIBS} ${GeographicLib_LIBRARIES})
    add_test(Integration_Tests ${EXE_NAME}_IntegrationTests)
else()
    message("No Integration Tests!")
endif()
