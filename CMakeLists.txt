####################################################################################################################
##											Autonomy Software												   	  ##
##											v24.01.00 Build 001												 	  ##
##										  Mars Rover Design Team												  ##
##								   Copyright 2023 - All Rights Reserved										 	  ##
####################################################################################################################


## Set CMake Minimum Version
cmake_minimum_required(VERSION 3.24.3)

## C++ Version
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Project Name and Software Version Number
project(Autonomy_Software VERSION 24.01.00)

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
option(BUILD_SIM_MODE "Enable Simulation Mode" ON)
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
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_VENDOR "Mars Rover Design Team")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/data/CPACK/logo.ico")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_SYSTEM_NAME}")
set(CPACK_GENERATOR "STGZ")

## Find RoveComm
FetchContent_Declare(RoveComm_CPP                                                   # Custom Network Protocol (RoveComm C++)
        GIT_REPOSITORY https://github.com/MissouriMRDT/RoveComm_CPP.git             # Source Code URL
        GIT_TAG        608453f6b3e1a4445d281020d7dea568f7bf5e8d                     # Version 24.0.0
        GIT_SHALLOW    FALSE                                                        # Download only without history
        GIT_PROGRESS   TRUE                                                         # Forces progress status.
    )
FetchContent_GetProperties(RoveComm_CPP)
if(NOT RoveComm_CPP)
    # Fetch the content using previously declared details
    FetchContent_Populate(RoveComm_CPP)

    # Set Library Scope
    set(RoveComm_CPP_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/_deps/rovecomm_cpp-src/src)
    set(RoveComm_CPP_SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/_deps/rovecomm_cpp-src)

    # Bring the populated content into the build
    include_directories( ${RoveComm_CPP_INCLUDE_DIR} )
    add_subdirectory(${RoveComm_CPP_SOURCE_DIR})
endif()

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

## Find CUDA.
find_package(CUDA 11.4 REQUIRED)
message(STATUS "Found CUDA ${CUDA_VERSION_STRING} at ${CUDA_TOOLKIT_ROOT_DIR}")

## Find ZEDSDK. Add as system package to supress library warnings.
find_package(ZED REQUIRED)
include_directories(SYSTEM ${ZED_INCLUDE_DIRS})
link_directories(${ZED_LIBRARY_DIRS})

## Find CUDA. Must match ZEDSDK version. Add as system package to supress library warnings.
find_package(CUDAToolkit ${ZED_CUDA_VERSION} REQUIRED)
include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
## Determine if shared or static libraries will be used.
option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()
## Define libraries for ZED. Optionally static or dynamic.
if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} CUDA::toolkit)
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} CUDA::toolkit)
endif()

## Add special flag if compiling on aarch64.
IF (CMAKE_SYSTEM_PROCESSOR MATCHES aarch64)
    add_definitions(-DJETSON_STYLE)
ENDIF()

## Enable to CTest and Google Test Frameworks
enable_testing()

## Search Project Directories for CPP Files
file(GLOB_RECURSE SRC			  	    CONFIGURE_DEPENDS  "src/*.cpp")
file(GLOB_RECURSE Examples_SRC		  	CONFIGURE_DEPENDS  "examples/*/*.cpp")
file(GLOB_RECURSE External_SRC		  	CONFIGURE_DEPENDS  "external/src/*.cpp")
file(GLOB_RECURSE Tools_SRC			 	CONFIGURE_DEPENDS  "tools/*.cpp")

## Create Executable File
add_executable(${PROJECT_NAME}   ${External_SRC}
								 ${SRC}
								 ${Examples_SRC}
								 ${Tools_SRC}
	)

## Set Compile Options for Autonomy Software.
if(MSVC) # True when compiler is Microsoft Visual C++/simulation of Visual C++ CL.
	target_compile_options(${PROJECT_NAME} PRIVATE /W4 /WX)
else()
	target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

## Link Libraries to Executable
target_link_libraries(${PROJECT_NAME} PRIVATE   Threads::Threads
												Eigen3::Eigen
                                                quill::quill
												${OpenCV_LIBS}
                                                ${GeographicLib_LIBRARIES}
                                                ${ZED_LIBS} 
                                                RoveComm_CPP
    )

## Package Executable
install(TARGETS ${PROJECT_NAME} RUNTIME_DEPENDENCIES DIRECTORIES ${OpenCV_LIBS} ${GeographicLib_LIBRARIES} ${ZED_LIBS} quill::quill RoveComm_CPP RUNTIME DESTINATION bin)

## Unit/Integration Tests
file(GLOB_RECURSE UnitTests_SRC         CONFIGURE_DEPENDS  "tests/Unit/*.cc")
file(GLOB_RECURSE IntegrationTests_SRC  CONFIGURE_DEPENDS  "tests/Integration/*.cc")
file(GLOB Logging_SRC			  	    CONFIGURE_DEPENDS  "src/AutonomyLogging.cpp")

list(LENGTH UnitTests_SRC           UnitTests_LEN)
list(LENGTH IntegrationTests_SRC    IntegrationTests_LEN)

if (UnitTests_LEN GREATER 0)
    add_executable(${PROJECT_NAME}_UnitTests ${UnitTests_SRC} ${Logging_SRC})
    target_link_libraries(${PROJECT_NAME}_UnitTests GTest::gtest GTest::gtest_main quill::quill ${OpenCV_LIBS} ${GeographicLib_LIBRARIES})
    add_test(Unit_Tests ${PROJECT_NAME}_UnitTests)
else()
    message("No Unit Tests!")
endif()

if (IntegrationTests_LEN GREATER 0)
    add_executable(${PROJECT_NAME}_IntegrationTests ${IntegrationTests_SRC} ${Logging_SRC})
    target_link_libraries(${PROJECT_NAME}_IntegrationTests GTest::gtest GTest::gtest_main quill::quill ${OpenCV_LIBS} ${GeographicLib_LIBRARIES})
    add_test(Integration_Tests ${PROJECT_NAME}_IntegrationTests)
else()
    message("No Integration Tests!")
endif()
