####################################################################################################################
##											Autonomy Software												   	  ##
##											v24.00.00 Build 000												 	  ##
##										  Mars Rover Design Team												  ##
##								   Copyright 2023 - All Rights Reserved										 	  ##
####################################################################################################################


## Set CMake Minimum Version
cmake_minimum_required(VERSION 3.24.3)

## C++ Version
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Project Name and Software Version Number
project(Autonomy_Software VERSION 24.00.00)

## Include Required CMake Packages
if(NOT CPack_CMake_INCLUDED)
    include(CPack)                                                                  # CMake Packaging Library
endif()
if(NOT CTest_CMake_INCLUDED)
    include(CTest)                                                                  # CMake Testing Framework
endif()
include(FetchContent)                                                               # CMake Dependency Framework

## Define Project Name and Version Number for CPack
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_VENDOR "Mars Rover Design Team")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/data/CPACK/logo.ico")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_SYSTEM_NAME}")
set(CPACK_GENERATOR "STGZ")

## Find RoveComm
FetchContent_Declare(RoveComm_CPP                                                   # Custom Network Protocol (RoveComm)
        GIT_REPOSITORY https://github.com/MissouriMRDT/RoveComm_CPP.git             # Source Code URL
        GIT_TAG        16ba934e74a1c18e18c565c48000481e0451a966                     # Version 24.0.0
        GIT_SHALLOW    FALSE                                                         # Download only without history
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

## Find OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

## Enable to CTest and Google Test Frameworks
enable_testing()

## Search Project Directories for CPP Files
file(GLOB Algorithms_SRC		CONFIGURE_DEPENDS  "src/algorithms/*.cpp")
file(GLOB Interfaces_SRC		CONFIGURE_DEPENDS  "src/interfaces/*.cpp")
file(GLOB Drivers_SRC		    CONFIGURE_DEPENDS  "src/drivers/*.cpp")
file(GLOB States_SRC			CONFIGURE_DEPENDS  "src/states/*.cpp")
file(GLOB Util_SRC			  	CONFIGURE_DEPENDS  "src/util/*.cpp")
file(GLOB Vision_SRC			CONFIGURE_DEPENDS  "src/vision/*.cpp")
file(GLOB Threads_SRC			CONFIGURE_DEPENDS  "src/threads/*.cpp")
file(GLOB Main_SRC			  	CONFIGURE_DEPENDS  "src/*.cpp")
file(GLOB Examples_SRC		  	CONFIGURE_DEPENDS  "examples/*/*.cpp")
file(GLOB External_SRC		  	CONFIGURE_DEPENDS  "external/src/*.cpp")
file(GLOB Tools_SRC			 	CONFIGURE_DEPENDS  "tools/*.cpp")

## Create Executable File
add_executable(${PROJECT_NAME}   ${External_SRC}
								 ${Util_SRC}
								 ${Algorithms_SRC}
								 ${Interfaces_SRC}
                                 ${Drivers_SRC}
								 ${States_SRC}
								 ${Vision_SRC}
                                 ${Threads_SRC}
								 ${Main_SRC}
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
                                                RoveComm_CPP
    )

## Package Executable
install(TARGETS ${PROJECT_NAME} RUNTIME_DEPENDENCIES DIRECTORIES ${OpenCV_LIBS} quill::quill RoveComm_CPP RUNTIME DESTINATION bin)

## Unit/Integration Tests
file(GLOB UnitTests_SRC         CONFIGURE_DEPENDS  "tests/Unit/**/*.cc")
file(GLOB IntegrationTests_SRC  CONFIGURE_DEPENDS  "tests/Integration/**/*.cc")

list(LENGTH UnitTests_SRC           UnitTests_LEN)
list(LENGTH IntegrationTests_SRC    IntegrationTests_LEN)

if (UnitTests_LEN GREATER 0)
    add_executable(${PROJECT_NAME}_UnitTests "tests/Unit/main.cc" ${UnitTests_SRC})
    target_link_libraries(${PROJECT_NAME}_UnitTests GTest::gtest GTest::gtest_main)
    add_test(Unit_Tests ${PROJECT_NAME}_UnitTests)
endif()

if (IntegrationTests_LEN GREATER 0)
    add_executable(${PROJECT_NAME}_IntegrationTests "tests/Unit/main.cc" ${IntegrationTests_SRC})
    target_link_libraries(${PROJECT_NAME}_IntegrationTests GTest::gtest GTest::gtest_main)
    add_test(Integration_Tests ${PROJECT_NAME}_IntegrationTests)
endif()
