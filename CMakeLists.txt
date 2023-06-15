####################################################################################################################
##                                            Autonomy Software                                                   ##
##                                            v23.00.00 Build 000                                                 ##
##                                          Mars Rover Design Team                                                ##
##                                   Copyright 2023 - All Rights Reserved                                         ##
####################################################################################################################


## Set CMake Minimum Version
CMAKE_MINIMUM_REQUIRED(VERSION 3.24.3)

## Project Name and Software Version Number
PROJECT('Autonomy_Software' VERSION 23.00.00)

## Include Required CMake Packages
INCLUDE(CTest)                                                                      # CMake Testing Framework
INCLUDE(CPack)                                                                      # CMake Packaging Framework
INCLUDE(FetchContent)                                                               # CMake Dependency Framework

## Define Project Name and Version Number for CPack
SET(CPACK_PROJECT_NAME ${PROJECT_NAME})
SET(CPACK_PROJECT_VERSION ${PROJECT_VERSION})

## Fetch Dependencies
FETCHCONTENT_DECLARE(plog                                                           # C++ Logging Library
        GIT_REPOSITORY https://github.com/SergiusTheBest/plog.git                   # Source Code URL
        GIT_TAG        f47149410a4c927643148b96799f28b2d80d451b                     # Version 1.1.9
        GIT_SHALLOW TRUE                                                            # Download only tip of branch without rest of its history.       
        GIT_PROGRESS TRUE                                                           # Forces progress status.
        )
FETCHCONTENT_DECLARE(googletest                                                     # Google C++ Testing Framework
        GIT_REPOSITORY https://github.com/google/googletest.git                     # Source Code URL
        GIT_TAG        b796f7d44681514f58a683a3a71ff17c94edb0c1                     # Version 1.13.0
        GIT_SHALLOW TRUE                                                            # Download only tip of branch without rest of its history.       
        GIT_PROGRESS TRUE                                                           # Forces progress status.
        )
FETCHCONTENT_DECLARE(opencv                                                         # OpenCV C++ Library
        GIT_REPOSITORY https://github.com/opencv/opencv.git                         # Source Code URL
        GIT_TAG 4.7.0                                                               # Version 4.7.0
        GIT_SHALLOW TRUE                                                            # Download only tip of branch without rest of its history.       
        GIT_PROGRESS TRUE                                                           # Forces progress status.
        )

## Make Dependencies Available
FETCHCONTENT_MAKEAVAILABLE(plog)
FETCHCONTENT_MAKEAVAILABLE(googletest)
FETCHCONTENT_MAKEAVAILABLE(opencv)


## Find Threads
FIND_PACKAGE(Threads REQUIRED)

## Find PLOG
INCLUDE_DIRECTORIES(${plog_SOURCE_DIR}/include)
ADD_COMPILE_DEFINITIONS(PLOG_ENABLE_WCHAR_INPUT)

## Find OpenCV
#FIND_PACKAGE( OpenCV REQUIRED )
#INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

## Enable to CTest and Google Test Frameworks
ENABLE_TESTING()

## Search Project Directories for CPP Files
FILE(GLOB Algorithms_SRC        CONFIGURE_DEPENDS  "src/algorithms/*.cpp")
FILE(GLOB Interfaces_SRC        CONFIGURE_DEPENDS  "src/interfaces/*.cpp")
FILE(GLOB States_SRC            CONFIGURE_DEPENDS  "src/states/*.cpp")
FILE(GLOB Util_SRC              CONFIGURE_DEPENDS  "src/util/*.cpp")
FILE(GLOB Vision_SRC            CONFIGURE_DEPENDS  "src/vision/*.cpp")
FILE(GLOB Main_SRC              CONFIGURE_DEPENDS  "src/*.cpp")
FILE(GLOB Examples_SRC          CONFIGURE_DEPENDS  "examples/*.cpp")
FILE(GLOB External_SRC          CONFIGURE_DEPENDS  "external/src/*.cpp")
FILE(GLOB Tests_SRC             CONFIGURE_DEPENDS  "tests/*.cpp")
FILE(GLOB Tools_SRC             CONFIGURE_DEPENDS  "tools/*.cpp")


## Create Executable File
ADD_EXECUTABLE(Autonomy_Software ${Algorithms_SRC}
                                 ${Interfaces_SRC}
                                 ${States_SRC}
                                 ${Util_SRC}
                                 ${Vision_SRC}
                                 ${Main_SRC}
                                 ${Examples_SRC}
                                 ${External_SRC}
                                 ${Tests_SRC}
                                 ${Tools_SRC}
    )

## Link Libraries to Executable
TARGET_LINK_LIBRARIES(Autonomy_Software PRIVATE gtest_main
                                                plog::plog
                                                Threads::Threads
                                                opencv_core 
                                                opencv_highgui
    )