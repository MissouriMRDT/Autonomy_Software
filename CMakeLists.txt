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
        )
FETCHCONTENT_DECLARE(googletest                                                     # Google C++ Testing Framework
        GIT_REPOSITORY https://github.com/google/googletest.git                     # Source Code URL
        GIT_TAG        b796f7d44681514f58a683a3a71ff17c94edb0c1                     # Version 1.13.0
        )

## Make Dependencies Available
FETCHCONTENT_MAKEAVAILABLE(plog)
FETCHCONTENT_MAKEAVAILABLE(googletest)

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
FILE(GLOB Communication_SRC         CONFIGURE_DEPENDS   "Source/Autonomy/Communication/*.cpp")
FILE(GLOB DataAndComputation_SRC    CONFIGURE_DEPENDS   "Source/Autonomy/DataAndComputation/Algorithms/*.cpp"
                                                        "Source/Autonomy/DataAndComputation/Hardware/*.cpp"
                                                        "Source/Autonomy/DataAndComputation/Math/*.cpp"
    )
FILE(GLOB Main_SRC                  CONFIGURE_DEPENDS   "Source/Autonomy/Main/*.cpp")
FILE(GLOB SetUp_SRC                 CONFIGURE_DEPENDS   "Source/Autonomy/SetUp/*.cpp")
FILE(GLOB Vision_SRC                CONFIGURE_DEPENDS   "Source/Autonomy/Vision/*.cpp")

## Create Executable File
ADD_EXECUTABLE(Autonomy_Software ${Communication_SRC} ${DataAndComputation_SRC} ${Main_SRC} ${SetUp_SRC} ${Vision_SRC})

## Link Libraries to Executable
TARGET_LINK_LIBRARIES(Autonomy_Software PRIVATE gtest_main
                                                plog::plog
                                                Threads::Threads
    )