####################################################################################################################
##											Autonomy Software												   	  ##
##											v24.00.00 Build 000												 	  ##
##										  Mars Rover Design Team												  ##
##								   Copyright 2023 - All Rights Reserved										 	  ##
####################################################################################################################


## Set CMake Minimum Version
CMAKE_MINIMUM_REQUIRED(VERSION 3.24.3)

## C++ Version
SET(CMAKE_CXX_STANDARD 20)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

## Project Name and Software Version Number
PROJECT(Autonomy_Software VERSION 24.00.00)

## Include Required CMake Packages
INCLUDE(CPack)																		# CMake Packaging Library
INCLUDE(CTest)																	  	# CMake Testing Framework
INCLUDE(FetchContent)															   	# CMake Dependency Framework

## Define Project Name and Version Number for CPack
SET(CPACK_PROJECT_NAME ${PROJECT_NAME})
SET(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
SET(CPACK_PACKAGE_VENDOR "Mars Rover Design Team")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
SET(CPACK_GENERATOR "SH")

## Find Threads
FIND_PACKAGE(Threads REQUIRED)

## Find Quill
FIND_PACKAGE(quill REQUIRED)

## Find Google Test
FIND_PACKAGE(GTest CONFIG REQUIRED)

## Find Eigen3.
FIND_PACKAGE(Eigen3 3.3 REQUIRED NO_MODULE)

## Find OpenCV
FIND_PACKAGE( OpenCV REQUIRED )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

## Enable to CTest and Google Test Frameworks
ENABLE_TESTING()

## Search Project Directories for CPP Files
FILE(GLOB Algorithms_SRC		CONFIGURE_DEPENDS  "src/algorithms/*.cpp")
FILE(GLOB Drivers_SRC		    CONFIGURE_DEPENDS  "src/drivers/*.cpp")
FILE(GLOB Interfaces_SRC		CONFIGURE_DEPENDS  "src/interfaces/*.cpp")
## FILE(GLOB States_SRC			CONFIGURE_DEPENDS  "src/states/*.cpp")
FILE(GLOB Util_SRC			  	CONFIGURE_DEPENDS  "src/util/*.cpp")
FILE(GLOB Vision_SRC			CONFIGURE_DEPENDS  "src/vision/*.cpp")
FILE(GLOB Main_SRC			  	CONFIGURE_DEPENDS  "src/*.cpp")
FILE(GLOB Examples_SRC		  	CONFIGURE_DEPENDS  "examples/*/*.cpp")
FILE(GLOB External_SRC		  	CONFIGURE_DEPENDS  "external/src/*.cpp")
FILE(GLOB Tests_SRC			 	CONFIGURE_DEPENDS  "tests/*.cpp")
FILE(GLOB Tools_SRC			 	CONFIGURE_DEPENDS  "tools/*.cpp")

## Create Executable File
ADD_EXECUTABLE(${PROJECT_NAME}   ${External_SRC}
								 ${Util_SRC}
								 ${Algorithms_SRC}
                                 ${Drivers_SRC}
								 ${Interfaces_SRC}
								 ${States_SRC}
								 ${Vision_SRC}
								 ${Main_SRC}
								 ${Examples_SRC}
								 ${Tests_SRC}
								 ${Tools_SRC}
	)

## Set Compile Options for Autonomy Software.
IF(MSVC) # True when compiler is Microsoft Visual C++/simulation of Visual C++ CL.
	TARGET_COMPILE_OPTIONS(${PROJECT_NAME} PRIVATE /W4 /WX)
ELSE()
	TARGET_COMPILE_OPTIONS(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
ENDIF()

## Link Libraries to Executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PRIVATE 	gtest_main
                                                Threads::Threads
												Eigen3::Eigen
                                                quill::quill
												${OpenCV_LIBS}
    )

INSTALL(TARGETS ${PROJECT_NAME} RUNTIME_DEPENDENCIES DIRECTORIES ${OpenCV_LIBS} quill::quill RUNTIME DESTINATION bin )