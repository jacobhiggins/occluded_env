#
# Project settings
#

# Minimum required version of cmake
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Project name and programming languages used
PROJECT( MY_COOL_PROJECT CXX )

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )

#
# Include directories
#
INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} )

#
# Build an executable
#
ADD_DEFINITIONS(-std=c++11)
ADD_EXECUTABLE(        point_mass6 point_mass6.cpp )
ADD_EXECUTABLE(        point_mass7 point_mass7.cpp )
TARGET_LINK_LIBRARIES( point_mass6 ${ACADO_SHARED_LIBRARIES} )
TARGET_LINK_LIBRARIES( point_mass7 ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES( point_mass6 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )
SET_TARGET_PROPERTIES( point_mass7 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )
