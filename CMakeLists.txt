project(OpenSimTaskSpaceControl)

cmake_minimum_required(VERSION 2.6)

SET(KIT TaskSpaceControl)
SET(UKIT TASKSPACECONTROL)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
FIND_PACKAGE(OpenSim REQUIRED)

SET(CMAKE_CONFIGURATION_TYPES "RelWithDebInfo;Release"
      CACHE STRING "Semicolon separated list of supported configuration types.
      Only supports Debug, Release, MinSizeRel, and RelWithDebInfo; anything
      else will be ignored." FORCE )

INCLUDE_DIRECTORIES(${OpenSimSimbody_INCLUDE_DIRS})

SUBDIRS(TaskSpace examples)


