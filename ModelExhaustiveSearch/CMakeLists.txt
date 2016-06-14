cmake_minimum_required(VERSION 3.5)
project(MatlabNewtonDynamics)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

#add NEWTON-DYNAMICS to the build path
SET(NEWTON_DEMOS_SANDBOX OFF CACHE BOOL "Disable sandbox demo build" FORCE)
add_subdirectory("newton-dynamics")

find_package( GLFW3 REQUIRED)
include_directories(${GLFW3_INCLUDE_DIR})
link_libraries(${GLFW3_LIBRARY})

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})
link_libraries(${OPENGL_LIBRARIES})

if(APPLE)
        find_library(COCOA NAMES Cocoa)
        link_libraries(${COCOA})

        find_library(IOKIT_FRAMEWORK NAMES IOKit)
        link_libraries(${IOKIT_FRAMEWORK})

        find_library(COREVIDEO CoreVideo)
        link_Libraries(${COREVIDEO})
endif()

set(SOURCE_FILES main.cpp)
add_executable(MatlabNewtonDynamics ${SOURCE_FILES})
target_link_libraries(MatlabNewtonDynamics Newton dMath)
