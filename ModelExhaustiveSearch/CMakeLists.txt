cmake_minimum_required(VERSION 3.5)
project(MatlabNewtonDynamics)


file(COPY "obj51.3ds" DESTINATION "./")
file(COPY "obj52.3ds" DESTINATION "./")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

#add NEWTON-DYNAMICS to the build path
SET(NEWTON_DEMOS_SANDBOX OFF CACHE BOOL "Disable sandbox demo build" FORCE)
add_subdirectory("newton-dynamics")
#add_subdirectory("DEMO")

find_package( GLFW3 REQUIRED)
include_directories(${GLFW3_INCLUDE_DIR})
link_libraries(${GLFW3_LIBRARY})

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})
link_libraries(${OPENGL_LIBRARIES})

find_package(GLEW)
include_directories(${GLEW_INCLUDE_DIR})
link_libraries(${GLEW_LIBRARY})

find_package(TinyXML REQUIRED)
include_directories(${TinyXML_INCLUDE_DIRS})
link_libraries(${TinyXML_LIBRARIES})

if(APPLE)
        find_library(COCOA NAMES Cocoa)
        link_libraries(${COCOA})

        find_library(IOKIT_FRAMEWORK NAMES IOKit)
        link_libraries(${IOKIT_FRAMEWORK})

        find_library(COREVIDEO CoreVideo)
        link_Libraries(${COREVIDEO})
endif()

include_directories("./")
include_directories("rendering")
include_directories("lib3ds")

FILE(GLOB LIB3DS "lib3ds/*.c")

set(SOURCE_FILES
        main.cpp
        Simulation.cpp
        rendering/Util.cpp
        rendering/GraphicsManager.cpp
        rendering/GraphicsMesh.cpp
        rendering/GraphicsEntity.cpp
        rendering/ListenerBase.cpp
        rendering/Camera.cpp
        rendering/CameraListener.cpp
        rendering/dHighResolutionTimer.cpp
        rendering/DebugDisplay.cpp
        rendering/PhysicsUtils.cpp
        ${LIB3DS}
    )

add_executable(MatlabNewtonDynamics ${SOURCE_FILES})
target_link_libraries(MatlabNewtonDynamics dMath dScene dCustomJoints dContainers dAnimation )

set(CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++")
#if (UNIX)
#        add_definitions(-DD_JOINTLIBRARY_STATIC_LIB)
#
#        if (BUILD_64)
#                add_definitions(-D_POSIX_VER -D_POSIX_VER_64)
#        else (BUILD_64)
#                add_definitions(-D_POSIX_VER)
#        endif (BUILD_64)
#endif(UNIX)
#
#if (CMAKE_COMPILER_IS_GNUCC)
#        add_definitions(-fpic -msse -msse2 -c -Wall -std=c++11)
#endif(CMAKE_COMPILER_IS_GNUCC)

