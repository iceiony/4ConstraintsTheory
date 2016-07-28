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

set(CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++")

include_directories("./")
include_directories("rendering")
include_directories("lib3ds")

FILE(GLOB LIB3DS "lib3ds/*.c")

set(GRAPHICS_FILES
        rendering/GraphicsManager.cpp
        rendering/GraphicsMesh.cpp
        rendering/GraphicsEntity.cpp
        rendering/ListenerBase.cpp
        rendering/Camera.cpp
        rendering/CameraListener.cpp
        rendering/DebugDisplay.cpp
    )

set(SOURCE_FILES
        Util.cpp
        Simulation.cpp
        PhysicsUtils.cpp
        ${LIB3DS}
    )


add_executable(ExhaustiveSearchGUI exhaustiveSearchGUI.cpp ${SOURCE_FILES} ${GRAPHICS_FILES})
target_link_libraries(ExhaustiveSearchGUI dMath dScene dCustomJoints dContainers dAnimation )

add_executable(ExhaustiveSearch exhaustiveSearch.cpp ${SOURCE_FILES})
target_link_libraries(ExhaustiveSearch dMath dScene dCustomJoints dContainers dAnimation )
