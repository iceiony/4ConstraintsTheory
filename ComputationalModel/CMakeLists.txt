cmake_minimum_required(VERSION 3.2)
project(ComputationalModel)


file(COPY "obj51.3ds" DESTINATION "./")
file(COPY "obj52.3ds" DESTINATION "./")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

find_package(GLFW3 REQUIRED)
include_directories(${GLFW3_INCLUDE_DIR})

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

find_package(GLEW)
include_directories(${GLEW_INCLUDE_DIR})

find_package(TinyXML REQUIRED)
include_directories(${TinyXML_INCLUDE_DIR})

find_package(Threads)

#add NEWTON-DYNAMICS to the build path
SET(NEWTON_DEMOS_SANDBOX OFF CACHE BOOL "Disable sandbox demo build" FORCE)
add_subdirectory("newton-dynamics")
#add_subdirectory("DEMO")

set(CMAKE_CXX_FLAGS "-std=c++11")

set(TARGET_LIBRARIES
        dMath
        dScene
        dCustomJoints
        dContainers
        dAnimation
        ${GLFW3_LIBRARY}
        ${GLEW_LIBRARY}
        ${TinyXML_LIBRARIES}
        ${OPENGL_LIBRARIES}
        Threads::Threads
        )

if(APPLE)
        find_library(COCOA NAMES Cocoa)
        find_library(IOKIT_FRAMEWORK NAMES IOKit)
        find_library(COREVIDEO CoreVideo)

        list(APPEND TARGET_LIBRARIES
                ${COCOA}
                ${IOKIT_FRAMEWORK}
                ${COREVIDEO}
                )
endif()


include_directories("./")
include_directories("rendering")
include_directories("lib3ds")
include_directories("alglib/src")

FILE(GLOB LIB3DS "lib3ds/*.c")
FILE(GLOB ALGLIB "alglib/src/*.cpp")

set(GRAPHICS_FILES
        rendering/GraphicsManager.cpp
        rendering/GraphicsMesh.cpp
        rendering/GraphicsEntity.cpp
        rendering/ListenerBase.cpp
        rendering/Camera.cpp
        rendering/CameraListener.cpp
        rendering/DebugDisplay.cpp
        rendering/RayCastEntity.cpp
    )

set(EXHAUSTIVE_SEARCH
        Util.cpp
        ExhaustiveSimulation.cpp
        PhysicsUtils.cpp
        ${LIB3DS}
    )

set(VISUAL_SEARCH
        Util.cpp
        VisualSimulation.cpp
        PhysicsUtils.cpp
        ${LIB3DS}
        ${ALGLIB}
        )

#utility aplicaiton for extracting 3D model surface points
add_executable(ExtractSurfacePointsGUI.exe ExtractSurfacePointsGUI.cpp ${EXHAUSTIVE_SEARCH} ${GRAPHICS_FILES})
target_link_libraries(ExtractSurfacePointsGUI.exe ${TARGET_LIBRARIES})

#exhaustive search simulation for finding tool-object interaction
add_executable(ExhaustiveSearchGUI.exe ExhaustiveSearchGUI.cpp ${EXHAUSTIVE_SEARCH} ${GRAPHICS_FILES})
target_link_libraries(ExhaustiveSearchGUI.exe ${TARGET_LIBRARIES})

add_executable(ExhaustiveSearch.exe ExhaustiveSearch.cpp ${EXHAUSTIVE_SEARCH})
target_link_libraries(ExhaustiveSearch.exe ${TARGET_LIBRARIES})

#visual search simulation for finding tool-object interaction
add_executable(VisualSearchGUI.exe VisualSearchGUI.cpp ${VISUAL_SEARCH} ${GRAPHICS_FILES})
target_link_libraries(VisualSearchGUI.exe ${TARGET_LIBRARIES})
