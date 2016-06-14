cmake_minimum_required(VERSION 3.5)
project(MatlabNewtonDynamics)


SET(NEWTON_DEMOS_SANDBOX OFF CACHE BOOL "Disable sandbox demo build" FORCE)
add_subdirectory("newton-dynamics")
add_subdirectory("OpenGLRender")

set(SOURCE_FILES main.cpp)
add_executable(MatlabNewtonDynamics ${SOURCE_FILES})
target_link_libraries(MatlabNewtonDynamics dMath dScene dCustomJoints dContainers dAnimation)
