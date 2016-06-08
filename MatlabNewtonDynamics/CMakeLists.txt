cmake_minimum_required(VERSION 3.5)
project(MatlabNewtonDynamics)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include(ExternalProject)
ExternalProject_Add(NewtonSDK
        GIT_REPOSITORY git@github.com:MADEAPPS/newton-dynamics.git
        GIT_TAG master
        UPDATE_COMMAND ""
        CMAKE_ARGS
        "-DNEWTON_DEMOS_SANDBOX=OFF"
        )

set(SOURCE_FILES main.cpp)

add_executable(MatlabNewtonDynamics ${SOURCE_FILES})
add_dependencies(MatlabNewtonDynamics NewtonSDK)
link_libraries(NewtonSDK)
