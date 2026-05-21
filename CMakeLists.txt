cmake_minimum_required(VERSION 3.12)
project(simple-sim)

set(CMAKE_CXX_STANDARD 17)

include_directories(
    include
)

file(COPY test/scene.json DESTINATION ${CMAKE_BINARY_DIR})

add_executable(simple-sim
    src/physics_engine.cpp
    test/test.cpp
)