cmake_minimum_required(VERSION 3.12)
project(simple-sim)

set(CMAKE_CXX_STANDARD 17)

include_directories(
    include
)

find_package(SFML 2 REQUIRED COMPONENTS graphics)

file(COPY test/scene.json DESTINATION ${CMAKE_BINARY_DIR})

add_executable(simple-sim
    src/sim.cpp
    test/test.cpp
)

target_link_libraries(simple-sim PRIVATE sfml-graphics)
