cmake_minimum_required(VERSION 3.20)
project(legged_terrain_state VERSION 0.1.0 LANGUAGES CXX)

include(CTest)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(legged_terrain_state src/terrain_assessor.cpp)
add_library(legged_terrain::state ALIAS legged_terrain_state)
target_include_directories(legged_terrain_state PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(legged_terrain_state PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(terrain_demo src/terrain_demo.cpp)
target_link_libraries(terrain_demo PRIVATE legged_terrain::state)

if(BUILD_TESTING)
  add_executable(terrain_tests tests/terrain_tests.cpp)
  target_link_libraries(terrain_tests PRIVATE legged_terrain::state)
  add_test(NAME terrain_tests COMMAND terrain_tests)
endif()
