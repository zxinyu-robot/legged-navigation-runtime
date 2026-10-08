cmake_minimum_required(VERSION 3.20)

project(
  legged_navigation_runtime
  VERSION 0.1.0
  DESCRIPTION "Fault-tolerant localization runtime for legged robots"
  LANGUAGES CXX
)

include(CTest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LNR_BUILD_DEMO "Build the recovery scenario demo" ON)

add_library(legged_navigation_runtime
  src/relocalization_supervisor.cpp
  src/latency_window.cpp
)
add_library(legged_navigation::runtime ALIAS legged_navigation_runtime)

target_include_directories(legged_navigation_runtime
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(legged_navigation_runtime PRIVATE -Wall -Wextra -Wpedantic)
elseif(MSVC)
  target_compile_options(legged_navigation_runtime PRIVATE /W4)
endif()

if(LNR_BUILD_DEMO)
  add_executable(recovery_demo src/recovery_demo.cpp)
  target_link_libraries(recovery_demo PRIVATE legged_navigation::runtime)
endif()

if(BUILD_TESTING)
  add_executable(runtime_tests tests/runtime_tests.cpp)
  target_link_libraries(runtime_tests PRIVATE legged_navigation::runtime)
  add_test(NAME runtime_tests COMMAND runtime_tests)
endif()
