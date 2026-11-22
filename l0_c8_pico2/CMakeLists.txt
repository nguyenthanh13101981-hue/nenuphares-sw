cmake_minimum_required(VERSION 3.16)
project(nenuphar_c8_pico2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(c8_core STATIC
  core/lora_protocol.cpp
  core/safety_fsm.cpp
)
target_include_directories(c8_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(c8_core PRIVATE -Wall -Wextra -Werror)

enable_testing()
find_package(GTest REQUIRED)

add_executable(test_c8_safety tests/test_c8_safety.cpp)
target_link_libraries(test_c8_safety PRIVATE c8_core GTest::gtest_main)
target_compile_options(test_c8_safety PRIVATE -Wall -Wextra -Werror)

include(GoogleTest)
gtest_discover_tests(test_c8_safety)
