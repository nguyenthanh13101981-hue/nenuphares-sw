cmake_minimum_required(VERSION 3.16)

option(C8_BUILD_PICO_FIRMWARE "Build RP2350 Pico 2 bare-metal firmware using pico-sdk" OFF)

if(C8_BUILD_PICO_FIRMWARE)
  # When building for RP2350 hardware, set PICO_BOARD=pico2 and PICO_SDK_PATH before cmake.
  include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
endif()

project(nenuphar_c8_pico2 LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(c8_core STATIC
  core/lora_protocol.cpp
  core/safety_fsm.cpp
)
target_include_directories(c8_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(c8_core PRIVATE -Wall -Wextra -Werror)

if(C8_BUILD_PICO_FIRMWARE)
  pico_sdk_init()
  add_executable(nenuphar_c8_fw
    firmware/main.cpp
    firmware/pico_hal.cpp
  )
  target_link_libraries(nenuphar_c8_fw PRIVATE c8_core pico_stdlib hardware_gpio)
  target_compile_options(nenuphar_c8_fw PRIVATE -Wall -Wextra -Werror)
  pico_add_extra_outputs(nenuphar_c8_fw)
else()
  enable_testing()
  find_package(GTest REQUIRED)

  add_executable(test_c8_safety tests/test_c8_safety.cpp)
  target_link_libraries(test_c8_safety PRIVATE c8_core GTest::gtest_main)
  target_compile_options(test_c8_safety PRIVATE -Wall -Wextra -Werror)

  include(GoogleTest)
  gtest_discover_tests(test_c8_safety)
endif()
