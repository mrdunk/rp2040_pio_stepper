cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include_directories (./src/shared/)

# Set RP chip version


if(NOT DEFINED PICO_PLATFORM)
    set(PICO_BOARD pico)
#   set(PICO_BOARD pico2)
endif()

# Set ethernet chip


if(NOT DEFINED WIZNET_CHIP)
#   set(WIZNET_CHIP W5100S)
    set(WIZNET_CHIP W5500)
endif()

####################################################################################

OPTION(BUILD_RP "Build CNC firmware for RP2040/RP2350" ON)
if(BUILD_RP)
    # Pull in SDK (must be before project)
    if(NOT DEFINED PICO_SDK_PATH)
        set(PICO_SDK_PATH ${CMAKE_SOURCE_DIR}/libraries/pico-sdk)
    endif()

    if(NOT DEFINED WIZNET_DIR)
        set(WIZNET_DIR ${CMAKE_SOURCE_DIR}/libraries/ioLibrary_Driver)
    endif()

    if(NOT DEFINED PORT_DIR)
        set(PORT_DIR ${CMAKE_SOURCE_DIR}/port)
    endif()

    message(STATUS "PICO_SDK_PATH = ${PICO_SDK_PATH}")
    message(STATUS "WIZNET_DIR = ${WIZNET_DIR}")
    message(STATUS "PORT_DIR = ${PORT_DIR}")

    include(rp2040_hat_c-patch.cmake)
    include(pico_sdk_import.cmake)
    include(rp2040_hat_c_sdk_version.cmake)

    # Set project name
    set(PROJECT_NAME RP2040-STEPPER-C)

    # Set project informations
    project(test_project C CXX ASM)

    # Initialize the SDK
    pico_sdk_init()

    if(${WIZNET_CHIP} STREQUAL W5100S)
        add_definitions(-D_WIZCHIP_=W5100S)
    elseif(${WIZNET_CHIP} STREQUAL W5500)
        add_definitions(-D_WIZCHIP_=W5500)
    else()
        message(FATAL_ERROR "WIZNET_CHIP is wrong = ${WIZNET_CHIP}")
    endif()

    message(STATUS "*************************************************")
    message(STATUS "Hardware target:")
    message(STATUS "")
    message(STATUS "PICO_BOARD = ${PICO_BOARD}")
    message(STATUS "PICO_PLATFORM = ${PICO_PLATFORM}")
    message(STATUS "WIZNET_CHIP = ${WIZNET_CHIP}")
    message(STATUS "*************************************************")


    # Code to include:
    add_subdirectory(src/rp2040)

    # Add libraries in subdirectories
    add_subdirectory(${CMAKE_SOURCE_DIR}/libraries)
    add_subdirectory(${PORT_DIR})
endif()

OPTION(BUILD_TESTS "Build the test binary" OFF)
if(BUILD_TESTS)
  enable_testing()

  enable_language(C CXX)
  add_subdirectory(./src/test)
endif()

