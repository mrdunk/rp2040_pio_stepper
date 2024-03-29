cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include_directories (./src/shared/)


OPTION(BUILD_RP "Build all examples" OFF)
if(BUILD_RP)
    # Pull in SDK (must be before project)
    if(NOT DEFINED PICO_SDK_PATH)
        set(PICO_SDK_PATH ${CMAKE_SOURCE_DIR}/libraries/pico-sdk)
        message(STATUS "PICO_SDK_PATH = ${PICO_SDK_PATH}")
    endif()

    if(NOT DEFINED WIZNET_DIR)
        set(WIZNET_DIR ${CMAKE_SOURCE_DIR}/libraries/ioLibrary_Driver)
        message(STATUS "WIZNET_DIR = ${WIZNET_DIR}")
    endif()

    if(NOT DEFINED MBEDTLS_DIR)
        set(MBEDTLS_DIR ${CMAKE_SOURCE_DIR}/libraries/mbedtls)
        message(STATUS "MBEDTLS_DIR = ${MBEDTLS_DIR}")
    endif()

    if(NOT DEFINED PORT_DIR)
        set(PORT_DIR ${CMAKE_SOURCE_DIR}/port)
        message(STATUS "PORT_DIR = ${PORT_DIR}")
    endif()

    include(rp2040_hat_c-patch.cmake)
    include(pico_sdk_import.cmake)
    include(rp2040_hat_c_sdk_version.cmake)

    # Set project name
    set(PROJECT_NAME RP2040-STEPPER-C)

    # Set project informations
    project(test_project C CXX ASM)

    # Initialize the SDK
    pico_sdk_init()

    # Set ethernet chip
    set(WIZNET_CHIP W5500)

    if(${WIZNET_CHIP} STREQUAL W5100S)
        add_definitions(-D_WIZCHIP_=W5100S)
    elseif(${WIZNET_CHIP} STREQUAL W5500)
        add_definitions(-D_WIZCHIP_=W5500)
    else()
        message(FATAL_ERROR "WIZNET_CHIP is wrong = ${WIZNET_CHIP}")
    endif()

    message(STATUS "WIZNET_CHIP = ${WIZNET_CHIP}")


    # Turn off mbedtls test mode
    set(ENABLE_PROGRAMS OFF CACHE BOOL "Build mbedtls programs")
    set(ENABLE_TESTING OFF CACHE BOOL "Build mbedtls testing")
    add_definitions(-DMBEDTLS_CONFIG_FILE="${PORT_DIR}/mbedtls/inc/ssl_config.h")
    add_definitions(-DSET_TRUSTED_CERT_IN_SAMPLES)

    # Code to include:
    add_subdirectory(src/rp2040)

    # Add libraries in subdirectories
    add_subdirectory(${CMAKE_SOURCE_DIR}/libraries)
    add_subdirectory(${MBEDTLS_DIR})
    add_subdirectory(${PORT_DIR})
endif()

OPTION(BUILD_TESTS "Build the test binary" OFF)
if(BUILD_TESTS)
  enable_testing()

  enable_language(C CXX)
  add_subdirectory(./src/test)
endif()

