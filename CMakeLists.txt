cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Point git at the tracked hooks directory so all developers get the pre-commit hook.
execute_process(
    COMMAND git config core.hooksPath scripts/hooks
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

include_directories (./src/shared/)


OPTION(BUILD_RP "Build all examples" OFF)
if(BUILD_RP)
    if(NOT DEFINED PICO_SDK_PATH)
        set(PICO_SDK_PATH ${CMAKE_SOURCE_DIR}/libraries/pico-sdk)
        message(STATUS "PICO_SDK_PATH = ${PICO_SDK_PATH}")
    endif()

    if(NOT DEFINED WIZNET_DIR)
        set(WIZNET_DIR ${CMAKE_SOURCE_DIR}/libraries/ioLibrary_Driver)
        message(STATUS "WIZNET_DIR = ${WIZNET_DIR}")
    endif()

    if(NOT DEFINED PORT_DIR)
        set(PORT_DIR ${CMAKE_SOURCE_DIR}/port)
        message(STATUS "PORT_DIR = ${PORT_DIR}")
    endif()

    # ETH_CHIP: Ethernet controller.
    # W5500/W5100S: supported via libraries/ioLibrary_Driver.
    # W6100/W6300: require ioLibrary_Driver submodule update — see issues #35/#36.
    set(ETH_CHIP "W5500" CACHE STRING "Ethernet chip (W5500, W5100S, W6100, W6300)")
    set_property(CACHE ETH_CHIP PROPERTY STRINGS W5500 W5100S W6100 W6300)

    # RP_CHIP: MCU silicon variant.
    # RP2350 requires pico-sdk v2.x — update the libraries/pico-sdk submodule first.
    set(RP_CHIP "RP2040" CACHE STRING "RP MCU variant (RP2040, RP2350)")
    set_property(CACHE RP_CHIP PROPERTY STRINGS RP2040 RP2350)

    # Derive PICO_BOARD from RP_CHIP + ETH_CHIP unless the caller supplies it.
    if(NOT DEFINED PICO_BOARD)
        string(TOLOWER "${ETH_CHIP}" _eth_lower)
        if(RP_CHIP STREQUAL "RP2350")
            set(PICO_BOARD "wiznet_${_eth_lower}_evb_pico2")
        else()
            set(PICO_BOARD "wiznet_${_eth_lower}_evb_pico")
        endif()
        message(STATUS "PICO_BOARD = ${PICO_BOARD} (derived from RP_CHIP=${RP_CHIP}, ETH_CHIP=${ETH_CHIP})")
    endif()

    # Project-local board headers (W5500, W6100, W6300 variants not yet in upstream pico-sdk).
    # Searched after the SDK's own boards directory, so SDK-provided headers take precedence.
    list(APPEND PICO_BOARD_HEADER_DIRS ${CMAKE_SOURCE_DIR}/boards)

    include(rp2040_hat_c-patch.cmake)
    include(pico_sdk_import.cmake)
    include(rp2040_hat_c_sdk_version.cmake)

    set(PROJECT_NAME RP2040-STEPPER-C)
    project(test_project C CXX ASM)

    # Initialize the SDK
    pico_sdk_init()

    # Number of stepper joints (1-8); determines PIO layout and feedback allocation
    set(MAX_JOINT 8 CACHE STRING "Number of stepper joints (1-8)")

    # Print joint/GPIO/spindle config details on UART as they are received.
    # Disable for production builds; UART blocking during startup causes missed UDP replies.
    option(VERBOSE_CONFIG_LOG "Log config message details to UART" OFF)

    if(ETH_CHIP STREQUAL "W5100S")
        add_definitions(-D_WIZCHIP_=W5100S)
    elseif(ETH_CHIP STREQUAL "W5500")
        add_definitions(-D_WIZCHIP_=W5500)
    elseif(ETH_CHIP STREQUAL "W6100")
        add_definitions(-D_WIZCHIP_=W6100)
    elseif(ETH_CHIP STREQUAL "W6300")
        add_definitions(-D_WIZCHIP_=W6300)
    else()
        message(FATAL_ERROR "ETH_CHIP is invalid: ${ETH_CHIP}. Must be one of: W5500, W5100S, W6100, W6300")
    endif()

    message(STATUS "ETH_CHIP = ${ETH_CHIP}")
    message(STATUS "RP_CHIP  = ${RP_CHIP}")

    add_subdirectory(src/rp2040)
    add_subdirectory(${CMAKE_SOURCE_DIR}/libraries)
    add_subdirectory(${PORT_DIR})
endif()

OPTION(BUILD_TESTS "Build the test binary" OFF)
if(BUILD_TESTS)
  enable_testing()

  enable_language(C CXX)
  add_subdirectory(./src/test)
endif()

