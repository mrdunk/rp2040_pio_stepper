cmake_minimum_required(VERSION 3.14)

include(${CMAKE_SOURCE_DIR}/libraries/FetchCMocka.cmake)

include_directories("../rp2040")
include_directories("${CMAKE_SOURCE_DIR}/build/_deps/cmocka-src/include/")

add_compile_options(
  -Ofast
  #-O0
  -Wall
  -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
  -Wno-unused-function # we have some for the docs that aren't called
  -Wno-maybe-uninitialized
  )

add_definitions(-DBUILD_TESTS)

add_executable(
    ringBufferTest
    ${CMAKE_CURRENT_SOURCE_DIR}/ring_buffer_test.c
    ${CMAKE_SOURCE_DIR}/src/rp2040/ring_buffer.c
    )
target_compile_features(
  ringBufferTest PRIVATE
  c_std_99
  )
target_link_libraries(
    ringBufferTest
    cmocka
    -Wl,--wrap,time_us_64
    -Wl,--wrap,tight_loop_contents
    -Wl,--wrap,update_period
    )

add_executable(
  rpNetworkTest
    ${CMAKE_CURRENT_SOURCE_DIR}/rp_network_test.c
    ${CMAKE_SOURCE_DIR}/src/rp2040/core0.c
    ${CMAKE_SOURCE_DIR}/src/rp2040/config.c
    ${CMAKE_SOURCE_DIR}/src/rp2040/gpio.c
    ${CMAKE_SOURCE_DIR}/src/shared/buffer.c
    ${CMAKE_SOURCE_DIR}/src/shared/checksum.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks/rp_mocks.c
    #${CMAKE_CURRENT_SOURCE_DIR}/mocks/config_mocks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks/ringbuffer_mocks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks/network_mocks.c
    )
target_link_libraries(
    rpNetworkTest
    cmocka
    )
add_test(
    rpNetworkTest
    rpNetworkTest
    )


add_executable(
  rpNetworkTxTest
    ${CMAKE_CURRENT_SOURCE_DIR}/rp_network_tx_test.c
    ${CMAKE_SOURCE_DIR}/src/rp2040/core0.c
    ${CMAKE_SOURCE_DIR}/src/rp2040/config.c
    ${CMAKE_SOURCE_DIR}/src/rp2040/gpio.c
    ${CMAKE_SOURCE_DIR}/src/shared/buffer.c
    ${CMAKE_SOURCE_DIR}/src/shared/checksum.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks/rp_mocks.c
    #${CMAKE_CURRENT_SOURCE_DIR}/mocks/config_mocks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks/ringbuffer_mocks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks/network_mocks.c
    )
target_link_libraries(
    rpNetworkTxTest
    cmocka
    )
add_test(
    rpNetworkTxTest
    rpNetworkTxTest
    )


add_executable(
  rpGpioTest
    ${CMAKE_CURRENT_SOURCE_DIR}/rp_gpio_test.c
    ${CMAKE_SOURCE_DIR}/src/rp2040/gpio.c
    #${CMAKE_SOURCE_DIR}/src/rp2040/core0.c
    ${CMAKE_SOURCE_DIR}/src/rp2040/config.c
    ${CMAKE_SOURCE_DIR}/src/shared/buffer.c
    ${CMAKE_SOURCE_DIR}/src/shared/checksum.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks/rp_mocks.c
    #${CMAKE_CURRENT_SOURCE_DIR}/mocks/ringbuffer_mocks.c
    #${CMAKE_CURRENT_SOURCE_DIR}/mocks/network_mocks.c
    )
target_link_libraries(
    rpGpioTest
    cmocka
    -Wl,--wrap=gpio_put
    -Wl,--wrap=gpio_get
    )
add_test(
    rpGpioTest
    rpGpioTest
    )


add_executable(
    driverNetworkPCtoRPTest
    ${CMAKE_CURRENT_SOURCE_DIR}/driver_network_PCtoRP_test.c
    #${CMAKE_SOURCE_DIR}/src/driver/rp2040_network.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks/driver_mocks.c
    )
target_link_libraries(
    driverNetworkPCtoRPTest
    cmocka
    )
add_test(
    driverNetworkPCtoRPTest
    driverNetworkPCtoRPTest
    )


add_executable(
    driverNetworkRPtoPCTest
    ${CMAKE_CURRENT_SOURCE_DIR}/driver_network_RPtoPC_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks/driver_mocks.c
    )
target_link_libraries(
    driverNetworkRPtoPCTest
    cmocka
    )
add_test(
    driverNetworkRPtoPCTest
    driverNetworkRPtoPCTest
    )


add_executable(
    driverGpioTest
    ${CMAKE_CURRENT_SOURCE_DIR}/driver_gpio_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks/driver_mocks.c
    )
target_link_libraries(
    driverGpioTest
    cmocka
    )
add_test(
    driverGpioTest
    driverGpioTest
    )


add_executable(
    sharedBufferTest
    ${CMAKE_CURRENT_SOURCE_DIR}/shared_buffer_test.c
    )
target_link_libraries(
    sharedBufferTest
    cmocka::cmocka
    ${CMAKE_SOURCE_DIR}/src/shared/checksum.c
    ${CMAKE_SOURCE_DIR}/src/shared/buffer.c
    -Wl,--wrap=checksum
    )
add_test(
    sharedBufferTest
    sharedBufferTest
    )


add_executable(
    sharedChecksumTest
    ${CMAKE_CURRENT_SOURCE_DIR}/shared_checksum_test.c
    )
target_link_libraries(
    sharedChecksumTest
    cmocka::cmocka
    ${CMAKE_SOURCE_DIR}/src/shared/checksum.c
    )
add_test(
    sharedChecksumTest
    sharedChecksumTest
    )

