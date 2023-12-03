cmake_minimum_required(VERSION 3.13)


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
    cmocka #-static
    -Wl,--wrap,time_us_64
    -Wl,--wrap,tight_loop_contents
    -Wl,--wrap,update_period
    )

  #add_executable(
  #  timingTest
  #  ${CMAKE_SOURCE_DIR}/src/rp2040/timing.c
  #  )
  #target_link_libraries(
  #  timingTest
  #  cmocka #-static
  #  -Wl,--wrap,time_us_64
  #  -Wl,--wrap,ring_buf_uint_ave
  #  -Wl,--wrap,tight_loop_contents
  #  -Wl,--wrap,update_period
  #  )

# pass -D BUILD_TESTS when compiling
#target_compile_definitions(
#    networkTest PUBLIC
#    -D BUILD_TESTS
#    )
add_definitions(-DBUILD_TESTS)
add_executable(
    networkTest
    ${CMAKE_CURRENT_SOURCE_DIR}/network_test.c
    ${CMAKE_SOURCE_DIR}/src/rp2040/core0.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks/rp_mocks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks/config_mocks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks/ringbuffer_mocks.c
    )
target_link_libraries(
    networkTest
    cmocka #-static
    #-Wl,--wrap,time_us_64
    #-Wl,--wrap,ring_buf_uint_ave
    #-Wl,--wrap,tight_loop_contents
    #-Wl,--wrap,update_period
    )



