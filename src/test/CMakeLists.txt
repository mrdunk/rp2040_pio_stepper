cmake_minimum_required(VERSION 3.13)


include(${CMAKE_SOURCE_DIR}/libraries/FetchCMocka.cmake)

include_directories("../rp2040")
include_directories("${CMAKE_SOURCE_DIR}/build/_deps/cmocka-src/include/")

add_executable(
    ringBufferTest
    ${CMAKE_CURRENT_SOURCE_DIR}/ring_buffer_test.c
    #${CMAKE_CURRENT_SOURCE_DIR}/src/rp2040/timing.c
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
  #  ${CMAKE_CURRENT_SOURCE_DIR}/timing_test.c
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

#add_executable(
#    networkTest
#    ${CMAKE_CURRENT_SOURCE_DIR}/network_test.c
#    ${CMAKE_SOURCE_DIR}/src/rp2040/stepper_control.c
#    )
#target_link_libraries(
#    networkTest
#    cmocka #-static
#    -Wl,--wrap,time_us_64
#    -Wl,--wrap,ring_buf_uint_ave
#    -Wl,--wrap,tight_loop_contents
#    -Wl,--wrap,update_period
#    )



