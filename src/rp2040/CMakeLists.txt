add_compile_options(
  -Ofast
  #-O0
  -Wall
  -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
  -Wno-unused-function # we have some for the docs that aren't called
  -Wno-maybe-uninitialized
  )


add_executable(
  stepper_control
  stepper_control.c
)

# Compile .pio file.
# by default the header is generated into the build dir
pico_generate_pio_header(stepper_control ${CMAKE_CURRENT_LIST_DIR}/pico_stepper.pio)
# however, alternatively you can choose to generate it somewhere else (in this case in the source tree for check in)
#pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/${TARGET_NAME}.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})

# Generate build_info.h on every build by querying git and $ENV{USER}.
# stepper_control.c always recompiles so __DATE__/__TIME__ stay fresh.
add_custom_target(
    build_info
    COMMAND ${CMAKE_COMMAND}
        -DCMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}
        -DOUTPUT_FILE=${CMAKE_BINARY_DIR}/build_info.h
        -P ${CMAKE_CURRENT_SOURCE_DIR}/gen_build_info.cmake
    BYPRODUCTS ${CMAKE_BINARY_DIR}/build_info.h
)

target_include_directories(stepper_control PRIVATE ${CMAKE_BINARY_DIR})
target_compile_definitions(stepper_control PRIVATE MAX_JOINT=${MAX_JOINT})
if(VERBOSE_CONFIG_LOG)
  target_compile_definitions(stepper_control PRIVATE VERBOSE_CONFIG_LOG)
endif()
add_dependencies(stepper_control build_info)

target_sources(
  stepper_control PRIVATE
  stepper_control.c
  config.c
  network.c
  core0.c
  core1.c
  timing.c
  modbus.c
  modbus_fuling.c
  modbus_huanyang.c
  modbus_weiken.c
  pio.c
  ring_buffer.c
  gpio.c
  i2c.c
  mcp23017.c
  ../shared/buffer.c
  ../shared/checksum.c
)

target_link_libraries(
  stepper_control PRIVATE
  pico_stdlib
  pico_multicore
  hardware_spi
  hardware_dma
  hardware_pio
  hardware_i2c
  ETHERNET_FILES
  IOLIBRARY_FILES
  LOOPBACK_FILES
  )

pico_enable_stdio_usb(stepper_control 1)
pico_enable_stdio_uart(stepper_control 1)
pico_add_extra_outputs(stepper_control)

