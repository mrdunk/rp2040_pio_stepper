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

target_sources(
  stepper_control PRIVATE
  stepper_control.c
  config.c
  network.c
  core0.c
  core1.c
  modbus.c
  modbus_fuling.c
  modbus_huanyang.c
  modbus_weiken.c
  pio.c
  pio_utils.c
  ring_buffer.c
  gpio.c
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
  ETHERNET_FILES
  IOLIBRARY_FILES
  LOOPBACK_FILES
  )

pico_enable_stdio_usb(stepper_control 1)
pico_enable_stdio_uart(stepper_control 0)
pico_add_extra_outputs(stepper_control)

