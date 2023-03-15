set(TARGET_NAME sender)

add_executable(${TARGET_NAME}
        ${TARGET_NAME}.c
        )

# Compile .pio file.
# by default the header is generated into the build dir
pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/pico_stepper.pio)
# however, alternatively you can choose to generate it somewhere else (in this case in the source tree for check in)
#pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/${TARGET_NAME}.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})

target_sources(${TARGET_NAME} PRIVATE ${TARGET_NAME}.c pico_stepper.c)

target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stdlib
        hardware_pio
        hardware_spi
        hardware_dma
        ETHERNET_FILES
        IOLIBRARY_FILES
        LOOPBACK_FILES
        )

pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

pico_add_extra_outputs(${TARGET_NAME})
