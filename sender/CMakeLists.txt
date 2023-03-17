set(TARGET_NAME sender)

add_executable(${TARGET_NAME}
        ${TARGET_NAME}.c
        )

target_include_directories (${TARGET_NAME} PUBLIC ../shared/)
target_sources(${TARGET_NAME} PRIVATE ${TARGET_NAME}.c)

target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stepper
        pico_stdlib
        hardware_spi
        hardware_dma
        ETHERNET_FILES
        IOLIBRARY_FILES
        LOOPBACK_FILES
        )

pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

pico_add_extra_outputs(${TARGET_NAME})
