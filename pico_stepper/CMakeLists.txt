set(TARGET_NAME pico_stepper)

add_library(${TARGET_NAME} STATIC)

# Compile .pio file.
# by default the header is generated into the build dir
pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/${TARGET_NAME}.pio)
# however, alternatively you can choose to generate it somewhere else (in this case in the source tree for check in)
#pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/${TARGET_NAME}.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})

target_include_directories (${TARGET_NAME} PUBLIC ../shared/)
target_sources(${TARGET_NAME} PRIVATE ${TARGET_NAME}.c)
target_include_directories (${TARGET_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        )
