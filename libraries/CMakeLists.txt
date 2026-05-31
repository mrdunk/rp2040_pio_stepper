# Ethernet
add_library(ETHERNET_FILES STATIC)

target_sources(ETHERNET_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet/socket.c
        ${WIZNET_DIR}/Ethernet/wizchip_conf.c
        )

if(ETH_CHIP STREQUAL "W5100S")
    target_include_directories(ETHERNET_FILES INTERFACE
            ${WIZNET_DIR}/Ethernet
            ${WIZNET_DIR}/Ethernet/W5100S
            )
    target_link_libraries(ETHERNET_FILES PUBLIC W5100S_FILES)

elseif(ETH_CHIP STREQUAL "W5500")
    target_include_directories(ETHERNET_FILES INTERFACE
            ${WIZNET_DIR}/Ethernet
            ${WIZNET_DIR}/Ethernet/W5500
            )
    target_link_libraries(ETHERNET_FILES PUBLIC W5500_FILES)

elseif(ETH_CHIP STREQUAL "W6100")
    # W6100 driver is not yet wired up. Update ioLibrary_Driver submodule (issue #35)
    # and add W6100 source paths here once available.
    message(FATAL_ERROR "ETH_CHIP=W6100: ioLibrary_Driver W6100 support not yet integrated (issue #35)")

elseif(ETH_CHIP STREQUAL "W6300")
    # W6300 driver is not yet wired up. Update ioLibrary_Driver submodule (issue #36)
    # and add W6300 source paths here once available.
    message(FATAL_ERROR "ETH_CHIP=W6300: ioLibrary_Driver W6300 support not yet integrated (issue #36)")
endif()

if(ETH_CHIP STREQUAL "W5100S")
    add_library(W5100S_FILES STATIC)
    target_sources(W5100S_FILES PUBLIC
            ${WIZNET_DIR}/Ethernet/W5100S/w5100s.c
            )
    target_include_directories(W5100S_FILES INTERFACE
            ${WIZNET_DIR}/Ethernet
            ${WIZNET_DIR}/Ethernet/W5100S
            )
    target_link_libraries(W5100S_FILES PRIVATE ETHERNET_FILES)

elseif(ETH_CHIP STREQUAL "W5500")
    add_library(W5500_FILES STATIC)
    target_sources(W5500_FILES PUBLIC
            ${WIZNET_DIR}/Ethernet/W5500/w5500.c
            )
    target_include_directories(W5500_FILES INTERFACE
            ${WIZNET_DIR}/Ethernet
            ${WIZNET_DIR}/Ethernet/W5500
            )
    target_link_libraries(W5500_FILES PRIVATE ETHERNET_FILES)
endif()

# Loopback
add_library(LOOPBACK_FILES STATIC)

target_sources(LOOPBACK_FILES PUBLIC
        ${WIZNET_DIR}/Application/loopback/loopback.c
        )

target_include_directories(LOOPBACK_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Application/loopback
        )

