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
    target_include_directories(ETHERNET_FILES INTERFACE
            ${WIZNET_DIR}/Ethernet
            ${WIZNET_DIR}/Ethernet/W6100
            )
    target_link_libraries(ETHERNET_FILES PUBLIC W6100_FILES)

elseif(ETH_CHIP STREQUAL "W6300")
    target_include_directories(ETHERNET_FILES INTERFACE
            ${WIZNET_DIR}/Ethernet
            ${WIZNET_DIR}/Ethernet/W6300
            )
    target_link_libraries(ETHERNET_FILES PUBLIC W6300_FILES)
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

elseif(ETH_CHIP STREQUAL "W6100")
    add_library(W6100_FILES STATIC)
    target_sources(W6100_FILES PUBLIC
            ${WIZNET_DIR}/Ethernet/W6100/w6100.c
            )
    target_include_directories(W6100_FILES INTERFACE
            ${WIZNET_DIR}/Ethernet
            ${WIZNET_DIR}/Ethernet/W6100
            )
    target_link_libraries(W6100_FILES PRIVATE ETHERNET_FILES)

elseif(ETH_CHIP STREQUAL "W6300")
    add_library(W6300_FILES STATIC)
    target_sources(W6300_FILES PUBLIC
            ${WIZNET_DIR}/Ethernet/W6300/w6300.c
            )
    target_include_directories(W6300_FILES INTERFACE
            ${WIZNET_DIR}/Ethernet
            ${WIZNET_DIR}/Ethernet/W6300
            )
    target_link_libraries(W6300_FILES PRIVATE ETHERNET_FILES)
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

