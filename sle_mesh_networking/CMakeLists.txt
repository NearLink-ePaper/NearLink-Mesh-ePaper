#===============================================================================
# @brief    cmake file
# Copyright (c) CompanyNameMagicTag 2023-2023. All rights reserved.
#===============================================================================
if(DEFINED CONFIG_SAMPLE_SUPPORT_SLE_MESH_SERVER)
set(SOURCES_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_server/sle_uart_server_adv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_server/sle_uart_server.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart.c
)
set(HEADER_LIST ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_server)

elseif(DEFINED CONFIG_SAMPLE_SUPPORT_SLE_MESH_CLIENT)
set(SOURCES_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_client/sle_uart_client.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart.c
)
set(HEADER_LIST ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_client)

elseif(DEFINED CONFIG_SAMPLE_SUPPORT_SLE_MESH)
# ---- Mesh 模式: 同时编译 Server + Client + Mesh 模块 ----
set(SOURCES_LIST
    # 原有 server/client 代码
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_server/sle_uart_server_adv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_server/sle_uart_server.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_client/sle_uart_client.c
    # Mesh 组网核心 (network/)
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/network/mesh_main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/network/mesh_transport.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/network/mesh_forward.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/network/mesh_route.c
    # BLE 网关 (gateway/)
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/gateway/ble_gateway.c
    # 图传模块 (image/)
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/image/image_receiver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/image/image_rle.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/image/tjpgd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/image/jpeg_decoder.c
    # ePaper 模块 (epaper/)
    ${CMAKE_CURRENT_SOURCE_DIR}/epaper/epaper.c
    ${CMAKE_CURRENT_SOURCE_DIR}/epaper/epaper_init.c
    # WiFi SoftAP + Socket 图传 (wifi/)
    ${CMAKE_CURRENT_SOURCE_DIR}/wifi/wifi_softap.c
    ${CMAKE_CURRENT_SOURCE_DIR}/wifi/wifi_socket_server.c
)
set(HEADER_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_server
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_client
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/common
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/network
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/gateway
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/image
    ${CMAKE_CURRENT_SOURCE_DIR}/epaper
    ${CMAKE_CURRENT_SOURCE_DIR}/wifi
)

endif()

set(SOURCES "${SOURCES}" ${SOURCES_LIST} PARENT_SCOPE)
set(PUBLIC_HEADER "${PUBLIC_HEADER}" ${HEADER_LIST} PARENT_SCOPE)