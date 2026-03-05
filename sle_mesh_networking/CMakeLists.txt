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
    # 原有 server/client 代码 (已修改)
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_server/sle_uart_server_adv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_server/sle_uart_server.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_client/sle_uart_client.c
    # Mesh 新增代码
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/mesh_main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/mesh_transport.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/mesh_forward.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/mesh_route.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/ble_gateway.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh/image_receiver.c
)
set(HEADER_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_server
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_uart_client
    ${CMAKE_CURRENT_SOURCE_DIR}/mesh
)

endif()

set(SOURCES "${SOURCES}" ${SOURCES_LIST} PARENT_SCOPE)
set(PUBLIC_HEADER "${PUBLIC_HEADER}" ${HEADER_LIST} PARENT_SCOPE)