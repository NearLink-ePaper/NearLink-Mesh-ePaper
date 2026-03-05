/**
 * @file  sle_uart_server.h
 * @brief SLE UART Server 模块头文件（BearPi-Pico H3863 SLE Mesh 图传）
 * @details
 *   声明 SLE 服务端（Server 角色）对外接口，主要功能包括：
 *   - SSAP Service / Property / Descriptor 注册与管理
 *   - 通过 Notify（ssaps_notify_indicate）向已连接的 Client 发送数据
 *   - 接收 Client 的 read/write request 回调
 *   - Mesh 模式下多连接支持：按 conn_id 发送 Notify
 *   - 与 mesh_transport 层交互：连接/断开事件上报、数据收发转发
 *
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * History:
 *   2023-07-17, Create file.
 *   2026-02-07, Modified for Mesh support.
 */

#ifndef SLE_UART_SERVER_H
#define SLE_UART_SERVER_H

#include <stdint.h>
#include "sle_ssap_server.h"
#include "errcode.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/* === UUID 与权限宏定义 === */

#define SLE_UUID_SERVER_SERVICE        0x2222  /**< SSAP 服务 UUID，标识本 Server 提供的服务 */

#define SLE_UUID_SERVER_NTF_REPORT     0x2323  /**< SSAP Property UUID，用于 Notify 上报数据 */

#define SLE_UUID_TEST_PROPERTIES  (SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE)  /**< Property 读写权限 */

#define SLE_UUID_TEST_OPERATION_INDICATION  (SSAP_OPERATE_INDICATION_BIT_READ | SSAP_OPERATE_INDICATION_BIT_WRITE)  /**< Property 操作指示位：支持读与写 */

#define SLE_UUID_TEST_DESCRIPTOR   (SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE)  /**< Descriptor 读写权限 */

/* ============================================================
 *  原有接口（保持兼容，非 Mesh 场景亦可使用）
 * ============================================================ */

/**
 * @brief  初始化 SLE UART Server 模块
 * @param  ssaps_read_callback  [in] SSAP 读请求回调（Client 读取 Property 时触发）
 * @param  ssaps_write_callback [in] SSAP 写请求回调（Client 写入 Property 时触发）
 * @return ERRCODE_SLE_SUCCESS 成功，其他值失败
 * @note   内部流程：使能 SLE → 注册广播/连接/SSAP 回调 → 注册服务 → 启动广播
 */
errcode_t sle_uart_server_init(ssaps_read_request_callback ssaps_read_callback,
    ssaps_write_request_callback ssaps_write_callback);

/**
 * @brief  通过 UUID 方式向当前连接的 Client 发送 Notify 数据
 * @param  data [in] 待发送数据指针
 * @param  len  [in] 数据长度（字节）
 * @return ERRCODE_SLE_SUCCESS 成功，其他值失败
 */
errcode_t sle_uart_server_send_report_by_uuid(const uint8_t *data, uint8_t len);

/**
 * @brief  通过 Handle 方式向当前连接的 Client 发送 Notify 数据
 * @param  data [in] 待发送数据指针
 * @param  len  [in] 数据长度（字节）
 * @return ERRCODE_SLE_SUCCESS 成功，其他值失败
 */
errcode_t sle_uart_server_send_report_by_handle(const uint8_t *data, uint16_t len);

/**
 * @brief  查询当前是否有 Client 已配对（连接且完成配对）
 * @return 非零表示已配对的连接句柄+1，0 表示未配对
 */
uint16_t sle_uart_client_is_connected(void);

/**
 * @brief 消息队列回调函数类型，用于将 SLE 事件通知上层
 * @param buffer_addr [in] 消息缓冲区地址
 * @param buffer_size [in] 消息长度
 */
typedef void (*sle_uart_server_msg_queue)(uint8_t *buffer_addr, uint16_t buffer_size);

/**
 * @brief  注册上层消息队列回调（断连等事件会通过此回调通知上层）
 * @param  sle_uart_server_msg [in] 回调函数指针
 */
void sle_uart_server_register_msg(sle_uart_server_msg_queue sle_uart_server_msg);

/**
 * @brief  获取当前活动的 SLE 连接 ID
 * @return 当前连接 ID（conn_id）
 */
uint16_t get_connect_id(void);

/* ============================================================
 *  Mesh 新增接口（仅在 CONFIG_SAMPLE_SUPPORT_SLE_MESH 启用时可用）
 * ============================================================ */
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)

/**
 * @brief  向指定 conn_id 的 Client 发送 Notify 数据（Server 角色）
 * @param  conn_id [in] 目标 SLE 连接 ID
 * @param  data    [in] 待发送数据指针
 * @param  len     [in] 数据长度（字节）
 * @return ERRCODE_SUCC 成功，其他值失败
 * @note   mesh_transport 层通过此接口向指定邻居节点下发数据
 */
errcode_t sle_uart_server_send_to_conn(uint16_t conn_id, const uint8_t *data, uint16_t len);

/**
 * @brief  获取当前 SSAP Server ID
 * @return server_id（mesh_transport 层用于定向 Notify）
 */
uint8_t sle_uart_server_get_server_id(void);

/**
 * @brief  获取 Notify Property Handle
 * @return property_handle（mesh_transport 层用于构造 Notify 参数）
 */
uint16_t sle_uart_server_get_property_handle(void);

/**
 * @brief  获取 Server 角色当前已连接的 Client 数量
 * @return 已连接数（0 ~ MAX_SERVER_CONN）
 */
uint8_t sle_uart_server_get_conn_count(void);

#endif /* CONFIG_SAMPLE_SUPPORT_SLE_MESH */

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif
