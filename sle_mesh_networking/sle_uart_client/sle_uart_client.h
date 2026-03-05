/**
 * @file   sle_uart_client.h
 * @brief  SLE UART 客户端模块 —— 对外接口声明
 * @details
 *   本模块是 BearPi-Pico H3863 SLE Mesh 图传方案中的 **Client 角色** 头文件。
 *   主要职责：
 *   - 声明扫描 / 连接 / 服务发现 / 数据收发的 API
 *   - 在 Mesh 模式下，额外暴露出站连接判断、统一回调转接、
 *     断线重连与退避清理等函数，供 Server 端统一回调及主循环调用
 *
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * History:
 *   2023-04-03  创建文件
 *   2026-02-07  增加 Mesh 支持（mesh_transport 集成）
 *   2026-02-08  统一连接回调 + 出站连接追踪机制
 */
#ifndef SLE_UART_CLIENT_H
#define SLE_UART_CLIENT_H

/* === 依赖头文件 === */
#include "sle_ssap_client.h"

/* ===================================================================
 *  基础 API —— 初始化、扫描控制、数据收发
 *  无论 Mesh 模式还是单机模式均可使用
 * =================================================================== */

/**
 * @brief  初始化 SLE UART 客户端
 * @param  notification_cb  对端 Notification 数据到达时的回调
 * @param  indication_cb    对端 Indication 数据到达时的回调
 * @note   内部完成：注册扫描回调、注册 SSAPC 回调。
 *         Mesh 模式下不注册连接回调（由 Server 统一管理）。
 */
void sle_uart_client_init(ssapc_notification_callback notification_cb, ssapc_indication_callback indication_cb);

/**
 * @brief  启动 SLE 设备扫描
 * @note   Mesh 模式下若 g_scan_paused 为 true（FC 传输中），本函数直接返回，
 *         只有 sle_uart_resume_scan() 能解除暂停。
 */
void sle_uart_start_scan(void);

/**
 * @brief  暂停 SLE 扫描（P3: 传输期间暂停，避免扫描干扰吞吐）
 * @note   设置持久暂停标记 g_scan_paused，阻止主循环和回调中重启扫描。
 */
void sle_uart_pause_scan(void);   /* P3: 传输期间暂停扫描 */

/**
 * @brief  恢复 SLE 扫描（P5: FC 结束后恢复）
 * @note   清除 g_scan_paused 标记后调用 sle_uart_start_scan()。
 */
void sle_uart_resume_scan(void);  /* P5: FC结束后恢复扫描 */

/**
 * @brief  获取全局写请求参数结构体指针
 * @return 指向 g_sle_uart_send_param 的指针，用于 ssapc_write_req 发送数据
 */
ssapc_write_param_t *get_g_sle_uart_send_param(void);

/**
 * @brief  SLE Notification 数据接收回调
 * @param  client_id  SSAP 客户端实例 ID
 * @param  conn_id    连接 ID
 * @param  data       收到的属性数据（handle + payload）
 * @param  status     操作状态码
 */
void sle_uart_notification_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data, errcode_t status);

/**
 * @brief  SLE Indication 数据接收回调
 * @param  client_id  SSAP 客户端实例 ID
 * @param  conn_id    连接 ID
 * @param  data       收到的属性数据（handle + payload）
 * @param  status     操作状态码
 */
void sle_uart_indication_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data, errcode_t status);

/* ===================================================================
 *  Mesh 扩展 API —— 统一连接回调转接 & 多连接管理
 *
 *  设计思路:
 *    SLE 协议栈的 sle_connection_register_callbacks() 是全局覆盖
 *    (SET) 型注册，Server 和 Client 不能各自独立注册。因此：
 *    - 连接回调统一在 Server 端注册
 *    - Server 回调中通过 sle_uart_client_check_outgoing() 判断
 *      连接方向：匹配 → Client 出站，不匹配 → Server 入站
 *    - 对应方向调用 client_on_connected / server_on_connected
 *
 *  另外提供断线重连记忆、退避表老化等周期性维护接口。
 * =================================================================== */
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
#include "sle_common.h"
#include "sle_connection_manager.h"

/**
 * @brief 检查某个连接是否是本节点主动发起的 (Client 出站连接)
 *        如果匹配, 内部会清除 pending 状态.
 * @param addr  连接回调中收到的对端地址
 * @return true  = 这是我们主动发起的 Client 连接
 *         false = 这是对方连入我们 Server 的连接
 */
bool sle_uart_client_check_outgoing(const sle_addr_t *addr);

/**
 * @brief Client 端连接建立处理 (由 server 的统一回调调用)
 */
void sle_uart_client_on_connected(uint16_t conn_id, const sle_addr_t *addr);

/**
 * @brief Client 端连接断开处理 (由 server 的统一回调调用)
 */
void sle_uart_client_on_disconnected(uint16_t conn_id, const sle_addr_t *disc_addr);

/**
 * @brief Client 端配对完成处理
 */
void sle_uart_client_on_pair_complete(uint16_t conn_id, const sle_addr_t *addr, errcode_t status);

/**
 * @brief P8-B fix: Server 连接建立通知
 *        当对方通过 server 方向连入后, 通知 client 逻辑停止对该地址的 outgoing 重试
 */
void sle_uart_client_on_server_neighbor(const sle_addr_t *addr);

/**
 * @brief 断线重连 & 退避表老化清理 (由主循环周期调用)
 */
void sle_uart_client_reconnect_cleanup(void);

#endif /* CONFIG_SAMPLE_SUPPORT_SLE_MESH */

#endif