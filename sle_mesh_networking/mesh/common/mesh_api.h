/**
 * Copyright (c) 2026. All rights reserved.
 *
 * @file   mesh_api.h
 * @brief  SLE Mesh 应用层公共 API
 *
 * @details
 *   本文件定义了 Mesh 网络协议栈对外暴露的应用层接口，包括：
 *     - 初始化/反初始化
 *     - 单播/广播数据发送
 *     - 数据接收回调注册
 *     - 网络状态查询 (本地地址、邻居数、路由数、可达性)
 *
 *   调用顺序：
 *     1. mesh_init()         → 启动协议栈（由 mesh_main_task 内部调用）
 *     2. mesh_register_rx_callback() → 注册应用数据接收回调
 *     3. mesh_send() / mesh_broadcast() → 发送数据
 *     4. mesh_deinit()       → 关闭协议栈（通常无需显式调用）
 *
 *   实现说明：
 *     mesh_send()    → mesh_forward_send_unicast()  → AODV 路由 → 逐跳转发
 *     mesh_broadcast() → mesh_forward_send_broadcast() → 全网洪泛
 *
 * History:
 * 2026-02-07, Create file.
 */

#ifndef MESH_API_H
#define MESH_API_H

#include <stdint.h>
#include <stdbool.h>
#include "errcode.h"
#include "mesh_types.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/* ============================================================
 *  初始化 / 反初始化
 * ============================================================ */

/**
 * @brief  初始化 Mesh 网络协议栈
 *
 * 当前实现为空操作（返回 ERRCODE_SUCC），实际的初始化逻辑
 * 分散在 mesh_main_task() 中，按顺序调用 transport/forward/route
 * 各模块的 init 函数。保留此接口以供未来统一初始化入口。
 *
 * @return ERRCODE_SUCC 始终成功
 */
errcode_t mesh_init(void);

/**
 * @brief  反初始化 Mesh 网络协议栈
 *
 * 依次调用 mesh_transport_deinit()、mesh_forward_deinit()、
 * mesh_route_deinit() 释放所有模块资源。
 * 通常仅在设备关机或 Mesh 功能禁用时调用。
 */
void mesh_deinit(void);

/* ============================================================
 *  数据发送
 * ============================================================ */

/**
 * @brief  单播发送数据到指定目标节点
 *
 * 内部调用 mesh_forward_send_unicast()，自动处理路由查找：
 *   - 若目标是直连邻居，直接单跳发送
 *   - 若存在有效路由，沿下一跳转发
 *   - 若无路由，暂存到待发送队列并发起 AODV RREQ
 *
 * @param dst_addr  目标节点 Mesh 地址 (0x0001 ~ 0xFFFD)
 * @param data      有效载荷指针 (不含 mesh 帧头，由本函数自动封装)
 * @param len       有效载荷长度 (字节)，不得超过 MESH_MAX_PAYLOAD (504)
 * @return ERRCODE_SUCC 帧已发送或已入队等待路由
 */
errcode_t mesh_send(uint16_t dst_addr, const uint8_t *data, uint16_t len);

/**
 * @brief  广播发送数据到网络中所有节点
 *
 * 内部调用 mesh_forward_send_broadcast()，帧以洪泛方式扩散，
 * 每个中间节点根据去重缓存决定是否转发。
 *
 * @param data      有效载荷指针
 * @param len       有效载荷长度 (字节)，不得超过 MESH_MAX_PAYLOAD (504)
 * @return ERRCODE_SUCC 帧已发送
 */
errcode_t mesh_broadcast(const uint8_t *data, uint16_t len);

/* ============================================================
 *  数据接收
 * ============================================================ */

/**
 * @brief  注册应用层数据接收回调
 *
 * 当本节点收到目标为自己的单播帧，或任意广播帧时，
 * mesh_forward 在完成帧头解析和 CRC 校验后，调用此回调传递 payload。
 *
 * @param cb  回调函数指针。签名：void cb(uint16_t src_addr, const uint8_t *data, uint16_t len)
 *            - src_addr : 发送端 Mesh 地址
 *            - data     : 有效载荷指针 (帧头已剥离)
 *            - len      : 有效载荷长度
 *
 * @note  回调在 Mesh 主任务上下文中执行，应避免耗时操作。
 *        如需处理大量数据，建议将数据拷贝到独立缓冲区后通过信号量通知工作线程。
 */
void mesh_register_rx_callback(mesh_rx_callback_t cb);

/* ============================================================
 *  状态查询
 * ============================================================ */

/**
 * @brief  获取本节点 Mesh 短地址
 * @return 16-bit Mesh 地址 (即 g_mesh_node_addr)
 */
uint16_t mesh_get_local_addr(void);

/**
 * @brief  获取当前直连邻居数量
 * @return 活跃连接池中 state == CONNECTED 的条目数
 */
uint8_t mesh_get_neighbor_count(void);

/**
 * @brief  获取当前有效路由条目数量
 * @return AODV 路由表中未过期的条目数
 */
uint8_t mesh_get_route_count(void);

/**
 * @brief  检查指定目标节点是否可达
 *
 * 依次检查：
 *   1. 目标是否为直连邻居 (mesh_transport_is_direct_neighbor)
 *   2. AODV 路由表中是否存在有效路由 (mesh_route_is_reachable)
 *
 * @param dst_addr  目标节点 Mesh 地址
 * @return true 可达 (直连或有路由)，false 不可达
 */
bool mesh_is_reachable(uint16_t dst_addr);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* MESH_API_H */
