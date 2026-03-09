/**
 * @file  mesh_transport.c
 * @brief SLE Mesh 传输层实现 —— 连接池管理、双角色数据收发、僵尸清理、Turbo 模式
 *
 * @details 本文件实现 mesh_transport.h 中声明的所有传输层接口，包括:
 *          - 连接池的分配/释放/查询
 *          - Server 端与 Client 端的连接/断开事件处理
 *          - 基于角色的数据发送（Server=Notify, Client=WriteReq）
 *          - 邻居 Mesh 地址管理（P1 锁定、P3 fallback 检测）
 *          - 僵尸连接清理（P6/P7/P10 多维度检测）
 *          - Turbo 模式（O2 动态调整连接间隔）
 *
 * Copyright (c) 2026. All rights reserved.
 *
 * History: \n
 * 2026-02-07, 创建文件。 \n
 * 2026-02-08, P2: 断开回调中增加 mesh_route_on_link_break 路由通知。 \n
 */

#include "common_def.h"
#include "securec.h"
#include "soc_osal.h"
#include "osal_debug.h"
#include "sle_common.h"
#include "sle_connection_manager.h"
#include "sle_ssap_server.h"
#include "sle_ssap_client.h"
#include "sle_transmition_manager.h"  /* O13: QoS flow control callback */

#include "mesh_types.h"
#include "mesh_config.h"
#include "mesh_transport.h"
#include "mesh_route.h"      /* P2: 链路断开时调用 mesh_route_on_link_break() 通知路由层生成 RERR */

/* ============================================================
 *  静态变量 —— 全局连接池与接收回调
 * ============================================================ */
static mesh_conn_pool_t g_conn_pool = { 0 };               /**< 全局连接池实例，存储所有活跃连接条目 */
static mesh_transport_rx_callback_t g_rx_callback = NULL;   /**< 数据接收回调函数指针，由上层注册 */

/* O13: SLE QoS flow-control state — 由 SLE 协议栈回调更新 */
static volatile uint8_t g_sle_qos_busy = 0;  /**< >0 时表示有连接处于 FLOWCTRL/BUSY */

/*
 * 外部函数声明: 由 sle_uart_server.c 提供
 * - sle_uart_server_send_to_conn(): 通过指定 conn_id 以 Notify 方式发送数据
 * - sle_uart_server_get_server_id(): 获取 SLE Server 服务 ID
 * - sle_uart_server_get_property_handle(): 获取 SLE Server 属性句柄
 */
extern errcode_t sle_uart_server_send_to_conn(uint16_t conn_id, const uint8_t *data, uint16_t len);
extern uint8_t sle_uart_server_get_server_id(void);
extern uint16_t sle_uart_server_get_property_handle(void);

/*
 * 外部函数声明: 由 sle_uart_client.c 提供
 * - get_g_sle_uart_send_param(): 获取 Client 端发送参数结构体指针，
 *   用于填充 data/len 后调用 ssapc_write_req() 发送
 */
extern ssapc_write_param_t *get_g_sle_uart_send_param(void);

/* ============================================================
 *  O13: SLE QoS 流控回调 — 链路忙闲状态通知
 * ============================================================ */
static void sle_qos_callback(uint16_t conn_id, sle_link_qos_state_t link_state)
{
    if (link_state == SLE_QOS_IDLE) {
        if (g_sle_qos_busy > 0) {
            g_sle_qos_busy--;
        }
    } else {
        g_sle_qos_busy++;
        osal_printk("%s QoS: conn_id=%d state=%d (busy=%d)\r\n",
                    MESH_LOG_TAG, conn_id, link_state, g_sle_qos_busy);
    }
}

/* ============================================================
 *  初始化 / 反初始化
 * ============================================================ */

/** @brief 初始化传输层，清空连接池并重置接收回调 */
errcode_t mesh_transport_init(void)
{
    (void)memset_s(&g_conn_pool, sizeof(g_conn_pool), 0, sizeof(g_conn_pool));
    g_rx_callback = NULL;
    g_sle_qos_busy = 0;

    /* O13: 注册 SLE QoS 流控回调，获取链路忙闲状态 */
    sle_transmission_callbacks_t qos_cb = { 0 };
    qos_cb.send_data_cb = sle_qos_callback;
    errcode_t qret = sle_transmission_register_callbacks(&qos_cb);
    osal_printk("%s transport init ok (QoS cb: %s)\r\n",
                MESH_LOG_TAG, (qret == ERRCODE_SUCC) ? "OK" : "FAIL");
    return ERRCODE_SUCC;
}

/** @brief 反初始化传输层，释放所有连接池资源，调用后所有连接信息丢失 */
void mesh_transport_deinit(void)
{
    (void)memset_s(&g_conn_pool, sizeof(g_conn_pool), 0, sizeof(g_conn_pool));
    g_rx_callback = NULL;
}

/* ============================================================
 *  内部辅助: 分配空闲连接槽位
 * ============================================================ */

/**
 * @brief  从连接池中查找第一个 state == IDLE 的空闲槽位
 *
 * @return 指向空闲槽位的指针; 连接池已满时返回 NULL
 *
 * @note   调用方获取指针后需立即填充所有字段并设置 state=CONNECTED，
 *         否则该槽位可能被其他并发路径再次分配。
 */
static mesh_conn_entry_t *alloc_entry(void)
{
    for (uint8_t i = 0; i < MESH_MAX_CONNECTIONS; i++) {
        if (g_conn_pool.entries[i].state == MESH_CONN_STATE_IDLE) {
            return &g_conn_pool.entries[i];
        }
    }
    return NULL;
}

/* ============================================================
 *  内部辅助: 释放连接槽位
 * ============================================================ */

/**
 * @brief  释放指定连接槽位，清零所有字段并将状态置为 IDLE
 *
 * @param  entry  待释放的连接条目指针，为 NULL 时安全返回
 *
 * @note   释放后调用方需自行维护 count/server_count/client_count 计数器。
 */
static void free_entry(mesh_conn_entry_t *entry)
{
    if (entry != NULL) {
        (void)memset_s(entry, sizeof(mesh_conn_entry_t), 0, sizeof(mesh_conn_entry_t));
        entry->state = MESH_CONN_STATE_IDLE;
    }
}

/* ============================================================
 *  从 SLE 地址提取 Mesh 地址（取后两字节，小端序: addr[5]<<8 | addr[4]）
 *
 *  @note 仅适用于 SLE 广播地址（Client 端建连场景）。
 *        Server 端不应使用（P4: 客户端连接地址 ≠ 对方广播地址）。
 * ============================================================ */
uint16_t mesh_transport_addr_from_sle(const sle_addr_t *sle_addr)
{
    if (sle_addr == NULL) {
        return MESH_ADDR_UNASSIGNED;
    }
    return (uint16_t)((sle_addr->addr[5] << 8) | sle_addr->addr[4]);
}

/* ============================================================
 *  Server 端连接事件处理
 *
 *  @note Server 角色表示本节点作为被连接方，对端主动发起连接。
 *        发送数据时使用 Notify 方式（sle_uart_server_send_to_conn）。
 * ============================================================ */
void mesh_transport_on_server_connected(uint16_t conn_id, const sle_addr_t *addr)
{
    if (g_conn_pool.server_count >= MESH_MAX_SERVER_CONN) {
        osal_printk("%s server conn full, reject conn_id:%d\r\n", MESH_LOG_TAG, conn_id);
        return;
    }

    mesh_conn_entry_t *entry = alloc_entry();
    if (entry == NULL) {
        osal_printk("%s conn pool full\r\n", MESH_LOG_TAG);
        return;
    }

    entry->conn_id = conn_id;
    entry->role = MESH_ROLE_SERVER;
    entry->state = MESH_CONN_STATE_CONNECTED;
    /* P4 补丁: Server 端不从 SLE 地址推导 mesh_addr。
     * 原因: Client 连接过来时使用的是客户端连接地址，并非对方的 Server 广播地址，
     * 二者不同会导致错误地址（如 0x5C8D）被注册并通过 HELLO 扩散到全网。
     * 解决: 设为 UNASSIGNED，等待第一个 HELLO 消息到达后，由
     * mesh_transport_update_mesh_addr() 设置为真实的对端 Mesh 地址。
     * 若需修改该策略（如信任 SLE 地址），需确保 SLE 地址映射的正确性。 */
    uint16_t sle_derived = mesh_transport_addr_from_sle(addr);
    entry->mesh_addr = MESH_ADDR_UNASSIGNED;
    entry->last_active_ms = osal_get_tick_ms();
    /* P10 补丁: Server 端 Notify 就绪延迟。
     * 新建连接后立即发送 Notify 很可能失败（对端尚未完成服务发现），
     * 因此设置一个延迟窗口（MESH_SERVER_NOTIFY_DELAY_MS），
     * 在窗口内的发送请求将被静默丢弃，不计入失败计数。
     * 可通过调整 MESH_SERVER_NOTIFY_DELAY_MS（默认 300ms）优化时序。 */
    entry->notify_ready_ms = osal_get_tick_ms() + MESH_SERVER_NOTIFY_DELAY_MS;
    entry->rx_from_peer = false;
    if (addr != NULL) {
        (void)memcpy_s(entry->sle_addr, sizeof(entry->sle_addr), addr->addr, SLE_ADDR_LEN);
    }

    g_conn_pool.server_count++;
    g_conn_pool.count++;

    /* O12: 新连接建立时预先设置 DLE, 避免 turbo 时才协商带来额外延迟 */
    sle_set_data_len(conn_id, 512);

    osal_printk("%s server connected: conn_id=%d, sle_derived=0x%04X (pending HELLO), "
                "notify_ready in %dms, total=%d\r\n",
                MESH_LOG_TAG, conn_id, sle_derived, MESH_SERVER_NOTIFY_DELAY_MS,
                g_conn_pool.count);
}

/**
 * @brief  Server 端连接断开: 释放槽位、递减计数、通知路由层触发 RERR
 * @param  conn_id  断开的 SLE 连接 ID
 */
void mesh_transport_on_server_disconnected(uint16_t conn_id)
{
    mesh_conn_entry_t *entry = mesh_transport_find_by_conn_id(conn_id);
    if (entry == NULL || entry->role != MESH_ROLE_SERVER) {
        return;
    }

    uint16_t mesh_addr = entry->mesh_addr;
    free_entry(entry);

    if (g_conn_pool.server_count > 0) {
        g_conn_pool.server_count--;
    }
    if (g_conn_pool.count > 0) {
        g_conn_pool.count--;
    }

    osal_printk("%s server disconnected: conn_id=%d, mesh_addr=0x%04X, total=%d\r\n",
                MESH_LOG_TAG, conn_id, mesh_addr, g_conn_pool.count);

    /* P2 补丁: 通知路由层链路断开。
     * 若该连接已有有效 mesh_addr，则调用 mesh_route_on_link_break()
     * 触发 RERR 消息生成，通知网络中其他节点该链路已不可用。
     * 未分配地址的连接（未收到过 HELLO）断开时不触发 RERR。 */
    if (mesh_addr != MESH_ADDR_UNASSIGNED) {
        mesh_route_on_link_break(mesh_addr);
    }
}

/* ============================================================
 *  Client 端连接事件处理
 *
 *  @note Client 角色表示本节点主动发起连接，连接的是对端的广播地址。
 *        发送数据时使用 WriteReq 方式（ssapc_write_req）。
 * ============================================================ */
void mesh_transport_on_client_connected(uint16_t conn_id, const sle_addr_t *addr)
{
    if (g_conn_pool.client_count >= MESH_MAX_CLIENT_CONN) {
        osal_printk("%s client conn full, reject conn_id:%d\r\n", MESH_LOG_TAG, conn_id);
        return;
    }

    mesh_conn_entry_t *entry = alloc_entry();
    if (entry == NULL) {
        osal_printk("%s conn pool full\r\n", MESH_LOG_TAG);
        return;
    }

    entry->conn_id = conn_id;
    entry->role = MESH_ROLE_CLIENT;
    entry->state = MESH_CONN_STATE_CONNECTED;
    entry->mesh_addr = mesh_transport_addr_from_sle(addr);
    entry->last_active_ms = osal_get_tick_ms();
    /* P10 补丁: Client 端不需要 Notify 延迟窗口。
     * Client 使用 WriteReq 发送，连接建立后即可写入，无需等待对端服务发现。 */
    entry->notify_ready_ms = 0;
    entry->rx_from_peer = false;
    if (addr != NULL) {
        (void)memcpy_s(entry->sle_addr, sizeof(entry->sle_addr), addr->addr, SLE_ADDR_LEN);
    }

    g_conn_pool.client_count++;
    g_conn_pool.count++;

    /* O12: 新连接建立时预先设置 DLE */
    sle_set_data_len(conn_id, 512);

    osal_printk("%s client connected: conn_id=%d, mesh_addr=0x%04X, total=%d\r\n",
                MESH_LOG_TAG, conn_id, entry->mesh_addr, g_conn_pool.count);
}

/**
 * @brief  Client 端连接断开: 释放槽位、递减计数、通知路由层触发 RERR
 * @param  conn_id  断开的 SLE 连接 ID
 */
void mesh_transport_on_client_disconnected(uint16_t conn_id)
{
    mesh_conn_entry_t *entry = mesh_transport_find_by_conn_id(conn_id);
    if (entry == NULL || entry->role != MESH_ROLE_CLIENT) {
        return;
    }

    uint16_t mesh_addr = entry->mesh_addr;
    free_entry(entry);

    if (g_conn_pool.client_count > 0) {
        g_conn_pool.client_count--;
    }
    if (g_conn_pool.count > 0) {
        g_conn_pool.count--;
    }

    osal_printk("%s client disconnected: conn_id=%d, mesh_addr=0x%04X, total=%d\r\n",
                MESH_LOG_TAG, conn_id, mesh_addr, g_conn_pool.count);

    /* P2 补丁: 通知路由层链路断开。
     * 若该连接已有有效 mesh_addr，则触发 RERR，通知全网该链路已不可用。
     * 未分配地址的连接断开时不触发 RERR。 */
    if (mesh_addr != MESH_ADDR_UNASSIGNED) {
        mesh_route_on_link_break(mesh_addr);
    }
}

/* ============================================================
 *  连接池查询 —— 根据 conn_id / mesh_addr / SLE 地址查找条目
 *  所有查询均为 O(N) 线性遍历, N = MESH_MAX_CONNECTIONS (默认 5)。
 * ============================================================ */

/** @brief 根据 conn_id 查找已连接条目, IDLE 槽位不匹配; 未找到返回 NULL */
mesh_conn_entry_t *mesh_transport_find_by_conn_id(uint16_t conn_id)
{
    for (uint8_t i = 0; i < MESH_MAX_CONNECTIONS; i++) {
        if (g_conn_pool.entries[i].state == MESH_CONN_STATE_CONNECTED &&
            g_conn_pool.entries[i].conn_id == conn_id) {
            return &g_conn_pool.entries[i];
        }
    }
    return NULL;
}

/** @brief 根据 Mesh 16 位网络地址查找已连接条目; 未找到返回 NULL */
mesh_conn_entry_t *mesh_transport_find_by_mesh_addr(uint16_t mesh_addr)
{
    for (uint8_t i = 0; i < MESH_MAX_CONNECTIONS; i++) {
        if (g_conn_pool.entries[i].state == MESH_CONN_STATE_CONNECTED &&
            g_conn_pool.entries[i].mesh_addr == mesh_addr) {
            return &g_conn_pool.entries[i];
        }
    }
    return NULL;
}

/** @brief 按 SLE 6 字节地址检查是否已是直连邻居（建连前去重用） */
bool mesh_transport_is_neighbor(const sle_addr_t *addr)
{
    if (addr == NULL) {
        return false;
    }
    for (uint8_t i = 0; i < MESH_MAX_CONNECTIONS; i++) {
        if (g_conn_pool.entries[i].state == MESH_CONN_STATE_CONNECTED) {
            if (memcmp(g_conn_pool.entries[i].sle_addr, addr->addr, SLE_ADDR_LEN) == 0) {
                return true;
            }
        }
    }
    return false;
}

/** @brief 按 Mesh 地址检查是否直连邻居（路由层单播投递前判断） */
bool mesh_transport_is_direct_neighbor(uint16_t mesh_addr)
{
    return (mesh_transport_find_by_mesh_addr(mesh_addr) != NULL);
}

/** @brief 获取当前活跃连接总数 (Server + Client) */
uint8_t mesh_transport_get_neighbor_count(void)
{
    return g_conn_pool.count;
}

/** @brief 获取 Server 角色活跃连接数（本节点作为被连接方） */
uint8_t mesh_transport_get_server_conn_count(void)
{
    return g_conn_pool.server_count;
}

/** @brief 获取 Client 角色活跃连接数（本节点作为主动连接方） */
uint8_t mesh_transport_get_client_conn_count(void)
{
    return g_conn_pool.client_count;
}

/** @brief 获取连接池只读指针, 供外部遍历查看 (如拓扑信息收集) */
const mesh_conn_pool_t *mesh_transport_get_pool(void)
{
    return &g_conn_pool;
}

/* ============================================================
 *  数据发送: 通过 conn_id 发送
 *  内部根据连接角色自动选择发送方式:
 *  - Server 角色 → Notify (sle_uart_server_send_to_conn)
 *  - Client 角色 → WriteReq (ssapc_write_req)
 *  P10: Server 在 notify_ready_ms 窗口内静默丢弃,
 *  P7: 维护 send_fail_count 用于僵尸检测。
 * ============================================================ */
errcode_t mesh_transport_send_by_conn_id(uint16_t conn_id, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) {
        return ERRCODE_INVALID_PARAM;
    }

    mesh_conn_entry_t *entry = mesh_transport_find_by_conn_id(conn_id);
    if (entry == NULL) {
        osal_printk("%s send fail: conn_id %d not found\r\n", MESH_LOG_TAG, conn_id);
        return ERRCODE_FAIL;
    }

    errcode_t ret;

    if (entry->role == MESH_ROLE_SERVER) {
        /* P10: 在 notify_ready 窗口内静默跳过, 避免污染通道 */
        uint32_t now = osal_get_tick_ms();
        if (now < entry->notify_ready_ms) {
            return ERRCODE_SUCC;  /* 静默丢弃, 不算失败 */
        }
        ret = sle_uart_server_send_to_conn(conn_id, data, len);
        if (ret != ERRCODE_SUCC) {
            osal_printk("%s P10 notify FAIL: conn_id=%d, ret=0x%x\r\n",
                        MESH_LOG_TAG, conn_id, ret);
        }
    } else {
        ssapc_write_param_t *param = get_g_sle_uart_send_param();
        if (param == NULL) {
            return ERRCODE_FAIL;
        }
        param->data_len = len;
        param->data = (uint8_t *)data;
        ret = ssapc_write_req(0, conn_id, param);
    }

    if (ret != ERRCODE_SUCC) {
        osal_printk("%s send fail: conn_id=%d, role=%d, ret=0x%x\r\n",
                    MESH_LOG_TAG, conn_id, entry->role, ret);
        /* P7: 累计 API 级发送失败 */
        if (entry->send_fail_count < 255) {
            entry->send_fail_count++;
        }
    } else {
        /* P7: 发送成功 → 重置失败计数 (但不更新 last_active_ms,
         * 僵尸检测仅依赖 RX 活跃度) */
        entry->send_fail_count = 0;
    }

    return ret;
}

/* ============================================================
 *  数据发送: 按 Mesh 地址发送到直连邻居
 *  内部先 find_by_mesh_addr() 查条目, 再调用 send_by_conn_id()。
 *  若目标不在一跳范围内, 应由路由层查询下一跳后使用 send_by_conn_id()。
 * ============================================================ */
errcode_t mesh_transport_send_to_neighbor(uint16_t mesh_addr, const uint8_t *data, uint16_t len)
{
    mesh_conn_entry_t *entry = mesh_transport_find_by_mesh_addr(mesh_addr);
    if (entry == NULL) {
        return ERRCODE_FAIL;
    }
    return mesh_transport_send_by_conn_id(entry->conn_id, data, len);
}

/* ============================================================
 *  数据发送: 遍历连接池向所有邻居广播, 可排除指定 conn_id (防止回环)
 *  典型用途: HELLO 广播、RERR 泛洪、拓扑消息扩散。
 *  部分邻居发送失败不影响其余邻居, 返回最后一个错误码。
 * ============================================================ */
errcode_t mesh_transport_send_to_all_neighbors(const uint8_t *data, uint16_t len, uint16_t exclude_conn_id)
{
    errcode_t last_err = ERRCODE_SUCC;
    uint8_t sent_count = 0;

    for (uint8_t i = 0; i < MESH_MAX_CONNECTIONS; i++) {
        mesh_conn_entry_t *entry = &g_conn_pool.entries[i];
        if (entry->state != MESH_CONN_STATE_CONNECTED) {
            continue;
        }
        if (entry->conn_id == exclude_conn_id) {
            continue;
        }

        errcode_t ret = mesh_transport_send_by_conn_id(entry->conn_id, data, len);
        if (ret != ERRCODE_SUCC) {
            last_err = ret;
        } else {
            sent_count++;
        }
    }

    if (!g_mesh_log_suppress) {
        osal_printk("%s broadcast to %d neighbors (exclude conn_id=%d)\r\n",
                    MESH_LOG_TAG, sent_count, exclude_conn_id);
    }

    return last_err;
}

/* ============================================================
 *  数据接收回调注册与入口处理
 *  register: 上层在 mesh_protocol_init() 中注册解包函数, 只支持一个。
 *  on_data_received: 由 SLE 底层收数回调触发, 更新活跃时间 / 失败计数,
 *  标记双向通信（P10）, 最后转发给已注册的协议层回调。
 * ============================================================ */

/** @brief 注册数据接收回调, 同一时刻只支持一个, 后注册覆盖先前 */
void mesh_transport_register_rx_callback(mesh_transport_rx_callback_t cb)
{
    g_rx_callback = cb;
}

/**
 * @brief  数据接收入口, 更新 P7/P10 状态后转发给上层回调
 * @param  conn_id  数据来源连接 ID
 * @param  data     原始数据指针
 * @param  len      数据长度（字节）
 */
void mesh_transport_on_data_received(uint16_t conn_id, const uint8_t *data, uint16_t len)
{
    mesh_conn_entry_t *entry = mesh_transport_find_by_conn_id(conn_id);
    if (entry != NULL) {
        entry->last_active_ms = osal_get_tick_ms();
        entry->send_fail_count = 0;  /* P7: 收到数据说明连接活跃 */
        /* P10: 确认双向通信 */
        if (!entry->rx_from_peer) {
            entry->rx_from_peer = true;
            osal_printk("%s P10: conn_id=%d rx confirmed, bidirectional OK\r\n",
                        MESH_LOG_TAG, conn_id);
        }
    }

    if (g_rx_callback != NULL) {
        g_rx_callback(conn_id, data, len);
    }
}

/* ============================================================
 *  P1: 更新邻居的 Mesh 地址 —— 从 HELLO 消息中获取真实地址
 *  P1-fix: 锁定机制 — 若已有非 UNASSIGNED 地址则拒绝覆写。
 *  P3: 检测远距离弱信号直连 — 若路由表中为多跳，标记 fallback。
 * ============================================================ */
void mesh_transport_update_mesh_addr(uint16_t conn_id, uint16_t mesh_addr)
{
    mesh_conn_entry_t *entry = mesh_transport_find_by_conn_id(conn_id);
    if (entry == NULL || entry->mesh_addr == mesh_addr) {
        return;
    }

    if (entry->mesh_addr != MESH_ADDR_UNASSIGNED) {
        /* P1-fix: mesh_addr 已锁定, 拒绝覆写.
         * 同一 conn_id 可能被 SLE 协议栈同时用于 server/client 通道,
         * 导致两个不同节点的 HELLO 都以 conn_id=N 到达.
         * 第一个 HELLO 确定身份后不再变更. */
        osal_printk("%s WARN: conn_id=%d addr locked 0x%04X, reject 0x%04X\r\n",
                    MESH_LOG_TAG, conn_id, entry->mesh_addr, mesh_addr);
        return;
    }

    osal_printk("%s neighbor addr update: conn_id=%d, 0x%04X -> 0x%04X\r\n",
                MESH_LOG_TAG, conn_id, entry->mesh_addr, mesh_addr);
    entry->mesh_addr = mesh_addr;

    /* P22a: 双向连接消除 —— 检测同一 mesh_addr 是否已有另一方向的连接。
     * 若存在, 说明两节点互连 (A 是 B 的 client, B 也是 A 的 client),
     * 浪费了双方的 server 槽位。按 P3 约定: 地址小的做 Client, 大的做 Server。
     * 由大地址节点负责断开其 client 连接 (即不符合约定的方向)。 */
    {
        bool has_server = false, has_client = false;
        uint16_t server_conn_id = 0xFFFF, client_conn_id = 0xFFFF;
        for (uint8_t i = 0; i < MESH_MAX_CONNECTIONS; i++) {
            mesh_conn_entry_t *e = &g_conn_pool.entries[i];
            if (e->state != MESH_CONN_STATE_CONNECTED || e->mesh_addr != mesh_addr) continue;
            if (e->role == MESH_ROLE_SERVER) { has_server = true; server_conn_id = e->conn_id; }
            if (e->role == MESH_ROLE_CLIENT) { has_client = true; client_conn_id = e->conn_id; }
        }
        if (has_server && has_client) {
            /* 双向连接! 决定断开哪一条: */
            uint16_t to_drop;
            if (g_mesh_node_addr < mesh_addr) {
                /* 我地址小 → 我应该是 client → 断开 server 连接 (对方不该连我) */
                to_drop = server_conn_id;
                osal_printk("%s P22a: bidir with 0x%04X, I'm smaller, drop SERVER conn_id=%d\r\n",
                            MESH_LOG_TAG, mesh_addr, to_drop);
            } else {
                /* 我地址大 → 我应该是 server → 断开 client 连接 (我不该连对方) */
                to_drop = client_conn_id;
                osal_printk("%s P22a: bidir with 0x%04X, I'm larger, drop CLIENT conn_id=%d\r\n",
                            MESH_LOG_TAG, mesh_addr, to_drop);
            }
            mesh_transport_force_disconnect(to_drop);
        }
    }

    /* P3 fallback detection: 如果此节点在路由表中是多跳, 但现在成了直连邻居,
     * 说明是远距离弱信号 P3 连接, 标记此邻居 */
    uint8_t table_hops = mesh_route_get_table_hop_count(mesh_addr);
    if (table_hops >= 2 && table_hops != 0xFF) {
        entry->is_p3_fallback   = true;
        entry->p3_original_hops = table_hops;
        osal_printk("%s P3 fallback neighbor: conn_id=%d addr=0x%04X table_hops=%d\r\n",
                    MESH_LOG_TAG, conn_id, mesh_addr, table_hops);
    }
}

/** @brief 获取指定 conn_id 对应的 Mesh 地址; 未找到返回 MESH_ADDR_UNASSIGNED */
uint16_t mesh_transport_get_mesh_addr(uint16_t conn_id)
{
    mesh_conn_entry_t *entry = mesh_transport_find_by_conn_id(conn_id);
    if (entry != NULL) {
        return entry->mesh_addr;
    }
    return MESH_ADDR_UNASSIGNED;
}

/**
 * @brief  获取所有已连接邻居的 Mesh 地址列表（内部去重）
 * @param  addrs      输出数组, 调用方分配
 * @param  max_count  输出数组最大容量
 * @return 实际填入的邻居数量
 */
uint8_t mesh_transport_get_all_neighbor_addrs(uint16_t *addrs, uint8_t max_count)
{
    uint8_t count = 0;
    if (addrs == NULL) {
        return 0;
    }
    for (uint8_t i = 0; i < MESH_MAX_CONNECTIONS && count < max_count; i++) {
        if (g_conn_pool.entries[i].state == MESH_CONN_STATE_CONNECTED &&
            g_conn_pool.entries[i].mesh_addr != MESH_ADDR_UNASSIGNED) {
            /* P6: 去重 — 同一 mesh_addr 可能在 server 和 client 池中各有一个条目 */
            bool dup = false;
            for (uint8_t j = 0; j < count; j++) {
                if (addrs[j] == g_conn_pool.entries[i].mesh_addr) {
                    dup = true;
                    break;
                }
            }
            if (!dup) {
                addrs[count++] = g_conn_pool.entries[i].mesh_addr;
            }
        }
    }
    return count;
}

/* ============================================================
 *  P3 fallback 查询 —— 检测远距离弱信号直连邻居
 *  P3 邻居: 建连前在路由表中为多跳（≥2），说明物理距离远、信号弱。
 *  路由层可据此优先使用多跳中继路径，仅在中继不可用时才走 P3 直连。
 * ============================================================ */

/** @brief 检查指定 Mesh 地址是否为 P3 fallback 邻居 */
bool mesh_transport_is_p3_neighbor(uint16_t mesh_addr)
{
    mesh_conn_entry_t *entry = mesh_transport_find_by_mesh_addr(mesh_addr);
    if (entry != NULL && entry->state == MESH_CONN_STATE_CONNECTED) {
        return entry->is_p3_fallback;
    }
    return false;
}

/** @brief 获取 P3 邻居建连前的路由表跳数; 非 P3 邻居返回 0 */
uint8_t mesh_transport_get_p3_original_hops(uint16_t mesh_addr)
{
    mesh_conn_entry_t *entry = mesh_transport_find_by_mesh_addr(mesh_addr);
    if (entry != NULL && entry->state == MESH_CONN_STATE_CONNECTED && entry->is_p3_fallback) {
        return entry->p3_original_hops;
    }
    return 0;
}

/* ============================================================
 *  P6: 僵尸连接定期清理
 *
 *  问题: 当远端 reboot 或 SLE 协议栈复用 conn_id 时,
 *  disconnect 回调的地址可能与注册地址不匹配, P4 逻辑选择
 *  保守不清理 (因为也可能是活连接 + 失败出站的情况).
 *  但如果确实是死连接, entry 会变成 "僵尸":
 *  - 占用连接池槽位
 *  - 拓扑响应中包含不可达邻居
 *  - 广播/单播发送失败
 *
 *  方案: 通过 last_active_ms 检测. mesh_transport_on_data_received()
 *  在每次收到数据时更新 last_active_ms (P7: 发送不再更新).
 *  如果超过 ZOMBIE_TIMEOUT_MS 没有收到任何数据, 说明该连接已死.
 *  另外, 连续 SEND_FAIL_THRESHOLD 次 API 级发送失败也视为僵尸 (快速检测).
 * ============================================================ */
uint8_t mesh_transport_cleanup_stale(void)
{
    uint32_t now = osal_get_tick_ms();
    uint8_t cleaned = 0;

    /* 先收集需要清理的连接信息, 避免在遍历中修改池 */
    struct {
        uint16_t conn_id;
        uint8_t  sle_addr[6];
    } zombies[MESH_MAX_CONNECTIONS];
    uint8_t  zombie_count = 0;

    for (uint8_t i = 0; i < MESH_MAX_CONNECTIONS; i++) {
        mesh_conn_entry_t *entry = &g_conn_pool.entries[i];
        if (entry->state != MESH_CONN_STATE_CONNECTED) {
            continue;
        }

        uint32_t age_ms = now - entry->last_active_ms;
        bool is_rx_timeout = (age_ms >= MESH_ZOMBIE_TIMEOUT_MS);
        bool is_send_dead  = (entry->send_fail_count >= MESH_SEND_FAIL_THRESHOLD);

        /* P10: SERVER 已过 notify 延迟, 但从未收到对端数据 → 单向死亡 */
        bool is_oneway_dead = false;
        if (entry->role == MESH_ROLE_SERVER &&
            now >= entry->notify_ready_ms &&
            !entry->rx_from_peer) {
            uint32_t conn_age = now - (entry->notify_ready_ms - MESH_SERVER_NOTIFY_DELAY_MS);
            if (conn_age >= MESH_SERVER_ONEWAY_TIMEOUT_MS) {
                is_oneway_dead = true;
            }
        }

        if (!is_rx_timeout && !is_send_dead && !is_oneway_dead) {
            continue;
        }

        /* 超时 或 发送持续失败 — 僵尸连接 */
        uint16_t mesh_addr = entry->mesh_addr;
        uint16_t conn_id = entry->conn_id;
        mesh_role_t role = entry->role;

        osal_printk("%s P7 zombie detected: conn_id=%d, mesh_addr=0x%04X, role=%s, "
                    "inactive %lums, send_fails=%d%s\r\n",
                    MESH_LOG_TAG, conn_id, mesh_addr,
                    (role == MESH_ROLE_SERVER) ? "SRV" : "CLI",
                    (unsigned long)age_ms, entry->send_fail_count,
                    is_oneway_dead ? " [P10:oneway]" : "");

        /* 记录需要 SLE 断开的连接信息 */
        if (zombie_count < MESH_MAX_CONNECTIONS) {
            zombies[zombie_count].conn_id = conn_id;
            (void)memcpy_s(zombies[zombie_count].sle_addr, 6, entry->sle_addr, 6);
            zombie_count++;
        }

        free_entry(entry);

        if (role == MESH_ROLE_SERVER && g_conn_pool.server_count > 0) {
            g_conn_pool.server_count--;
        } else if (role == MESH_ROLE_CLIENT && g_conn_pool.client_count > 0) {
            g_conn_pool.client_count--;
        }
        if (g_conn_pool.count > 0) {
            g_conn_pool.count--;
        }

        /* 通知路由层 */
        if (mesh_addr != MESH_ADDR_UNASSIGNED) {
            mesh_route_on_link_break(mesh_addr);
        }

        /* P6: 通知广播层需要重启广播 (腾出了 server 槽位) */
        if (role == MESH_ROLE_SERVER) {
            extern volatile bool g_need_re_announce;
            g_need_re_announce = true;
        }

        cleaned++;
    }

    /* P7: 在池操作完成后, 实际断开 SLE 连接 (避免回调重入) */
    for (uint8_t i = 0; i < zombie_count; i++) {
        sle_addr_t disc_addr = {0};
        disc_addr.type = 0;
        (void)memcpy_s(disc_addr.addr, SLE_ADDR_LEN, zombies[i].sle_addr, 6);
        errcode_t disc_ret = sle_disconnect_remote_device(&disc_addr);
        osal_printk("%s P7 disconnect zombie conn_id=%d, ret=0x%x\r\n",
                    MESH_LOG_TAG, zombies[i].conn_id, disc_ret);
        (void)disc_ret;  /* 断开可能失败, 不影响清理 */
    }

    return cleaned;
}

/* ============================================================
 *  O2: Turbo 模式 — 动态调整所有邻居的 SLE 连接间隔
 *  O12: PHY 2M + DLE — 在 turbo 期间提升物理层吞吐
 *
 *  FC 活跃时调用 enable=true:
 *    - 连接间隔降到 7.5ms
 *    - PHY 切换 1M→2M (吞吐翻倍)
 *    - DLE 设置 512B (减少 PDU 分片)
 *  FC 结束时调用 enable=false:
 *    - 连接间隔恢复 50ms
 *    - PHY 恢复 1M (省功耗)
 * ============================================================ */
void mesh_transport_set_turbo_mode(bool enable)
{
    uint16_t intv = enable ? MESH_SLE_TURBO_INTV : MESH_SLE_CONN_INTV_MIN;
    /* O12: turbo 模式使用 2M PHY, 正常模式恢复 1M (省功耗)
     * 注: 4M PHY 在实测中导致大量 write_req err_code:2, 已回退至 2M */
    uint8_t target_phy = enable ? SLE_PHY_2M : SLE_PHY_1M;

    uint8_t updated = 0;
    for (uint8_t i = 0; i < MESH_MAX_CONNECTIONS; i++) {
        mesh_conn_entry_t *entry = &g_conn_pool.entries[i];
        if (entry->state != MESH_CONN_STATE_CONNECTED) {
            continue;
        }

        sle_connection_param_update_t param = { 0 };
        param.conn_id             = entry->conn_id;
        param.interval_min        = intv;
        param.interval_max        = intv;
        param.max_latency         = 0;
        param.supervision_timeout = 500;  /* 5s */

        errcode_t ret = sle_update_connect_param(&param);
        if (ret == ERRCODE_SUCC) {
            updated++;
        } else {
            osal_printk("%s turbo: conn_id=%d update FAIL ret=0x%x\r\n",
                        MESH_LOG_TAG, entry->conn_id, ret);
        }

        /* O12: 设置 PHY (2M turbo / 1M normal) */
        sle_set_phy_t phy = { 0 };
        phy.tx_format = SLE_RADIO_FRAME_1;
        phy.rx_format = SLE_RADIO_FRAME_1;
        phy.tx_phy    = target_phy;
        phy.rx_phy    = target_phy;
        phy.tx_pilot_density = 0;
        phy.rx_pilot_density = 0;
        sle_set_phy_param(entry->conn_id, &phy);

        /* O12: turbo 时增大 DLE 以减少 PDU 分片数 (512B 足够覆盖 480B 数据包) */
        if (enable) {
            sle_set_data_len(entry->conn_id, 512);
        }
    }
    osal_printk("%s O2 turbo %s: intv=0x%04X phy=%s, updated %d conns\r\n",
                MESH_LOG_TAG, enable ? "ON" : "OFF", intv,
                enable ? "2M" : "1M", updated);
}

/* ============================================================
 *  O13: SLE QoS 查询 — 供上层检查是否有链路拥塞
 * ============================================================ */
bool mesh_transport_is_sle_busy(void)
{
    return g_sle_qos_busy > 0;
}

/* ============================================================
 *  P22: 网络分裂自愈 — 双向连接消除 + 冗余环路检测 + 强制断开
 * ============================================================ */

/** @brief P22b: 获取连接池中去重后的唯一邻居数量 */
uint8_t mesh_transport_get_unique_neighbor_count(void)
{
    uint16_t seen[MESH_MAX_CONNECTIONS];
    uint8_t count = 0;
    for (uint8_t i = 0; i < MESH_MAX_CONNECTIONS; i++) {
        mesh_conn_entry_t *e = &g_conn_pool.entries[i];
        if (e->state != MESH_CONN_STATE_CONNECTED ||
            e->mesh_addr == MESH_ADDR_UNASSIGNED) {
            continue;
        }
        bool dup = false;
        for (uint8_t j = 0; j < count; j++) {
            if (seen[j] == e->mesh_addr) { dup = true; break; }
        }
        if (!dup && count < MESH_MAX_CONNECTIONS) {
            seen[count++] = e->mesh_addr;
        }
    }
    return count;
}

/** @brief P22a: 检测是否与指定 mesh_addr 存在双向连接 */
bool mesh_transport_has_bidirectional(uint16_t mesh_addr)
{
    if (mesh_addr == MESH_ADDR_UNASSIGNED) return false;
    bool has_server = false, has_client = false;
    for (uint8_t i = 0; i < MESH_MAX_CONNECTIONS; i++) {
        mesh_conn_entry_t *e = &g_conn_pool.entries[i];
        if (e->state != MESH_CONN_STATE_CONNECTED || e->mesh_addr != mesh_addr) continue;
        if (e->role == MESH_ROLE_SERVER) has_server = true;
        if (e->role == MESH_ROLE_CLIENT) has_client = true;
    }
    return has_server && has_client;
}

/** @brief P22c/d: 断开一条指定 conn_id 的连接并清理连接池 */
void mesh_transport_force_disconnect(uint16_t conn_id)
{
    mesh_conn_entry_t *entry = mesh_transport_find_by_conn_id(conn_id);
    if (entry == NULL) return;

    uint16_t mesh_addr = entry->mesh_addr;
    mesh_role_t role = entry->role;
    uint8_t sle_addr_copy[6];
    (void)memcpy_s(sle_addr_copy, 6, entry->sle_addr, 6);

    free_entry(entry);

    if (role == MESH_ROLE_SERVER && g_conn_pool.server_count > 0) {
        g_conn_pool.server_count--;
    } else if (role == MESH_ROLE_CLIENT && g_conn_pool.client_count > 0) {
        g_conn_pool.client_count--;
    }
    if (g_conn_pool.count > 0) {
        g_conn_pool.count--;
    }

    osal_printk("%s P22 force disconnect: conn_id=%d, mesh_addr=0x%04X, role=%s, total=%d\r\n",
                MESH_LOG_TAG, conn_id, mesh_addr,
                (role == MESH_ROLE_SERVER) ? "SRV" : "CLI",
                g_conn_pool.count);

    if (mesh_addr != MESH_ADDR_UNASSIGNED) {
        mesh_route_on_link_break(mesh_addr);
    }

    if (role == MESH_ROLE_SERVER) {
        extern volatile bool g_need_re_announce;
        g_need_re_announce = true;
    }

    /* 实际断开 SLE 链路 */
    sle_addr_t disc_addr = {0};
    disc_addr.type = 0;
    (void)memcpy_s(disc_addr.addr, SLE_ADDR_LEN, sle_addr_copy, 6);
    sle_disconnect_remote_device(&disc_addr);
}

/** @brief P22d: 查找一条存在冗余路径的 server 连接 (环路检测) */
uint16_t mesh_transport_find_redundant_server(void)
{
    for (uint8_t i = 0; i < MESH_MAX_CONNECTIONS; i++) {
        mesh_conn_entry_t *e = &g_conn_pool.entries[i];
        if (e->state != MESH_CONN_STATE_CONNECTED) continue;
        if (e->role != MESH_ROLE_SERVER) continue;
        if (e->mesh_addr == MESH_ADDR_UNASSIGNED) continue;

        /* 检查路由表中是否有通过其他邻居到达该节点的条目 */
        uint16_t route_next = mesh_route_lookup(e->mesh_addr);
        if (route_next != MESH_ADDR_UNASSIGNED && route_next != e->mesh_addr) {
            /* 路由表中存在替代路径 (经由另一个节点中继), 此 server 连接冗余 */
            osal_printk("%s P22d: server conn_id=%d to 0x%04X is redundant "
                        "(alt route via 0x%04X)\r\n",
                        MESH_LOG_TAG, e->conn_id, e->mesh_addr, route_next);
            return e->conn_id;
        }
    }
    return 0xFFFF;
}