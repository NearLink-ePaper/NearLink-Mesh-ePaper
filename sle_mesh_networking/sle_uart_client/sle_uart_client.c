/**
 * @file   sle_uart_client.c
 * @brief  SLE UART 客户端模块 —— 扫描 / 连接 / 服务发现 / 数据收发
 * @details
 *   本文件实现 BearPi-Pico H3863 SLE Mesh 图传项目中的 **Client 角色** 逻辑。
 *   主要职责：
 *   1. 主动扫描 (sle_start_seek) 发现附近 SLE 广播设备
 *   2. 管理最多 MESH_MAX_CLIENT_CONN 个并发客户端连接
 *   3. 服务发现 (ssapc_find_structure) 获取属性句柄
 *   4. 通过 ssapc_write_req 向对端发送数据
 *   5. 接收对端 Notification/Indication 数据
 *   6. 与 mesh_transport 层交互：上报连接/断开事件、接收数据转发
 *
 *   关键设计：
 *   - conn_info 数组管理多连接状态（conn_id、pair_status）
 *   - 扫描暂停 / 恢复机制（g_scan_paused）避免传输与扫描冲突
 *   - 出站连接追踪（g_outgoing_pending + g_outgoing_addr）区分方向
 *   - 指数退避（backoff_table）防止对同一目标快速重试死循环
 *   - 断线重连记忆（reconnect_list）优先连接失联邻居
 *   - conn_id 冲突检测（地址校验）保护已有连接不被误删
 *
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * History:
 *   2023-04-03  创建文件
 *   2026-02-07  增加 Mesh 支持
 *   2026-02-08  统一连接回调 + 出站连接追踪机制
 */
/* === 系统头文件 === */
#include "common_def.h"
#include "soc_osal.h"
#include "securec.h"
#include "product.h"
#include "bts_le_gap.h"

/* === SLE 协议栈头文件 === */
#include "sle_device_discovery.h"
#include "sle_connection_manager.h"
#include "sle_uart_client.h"

/* === Mesh 专用头文件（仅 Mesh 模式编译） === */
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
#include "mesh_types.h"
#include "mesh_config.h"
#include "mesh_transport.h"
#include "mesh_route.h"
#endif

/* ===================================================================
 *  常量定义 —— MTU / 扫描参数 / 超时等
 * =================================================================== */
#define SLE_MTU_SIZE_DEFAULT            520   /**< 默认 MTU 大小（字节） */
#define SLE_SEEK_INTERVAL_DEFAULT       100   /**< 扫描间隔（单位: 0.625ms，即 62.5ms） */
#define SLE_SEEK_WINDOW_DEFAULT         100   /**< 扫描窗口（同上） */
#define UUID_16BIT_LEN                  2     /**< 16-bit UUID 字节长度 */
#define UUID_128BIT_LEN                 16    /**< 128-bit UUID 字节长度 */
#define SLE_UART_TASK_DELAY_MS          1000  /**< 任务延迟（ms） */
#define SLE_UART_WAIT_SLE_CORE_READY_MS 5000  /**< 等待 SLE Core 就绪超时（ms） */
#define SLE_UART_RECV_CNT               1000  /**< 接收计数阀值 */
#define SLE_UART_LOW_LATENCY_2K         2000  /**< 低延迟模式参数（2K） */

/* ===================================================================
 *  扫描目标名称
 *  Mesh 模式: 扫描 MESH_ADV_NAME_PREFIX 前缀的广播名
 *  单机模式: 扫描 "sle_uart_server"
 * =================================================================== */
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
#define SLE_UART_SERVER_SCAN_NAME       MESH_ADV_NAME_PREFIX  /**< Mesh 模式扫描前缀 */
#else
#ifndef SLE_UART_SERVER_NAME
#define SLE_UART_SERVER_NAME            "sle_uart_server"  /**< 单机模式默认服务器名 */
#endif
#define SLE_UART_SERVER_SCAN_NAME       SLE_UART_SERVER_NAME  /**< 单机模式扫描名 */
#endif

#define SLE_UART_CLIENT_LOG             "[sle uart client]"   /**< 日志前缀 */

/* 最大客户端并发连接数: Mesh 取配置值，单机模式默认 8 */
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
#define SLE_UART_CLIENT_MAX_CON         MESH_MAX_CLIENT_CONN  /**< Mesh 模式客户端上限 */
#else
#define SLE_UART_CLIENT_MAX_CON         8                     /**< 单机模式客户端上限 */
#endif

/* ===================================================================
 *  全局变量 —— 服务发现结果、回调结构体、连接管理
 * =================================================================== */
static ssapc_find_service_result_t g_sle_uart_find_service_result = { 0 };  /**< 服务发现结果缓存 */
static sle_announce_seek_callbacks_t g_sle_uart_seek_cbk = { 0 };          /**< 扫描回调集合 */
static ssapc_callbacks_t g_sle_uart_ssapc_cbk = { 0 };                     /**< SSAP Client 回调集合 */
static sle_addr_t g_sle_uart_remote_addr = { 0 };                          /**< 待连接的远端地址（扫描发现后填写） */
ssapc_write_param_t g_sle_uart_send_param = { 0 };                         /**< 写请求参数（handle + type） */
uint16_t g_sle_uart_conn_id[SLE_UART_CLIENT_MAX_CON] = { 0 };              /**< 已连接 conn_id 列表 */
uint16_t g_sle_uart_conn_num = 0;                                          /**< 当前客户端连接数 */

/* ===================================================================
 *  非 Mesh 模式: 保留原有连接回调结构体
 *  Mesh 模式下该回调由 Server 统一注册，此处不需要
 * =================================================================== */
#if !defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
static sle_connection_callbacks_t g_sle_uart_connect_cbk = { 0 };  /**< 单机模式连接回调 */
#endif

/* ============================================================
 *  Mesh 模式: 出站连接追踪
 *
 *  问题背景:
 *    sle_connection_register_callbacks 是全局覆盖 (SET) 型,
 *    Server 和 Client 不能各自注册, 所以统一在 Server 注册.
 *    但统一回调需要区分 "我连出去的" (Client) vs "别人连进来的" (Server).
 *
 *  方案:
 *    Client 发起连接前设置 g_outgoing_pending = true 并记录目标地址.
 *    统一回调中通过 sle_uart_client_check_outgoing() 匹配, 匹配成功
 *    即为 Client 出站连接.
 * ============================================================ */
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
static bool g_outgoing_pending = false;       /**< 是否有出站连接正在进行 */
static sle_addr_t g_outgoing_addr = { 0 };    /**< 当前出站连接的目标 SLE 地址 */
static uint32_t g_outgoing_start_ms = 0;      /**< 出站连接发起时间戳（超时保护用） */
static bool g_seek_stop_to_connect = false;   /**< true=扫描停止是为了发起连接, false=清理性停止 */
static bool g_scan_paused = false;            /**< P5: FC 传输期间持久暂停标记 */
static bool g_seek_running = false;           /**< seek 是否正在运行，只有 true 时才调用 sle_stop_seek */
#define MESH_OUTGOING_TIMEOUT_MS  8000        /**< 出站连接超时 8 秒，防止永久堵塞 */

/* P9: 记住最近超时的 outgoing 目标地址
 * 用途：SLE 协议栈在 outgoing 失败时会延迟触发已有 conn_id 的 disconnect，
 * 到达时 g_outgoing_pending 已被超时处理清除。如不保存最近地址，
 * 将无法识别这些迟到的“假 disconnect”，导致冲突退避永远不触发。 */
static sle_addr_t g_last_outgoing_addr = { 0 };    /**< 上次 outgoing 的目标地址 */
static uint32_t g_last_outgoing_done_ms = 0;       /**< 上次 outgoing 结束（超时/失败）时间 */
#define MESH_LATE_DISCONNECT_WINDOW_MS  3000       /**< 超时后 3s 内的 MISMATCH 仍视为冲突 */

/* ===================================================================
 *  P4: 连接失败指数退避机制
 *
 *  设计目的: 防止对同一目标快速重试形成死循环。
 *  策略: 失败后等待递增的冷却时间 (5s→2min)，
 *         连续失败超过 MESH_BACKOFF_GIVEUP 次则放弃重试。
 *  P8-B 扩展: conn_id 冲突使用更激进的退避倍率。
 * =================================================================== */
#define MESH_BACKOFF_MAX          8       /**< 退避表容量（最多追踪的目标数） */
#define MESH_BACKOFF_INIT_MS      5000    /**< 首次退避 5 秒 */
#define MESH_BACKOFF_MAX_MS       120000  /**< 最大退避 2 分钟 */
#define MESH_BACKOFF_GIVEUP       4       /**< 连续失败 4 次后放弃 (P9: 从 8 降至 4) */

/**
 * @brief 退避表条目 —— 记录单个目标地址的重试状态
 */
typedef struct {
    uint8_t  addr[SLE_ADDR_LEN];   /**< 目标 SLE 地址 */
    bool     active;               /**< 本条目是否有效 */
    uint8_t  fail_count;           /**< 连续失败次数 */
    uint8_t  conflict_count;       /**< P8-B: 连续 conn_id 冲突次数 */
    uint32_t next_retry_ms;        /**< 下次允许重试的时间戳 (tick) */
    bool     connecting;           /**< 当前是否正在尝试连接该目标 */
} mesh_backoff_entry_t;

static mesh_backoff_entry_t g_backoff_table[MESH_BACKOFF_MAX] = { 0 };  /**< 退避表全局实例 */

/**
 * @brief  在退避表中查找指定地址的条目
 * @param  addr  目标 SLE 地址（6 字节）
 * @return 找到则返回指针，否则返回 NULL
 */
static mesh_backoff_entry_t *backoff_find(const uint8_t *addr)
{
    for (int i = 0; i < MESH_BACKOFF_MAX; i++) {
        if (g_backoff_table[i].active &&
            memcmp(g_backoff_table[i].addr, addr, SLE_ADDR_LEN) == 0) {
            return &g_backoff_table[i];
        }
    }
    return NULL;
}

/**
 * @brief  分配一个新的退避表条目
 * @param  addr  目标 SLE 地址（6 字节）
 * @return 新分配（或淨汰后复用）的条目指针
 * @note   当表满时，淨汰策略为替换失败次数最多的条目
 */
static mesh_backoff_entry_t *backoff_alloc(const uint8_t *addr)
{
    /* 先找空位 */
    for (int i = 0; i < MESH_BACKOFF_MAX; i++) {
        if (!g_backoff_table[i].active) {
            memcpy_s(g_backoff_table[i].addr, SLE_ADDR_LEN, addr, SLE_ADDR_LEN);
            g_backoff_table[i].active = true;
            g_backoff_table[i].fail_count = 0;
            g_backoff_table[i].conflict_count = 0;
            g_backoff_table[i].next_retry_ms = 0;
            g_backoff_table[i].connecting = false;
            return &g_backoff_table[i];
        }
    }
    /* 满了, 淘汰失败次数最多的 */
    int worst = 0;
    for (int i = 1; i < MESH_BACKOFF_MAX; i++) {
        if (g_backoff_table[i].fail_count > g_backoff_table[worst].fail_count) {
            worst = i;
        }
    }
    memcpy_s(g_backoff_table[worst].addr, SLE_ADDR_LEN, addr, SLE_ADDR_LEN);
    g_backoff_table[worst].fail_count = 0;
    g_backoff_table[worst].conflict_count = 0;
    g_backoff_table[worst].next_retry_ms = 0;
    g_backoff_table[worst].connecting = false;
    return &g_backoff_table[worst];
}

/**
 * @brief  记录一次连接失败，计算下次重试时间
 * @param  addr  失败的目标 SLE 地址
 * @note   指数退避: 5s → 10s → 20s → 40s → 80s → 120s(cap)。
 *         达到 MESH_BACKOFF_GIVEUP 次后不再重试。
 */
static void backoff_record_fail(const uint8_t *addr)
{
    mesh_backoff_entry_t *e = backoff_find(addr);
    if (e == NULL) {
        e = backoff_alloc(addr);
    }
    e->fail_count++;
    e->connecting = false;
    /* 指数退避: 5s, 10s, 20s, 40s, 80s, 120s(cap) */
    uint32_t delay = MESH_BACKOFF_INIT_MS;
    for (uint8_t i = 1; i < e->fail_count && i < 6; i++) {
        delay *= 2;
    }
    if (delay > MESH_BACKOFF_MAX_MS) delay = MESH_BACKOFF_MAX_MS;
    e->next_retry_ms = osal_get_tick_ms() + delay;

    if (e->fail_count >= MESH_BACKOFF_GIVEUP) {
        osal_printk("[sle mesh] backoff: give up [%02x:%02x:%02x:%02x:%02x:%02x] after %d fails\r\n",
                    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], e->fail_count);
    } else {
        osal_printk("[sle mesh] backoff: [%02x:%02x:%02x:%02x:%02x:%02x] fail #%d, retry in %lums\r\n",
                    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],
                    e->fail_count, (unsigned long)delay);
    }
}

/**
 * @brief  记录 conn_id 冲突，使用更激进的退避倍率
 * @param  addr  冲突的目标 SLE 地址
 * @note   P8-B: 冲突思路 —— 新连接打断已有连接的 conn_id，
 *         使用 MESH_CONNID_CONFLICT_BACKOFF_MULT 倍率快速拉开重试间隔。
 *         达到 MESH_CONNID_CONFLICT_GIVEUP 次则彻底放弃。
 */
static void backoff_record_conflict(const uint8_t *addr)
{
    mesh_backoff_entry_t *e = backoff_find(addr);
    if (e == NULL) {
        e = backoff_alloc(addr);
    }
    e->fail_count++;
    e->conflict_count++;
    e->connecting = false;

    /* 冲突使用更大的退避倍率 */
    uint32_t delay = MESH_BACKOFF_INIT_MS;
    for (uint8_t i = 1; i < e->fail_count && i < 6; i++) {
        delay *= MESH_CONNID_CONFLICT_BACKOFF_MULT;
    }
    if (delay > MESH_BACKOFF_MAX_MS) delay = MESH_BACKOFF_MAX_MS;
    e->next_retry_ms = osal_get_tick_ms() + delay;

    if (e->conflict_count >= MESH_CONNID_CONFLICT_GIVEUP) {
        /* 连续冲突太多次, 标记为放弃 (不再主动 outgoing) */
        e->fail_count = MESH_BACKOFF_GIVEUP;
        osal_printk("%s P8-B: connid conflict give up [%02x:%02x] after %d conflicts\r\n",
                    SLE_UART_CLIENT_LOG, addr[4], addr[5], e->conflict_count);
    } else {
        osal_printk("%s P8-B: connid conflict [%02x:%02x] #%d, backoff %lums\r\n",
                    SLE_UART_CLIENT_LOG, addr[4], addr[5],
                    e->conflict_count, (unsigned long)delay);
    }
}

/**
 * @brief  连接成功后清除退避记录
 * @param  addr  成功连接的目标 SLE 地址
 */
static void backoff_clear(const uint8_t *addr)
{
    mesh_backoff_entry_t *e = backoff_find(addr);
    if (e != NULL) {
        if (e->fail_count > 0) {
            osal_printk("[sle mesh] backoff: cleared [%02x:%02x:%02x:%02x:%02x:%02x] (was %d fails)\r\n",
                        addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], e->fail_count);
        }
        e->active = false;
    }
}

/**
 * @brief  检查指定地址是否应该跳过（处于退避期 / 已放弃 / 正在连接中）
 * @param  addr  目标 SLE 地址
 * @return true = 应该跳过，false = 可以尝试连接
 */
static bool backoff_should_skip(const uint8_t *addr)
{
    mesh_backoff_entry_t *e = backoff_find(addr);
    if (e == NULL) return false;
    if (e->fail_count >= MESH_BACKOFF_GIVEUP) return true;  /* 已放弃 */
    if (e->connecting) return true;  /* 正在尝试中 */
    uint32_t now = osal_get_tick_ms();
    if (now < e->next_retry_ms) return true;  /* 还在冷却 */
    return false;
}

/**
 * @brief  标记正在尝试连接某目标（防止重复发起）
 * @param  addr  目标 SLE 地址
 */
static void backoff_mark_connecting(const uint8_t *addr)
{
    mesh_backoff_entry_t *e = backoff_find(addr);
    if (e == NULL) {
        e = backoff_alloc(addr);
    }
    e->connecting = true;
}

/* ===================================================================
 *  P4: 闪断检测 —— 连接建立后短时间内断开视为连接失败
 *  记录每个 conn_id 的连接时间和地址，断开时检查存活时间。
 *  若 < MESH_FLASH_DISCONNECT_MS 则触发退避，否则清除退避。
 * =================================================================== */
#define MESH_FLASH_DISCONNECT_MS  3000  /**< <3秒断开视为“闪断”（连接失败） */
static uint32_t g_client_conn_time[MESH_MAX_CLIENT_CONN + MESH_MAX_SERVER_CONN + 1];  /**< conn_id 对应的连接建立时间 */
static sle_addr_t g_client_conn_addr[MESH_MAX_CLIENT_CONN + MESH_MAX_SERVER_CONN + 1]; /**< conn_id 对应的对端地址 */

/* ===================================================================
 *  P3: 断线重连记忆 —— 记录失联邻居，扫描时优先重连
 *
 *  设计思路:
 *  - 邻居断开时记录其 mesh_addr
 *  - 扫描结果中若匹配则优先发起连接
 *  - MESH_RECONNECT_TTL_MS 超时后自动放弃
 * =================================================================== */
#define MESH_RECONNECT_MAX  4          /**< 重连记忆表容量 */
#define MESH_RECONNECT_TTL_MS  60000   /**< 60 秒后放弃重连 */

/**
 * @brief 重连记忆表条目
 */
typedef struct {
    uint16_t mesh_addr;  /**< 失联邻居的 Mesh 地址 */
    uint32_t lost_ms;    /**< 失联时间戳 */
    bool     active;     /**< 本条目是否有效 */
} mesh_reconnect_entry_t;

static mesh_reconnect_entry_t g_reconnect_list[MESH_RECONNECT_MAX] = { 0 };  /**< 重连记忆表全局实例 */

/**
 * @brief  添加一个失联邻居到重连记忆表
 * @param  mesh_addr  失联邻居的 Mesh 地址
 * @note   处理步骤:
 *         1. 跳过未分配地址
 *         2. 已存在则刷新时间
 *         3. 找空位插入
 *         4. 表满则覆盖最旧条目
 */
static void mesh_reconnect_add(uint16_t mesh_addr)
{
    if (mesh_addr == MESH_ADDR_UNASSIGNED) return;

    /* 已存在则刷新时间 */
    for (uint8_t i = 0; i < MESH_RECONNECT_MAX; i++) {
        if (g_reconnect_list[i].active && g_reconnect_list[i].mesh_addr == mesh_addr) {
            g_reconnect_list[i].lost_ms = osal_get_tick_ms();
            return;
        }
    }
    /* 找空位 */
    for (uint8_t i = 0; i < MESH_RECONNECT_MAX; i++) {
        if (!g_reconnect_list[i].active) {
            g_reconnect_list[i].mesh_addr = mesh_addr;
            g_reconnect_list[i].lost_ms = osal_get_tick_ms();
            g_reconnect_list[i].active = true;
            osal_printk("[sle mesh] reconnect: remember 0x%04X\r\n", mesh_addr);
            return;
        }
    }
    /* 满了, 覆盖最旧的 */
    uint8_t oldest = 0;
    uint32_t oldest_ms = 0xFFFFFFFF;
    for (uint8_t i = 0; i < MESH_RECONNECT_MAX; i++) {
        if (g_reconnect_list[i].lost_ms < oldest_ms) {
            oldest_ms = g_reconnect_list[i].lost_ms;
            oldest = i;
        }
    }
    g_reconnect_list[oldest].mesh_addr = mesh_addr;
    g_reconnect_list[oldest].lost_ms = osal_get_tick_ms();
    g_reconnect_list[oldest].active = true;
}

/**
 * @brief  从重连记忆表中移除指定 Mesh 地址
 * @param  mesh_addr  已重连成功的邻居 Mesh 地址
 */
static void mesh_reconnect_remove(uint16_t mesh_addr)
{
    for (uint8_t i = 0; i < MESH_RECONNECT_MAX; i++) {
        if (g_reconnect_list[i].active && g_reconnect_list[i].mesh_addr == mesh_addr) {
            g_reconnect_list[i].active = false;
        }
    }
}

/**
 * @brief  断线重连 & 退避表老化清理（由主循环周期调用）
 * @note   处理步骤:
 *         1. 清理重连表中超过 MESH_RECONNECT_TTL_MS 的条目
 *         2. 清理退避表中“已放弃”且冷却超过 5 分钟的条目
 *            （P9: 因 conn_id 冲突放弃的不重置，因为冲突是结构性问题）
 */
void sle_uart_client_reconnect_cleanup(void)
{
    uint32_t now = osal_get_tick_ms();
    for (uint8_t i = 0; i < MESH_RECONNECT_MAX; i++) {
        if (g_reconnect_list[i].active &&
            (now - g_reconnect_list[i].lost_ms > MESH_RECONNECT_TTL_MS)) {
            osal_printk("[sle mesh] reconnect: forget 0x%04X (timeout)\r\n",
                        g_reconnect_list[i].mesh_addr);
            g_reconnect_list[i].active = false;
        }
    }

    /* P4: 退避表老化 — "已放弃"的条目在 5 分钟后重置, 允许重新尝试
     * 冲突条目使用更长的冷却时间 (30 分钟) 后也重置, 因为长时间运行后
     * SLE conn_id 池状态会变化, 之前冲突的目标可能不再冲突.
     * 永久黑名单会导致长时间运行后网络分裂无法自愈. */
    for (uint8_t i = 0; i < MESH_BACKOFF_MAX; i++) {
        if (g_backoff_table[i].active &&
            g_backoff_table[i].fail_count >= MESH_BACKOFF_GIVEUP &&
            now > g_backoff_table[i].next_retry_ms) {
            uint32_t cooldown = (g_backoff_table[i].conflict_count > 0)
                                ? MESH_CONFLICT_COOLDOWN_MS  /* 冲突条目: 30 分钟冷却 */
                                : 300000;                    /* 普通失败: 5 分钟冷却 */
            if ((now - g_backoff_table[i].next_retry_ms) > cooldown) {
                osal_printk("[sle mesh] backoff: reset [%02x:%02x:%02x:%02x:%02x:%02x] "
                            "(age-out, conflicts=%d)\r\n",
                            g_backoff_table[i].addr[0], g_backoff_table[i].addr[1],
                            g_backoff_table[i].addr[2], g_backoff_table[i].addr[3],
                            g_backoff_table[i].addr[4], g_backoff_table[i].addr[5],
                            g_backoff_table[i].conflict_count);
                g_backoff_table[i].active = false;
            }
        }
    }
}
#endif

/**
 * @brief  获取全局写请求参数结构体指针
 * @return 指向 g_sle_uart_send_param，调用方填充 data 后用于 ssapc_write_req
 */
ssapc_write_param_t *get_g_sle_uart_send_param(void)
{
    return &g_sle_uart_send_param;
}

/* ===================================================================
 *  扫描控制 —— 暂停 / 恢复 / 启动
 *
 *  设计思路:
 *  - pause/resume 用于 FC 传输期间避免扫描干扰吞吐
 *  - start_scan 为统一入口，内部先 stop 再 start，确保干净状态
 *  - g_scan_paused 为持久标记，阻止主循环/回调中意外重启扫描
 * =================================================================== */

/**
 * @brief  暂停 SLE 扫描（P3: 传输期间）
 * @note   设置 g_scan_paused = true 后调用 sle_stop_seek()。
 *         后续任何 sle_uart_start_scan() 调用均被拦截。
 */
void sle_uart_pause_scan(void)
{
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
    g_scan_paused = true;
    g_seek_stop_to_connect = false;
    if (g_seek_running) {
        g_seek_running = false;
        sle_stop_seek();
    }
#else
    sle_stop_seek();
#endif
}

/**
 * @brief  恢复 SLE 扫描（P5: FC 传输结束后）
 * @note   清除 g_scan_paused 标记后调用 sle_uart_start_scan()。
 */
void sle_uart_resume_scan(void)
{
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
    g_scan_paused = false;
#endif
    sle_uart_start_scan();
}

/**
 * @brief  启动 SLE 设备扫描
 * @note   处理步骤:
 *         1. Mesh 模式下检查 g_scan_paused，若为 true 直接返回
 *         2. 先 sle_stop_seek() 清理残留扫描（标记为清理性停止）
 *         3. 等待 50ms 确保停止完成
 *         4. 配置扫描参数并启动
 *         P12: filter_duplicates 必须关闭，因为 SLE 控制器的 duplicate
 *              cache 在 stop/start 间不清除，会导致网络孤岛。
 */
void sle_uart_start_scan(void)
{
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
    /* P5: FC 传输期间, 阻止所有扫描启动 (包括主循环周期扫描
     * 和连接/断连回调中的扫描重启), 只有 sle_uart_resume_scan()
     * 能清除标记并真正恢复扫描 */
    if (g_scan_paused) {
        return;
    }
#endif
    /* 先停止可能残留的扫描, 确保干净状态
     * 标记为清理性停止, seek_disable_cbk 不会触发连接
     * 只有 seek 真正在运行时才调用 sle_stop_seek，
     * 避免对已停止的 seek 再次 stop 产生 error=20 回调及 OAL id=3 内存泄漏 */
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
    g_seek_stop_to_connect = false;
    if (g_seek_running) {
        g_seek_running = false;
        sle_stop_seek();
        osal_msleep(50);
    }
#else
    sle_stop_seek();
    osal_msleep(50);
#endif

    sle_seek_param_t param = { 0 };
    param.own_addr_type = 0;
    param.filter_duplicates = 0;    /* P12: 必须关闭! SLE控制器的 duplicate cache 在
                                     * stop/start 间不清除, 导致已见设备永远不再上报,
                                     * 造成网络孤岛. is_neighbor() 已做软件去重. */
    param.seek_filter_policy = 0;
    param.seek_phys = 1;
    param.seek_type[0] = 1;
    param.seek_interval[0] = SLE_SEEK_INTERVAL_DEFAULT;
    param.seek_window[0] = SLE_SEEK_WINDOW_DEFAULT;
    sle_set_seek_param(&param);
    sle_start_seek();
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
    g_seek_running = true;
#endif
}

/**
 * @brief  SLE 使能完成回调
 * @param  status  操作状态码
 * @note   非 Mesh 模式: 自行初始化并启动扫描。
 *         Mesh 模式: 由 Server 端统一初始化，此处不做任何动作。
 */
static void sle_uart_client_sample_sle_enable_cbk(errcode_t status)
{
    osal_printk("sle enable: %d.\r\n", status);
#if !defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
    /* 非Mesh模式: 自行初始化 */
    sle_uart_client_init(sle_uart_notification_cb, sle_uart_indication_cb);
    sle_uart_start_scan();
#endif
}

/**
 * @brief  扫描启动回调
 * @param  status  操作状态码，非 0 表示失败
 */
static void sle_uart_client_sample_seek_enable_cbk(errcode_t status)
{
    if (status != 0) {
        osal_printk("%s seek_enable_cbk, status error\r\n", SLE_UART_CLIENT_LOG);
    }
}

/* ===================================================================
 *  扫描结果回调 —— Mesh 模式增加重复连接检查
 *
 *  此回调是 Client 的核心决策函数，按顺序执行以下检查：
 *  1. 连接数上限 / 名称匹配 / 数据有效性
 *  2. [仅Mesh] Mesh 地址解析与校验
 *  3. [仅Mesh] 邻居去重（SLE 地址 + mesh_addr 双维度）
 *  4. [仅Mesh] 连接白名单过滤
 *  5. [仅Mesh] P3 地址大小约定（小做 Client，大做 Server）
 *  6. [仅Mesh] 出站超时保护 / 退避检查
 *  7. [仅Mesh] 路由可达性检查（P9）
 *  8. [仅Mesh] P8-B 客户端连接数上限预检
 *  9. 记录远端地址并停止扫描（待 seek_disable_cbk 发起连接）
 * =================================================================== */
static void sle_uart_client_sample_seek_result_info_cbk(sle_seek_result_info_t *seek_result_data)
{
    if (seek_result_data == NULL) {
        return;
    }

    if (g_sle_uart_conn_num >= SLE_UART_CLIENT_MAX_CON) {
        return;
    }

    /* 检查名称是否匹配 (静默跳过非 mesh 设备) */
    if (seek_result_data->data == NULL) {
        return;
    }
    if (strstr((const char *)seek_result_data->data, SLE_UART_SERVER_SCAN_NAME) == NULL) {
        return;
    }

#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
    /* ---- 从广播名称中解析 Mesh 地址 ---- */
    uint16_t remote_mesh_addr = 0;
    const char *name = (const char *)seek_result_data->data;
    const char *addr_start = strstr(name, MESH_ADV_NAME_PREFIX);
    if (addr_start != NULL) {
        addr_start += strlen(MESH_ADV_NAME_PREFIX);

        /* Fix: 校验名称尾部恰好是 4 位十六进制 (防止 UART 噪声追加字符) */
        char *endptr = NULL;
        long parsed = strtol(addr_start, &endptr, 16);
        int hex_len = (int)(endptr - addr_start);
        if (hex_len != 4 || parsed <= 0 || parsed > 0xFFFE) {
            /* 名称格式异常, 静默跳过 */
            return;
        }
        remote_mesh_addr = (uint16_t)parsed;

        if (remote_mesh_addr == g_mesh_node_addr) {
            return; /* 是自己, 跳过 */
        }
    }

    /* 已经是邻居, 静默跳过 (不再打印, 避免刷屏) */
    if (mesh_transport_is_neighbor(&seek_result_data->addr)) {
        return;
    }

    /* P5: 补充 mesh_addr 维度的邻居去重.
     * 问题: Server 连接存储的是对方的 client 连接地址, 与广播地址不同,
     * 导致上面的 SLE 地址比对 miss. 这里用 mesh_addr 兜底,
     * 彻底防止对已连接邻居发起重复 outgoing 连接. */
    if (remote_mesh_addr != 0 && mesh_transport_is_direct_neighbor(remote_mesh_addr)) {
        return;
    }

    /* P2 测试: 连接白名单过滤 (强制拓扑, 静默跳过) */
#if (MESH_CONNECT_FILTER_ADDR != 0x0000)
    if (remote_mesh_addr != MESH_CONNECT_FILTER_ADDR) {
        return;
    }
#endif

    /* P3: 连接去重 — 约定地址小的做 Client, 大的做 Server.
     * 如果对方地址比我小, 应由对方主动连我, 我不连它. 静默跳过.
     * 例外: 当 FILTER 明确指定了目标时, 无论地址大小都强制连接
     *       (用于构造菱形等非线性拓扑)
     *
     * P3-Fallback: 两种触发条件 (满足其一即放宽):
     *   1. 启动初期: 超过 MESH_P3_FALLBACK_TIMEOUT_MS 且唯一邻居 < 2
     *   2. 分区检测: 唯一邻居在过去 PARTITION_HEAL_TIMEOUT 内持续 < 3,
     *      说明网络可能已分裂, 需要主动向小地址节点发起连接来修复分区.
     * P22b: 使用唯一邻居计数 (去重), 避免双向连接虚高邻居数. */
    uint16_t my_addr = g_mesh_node_addr;
#if (MESH_CONNECT_FILTER_ADDR == 0x0000)
    if (remote_mesh_addr != 0 && remote_mesh_addr < my_addr) {
        /* 检查回退条件 */
        static uint32_t s_p3_first_check_ms = 0;
        static uint32_t s_last_nbr_sufficient_ms = 0;  /* 上次邻居 >= 3 的时间 */
        uint32_t now_ms = osal_get_tick_ms();
        if (s_p3_first_check_ms == 0) {
            s_p3_first_check_ms = now_ms;
            if (s_p3_first_check_ms == 0) s_p3_first_check_ms = 1;
            s_last_nbr_sufficient_ms = now_ms;
        }
        uint32_t elapsed = now_ms - s_p3_first_check_ms;
        /* P22b: 使用唯一邻居数而非连接数, 双向连接不重复计算 */
        uint8_t nbr_count = mesh_transport_get_unique_neighbor_count();

        /* 跟踪邻居充足的最近时间点 */
        if (nbr_count >= 3) {
            s_last_nbr_sufficient_ms = now_ms;
        }

        /* 条件1: 启动初期邻居不足 */
        bool startup_fallback = (elapsed >= MESH_P3_FALLBACK_TIMEOUT_MS && nbr_count < 2);
        /* 条件2: 分区检测 — 邻居持续不足则尝试自愈 */
        bool partition_fallback = (elapsed >= MESH_PARTITION_HEAL_TIMEOUT_MS &&
                                   nbr_count < 3 &&
                                   (now_ms - s_last_nbr_sufficient_ms) >= MESH_PARTITION_HEAL_TIMEOUT_MS);

        if (!startup_fallback && !partition_fallback) {
            return;  /* 正常模式: 跳过小地址 */
        }
        /* Fallback 激活, 但要先检查是否真的能发起连接,
         * 避免在 outgoing_pending/backoff 期间反复打印日志 */
        if (g_outgoing_pending || backoff_should_skip(seek_result_data->addr.addr)) {
            return;  /* 当前无法连接, 静默跳过 */
        }
        /* 真正要连了才打印一次 */
        osal_printk("%s P3 fallback: connect to 0x%04X (nbrs=%d, %s, wait=%lus)\r\n",
                    SLE_UART_CLIENT_LOG, remote_mesh_addr, nbr_count,
                    partition_fallback ? "partition-heal" : "startup",
                    (unsigned long)(elapsed / 1000));
    }
#endif

    /* 出站连接超时保护: 如果上次连接尝试超时未完成, 自动清除 */
    if (g_outgoing_pending) {
        uint32_t now_ms = osal_get_tick_ms();
        if (now_ms - g_outgoing_start_ms > MESH_OUTGOING_TIMEOUT_MS) {
            osal_printk("%s outgoing timeout, clearing pending\r\n",
                        SLE_UART_CLIENT_LOG);
            backoff_record_fail(g_outgoing_addr.addr);
            /* P9: 保存最近 outgoing 目标, 供迟到的 MISMATCH disconnect 识别 */
            (void)memcpy_s(&g_last_outgoing_addr, sizeof(sle_addr_t),
                           &g_outgoing_addr, sizeof(sle_addr_t));
            g_last_outgoing_done_ms = now_ms;
            g_outgoing_pending = false;
            return;
        }
        /* 已有一个出站连接在等待, 静默跳过 */
        return;
    }
    /* P4: 连接退避检查 — 如果最近连接该目标反复失败, 暂不重试 */
    /* P6: 如果已放弃的设备仍在广播, 说明它可能 reboot 了或 SLE 状态已变化, 重置退避.
     * 冲突条目也参与重置: 虽然冲突曾是 conn_id 串台导致, 但长时间后
     * SLE 协议栈内部 conn_id 分配状态已变化, 之前冲突的节点可能不再冲突.
     * 永久拒绝冲突节点会导致长时间运行后网络分裂无法自愈. */
    {
        mesh_backoff_entry_t *be = backoff_find(seek_result_data->addr.addr);
        if (be != NULL && be->fail_count >= MESH_BACKOFF_GIVEUP) {
            osal_printk("%s backoff reset: scanned alive [%02x:%02x], was given up"
                        " (conflicts=%d)\r\n",
                        SLE_UART_CLIENT_LOG,
                        seek_result_data->addr.addr[4],
                        seek_result_data->addr.addr[5],
                        be->conflict_count);
            be->fail_count = 0;
            be->conflict_count = 0;
            be->next_retry_ms = 0;
            be->connecting = false;
        }
    }
    if (backoff_should_skip(seek_result_data->addr.addr)) {
        return;
    }

    /* P9: 当邻居足够且目标已可通过路由中继到达时, 跳过直连.
     * 这避免了不必要的 outgoing 连接触发 SLE 协议栈的 conn_id 串台 bug,
     * 从而防止假 disconnect 事件破坏既有连接的稳定性.
     * 阈值从 2 提升到 3: 仅有 2 个邻居的节点仍应尝试建立更多直连,
     * 以提供网络冗余度, 防止单点故障导致分裂后无法自愈.
     * P22b: 使用去重后的唯一邻居数, 避免双向连接导致虚高计数 */
    {
        uint8_t nbr_count = mesh_transport_get_unique_neighbor_count();
        if (nbr_count >= 3 && remote_mesh_addr != 0) {
            uint16_t relay_hop = mesh_route_lookup(remote_mesh_addr);
            if (relay_hop != MESH_ADDR_UNASSIGNED) {
                /* 目标已可经路由中继到达, 且邻居充足, 无需建立直连 */
                return;
            }
        }
    }

    /* ============================================================
     *  P8-B fix: 提前检查 client 连接数上限
     *
     *  旧逻辑: 在 seek_disable_cbk (停止扫描后) 才检查,
     *  导致 "will connect → stop scan → P8-B skip → start scan"
     *  死循环, 每个扫描周期都重复, 数百条日志刷屏.
     *
     *  修复: 在此处 (扫描仍在运行) 就检查, 命中则静默跳过.
     *  扫描继续运行不受影响, 彻底消除 stop/start 循环.
     * ============================================================ */
    {
        uint8_t active_clients = mesh_transport_get_client_conn_count();
        if (active_clients >= MESH_OUTGOING_SAFE_CLIENT_LIMIT) {
            /* 限速日志: 30秒打印一次, 避免刷屏 */
            static uint32_t s_p8b_last_log_ms = 0;
            uint32_t now = osal_get_tick_ms();
            if (s_p8b_last_log_ms == 0 || (now - s_p8b_last_log_ms) > 30000) {
                osal_printk("%s P8-B: client limit reached (%d/%d), skip outgoing to 0x%04X\r\n",
                            SLE_UART_CLIENT_LOG, active_clients,
                            MESH_OUTGOING_SAFE_CLIENT_LIMIT, remote_mesh_addr);
                s_p8b_last_log_ms = now;
            }
            return;  /* 扫描仍在运行, 静默跳过 */
        }
    }
#endif

    osal_printk("%s will connect [%02x:%02x:%02x:%02x:%02x:%02x]\r\n",
                SLE_UART_CLIENT_LOG,
                seek_result_data->addr.addr[0], seek_result_data->addr.addr[1],
                seek_result_data->addr.addr[2], seek_result_data->addr.addr[3],
                seek_result_data->addr.addr[4], seek_result_data->addr.addr[5]);
    (void)memcpy_s(&g_sle_uart_remote_addr, sizeof(sle_addr_t),
                   &seek_result_data->addr, sizeof(sle_addr_t));
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
    g_seek_stop_to_connect = true;
#endif
    sle_stop_seek();
}

/**
 * @brief  扫描停止回调 —— 在扫描停止后决定是否发起连接
 * @param  status  sle_stop_seek 的返回状态
 * @note   Mesh 模式:
 *         - 只有 g_seek_stop_to_connect == true 时才发起连接
 *         - P8-B 安全网: 发起前再次检查客户端连接数上限
 *         - 记录出站连接地址并调用 sle_connect_remote_device
 *         非 Mesh 模式: 直接发起连接
 */
static void sle_uart_client_sample_seek_disable_cbk(errcode_t status)
{
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
    g_seek_running = false;  /* seek 已停止，无论原因都清除运行标记 */
#endif
    if (status != 0) {
        osal_printk("%s seek_disable_cbk, status error = %x\r\n", SLE_UART_CLIENT_LOG, status);
    }

#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
    /* 只有在"找到目标后停止"的情况下才发起连接
     * 清理性停止 (sle_uart_start_scan 中调用) 不触发连接 */
    if (!g_seek_stop_to_connect) {
        return;
    }
    g_seek_stop_to_connect = false;

    if (status != 0) {
        /* 停止失败, 不连接, 让主循环重试 */
        return;
    }

    /* ============================================================
     *  P8-B: conn_id 冲突防护 — 发起前预检 (保险层)
     *
     *  主检查已提前到 seek_result_info_cbk, 此处为双重保险.
     *  命中时不重启扫描 — 等主循环的周期性扫描触发, 避免死循环.
     * ============================================================ */
    {
        uint8_t active_clients = mesh_transport_get_client_conn_count();
        if (active_clients >= MESH_OUTGOING_SAFE_CLIENT_LIMIT) {
            osal_printk("%s P8-B: safety net in seek_disable, clients=%d >= limit=%d\r\n",
                        SLE_UART_CLIENT_LOG, active_clients,
                        MESH_OUTGOING_SAFE_CLIENT_LIMIT);
            /* 不调用 sle_uart_start_scan(), 等主循环 MESH_SCAN_INTERVAL_MS 后重试 */
            return;
        }
    }

    /* 记录出站连接目标地址, 用于统一回调中区分方向 */
    g_outgoing_pending = true;
    g_outgoing_start_ms = osal_get_tick_ms();
    (void)memcpy_s(&g_outgoing_addr, sizeof(sle_addr_t),
                   &g_sle_uart_remote_addr, sizeof(sle_addr_t));
    osal_printk("%s connecting to [%02x,%02x,%02x,%02x,%02x,%02x] (outgoing)\r\n",
                SLE_UART_CLIENT_LOG,
                g_outgoing_addr.addr[0], g_outgoing_addr.addr[1],
                g_outgoing_addr.addr[2], g_outgoing_addr.addr[3],
                g_outgoing_addr.addr[4], g_outgoing_addr.addr[5]);
    backoff_mark_connecting(g_outgoing_addr.addr);
    sle_connect_remote_device(&g_sle_uart_remote_addr);
#else
    if (status == 0) {
        sle_connect_remote_device(&g_sle_uart_remote_addr);
    }
#endif
}

/**
 * @brief  注册扫描相关回调（enable / seek_enable / seek_result / seek_disable）
 */
static void sle_uart_client_sample_seek_cbk_register(void)
{
    g_sle_uart_seek_cbk.sle_enable_cb = sle_uart_client_sample_sle_enable_cbk;
    g_sle_uart_seek_cbk.seek_enable_cb = sle_uart_client_sample_seek_enable_cbk;
    g_sle_uart_seek_cbk.seek_result_cb = sle_uart_client_sample_seek_result_info_cbk;
    g_sle_uart_seek_cbk.seek_disable_cb = sle_uart_client_sample_seek_disable_cbk;
    sle_announce_seek_register_callbacks(&g_sle_uart_seek_cbk);
}

/* ===================================================================
 *  非 Mesh 模式: 原有连接状态回调
 *  Mesh 模式下连接回调由 Server 统一注册，此处不编译
 * =================================================================== */
#if !defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)

/**
 * @brief  [非 Mesh] 连接状态变更回调
 * @param  conn_id      连接 ID
 * @param  addr         对端地址
 * @param  conn_state   当前连接状态（CONNECTED / DISCONNECTED / NONE）
 * @param  pair_state   配对状态（未使用）
 * @param  disc_reason  断开原因
 * @note   CONNECTED: 记录 conn_id、发起 MTU 交换、递增连接数
 *         DISCONNECTED: 从列表移除 conn_id、重新扫描
 */
static void sle_uart_client_sample_connect_state_changed_cbk(uint16_t conn_id, const sle_addr_t *addr,
                                                             sle_acb_state_t conn_state,
                                                             sle_pair_state_t pair_state,
                                                             sle_disc_reason_t disc_reason)
{
    unused(pair_state);
    osal_printk("%s conn state changed disc_reason:0x%x\r\n", SLE_UART_CLIENT_LOG, disc_reason);

    if (conn_state == SLE_ACB_STATE_CONNECTED) {
        osal_printk("%s SLE_ACB_STATE_CONNECTED conn_id=%d\r\n", SLE_UART_CLIENT_LOG, conn_id);

        if (g_sle_uart_conn_num < SLE_UART_CLIENT_MAX_CON) {
            g_sle_uart_conn_id[g_sle_uart_conn_num] = conn_id;

            ssap_exchange_info_t info = {0};
            info.mtu_size = SLE_MTU_SIZE_DEFAULT;
            info.version = 1;
            ssapc_exchange_info_req(1, conn_id, &info);

            g_sle_uart_conn_num++;
        }
    } else if (conn_state == SLE_ACB_STATE_NONE) {
        osal_printk("%s SLE_ACB_STATE_NONE\r\n", SLE_UART_CLIENT_LOG);
    } else if (conn_state == SLE_ACB_STATE_DISCONNECTED) {
        osal_printk("%s SLE_ACB_STATE_DISCONNECTED conn_id=%d\r\n", SLE_UART_CLIENT_LOG, conn_id);

        for (uint16_t i = 0; i < g_sle_uart_conn_num; i++) {
            if (g_sle_uart_conn_id[i] == conn_id) {
                g_sle_uart_conn_id[i] = g_sle_uart_conn_id[g_sle_uart_conn_num - 1];
                break;
            }
        }
        if (g_sle_uart_conn_num > 0) {
            g_sle_uart_conn_num--;
        }
        sle_uart_start_scan();
    } else {
        osal_printk("%s status error\r\n", SLE_UART_CLIENT_LOG);
    }
}

/**
 * @brief  [非 Mesh] 配对完成回调
 * @param  conn_id  连接 ID
 * @param  addr     对端地址
 * @param  status   配对结果，0 = 成功
 * @note   配对成功后发起 MTU 交换请求
 */
static void sle_uart_client_sample_pair_complete_cbk(uint16_t conn_id, const sle_addr_t *addr, errcode_t status)
{
    osal_printk("%s pair complete conn_id:%d, addr:%02x***%02x%02x\r\n", SLE_UART_CLIENT_LOG, conn_id,
                addr->addr[0], addr->addr[4], addr->addr[5]);
    if (status == 0) {
        ssap_exchange_info_t info = {0};
        info.mtu_size = SLE_MTU_SIZE_DEFAULT;
        info.version = 1;
        ssapc_exchange_info_req(0, conn_id, &info);
    }
}

/**
 * @brief  [非 Mesh] 注册连接状态回调（connect_state_changed + pair_complete）
 */
static void sle_uart_client_sample_connect_cbk_register(void)
{
    g_sle_uart_connect_cbk.connect_state_changed_cb = sle_uart_client_sample_connect_state_changed_cbk;
    g_sle_uart_connect_cbk.pair_complete_cb = sle_uart_client_sample_pair_complete_cbk;
    sle_connection_register_callbacks(&g_sle_uart_connect_cbk);
}
#endif /* !CONFIG_SAMPLE_SUPPORT_SLE_MESH */

/* ===================================================================
 *  Mesh 模式: 出站连接检查 & 事件处理函数
 *
 *  这些函数由 Server 的统一连接回调调用，
 *  不由 Client 自己注册。具体流程:
 *    1. Server 回调收到 CONNECTED 事件
 *    2. 调用 sle_uart_client_check_outgoing() 判断方向
 *    3. 匹配 → 调用 client_on_connected()
 *       不匹配 → 调用 server_on_connected()
 * =================================================================== */
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)

/**
 * @brief  检查某个连接是否是本节点主动发起的出站连接
 * @param  addr  连接回调中收到的对端 SLE 地址
 * @return true  = Client 出站连接（内部会清除 pending 标记）
 *         false = Server 入站连接（或无 pending）
 */
bool sle_uart_client_check_outgoing(const sle_addr_t *addr)
{
    if (!g_outgoing_pending || addr == NULL) {
        return false;
    }

    if (memcmp(addr->addr, g_outgoing_addr.addr, SLE_ADDR_LEN) == 0) {
        /* 匹配! 这是我们主动发起的出站连接 */
        g_outgoing_pending = false;
        return true;
    }

    return false;
}

/**
 * @brief  P8-B: Server 连接建立通知 —— 停止对该地址的 outgoing 重试
 * @param  addr  对方通过 Server 方向连入时的 SLE 地址
 * @note   当对方已通过 Server 方向连入，Client 无需再发起 outgoing，
 *         清除退避表中对应条目以避免无意义的重试。
 */
void sle_uart_client_on_server_neighbor(const sle_addr_t *addr)
{
    if (addr == NULL) return;
    mesh_backoff_entry_t *e = backoff_find(addr->addr);
    if (e != NULL) {
        osal_printk("%s P8-B: server conn established, clear backoff [%02x:%02x]\r\n",
                    SLE_UART_CLIENT_LOG, addr->addr[4], addr->addr[5]);
        e->active = false;
    }
}

/**
 * @brief  [Mesh] Client 端连接建立处理（由 Server 统一回调调用）
 * @param  conn_id  新建立的连接 ID
 * @param  addr     对端 SLE 地址
 * @note   处理步骤:
 *         1. 记录连接时间和地址（闪断检测用）
 *         2. 加入 conn_id 列表、发起 MTU 交换
 *         3. 通知 mesh_transport 新邻居上线
 *         4. 从重连列表中移除该地址
 *         5. 若连接数未满则继续扫描
 */
void sle_uart_client_on_connected(uint16_t conn_id, const sle_addr_t *addr)
{
    osal_printk("%s client_on_connected conn_id=%d\r\n", SLE_UART_CLIENT_LOG, conn_id);

    /* P4: 记录连接建立时间和地址, 用于闪断检测 */
    if (conn_id < (MESH_MAX_CLIENT_CONN + MESH_MAX_SERVER_CONN + 1)) {
        g_client_conn_time[conn_id] = osal_get_tick_ms();
        memcpy_s(&g_client_conn_addr[conn_id], sizeof(sle_addr_t), addr, sizeof(sle_addr_t));
    }

    if (g_sle_uart_conn_num < SLE_UART_CLIENT_MAX_CON) {
        g_sle_uart_conn_id[g_sle_uart_conn_num] = conn_id;

        /* 发起 MTU 交换 */
        ssap_exchange_info_t info = {0};
        info.mtu_size = SLE_MTU_SIZE_DEFAULT;
        info.version = 1;
        ssapc_exchange_info_req(1, conn_id, &info);

        /* 通知 mesh_transport */
        mesh_transport_on_client_connected(conn_id, addr);

        /* P3: 连上了, 从重连列表中移除 */
        uint16_t connected_addr = mesh_transport_addr_from_sle(addr);
        mesh_reconnect_remove(connected_addr);

        g_sle_uart_conn_num++;
    }

    /* 继续扫描更多邻居 */
    if (g_sle_uart_conn_num < SLE_UART_CLIENT_MAX_CON) {
        osal_msleep(500);
        sle_uart_start_scan();
    }
}

/**
 * @brief  [Mesh] Client 端连接断开处理（由 Server 统一回调调用）
 * @param  conn_id    断开的连接 ID
 * @param  disc_addr  断开事件中携带的对端地址
 * @note   处理步骤:
 *         1. P11: 检查 conn_id 在 transport 中是否为 SERVER 角色
 *            — 若是，说明是失败的 CLIENT 出站复用了 SERVER 的 conn_id，
 *              只清理出站状态，不动 transport 表
 *         2. P4: 地址校验 —— 比较 disc_addr 与 transport 中该 conn_id 的注册地址
 *            — 不匹配则为“假 disconnect”，不动 transport，不删邻居
 *         3. 退避决策: 区分普通失败 / conn_id 冲突 / 迟到 MISMATCH / 闪断
 *         4. 正常断开: 记录重连、通知 transport、移除 conn_id、重新扫描
 */
void sle_uart_client_on_disconnected(uint16_t conn_id, const sle_addr_t *disc_addr)
{
    osal_printk("%s client_on_disconnected conn_id=%d addr=%02x:%02x\r\n",
                SLE_UART_CLIENT_LOG, conn_id,
                disc_addr ? disc_addr->addr[4] : 0xFF,
                disc_addr ? disc_addr->addr[5] : 0xFF);

    /* ============================================================
     *  P4 关键修复: conn_id 地址校验
     *
     *  问题: SLE 协议栈对失败的出站连接尝试报的 disconnect 事件
     *  可能复用了已有连接的 conn_id. 如果不校验, 就会误删
     *  还活着的邻居, 导致 "能收不能发".
     *
     *  方案: 比较 disconnect 的 addr 与传输表中该 conn_id
     *  注册的邻居 addr, 不匹配则只清理出站状态, 不动传输表.
     * ============================================================ */
    bool addr_mismatch = false;
    mesh_conn_entry_t *entry = mesh_transport_find_by_conn_id(conn_id);

    /* P11: 如果 transport 中该 conn_id 是 SERVER 角色, 说明这个 disconnect
     * 是失败的 CLIENT 出站连接复用了 SERVER 的 conn_id.
     * 不应该当作 CLIENT 断开来处理 — 直接清理出站状态即可. */
    if (entry != NULL && entry->role == MESH_ROLE_SERVER) {
        osal_printk("%s P11: conn_id=%d is SERVER, ignoring CLIENT disconnect event\r\n",
                    SLE_UART_CLIENT_LOG, conn_id);
        if (g_outgoing_pending) {
            backoff_record_conflict(g_outgoing_addr.addr);
            (void)memcpy_s(&g_last_outgoing_addr, sizeof(sle_addr_t),
                           &g_outgoing_addr, sizeof(sle_addr_t));
            g_last_outgoing_done_ms = osal_get_tick_ms();
            g_outgoing_pending = false;
        }
        sle_uart_start_scan();
        return;
    }

    if (disc_addr != NULL && entry != NULL && entry->role == MESH_ROLE_CLIENT) {
        if (memcmp(entry->sle_addr, disc_addr->addr, SLE_ADDR_LEN) != 0) {
            addr_mismatch = true;
            osal_printk("%s *** conn_id=%d addr MISMATCH: registered=%02x:%02x, "
                        "disc=%02x:%02x → keep neighbor alive! ***\r\n",
                        SLE_UART_CLIENT_LOG, conn_id,
                        entry->sle_addr[4], entry->sle_addr[5],
                        disc_addr->addr[4], disc_addr->addr[5]);
        }
    }

    /* P4 + P8-B: 退避决策 — 处理出站失败, 区分普通失败和 conn_id 冲突 */
    if (g_outgoing_pending) {
        if (addr_mismatch) {
            /* P8-B: 出站连接触发了已有连接的虚假 disconnect — conn_id 冲突!
             * 使用更激进的退避, 避免反复冲击已有连接 */
            backoff_record_conflict(g_outgoing_addr.addr);
        } else {
            /* 普通出站连接失败 */
            backoff_record_fail(g_outgoing_addr.addr);
        }
        /* P9: 保存最近 outgoing 目标 */
        (void)memcpy_s(&g_last_outgoing_addr, sizeof(sle_addr_t),
                       &g_outgoing_addr, sizeof(sle_addr_t));
        g_last_outgoing_done_ms = osal_get_tick_ms();
        g_outgoing_pending = false;
    } else if (addr_mismatch && g_last_outgoing_done_ms > 0) {
        /* P9: 迟到的 MISMATCH disconnect — outgoing 已超时但 SLE 栈的假断开
         * 事件延迟到达. 仍需记录为冲突, 否则冲突退避永远不触发. */
        uint32_t since_outgoing = osal_get_tick_ms() - g_last_outgoing_done_ms;
        if (since_outgoing < MESH_LATE_DISCONNECT_WINDOW_MS) {
            osal_printk("%s P9: late MISMATCH %lums after outgoing, record conflict\r\n",
                        SLE_UART_CLIENT_LOG, (unsigned long)since_outgoing);
            backoff_record_conflict(g_last_outgoing_addr.addr);
        }
    } else if (!addr_mismatch &&
               conn_id < (MESH_MAX_CLIENT_CONN + MESH_MAX_SERVER_CONN + 1)) {
        /* 只在地址匹配时才做闪断/稳定断开判断 */
        uint32_t age_ms = osal_get_tick_ms() - g_client_conn_time[conn_id];
        if (age_ms < MESH_FLASH_DISCONNECT_MS) {
            backoff_record_fail(g_client_conn_addr[conn_id].addr);
        } else {
            backoff_clear(g_client_conn_addr[conn_id].addr);
        }
    }

    /* 地址不匹配 → 这是对另一个设备的失败连接, 不要动传输表! */
    if (addr_mismatch) {
        /* 重新扫描 (可能还能连到别的节点) */
        sle_uart_start_scan();
        return;
    }

    /* ---- 以下是正常断开处理 (地址匹配) ---- */

    /* P3: 记录断线邻居的地址 (必须在 transport 释放条目之前获取) */
    uint16_t lost_addr = mesh_transport_get_mesh_addr(conn_id);
    if (lost_addr != MESH_ADDR_UNASSIGNED) {
        mesh_reconnect_add(lost_addr);
    }

    /* 通知 mesh_transport */
    mesh_transport_on_client_disconnected(conn_id);

    /* 从连接列表中移除 */
    for (uint16_t i = 0; i < g_sle_uart_conn_num; i++) {
        if (g_sle_uart_conn_id[i] == conn_id) {
            g_sle_uart_conn_id[i] = g_sle_uart_conn_id[g_sle_uart_conn_num - 1];
            break;
        }
    }
    if (g_sle_uart_conn_num > 0) {
        g_sle_uart_conn_num--;
    }

    /* 重新扫描 */
    sle_uart_start_scan();
}

/**
 * @brief  [Mesh] Client 端配对完成处理
 * @param  conn_id  连接 ID
 * @param  addr     对端地址（未使用）
 * @param  status   配对结果，0 = 成功
 * @note   配对成功后发起 MTU 交换请求，触发后续服务发现流程
 */
void sle_uart_client_on_pair_complete(uint16_t conn_id, const sle_addr_t *addr, errcode_t status)
{
    osal_printk("%s client pair complete conn_id:%d status:%x\r\n", SLE_UART_CLIENT_LOG, conn_id, status);
    unused(addr);
    if (status == 0) {
        ssap_exchange_info_t info = {0};
        info.mtu_size = SLE_MTU_SIZE_DEFAULT;
        info.version = 1;
        ssapc_exchange_info_req(0, conn_id, &info);
    }
}

#endif /* CONFIG_SAMPLE_SUPPORT_SLE_MESH */

/* ===================================================================
 *  SSAPC 回调函数 —— MTU 交换、服务发现、属性查找、写确认
 *
 *  流程: exchange_info → find_structure → find_property → 就绪
 *  写数据时: ssapc_write_req → write_cfm_cb 确认
 * =================================================================== */

/**
 * @brief  MTU 交换完成回调
 * @param  client_id  SSAP 客户端实例 ID
 * @param  conn_id    连接 ID
 * @param  param      交换结果（包含協商后的 MTU 大小）
 * @param  status     操作状态码
 * @note   成功后自动发起服务发现 (ssapc_find_structure)。
 *         P11: 失败时会重试最多 MESH_EXCHANGE_INFO_RETRY_MAX 次，
 *         重试耗尽则标记该连接为僵尸。
 */
static void sle_uart_client_sample_exchange_info_cbk(uint8_t client_id, uint16_t conn_id,
    ssap_exchange_info_t *param, errcode_t status)
{
    osal_printk("%s exchange_info_cbk client_id:%d conn_id:%d status:%d\r\n",
                SLE_UART_CLIENT_LOG, client_id, conn_id, status);

    /* P11: exchange_info 失败时重试 (0xEE 可能是临时握手冲突, 非永久故障) */
    if (status != ERRCODE_SUCC) {
        /* 用 conn_entry 的 send_fail_count 暂作重试计数 (连接刚建立, 此值为0) */
        mesh_conn_entry_t *entry = mesh_transport_find_by_conn_id(conn_id);
        if (entry != NULL && entry->send_fail_count < MESH_EXCHANGE_INFO_RETRY_MAX) {
            entry->send_fail_count++;
            osal_printk("%s P11: exchange_info failed (0x%x), retry %d/%d for conn_id=%d\r\n",
                        SLE_UART_CLIENT_LOG, status, entry->send_fail_count,
                        MESH_EXCHANGE_INFO_RETRY_MAX, conn_id);
            /* 短延迟后重试 — 让对端 SERVER 完成资源分配 */
            osal_msleep(MESH_EXCHANGE_INFO_RETRY_MS);
            ssap_exchange_info_t info = {0};
            info.mtu_size = 520;
            info.version = 1;
            ssapc_exchange_info_req(0, conn_id, &info);
            return;
        }
        /* 重试耗尽 → 标记僵尸 */
        osal_printk("%s P11: exchange_info FAILED after %d retries, conn_id=%d dead\r\n",
                    SLE_UART_CLIENT_LOG, MESH_EXCHANGE_INFO_RETRY_MAX, conn_id);
        if (entry != NULL) {
            entry->send_fail_count = MESH_SEND_FAIL_THRESHOLD;  /* 立即标记为僵尸 */
        }
        return;
    }

    osal_printk("%s exchange mtu size: %d, version: %d.\r\n", SLE_UART_CLIENT_LOG,
                param->mtu_size, param->version);
    ssapc_find_structure_param_t find_param = { 0 };
    find_param.type = SSAP_FIND_TYPE_PROPERTY;
    find_param.start_hdl = 1;
    find_param.end_hdl = 0xFFFF;
    ssapc_find_structure(0, conn_id, &find_param);
}

/**
 * @brief  服务发现回调 —— 缓存服务的 start_hdl / end_hdl / uuid
 * @param  client_id  SSAP 客户端实例 ID
 * @param  conn_id    连接 ID
 * @param  service    发现的服务信息
 * @param  status     操作状态码
 */
static void sle_uart_client_sample_find_structure_cbk(uint8_t client_id, uint16_t conn_id,
    ssapc_find_service_result_t *service, errcode_t status)
{
    osal_printk("%s find structure cbk client:%d conn_id:%d status:%d\r\n",
                SLE_UART_CLIENT_LOG, client_id, conn_id, status);
    osal_printk("%s find structure start_hdl:[0x%02x], end_hdl:[0x%02x], uuid len:%d\r\n",
                SLE_UART_CLIENT_LOG, service->start_hdl, service->end_hdl, service->uuid.len);
    g_sle_uart_find_service_result.start_hdl = service->start_hdl;
    g_sle_uart_find_service_result.end_hdl = service->end_hdl;
    memcpy_s(&g_sle_uart_find_service_result.uuid, sizeof(sle_uuid_t), &service->uuid, sizeof(sle_uuid_t));
}

/**
 * @brief  属性发现回调 —— 记录属性 handle，用于后续写操作
 * @param  client_id  SSAP 客户端实例 ID
 * @param  conn_id    连接 ID
 * @param  property   发现的属性信息（handle）
 * @param  status     操作状态码
 */
static void sle_uart_client_sample_find_property_cbk(uint8_t client_id, uint16_t conn_id,
    ssapc_find_property_result_t *property, errcode_t status)
{
    osal_printk("%s find_property_cbk client_id:%d, conn_id:%d, handle:%d status:%d\r\n",
                SLE_UART_CLIENT_LOG, client_id, conn_id, property->handle, status);
    g_sle_uart_send_param.handle = property->handle;
    g_sle_uart_send_param.type = SSAP_PROPERTY_TYPE_VALUE;
}

/**
 * @brief  服务发现完成回调（所有服务/属性枚举结束）
 * @param  client_id        SSAP 客户端实例 ID
 * @param  conn_id          连接 ID
 * @param  structure_result 发现结果类型
 * @param  status           操作状态码
 */
static void sle_uart_client_sample_find_structure_cmp_cbk(uint8_t client_id, uint16_t conn_id,
    ssapc_find_structure_result_t *structure_result, errcode_t status)
{
    unused(conn_id);
    osal_printk("%s find_structure_cmp_cbk client_id:%d status:%d type:%d\r\n",
                SLE_UART_CLIENT_LOG, client_id, status, structure_result->type);
}

/**
 * @brief  写操作确认回调
 * @param  client_id    SSAP 客户端实例 ID
 * @param  conn_id      连接 ID
 * @param  write_result 写结果（handle）
 * @param  status       操作状态码
 * @note   P7: 写失败时累加 send_fail_count，供僵尸连接检测用。
 *         写成功则重置 send_fail_count。
 */
static void sle_uart_client_sample_write_cfm_cb(uint8_t client_id, uint16_t conn_id,
    ssapc_write_result_t *write_result, errcode_t status)
{
    if (!g_mesh_log_suppress) {
        osal_printk("%s write_cfm_cb conn_id:%d client_id:%d status:%d handle:%02x\r\n",
                    SLE_UART_CLIENT_LOG, conn_id, client_id, status, write_result->handle);
    }

    /* P7: 异步写确认失败 (err_code != 0) — 连接可能坏死
     * 累加 send_fail_count, 让 cleanup_stale() 快速检测到僵尸 */
    if (status != ERRCODE_SUCC) {
        mesh_conn_entry_t *entry = mesh_transport_find_by_conn_id(conn_id);
        if (entry != NULL && entry->send_fail_count < 255) {
            entry->send_fail_count++;
            osal_printk("%s write_cfm FAIL conn_id=%d, send_fails=%d\r\n",
                        SLE_UART_CLIENT_LOG, conn_id, entry->send_fail_count);
        }
    } else {
        /* 写确认成功 → 重置 */
        mesh_conn_entry_t *entry = mesh_transport_find_by_conn_id(conn_id);
        if (entry != NULL) {
            entry->send_fail_count = 0;
        }
    }
}

/**
 * @brief  注册 SSAPC 回调集合（exchange_info / find_structure / find_property / write_cfm / notification / indication）
 * @param  notification_cb  Notification 数据到达回调
 * @param  indication_cb    Indication 数据到达回调
 */
static void sle_uart_client_sample_ssapc_cbk_register(ssapc_notification_callback notification_cb,
                                                      ssapc_notification_callback indication_cb)
{
    g_sle_uart_ssapc_cbk.exchange_info_cb = sle_uart_client_sample_exchange_info_cbk;
    g_sle_uart_ssapc_cbk.find_structure_cb = sle_uart_client_sample_find_structure_cbk;
    g_sle_uart_ssapc_cbk.ssapc_find_property_cbk = sle_uart_client_sample_find_property_cbk;
    g_sle_uart_ssapc_cbk.find_structure_cmp_cb = sle_uart_client_sample_find_structure_cmp_cbk;
    g_sle_uart_ssapc_cbk.write_cfm_cb = sle_uart_client_sample_write_cfm_cb;
    g_sle_uart_ssapc_cbk.notification_cb = notification_cb;
    g_sle_uart_ssapc_cbk.indication_cb = indication_cb;
    ssapc_register_callbacks(&g_sle_uart_ssapc_cbk);
}

/* ===================================================================
 *  初始化入口
 *
 *  调用顺序:
 *  1. 注册扫描回调 (seek_cbk_register)
 *  2. 非 Mesh: 注册连接回调 (connect_cbk_register)
 *     Mesh: 跳过，由 Server 统一管理
 *  3. 注册 SSAPC 回调 (ssapc_cbk_register)
 *  4. 非 Mesh: 调用 enable_sle()
 *     Mesh: 跳过，由 Server 已完成 SLE 使能
 * =================================================================== */

/**
 * @brief  初始化 SLE UART 客户端
 * @param  notification_cb  Notification 数据回调
 * @param  indication_cb    Indication 数据回调
 * @note   详见上方流程说明
 */
void sle_uart_client_init(ssapc_notification_callback notification_cb, ssapc_indication_callback indication_cb)
{
    (void)osal_msleep(1000);
    osal_printk("[SLE Client] try enable.\r\n");
    sle_uart_client_sample_seek_cbk_register();

#if !defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
    /* 非Mesh模式: Client 自行注册连接回调 */
    sle_uart_client_sample_connect_cbk_register();
#else
    /* Mesh模式: 不注册连接回调!
     * sle_connection_register_callbacks 是全局覆盖型,
     * Server 端已注册统一回调, 这里不能再注册.
     * Server 的统一回调通过 sle_uart_client_check_outgoing() 区分方向. */
    osal_printk("[SLE Client] mesh mode, connection callbacks managed by server\r\n");
#endif

    sle_uart_client_sample_ssapc_cbk_register(notification_cb, indication_cb);

#if !defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
    if (enable_sle() != ERRCODE_SUCC) {
        osal_printk("[SLE Client] sle enable fail !\r\n");
    }
#else
    osal_printk("[SLE Client] mesh mode, skip enable_sle (already done by server)\r\n");
#endif
}