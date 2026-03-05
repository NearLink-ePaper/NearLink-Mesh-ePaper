/**
 * Copyright (c) 2026. All rights reserved.
 *
 * @file   mesh_types.h
 * @brief  SLE Mesh 核心类型定义与帧格式
 *
 * @details
 *   本文件定义了 Mesh 协议栈的基础数据类型，包括：
 *     - 帧头结构 (mesh_header_t)：16 字节 packed 格式，所有 mesh 帧共用
 *     - 消息类型枚举 (mesh_msg_type_t)：区分单播/广播/路由控制帧
 *     - 连接条目 (mesh_conn_entry_t)：连接池中每条连接的完整状态信息
 *     - 节点状态机 (mesh_node_state_t)：跟踪节点初始化进度
 *     - 回调函数类型：应用层数据接收和连接状态变化通知
 *
 *   帧格式示意图：
 *     +-------+-------+------+-----+--------+--------+--------+--------+--------+--------+
 *     | magic | ver   | type | ttl | src(2) | dst(2) | seq(2) | len(2) | hop(2) | crc(2) |
 *     | 0xAE  | 0x01  |      |     |        |        |        |        |        |        |
 *     +-------+-------+------+-----+--------+--------+--------+--------+--------+--------+
 *     |                         payload (0 ~ 504 bytes)                                   |
 *     +-----------------------------------------------------------------------------------+
 *
 * @note 所有多字节字段均使用小端字节序 (Little-Endian)。
 *
 * History:
 * 2026-02-07, Create file.
 */

#ifndef MESH_TYPES_H
#define MESH_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/* ============================================================
 *  帧头常量
 *
 *  MESH_FRAME_MAGIC:  帧起始标识字节，接收端首先校验此值以过滤非 mesh 帧。
 *  MESH_FRAME_VERSION: 协议版本号，用于未来兼容性检查。
 *  MESH_HEADER_SIZE:  帧头固定长度 (16 字节)，payload 紧随其后。
 *  MESH_MAX_PAYLOAD:  最大有效载荷 = SLE MTU(520) - SSAP 开销(~16) = 504 字节。
 *                     实际可用取决于 MTU 协商结果。
 *  MESH_DEFAULT_TTL:  默认生存时间 (Time-To-Live)，每跳减 1，降为 0 时丢弃。
 *                     7 跳可覆盖中等规模网络。若网络直径超过 7，需增大此值。
 * ============================================================ */
#define MESH_FRAME_MAGIC        0xAE        /* 帧起始标识字节 */
#define MESH_FRAME_VERSION      0x01        /* 协议版本号 */
#define MESH_HEADER_SIZE        16          /* 帧头固定长度 (字节) */
#define MESH_MAX_PAYLOAD        504         /* 最大有效载荷 = MTU(520) - 帧头(16) */
#define MESH_DEFAULT_TTL        7           /* 默认 TTL，建议范围: 4 ~ 15 */

/* ============================================================
 *  Mesh 地址常量
 *
 *  16-bit 地址空间：0x0001 ~ 0xFFFD 为可用节点地址。
 *  以下三个值为保留地址，不可分配给任何节点。
 * ============================================================ */
#define MESH_ADDR_BROADCAST     0xFFFF      /* 广播地址：dst_addr 为此值时全网洪泛 */
#define MESH_ADDR_UNASSIGNED    0x0000      /* 未分配/无效地址：用于标识空条目 */
#define MESH_ADDR_GATEWAY       0xFFFE      /* 网关保留地址 (用于 BLE Gateway 内部标识) */

/* ============================================================
 *  消息类型枚举
 *
 *  存储于帧头的 msg_type 字段 (1 字节)。
 *  值的分区设计：
 *    0x01~0x0F : 数据帧 (单播/广播)
 *    0x10~0x1F : AODV 路由控制帧 (RREQ/RREP/RERR)
 *    0x20~0x2F : 邻居发现帧 (HELLO)
 * ============================================================ */
typedef enum {
    MESH_MSG_UNICAST     = 0x01,    /* 单播数据帧：携带应用层有效载荷，逐跳转发 */
    MESH_MSG_BROADCAST   = 0x02,    /* 广播数据帧：全网洪泛，去重缓存防止环路 */
    MESH_MSG_RREQ        = 0x10,    /* 路由请求 (Route Request)：洪泛寻找目标路由 */
    MESH_MSG_RREP        = 0x11,    /* 路由应答 (Route Reply)：沿反向路径单播回源 */
    MESH_MSG_RERR        = 0x12,    /* 路由错误 (Route Error)：通知链路断裂影响的路由 */
    MESH_MSG_HELLO       = 0x20,    /* 邻居发现报文：周期广播，携带本节点邻居列表 */
} mesh_msg_type_t;

/* ============================================================
 *  Mesh 帧头结构 (16 字节, __attribute__((packed)))
 *
 *  字节偏移  字段          大小    说明
 *  ────────  ──────────    ────    ──────────────────────────────
 *  [0]       magic         1B     帧起始标识 0xAE
 *  [1]       version       1B     协议版本号 0x01
 *  [2]       msg_type      1B     消息类型 (mesh_msg_type_t)
 *  [3]       ttl           1B     生存时间，每跳减 1，为 0 时丢弃
 *  [4:5]     src_addr      2B     源节点 Mesh 短地址 (小端序)
 *  [6:7]     dst_addr      2B     目标地址 (0xFFFF=广播) (小端序)
 *  [8:9]     seq_num       2B     源节点自增序列号，用于去重 (小端序)
 *  [10:11]   payload_len   2B     有效载荷长度 (小端序)
 *  [12:13]   hop_count     2B     已跳转次数 (小端序)
 *  [14:15]   checksum      2B     CRC16-CCITT 校验值 (小端序)
 *
 *  CRC 校验范围：从 magic 到 payload 末尾 (不含 checksum 自身)。
 *  发送前由 mesh_forward_build_frame() 计算并填充。
 * ============================================================ */
typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  msg_type;
    uint8_t  ttl;
    uint16_t src_addr;
    uint16_t dst_addr;
    uint16_t seq_num;
    uint16_t payload_len;
    uint16_t hop_count;
    uint16_t checksum;
} mesh_header_t;

/* ============================================================
 *  完整 Mesh 帧 = 帧头 (16B) + 有效载荷 (0~504B)
 *
 *  注意：实际发送时帧长 = MESH_HEADER_SIZE + header.payload_len，
 *  并非总是 sizeof(mesh_frame_t)。接收端应根据 payload_len 截断。
 * ============================================================ */
typedef struct {
    mesh_header_t header;
    uint8_t       payload[MESH_MAX_PAYLOAD];
} mesh_frame_t;

/* ============================================================
 *  节点角色枚举
 *
 *  每条 SLE 连接中，本节点可能是 Server（被动接受连入）或
 *  Client（主动发起连出）。角色在连接建立时确定，生命周期内不变。
 *  用于连接池管理和 disconnect 事件的方向判断。
 * ============================================================ */
typedef enum {
    MESH_ROLE_SERVER = 0,   /* Server 角色：对端主动连入本节点 */
    MESH_ROLE_CLIENT = 1,   /* Client 角色：本节点主动连出到对端 */
} mesh_role_t;

/* ============================================================
 *  连接状态枚举
 *
 *  跟踪单条 SLE 连接的生命周期阶段。
 *  状态转换：IDLE → CONNECTING → CONNECTED → DISCONNECTED → IDLE
 * ============================================================ */
typedef enum {
    MESH_CONN_STATE_IDLE         = 0,   /* 空闲 */
    MESH_CONN_STATE_CONNECTING   = 1,   /* 连接中 */
    MESH_CONN_STATE_CONNECTED    = 2,   /* 已连接 */
    MESH_CONN_STATE_DISCONNECTED = 3,   /* 已断开 */
} mesh_conn_state_t;

/* ============================================================
 *  连接条目结构 — 连接池 (mesh_transport.c) 中的单条记录
 *
 *  每个活跃的 SLE 连接对应一个条目，存储连接标识、对端地址、
 *  角色、时间戳、故障计数等信息。各字段的详细用途如下：
 *
 *  核心标识字段：
 *    conn_id      : SLE 协议栈分配的连接句柄 (0~7)
 *    mesh_addr    : 对端 Mesh 16-bit 短地址
 *    sle_addr[6]  : 对端 SLE 6-byte MAC 地址 (用于地址校验)
 *    role         : 本端在该连接中的角色 (Server/Client)
 *    state        : 连接状态
 *
 *  活性检测字段 (P6/P7 僵尸检测)：
 *    last_active_ms   : 最后收到数据的时间戳 (仅 RX 方向更新)。
 *                       若 now - last_active_ms > MESH_ZOMBIE_TIMEOUT_MS 则疑似僵尸。
 *    send_fail_count  : API 级连续发送失败计数 (ssaps_notify_indicate / ssapc_write_req)。
 *                       ≥ MESH_SEND_FAIL_THRESHOLD 时标记为僵尸。
 *                       写确认成功时重置为 0。
 *
 *  P10 Server 通知就绪延迟字段：
 *    notify_ready_ms  : Server 连接建立后，在此时刻之前不向该连接发送 notify。
 *                       = 连接建立时间 + MESH_SERVER_NOTIFY_DELAY_MS。
 *    rx_from_peer     : 是否曾从该连接收到过数据。用于 P10 单向检测：
 *                       若发了 notify 但长时间 rx_from_peer=false，视为单向死亡。
 *
 *  P3 Fallback 标记字段：
 *    is_p3_fallback   : 此连接是否为 P3 回退机制建立的"远距离直连"。
 *                       当连接建立前路由表显示该目标需要多跳，但因邻居不足
 *                       而放宽限制直接连接时，设为 true。
 *    p3_original_hops : 建连前路由表中到该目标的跳数。用于评估直连是否改善了路径。
 * ============================================================ */
typedef struct {
    uint16_t         conn_id;           /* SLE 连接句柄 (由协议栈分配, 0~7) */
    uint16_t         mesh_addr;         /* 对端 Mesh 16-bit 短地址 */
    uint8_t          sle_addr[6];       /* 对端 SLE 6-byte MAC 地址 (用于 disconnect 地址校验) */
    mesh_role_t      role;              /* 本端角色: MESH_ROLE_SERVER / MESH_ROLE_CLIENT */
    mesh_conn_state_t state;            /* 连接状态 */
    int8_t           rssi;              /* 信号强度 (预留, 当前未使用) */
    uint32_t         last_active_ms;    /* 最后收到数据的时间戳 (仅 RX 更新, 用于僵尸检测) */
    uint8_t          send_fail_count;   /* 连续 API 级发送失败计数 (成功时重置, 用于僵尸检测) */
    uint32_t         notify_ready_ms;   /* Server 连接: 此时刻后才允许发 notify (P10 延迟) */
    bool             rx_from_peer;      /* 是否曾收到对端数据 (P10 单向死亡检测) */
    bool             is_p3_fallback;    /* 是否为 P3 远距离回退直连 */
    uint8_t          p3_original_hops;  /* P3 连接前路由表中到该目标的跳数 */
} mesh_conn_entry_t;

/* ============================================================
 *  节点状态机枚举
 *
 *  跟踪 Mesh 主任务 (mesh_main.c) 的初始化进度。
 *  启动顺序：INIT → SERVER_SETUP → CLIENT_SETUP → ACTIVE
 *  进入 ACTIVE 后开始正常的 HELLO 广播、扫描和数据收发。
 * ============================================================ */
typedef enum {
    MESH_NODE_STATE_INIT         = 0,   /* 初始化中 */
    MESH_NODE_STATE_SERVER_SETUP = 1,   /* Server角色启动中 */
    MESH_NODE_STATE_CLIENT_SETUP = 2,   /* Client角色启动中 */
    MESH_NODE_STATE_ACTIVE       = 3,   /* 双角色正常运行 */
} mesh_node_state_t;

/* ============================================================
 *  回调函数类型定义
 * ============================================================ */

/**
 * @brief 应用层数据接收回调
 * @param src_addr  发送端 Mesh 地址
 * @param data      有效载荷指针 (不含 mesh 帧头)
 * @param len       有效载荷长度 (字节)
 *
 * 当本节点收到目标为自己的单播帧或广播帧时，
 * mesh_forward 解析帧头后通过此回调将 payload 传递给应用层。
 */
typedef void (*mesh_rx_callback_t)(uint16_t src_addr, const uint8_t *data, uint16_t len);

/**
 * @brief 连接状态变化回调 (预留, 当前未使用)
 * @param mesh_addr  对端 Mesh 地址
 * @param state      新的连接状态
 * @param role       本端在该连接中的角色
 */
typedef void (*mesh_conn_change_callback_t)(uint16_t mesh_addr, mesh_conn_state_t state, mesh_role_t role);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* MESH_TYPES_H */