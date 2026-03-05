/**
 * @file mesh_route.h
 * @brief SLE Mesh 路由层头文件 — 简化 AODV（按需距离矢量）路由协议
 *
 * Copyright (c) 2026. All rights reserved.
 *
 * @details
 * 本模块实现了面向 SLE（星闪近距离无线通信）Mesh 网络的简化 AODV 路由协议，
 * 提供多跳 ad-hoc 组网能力。主要包含以下核心功能：
 *
 * 1. RREQ（路由请求）：当源节点没有到达目标节点的有效路由时，广播 RREQ
 *    报文进行路由发现。沿途中间节点建立到发起者（originator）的反向路由。
 *
 * 2. RREP（路由应答）：目标节点或拥有新鲜路由的中间节点收到 RREQ 后，
 *    沿反向路由单播回传 RREP，沿途建立到目标节点的正向路由。
 *
 * 3. RERR（路由错误）：检测到链路断开时，生成 RERR 报文广播通知受影响
 *    节点，使其将对应路由条目标记为无效。
 *
 * 4. 路由表管理：支持路由查找、添加、更新、删除、老化清理等基础操作，
 *    与 P1 阶段被动路由学习（HELLO 两跳、反向路由）兼容。
 *
 * @note 本协议为简化版 AODV，与标准 RFC 3561 存在以下差异：
 *       - 序列号为 32 位无符号整数，使用有符号差值法处理回绕
 *       - 不支持 Gratuitous RREP、本地修复（Local Repair）等高级特性
 *       - RERR 最多携带 8 个不可达目标
 *       - 路由表大小固定为 MESH_ROUTE_TABLE_SIZE（默认 32 条）
 *       - 可根据实际网络规模在 mesh_config.h 中调整各项参数
 *
 * History:
 * 2026-02-07, 创建文件（桩实现）。
 * 2026-02-08, P2: 完整 AODV 路由协议实现。
 */

#ifndef MESH_ROUTE_H
#define MESH_ROUTE_H

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
 *  AODV 控制报文 Payload 结构
 *
 *  以下结构体采用 __attribute__((packed)) 紧凑排列，直接填入
 *  mesh 帧的 payload 字段，跨节点传输时无需额外序列化。
 *  各字段均采用小端字节序（与 ARM Cortex-M 默认一致）。
 *
 *  根据实际网络需求，可在 mesh_config.h 中调整最大目标数量等参数。
 * ============================================================ */

/**
 * @brief RREQ（路由请求）报文 Payload 结构体，共 16 字节
 *
 * @details 当源节点需要到达某个目标节点但没有有效路由时，广播此报文。
 *          沿途每个中间节点收到后递增 hop_count，用 (orig_addr, rreq_id)
 *          进行去重，并建立到 originator 的反向路由。
 *
 * @note 字段 dest_seq 为 0 表示发起者不知道目标的序列号，
 *       此时中间节点不能用缓存路由代答（需要序列号不低于请求中的值才可代答）。
 */
typedef struct __attribute__((packed)) {
    uint8_t  flags;         /**< 标志位，当前保留未使用，固定填 0。可根据实际需要扩展 */
    uint8_t  hop_count;     /**< 跳数：从 originator 到当前转发节点经过的跳数，初始为 0，每转发一次加 1 */
    uint16_t rreq_id;       /**< RREQ 唯一标识：由 originator 本地自增生成，配合 orig_addr 做全局去重 */
    uint16_t dest_addr;     /**< 目标地址：需要发现路由的目标节点 mesh 地址 */
    uint32_t dest_seq;      /**< 目标序列号：originator 已知的最新目标序列号，0 表示未知 */
    uint16_t orig_addr;     /**< 发起者地址：发起本次路由请求的源节点 mesh 地址 */
    uint32_t orig_seq;      /**< 发起者序列号：originator 当前的自身序列号，用于反向路由新鲜度判断 */
} mesh_rreq_payload_t;

/**
 * @brief RREP（路由应答）报文 Payload 结构体，共 14 字节
 *
 * @details 目标节点或拥有足够新鲜路由的中间节点收到 RREQ 后，通过此报文
 *          沿反向路由单播回传给 originator。沿途每个节点建立到 dest 的
 *          正向路由，形成双向路由路径。
 *
 * @note hop_count 在目标节点生成 RREP 时初始为 0，每经过一个中间节点加 1，
 *       最终到达 originator 时即为完整路径跳数。
 *       lifetime_ms 决定路由条目有效时长，可在 mesh_config.h 中通过
 *       MESH_AODV_ROUTE_LIFETIME_MS 调整，默认建议 30000ms，
 *       根据网络流量频率可适当增减。
 */
typedef struct __attribute__((packed)) {
    uint8_t  flags;         /**< 标志位，当前保留未使用，固定填 0 */
    uint8_t  hop_count;     /**< 跳数：从目标节点到当前转发节点的跳数，每转发一次加 1 */
    uint16_t dest_addr;     /**< 目标地址：RREP 所回复的目标节点 mesh 地址 */
    uint32_t dest_seq;      /**< 目标序列号：目标节点的最新序列号，保证路由新鲜度 */
    uint16_t orig_addr;     /**< 发起者地址：RREQ 的原始发起节点，RREP 需回传到此地址 */
    uint32_t lifetime_ms;   /**< 路由有效时间（毫秒）：路由条目的生存时间，超时后条目失效 */
} mesh_rrep_payload_t;

/**
 * @brief RERR（路由错误）中单个不可达目标条目
 *
 * @details 每个条目描述一个因链路断开而不再可达的目标节点。
 *          序列号在生成 RERR 时会在原值基础上加 1，
 *          表示该路由已过期（接收方据此判断自身路由是否受影响）。
 */
typedef struct __attribute__((packed)) {
    uint16_t dest_addr;     /**< 不可达目标的 mesh 地址 */
    uint32_t dest_seq;      /**< 不可达目标的递增后序列号（原 seq + 1，标示失效） */
} mesh_rerr_dest_t;

/**
 * @brief RERR（路由错误）报文 Payload 结构体，1 + N*6 字节，最大 49 字节
 *
 * @details 当节点检测到与某邻居的链路断开时，将所有经由该邻居转发的路由
 *          收集到 RERR 中广播通知，收到的节点检查本地路由表并按需继续传播。
 *
 * @note dests 数组最大容量由 MESH_AODV_RERR_MAX_DEST 决定（默认 8），
 *       如果网络规模较大且路由表条目较多，可适当增大该值。
 *       dest_count 字段表示 dests 数组中有效条目数量。
 */
typedef struct __attribute__((packed)) {
    uint8_t          dest_count; /**< 不可达目标数量，取值范围 [0, MESH_AODV_RERR_MAX_DEST] */
    mesh_rerr_dest_t dests[8];   /**< 不可达目标数组，最大 MESH_AODV_RERR_MAX_DEST 个条目 */
} mesh_rerr_payload_t;

/* ============================================================
 *  路由表条目结构
 *
 *  每个条目记录到某个目标节点的完整路由信息，包含 AODV 协议所需的
 *  序列号、有效性标志和过期时间等字段。
 *
 *  路由表大小由 MESH_ROUTE_TABLE_SIZE 定义（默认 32 条），
 *  可根据实际网络节点数量在 mesh_config.h 中调整。
 *  当表满时新路由会添加失败，建议表大小 >= 预期网络节点数。
 * ============================================================ */

/**
 * @brief 路由表条目结构体（AODV 增强版）
 *
 * @details 每条路由记录到达特定目标节点的转发信息。AODV 协议通过
 *          dest_seq（目标序列号）保证路由新鲜度，避免路由环路。
 *          expire_ms 为绝对时间戳，超过该时间后路由自动失效。
 */
typedef struct {
    uint16_t dest_addr;         /**< 目标节点 mesh 地址 */
    uint16_t next_hop;          /**< 下一跳节点 mesh 地址（数据帧实际发送的直连邻居） */
    uint8_t  hop_count;         /**< 到目标的跳数（直连邻居为 1） */
    uint32_t dest_seq;          /**< 目标序列号：AODV 核心字段，用于判断路由新鲜度和防环 */
    bool     dest_seq_valid;    /**< 目标序列号有效标志：false 表示 seq 未知（被动学习路由） */
    uint32_t expire_ms;         /**< 路由过期绝对时间戳（毫秒），由 osal_get_tick_ms() 基准计算 */
    bool     valid;             /**< 路由有效标志：false 表示该条目可被覆盖或已被 RERR 置无效 */
} mesh_route_entry_t;

/* ============================================================
 *  路由模块初始化与反初始化
 * ============================================================ */

/**
 * @brief 初始化路由模块
 *
 * @details 清空路由表、RREQ 去重缓存和 RREQ 等待队列，
 *          重置本节点序列号和 RREQ ID 计数器。
 *          应在 mesh 协议栈启动时调用一次。
 *
 * @return ERRCODE_SUCC 初始化成功
 */
errcode_t mesh_route_init(void);

/**
 * @brief 反初始化路由模块
 *
 * @details 清空路由表，释放路由模块占用的资源。
 *          应在 mesh 协议栈关闭时调用。
 */
void mesh_route_deinit(void);

/* ============================================================
 *  路由表基础操作接口
 *
 *  以下接口兼容 P1 阶段被动路由学习（HELLO 两跳反向路由），
 *  同时也被 AODV 主动路由发现流程内部使用。
 *  外部模块（如转发层、传输层）通过这些接口查询和维护路由。
 * ============================================================ */

/**
 * @brief 查找到目标节点的下一跳地址
 *
 * @param dest_addr 目标节点 mesh 地址
 * @return 下一跳 mesh 地址；若无有效路由则返回 MESH_ADDR_UNASSIGNED
 *
 * @note 仅返回未过期且 valid 为 true 的路由条目。
 *       调用前无需检查路由是否存在，直接判断返回值即可。
 */
uint16_t mesh_route_lookup(uint16_t dest_addr);

/**
 * @brief 添加或更新路由条目（P1 兼容接口）
 *
 * @param dest_addr 目标节点 mesh 地址
 * @param next_hop  下一跳节点 mesh 地址
 * @param hop_count 到目标的跳数
 * @param dest_seq  目标序列号；为 0 时标记为被动学习路由（seq 无效），
 *                  不会覆盖已有的 AODV 主动学习路由
 * @return ERRCODE_SUCC 成功
 * @return ERRCODE_FAIL 路由表已满，无法添加
 *
 * @note 被动学习路由（dest_seq=0）使用 MESH_ROUTE_TIMEOUT_MS 作为有效期；
 *       AODV 主动学习路由（dest_seq>0）使用 MESH_AODV_ROUTE_LIFETIME_MS。
 *       可在 mesh_config.h 中调整这两个超时参数。
 */
errcode_t mesh_route_update(uint16_t dest_addr, uint16_t next_hop,
                             uint8_t hop_count, uint32_t dest_seq);

/**
 * @brief 删除指定目标的路由条目
 *
 * @param dest_addr 目标节点 mesh 地址
 *
 * @note 将匹配条目的 valid 标志置为 false，不做物理清除。
 */
void mesh_route_remove(uint16_t dest_addr);

/**
 * @brief 删除所有经由指定下一跳的路由条目
 *
 * @param next_hop 下一跳节点 mesh 地址
 *
 * @note 当检测到某邻居断开连接时，调用此函数批量失效相关路由。
 *       通常由 mesh_route_on_link_break() 内部调用。
 */
void mesh_route_remove_by_next_hop(uint16_t next_hop);

/**
 * @brief 路由表老化清理
 *
 * @details 遍历路由表，将所有已超过 expire_ms 的条目标记为无效。
 *          应由主循环周期性调用（建议间隔 1~5 秒，可根据实际需要调整）。
 */
void mesh_route_cleanup(void);

/**
 * @brief 获取当前有效路由条目数量
 *
 * @return 有效（valid=true）的路由条目总数
 */
uint8_t mesh_route_get_count(void);

/**
 * @brief 判断目标节点是否可达
 *
 * @param dest_addr 目标节点 mesh 地址
 * @return true 存在有效路由可达
 * @return false 无有效路由
 */
bool mesh_route_is_reachable(uint16_t dest_addr);

/**
 * @brief 查询到目标节点的跳数
 *
 * @param dest_addr 目标节点 mesh 地址
 * @return 跳数；直连邻居返回 1，若未找到路由则返回 0xFF
 *
 * @note 优先检查邻居连接池（直连邻居固定返回 1），
 *       然后再查路由表，确保返回最准确的跳数。
 */
uint8_t mesh_route_get_hop_count(uint16_t dest_addr);

/**
 * @brief 仅从路由表查询到目标节点的跳数（跳过邻居池检查）
 *
 * @param dest_addr 目标节点 mesh 地址
 * @return 路由表中记录的跳数；若未找到返回 0xFF
 *
 * @note 用于检测 P3 fallback 场景：当目标是直连邻居（hop_count=1）
 *       但路由表中记录跳数 >= 2 时，说明该连接可能是远距离 fallback
 *       建立的，链路质量较差，需要特殊处理。
 */
uint8_t mesh_route_get_table_hop_count(uint16_t dest_addr);

/**
 * @brief 获取路由表中所有可达目标的地址和跳数
 *
 * @param[out] addrs     输出数组，存放目标节点 mesh 地址
 * @param[out] hops      输出数组，存放对应的跳数
 * @param[in]  max_count 输出数组最大容量
 * @return 实际写入的条目数量
 *
 * @note 供 BLE 网关拓扑查询等上层功能使用。
 *       调用者需确保 addrs 和 hops 数组长度不小于 max_count。
 */
uint8_t mesh_route_get_all_destinations(uint16_t *addrs, uint8_t *hops, uint8_t max_count);

/* ============================================================
 *  AODV 核心协议接口
 *
 *  以下接口实现 AODV 路由协议的核心报文处理逻辑：
 *  - mesh_route_request()：发起路由发现（RREQ 广播）
 *  - mesh_route_process_rreq/rrep/rerr()：处理收到的控制报文
 *  - mesh_route_on_link_break()：链路断开通知
 *  - mesh_route_process_rreq_timeouts()：RREQ 超时重试
 *  - mesh_route_touch()：活跃路由保活
 *
 *  这些函数由转发层（mesh_forward）在收到对应类型帧时回调。
 * ============================================================ */

/**
 * @brief 发起路由请求（RREQ 广播）
 *
 * @param dest_addr 需要发现路由的目标节点 mesh 地址
 *
 * @details 当转发层发现没有到达 dest_addr 的有效路由时调用此函数。
 *          内部自增本节点序列号和 RREQ ID，构建 RREQ 报文并广播给所有邻居。
 *          同时将请求加入 RREQ 等待队列，等待 RREP 回复。
 *
 * @note 如果该目标已有 RREQ 正在等待中（尚未超时），不会重复发起。
 *       RREQ 超时后会自动重试，最大重试次数由 MESH_AODV_RREQ_RETRIES 控制。
 *       超时时间由 MESH_AODV_RREQ_TIMEOUT_MS 控制，可在 mesh_config.h 中调整。
 */
void mesh_route_request(uint16_t dest_addr);

/**
 * @brief 处理收到的 RREQ（路由请求）报文
 *
 * @param conn_id     接收该帧的 SLE 连接 ID，用于确定发送方邻居
 * @param header      已解析的 mesh 帧头，包含 src_addr 等信息
 * @param payload     RREQ payload 原始数据指针
 * @param payload_len payload 数据长度（字节），应 >= sizeof(mesh_rreq_payload_t)
 *
 * @details 处理流程：
 *          1. 忽略自己发起的 RREQ（orig_addr == 本节点地址）
 *          2. RREQ 去重：(orig_addr, rreq_id) 已缓存则丢弃
 *          3. 建立/更新到 originator 的反向路由（next_hop = 发送方邻居）
 *          4. 若本节点是目标 → 生成 RREP 单播回传
 *          5. 若本节点拥有到目标的新鲜路由 → 中间节点代答 RREP
 *          6. 否则 → hop_count++ 重新广播 RREQ（洪泛转发）
 *
 * @note 由转发层在收到 MESH_MSG_RREQ 类型帧时回调。
 */
void mesh_route_process_rreq(uint16_t conn_id, const mesh_header_t *header,
                              const uint8_t *payload, uint16_t payload_len);

/**
 * @brief 处理收到的 RREP（路由应答）报文
 *
 * @param conn_id     接收该帧的 SLE 连接 ID
 * @param header      已解析的 mesh 帧头
 * @param payload     RREP payload 原始数据指针
 * @param payload_len payload 数据长度（字节），应 >= sizeof(mesh_rrep_payload_t)
 *
 * @details 处理流程：
 *          1. 建立/更新到目标节点（dest）的正向路由（next_hop = 发送方邻居）
 *          2. 若本节点是 originator → 路由发现成功，从等待队列移除，
 *             触发 pending queue 中缓存数据的立即发送
 *          3. 否则（中间节点）→ hop_count++ 转发 RREP 给 originator 方向
 *
 * @note 由转发层在收到 MESH_MSG_RREP 类型帧时回调。
 */
void mesh_route_process_rrep(uint16_t conn_id, const mesh_header_t *header,
                              const uint8_t *payload, uint16_t payload_len);

/**
 * @brief 处理收到的 RERR（路由错误）报文
 *
 * @param conn_id     接收该帧的 SLE 连接 ID（本函数中未使用）
 * @param header      已解析的 mesh 帧头（本函数中未使用）
 * @param payload     RERR payload 原始数据指针
 * @param payload_len payload 数据长度（字节），最小 1 字节
 *
 * @details 处理流程：
 *          1. 解析 RERR 中列出的不可达目标列表
 *          2. 遍历本节点路由表，将匹配的路由条目标记为无效
 *          3. 将本节点受影响的条目收集到转发 RERR 中，继续广播传播
 *
 * @note 由转发层在收到 MESH_MSG_RERR 类型帧时回调。
 *       仅当本节点确实有路由受影响时才会传播 RERR，避免不必要的广播。
 */
void mesh_route_process_rerr(uint16_t conn_id, const mesh_header_t *header,
                              const uint8_t *payload, uint16_t payload_len);

/**
 * @brief 链路断开事件处理，生成并广播 RERR 报文
 *
 * @param broken_next_hop 断开连接的邻居节点 mesh 地址
 *
 * @details 当检测到与某个邻居的 SLE 连接断开时，由传输层调用此函数。
 *          遍历路由表找出所有 next_hop 为该邻居的路由条目，
 *          将它们全部标记为无效，并将序列号加 1 后封装到 RERR 中广播。
 *
 * @note broken_next_hop 为 MESH_ADDR_UNASSIGNED 时直接返回不处理。
 *       RERR 中最多携带 MESH_AODV_RERR_MAX_DEST 个不可达目标。
 */
void mesh_route_on_link_break(uint16_t broken_next_hop);

/**
 * @brief RREQ 超时检查与重试
 *
 * @details 遍历 RREQ 等待队列，对已超时的条目执行重试：
 *          - 若重试次数未超过 MESH_AODV_RREQ_RETRIES → 重新生成 RREQ 广播
 *          - 若已达最大重试次数 → 放弃该路由发现请求，从等待队列移除
 *
 * @note 应由主循环周期性调用（建议与主循环间隔一致，默认约 100~500ms）。
 *       超时时间和最大重试次数可在 mesh_config.h 中调整。
 */
void mesh_route_process_rreq_timeouts(void);

/**
 * @brief 刷新路由条目的有效期（保活机制）
 *
 * @param dest_addr 目标节点 mesh 地址
 *
 * @details 当数据帧成功通过某条路由转发时，调用此函数延长该路由的
 *          有效期至 now + MESH_AODV_ACTIVE_TIMEOUT_MS，防止活跃路由
 *          因超时而失效。仅在新过期时间晚于当前过期时间时才更新。
 *
 * @note 建议在转发层每次成功转发数据帧后调用。
 *       MESH_AODV_ACTIVE_TIMEOUT_MS 可在 mesh_config.h 中调整。
 */
void mesh_route_touch(uint16_t dest_addr);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* MESH_ROUTE_H */