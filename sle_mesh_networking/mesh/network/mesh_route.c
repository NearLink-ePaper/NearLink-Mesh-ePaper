/**
 * @file mesh_route.c
 * @brief SLE Mesh 路由层实现 — 简化 AODV（按需距离矢量）路由协议
 *
 * Copyright (c) 2026. All rights reserved.
 *
 * @details
 * 本文件实现了 SLE Mesh 网络的简化 AODV 路由协议核心逻辑，包括：
 *
 * - RREQ（路由请求）：广播洪泛发现路由，沿途中间节点建立到发起者的反向路由
 * - RREP（路由应答）：目标节点或中间节点单播回传，沿途建立到目标的正向路由
 * - RERR（路由错误）：链路断开时生成并广播，通知受影响节点失效相关路由
 * - 路由表管理：支持查找、添加、更新、删除、老化清理等操作
 * - 兼容 P1 阶段的被动路由学习（通过 HELLO 两跳、反向路由学习建立路由）
 *
 * 内部核心数据结构：
 * - g_route_table[]：路由表，最多 MESH_ROUTE_TABLE_SIZE 条（默认 32）
 * - g_rreq_cache[]：RREQ 去重缓存，防止重复处理相同的 RREQ
 * - g_rreq_wait[]：RREQ 等待队列，跟踪待答复的 RREQ 并支持超时重试
 *
 * @note 各项配置参数（路由表大小、超时时间、重试次数等）均在 mesh_config.h 中定义，
 *       可根据实际网络规模和性能需求进行调整。
 *
 * History:
 * 2026-02-07, 创建文件（桩实现）。
 * 2026-02-08, P2: 完整 AODV 路由协议实现。
 */

#include "common_def.h"
#include "securec.h"
#include "soc_osal.h"
#include "osal_debug.h"

#include "mesh_types.h"
#include "mesh_config.h"
#include "mesh_route.h"
#include "mesh_transport.h"
#include "mesh_forward.h"

/* ============================================================
 *  路由表
 *
 *  静态数组实现，最大容量由 MESH_ROUTE_TABLE_SIZE 控制（默认 32）。
 *  每个条目包含目标地址、下一跳、跳数、序列号等 AODV 字段。
 *  valid=false 的条目视为空闲槽位，可被新路由覆写。
 *  可根据网络节点数量在 mesh_config.h 中调整表大小。
 * ============================================================ */
static mesh_route_entry_t g_route_table[MESH_ROUTE_TABLE_SIZE];

/* ============================================================
 *  AODV 节点本地状态
 *
 *  g_own_seq:     本节点的 AODV 序列号，每次发起 RREQ 或回复 RREP 时自增。
 *                 初始值由 MESH_AODV_SEQ_INIT 定义（默认 1）。
 *  g_rreq_id_seq: RREQ ID 自增计数器，配合 orig_addr 实现全局唯一标识。
 * ============================================================ */
static uint32_t g_own_seq = MESH_AODV_SEQ_INIT;   /**< 本节点 AODV 序列号，单调递增，用于路由新鲜度判断 */
static uint16_t g_rreq_id_seq = 0;                 /**< RREQ ID 自增计数器，每次发起新 RREQ 时加 1 */

/* ============================================================
 *  RREQ 去重缓存
 *
 *  用于防止重复处理相同的 RREQ 报文。每条记录以 (orig_addr, rreq_id)
 *  为键进行去重，配合过期时间自动清理。
 *  缓存大小由 MESH_AODV_RREQ_CACHE_SIZE 控制（默认 16），
 *  当缓存满时会替换最早过期的条目。
 *  可根据网络广播频率和节点数调整缓存大小。
 * ============================================================ */
typedef struct {
    uint16_t orig_addr;     /**< RREQ 发起者的 mesh 地址 */
    uint16_t rreq_id;       /**< RREQ 唯一标识 ID */
    uint32_t expire_ms;     /**< 该缓存条目的过期绝对时间戳（毫秒） */
} mesh_rreq_cache_entry_t;

static mesh_rreq_cache_entry_t g_rreq_cache[MESH_AODV_RREQ_CACHE_SIZE];

/* ============================================================
 *  RREQ 等待队列
 *
 *  跟踪本节点已发起但尚未收到 RREP 的 RREQ 请求。
 *  每个条目记录目标地址、重试次数和下次重试时间。
 *  队列大小由 MESH_AODV_RREQ_WAIT_SIZE 控制（默认 4），
 *  表示同时最多支持 4 个并行的路由发现请求。
 *  可根据实际并发需求在 mesh_config.h 中调整。
 * ============================================================ */
typedef struct {
    bool     active;        /**< 该条目是否活跃（true = 正在等待 RREP） */
    uint16_t dest_addr;     /**< 等待路由的目标节点 mesh 地址 */
    uint16_t rreq_id;       /**< 当前使用的 RREQ ID（重试时会更新） */
    uint8_t  retry_count;   /**< 已重试次数，达到 MESH_AODV_RREQ_RETRIES 后放弃 */
    uint32_t next_retry_ms; /**< 下次重试的绝对时间戳（毫秒） */
} mesh_rreq_wait_entry_t;

static mesh_rreq_wait_entry_t g_rreq_wait[MESH_AODV_RREQ_WAIT_SIZE];

/* ============================================================
 *  辅助函数：序列号新鲜度比较（处理 32 位无符号整数回绕）
 *
 *  采用有符号差值法：将 (a - b) 转换为 int32_t，若差值 > 0 则认为 a 更新。
 *  这种方法在序列号回绕时仍能正确比较，前提是两个序列号的
 *  差值不超过 2^31（即约 21 亿），实际使用中远达不到此限制。
 *
 *  @param a 待比较的序列号 a
 *  @param b 待比较的序列号 b
 *  @return true  a 比 b 更新（即 a 是较近期生成的序列号）
 *  @return false a 不比 b 新（a == b 或 a 更旧）
 * ============================================================ */
static bool seq_newer(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) > 0;
}

/* ============================================================
 *  辅助函数：判断是否应该更新已有路由条目
 *
 *  根据 AODV 协议规则（简化版 RFC 3561 Section 6.2）：
 *    1. 已有路由无效（valid=false）→ 无条件更新
 *    2. 旧条目序列号无效而新条目有效 → 更新（有效 seq 始终优先）
 *    3. 新 seq 比旧 seq 更新（seq_newer 判断）→ 更新（更新鲜的路由信息）
 *    4. 新旧 seq 相同且新跳数更少 → 更新（相同新鲜度下选择更短路径）
 *
 *  @param existing     已有路由条目指针
 *  @param new_seq      新路由信息中的目标序列号
 *  @param new_seq_valid 新序列号是否有效
 *  @param new_hops     新路由信息中的跳数
 *  @return true  应当更新已有路由
 *  @return false 应当保留已有路由不变
 * ============================================================ */
static bool should_update_route(const mesh_route_entry_t *existing,
                                 uint32_t new_seq, bool new_seq_valid,
                                 uint8_t new_hops)
{
    if (!existing->valid) {
        return true;
    }
    if (!existing->dest_seq_valid && new_seq_valid) {
        return true;
    }
    if (new_seq_valid && existing->dest_seq_valid) {
        if (seq_newer(new_seq, existing->dest_seq)) {
            return true;
        }
        if (new_seq == existing->dest_seq && new_hops < existing->hop_count) {
            return true;
        }
    }
    /* P23: 允许被动路由之间的跳数优化更新。
     * 当新旧路由都没有有效序列号时（均为被动学习，如 HELLO 2-hop
     * 或转发帧反向路由），仍允许跳数更小的路由覆盖跳数更大的路由，
     * 防止首次学到的次优路径永久锁定，确保 HELLO 持续优化路由。 */
    if (!new_seq_valid && !existing->dest_seq_valid && new_hops < existing->hop_count) {
        return true;
    }
    return false;
}

/* ============================================================
 *  辅助函数：RREQ 缓存去重
 *
 *  检查 (orig_addr, rreq_id) 是否已在缓存中：
 *  - 若已存在且未过期，返回 true（重复 RREQ，应丢弃）
 *  - 若不存在，将其加入缓存并返回 false（新 RREQ，应处理）
 *
 *  缓存替换策略：优先使用已过期的槽位，若无过期槽位则替换最早过期的。
 *  缓存有效期为 MESH_AODV_RREQ_TIMEOUT_MS * 2，确保覆盖完整的重试周期。
 *
 *  @param orig_addr RREQ 发起者 mesh 地址
 *  @param rreq_id   RREQ 唯一标识 ID
 *  @return true  重复 RREQ（已见过，应丢弃）
 *  @return false 新 RREQ（已加入缓存，应继续处理）
 * ============================================================ */
static bool rreq_cache_check_and_add(uint16_t orig_addr, uint16_t rreq_id)
{
    uint32_t now = osal_get_tick_ms();

    /* 遍历缓存，检查是否已存在未过期的相同 (orig_addr, rreq_id) 记录 */
    for (uint8_t i = 0; i < MESH_AODV_RREQ_CACHE_SIZE; i++) {
        if (g_rreq_cache[i].orig_addr == orig_addr &&
            g_rreq_cache[i].rreq_id == rreq_id &&
            now < g_rreq_cache[i].expire_ms) {
            return true; /* 重复 RREQ，直接返回 */
        }
    }

    /* 未找到重复记录，需要加入缓存。优先查找已过期的空闲槽位，否则替换最旧的槽位 */
    uint8_t slot = 0;
    uint32_t oldest = UINT32_MAX;
    for (uint8_t i = 0; i < MESH_AODV_RREQ_CACHE_SIZE; i++) {
        if (now >= g_rreq_cache[i].expire_ms) {
            slot = i;
            break;
        }
        if (g_rreq_cache[i].expire_ms < oldest) {
            oldest = g_rreq_cache[i].expire_ms;
            slot = i;
        }
    }

    g_rreq_cache[slot].orig_addr = orig_addr;
    g_rreq_cache[slot].rreq_id = rreq_id;
    g_rreq_cache[slot].expire_ms = now + MESH_AODV_RREQ_TIMEOUT_MS * 2;
    return false;
}

/* ============================================================
 *  辅助函数：在 RREQ 等待队列中查找指定目标
 *
 *  @param dest_addr 目标节点 mesh 地址
 *  @return 找到则返回对应等待条目指针，未找到返回 NULL
 * ============================================================ */
static mesh_rreq_wait_entry_t *rreq_wait_find(uint16_t dest_addr)
{
    for (uint8_t i = 0; i < MESH_AODV_RREQ_WAIT_SIZE; i++) {
        if (g_rreq_wait[i].active && g_rreq_wait[i].dest_addr == dest_addr) {
            return &g_rreq_wait[i];
        }
    }
    return NULL;
}

/* ============================================================
 *  辅助函数：从 RREQ 等待队列中移除指定目标
 *
 *  收到 RREP 后调用，将对应目标的等待条目标记为非活跃。
 *
 *  @param dest_addr 目标节点 mesh 地址
 * ============================================================ */
static void rreq_wait_remove(uint16_t dest_addr)
{
    for (uint8_t i = 0; i < MESH_AODV_RREQ_WAIT_SIZE; i++) {
        if (g_rreq_wait[i].active && g_rreq_wait[i].dest_addr == dest_addr) {
            g_rreq_wait[i].active = false;
            return;
        }
    }
}

/* ============================================================
 *  辅助函数：构建并广播发送 RREQ 帧
 *
 *  将 RREQ payload 封装为 mesh 帧，目标地址设为 MESH_ADDR_BROADCAST，
 *  通过 mesh_transport_send_to_all_neighbors() 发送给所有直连邻居。
 *
 *  @param rreq RREQ payload 结构体指针
 * ============================================================ */
static void send_rreq(const mesh_rreq_payload_t *rreq)
{
    mesh_frame_t frame;
    uint16_t total_len = mesh_forward_build_frame(&frame, MESH_MSG_RREQ,
                                                   MESH_ADDR_BROADCAST,
                                                   (const uint8_t *)rreq,
                                                   sizeof(mesh_rreq_payload_t));
    if (total_len > 0) {
        mesh_transport_send_to_all_neighbors((const uint8_t *)&frame, total_len, 0xFFFF);
    }
}

/* ============================================================
 *  辅助函数：构建并单播发送 RREP 帧
 *
 *  RREP 需要沿反向路由回传给 originator。发送策略：
 *  1. 优先检查 dst_addr 是否为直连邻居，若是则直接发送
 *  2. 否则查路由表找到下一跳，转发给下一跳邻居
 *  3. 若无反向路由（异常容错），退化为洪泛发送
 *
 *  @param dst_addr RREP 的目标接收方（即 RREQ 的 originator）
 *  @param rrep     RREP payload 结构体指针
 * ============================================================ */
static void send_rrep(uint16_t dst_addr, const mesh_rrep_payload_t *rrep)
{
    mesh_frame_t frame;
    uint16_t total_len = mesh_forward_build_frame(&frame, MESH_MSG_RREP,
                                                   dst_addr,
                                                   (const uint8_t *)rrep,
                                                   sizeof(mesh_rrep_payload_t));
    if (total_len == 0) {
        return;
    }

    /* RREP 沿反向路由回传给 originator，优先用直连邻居发送 */
    /* 检查目标是否为直连邻居 */
    if (mesh_transport_is_direct_neighbor(dst_addr)) {
        mesh_transport_send_to_neighbor(dst_addr, (const uint8_t *)&frame, total_len);
        return;
    }
    /* 查路由表获取下一跳地址 */
    uint16_t next_hop = mesh_route_lookup(dst_addr);
    if (next_hop != MESH_ADDR_UNASSIGNED) {
        mesh_transport_send_to_neighbor(next_hop, (const uint8_t *)&frame, total_len);
    } else {
        /* 无反向路由（理论上不应发生，因为 RREQ 沿途已建立反向路由），作为容错退化为洪泛发送 */
        osal_printk("%s RREP: no reverse route to orig=0x%04X, flooding\r\n",
                    MESH_LOG_TAG, dst_addr);
        mesh_transport_send_to_all_neighbors((const uint8_t *)&frame, total_len, 0xFFFF);
    }
}

/* ============================================================
 *  辅助函数：构建并广播发送 RERR 帧
 *
 *  将 RERR payload 封装为 mesh 帧，目标地址设为 MESH_ADDR_BROADCAST，
 *  通过 mesh_transport_send_to_all_neighbors() 广播给所有直连邻居。
 *  payload 实际长度为 1 + dest_count * sizeof(mesh_rerr_dest_t)，
 *  不发送末尾未使用的空闲字节。
 *
 *  @param rerr RERR payload 结构体指针
 * ============================================================ */
static void send_rerr(const mesh_rerr_payload_t *rerr)
{
    uint16_t payload_len = 1 + rerr->dest_count * sizeof(mesh_rerr_dest_t);

    mesh_frame_t frame;
    uint16_t total_len = mesh_forward_build_frame(&frame, MESH_MSG_RERR,
                                                   MESH_ADDR_BROADCAST,
                                                   (const uint8_t *)rerr,
                                                   payload_len);
    if (total_len > 0) {
        mesh_transport_send_to_all_neighbors((const uint8_t *)&frame, total_len, 0xFFFF);
    }
}

/* ============================================================
 *  辅助函数：内部路由更新（带完整 AODV 字段）
 *
 *  所有路由更新操作的核心入口。处理流程：
 *  1. 遍历路由表查找已有的同目标条目
 *  2. 若找到，通过 should_update_route() 判断是否需要更新
 *  3. 若未找到，在空闲槽位中创建新条目
 *  4. 若无空闲槽位，返回 ERRCODE_FAIL（路由表已满）
 *
 *  @param dest_addr     目标节点 mesh 地址
 *  @param next_hop      下一跳节点 mesh 地址
 *  @param hop_count     到目标的跳数
 *  @param dest_seq      目标序列号
 *  @param dest_seq_valid 目标序列号是否有效
 *  @param lifetime_ms   路由有效时间（毫秒），用于计算过期绝对时间戳
 *  @return ERRCODE_SUCC 成功
 *  @return ERRCODE_FAIL 路由表已满
 * ============================================================ */
static errcode_t route_update_internal(uint16_t dest_addr, uint16_t next_hop,
                                        uint8_t hop_count, uint32_t dest_seq,
                                        bool dest_seq_valid, uint32_t lifetime_ms)
{
    /* 第一步：遍历路由表，查找已有的同目标条目并尝试更新 */
    for (uint8_t i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (g_route_table[i].valid && g_route_table[i].dest_addr == dest_addr) {
            if (should_update_route(&g_route_table[i], dest_seq, dest_seq_valid, hop_count)) {
                g_route_table[i].next_hop = next_hop;
                g_route_table[i].hop_count = hop_count;
                g_route_table[i].dest_seq = dest_seq;
                g_route_table[i].dest_seq_valid = dest_seq_valid;
                g_route_table[i].expire_ms = osal_get_tick_ms() + lifetime_ms;
                if (!g_mesh_log_suppress) {
                    osal_printk("%s route update: dest=0x%04X next=0x%04X hops=%d seq=%u\r\n",
                                MESH_LOG_TAG, dest_addr, next_hop, hop_count,
                                (unsigned int)dest_seq);
                }
            }
            return ERRCODE_SUCC;
        }
    }

    /* 第二步：未找到已有条目，查找空闲槽位创建新路由条目 */
    for (uint8_t i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (!g_route_table[i].valid) {
            g_route_table[i].dest_addr = dest_addr;
            g_route_table[i].next_hop = next_hop;
            g_route_table[i].hop_count = hop_count;
            g_route_table[i].dest_seq = dest_seq;
            g_route_table[i].dest_seq_valid = dest_seq_valid;
            g_route_table[i].expire_ms = osal_get_tick_ms() + lifetime_ms;
            g_route_table[i].valid = true;
            if (!g_mesh_log_suppress) {
                osal_printk("%s route add: dest=0x%04X next=0x%04X hops=%d seq=%u\r\n",
                            MESH_LOG_TAG, dest_addr, next_hop, hop_count,
                            (unsigned int)dest_seq);
            }
            return ERRCODE_SUCC;
        }
    }

    osal_printk("%s route table full!\r\n", MESH_LOG_TAG);
    return ERRCODE_FAIL;
}

/* ============================================================
 *  公开接口：路由模块初始化
 *
 *  清空所有内部数据结构（路由表、RREQ 缓存、RREQ 等待队列），
 *  重置 AODV 节点状态。应在 mesh 协议栈启动时调用一次。
 * ============================================================ */
errcode_t mesh_route_init(void)
{
    (void)memset_s(g_route_table, sizeof(g_route_table), 0, sizeof(g_route_table));
    (void)memset_s(g_rreq_cache, sizeof(g_rreq_cache), 0, sizeof(g_rreq_cache));
    (void)memset_s(g_rreq_wait, sizeof(g_rreq_wait), 0, sizeof(g_rreq_wait));
    g_own_seq = MESH_AODV_SEQ_INIT;
    g_rreq_id_seq = 0;
    osal_printk("%s route init ok (P2 AODV)\r\n", MESH_LOG_TAG);
    return ERRCODE_SUCC;
}

void mesh_route_deinit(void)
{
    (void)memset_s(g_route_table, sizeof(g_route_table), 0, sizeof(g_route_table));
}

/* ============================================================
 *  公开接口：路由查找
 *
 *  遍历路由表，返回到指定目标的有效且未过期的下一跳地址。
 *  若未找到有效路由，返回 MESH_ADDR_UNASSIGNED。
 * ============================================================ */
uint16_t mesh_route_lookup(uint16_t dest_addr)
{
    uint32_t now = osal_get_tick_ms();
    for (uint8_t i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (g_route_table[i].valid &&
            g_route_table[i].dest_addr == dest_addr &&
            now < g_route_table[i].expire_ms) {
            return g_route_table[i].next_hop;
        }
    }
    return MESH_ADDR_UNASSIGNED;
}

uint8_t mesh_route_get_hop_count(uint16_t dest_addr)
{
    /* 优先检查邻居连接池：若目标为直连邻居，固定返回 1 跳（无需查路由表） */
    if (mesh_transport_is_direct_neighbor(dest_addr)) {
        return 1;
    }

    uint32_t now = osal_get_tick_ms();
    for (uint8_t i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (g_route_table[i].valid &&
            g_route_table[i].dest_addr == dest_addr &&
            now < g_route_table[i].expire_ms) {
            return g_route_table[i].hop_count;
        }
    }
    return 0xFF;  /* 未找到有效路由，返回 0xFF 作为无效跳数标识 */
}

uint8_t mesh_route_get_table_hop_count(uint16_t dest_addr)
{
    /* 仅查询路由表，不检查邻居池。用于检测 P3 fallback 场景：
     * 当目标为直连邻居（hops=1）但路由表记录跳数 >= 2 时，
     * 说明连接是通过 P3 fallback 建立的远距离连接，链路质量可能较差，
     * 转发层可据此做特殊处理（如优先选择多跳路由）。 */
    uint32_t now = osal_get_tick_ms();
    for (uint8_t i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (g_route_table[i].valid &&
            g_route_table[i].dest_addr == dest_addr &&
            now < g_route_table[i].expire_ms) {
            return g_route_table[i].hop_count;
        }
    }
    return 0xFF;
}

/* ============================================================
 *  公开接口：路由更新（P1 兼容接口）
 *
 *  封装 route_update_internal()，根据 dest_seq 自动判断：
 *  - dest_seq > 0：AODV 主动学习路由，序列号有效，
 *    使用 MESH_AODV_ROUTE_LIFETIME_MS 作为生存时间
 *  - dest_seq == 0：被动学习路由（如 HELLO 反向学习），序列号无效，
 *    使用 MESH_ROUTE_TIMEOUT_MS 作为生存时间，不会覆盖已有的 AODV 路由
 * ============================================================ */
errcode_t mesh_route_update(uint16_t dest_addr, uint16_t next_hop,
                             uint8_t hop_count, uint32_t dest_seq)
{
    bool seq_valid = (dest_seq > 0);
    uint32_t lifetime = (dest_seq > 0) ? MESH_AODV_ROUTE_LIFETIME_MS : MESH_ROUTE_TIMEOUT_MS;
    return route_update_internal(dest_addr, next_hop, hop_count, dest_seq, seq_valid, lifetime);
}

void mesh_route_remove(uint16_t dest_addr)
{
    for (uint8_t i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (g_route_table[i].valid && g_route_table[i].dest_addr == dest_addr) {
            g_route_table[i].valid = false;
            return;
        }
    }
}

void mesh_route_remove_by_next_hop(uint16_t next_hop)
{
    for (uint8_t i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (g_route_table[i].valid && g_route_table[i].next_hop == next_hop) {
            osal_printk("%s route remove: dest=0x%04X (next_hop 0x%04X down)\r\n",
                        MESH_LOG_TAG, g_route_table[i].dest_addr, next_hop);
            g_route_table[i].valid = false;
        }
    }
}

void mesh_route_cleanup(void)
{
    uint32_t now = osal_get_tick_ms();
    for (uint8_t i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (g_route_table[i].valid && now >= g_route_table[i].expire_ms) {
            osal_printk("%s route expired: dest=0x%04X\r\n",
                        MESH_LOG_TAG, g_route_table[i].dest_addr);
            g_route_table[i].valid = false;
        }
    }
}

uint8_t mesh_route_get_count(void)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (g_route_table[i].valid) {
            count++;
        }
    }
    return count;
}

bool mesh_route_is_reachable(uint16_t dest_addr)
{
    return (mesh_route_lookup(dest_addr) != MESH_ADDR_UNASSIGNED);
}

void mesh_route_touch(uint16_t dest_addr)
{
    uint32_t now = osal_get_tick_ms();
    for (uint8_t i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (g_route_table[i].valid && g_route_table[i].dest_addr == dest_addr) {
            uint32_t new_expire = now + MESH_AODV_ACTIVE_TIMEOUT_MS;
            if (new_expire > g_route_table[i].expire_ms) {
                g_route_table[i].expire_ms = new_expire;
            }
            return;
        }
    }
}

/* ============================================================
 *  AODV 核心：发起路由请求（RREQ 广播）
 *
 *  当转发层发现没有到达目标的有效路由时调用。
 *  内部自增本节点序列号和 RREQ ID，构建 RREQ 报文并广播。
 *  同时记录到去重缓存和等待队列，用于后续去重和超时重试。
 * ============================================================ */
void mesh_route_request(uint16_t dest_addr)
{
    /* 防止重复发起：若该目标已有 RREQ 正在等待队列中，直接返回 */
    if (rreq_wait_find(dest_addr) != NULL) {
        return;
    }

    /* 自增本节点序列号和 RREQ ID 计数器，保证全局唯一性 */
    g_own_seq++;
    g_rreq_id_seq++;

    /* 从路由表中查找已知的目标序列号（若存在，填入 RREQ 可帮助中间节点代答） */
    uint32_t known_dest_seq = 0;
    for (uint8_t i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (g_route_table[i].dest_addr == dest_addr && g_route_table[i].dest_seq_valid) {
            known_dest_seq = g_route_table[i].dest_seq;
            break;
        }
    }

    /* 构建 RREQ payload，hop_count 初始为 0（源节点） */
    mesh_rreq_payload_t rreq;
    (void)memset_s(&rreq, sizeof(rreq), 0, sizeof(rreq));
    rreq.flags = 0;
    rreq.hop_count = 0;
    rreq.rreq_id = g_rreq_id_seq;
    rreq.dest_addr = dest_addr;
    rreq.dest_seq = known_dest_seq;
    rreq.orig_addr = g_mesh_node_addr;
    rreq.orig_seq = g_own_seq;

    /* 将自己的 RREQ 记录到去重缓存，防止转发回来时重复处理 */
    rreq_cache_check_and_add(g_mesh_node_addr, g_rreq_id_seq);

    /* 加入 RREQ 等待队列，用于超时重试和收到 RREP 后清理 */
    for (uint8_t i = 0; i < MESH_AODV_RREQ_WAIT_SIZE; i++) {
        if (!g_rreq_wait[i].active) {
            g_rreq_wait[i].active = true;
            g_rreq_wait[i].dest_addr = dest_addr;
            g_rreq_wait[i].rreq_id = g_rreq_id_seq;
            g_rreq_wait[i].retry_count = 0;
            g_rreq_wait[i].next_retry_ms = osal_get_tick_ms() + MESH_AODV_RREQ_TIMEOUT_MS;
            break;
        }
    }

    osal_printk("%s RREQ TX: dest=0x%04X orig=0x%04X rreq_id=%d orig_seq=%u\r\n",
                MESH_LOG_TAG, dest_addr, g_mesh_node_addr, g_rreq_id_seq,
                (unsigned int)g_own_seq);

    send_rreq(&rreq);
}

/* ============================================================
 *  AODV 核心：处理收到的 RREQ（路由请求）
 *
 *  完整处理流程：
 *    1. 校验 payload 长度，解析 RREQ 结构体
 *    2. 忽略本节点自己发起的 RREQ（防止自环）
 *    3. RREQ 去重：检查 (orig_addr, rreq_id) 是否已处理过
 *    4. 建立/更新到 originator 的反向路由，next_hop 为发送方邻居
 *    5. 若本节点是目标 → 生成 RREP 单播回传
 *    6. 若本节点拥有到目标的新鲜路由 → 中间节点代答 RREP
 *    7. 否则 → hop_count++ 重新广播 RREQ（洪泛转发）
 * ============================================================ */
void mesh_route_process_rreq(uint16_t conn_id, const mesh_header_t *header,
                              const uint8_t *payload, uint16_t payload_len)
{
    if (payload_len < sizeof(mesh_rreq_payload_t)) {
        return;
    }

    const mesh_rreq_payload_t *rreq = (const mesh_rreq_payload_t *)payload;

    osal_printk("%s RREQ RX: dest=0x%04X orig=0x%04X rreq_id=%d hops=%d\r\n",
                MESH_LOG_TAG, rreq->dest_addr, rreq->orig_addr,
                rreq->rreq_id, rreq->hop_count);

    /* 步骤 1：忽略本节点自己发起的 RREQ（广播可能经多跳转回） */
    if (rreq->orig_addr == g_mesh_node_addr) {
        return;
    }

    /* 步骤 2：RREQ 去重 — 检查 (orig_addr, rreq_id) 是否已处理过 */
    if (rreq_cache_check_and_add(rreq->orig_addr, rreq->rreq_id)) {
        osal_printk("%s RREQ: dup (orig=0x%04X rreq_id=%d), drop\r\n",
                    MESH_LOG_TAG, rreq->orig_addr, rreq->rreq_id);
        return;
    }

    /* 步骤 3：建立/更新到 originator 的反向路由，下一跳为发送方邻居 */
    uint16_t sender_addr = mesh_transport_get_mesh_addr(conn_id);
    if (sender_addr == MESH_ADDR_UNASSIGNED) {
        /* transport 层尚未完成地址解析时，用帧头 src_addr 作为发送方地址 */
        sender_addr = header->src_addr;
    }

    route_update_internal(rreq->orig_addr, sender_addr,
                           rreq->hop_count + 1, rreq->orig_seq,
                           true, MESH_AODV_ROUTE_LIFETIME_MS);

    /* 步骤 4：检查本节点是否为 RREQ 的目标节点 */
    if (rreq->dest_addr == g_mesh_node_addr) {
        osal_printk("%s RREQ: I am the destination!\r\n", MESH_LOG_TAG);

        /* AODV 规则：目标节点更新自己的序列号为 max(own_seq, rreq.dest_seq) + 1
         * 确保回复的 RREP 序列号始终大于发起者已知的序列号 */
        if (rreq->dest_seq > 0 && seq_newer(rreq->dest_seq, g_own_seq)) {
            g_own_seq = rreq->dest_seq;
        }
        g_own_seq++;

        /* 生成 RREP：目标地址填本节点，hop_count=0（起始点），沿反向路由发送 */
        mesh_rrep_payload_t rrep;
        (void)memset_s(&rrep, sizeof(rrep), 0, sizeof(rrep));
        rrep.flags = 0;
        rrep.hop_count = 0;
        rrep.dest_addr = g_mesh_node_addr;
        rrep.dest_seq = g_own_seq;
        rrep.orig_addr = rreq->orig_addr;
        rrep.lifetime_ms = MESH_AODV_ROUTE_LIFETIME_MS;

        osal_printk("%s RREP TX: dest=0x%04X dest_seq=%u orig=0x%04X hops=0\r\n",
                    MESH_LOG_TAG, g_mesh_node_addr,
                    (unsigned int)g_own_seq, rreq->orig_addr);

        send_rrep(rreq->orig_addr, &rrep);
        return;
    }

    /* 步骤 5：中间节点检查是否拥有到目标的新鲜路由（可代答 RREP）
     * 代答条件：路由有效且序列号有效，并且本地缓存的 dest_seq >= RREQ 中的 dest_seq */
    for (uint8_t i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (g_route_table[i].valid &&
            g_route_table[i].dest_addr == rreq->dest_addr &&
            g_route_table[i].dest_seq_valid &&
            (rreq->dest_seq == 0 || !seq_newer(rreq->dest_seq, g_route_table[i].dest_seq))) {
            /* 本节点拥有足够新鲜的路由，代替目标节点回复 RREP */
            osal_printk("%s RREQ: intermediate RREP for dest=0x%04X seq=%u\r\n",
                        MESH_LOG_TAG, rreq->dest_addr,
                        (unsigned int)g_route_table[i].dest_seq);

            mesh_rrep_payload_t rrep;
            (void)memset_s(&rrep, sizeof(rrep), 0, sizeof(rrep));
            rrep.flags = 0;
            rrep.hop_count = g_route_table[i].hop_count;
            rrep.dest_addr = rreq->dest_addr;
            rrep.dest_seq = g_route_table[i].dest_seq;
            rrep.orig_addr = rreq->orig_addr;
            rrep.lifetime_ms = MESH_AODV_ROUTE_LIFETIME_MS;

            send_rrep(rreq->orig_addr, &rrep);
            return;
        }
    }

    /* 步骤 6：无法代答，转发 RREQ（hop_count 加 1 后重新广播，实现洪泛转发） */
    mesh_rreq_payload_t fwd_rreq;
    (void)memcpy_s(&fwd_rreq, sizeof(fwd_rreq), rreq, sizeof(mesh_rreq_payload_t));
    fwd_rreq.hop_count++;

    osal_printk("%s RREQ FWD: dest=0x%04X orig=0x%04X hops=%d\r\n",
                MESH_LOG_TAG, fwd_rreq.dest_addr, fwd_rreq.orig_addr,
                fwd_rreq.hop_count);

    send_rreq(&fwd_rreq);
}

/* ============================================================
 *  AODV 核心：处理收到的 RREP（路由应答）
 *
 *  完整处理流程：
 *    1. 校验 payload 长度，解析 RREP 结构体
 *    2. 建立/更新到目标节点（dest）的正向路由
 *    3. 若本节点是 RREQ 发起者 → 路由发现成功，发送缓存数据
 *    4. 否则（中间节点）→ hop_count++ 转发 RREP 给 originator
 * ============================================================ */
void mesh_route_process_rrep(uint16_t conn_id, const mesh_header_t *header,
                              const uint8_t *payload, uint16_t payload_len)
{
    if (payload_len < sizeof(mesh_rrep_payload_t)) {
        return;
    }

    const mesh_rrep_payload_t *rrep = (const mesh_rrep_payload_t *)payload;

    osal_printk("%s RREP RX: dest=0x%04X dest_seq=%u orig=0x%04X hops=%d\r\n",
                MESH_LOG_TAG, rrep->dest_addr,
                (unsigned int)rrep->dest_seq, rrep->orig_addr, rrep->hop_count);

    /* 步骤 1：建立/更新到目标节点（dest）的正向路由，下一跳为 RREP 的发送方 */
    uint16_t sender_addr = mesh_transport_get_mesh_addr(conn_id);
    if (sender_addr == MESH_ADDR_UNASSIGNED) {
        sender_addr = header->src_addr;
    }

    route_update_internal(rrep->dest_addr, sender_addr,
                           rrep->hop_count + 1, rrep->dest_seq,
                           true, rrep->lifetime_ms);

    /* 步骤 2：检查本节点是否为 RREQ 的发起者（originator） */
    if (rrep->orig_addr == g_mesh_node_addr) {
        osal_printk("%s RREP: route to 0x%04X established! (hops=%d)\r\n",
                    MESH_LOG_TAG, rrep->dest_addr, rrep->hop_count + 1);

        /* 路由发现成功！从 RREQ 等待队列中移除该目标 */
        rreq_wait_remove(rrep->dest_addr);

        /* 触发转发层 pending queue 中缓存数据的立即发送 */
        mesh_forward_flush_pending(rrep->dest_addr);
        return;
    }

    /* 步骤 3：本节点为中间节点，hop_count++ 后转发 RREP 给 originator 方向 */
    mesh_rrep_payload_t fwd_rrep;
    (void)memcpy_s(&fwd_rrep, sizeof(fwd_rrep), rrep, sizeof(mesh_rrep_payload_t));
    fwd_rrep.hop_count++;

    osal_printk("%s RREP FWD: dest=0x%04X -> orig=0x%04X hops=%d\r\n",
                MESH_LOG_TAG, fwd_rrep.dest_addr, fwd_rrep.orig_addr,
                fwd_rrep.hop_count);

    send_rrep(fwd_rrep.orig_addr, &fwd_rrep);
}

/* ============================================================
 *  AODV 核心：处理收到的 RERR（路由错误）
 *
 *  完整处理流程：
 *    1. 校验 payload 长度，解析 RERR 结构体
 *    2. 遍历 RERR 中列出的不可达目标列表
 *    3. 检查本节点路由表，将受影响的条目标记为无效
 *    4. 将本节点受影响的条目收集后继续广播传播
 * ============================================================ */
void mesh_route_process_rerr(uint16_t conn_id, const mesh_header_t *header,
                              const uint8_t *payload, uint16_t payload_len)
{
    unused(conn_id);
    unused(header);

    if (payload_len < 1) {
        return;
    }

    const mesh_rerr_payload_t *rerr = (const mesh_rerr_payload_t *)payload;
    uint8_t dest_count = rerr->dest_count;
    if (dest_count > MESH_AODV_RERR_MAX_DEST) {
        dest_count = MESH_AODV_RERR_MAX_DEST;
    }

    uint16_t needed_len = 1 + dest_count * sizeof(mesh_rerr_dest_t);
    if (payload_len < needed_len) {
        return;
    }

    osal_printk("%s RERR RX: %d unreachable dests\r\n", MESH_LOG_TAG, dest_count);

    /* 构建转发 RERR，仅包含本节点实际受影响的条目（避免无关的广播扩散） */
    mesh_rerr_payload_t fwd_rerr;
    (void)memset_s(&fwd_rerr, sizeof(fwd_rerr), 0, sizeof(fwd_rerr));
    fwd_rerr.dest_count = 0;

    for (uint8_t d = 0; d < dest_count; d++) {
        uint16_t unreachable_dest = rerr->dests[d].dest_addr;
        uint32_t unreachable_seq = rerr->dests[d].dest_seq;

        /* 在本节点路由表中查找该不可达目标是否存在对应路由 */
        for (uint8_t i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
            if (g_route_table[i].valid &&
                g_route_table[i].dest_addr == unreachable_dest) {

                osal_printk("%s RERR: invalidate route dest=0x%04X\r\n",
                            MESH_LOG_TAG, unreachable_dest);
                g_route_table[i].valid = false;

                /* 该路由受影响，加入转发 RERR 列表（用于后续继续广播通知） */
                if (fwd_rerr.dest_count < MESH_AODV_RERR_MAX_DEST) {
                    fwd_rerr.dests[fwd_rerr.dest_count].dest_addr = unreachable_dest;
                    fwd_rerr.dests[fwd_rerr.dest_count].dest_seq = unreachable_seq;
                    fwd_rerr.dest_count++;
                }
                break;
            }
        }
    }

    /* 仅当本节点确实有路由受影响时，才转发 RERR 继续传播 */
    if (fwd_rerr.dest_count > 0) {
        osal_printk("%s RERR FWD: %d affected dests\r\n",
                    MESH_LOG_TAG, fwd_rerr.dest_count);
        send_rerr(&fwd_rerr);
    }
}

/* ============================================================
 *  AODV 核心：链路断开事件处理 → 生成并广播 RERR
 *
 *  当传输层检测到与某邻居的 SLE 连接断开时调用。
 *  遍历路由表找出所有 next_hop 为该邻居的路由条目，
 *  将它们全部标记为无效，并将序列号加 1 后封装到 RERR 中广播。
 *  序列号加 1 是 AODV 规则，表示该路由已失效，接收方可据此更新。
 * ============================================================ */
void mesh_route_on_link_break(uint16_t broken_next_hop)
{
    if (broken_next_hop == MESH_ADDR_UNASSIGNED) {
        return;
    }

    osal_printk("%s link break: next_hop=0x%04X\r\n", MESH_LOG_TAG, broken_next_hop);

    mesh_rerr_payload_t rerr;
    (void)memset_s(&rerr, sizeof(rerr), 0, sizeof(rerr));
    rerr.dest_count = 0;

    /* 遍历路由表，找出所有经过断开邻居转发的路由条目，收集到 RERR 中 */
    for (uint8_t i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (g_route_table[i].valid && g_route_table[i].next_hop == broken_next_hop) {
            osal_printk("%s link break: invalidate dest=0x%04X\r\n",
                        MESH_LOG_TAG, g_route_table[i].dest_addr);

            if (rerr.dest_count < MESH_AODV_RERR_MAX_DEST) {
                /* AODV 规则：失效路由的序列号加 1，表示路由已过期 */
                uint32_t new_seq = g_route_table[i].dest_seq + 1;
                rerr.dests[rerr.dest_count].dest_addr = g_route_table[i].dest_addr;
                rerr.dests[rerr.dest_count].dest_seq = new_seq;
                rerr.dest_count++;
            }

            g_route_table[i].valid = false;
        }
    }

    /* 若有路由受影响，广播 RERR 通知周围节点 */
    if (rerr.dest_count > 0) {
        osal_printk("%s RERR TX: %d unreachable dests\r\n", MESH_LOG_TAG, rerr.dest_count);
        send_rerr(&rerr);
    }
}

/* ============================================================
 *  AODV 核心：RREQ 超时检查与重试
 *
 *  由主循环周期性调用。遍历 RREQ 等待队列：
 *  - 尚未超时的条目：跳过
 *  - 已超时且重试次数未达上限：重新生成 RREQ ID 并广播
 *  - 已超时且达到最大重试次数：放弃该路由发现请求
 *
 *  MESH_AODV_RREQ_TIMEOUT_MS 控制单次等待时间，
 *  MESH_AODV_RREQ_RETRIES 控制最大重试次数，
 *  均可在 mesh_config.h 中根据网络规模和延迟进行调整。
 * ============================================================ */
void mesh_route_process_rreq_timeouts(void)
{
    uint32_t now = osal_get_tick_ms();

    for (uint8_t i = 0; i < MESH_AODV_RREQ_WAIT_SIZE; i++) {
        if (!g_rreq_wait[i].active) {
            continue;
        }

        if (now < g_rreq_wait[i].next_retry_ms) {
            continue;
        }

        /* 等待时间已到，检查是否超过最大重试次数 */
        if (g_rreq_wait[i].retry_count >= MESH_AODV_RREQ_RETRIES) {
            /* 已达最大重试次数，放弃该路由发现请求 */
            osal_printk("%s RREQ timeout: dest=0x%04X, gave up after %d retries\r\n",
                        MESH_LOG_TAG, g_rreq_wait[i].dest_addr,
                        g_rreq_wait[i].retry_count);
            g_rreq_wait[i].active = false;
            continue;
        }

        /* 重试 RREQ：递增重试计数和 RREQ ID，重新构建并广播 */
        g_rreq_wait[i].retry_count++;
        g_rreq_id_seq++;

        /* 从路由表中查找已知的目标序列号（可能在超时期间从其他路径获得） */
        uint32_t known_seq = 0;
        for (uint8_t j = 0; j < MESH_ROUTE_TABLE_SIZE; j++) {
            if (g_route_table[j].dest_addr == g_rreq_wait[i].dest_addr &&
                g_route_table[j].dest_seq_valid) {
                known_seq = g_route_table[j].dest_seq;
                break;
            }
        }

        mesh_rreq_payload_t rreq;
        (void)memset_s(&rreq, sizeof(rreq), 0, sizeof(rreq));
        rreq.flags = 0;
        rreq.hop_count = 0;
        rreq.rreq_id = g_rreq_id_seq;
        rreq.dest_addr = g_rreq_wait[i].dest_addr;
        rreq.dest_seq = known_seq;
        rreq.orig_addr = g_mesh_node_addr;
        rreq.orig_seq = g_own_seq;

        /* 将新 RREQ ID 记录到去重缓存，防止转发回来时重复处理 */
        rreq_cache_check_and_add(g_mesh_node_addr, g_rreq_id_seq);

        g_rreq_wait[i].rreq_id = g_rreq_id_seq;
        g_rreq_wait[i].next_retry_ms = now + MESH_AODV_RREQ_TIMEOUT_MS;

        osal_printk("%s RREQ retry #%d: dest=0x%04X rreq_id=%d\r\n",
                    MESH_LOG_TAG, g_rreq_wait[i].retry_count,
                    g_rreq_wait[i].dest_addr, g_rreq_id_seq);

        send_rreq(&rreq);
    }
}

/* ============================================================
 *  公开接口：获取所有可达目标节点的地址和跳数
 *
 *  遍历路由表，将所有有效且未过期的目标地址及其跳数填入输出数组。
 *  供 BLE 网关拓扑查询、调试信息输出等上层功能使用。
 * ============================================================ */
uint8_t mesh_route_get_all_destinations(uint16_t *addrs, uint8_t *hops, uint8_t max_count)
{
    uint8_t count = 0;
    uint32_t now = osal_get_tick_ms();
    if (addrs == NULL || hops == NULL) {
        return 0;
    }
    for (uint8_t i = 0; i < MESH_ROUTE_TABLE_SIZE && count < max_count; i++) {
        if (g_route_table[i].valid && g_route_table[i].expire_ms > now) {
            addrs[count] = g_route_table[i].dest_addr;
            hops[count] = g_route_table[i].hop_count;
            count++;
        }
    }
    return count;
}