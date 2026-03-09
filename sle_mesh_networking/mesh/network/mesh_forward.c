/**
 * @file  mesh_forward.c
 * @brief SLE Mesh 网络转发层实现（P1 + P2 阶段）
 *
 * @details 本文件实现了 SLE Mesh 转发层的全部内部逻辑，包含以下核心功能：
 *   - CRC16-CCITT 校验：多项式 0x1021，初始值 0xFFFF，用于帧数据完整性保护
 *   - 环形去重缓存（默认 64 条目）：基于 (src_addr << 16 | seq_num) 组合判重，
 *     采用环形缓冲区策略，无需显式清理，新条目自动覆盖最旧条目
 *   - 广播洪泛 + 流量控制：滑动窗口计数器（默认 20 帧/1000ms），防止广播风暴
 *   - 单播转发策略：直连邻居 → 路由表查询 → RREQ 路由发现回退（P2 阶段）
 *   - HELLO 报文处理：解析邻居列表，更新 conn_id → mesh_addr 映射，
 *     学习两跳路由（邻居的邻居可经该邻居转发到达）
 *   - 待发送队列：缓存等待路由发现的数据包，带 TTL 超时机制
 *   - AODV 路由控制报文分发：RREQ/RREP/RERR 转交 mesh_route.c 处理（P2 阶段）
 *   - 反向路由学习：从转发帧中提取源地址和下一跳信息，动态更新路由表
 *   - 统计信息采集与打印，用于运行时性能监控和问题诊断
 *
 * @note 关键配置参数均定义在 mesh_config.h 中，可根据实际场景调整：
 *       - MESH_DEDUP_CACHE_SIZE：去重缓存大小，默认 64，网络规模大时建议增大
 *       - MESH_FLOOD_WINDOW_MS / MESH_FLOOD_RATE_LIMIT：洪泛窗口和速率限制
 *       - MESH_PENDING_QUEUE_SIZE / MESH_PENDING_TTL_MS：待发送队列容量和超时
 *       - MESH_DEFAULT_TTL：默认 TTL 跳数，决定帧的最大传播范围
 *       - MESH_HELLO_MAX_NEIGHBORS：HELLO 报文携带的最大邻居数
 *
 * Copyright (c) 2026. All rights reserved.
 *
 * History:
 * 2026-02-07, 创建文件（P0 基础版本）。
 * 2026-02-07, P1 阶段：新增洪泛速率限制、HELLO 处理、待发送队列、统计信息。
 * 2026-02-08, P2 阶段：集成 AODV RREQ/RREP/RERR 分发，无路由时发起 RREQ 路由发现。
 */

#include "common_def.h"
#include "securec.h"
#include "soc_osal.h"
#include "osal_debug.h"

#include "mesh_types.h"
#include "mesh_config.h"
#include "mesh_transport.h"
#include "mesh_route.h"
#include "mesh_forward.h"

/* ============================================================
 *  去重缓存（环形缓冲区）
 *
 *  @note 采用环形缓冲区（Ring Buffer）策略存储已处理帧的唯一标识，
 *        用于检测并丢弃重复帧。每个条目由 (src_addr << 16 | seq_num)
 *        组成 32 位唯一标识。当缓存满时，新条目自动覆盖最旧条目，
 *        无需显式清理。缓存大小由 MESH_DEDUP_CACHE_SIZE 宏控制（默认 64），
 *        网络规模大或帧频率高时建议适当增大以降低误判率。
 * ============================================================ */

/**
 * @brief  去重缓存条目结构体
 */
typedef struct {
    uint32_t msg_id;    /**< 帧唯一标识：高 16 位 = src_addr，低 16 位 = seq_num */
} mesh_dedup_entry_t;

/** @brief 去重缓存数组，容量由 MESH_DEDUP_CACHE_SIZE 宏定义 */
static mesh_dedup_entry_t g_dedup_cache[MESH_DEDUP_CACHE_SIZE];
/** @brief 环形缓冲区写入头指针，指向下一个将被覆盖的位置 */
static uint8_t g_dedup_head = 0;
/** @brief 环形缓冲区当前有效条目数（最大不超过 MESH_DEDUP_CACHE_SIZE） */
static uint8_t g_dedup_count = 0;

/* ============================================================
 *  本节点序列号
 *  @note 每发出一个新帧时自动递增，与源地址组合构成全网唯一标识，
 *        用于接收端去重。重启后从 0 开始，不影响去重正确性
 *        （因为去重缓存也会被清零）。
 * ============================================================ */
static uint16_t g_local_seq_num = 0;

/* ============================================================
 *  应用层接收回调指针
 *  @note 当收到目标为本节点的单播帧或广播帧时，通过此回调
 *        将载荷数据递交给应用层。通过 mesh_forward_register_app_rx_callback()
 *        注册，传入 NULL 可取消注册。
 * ============================================================ */
static mesh_rx_callback_t g_app_rx_callback = NULL;

/* ============================================================
 *  洪泛流量控制 - 滑动窗口计数器
 *
 *  @note 采用固定时间窗口的滑动计数器策略：
 *        - 窗口时长由 MESH_FLOOD_WINDOW_MS 宏控制（默认 1000ms）
 *        - 窗口内最大允许转发帧数由 MESH_FLOOD_RATE_LIMIT 宏控制（默认 20）
 *        - 超过限制的帧将被丢弃，计入 rx_dropped_rate 统计
 *        实际部署时可根据网络密度和带宽调整这两个参数：
 *        网络密度高时可降低 MESH_FLOOD_RATE_LIMIT，
 *        带宽充裕时可增大 MESH_FLOOD_RATE_LIMIT 提高吸收率。
 * ============================================================ */

/** @brief 当前流控窗口的起始时间戳（ms，由 osal_get_tick_ms() 获取） */
static uint32_t g_flood_window_start_ms = 0;
/** @brief 当前窗口内已转发的帧计数 */
static uint16_t g_flood_count_in_window = 0;

/* ============================================================
 *  待发送队列（等待路由发现的数据包缓存）
 *
 *  @note 当单播发送时无可用路由，原始应用数据被缓存到此队列，
 *        同时发起 RREQ 路由发现。队列大小由 MESH_PENDING_QUEUE_SIZE 宏控制。
 *        每个条目记录目标地址、数据内容和入队时间，
 *        在主循环中由 mesh_forward_process_pending_queue() 周期检查：
 *        - 路由已发现 → 构建帧并发送
 *        - 超时（超过 MESH_PENDING_TTL_MS）→ 丢弃
 *        另外，RREP 到达时也会通过 mesh_forward_flush_pending() 立即触发发送。
 * ============================================================ */

/**
 * @brief  待发送队列条目结构体
 */
typedef struct {
    uint8_t  used;              /**< 条目是否占用：1 = 占用，0 = 空闲 */
    uint16_t dst_addr;          /**< 目标节点的 Mesh 地址 */
    uint16_t data_len;          /**< 缓存的应用层数据长度（字节） */
    uint32_t enqueue_ms;        /**< 入队时间戳（ms），用于超时判断 */
    uint8_t  data[MESH_MAX_PAYLOAD]; /**< 缓存的原始应用层数据 */
} mesh_pending_entry_t;

/** @brief 待发送队列数组，大小由 MESH_PENDING_QUEUE_SIZE 宏定义 */
static mesh_pending_entry_t g_pending_queue[MESH_PENDING_QUEUE_SIZE];

/* ============================================================
 *  转发层统计计数器
 *  @note 统计各类帧的收发数量、丢弃原因分布、待发送队列状态等，
 *        在 mesh_forward_init() 和 mesh_forward_reset_stats() 时清零。
 *        可通过 mesh_forward_print_stats() 在调试串口输出。
 * ============================================================ */
static mesh_forward_stats_t g_stats = { 0 };

/* ============================================================
 *  CRC16-CCITT 校验算法
 *
 *  @note 算法参数：
 *        - 多项式：0x1021（CRC-CCITT 标准）
 *        - 初始值：0xFFFF
 *        - 处理方式：逐字节，每字节左移 8 位与 CRC 异或，然后逐位处理
 *        - 最高位为 1 时左移并异或多项式，否则仅左移
 *        - 无最终异或操作
 *        此算法广泛用于嵌入式通信协议，能有效检测数据传输中的位错误。
 * ============================================================ */

/**
 * @brief  计算 CRC16-CCITT 校验值
 * @param[in] data  待计算数据的缓冲区指针
 * @param[in] len   数据长度（字节）
 * @return 16 位 CRC 校验值
 */
uint16_t mesh_forward_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;  /* CRC 初始值为全 1 */
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;  /* 将当前字节左移到高 8 位，与 CRC 异或 */
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                /* 最高位为 1：左移一位后与多项式 0x1021 异或 */
                crc = (crc << 1) ^ 0x1021;
            } else {
                /* 最高位为 0：仅左移一位 */
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* ============================================================
 *  去重检查：基于环形缓冲区的帧重复检测
 *
 *  @note 算法原理：
 *        1. 将 (src_addr, seq_num) 组合为 32 位 msg_id 作为帧唯一标识
 *        2. 从环形缓冲区最新位置向前遍历查找是否存在相同 msg_id
 *        3. 若找到则为重复帧（返回 true），不再插入
 *        4. 若未找到则为新帧，将 msg_id 插入头指针位置，头指针前移
 *        5. 缓存满后自动覆盖最旧条目（环形覆盖策略）
 *
 *  @note 性能考量：线性遍历 O(N)，但 N 较小（默认 64），
 *        对于嵌入式场景已足够高效。若网络规模极大，
 *        可考虑改用哈希表实现。
 * ============================================================ */

/**
 * @brief  检查帧是否重复，若不重复则加入去重缓存
 * @param[in] src_addr  帧的源地址
 * @param[in] seq_num   帧的序列号
 * @return true  = 重复帧（已在缓存中找到，应丢弃）
 * @return false = 新帧（已加入缓存，应继续处理）
 */
static bool dedup_check_and_add(uint16_t src_addr, uint16_t seq_num)
{
    /* 组合源地址和序列号为 32 位唯一标识 */
    uint32_t msg_id = ((uint32_t)src_addr << 16) | seq_num;

    /* 从最新条目向最旧条目遍历，查找是否存在相同 msg_id */
    for (uint8_t i = 0; i < g_dedup_count && i < MESH_DEDUP_CACHE_SIZE; i++) {
        uint8_t idx = (g_dedup_head + MESH_DEDUP_CACHE_SIZE - 1 - i) % MESH_DEDUP_CACHE_SIZE;
        if (g_dedup_cache[idx].msg_id == msg_id) {
            return true;  /* 找到重复，返回 true 指示丢弃 */
        }
    }

    /* 未找到重复，将新条目插入环形缓冲区头指针位置 */
    g_dedup_cache[g_dedup_head].msg_id = msg_id;
    g_dedup_head = (g_dedup_head + 1) % MESH_DEDUP_CACHE_SIZE;  /* 头指针前移，环形回绕 */
    if (g_dedup_count < MESH_DEDUP_CACHE_SIZE) {
        g_dedup_count++;  /* 未满时递增有效计数 */
    }
    return false;  /* 新帧，返回 false 指示继续处理 */
}

/* ============================================================
 *  洪泛速率限制 - 滑动窗口流量控制
 *
 *  @note 算法原理：
 *        1. 判断当前时间是否超出当前窗口（MESH_FLOOD_WINDOW_MS）
 *        2. 超出则重置窗口起始时间和计数器
 *        3. 判断当前窗口内计数器是否达到上限（MESH_FLOOD_RATE_LIMIT）
 *        4. 达到上限则拒绝转发（返回 false），否则允许并递增计数
 * ============================================================ */

/**
 * @brief  检查洪泛速率是否允许继续转发
 * @return true  = 允许转发（未超过速率限制）
 * @return false = 拒绝转发（已达到当前窗口的最大转发数）
 */
static bool flood_rate_check(void)
{
    uint32_t now = osal_get_tick_ms();

    /* 检查是否进入新窗口：当前时间超出窗口时长则重置 */
    if (now - g_flood_window_start_ms >= MESH_FLOOD_WINDOW_MS) {
        g_flood_window_start_ms = now;      /* 重置窗口起始时间 */
        g_flood_count_in_window = 0;        /* 重置窗口内计数器 */
    }

    if (g_flood_count_in_window >= MESH_FLOOD_RATE_LIMIT) {
        return false; /* 已达速率上限，拒绝转发 */
    }

    g_flood_count_in_window++;  /* 允许转发，计数器 +1 */
    return true;
}

/* ============================================================
 *  帧封装辅助：计算 CRC16 并填写到帧头 checksum 字段
 *  @note 计算前先将 checksum 字段置 0，确保 CRC 计算覆盖整个帧内容
 *        （包含帧头 + 载荷），保证校验一致性。
 * ============================================================ */

/**
 * @brief  计算帧的 CRC16 并填入 header.checksum
 * @param[in,out] frame      帧结构体指针，checksum 字段将被更新
 * @param[in]     total_len  帧总长度（MESH_HEADER_SIZE + payload_len）
 */
static void frame_calc_and_set_crc(mesh_frame_t *frame, uint16_t total_len)
{
    frame->header.checksum = 0;  /* 先清零 checksum 字段，避免旧值影响 CRC 计算 */
    frame->header.checksum = mesh_forward_crc16((const uint8_t *)frame, total_len);
}

/* ============================================================
 *  帧转发辅助：复制帧数据并调整转发字段
 *  @note 转发前将 TTL 减 1、hop_count 加 1，然后重新计算 CRC。
 *        这确保每一次转发都会更新帧的转发状态，
 *        接收方可通过 hop_count 区分直连帧和转发帧。
 * ============================================================ */

/**
 * @brief  准备转发帧：复制原始数据、TTL-1、hop_count+1、重算 CRC
 * @param[out] fwd        输出的转发帧结构体
 * @param[in]  data       原始接收到的帧数据
 * @param[in]  total_len  原始帧的总长度（字节）
 * @return 成功返回帧总长度；输入长度超过 mesh_frame_t 大小时返回 0
 */
static uint16_t frame_prepare_forward(mesh_frame_t *fwd, const uint8_t *data, uint16_t total_len)
{
    if (total_len > sizeof(mesh_frame_t)) {
        return 0;  /* 超出帧结构体容量，无法复制 */
    }
    (void)memcpy_s(fwd, sizeof(mesh_frame_t), data, total_len);
    fwd->header.ttl--;          /* TTL 递减：每经过一跳减 1 */
    fwd->header.hop_count++;    /* 跳数计数递增：记录已经过的跳数 */
    frame_calc_and_set_crc(fwd, total_len);  /* 重新计算 CRC（因为 TTL 和 hop_count 已变更） */
    return total_len;
}

/* ============================================================
 *  初始化 / 反初始化 / 回调注册
 * ============================================================ */

/**
 * @brief  初始化转发层：清零所有内部状态
 * @details 依次清零去重缓存、本地序列号、应用层回调、流控窗口、
 *          待发送队列和统计计数器，以确保转发层从干净状态启动。
 * @return ERRCODE_SUCC 成功
 */
errcode_t mesh_forward_init(void)
{
    (void)memset_s(g_dedup_cache, sizeof(g_dedup_cache), 0, sizeof(g_dedup_cache));
    g_dedup_head = 0;
    g_dedup_count = 0;
    g_local_seq_num = 0;
    g_app_rx_callback = NULL;

    g_flood_window_start_ms = 0;
    g_flood_count_in_window = 0;

    (void)memset_s(g_pending_queue, sizeof(g_pending_queue), 0, sizeof(g_pending_queue));
    (void)memset_s(&g_stats, sizeof(g_stats), 0, sizeof(g_stats));

    osal_printk("%s forward init ok (P2)\r\n", MESH_LOG_TAG);
    return ERRCODE_SUCC;
}

/**
 * @brief  反初始化转发层：清除应用层回调指针
 */
void mesh_forward_deinit(void)
{
    g_app_rx_callback = NULL;
}

/**
 * @brief  注册应用层数据接收回调
 * @param[in] cb  回调函数指针，传 NULL 可取消注册
 */
void mesh_forward_register_app_rx_callback(mesh_rx_callback_t cb)
{
    g_app_rx_callback = cb;
}

/* ============================================================
 *  构建 Mesh 帧
 *  @note 填充帧头各字段（魔数、版本、消息类型、TTL、源/目标地址、
 *        序列号、跳数），复制载荷数据，最后计算并填入 CRC16 校验值。
 *        源地址自动使用本节点地址 g_mesh_node_addr，
 *        序列号 g_local_seq_num 在每次调用时自动递增。
 * ============================================================ */

/**
 * @brief  构建完整的 Mesh 帧（帧头 + 载荷 + CRC）
 * @param[out] frame        输出帧结构体指针，不可为 NULL
 * @param[in]  msg_type     消息类型（MESH_MSG_UNICAST / MESH_MSG_BROADCAST 等）
 * @param[in]  dst_addr     目标地址；广播时使用 MESH_ADDR_BROADCAST
 * @param[in]  payload      载荷数据指针，可为 NULL（当 payload_len == 0）
 * @param[in]  payload_len  载荷长度，不超过 MESH_MAX_PAYLOAD
 * @return 帧总长度（字节）；参数非法返回 0
 */
uint16_t mesh_forward_build_frame(mesh_frame_t *frame,
                                   uint8_t msg_type,
                                   uint16_t dst_addr,
                                   const uint8_t *payload,
                                   uint16_t payload_len)
{
    if (frame == NULL || payload_len > MESH_MAX_PAYLOAD) {
        return 0;
    }

    frame->header.magic = MESH_FRAME_MAGIC;
    frame->header.version = MESH_FRAME_VERSION;
    frame->header.msg_type = msg_type;
    frame->header.ttl = MESH_DEFAULT_TTL;
    frame->header.src_addr = g_mesh_node_addr;
    frame->header.dst_addr = dst_addr;
    frame->header.seq_num = g_local_seq_num++;
    frame->header.payload_len = payload_len;
    frame->header.hop_count = 0;
    frame->header.checksum = 0;

    if (payload != NULL && payload_len > 0) {
        (void)memcpy_s(frame->payload, MESH_MAX_PAYLOAD, payload, payload_len);
    }

    uint16_t total_len = MESH_HEADER_SIZE + payload_len;
    frame_calc_and_set_crc(frame, total_len);
    return total_len;
}

/* ============================================================
 *  帧头合法性验证
 *  @note 依次检查：魔数字段、版本号、数据最小长度、载荷长度合理性、
 *        实际数据长度是否足够包含声明的载荷。任一检查失败则返回 false。
 * ============================================================ */

/**
 * @brief  验证帧头合法性
 * @param[in] header    帧头指针
 * @param[in] data_len  接收到的原始数据总长度（字节）
 * @return true = 帧头合法，false = 帧头异常（应丢弃）
 */
static bool validate_frame(const mesh_header_t *header, uint16_t data_len)
{
    if (header->magic != MESH_FRAME_MAGIC) {
        return false;   /* 魔数不匹配，非 Mesh 帧 */
    }
    if (header->version != MESH_FRAME_VERSION) {
        return false;   /* 版本号不兼容 */
    }
    if (data_len < MESH_HEADER_SIZE) {
        return false;   /* 数据长度不足一个完整帧头 */
    }
    if (header->payload_len > MESH_MAX_PAYLOAD) {
        return false;   /* 声明的载荷长度超出允许范围 */
    }
    if (data_len < MESH_HEADER_SIZE + header->payload_len) {
        return false;   /* 实际数据不足以包含声明的载荷 */
    }
    return true;
}

/* ============================================================
 *  CRC 校验：将接收到的帧复制到临时缓冲区，清零 checksum 字段后
 *  重新计算 CRC16，与原始帧中保存的 checksum 比对。
 *  @note 必须使用临时副本，因为需要将 checksum 字段置 0 再计算，
 *        不能修改原始数据。
 * ============================================================ */

/**
 * @brief  校验帧的 CRC16 完整性
 * @param[in] data       原始帧数据指针
 * @param[in] total_len  帧总长度（字节）
 * @param[in] saved_crc  帧头中保存的 CRC16 值
 * @return true = CRC 校验通过，数据完整；false = 数据损坏
 */
static bool verify_crc(const uint8_t *data, uint16_t total_len, uint16_t saved_crc)
{
    mesh_frame_t tmp;
    if (total_len > sizeof(mesh_frame_t)) {
        return false;
    }
    (void)memcpy_s(&tmp, sizeof(tmp), data, total_len);  /* 复制到临时缓冲区 */
    tmp.header.checksum = 0;  /* 清零 checksum 字段后再计算 CRC */
    uint16_t calc_crc = mesh_forward_crc16((const uint8_t *)&tmp, total_len);
    return (calc_crc == saved_crc);  /* 比对计算值与原始值 */
}

/* ============================================================
 *  HELLO 报文处理：邻居发现 + 两跳路由学习
 *
 *  @note HELLO 报文由邻居节点周期发送，TTL=1 仅传播一跳。
 *        载荷格式（小端序）：
 *          [0-1]  发送者的 mesh_addr（2 字节）
 *          [2]    发送者的邻居数量 N（1 字节）
 *          [3..2+2N]  各邻居的 mesh_addr（每个 2 字节）
 *
 *        处理流程：
 *        1. 解析发送者的 mesh_addr，更新 conn_id → mesh_addr 映射
 *        2. 验证 conn_id 的已锁定地址是否匹配（防止 SLE conn_id 复用导致的污染）
 *        3. 解析邻居列表，对每个非本节点、非直连的邻居地址，
 *           学习两跳路由：该地址可经发送者地址作为下一跳到达（跳数 = 2）
 *
 *        两跳路由学习的意义：
 *        假设节点 A 收到节点 B 的 HELLO，其中邻居列表包含节点 C，
 *        则 A 学到：C 可经 B 到达（A → B → C），无需等待 RREQ。
 * ============================================================ */

/**
 * @brief  处理接收到的 HELLO 报文
 * @param[in] conn_id      接收此 HELLO 的 SLE 连接标识符
 * @param[in] payload      HELLO 载荷数据指针
 * @param[in] payload_len  HELLO 载荷长度（字节），最小为 3
 */
static void process_hello(uint16_t conn_id, const uint8_t *payload, uint16_t payload_len)
{
    g_stats.rx_hello++;

    if (payload_len < 3) {
        return;
    }

    /* 解析发送者的 mesh_addr（小端序：低字节在前） */
    uint16_t sender_addr = (uint16_t)(payload[0] | (payload[1] << 8));

    /* 更新 transport 层中该 conn_id 对应的 mesh_addr 映射关系 */
    mesh_transport_update_mesh_addr(conn_id, sender_addr);

    /* P1-fix: 验证 conn_id 的已锁定地址是否与 HELLO 中声明的地址一致。
     * SLE 协议栈可能将 server/client 双通道复用同一 conn_id，
     * 导致同一 conn_id 上收到不同节点的 HELLO。
     * 若地址不匹配，丢弃此 HELLO 避免路由表污染。 */
    uint16_t locked_addr = mesh_transport_get_mesh_addr(conn_id);
    if (locked_addr != MESH_ADDR_UNASSIGNED && locked_addr != sender_addr) {
        return;  /* 身份不匹配：该 conn_id 已绑定其他地址，丢弃此 HELLO */
    }

    /* 解析邻居数量，截断为 MESH_HELLO_MAX_NEIGHBORS 以防越界 */
    uint8_t neighbor_count = payload[2];
    if (neighbor_count > MESH_HELLO_MAX_NEIGHBORS) {
        neighbor_count = MESH_HELLO_MAX_NEIGHBORS;
    }

    /* 遍历邻居列表，学习两跳路由：发送者的邻居可经发送者作为下一跳到达 */
    uint16_t idx = 3;
    for (uint8_t i = 0; i < neighbor_count && (idx + 1) < payload_len; i++) {
        uint16_t nbr_addr = (uint16_t)(payload[idx] | (payload[idx + 1] << 8));
        idx += 2;   /* 每个邻居地址占 2 字节 */

        /* 跳过本节点和无效地址：不需要为自己添加路由 */
        if (nbr_addr == g_mesh_node_addr || nbr_addr == MESH_ADDR_UNASSIGNED) {
            continue;
        }

        /* 已是直连邻居时不需要多跳路由：直连路径始终优先 */
        if (mesh_transport_is_direct_neighbor(nbr_addr)) {
            continue;
        }

        /* 学习两跳路由：nbr_addr 可经 sender_addr 到达，跳数 = 2。
         * 仅在路由表中不存在该目的地址的条目时才添加，
         * 避免覆盖可能存在的更优路由。 */
        uint16_t existing = mesh_route_lookup(nbr_addr);
        if (existing == MESH_ADDR_UNASSIGNED) {
            mesh_route_update(nbr_addr, sender_addr, 2, 0);
            osal_printk("%s HELLO learned 2-hop route: 0x%04X via 0x%04X\r\n",
                        MESH_LOG_TAG, nbr_addr, sender_addr);
        }
    }
}

/* ============================================================
 *  单播转发核心逻辑（内部函数，处理接收到的非本节点单播帧）
 *
 *  @note 转发策略（P2 阶段改进）：
 *    1. 目标是直连邻居 → 直接发送（优先于路由表，避免过时路由覆盖邻居关系）
 *    2. 查路由表 → 经 next_hop 定向转发，并刷新路由有效期
 *    3. 无路由 → 受流控限制的洪泛转发（因为这是转发别人的帧，
 *       无法代替源节点发起 RREQ，只能尽力洪泛）
 * ============================================================ */

/**
 * @brief  单播帧转发处理
 * @param[in] data       原始接收到的帧数据
 * @param[in] total_len  帧总长度（字节）
 * @param[in] conn_id    接收此帧的 SLE 连接 ID（洪泛时排除该连接）
 */
static void forward_unicast(const uint8_t *data, uint16_t total_len, uint16_t conn_id)
{
    mesh_frame_t fwd;
    if (frame_prepare_forward(&fwd, data, total_len) == 0) {
        return;
    }

    if (fwd.header.ttl == 0) {
        return;
    }

    uint16_t dst_addr = fwd.header.dst_addr;

    /* 策略 1: 目标是直连邻居 → 直接发送（优先于路由表查询） */
    uint16_t next_hop = MESH_ADDR_UNASSIGNED;
    if (mesh_transport_is_direct_neighbor(dst_addr)) {
        next_hop = dst_addr;    /* 直连邻居，直接发送 */
    }

    /* 策略 2: 非直连邻居，查询路由表获取下一跳 */
    if (next_hop == MESH_ADDR_UNASSIGNED) {
        next_hop = mesh_route_lookup(dst_addr);
    }

    if (next_hop != MESH_ADDR_UNASSIGNED) {
        /* 定向转发：将帧发送给 next_hop，并刷新路由表中该目标的有效期 */
        errcode_t ret = mesh_transport_send_to_neighbor(next_hop,
                                                         (const uint8_t *)&fwd, total_len);
        if (ret == ERRCODE_SUCC) {
            g_stats.rx_forwarded++;
            mesh_route_touch(dst_addr);
            if (!g_mesh_log_suppress) {
                osal_printk("%s fwd unicast: dst=0x%04X via next_hop=0x%04X\r\n",
                            MESH_LOG_TAG, dst_addr, next_hop);
            }
        }
        return;
    }

    /* 策略 3: 无路由可用 → 洪泛转发（受流控限制）
     * 注意：这里是转发别人的帧，不是本节点发起的。
     * 我们无法代替源节点发起 RREQ，只能在流控允许的范围内尽力洪泛。
     * 这避免了无流控洪泛带来的广播风暴问题。 */
    if (!flood_rate_check()) {
        g_stats.rx_dropped_rate++;
        osal_printk("%s fwd unicast: rate limited, dropping dst=0x%04X\r\n",
                    MESH_LOG_TAG, dst_addr);
        return;
    }

    mesh_transport_send_to_all_neighbors((const uint8_t *)&fwd, total_len, conn_id);
    g_stats.rx_forwarded++;
}

/* ============================================================
 *  广播转发核心逻辑（内部函数，处理接收到的广播帧拥续转发）
 *  @note TTL 递减、hop_count 递增后，受洪泛速率限制约束，
 *        洪泛至所有邻居（排除接收来源 conn_id）。
 * ============================================================ */

/**
 * @brief  广播帧转发处理
 * @param[in] data       原始接收到的帧数据
 * @param[in] total_len  帧总长度（字节）
 * @param[in] conn_id    接收此帧的 SLE 连接 ID（洪泛时排除该连接以避免回弹）
 */
static void forward_broadcast(const uint8_t *data, uint16_t total_len, uint16_t conn_id)
{
    mesh_frame_t fwd;
    if (frame_prepare_forward(&fwd, data, total_len) == 0) {
        return;
    }

    if (fwd.header.ttl == 0) {
        return;
    }

    /* 洪泛速率检查：超过窗口配额则丢弃，防止广播风暴 */
    if (!flood_rate_check()) {
        g_stats.rx_dropped_rate++;
        return;
    }

    mesh_transport_send_to_all_neighbors((const uint8_t *)&fwd, total_len, conn_id);
    g_stats.rx_forwarded++;
}

/* ============================================================
 *  核心数据接收处理入口
 *
 *  @note 转发层处理所有入站数据的统一入口，由 mesh_transport 层回调。
 *        内部处理流程严格按顺序执行：
 *        1. 帧头合法性验证（魔数、版本、长度）
 *        2. CRC16 完整性校验
 *        3. 自环检测（丢弃自己发出后被其他节点转回的帧）
 *        4. 环形去重缓存检查（基于 src_addr + seq_num）
 *        5. TTL 存活检查
 *        6. conn_id 身份守卫（对直连帧验证源地址与 conn_id 的绑定关系）
 *        7. 反向路由学习（从转发帧和直连单播帧中提取路由信息）
 *        8. 按消息类型分发：
 *           - MESH_MSG_HELLO:    转 HELLO 处理流程
 *           - MESH_MSG_BROADCAST: 交应用层 + 洪泛转发
 *           - MESH_MSG_UNICAST:  目标为本节点则交应用层，否则转发
 *           - MESH_MSG_RREQ/RREP/RERR: 转交 mesh_route 处理（P2）
 * ============================================================ */

/**
 * @brief  数据接收处理入口（由 mesh_transport 层回调）
 * @param[in] conn_id  SLE 连接标识符
 * @param[in] data     原始数据缓冲区（帧头 + 载荷），不可为 NULL
 * @param[in] len      数据长度（字节），必须 >= MESH_HEADER_SIZE
 */
void mesh_forward_on_data_received(uint16_t conn_id, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len < MESH_HEADER_SIZE) {
        return;
    }

    const mesh_header_t *header = (const mesh_header_t *)data;

    /* 步骤 1: 验证帧头合法性（魔数、版本、长度） */
    if (!validate_frame(header, len)) {
        return;
    }

    uint16_t total_len = MESH_HEADER_SIZE + header->payload_len;
    g_stats.rx_total++;

    /* 步骤 2: CRC16 完整性校验，检测数据传输中是否发生损坏 */
    if (!verify_crc(data, total_len, header->checksum)) {
        g_stats.rx_dropped_crc++;
        osal_printk("%s fwd: CRC fail from 0x%04X seq=%d\r\n",
                    MESH_LOG_TAG, header->src_addr, header->seq_num);
        return;
    }

    /* 步骤 3: 自环检测：丢弃本节点发出后被其他节点转回的帧 */
    if (header->src_addr == g_mesh_node_addr) {
        return;
    }

    /* 步骤 4: 环形去重缓存检查：通过 (src_addr, seq_num) 组合判重 */
    if (dedup_check_and_add(header->src_addr, header->seq_num)) {
        g_stats.rx_dropped_dup++;
        return;
    }

    /* 步骤 5: TTL 存活检查：TTL 为 0 表示帧已经达到最大传播距离 */
    if (header->ttl == 0) {
        g_stats.rx_dropped_ttl++;
        return;
    }

    const uint8_t *payload = data + MESH_HEADER_SIZE;
    uint16_t payload_len = header->payload_len;

    if (!g_mesh_log_suppress) {
        osal_printk("%s fwd: type=0x%02X src=0x%04X dst=0x%04X seq=%d ttl=%d hop=%d\r\n",
                    MESH_LOG_TAG, header->msg_type, header->src_addr,
                    header->dst_addr, header->seq_num, header->ttl, header->hop_count);
    }

    /* P1-fix: conn_id 身份守卫机制。
     * SLE 协议栈的 server/client 双通道可能复用同一 conn_id，
     * 导致来自不同节点的帧以相同 conn_id 到达。
     * 对于直连帧（hop_count == 0），源地址必须与该 conn_id 的锁定地址一致，
     * 否则丢弃以防止路由表污染。 */
    uint16_t sender_mesh_addr = mesh_transport_get_mesh_addr(conn_id);

    /* F17 补丁: Server 端连接的 mesh_addr 快速填充。
     * P4 补丁让 Server 端的 mesh_addr 初始为 UNASSIGNED，等待 HELLO 到达。
     * 但 HELLO 周期 5 秒，在此窗口内拓扑查询、反向路由学习等均无法工作。
     * 优化: 对于直连帧（hop_count == 0），src_addr 就是直连邻居的真实地址，
     * 可安全地立即填充 mesh_addr，不必等待 HELLO。
     * 此操作受 mesh_transport_update_mesh_addr() 的 P1-fix 锁定机制保护，
     * 一旦设定不会被后续帧覆盖。 */
    if (sender_mesh_addr == MESH_ADDR_UNASSIGNED && header->hop_count == 0) {
        mesh_transport_update_mesh_addr(conn_id, header->src_addr);
        sender_mesh_addr = header->src_addr;
        osal_printk("%s F17: fast addr fill conn_id=%d -> 0x%04X\r\n",
                    MESH_LOG_TAG, conn_id, header->src_addr);
    }

    /* P20: 原 P1-fix 在 hop_count==0 且 src_addr != sender_mesh_addr 时直接丢弃。
     * 但 SLE 协议栈存在 conn_id 路由异常（如 exchange_info 失败后数据被送到
     * 错误的 conn_id），会导致合法帧被误杀，引发 topo 查询 0 回复等问题。
     * 改进: 不再 hard-drop，而是标记 identity_mismatch，
     * 跳过反向路由学习（保留 P1 路由保护），但仍允许帧进入消息分发。
     * HELLO 的 process_hello() 已有独立的身份校验，不受影响。 */
    bool identity_mismatch = false;
    if (header->hop_count == 0 &&
        sender_mesh_addr != MESH_ADDR_UNASSIGNED &&
        header->src_addr != sender_mesh_addr) {
        identity_mismatch = true;
        osal_printk("%s P20: conn_id=%d identity mismatch: locked=0x%04X src=0x%04X (allow)\r\n",
                    MESH_LOG_TAG, conn_id, sender_mesh_addr, header->src_addr);
    }

    /* ---- 反向路由学习：从转发帧中提取路由信息 ----
     * 如果帧的源地址不是直连邻居（即该帧经过了多跳转发），
     * 则可学习到：src_addr 可经 sender_mesh_addr 作为下一跳到达。
     * 路由跳数 = hop_count + 1（因为从本节点到 sender 还需一跳）。
     * P20: 身份不匹配时跳过，避免用错误的 next_hop 污染路由表。
     * P23: 使用被动学习 (dest_seq=0)，不再以帧序列号作为路由序列号。
     *   帧的 seq_num 是单调递增的消息序列号 (非 AODV 路由序列号)，
     *   用它作为路由 seq 会导致图片传输期间每个数据帧都覆盖路由表，
     *   锁住 HELLO 学习路由的刷新能力，造成跳数在传输过程中反复振荡。 */
    if (!identity_mismatch &&
        sender_mesh_addr != MESH_ADDR_UNASSIGNED &&
        header->src_addr != sender_mesh_addr) {
        /* src 不是直连邻居，学习反向路由：src_addr 可经 sender_mesh_addr 到达 */
        mesh_route_update(header->src_addr, sender_mesh_addr,
                          header->hop_count + 1, 0);  /* P23: 被动学习 */
    }

    /* 直连邻居的单播帧：更新/添加 1 跳路由。
     * 当直连邻居发来单播帧时（hop_count == 0 且 src == sender），
     * 主动添加/刷新 1 跳路由，替换可能存在的过时多跳条目。 */
    if (header->msg_type == MESH_MSG_UNICAST &&
        header->hop_count == 0 &&
        sender_mesh_addr != MESH_ADDR_UNASSIGNED &&
        header->src_addr == sender_mesh_addr) {
        mesh_route_update(header->src_addr, sender_mesh_addr,
                          1, header->seq_num);
    }

    /* ---- 按消息类型分发处理 ---- */
    switch (header->msg_type) {
        case MESH_MSG_HELLO:
            process_hello(conn_id, payload, payload_len);
            return;

        case MESH_MSG_BROADCAST:
            /* 广播帧：先交给应用层处理本地数据，然后转发给其他邻居 */
            if (g_app_rx_callback != NULL) {
                g_app_rx_callback(header->src_addr, payload, payload_len);
            }
            g_stats.rx_for_me++;
            forward_broadcast(data, total_len, conn_id);
            return;

        case MESH_MSG_UNICAST:
            if (header->dst_addr == g_mesh_node_addr) {
                /* 目标是本节点：交给应用层处理，并刷新源地址路由有效期 */
                g_stats.rx_for_me++;
                mesh_route_touch(header->src_addr);
                if (g_app_rx_callback != NULL) {
                    g_app_rx_callback(header->src_addr, payload, payload_len);
                }
            } else {
                /* 目标不是本节点：需要转发给下一跳 */
                forward_unicast(data, total_len, conn_id);
            }
            return;

        /* ---- P2 阶段：AODV 路由控制报文分发给 mesh_route 处理 ---- */
        case MESH_MSG_RREQ:
            g_stats.rx_rreq++;  /* 统计接收到的 RREQ 报文数 */
            mesh_route_process_rreq(conn_id, header, payload, payload_len);
            return;

        case MESH_MSG_RREP:
            g_stats.rx_rrep++;  /* 统计接收到的 RREP 报文数 */
            mesh_route_process_rrep(conn_id, header, payload, payload_len);
            return;

        case MESH_MSG_RERR:
            g_stats.rx_rerr++;  /* 统计接收到的 RERR 报文数 */
            mesh_route_process_rerr(conn_id, header, payload, payload_len);
            return;

        default:
            osal_printk("%s fwd: unknown type 0x%02X\r\n",
                        MESH_LOG_TAG, header->msg_type);
            return;
    }
}

/* ============================================================
 *  应用层单播发送接口
 *
 *  @note P2 阶段改进：当无可用路由时，不再盲目洪泛数据帧，改为：
 *    1. 将原始应用数据缓存到 pending queue
 *    2. 调用 mesh_route_request() 发起 RREQ 路由发现
 *    3. 待 RREP 到达后由 flush_pending 或 process_pending_queue 发出
 * ============================================================ */

/**
 * @brief  发送单播数据（应用层调用）
 * @param[in] dst_addr  目标节点 Mesh 地址
 * @param[in] data      待发送的应用层数据，不可为 NULL
 * @param[in] len       数据长度，范围 [1, MESH_MAX_PAYLOAD]
 * @return ERRCODE_SUCC / ERRCODE_INVALID_PARAM / ERRCODE_FAIL
 */
errcode_t mesh_forward_send_unicast(uint16_t dst_addr, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0 || len > MESH_MAX_PAYLOAD) {
        return ERRCODE_INVALID_PARAM;
    }

    /* 目标地址为本节点时，直接回调应用层（本地回环，无需发送） */
    if (dst_addr == g_mesh_node_addr) {
        if (g_app_rx_callback != NULL) {
            g_app_rx_callback(g_mesh_node_addr, data, len);
        }
        return ERRCODE_SUCC;
    }

    mesh_frame_t frame;
    uint16_t total_len = mesh_forward_build_frame(&frame, MESH_MSG_UNICAST, dst_addr, data, len);
    if (total_len == 0) {
        return ERRCODE_FAIL;
    }

    g_stats.tx_unicast++;

    /* 路径 1: 目标是直连邻居 → 直接发送（最优路径） */
    if (mesh_transport_is_direct_neighbor(dst_addr)) {
        mesh_route_touch(dst_addr);  /* 刷新路由有效期 */
        return mesh_transport_send_to_neighbor(dst_addr, (const uint8_t *)&frame, total_len);
    }

    /* 路径 2: 查询路由表获取下一跳 */
    uint16_t next_hop = mesh_route_lookup(dst_addr);
    if (next_hop != MESH_ADDR_UNASSIGNED) {
        mesh_route_touch(dst_addr);  /* 刷新路由有效期 */
        return mesh_transport_send_to_neighbor(next_hop, (const uint8_t *)&frame, total_len);
    }

    /* 路径 3: 无可用路由 → 缓存数据 + 发起 RREQ 路由发现（P2 策略，不再洪泛） */
    osal_printk("%s send unicast: no route to 0x%04X, starting RREQ\r\n",
                MESH_LOG_TAG, dst_addr);

    /* 尝试加入待发送队列（查找第一个空闲槽位） */
    for (uint8_t i = 0; i < MESH_PENDING_QUEUE_SIZE; i++) {
        if (!g_pending_queue[i].used) {
            g_pending_queue[i].used = 1;
            g_pending_queue[i].dst_addr = dst_addr;
            g_pending_queue[i].data_len = len;
            g_pending_queue[i].enqueue_ms = osal_get_tick_ms();
            (void)memcpy_s(g_pending_queue[i].data, MESH_MAX_PAYLOAD, data, len);
            g_stats.pending_enqueued++;
            break;
        }
    }

    /* 发起 RREQ 路由发现：广播请求其他节点帮助寻找到 dst_addr 的路径 */
    mesh_route_request(dst_addr);

    return ERRCODE_SUCC;
}

/* ============================================================
 *  应用层广播发送接口
 *  @note 构建广播帧并洪泛至所有已连接的邻居节点。
 *        目标地址自动设为 MESH_ADDR_BROADCAST (0xFFFF)。
 * ============================================================ */

/**
 * @brief  发送广播数据（应用层调用）
 * @param[in] data  待发送的应用层数据，不可为 NULL
 * @param[in] len   数据长度，范围 [1, MESH_MAX_PAYLOAD]
 * @return ERRCODE_SUCC / ERRCODE_INVALID_PARAM / ERRCODE_FAIL
 */
errcode_t mesh_forward_send_broadcast(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0 || len > MESH_MAX_PAYLOAD) {
        return ERRCODE_INVALID_PARAM;
    }

    mesh_frame_t frame;
    uint16_t total_len = mesh_forward_build_frame(&frame, MESH_MSG_BROADCAST,
                                                   MESH_ADDR_BROADCAST, data, len);
    if (total_len == 0) {
        return ERRCODE_FAIL;
    }

    g_stats.tx_broadcast++;
    return mesh_transport_send_to_all_neighbors((const uint8_t *)&frame, total_len, 0xFFFF);
}

/* ============================================================
 *  HELLO 报文发送
 *  @note 构建 HELLO 载荷（本节点地址 + 邻居列表），TTL 固定为 1，
 *        广播至所有邻居。接收方通过 process_hello() 处理。
 * ============================================================ */

/**
 * @brief  发送 HELLO 报文（邻居发现 + 两跳路由学习）
 * @return ERRCODE_SUCC / ERRCODE_FAIL
 * @note   应以 MESH_HELLO_INTERVAL_MS 为间隔周期调用
 */
errcode_t mesh_forward_send_hello(void)
{
    uint8_t hello_payload[3 + MESH_HELLO_MAX_NEIGHBORS * 2];
    uint16_t idx = 0;

    /* 填入本节点 Mesh 地址（小端序：低字节在前） */
    hello_payload[idx++] = (uint8_t)(g_mesh_node_addr & 0xFF);
    hello_payload[idx++] = (uint8_t)((g_mesh_node_addr >> 8) & 0xFF);

    /* 获取当前所有直连邻居的 Mesh 地址列表 */
    uint16_t nbr_addrs[MESH_HELLO_MAX_NEIGHBORS];
    uint8_t nbr_count = mesh_transport_get_all_neighbor_addrs(nbr_addrs, MESH_HELLO_MAX_NEIGHBORS);

    hello_payload[idx++] = nbr_count;   /* 邻居数量 */
    for (uint8_t i = 0; i < nbr_count; i++) {
        hello_payload[idx++] = (uint8_t)(nbr_addrs[i] & 0xFF);          /* 邻居地址低字节 */
        hello_payload[idx++] = (uint8_t)((nbr_addrs[i] >> 8) & 0xFF);   /* 邻居地址高字节 */
    }

    /* 构建帧，然后将 TTL 强制设为 1（HELLO 仅传播一跳），并重新计算 CRC */
    mesh_frame_t frame;
    uint16_t total_len = mesh_forward_build_frame(&frame, MESH_MSG_HELLO,
                                                   MESH_ADDR_BROADCAST,
                                                   hello_payload, idx);
    if (total_len == 0) {
        return ERRCODE_FAIL;
    }
    frame.header.ttl = 1;
    frame_calc_and_set_crc(&frame, total_len);

    g_stats.tx_hello++;
    return mesh_transport_send_to_all_neighbors((const uint8_t *)&frame, total_len, 0xFFFF);
}

/* ============================================================
 *  待发送队列处理（主循环周期调用）
 *
 *  @note 遍历队列中所有占用的条目，对每个条目执行：
 *        1. 超时检查：当前时间 - 入队时间 > MESH_PENDING_TTL_MS → 丢弃
 *        2. 路由可用性检查：查询路由表 + 检查直连邻居 → 构建帧并发送
 *        建议调用间隔：100~500ms，根据系统性能和实时性需求调整。
 * ============================================================ */

/**
 * @brief  处理待发送队列（主循环周期调用）
 */
void mesh_forward_process_pending_queue(void)
{
    uint32_t now = osal_get_tick_ms();

    for (uint8_t i = 0; i < MESH_PENDING_QUEUE_SIZE; i++) {
        if (!g_pending_queue[i].used) {
            continue;
        }

        /* 检查是否超时：入队时间超过 MESH_PENDING_TTL_MS 则放弃 */
        if (now - g_pending_queue[i].enqueue_ms > MESH_PENDING_TTL_MS) {
            g_pending_queue[i].used = 0;
            g_stats.pending_expired++;
            osal_printk("%s pending expired: dst=0x%04X\r\n",
                        MESH_LOG_TAG, g_pending_queue[i].dst_addr);
            continue;
        }

        /* 检查路由是否已可用（路由表查询） */
        uint16_t dst = g_pending_queue[i].dst_addr;
        uint16_t next_hop = mesh_route_lookup(dst);

        /* 同时检查目标是否已成为直连邻居（可能在等待期间新连接建立） */
        if (next_hop == MESH_ADDR_UNASSIGNED && mesh_transport_is_direct_neighbor(dst)) {
            next_hop = dst;
        }

        if (next_hop != MESH_ADDR_UNASSIGNED) {
            /* 路由已发现：构建单播帧并发送，然后释放队列槽位 */
            mesh_frame_t frame;
            uint16_t total_len = mesh_forward_build_frame(&frame, MESH_MSG_UNICAST, dst,
                                                           g_pending_queue[i].data,
                                                           g_pending_queue[i].data_len);
            if (total_len > 0) {
                mesh_transport_send_to_neighbor(next_hop, (const uint8_t *)&frame, total_len);
                g_stats.pending_sent++;
                osal_printk("%s pending sent: dst=0x%04X via 0x%04X\r\n",
                            MESH_LOG_TAG, dst, next_hop);
            }
            g_pending_queue[i].used = 0;
        }
    }
}

/* ============================================================
 *  立即刷新待发送队列中指定目标的缓存包（P2 阶段）
 *
 *  @note 当 RREP 路由应答到达后，由 mesh_route 模块调用本函数，
 *        将队列中目标地址为 dest_addr 的所有缓存包立即发出。
 *        这避免了等待 process_pending_queue 的下一次轮询周期，
 *        减小路由发现到数据发出的延迟。
 * ============================================================ */

/**
 * @brief  刷新待发送队列中指定目标的缓存包（RREP 触发）
 * @param[in] dest_addr  目标节点 Mesh 地址
 */
void mesh_forward_flush_pending(uint16_t dest_addr)
{
    uint16_t next_hop = mesh_route_lookup(dest_addr);

    if (next_hop == MESH_ADDR_UNASSIGNED && mesh_transport_is_direct_neighbor(dest_addr)) {
        next_hop = dest_addr;
    }

    if (next_hop == MESH_ADDR_UNASSIGNED) {
        osal_printk("%s flush_pending: still no route to 0x%04X\r\n",
                    MESH_LOG_TAG, dest_addr);
        return;
    }

    for (uint8_t i = 0; i < MESH_PENDING_QUEUE_SIZE; i++) {
        if (g_pending_queue[i].used && g_pending_queue[i].dst_addr == dest_addr) {
            mesh_frame_t frame;
            uint16_t total_len = mesh_forward_build_frame(&frame, MESH_MSG_UNICAST, dest_addr,
                                                           g_pending_queue[i].data,
                                                           g_pending_queue[i].data_len);
            if (total_len > 0) {
                mesh_transport_send_to_neighbor(next_hop, (const uint8_t *)&frame, total_len);
                g_stats.pending_sent++;
                osal_printk("%s pending sent (RREP): dst=0x%04X via 0x%04X\r\n",
                            MESH_LOG_TAG, dest_addr, next_hop);
            }
            g_pending_queue[i].used = 0;
        }
    }
}

/* ============================================================
 *  统计信息接口
 *  @note 提供统计信息的获取、重置和打印功能，
 *        用于运行时性能监控和问题诊断。
 * ============================================================ */

/**
 * @brief  获取转发层统计信息（只读）
 * @return 指向内部统计结构体的常量指针
 */
const mesh_forward_stats_t *mesh_forward_get_stats(void)
{
    return &g_stats;
}

/**
 * @brief  重置转发层统计信息（所有计数器清零）
 */
void mesh_forward_reset_stats(void)
{
    (void)memset_s(&g_stats, sizeof(g_stats), 0, sizeof(g_stats));
}

/**
 * @brief  打印转发层统计信息到调试串口
 * @details 输出内容包括 TX、RX、DROP、HELLO、PENDING、AODV 各项计数。
 */
void mesh_forward_print_stats(void)
{
    osal_printk("%s === Forward Stats ===\r\n", MESH_LOG_TAG);
    osal_printk("%s TX: unicast=%d, bcast=%d, hello=%d\r\n", MESH_LOG_TAG,
                g_stats.tx_unicast, g_stats.tx_broadcast, g_stats.tx_hello);
    osal_printk("%s RX: total=%d, for_me=%d, fwd=%d\r\n", MESH_LOG_TAG,
                g_stats.rx_total, g_stats.rx_for_me, g_stats.rx_forwarded);
    osal_printk("%s DROP: dup=%d, ttl=%d, crc=%d, rate=%d\r\n", MESH_LOG_TAG,
                g_stats.rx_dropped_dup, g_stats.rx_dropped_ttl,
                g_stats.rx_dropped_crc, g_stats.rx_dropped_rate);
    osal_printk("%s HELLO rx=%d\r\n", MESH_LOG_TAG, g_stats.rx_hello);
    osal_printk("%s PENDING: enq=%d, sent=%d, expired=%d\r\n", MESH_LOG_TAG,
                g_stats.pending_enqueued, g_stats.pending_sent, g_stats.pending_expired);
    osal_printk("%s AODV: rreq tx/rx=%d/%d, rrep tx/rx=%d/%d, rerr tx/rx=%d/%d\r\n",
                MESH_LOG_TAG,
                g_stats.tx_rreq, g_stats.rx_rreq,
                g_stats.tx_rrep, g_stats.rx_rrep,
                g_stats.tx_rerr, g_stats.rx_rerr);
}