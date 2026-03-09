/**
 * @file  mesh_forward.h
 * @brief SLE Mesh 网络转发层头文件（P1 + P2 阶段）
 *
 * @details 本文件定义了 SLE Mesh 转发层的所有公共接口和数据结构。
 *          转发层是 Mesh 网络的核心数据平面，负责以下功能：
 *          - CRC16-CCITT 校验：保障帧数据完整性（多项式 0x1021，初始值 0xFFFF）
 *          - 环形去重缓存（默认 64 条目）：基于 (src_addr, seq_num) 组合判重，防止重复帧处理
 *          - 洪泛速率限制（默认每 1000ms 窗口最多 20 帧）：滑动窗口计数器，抑制广播风暴
 *          - HELLO 报文处理与两跳路由学习：周期性广播邻居信息，构建初始路由拓扑
 *          - 单播转发策略：直连邻居 → 路由表查询 → RREQ 路由发现回退
 *          - 广播转发与洪泛控制（TTL 递减 + 速率限制）
 *          - 帧合法性验证（魔数 MESH_FRAME_MAGIC、版本号、CRC 校验）
 *          - AODV 路由控制报文分发（RREQ/RREP/RERR 转交 mesh_route.c 处理）
 *          - 待发送队列（带 TTL 超时机制）：缓存等待路由发现的数据帧
 *          - 转发层统计信息采集与打印，便于运行时诊断
 *
 * @note 调用方需在系统启动时先调用 mesh_forward_init() 完成初始化，
 *       并在主循环中周期性调用 mesh_forward_process_pending_queue() 处理待发送队列。
 *       如需接收应用层数据，需通过 mesh_forward_register_app_rx_callback() 注册回调。
 *       相关配置宏（如去重缓存大小、洪泛速率等）定义在 mesh_config.h 中，
 *       可根据实际网络规模和硬件资源进行调整。
 *
 * Copyright (c) 2026. All rights reserved.
 *
 * History:
 * 2026-02-07, 创建文件。
 * 2026-02-07, P1 阶段：新增速率限制、待发送队列、HELLO 处理、统计信息。
 * 2026-02-08, P2 阶段：集成 AODV 路由协议，新增 flush_pending、RREQ/RREP/RERR 统计。
 */

#ifndef MESH_FORWARD_H
#define MESH_FORWARD_H

#include <stdint.h>
#include "mesh_types.h"
#include "errcode.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/* ============================================================
 *  初始化 / 反初始化
 *  @note 初始化函数清零所有内部状态（去重缓存、序列号、
 *        流量控制计数器、待发送队列、统计计数器），
 *        必须在使用转发层任何其他接口之前调用。
 *        反初始化函数仅清除应用层回调指针。
 * ============================================================ */

/**
 * @brief  初始化转发层
 * @details 清零去重缓存、本地序列号、流量控制窗口、待发送队列及统计计数器，
 *          使转发层进入就绪状态。
 * @return ERRCODE_SUCC 表示初始化成功
 * @note   必须在调用其他 mesh_forward_xxx 接口之前调用本函数。
 *         如需重新初始化，可直接再次调用。
 */
errcode_t mesh_forward_init(void);

/**
 * @brief  反初始化转发层
 * @details 清除应用层接收回调指针，释放转发层资源。
 * @note   调用后不应再使用转发层的发送/接收功能。
 */
void mesh_forward_deinit(void);

/* ============================================================
 *  数据接收入口
 *  @note 本函数由 mesh_transport 层在收到完整数据帧时回调，
 *        是转发层处理所有入站数据的唯一入口。
 *        内部依次执行：帧验证 → CRC 校验 → 自环检测 → 去重 →
 *        TTL 检查 → 反向路由学习 → 按消息类型分发处理。
 * ============================================================ */

/**
 * @brief  数据接收处理入口（注册给 mesh_transport 的回调函数）
 * @details 从 mesh_transport 层接收原始数据，依次执行以下处理流程：
 *          1. 帧头验证（魔数、版本号、长度合法性）
 *          2. CRC16 完整性校验
 *          3. 自环检测（丢弃源地址为本节点的帧）
 *          4. 环形去重缓存检查（基于 src_addr + seq_num 组合判重）
 *          5. TTL 存活检查
 *          6. conn_id 身份守卫（防止 SLE 双通道复用导致的身份混淆）
 *          7. 反向路由学习（从转发帧中提取路由信息）
 *          8. 按 msg_type 分发：HELLO / 广播 / 单播 / RREQ / RREP / RERR
 *
 * @param[in] conn_id  SLE 连接标识符，标识数据来源的物理连接，
 *                     用于关联 mesh_addr 和身份验证
 * @param[in] data     接收到的原始数据缓冲区指针，包含完整的 mesh 帧
 *                     （帧头 + 载荷），不可为 NULL
 * @param[in] len      数据缓冲区有效字节数，必须 >= MESH_HEADER_SIZE
 *
 * @note   本函数在 mesh_transport 的接收上下文中被调用，
 *         处理过程中可能触发帧转发（调用 mesh_transport_send_xxx）。
 *         实际部署时需根据系统负载评估是否需要异步化处理。
 */
void mesh_forward_on_data_received(uint16_t conn_id, const uint8_t *data, uint16_t len);

/* ============================================================
 *  帧封装
 *  @note 构建完整的 Mesh 帧，填写帧头各字段并计算 CRC16 校验值。
 *        源地址自动填充为本节点地址（g_mesh_node_addr），
 *        序列号自动递增。
 * ============================================================ */

/**
 * @brief  构建 Mesh 帧（帧头 + 载荷 + CRC16）
 * @details 自动填充以下帧头字段：
 *          - magic: MESH_FRAME_MAGIC（帧起始魔数）
 *          - version: MESH_FRAME_VERSION（协议版本号）
 *          - src_addr: 本节点地址（g_mesh_node_addr）
 *          - seq_num: 本地序列号（自动递增，用于去重）
 *          - ttl: MESH_DEFAULT_TTL（默认生存时间跳数）
 *          - hop_count: 0（初始跳数计数）
 *          - checksum: 基于完整帧数据计算的 CRC16-CCITT 校验值
 *
 * @param[out] frame        输出帧结构体指针，不可为 NULL
 * @param[in]  msg_type     消息类型，取值参见 MESH_MSG_xxx 宏定义
 *                          （如 MESH_MSG_UNICAST / MESH_MSG_BROADCAST / MESH_MSG_HELLO 等）
 * @param[in]  dst_addr     目标地址；广播时使用 MESH_ADDR_BROADCAST (0xFFFF)
 * @param[in]  payload      载荷数据指针，可为 NULL（当 payload_len 为 0 时）
 * @param[in]  payload_len  载荷长度（字节），不可超过 MESH_MAX_PAYLOAD；
 *                          根据实际业务数据大小设置，超过限制将返回 0
 *
 * @return 构建成功返回帧总长度（MESH_HEADER_SIZE + payload_len）；
 *         参数非法时返回 0
 *
 * @note   返回的帧总长度可直接用于后续的 mesh_transport_send_xxx 调用。
 *         如需修改 TTL（例如 HELLO 报文设为 1），需在调用后手动修改
 *         frame->header.ttl 并重新调用 frame_calc_and_set_crc 更新 CRC。
 */
uint16_t mesh_forward_build_frame(mesh_frame_t *frame,
                                   uint8_t msg_type,
                                   uint16_t dst_addr,
                                   const uint8_t *payload,
                                   uint16_t payload_len);

/* ============================================================
 *  发送接口（供应用层和路由层调用）
 *  @note 提供单播、广播和 HELLO 三种发送方式。
 *        单播发送时按以下优先级选择路径：
 *          1. 直连邻居 → 直接发送（跳过路由表查询，避免过时路由覆盖）
 *          2. 路由表命中 → 经 next_hop 定向转发
 *          3. 无路由 → 缓存至待发送队列 + 发起 RREQ 路由发现（P2 阶段）
 *        广播发送时将帧洪泛至所有已连接邻居节点。
 *        HELLO 发送时 TTL 固定为 1，仅传播一跳。
 * ============================================================ */

/**
 * @brief  发送单播数据
 * @details 构建单播帧并按优先级选择发送路径：
 *          1. 目标是直连邻居 → 直接发送（最优路径，零中间跳）
 *          2. 路由表中存在到目标的路由 → 经 next_hop 定向转发
 *          3. 无可用路由 → 将原始数据缓存至待发送队列，并发起 RREQ 路由发现
 *          特殊情况：目标地址为本节点时，直接回调应用层（本地回环）
 *
 * @param[in] dst_addr  目标节点的 Mesh 地址（16 位）；
 *                      根据实际组网地址分配策略设置
 * @param[in] data      待发送的应用层数据指针，不可为 NULL
 * @param[in] len       数据长度（字节），范围 [1, MESH_MAX_PAYLOAD]；
 *                      超出范围返回 ERRCODE_INVALID_PARAM
 *
 * @return ERRCODE_SUCC           发送成功或已加入待发送队列
 * @return ERRCODE_INVALID_PARAM  参数非法（data 为 NULL、len 为 0 或超限）
 * @return ERRCODE_FAIL           帧构建失败
 *
 * @note   当待发送队列已满时，超出的数据包将被静默丢弃。
 *         可通过增大 MESH_PENDING_QUEUE_SIZE 宏扩展队列容量。
 *         RREQ 超时未收到 RREP 时，队列中的数据包将在 MESH_PENDING_TTL_MS 后过期丢弃。
 */
errcode_t mesh_forward_send_unicast(uint16_t dst_addr, const uint8_t *data, uint16_t len);

/**
 * @brief  发送广播数据
 * @details 构建广播帧（目标地址自动设为 MESH_ADDR_BROADCAST 0xFFFF）
 *          并洪泛至所有已连接的邻居节点。
 *
 * @param[in] data  待发送的应用层数据指针，不可为 NULL
 * @param[in] len   数据长度（字节），范围 [1, MESH_MAX_PAYLOAD]
 *
 * @return ERRCODE_SUCC           发送成功
 * @return ERRCODE_INVALID_PARAM  参数非法
 * @return ERRCODE_FAIL           帧构建失败
 *
 * @note   广播帧将被所有收到的节点转发（受 TTL 和洪泛速率限制约束），
 *         使用时需注意控制广播频率以避免网络拥塞。
 */
errcode_t mesh_forward_send_broadcast(const uint8_t *data, uint16_t len);

/**
 * @brief  发送 HELLO 报文
 * @details 构建并广播 HELLO 报文，用于邻居发现和两跳路由学习。
 *          HELLO 载荷格式（小端序）：
 *            [0-1]  本节点 Mesh 地址（2 字节）
 *            [2]    邻居数量 N（1 字节）
 *            [3..2+2N]  各邻居的 Mesh 地址（每个 2 字节）
 *          TTL 固定为 1，仅传播一跳，不会被进一步转发。
 *
 * @return ERRCODE_SUCC  发送成功
 * @return ERRCODE_FAIL  帧构建失败
 *
 * @note   应在主循环中以 MESH_HELLO_INTERVAL_MS 为间隔周期调用。
 *         邻居数量上限由 MESH_HELLO_MAX_NEIGHBORS 宏控制，
 *         可根据网络规模和 SLE 最大连接数进行调整。
 */
errcode_t mesh_forward_send_hello(void);

/* ============================================================
 *  待发送队列处理（在主循环中周期调用）
 *  @note 遍历待发送队列中所有已缓存的数据包，检查：
 *        - 路由已就绪（路由表命中或目标成为直连邻居）→ 立即构建帧并发送
 *        - 超时（超过 MESH_PENDING_TTL_MS，默认 5000ms）→ 丢弃并更新统计
 *        建议在主循环中以 100~500ms 的间隔周期调用。
 * ============================================================ */

/**
 * @brief  处理待发送队列（主循环中周期调用）
 * @details 遍历 g_pending_queue 中所有标记为 used 的条目：
 *          1. 检查是否超时（当前时间 - 入队时间 > MESH_PENDING_TTL_MS），
 *             超时则释放条目并递增 pending_expired 计数
 *          2. 查询路由表或检查目标是否已成为直连邻居，
 *             若路由可用则构建帧并发送，释放条目并递增 pending_sent 计数
 *
 * @note   超时时间 MESH_PENDING_TTL_MS（默认 5000ms）可根据网络规模和
 *         RREQ 响应延迟进行调整。大规模网络建议适当增大超时值。
 *         队列大小 MESH_PENDING_QUEUE_SIZE 可根据并发路由发现数量调整。
 */
void mesh_forward_process_pending_queue(void);

/* ============================================================
 *  立即刷新待发送队列中指定目标的缓存包（P2 阶段）
 *  @note 当 RREP 路由应答到达时，由 mesh_route 模块调用本函数，
 *        将队列中目标地址匹配的所有缓存包立即发送出去。
 *        这一机制避免了等待下一次 process_pending_queue 轮询的延迟。
 * ============================================================ */

/**
 * @brief  刷新待发送队列中指定目标的缓存数据包（由 RREP 触发）
 * @details 查询 dest_addr 的路由（路由表或直连邻居），
 *          若路由可用，则遍历待发送队列，将所有目标为 dest_addr 的
 *          缓存包构建帧并立即发送。若仍无路由可用，则打印警告日志。
 *
 * @param[in] dest_addr  目标节点的 Mesh 地址，即 RREP 中携带的目的地址
 *
 * @note   本函数由 mesh_route.c 在成功处理 RREP 后调用，
 *         确保路由发现完成后缓存数据能够及时发出。
 *         如果 RREP 到达前缓存包已在 process_pending_queue 中超时过期，
 *         则不会再发送。
 */
void mesh_forward_flush_pending(uint16_t dest_addr);

/* ============================================================
 *  统计信息
 *  @note 用于监控转发层的运行状态，包括各类帧的收发计数、
 *        丢弃原因分布、待发送队列状态以及 AODV 路由报文统计。
 *        可通过 mesh_forward_print_stats() 在串口输出所有统计信息。
 * ============================================================ */

/**
 * @brief  转发层统计信息结构体
 * @details 记录转发层运行过程中的各项计数器，用于性能监控和问题诊断。
 *          各字段在 mesh_forward_init() 或 mesh_forward_reset_stats() 时清零。
 */
typedef struct {
    uint32_t tx_unicast;        /**< 发送的单播帧总数（含直连发送、路由转发、待发送队列重发） */
    uint32_t tx_broadcast;      /**< 发送的广播帧总数 */
    uint32_t tx_hello;          /**< 发送的 HELLO 报文总数 */
    uint32_t rx_total;          /**< 接收到的有效帧总数（通过帧头验证后计入） */
    uint32_t rx_for_me;         /**< 目标为本节点的帧数（含广播帧和寻址到本节点的单播帧） */
    uint32_t rx_forwarded;      /**< 成功转发的帧数（单播定向转发 + 广播洪泛转发） */
    uint32_t rx_dropped_dup;    /**< 因重复而丢弃的帧数（环形去重缓存命中） */
    uint32_t rx_dropped_ttl;    /**< 因 TTL 耗尽而丢弃的帧数（TTL == 0） */
    uint32_t rx_dropped_crc;    /**< 因 CRC16 校验失败而丢弃的帧数（数据损坏） */
    uint32_t rx_dropped_rate;   /**< 因洪泛速率限制而丢弃的帧数（超出窗口配额） */
    uint32_t rx_hello;          /**< 接收到的 HELLO 报文总数 */
    uint32_t pending_enqueued;  /**< 加入待发送队列的数据包总数（因无路由而缓存） */
    uint32_t pending_sent;      /**< 从待发送队列成功发送的数据包总数（路由发现成功后） */
    uint32_t pending_expired;   /**< 因超时过期而从待发送队列丢弃的数据包总数 */
    /* P2 阶段新增：AODV 路由控制报文统计 */
    uint32_t tx_rreq;           /**< 发送的 RREQ（路由请求）报文总数 */
    uint32_t tx_rrep;           /**< 发送的 RREP（路由应答）报文总数 */
    uint32_t tx_rerr;           /**< 发送的 RERR（路由错误）报文总数 */
    uint32_t rx_rreq;           /**< 接收到的 RREQ 报文总数 */
    uint32_t rx_rrep;           /**< 接收到的 RREP 报文总数 */
    uint32_t rx_rerr;           /**< 接收到的 RERR 报文总数 */
} mesh_forward_stats_t;

/**
 * @brief  获取转发层统计信息（只读）
 * @return 指向内部统计结构体的常量指针，始终有效（非 NULL）
 * @note   返回的指针指向模块内部静态变量，调用方不应长期缓存，
 *         因为统计值可能在中断或其他任务上下文中被更新。
 */
const mesh_forward_stats_t *mesh_forward_get_stats(void);

/**
 * @brief  重置转发层统计信息（所有计数器清零）
 * @note   常用于测试场景或周期性统计采集后重置基准值。
 */
void mesh_forward_reset_stats(void);

/**
 * @brief  打印转发层统计信息到调试串口
 * @details 以格式化文本输出所有统计计数器的当前值，包括：
 *          TX（单播/广播/HELLO）、RX（总数/本节点/转发）、
 *          DROP（重复/TTL/CRC/速率限制）、PENDING（入队/发送/过期）、
 *          AODV（RREQ/RREP/RERR 收发计数）
 * @note   输出通过 osal_printk 发送至调试串口，适用于开发调试阶段。
 */
void mesh_forward_print_stats(void);

/* ============================================================
 *  应用层回调注册
 *  @note 注册应用层的数据接收回调函数。当转发层收到目标为本节点的
 *        单播帧或广播帧时，会调用此回调将载荷数据递交给应用层。
 *        同一时间仅支持一个回调函数，重复注册会覆盖前一个。
 * ============================================================ */

/**
 * @brief  注册应用层数据接收回调
 * @param[in] cb  回调函数指针，类型为 mesh_rx_callback_t；
 *                传入 NULL 可取消注册
 * @note   回调函数在转发层处理上下文中同步调用，
 *         应用层应避免在回调中执行耗时操作以免阻塞转发处理。
 */
void mesh_forward_register_app_rx_callback(mesh_rx_callback_t cb);

/* ============================================================
 *  CRC16-CCITT 校验接口
 *  @note 采用 CRC16-CCITT 标准算法（多项式 0x1021，初始值 0xFFFF），
 *        用于 Mesh 帧数据完整性校验。对外暴露供其他模块复用。
 * ============================================================ */

/**
 * @brief  计算 CRC16-CCITT 校验值
 * @details 算法参数：多项式 = 0x1021，初始值 = 0xFFFF，
 *          逐字节高位在前（MSB first）处理，不进行最终异或。
 *
 * @param[in] data  待计算数据的缓冲区指针
 * @param[in] len   数据长度（字节）
 *
 * @return 16 位 CRC 校验值
 */
uint16_t mesh_forward_crc16(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* MESH_FORWARD_H */