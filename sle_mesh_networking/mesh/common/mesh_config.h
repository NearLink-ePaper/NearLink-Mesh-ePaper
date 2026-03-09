/**
 * Copyright (c) 2026. All rights reserved.
 *
 * @file   mesh_config.h
 * @brief  SLE Mesh 全局配置参数定义
 *
 * @details
 *   本文件集中定义了 SLE Mesh 网络协议栈的所有可调参数，涵盖以下模块：
 *     - 节点寻址 (手动/自动)
 *     - SLE 连接管理 (Server/Client 角色、并发数、白名单)
 *     - 定时器周期 (HELLO、扫描、路由超时、僵尸检测)
 *     - AODV 路由协议 (RREQ 重试、路由生命周期、去重缓存)
 *     - 转发层 (去重、限速、待发送队列)
 *     - SLE 射频参数 (MTU、连接间隔、广播间隔、Turbo 模式)
 *     - 流控引擎 (FC 大包负载)
 *     - 拓扑收集与日志控制
 *
 *   调参总体原则：
 *     1. 网络规模越大，HELLO/扫描间隔可适当加大以降低广播风暴；
 *     2. 跳数越多，路由超时/RREQ 超时应适当延长；
 *     3. 实时性要求越高（如图片传输），可缩短 HELLO 间隔并启用 Turbo 模式；
 *     4. 修改参数后建议先在 2~3 节点小规模验证，再逐步扩展。
 *
 * @note 本文件的宏定义会被 mesh_types.h、mesh_forward.h、mesh_transport.h、
 *       ble_gateway.c、sle_uart_client.c 等多个模块引用，修改时需确保一致性。
 *
 * History:
 * 2026-02-07, Create file.
 * 2026-02-08, P2: Add AODV routing parameters.
 */

#ifndef MESH_CONFIG_H
#define MESH_CONFIG_H

#include <stdbool.h>

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/* ============================================================
 *  OSAL 兼容层
 *
 *  BearPi-Pico H3863 SDK 仅提供 osal_get_jiffies() (单位: 10ms/jiffy)，
 *  未导出 osal_get_tick_ms()。此宏将 jiffies 转换为毫秒，供全栈使用。
 *
 *  注意：该转换精度为 10ms 级别；若 SDK 后续版本提供原生
 *  osal_get_tick_ms()，应移除此宏以使用更高精度的系统实现。
 * ============================================================ */
#include "soc_osal.h"
#ifndef osal_get_tick_ms
#define osal_get_tick_ms()  ((uint32_t)(osal_get_jiffies() * 10))
#endif

/* ============================================================
 *  节点地址配置
 *
 *  每个 Mesh 节点拥有唯一的 16-bit 地址，用于路由寻址和帧头标识。
 *
 *  工作模式：
 *    - 手动模式：将 MESH_NODE_ADDR 设为 0x0001 ~ 0xFFFD 之间的非零值。
 *      适用于节点数量固定的调试/测试场景。
 *      !! 注意：同一网络中每个节点必须使用不同的地址，否则会引发路由混乱 !!
 *
 *    - 自动模式（默认）：MESH_NODE_ADDR = 0，运行时在 sle_uart_server_adv.c
 *      的 sle_uart_server_adv_init() 中从 EFUSE Die-ID 折叠推导出 16-bit 地址。
 *      Die-ID 是芯片出厂时烧录的唯一标识，断电/重启后不变。
 *      折叠算法：偶数下标字节 XOR → 低 8 位，奇数下标字节 XOR → 高 8 位
 *      存在极小概率的地址碰撞（约 1/65533），大规模部署时建议改用手动分配。
 *      若 Die-ID 读取失败，会降级到 SLE MAC 折叠（地址可能随重启变化）。
 *
 *  保留地址：
 *    - 0x0000 (MESH_ADDR_UNASSIGNED) : 未分配 / 无效
 *    - 0xFFFF (MESH_ADDR_BROADCAST)  : 广播地址
 *    - 0xFFFE                         : BLE Gateway 内部保留
 * ============================================================ */
#ifndef MESH_NODE_ADDR
#define MESH_NODE_ADDR              0
#endif

/**
 * @brief 运行时节点地址（全局变量）
 *
 * 所有模块应统一使用此变量获取本节点地址，切勿直接使用 MESH_NODE_ADDR 宏。
 * 在自动模式下，该变量由 sle_uart_server_adv_init() 基于 EFUSE Die-ID 赋值；
 * 在手动模式下，由 mesh_main.c 的 mesh_main_task() 初始化为 MESH_NODE_ADDR。
 */
extern uint16_t g_mesh_node_addr;

/* ============================================================
 *  组播 (Multicast) 参数
 *
 *  组播传输允许网关一次发送图片到多个目标节点。
 *  数据通过 mesh_broadcast() 广播，每个目标节点检查自己是否在目标列表中。
 *  MESH_MCAST_MAX_TARGETS 定义单次组播最多支持的目标节点数。
 *  受限于 BLE MTU 和 Mesh 帧空间，建议不超过 8 个。
 * ============================================================ */
#define MESH_MCAST_MAX_TARGETS  8   /**< 单次组播最大目标节点数 */

/* ============================================================
 *  SLE 广播名称
 *
 *  Mesh 节点使用 "{PREFIX}{ADDR_HEX}" 格式的广播名称进行设备发现。
 *  例如地址为 0xEBD8 的节点广播名为 "sle_mesh_EBD8"。
 *
 *  Client 扫描时通过 strstr() 匹配 MESH_ADV_NAME_PREFIX 来识别 Mesh 节点，
 *  并从名称尾部解析 4 位十六进制地址。广播名称的修改需同步适配
 *  sle_uart_server_adv.c 和 sle_uart_client.c 的解析逻辑。
 * ============================================================ */
#define MESH_ADV_NAME_PREFIX        "sle_mesh_"
#define MESH_ADV_NAME_MAX_LEN       16      /* 广播名称最大长度 (含末尾 '\0') */

/* ============================================================
 *  SLE 连接并发数限制
 *
 *  H3863 芯片硬件约束：
 *    - Server 角色：最多被 1 个远端 Client 连入（芯片硬限制，无法修改）
 *    - Client 角色：最多主动连出 4 个远端 Server
 *    - 单板总并发连接数 = Server + Client = 5
 *
 *  MESH_MAX_CONNECTIONS 用于分配连接池 (mesh_transport.c) 的容量。
 *  若芯片型号更换或固件升级放宽了限制，可相应增大这些值。
 *
 *  调参建议：
 *    - 线性拓扑（A→B→C）：每节点一般只需 2 条连接，可适当降低 CLIENT_CONN
 *      以减少 conn_id 冲突风险。
 *    - 星形/树形拓扑：中心节点需要更多连接，保持默认即可。
 * ============================================================ */
#define MESH_MAX_SERVER_CONN        1       /* Server 角色最大入向连接数 (芯片硬限制) */
#define MESH_MAX_CLIENT_CONN        4       /* Client 角色最大出向连接数 (芯片硬限制) */
#define MESH_MAX_CONNECTIONS        (MESH_MAX_SERVER_CONN + MESH_MAX_CLIENT_CONN)  /* 单板总并发连接数 (=5) */

/* ============================================================
 *  连接白名单过滤（调试/测试用，强制构造特定拓扑）
 *
 *  - 0x0000（默认）: 不过滤，Client 扫描到任何 Mesh 节点均可连接。
 *  - 非零值: Client 仅连接该指定地址的节点，忽略其他所有 Mesh 节点。
 *
 *  使用场景：
 *    调试线性链路 A→B→C 时，在 A 上设为 B 的地址、B 上设为 C 的地址，
 *    可精确控制拓扑结构。设置后 P3 地址大小去重规则将被绕过。
 *
 *  注意：生产环境必须设为 0x0000，否则会限制网络自动组网能力。
 * ============================================================ */
#ifndef MESH_CONNECT_FILTER_ADDR
#define MESH_CONNECT_FILTER_ADDR    0x0000
#endif

/* ============================================================
 *  定时器参数（单位：毫秒）
 *
 *  这些参数控制 Mesh 网络的心跳、扫描、路由维护和故障检测周期。
 *  各参数之间存在依赖关系，调整时需综合考虑：
 *
 *  [HELLO_INTERVAL] 与路由刷新的关系：
 *    HELLO 广播承载本节点的邻居列表，远端节点通过它学习 2-hop 路由。
 *    间隔越短路由越新鲜，但广播流量越大。建议 ≥ 3000ms。
 *
 *  [ROUTE_TIMEOUT] 必须 > HELLO_INTERVAL × 3，否则路由条目可能
 *    在 HELLO 到达之前就因超时被删除，导致路由震荡。
 *
 *  [ZOMBIE_TIMEOUT] 用于检测僵尸连接（SLE 链路存在但对端已不可达），
 *    必须 > HELLO_INTERVAL × 3 以避免将正常静默期误判为僵尸。
 * ============================================================ */
#define MESH_HELLO_INTERVAL_MS      5000     /* HELLO 心跳广播间隔，默认 5 秒。
                                              * 减小可加快路由收敛，增大可降低广播开销。
                                              * 建议范围: 3000 ~ 15000ms */

#define MESH_SCAN_INTERVAL_MS       5000     /* Client 主动扫描周期（由主循环触发）。
                                              * 扫描用于发现新的 Mesh 邻居节点。
                                              * 建议与 HELLO_INTERVAL 相近，范围: 3000 ~ 10000ms */

#define MESH_SCAN_DURATION_MS       2000     /* 单次扫描持续时长。
                                              * 过短可能错过低功耗广播节点，过长则占用射频资源。
                                              * 建议范围: 1000 ~ 5000ms */

#define MESH_ROUTE_TIMEOUT_MS       90000    /* HELLO 学习的路由条目超时时间，默认 90 秒。
                                              * 超时未收到 HELLO 刷新则删除该路由。
                                              * 必须 > HELLO_INTERVAL × 3。
                                              * 建议范围: 30000 ~ 180000ms */

#define MESH_CONN_TIMEOUT_MS        15000    /* 连接级超时（预留，当前由 SLE 协议栈管理）。
                                              * 建议范围: 10000 ~ 30000ms */

#define MESH_ZOMBIE_TIMEOUT_MS      30000    /* 僵尸连接检测超时，默认 30 秒。
                                              * 若连接在此时间内无任何数据收发，且 P10 单向检测
                                              * 也未通过，则由 mesh_transport 的 cleanup_stale()
                                              * 主动断开该连接。建议范围: 20000 ~ 60000ms */

#define MESH_SEND_FAIL_THRESHOLD    10       /* 连续 API 级发送失败阈值（P7 补丁）。
                                              * 当 ssaps_notify_indicate() 或 ssapc_write_req()
                                              * 连续失败次数 ≥ 此值时，该连接被标记为僵尸。
                                              * 增大该值可容忍更多瞬态失败，减小可加速故障清理。
                                              * 建议范围: 5 ~ 20 */

/* ============================================================
 *  P10: SERVER 角色通知就绪延迟
 *
 *  背景：当对端以 Client 角色连入本节点 Server 后，SLE 协议栈需要
 *  一定时间完成 MTU 交换和服务发现。在此期间发送 notify 可能失败，
 *  导致 send_fail_count 快速累积触发僵尸判定。
 *
 *  MESH_SERVER_NOTIFY_DELAY_MS：
 *    Server 连接建立后，在此时间窗口内不向该连接发送 notify。
 *    建议 ≥ 2000ms，确保 MTU/配对流程完成。
 *
 *  MESH_SERVER_ONEWAY_TIMEOUT_MS：
 *    Server 连接开始发送 notify 后，若在此时间内未从该连接收到
 *    任何数据（HELLO/路由/业务帧），则视为单向连接，主动断开。
 *    这可检测对端 Client 已崩溃但 SLE 链路未报断开的场景。
 *    建议范围: 15000 ~ 30000ms
 * ============================================================ */
#define MESH_SERVER_NOTIFY_DELAY_MS   3000   /* Server 连接建立后的 notify 静默期 (ms) */
#define MESH_SERVER_ONEWAY_TIMEOUT_MS 20000  /* Server 连接单向检测超时 (ms) */

/* ============================================================
 *  P11: exchange_info (MTU 交换) 重试机制
 *
 *  问题现象：CLIENT 连接建立后调用 ssapc_exchange_info_req() 偶尔
 *  返回 0xEE 错误。这并非永久故障，而是 SLE 双端还在协商连接参数，
 *  此时立即标记僵尸会浪费一个可用连接。
 *
 *  解决方案：失败后延迟 MESH_EXCHANGE_INFO_RETRY_MS 再重试，
 *  最多 MESH_EXCHANGE_INFO_RETRY_MAX 次。重试期间暂用连接条目的
 *  send_fail_count 字段计数（连接刚建立该值为 0），重试耗尽则
 *  将 send_fail_count 拉满为 MESH_SEND_FAIL_THRESHOLD 触发僵尸清理。
 *
 *  调参建议：
 *    - 若 0xEE 错误频繁，可增大 RETRY_MS（如 800ms）或 RETRY_MAX（如 5）。
 *    - 若连接建立后通信正常，无需修改此参数。
 * ============================================================ */
#define MESH_EXCHANGE_INFO_RETRY_MAX  3      /* exchange_info 最大重试次数 */
#define MESH_EXCHANGE_INFO_RETRY_MS   500    /* 每次重试的间隔 (ms) */
#define MESH_MAIN_LOOP_INTERVAL_MS  100      /* 主循环空闲 tick 间隔 (ms)。
                                              * 控制 HELLO 发送、扫描触发、路由清理等周期性任务的检查频率。
                                              * 减小可提高响应速度但增加 CPU 开销。建议范围: 50 ~ 500ms */

#define MESH_FC_TICK_MS             5        /* O8: 流控引擎活跃时的主循环 tick 间隔 (ms)。
                                              * 从 10ms 降至 5ms，减少状态机驱动延迟。
                                              * FC 需要高频 tick 充分利用 SLE TX 缓冲区。
                                              * 建议范围: 3 ~ 50ms */

#define MESH_SERVER_INIT_DELAY_MS   2000     /* Mesh 主任务启动后等待 SLE Server 初始化完成的延迟 (ms)。
                                              * 此延迟确保 SSAP 服务注册、广播启动等流程完成后
                                              * 再开始 Client 扫描和 HELLO 广播。建议范围: 1000 ~ 5000ms */

/* ============================================================
 *  P8-B: conn_id 冲突防护
 *
 *  问题背景：
 *    H3863 的 SLE 协议栈维护一个共享的 conn_id 池（约 8 个 ID: 0~7）。
 *    当 Client 发起出站连接失败时，协议栈有时会为该失败事件复用一个
 *    已被其他活跃连接占用的 conn_id，导致虚假的 disconnect 事件
 *    被错误地分派到活跃连接上（即 "conn_id 冲突/串台"）。
 *
 *  防护策略：
 *    1. OUTGOING_SAFE_CLIENT_LIMIT: Client 连接数达到此上限后，
 *       暂停发起新的出站连接，减少 conn_id 池的压力。
 *       芯片最多 4 个 Client + 1 个 Server = 5 个活跃连接，
 *       保留 3 个空闲 ID 可显著降低冲突概率。
 *       设为 4 表示 Client 全满才停止（最大连接模式）。
 *       若冲突频繁，可降至 3 以预留更多空闲 ID。
 *
 *    2. CONNID_CONFLICT_BACKOFF_MULT: 检测到冲突后的退避时间乘数。
 *       普通失败使用 ×2 指数退避，冲突使用此更大乘数（默认 ×4）
 *       以更快地达到退避上限，避免反复冲击已有连接的稳定性。
 *
 *    3. CONNID_CONFLICT_GIVEUP: 连续冲突次数达到此值后，
 *       暂时放弃向该目标发起出站连接。
 *       冲突由 SLE 协议栈 conn_id 分配问题导致，短期内重试无意义。
 *       但冲突条目会在 30 分钟后由退避老化机制重置——因为长时间运行后
 *       SLE 内部 conn_id 池状态会变化，之前冲突的目标可能不再冲突。
 *       永久黑名单会导致长时间运行后网络分裂无法自愈。
 * ============================================================ */
#define MESH_OUTGOING_SAFE_CLIENT_LIMIT  4     /* 活跃 Client 连接 ≥ 此值时暂停新的 outgoing 连接 */
#define MESH_CONNID_CONFLICT_BACKOFF_MULT 4    /* conn_id 冲突退避乘数 (普通失败用 ×2) */
#define MESH_CONNID_CONFLICT_GIVEUP      3     /* 连续冲突 N 次后永久放弃向该目标发起 outgoing */

/* ============================================================
 *  转发层参数
 *
 *  控制 mesh_forward.c 中的帧去重、路由表容量和待发送队列。
 * ============================================================ */
#define MESH_DEDUP_CACHE_SIZE       64       /* 帧去重环形缓存大小（条目数）。
                                              * 每条记录一个 (src, seq) 对，防止同一帧被重复处理。
                                              * 缓存满后覆盖最旧条目。网络节点数越多或广播频率越高，
                                              * 应适当增大此值。建议范围: 32 ~ 128 */

#define MESH_ROUTE_TABLE_SIZE       32       /* AODV 路由表最大条目数。
                                              * 必须 ≥ 网络中可达节点总数，否则新路由会淘汰旧路由。
                                              * 建议范围: 16 ~ 64 */

#define MESH_PENDING_QUEUE_SIZE     8        /* 待发送队列大小（条目数）。
                                              * 当目标节点的路由未知时，帧暂存于此队列，
                                              * 同时发起 RREQ 请求。RREP 返回后自动出队发送。
                                              * 超过 MESH_PENDING_TTL_MS 仍未获得路由则丢弃。
                                              * 建议范围: 4 ~ 16 */

/* ============================================================
 *  广播帧流量控制
 *
 *  防止广播风暴：在 MESH_FLOOD_WINDOW_MS 时间窗口内，单个节点
 *  最多转发/生成 MESH_FLOOD_RATE_LIMIT 个广播帧（含 HELLO、RREQ、
 *  用户广播）。超过限额的帧将被静默丢弃。
 *
 *  MESH_FORWARD_DELAY_MS: 转发每帧前的微延迟，错开多节点同时转发
 *  导致的射频碰撞。在节点密集的网络中可适当增大（如 10~20ms）。
 *
 *  MESH_HELLO_MAX_NEIGHBORS: 单个 HELLO 帧中携带的最大邻居数。
 *  HELLO 帧大小 ≈ 2 + neighbors × 2 字节，受限于单帧 MTU。
 * ============================================================ */
#define MESH_FLOOD_RATE_LIMIT       20       /* 广播限速：窗口期内最大广播帧数。建议范围: 10 ~ 50 */
#define MESH_FLOOD_WINDOW_MS        1000     /* 广播限速窗口（ms）。建议范围: 500 ~ 2000 */
#define MESH_FORWARD_DELAY_MS       5        /* 转发前微延迟（ms），用于错开射频碰撞。建议范围: 0 ~ 20 */
#define MESH_HELLO_MAX_NEIGHBORS    20       /* HELLO 帧最大邻居列表长度。建议范围: 10 ~ 30 */

/* ============================================================
 *  待发送队列 TTL
 *
 *  MESH_PENDING_TTL_MS: 帧在待发送队列中等待路由发现的最长时间。
 *  超时后帧被丢弃。该值应 > MESH_AODV_RREQ_TIMEOUT_MS，
 *  以确保至少一轮 RREQ 有机会完成。建议范围: 3000 ~ 10000ms
 * ============================================================ */
#define MESH_PENDING_TTL_MS         5000     /* 待发送队列条目超时 (ms) */

/* ============================================================
 *  AODV (Ad-hoc On-demand Distance Vector) 路由协议参数
 *
 *  实现位于 mesh_route.c，是简化版 AODV：
 *    - 支持 RREQ / RREP / RERR 三种路由消息
 *    - RREQ 使用洪泛（全网广播）发现路由
 *    - RREP 沿反向路径单播回源
 *    - RERR 在链路断裂时通知受影响节点
 *    - 序列号机制保证路由新鲜度
 *
 *  调参关系：
 *    RREQ_TIMEOUT_MS 应 > 最大跳数 × 单跳平均延迟 × 2 (来回)
 *    ROUTE_LIFETIME_MS 控制路由的被动老化，ACTIVE_TIMEOUT_MS 控制
 *    活跃使用的路由额外续命时间。
 * ============================================================ */
#define MESH_AODV_RREQ_RETRIES       3       /* RREQ 最大重试次数。每次超时后重新洪泛。
                                              * 增大可提高多跳路径发现概率，但增加广播流量。
                                              * 建议范围: 2 ~ 5 */

#define MESH_AODV_RREQ_TIMEOUT_MS    4000    /* 单次 RREQ 等待 RREP 的超时时间 (ms)。
                                              * 超时未收到 RREP 则触发重试。
                                              * 3-hop 网络建议 ≥4000ms，更多跳数适当增大。
                                              * 建议范围: 2000 ~ 10000ms */

#define MESH_AODV_ROUTE_LIFETIME_MS  60000   /* AODV 路由条目生命周期 (ms)。
                                              * 无数据流经的路由在此时间后被自动删除。
                                              * 建议范围: 30000 ~ 120000ms */

#define MESH_AODV_ACTIVE_TIMEOUT_MS  10000   /* 活跃路由额外续命时间 (ms)。
                                              * 数据帧经过某条路由时，该路由的过期时间延长此值。
                                              * 建议范围: 5000 ~ 30000ms */

#define MESH_AODV_RREQ_CACHE_SIZE    16      /* RREQ 去重缓存大小（条目数）。
                                              * 防止同一 RREQ 被重复处理/转发，每条记录
                                              * (originator, rreq_id) 对。建议范围: 8 ~ 32 */

#define MESH_AODV_RREQ_WAIT_SIZE     4       /* 同时等待 RREP 的 RREQ 最大并发数。
                                              * 即同时向多个不同目标发起路由发现的上限。
                                              * 建议范围: 2 ~ 8 */

#define MESH_AODV_RERR_MAX_DEST      8       /* 单个 RERR 帧最多携带的不可达目标数。
                                              * 建议范围: 4 ~ 16 */

#define MESH_AODV_SEQ_INIT           1       /* 节点启动时的初始路由序列号 */

/* ============================================================
 *  SLE 射频连接参数
 *
 *  这些参数直接影响 SLE 链路层的吞吐量和功耗。
 *  单位说明：
 *    - MTU: 字节，控制单次 notify/write 的最大数据长度
 *    - CONN_INTV: SLE slot (1 slot = 0.5ms)，控制连接调度间隔
 *    - ADV_INTV: SLE slot (1 slot = 0.5ms)，控制广播调度间隔
 *
 *  MTU 设定：
 *    MESH_SLE_MTU_SIZE = 520 可支持单帧承载最大 ~504 字节 mesh 数据。
 *    需要 Server (ssaps_set_info) 和 Client (ssapc_exchange_info_req)
 *    双端一致设置，否则以较小值生效。
 *
 *  连接间隔：
 *    0x64 = 100 slots = 50ms，这是常规工作间隔。
 *    较小间隔提高吞吐量但增大功耗；较大间隔省电但延迟增大。
 *    图片传输时由 Turbo 模式动态降低间隔（见下方 O2 配置）。
 * ============================================================ */
#define MESH_SLE_MTU_SIZE           520      /* SLE MTU 大小 (字节)，Server 与 Client 需一致 */
#define MESH_SLE_CONN_INTV_MIN      0x64     /* 常规连接间隔下界 (slot)。0x64 = 100 slots = 50ms */
#define MESH_SLE_CONN_INTV_MAX      0x64     /* 常规连接间隔上界 (slot)。与 MIN 相同表示固定间隔 */
#define MESH_SLE_ADV_INTV_MIN       0xC8     /* 广播间隔下界 (slot)。0xC8 = 200 slots = 100ms */
#define MESH_SLE_ADV_INTV_MAX       0xC8     /* 广播间隔上界 (slot)。与 MIN 相同表示固定间隔 */

/* ============================================================
 *  O2: Turbo 模式 — 图片传输期间动态降低 SLE 连接间隔
 *
 *  原理：常规连接间隔 50ms 意味着每秒最多 20 次数据交换机会，
 *  对于大数据量传输（如 10800 字节图片）来说吞吐瓶颈明显。
 *  Turbo 模式通过 sle_update_connect_param() 将所有邻居连接的
 *  间隔临时降至 MESH_SLE_TURBO_INTV (默认 15ms)，传输完成后恢复。
 *
 *  MESH_SLE_TURBO_INTV: Turbo 间隔 (slot)。
 *    0x1E = 30 slots = 15ms。更小的值（如 0x0C = 6ms）进一步提升
 *    吞吐量，但可能导致 SLE 调度器无法及时完成所有连接的调度，
 *    引发连接参数更新失败。建议范围: 0x0C ~ 0x32。
 *
 *  Turbo 控制帧格式 (F19 扩展)：[MAGIC(0xFD)] [ON/OFF] [SEQ_HI] [SEQ_LO]
 *    网关在 FC 开始时向目标路径上的中继节点发送 Turbo ON，
 *    FC 结束时发送 Turbo OFF。中继节点根据序列号过滤过时帧，
 *    防止旧 TURBO OFF 覆盖新 TURBO ON (F19 乱序防护)。
 *    F18: 自发自收 (hops=0) 不发送 Turbo 帧，避免无用广播。
 *    O9:  成功完成传输后延迟 1 秒广播 TURBO OFF，减少连续传输间的
 *         快速 ON/OFF 切换对 SLE 连接稳定性的影响。
 * ============================================================ */
#define MESH_SLE_TURBO_INTV         0x0F     /* O8: Turbo 连接间隔 (slot): 0x0F = 15 slots = 7.5ms
                                              * 从 0x1E(15ms) 降至 0x0F(7.5ms)，逐跳延时减半。
                                              * H3863 最多 5 连接, 7.5ms 间隔足够调度。
                                              * 若出现连接参数更新失败可回调至 0x14(10ms) */
#define MESH_TURBO_MAGIC            0xFD     /* Turbo 控制帧前缀魔数 */
#define MESH_TURBO_ON               0x01     /* Turbo 开启命令 */
#define MESH_TURBO_OFF              0x00     /* Turbo 关闭命令 */

/* ============================================================
 *  OSAL 任务参数
 *
 *  MESH_TASK_STACK_SIZE: Mesh 主任务的栈大小 (字节)。
 *    当前设为 0x1800 (6144 字节)，需容纳所有模块的函数调用栈、
 *    局部缓冲区（如 hex dump 的临时字符串）等。
 *    如编译提示栈溢出或运行时出现随机崩溃，应适当增大。
 *
 *  MESH_TASK_PRIO: Mesh 主任务优先级 (LiteOS 数值越小优先级越高)。
 *    28 为中等偏低优先级，不影响系统关键任务。
 *    若图片传输对实时性要求极高，可降至 25~26。
 * ============================================================ */
#define MESH_TASK_STACK_SIZE        0x1800   /* Mesh 主任务栈大小: 6144 字节 */
#define MESH_TASK_PRIO              28       /* Mesh 主任务优先级 (LiteOS, 数值越小越高) */

/* ============================================================
 *  P3: 连接去重回退（允许大地址节点在特定条件下主动连接小地址节点）
 *
 *  正常规则：为避免 A→B 和 B→A 的双向重复连接，约定地址小的节点
 *  做 Client 发起连接。但如果某节点长时间邻居不足（<2），说明
 *  正常规则阻碍了组网，此时放宽限制。
 *
 *  MESH_P3_FALLBACK_TIMEOUT_MS: 节点启动后经过此时间，
 *  若邻居数仍 <2，则允许向小地址节点发起连接。
 *  已有的 is_neighbor() 去重保证不会产生真正的重复连接。
 *  建议范围: 20000 ~ 60000ms
 *
 *  MESH_PARTITION_HEAL_TIMEOUT_MS: 分区自愈检测时间。
 *  若过去此时间段内邻居数持续 < 3，说明网络可能已分裂，
 *  触发 P3 分区自愈 Fallback，允许主动向小地址节点发起连接，
 *  以尝试跨分区建立桥接。建议范围: 300000 ~ 900000ms
 *
 *  MESH_CONFLICT_COOLDOWN_MS: 冲突退避条目冷却时间。
 *  因 conn_id 冲突放弃的节点在此时间后重置退避，允许重试。
 *  长时间后 SLE conn_id 池状态已变化，之前冲突的目标可能不再冲突。
 *  建议范围: 900000 ~ 3600000ms
 * ============================================================ */
#define MESH_P3_FALLBACK_TIMEOUT_MS     30000   /* P3 回退超时: 30 秒后允许向小地址连接 */
#define MESH_PARTITION_HEAL_TIMEOUT_MS  120000  /* P22: 分区自愈检测: 2 分钟内邻居持续不足则触发 (原 10 分钟太慢) */
#define MESH_CONFLICT_COOLDOWN_MS       1800000 /* 冲突退避冷却: 30 分钟后允许重试冲突目标 */

/* ============================================================
 *  P22: 网络分裂自愈参数
 *
 *  当 MESH_MAX_SERVER_CONN=1 时，如果两个节点互连（双向连接），
 *  它们的 server 槽位都被占满，导致无法接受新连接，网络永久分裂。
 *
 *  P22a: 检测到双向连接后立即消除冗余方向
 *  P22c: 小分区（唯一邻居 ≤ 1）持续时间超过此阈值后，
 *        主动断开 server 连接以释放槽位，允许重新广播
 *  P22d: 检测到 server 连接存在冗余路径（环路），主动断开
 *        以释放槽位给跨分区连接使用
 * ============================================================ */
#define MESH_P22C_HEAL_INTERVAL_MS      30000   /* P22c 分区自愈检查周期: 30 秒 */
#define MESH_P22C_ISOLATION_THRESHOLD_MS 60000  /* P22c 孤立判定: 唯一邻居 ≤ 1 持续超过 60 秒 */

/* ============================================================
 *  拓扑查询（全网拓扑收集功能）
 *
 *  手机 APP 可通过 BLE Gateway 发起拓扑查询命令（0xFE 0x01），
 *  Gateway 将该请求广播至全网。各节点收到后回复自身邻居列表
 *  （0xFE 0x02）。Gateway 在收集窗口内汇总所有响应后，
 *  一次性将完整拓扑发送给手机。
 *
 *  MESH_TOPO_COLLECT_TIMEOUT_MS: 收集窗口时长。
 *    窗口越大收集越完整，但响应延迟越大。
 *    多跳网络建议增大（如 3000~5000ms）。
 *
 *  MESH_TOPO_MAX_NODES: 单次拓扑收集最多记录的节点数。
 *    超出容量的节点将被忽略。
 * ============================================================ */
#define MESH_TOPO_COLLECT_TIMEOUT_MS   2000   /* 拓扑收集窗口 (ms) */
#define MESH_TOPO_MAX_NODES            16     /* 最大收集节点数 */
#define MESH_TOPO_MAGIC                0xFE   /* 拓扑协议帧前缀魔数 */
#define MESH_TOPO_REQ                  0x01   /* 0xFE 0x01 = 拓扑请求（由 Gateway 广播） */
#define MESH_TOPO_RESP                 0x02   /* 0xFE 0x02 = 拓扑响应（各节点单播回 Gateway） */

/* ============================================================
 *  日志抑制标志
 *
 *  当 BLE Gateway 的流控 (FC) 引擎处于活跃传输状态时，各模块
 *  （mesh_forward、mesh_transport、sle_uart_client 等）会检查
 *  此标志，跳过冗余日志输出。这是因为 UART 日志打印会阻塞
 *  CPU 数毫秒，在高吞吐传输期间会严重拖慢流控节奏。
 *
 *  由 ble_gateway.c 的 FC 状态机设置/清除。
 * ============================================================ */
extern volatile bool g_mesh_log_suppress;

/* ============================================================
 *  O4: FC 大包负载 — 网关流控引擎每个数据包的有效载荷字节数
 *
 *  帧格式：[cmd(1B)] [seq_hi(1B)] [seq_lo(1B)] [len_hi(1B)] [len_lo(1B)] [payload(480B)]
 *  即每帧开销 5 字节，加上 payload 共 485 字节。
 *
 *  与 Mesh 帧头 (16B) 组合后单帧总长 ≈ 501B，必须 ≤ SLE MTU(520) - SSAP 开销。
 *
 *  !! 重要：此值必须与 image_receiver.h 的 IMG_PKT_PAYLOAD 保持一致 !!
 *  两端不一致将导致接收端解析错位、CRC 校验失败。
 *
 *  调参建议：
 *    - MTU=520 时最大可用: 520 - 16(mesh header) - 5(cmd overhead) = 499
 *    - 当前保守值 480，在实测中稳定可靠。
 *    - 若 MTU 被更改，需同步调整此值。
 * ============================================================ */
#define IMG_FC_PKT_PAYLOAD          480      /* FC 单包有效载荷 (字节) */

/* ============================================================
 *  日志标签
 *
 *  Mesh 核心模块统一使用此前缀标识日志来源，便于串口日志过滤。
 *  各子模块（ble_gateway、sle_uart_client 等）有各自独立的标签。
 * ============================================================ */
#define MESH_LOG_TAG                "[sle mesh]"

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* MESH_CONFIG_H */