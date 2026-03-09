/**
 * @file  mesh_transport.h
 * @brief SLE Mesh 传输层 —— 连接池管理、双角色(Server/Client)数据收发接口
 *
 * @details 本模块是 SLE Mesh 协议栈的传输/连接管理层，负责管理所有 SLE 物理连接的
 *          完整生命周期（建连、维持、断开、僵尸清理），提供基于 conn_id/mesh_addr 的
 *          统一数据发送能力，并对上层（路由层 mesh_route / 协议层 mesh_protocol）屏蔽
 *          底层 SLE Server(Notify) 与 SLE Client(WriteReq) 的角色差异。
 *
 *          ┌─────────────────────────────────────────────────────────┐
 *          │                    mesh_protocol (协议层)              │
 *          │  mesh_transport_register_rx_callback() ←── 注册上行回调 │
 *          ├─────────────────────────────────────────────────────────┤
 *          │               mesh_transport (本模块: 传输层)           │
 *          │  连接池 ← alloc/free ←── server/client 建连/断开回调    │
 *          │  send_by_conn_id / send_to_neighbor / send_to_all       │
 *          ├─────────────────────────────────────────────────────────┤
 *          │  SLE Server (sle_uart_server) │ SLE Client (sle_uart_client) │
 *          └─────────────────────────────────────────────────────────┘
 *
 *          连接池容量: MESH_MAX_CONNECTIONS（默认5 = 1 Server + 4 Client），
 *          Server 端上限 MESH_MAX_SERVER_CONN，Client 端上限 MESH_MAX_CLIENT_CONN，
 *          可在 mesh_config.h 中根据硬件 RAM 资源和组网规模进行调整。
 *
 *          关键补丁说明（按编号索引，代码中以 P<N>/O<N> 标记）:
 *          - P1:  邻居 Mesh 地址仅从 HELLO 消息获取，首次锁定后拒绝覆写（防冲突）
 *          - P2:  连接断开时通知路由层生成 RERR（路由错误），触发全网路由收敛
 *          - P3:  远距离弱信号直连 fallback 检测（路由表中为多跳但物理直连）
 *          - P4:  Server 端不从 SLE 连接地址推导 mesh_addr（客户端地址≠广播地址）
 *          - P6:  僵尸连接定期清理（RX 超时 / 发送连续失败 / 单向检测 三维判定）
 *          - P7:  API 级发送失败计数（send_fail_count），用于快速僵尸判定
 *          - P10: Server Notify 延迟就绪窗口 + 单向通信超时检测（防止过早 Notify）
 *          - O2:  Turbo 模式，通过 sle_update_connect_param 动态调整连接间隔降低 RTT
 *
 *          关键可调参数（均定义于 mesh_config.h）:
 *          | 宏名                           | 默认值 | 说明                     |
 *          |--------------------------------|--------|-------------------------|
 *          | MESH_MAX_CONNECTIONS            | 5      | 连接池总容量             |
 *          | MESH_MAX_SERVER_CONN            | 1      | Server 端最大连接数       |
 *          | MESH_MAX_CLIENT_CONN            | 4      | Client 端最大连接数       |
 *          | MESH_SERVER_NOTIFY_DELAY_MS     | 300    | P10: Notify 就绪延迟(ms) |
 *          | MESH_ZOMBIE_TIMEOUT_MS          | 30000  | P6: RX 超时阈值(ms)      |
 *          | MESH_SEND_FAIL_THRESHOLD        | 5      | P7: 连续发送失败阈值      |
 *          | MESH_SERVER_ONEWAY_TIMEOUT_MS   | 15000  | P10: 单向超时阈值(ms)    |
 *          | MESH_SLE_TURBO_INTV             | 15     | O2: Turbo 间隔(×0.625ms) |
 *          | MESH_SLE_CONN_INTV_MIN          | 50     | O2: 默认间隔(×0.625ms)   |
 *
 *          线程安全: 本模块假设在单线程（LiteOS 主循环/任务）中调用，所有连接池操作
 *                    不加锁。若需多线程访问，调用方须自行保证互斥。
 *
 *          依赖模块: mesh_types.h, mesh_config.h, mesh_route.h, sle_common.h,
 *                    sle_uart_server.c, sle_uart_client.c
 *
 * Copyright (c) 2026. All rights reserved.
 *
 * History: \n
 * 2026-02-07, 创建文件。 \n
 * 2026-02-08, P2: 断开回调中增加路由层 RERR 通知。 \n
 * 2026-03-05, 全面规范化注释，增加参数调优指引。 \n
 */

#ifndef MESH_TRANSPORT_H
#define MESH_TRANSPORT_H

#include <stdint.h>
#include <stdbool.h>
#include "mesh_types.h"
#include "mesh_config.h"
#include "sle_common.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/* ============================================================
 *  连接池数据结构
 * ============================================================ */

/**
 * @brief Mesh 连接池结构体，集中管理所有活跃的 SLE 物理连接条目
 *
 * @note  1. 池容量为 MESH_MAX_CONNECTIONS（默认 5），占用约 sizeof(mesh_conn_entry_t)*5 字节 RAM，
 *           可在 mesh_config.h 中根据硬件资源和组网规模进行 capacity 调整。
 *        2. entries 数组中 state == MESH_CONN_STATE_IDLE 的槽位视为空闲，
 *           可被 alloc_entry() 分配给新连接；分配后需立即设置 state=CONNECTED。
 *        3. count / server_count / client_count 由连接/断开回调维护，
 *           count == server_count + client_count 恒成立。
 *        4. 线程安全: 无内置锁，假设所有操作在同一任务上下文中完成。
 *
 * @warning 调整 MESH_MAX_CONNECTIONS 时需同步检查 MESH_MAX_SERVER_CONN 和
 *          MESH_MAX_CLIENT_CONN，确保 server + client 上限总和 ≤ 池容量。
 */
typedef struct {
    mesh_conn_entry_t entries[MESH_MAX_CONNECTIONS]; /**< 连接条目数组，固定大小，空闲槽 state=IDLE;
                                                         每个条目包含 conn_id/role/mesh_addr/收发状态等字段 */
    uint8_t           count;            /**< 当前总活跃连接数，取值范围 [0, MESH_MAX_CONNECTIONS] */
    uint8_t           server_count;     /**< Server 角色（被连接方）的活跃连接数，上限 MESH_MAX_SERVER_CONN;
                                             超限时拒绝新的 Server 端连接 */
    uint8_t           client_count;     /**< Client 角色（主动连接方）的活跃连接数，上限 MESH_MAX_CLIENT_CONN;
                                             超限时不再发起新的 Client 连接 */
} mesh_conn_pool_t;

/* ============================================================
 *  初始化 / 反初始化
 * ============================================================ */

/**
 * @brief  初始化传输层，清空连接池并重置数据接收回调
 *
 * @return ERRCODE_SUCC 始终返回成功
 *
 * @pre    SLE 协议栈已完成基础初始化（sle_common 可用）。
 * @post   连接池所有条目 state=IDLE，count/server_count/client_count=0，
 *         g_rx_callback=NULL，日志输出 "transport init ok"。
 *
 * @note   - 应在系统上电启动时由 mesh_core_init() 调用，
 *           调用顺序: mesh_transport_init() → mesh_route_init() → mesh_protocol_init()。
 *         - 重复调用安全（幂等），会重新清空所有连接状态。
 *         - 不会主动断开已有的 SLE 物理连接，仅清除本模块的软件状态。
 */
errcode_t mesh_transport_init(void);

/**
 * @brief  反初始化传输层，释放所有连接池资源并清空回调
 *
 * @pre    无特殊前置条件，任何时刻可调用。
 * @post   连接池所有条目清零，g_rx_callback=NULL。
 *
 * @note   - 通常在系统关闭或 Mesh 模块热重启前调用。
 *         - 调用后所有连接信息丢失，需重新建连和注册回调。
 *         - 同 init，不会主动断开底层 SLE 连接。
 *
 * @warning 若有未完成的数据发送操作，反初始化后可能导致回调指针悬空。
 *          建议先停止所有发送任务再调用。
 */
void mesh_transport_deinit(void);

/* ============================================================
 *  连接池管理 —— 连接 / 断开事件处理
 *
 *  以下四个回调函数分别处理 Server 端和 Client 端的建连/断开事件，
 *  由 sle_uart_server.c / sle_uart_client.c 的 SLE 回调直接调用。
 *  处理流程: 分配/释放连接池槽位 → 初始化/清理字段 → 维护计数器 →
 *           通知路由层（断开时触发 RERR）。
 * ============================================================ */

/**
 * @brief  Server 端新连接建立回调 —— 分配槽位并初始化 Server 角色条目
 *
 * @param  conn_id  SLE 协议栈分配的连接 ID（uint16_t），后续所有收发操作均以此标识;
 *                  同一 conn_id 在连接生命周期内唯一，断开后可被 SLE 协议栈复用
 * @param  addr     对端 SLE 地址指针（6字节 sle_addr_t），可为 NULL;
 *                  即使非 NULL 也不用于推导 mesh_addr（见 P4）
 *
 * @note   调用来源: 由 sle_uart_server.c 中 SLE 连接建立回调触发，运行在 SLE 回调上下文中。
 *
 *         处理流程:
 *         1. 检查 server_count 是否达到 MESH_MAX_SERVER_CONN 上限，超限则拒绝
 *         2. 从连接池分配空闲槽位（alloc_entry），池满则拒绝
 *         3. 初始化条目: conn_id, role=SERVER, state=CONNECTED
 *         4. [P4] mesh_addr 设为 MESH_ADDR_UNASSIGNED，不从 SLE 地址推导
 *            ├── 原因: Client 连接过来时使用的是客户端连接地址,
 *            │         并非对方的 Server 广播地址，二者不同
 *            └── 真实地址等待首个 HELLO 消息到达后通过 update_mesh_addr() 设置
 *         5. [P10] 设置 notify_ready_ms = now + MESH_SERVER_NOTIFY_DELAY_MS
 *            ├── 窗口期内静默丢弃所有 Notify 发送请求（不计为失败）
 *            └── 默认 300ms，可在 mesh_config.h 中调整；若对端服务发现慢可适当增大
 *         6. 保存 SLE 地址到 entry->sle_addr（供后续僵尸清理断连使用）
 *         7. 递增 server_count 和 count
 *
 * @warning 若 Server 连接池已满（server_count >= MESH_MAX_SERVER_CONN），
 *          本次连接将被拒绝并打印日志。此时应检查是否有僵尸连接未被清理，
 *          或考虑增大 MESH_MAX_SERVER_CONN（需同步增大 MESH_MAX_CONNECTIONS）。
 */
void mesh_transport_on_server_connected(uint16_t conn_id, const sle_addr_t *addr);

/**
 * @brief  Server 端连接断开回调 —— 释放槽位并通知路由层
 *
 * @param  conn_id  断开的 SLE 连接 ID，对应之前 on_server_connected() 分配的条目
 *
 * @note   调用来源: 由 sle_uart_server.c 中 SLE 断连回调触发。
 *
 *         处理流程:
 *         1. 根据 conn_id 查找连接条目，验证 role==SERVER
 *         2. 保存 mesh_addr → 释放槽位(free_entry) → 递减 server_count 和 count
 *         3. [P2] 若 mesh_addr != UNASSIGNED，调用 mesh_route_on_link_break()
 *            通知路由层触发 RERR 消息生成，通知全网该链路不可用
 *         4. 若 mesh_addr == UNASSIGNED（从未收到过 HELLO），静默释放不触发 RERR
 *
 * @note   断开后该 conn_id 对应的条目归还为 IDLE 状态，可被后续新连接复用。
 */
void mesh_transport_on_server_disconnected(uint16_t conn_id);

/**
 * @brief  Client 端新连接建立回调 —— 分配槽位并初始化 Client 角色条目
 *
 * @param  conn_id  SLE 协议栈分配的连接 ID（uint16_t）
 * @param  addr     对端 SLE 地址指针（即 Server 端广播地址），可为 NULL;
 *                  Client 端会从此地址推导 mesh_addr（与 P4 Server 策略不同）
 *
 * @note   调用来源: 由 sle_uart_client.c 中 SLE 连接建立回调触发。
 *
 *         处理流程:
 *         1. 检查 client_count 是否达到 MESH_MAX_CLIENT_CONN 上限，超限则拒绝
 *         2. 从连接池分配空闲槽位
 *         3. 初始化条目: conn_id, role=CLIENT, state=CONNECTED
 *         4. mesh_addr 通过 mesh_transport_addr_from_sle(addr) 直接推导
 *            └── 与 Server 端不同: Client 连接的目标地址就是对方的广播地址，可靠
 *         5. notify_ready_ms = 0（Client 端使用 WriteReq 发送，无需等待）
 *         6. 保存 SLE 地址到 entry->sle_addr
 *         7. 递增 client_count 和 count
 *
 * @warning 若 Client 连接池已满（client_count >= MESH_MAX_CLIENT_CONN），
 *          本次连接将被拒绝。可调整 MESH_MAX_CLIENT_CONN 修改上限
 *          （需同步确保 MESH_MAX_CONNECTIONS >= server + client 上限之和）。
 */
void mesh_transport_on_client_connected(uint16_t conn_id, const sle_addr_t *addr);

/**
 * @brief  Client 端连接断开回调 —— 释放槽位并通知路由层
 *
 * @param  conn_id  断开的 SLE 连接 ID，对应之前 on_client_connected() 分配的条目
 *
 * @note   调用来源: 由 sle_uart_client.c 中 SLE 断连回调触发。
 *
 *         处理流程（与 Server 断开逻辑对称）:
 *         1. 根据 conn_id 查找连接条目，验证 role==CLIENT
 *         2. 保存 mesh_addr → 释放槽位 → 递减 client_count 和 count
 *         3. [P2] 若 mesh_addr != UNASSIGNED，调用 mesh_route_on_link_break()
 *            通知路由层触发 RERR，全网收敛路由
 *         4. 若 mesh_addr == UNASSIGNED（极少见，表示从未推导成功），静默释放
 *
 * @note   断开后该 conn_id 对应的条目归还为 IDLE 状态，可被后续新连接复用。
 */
void mesh_transport_on_client_disconnected(uint16_t conn_id);

/* ============================================================
 *  连接池查询接口
 *
 *  提供多种维度的连接条目检索: 按 conn_id、按 mesh_addr、按 SLE 地址，
 *  以及整体池状态查询。所有查询均为 O(N)，N = MESH_MAX_CONNECTIONS，
 *  在默认池大小（5）下无性能问题。
 * ============================================================ */

/**
 * @brief  根据 SLE 连接 ID 查找连接条目
 *
 * @param  conn_id  待查找的 SLE 连接 ID（uint16_t）
 * @return 指向匹配的 mesh_conn_entry_t 条目的可读写指针；未找到返回 NULL
 *
 * @note   - 仅匹配 state == MESH_CONN_STATE_CONNECTED 的条目，IDLE 槽位不会返回。
 *         - 返回的是可读写指针，调用方可直接修改条目字段，但需注意并发安全。
 *         - 时间复杂度: O(MESH_MAX_CONNECTIONS)，线性遍历。
 */
mesh_conn_entry_t *mesh_transport_find_by_conn_id(uint16_t conn_id);

/**
 * @brief  根据 Mesh 网络地址查找连接条目
 *
 * @param  mesh_addr  待查找的 Mesh 16位网络地址;
 *                    传入 MESH_ADDR_UNASSIGNED 将返回 NULL（未分配地址不参与匹配）
 * @return 指向匹配条目的可读写指针；未找到返回 NULL
 *
 * @note   - 仅匹配 state == MESH_CONN_STATE_CONNECTED 的条目。
 *         - 若同一 mesh_addr 同时存在于 Server 和 Client 条目中（理论上可能），
 *           返回首个匹配项（索引较小的槽位）。
 *         - 时间复杂度: O(MESH_MAX_CONNECTIONS)。
 */
mesh_conn_entry_t *mesh_transport_find_by_mesh_addr(uint16_t mesh_addr);

/**
 * @brief  检查指定 SLE 地址是否已是直连邻居（基于 SLE 地址比对）
 *
 * @param  addr  待检查的 SLE 地址指针（6字节 sle_addr_t），为 NULL 时直接返回 false
 * @return true:  该 SLE 地址已存在于连接池中（已建连）
 *         false: 不存在或 addr 为 NULL
 *
 * @note   - 通过识别 SLE 6字节地址的逻辑判定，适用于建连前的重复连接检测。
 *         - 与 is_direct_neighbor() 的区别: 本函数比对 SLE 物理地址，
 *           is_direct_neighbor() 比对 Mesh 网络地址。
 *         - 时间复杂度: O(MESH_MAX_CONNECTIONS × SLE_ADDR_LEN)。
 */
bool mesh_transport_is_neighbor(const sle_addr_t *addr);

/**
 * @brief  检查指定 Mesh 地址是否是直连邻居（基于 mesh_addr 比对）
 *
 * @param  mesh_addr  待检查的 Mesh 16位网络地址
 * @return true:  该地址在连接池中存在已连接条目
 *         false: 不存在或 mesh_addr == MESH_ADDR_UNASSIGNED
 *
 * @note   - 内部调用 find_by_mesh_addr()，适用于路由层判断是否可直接投递而无需多跳中继。
 *         - 放在快速路径上，每个数据包转发前都会调用，应保持 O(N) 低开销。
 */
bool mesh_transport_is_direct_neighbor(uint16_t mesh_addr);

/**
 * @brief  获取当前所有已连接邻居的总数（Server + Client）
 *
 * @return 当前连接池中的活跃连接数，取值范围 [0, MESH_MAX_CONNECTIONS]
 *
 * @note   等价于直接读取 g_conn_pool.count，O(1) 复杂度。
 */
uint8_t mesh_transport_get_neighbor_count(void);

/**
 * @brief  获取 Server 角色（被连接方）的当前连接数
 *
 * @return Server 端活跃连接数，取值范围 [0, MESH_MAX_SERVER_CONN]
 *
 * @note   可用于判断是否还能接受新的被连接请求。
 */
uint8_t mesh_transport_get_server_conn_count(void);

/**
 * @brief  获取 Client 角色（主动连接方）的当前连接数
 *
 * @return Client 端活跃连接数，取值范围 [0, MESH_MAX_CLIENT_CONN]
 *
 * @note   可用于判断是否还能主动发起新的连接。
 */
uint8_t mesh_transport_get_client_conn_count(void);

/**
 * @brief  获取连接池只读指针，供外部遍历查看所有连接条目
 *
 * @return 指向全局连接池 mesh_conn_pool_t 的 const 只读指针，常空指针
 *
 * @note   - 返回的是只读指针，调用方不应通过强制类型转换修改池内容。
 *         - 典型用途: 拓扑信息收集、调试日志打印、统计报告生成。
 *         - 注意: 池内容可能在下一次 SLE 回调时变更，若需快照应复制后使用。
 */
const mesh_conn_pool_t *mesh_transport_get_pool(void);

/* ============================================================
 *  数据发送接口
 * ============================================================ */

/**
 * @brief  通过指定连接 ID 发送原始数据
 *
 * @param  conn_id  目标连接 ID
 * @param  data     待发送数据缓冲区指针，不可为 NULL
 * @param  len      数据长度（字节），必须 > 0
 * @return ERRCODE_SUCC: 发送成功;
 *         ERRCODE_INVALID_PARAM: data 为 NULL 或 len 为 0;
 *         ERRCODE_FAIL: conn_id 不存在或底层 API 发送失败
 *
 * @note   内部根据连接角色自动选择发送方式:
 *         - Server 角色: 调用 sle_uart_server_send_to_conn()（Notify 方式）
 *         - Client 角色: 调用 ssapc_write_req()（WriteReq 方式）
 *         P10 补丁: Server 端在 notify_ready_ms 窗口内静默丢弃（返回 SUCC），不计为失败。
 *         P7 补丁: 发送失败时累加 send_fail_count，成功时清零，用于僵尸检测。
 */
errcode_t mesh_transport_send_by_conn_id(uint16_t conn_id, const uint8_t *data, uint16_t len);

/**
 * @brief  向指定 Mesh 地址的直连邻居发送数据
 *
 * @param  mesh_addr  目标邻居的 Mesh 网络地址
 * @param  data       待发送数据缓冲区指针
 * @param  len        数据长度（字节）
 * @return ERRCODE_SUCC: 发送成功; ERRCODE_FAIL: 该地址不在连接池中
 *
 * @note   内部先通过 find_by_mesh_addr() 查找条目，再调用 send_by_conn_id() 发送。
 *         若目标不是直连邻居，应由路由层先查询下一跳再使用 send_by_conn_id()。
 */
errcode_t mesh_transport_send_to_neighbor(uint16_t mesh_addr, const uint8_t *data, uint16_t len);

/**
 * @brief  向所有已连接邻居广播数据，可排除指定连接
 *
 * @param  data            待发送数据缓冲区指针
 * @param  len             数据长度（字节）
 * @param  exclude_conn_id 需要排除的连接 ID（用于转发时避免回环）；
 *                         若不需要排除，传入不存在的 conn_id（如 0xFFFF）即可
 * @return ERRCODE_SUCC: 全部发送成功; 否则返回最后一个失败的错误码
 *
 * @note   遍历连接池中所有 CONNECTED 状态的条目逐一发送。
 *         典型用途: HELLO 广播、RERR 泛洪、拓扑消息扩散。
 *         部分发送失败不影响其余邻居的发送。
 */
errcode_t mesh_transport_send_to_all_neighbors(const uint8_t *data, uint16_t len, uint16_t exclude_conn_id);

/* ============================================================
 *  数据接收回调注册（从 SLE 收到的原始数据）
 * ============================================================ */

/**
 * @brief  数据接收回调函数类型定义
 *
 * @param  conn_id  数据来源的连接 ID
 * @param  data     接收到的原始数据指针
 * @param  len      数据长度（字节）
 */
typedef void (*mesh_transport_rx_callback_t)(uint16_t conn_id, const uint8_t *data, uint16_t len);

/**
 * @brief  注册数据接收回调函数
 *
 * @param  cb  回调函数指针，每次收到 SLE 数据时被调用；传 NULL 可取消注册
 *
 * @note   通常在 mesh_protocol_init() 中注册协议层的解包函数。
 *         同一时刻只支持一个回调，后注册的会覆盖先前的。
 */
void mesh_transport_register_rx_callback(mesh_transport_rx_callback_t cb);

/**
 * @brief  数据接收入口，由底层 SLE 收数回调触发
 *
 * @param  conn_id  数据来源的连接 ID
 * @param  data     接收到的原始数据指针
 * @param  len      数据长度（字节）
 *
 * @note   由 sle_uart_server.c / sle_uart_client.c 的数据接收回调直接调用。
 *         内部会更新对应条目的 last_active_ms 和 send_fail_count（P7），
 *         并标记 rx_from_peer = true（P10 双向通信确认），然后转发给已注册的回调。
 */
void mesh_transport_on_data_received(uint16_t conn_id, const uint8_t *data, uint16_t len);

/* ============================================================
 *  P1: 邻居地址管理 —— Mesh 地址的获取与更新
 * ============================================================ */

/**
 * @brief  更新指定连接对应邻居的 Mesh 地址（从 HELLO 消息中获取真实地址）
 *
 * @param  conn_id    目标连接 ID
 * @param  mesh_addr  从 HELLO 消息中解析出的对端 Mesh 地址
 *
 * @note   P1-fix 锁定机制: 若该连接已有非 UNASSIGNED 的地址，拒绝覆写并打印警告。
 *         原因: 同一 conn_id 可能被 SLE 协议栈复用于 server/client 双通道，
 *         导致两个不同节点的 HELLO 以相同 conn_id 到达，首次确定身份后不再变更。
 *         P3 检测: 更新地址时检查路由表，若该节点原本是多跳可达现在变为直连，
 *         则标记为 P3 fallback 邻居（远距离弱信号直连）。
 */
void mesh_transport_update_mesh_addr(uint16_t conn_id, uint16_t mesh_addr);

/**
 * @brief  获取指定连接 ID 对应的 Mesh 地址
 *
 * @param  conn_id  目标连接 ID
 * @return 对应的 Mesh 地址; 未找到或未分配时返回 MESH_ADDR_UNASSIGNED
 */
uint16_t mesh_transport_get_mesh_addr(uint16_t conn_id);

/**
 * @brief  遍历连接池，获取所有已连接邻居的 Mesh 地址
 *
 * @param  addrs      输出数组指针，用于存放邻居地址，不可为 NULL
 * @param  max_count  输出数组的最大容量
 * @return 实际填入的邻居地址数量
 *
 * @note   内部会进行去重处理（同一 mesh_addr 可能在 Server 和 Client 池中各有条目）。
 *         跳过 mesh_addr == MESH_ADDR_UNASSIGNED 的条目（尚未收到 HELLO）。
 */
uint8_t mesh_transport_get_all_neighbor_addrs(uint16_t *addrs, uint8_t max_count);

/**
 * @brief  P3 fallback 查询: 检查指定邻居是否为远距离弱信号直连
 *
 * @param  mesh_addr  待查询的邻居 Mesh 地址
 * @return true: 该邻居被标记为 P3 fallback（建连前在路由表中为多跳）; false: 非 P3
 *
 * @note   P3 邻居通常信号弱、延迟高，路由层可据此决定优先使用多跳中继路径。
 */
bool mesh_transport_is_p3_neighbor(uint16_t mesh_addr);

/**
 * @brief  P3 fallback 查询: 获取 P3 邻居在建连前路由表中的原始跳数
 *
 * @param  mesh_addr  待查询的邻居 Mesh 地址
 * @return 建连前的路由表跳数; 非 P3 邻居返回 0
 *
 * @note   可用于评估 P3 直连的质量: 原始跳数越大，说明物理距离越远、信号越弱。
 */
uint8_t mesh_transport_get_p3_original_hops(uint16_t mesh_addr);

/* ============================================================
 *  SLE 地址到 Mesh 地址的转换
 * ============================================================ */

/**
 * @brief  从 SLE 6字节地址中提取 Mesh 16位网络地址
 *
 * @param  sle_addr  SLE 地址指针（6字节），可为 NULL
 * @return 提取的 Mesh 地址（取 addr[5]<<8 | addr[4]，即后两字节小端序）;
 *         sle_addr 为 NULL 时返回 MESH_ADDR_UNASSIGNED
 *
 * @note   Mesh 地址由 SLE 广播地址的最后两字节派生，因此要求每个节点的
 *         SLE 广播地址后两字节在网络中唯一。若实际部署中存在冲突，需修改
 *         地址分配策略或增加地址映射表。
 *         Client 端建连时可直接使用本函数推导对端地址（连接的是广播地址）；
 *         Server 端不应使用（P4 补丁: 连接地址 != 广播地址）。
 */
uint16_t mesh_transport_addr_from_sle(const sle_addr_t *sle_addr);

/* ============================================================
 *  P6: 僵尸连接定期清理
 * ============================================================ */

/**
 * @brief  清理过期的僵尸连接，释放连接池槽位
 *
 * @return 本次清理的僵尸连接数量
 *
 * @note   判定条件（满足任一即视为僵尸）:
 *         1. RX 超时: 超过 MESH_ZOMBIE_TIMEOUT_MS 未收到任何数据
 *         2. 发送连续失败: send_fail_count >= MESH_SEND_FAIL_THRESHOLD（P7 快速检测）
 *         3. 单向死亡（P10）: Server 端已过 notify 延迟窗口但从未收到对端数据，
 *            且连接时长超过 MESH_SERVER_ONEWAY_TIMEOUT_MS
 *
 *         清理流程: 释放条目 → 递减计数 → 通知路由层链路断开(RERR) →
 *         标记需重启广播(若为 Server 槽位) → 批量调用 SLE 断开远端设备。
 *
 *         典型调用: 由 mesh_core 的定时任务周期性调用，建议间隔 5~10 秒。
 *         可通过调整 MESH_ZOMBIE_TIMEOUT_MS / MESH_SEND_FAIL_THRESHOLD 控制灵敏度。
 */
uint8_t mesh_transport_cleanup_stale(void);

/* ============================================================
 *  O2: Turbo 模式 —— 动态调整 SLE 连接间隔
 * ============================================================ */

/**
 * @brief  启用或关闭 Turbo 模式，批量调整所有邻居的 SLE 连接参数
 *
 * @param  enable  true:  将所有连接间隔降到 MESH_SLE_TURBO_INTV（如 15ms），降低 RTT;
 *                 false: 恢复到默认间隔 MESH_SLE_CONN_INTV_MIN（如 50ms），节省功耗
 *
 * @note   FC（流量控制）活跃时启用 Turbo 可大幅降低多跳端到端延迟;
 *         FC 结束后应关闭以节省射频资源和电池。
 *         内部遍历连接池，对每条活跃连接调用 sle_update_connect_param()。
 *         个别连接更新失败不影响其余连接的参数调整。
 *         间隔参数可在 mesh_config.h 中根据应用场景调整。
 */
void mesh_transport_set_turbo_mode(bool enable);

/**
 * @brief  O13: 查询 SLE 链路是否处于拥塞状态
 *
 * @return true   至少有一条连接处于 FLOWCTRL 或 BUSY 状态
 * @return false  所有连接空闲, 可正常发送
 *
 * @note   基于 SLE QoS 流控回调实时更新, 可在 FC 引擎中用于拥塞感知发送节奏
 */
bool mesh_transport_is_sle_busy(void);

/* ============================================================
 *  P22: 网络分裂自愈
 * ============================================================ */

/**
 * @brief  获取连接池中去重后的唯一邻居数量
 * @return 去重后的邻居数 (不含 mesh_addr == UNASSIGNED 的条目)
 * @note   双向连接到同一节点只计为 1 个邻居。
 */
uint8_t mesh_transport_get_unique_neighbor_count(void);

/**
 * @brief  P22a: 检测是否与指定 mesh_addr 存在双向连接
 * @param  mesh_addr  待检测的对端 Mesh 地址
 * @return true = 同时存在 server 和 client 连接到该节点
 */
bool mesh_transport_has_bidirectional(uint16_t mesh_addr);

/**
 * @brief  P22c/d: 断开一条指定 conn_id 的连接并清理连接池
 * @param  conn_id  要断开的连接 ID
 * @note   内部完成: 释放池条目、递减计数、通知路由层、SLE 断开。
 *         若 conn_id 为 server 角色则设置 g_need_re_announce。
 */
void mesh_transport_force_disconnect(uint16_t conn_id);

/**
 * @brief  P22d: 查找一条存在冗余路径的 server 连接 (环路检测)
 * @return 冗余 server 连接的 conn_id; 未找到返回 0xFFFF
 * @note   判定: server 连接的对端 mesh_addr 在路由表中有通过
 *         其他邻居到达的条目 (即存在替代路径)。
 */
uint16_t mesh_transport_find_redundant_server(void);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* MESH_TRANSPORT_H */