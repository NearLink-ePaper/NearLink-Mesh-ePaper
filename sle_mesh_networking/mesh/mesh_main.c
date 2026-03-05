/**
 * @file   mesh_main.c
 * @brief  SLE Mesh 核心主循环 —— 双角色调度、UART 集成、BLE 网关桥接
 *
 * @details
 *   本文件是 BearPi-Pico H3863 SLE Mesh 图传网络的中枢模块，负责将
 *   SLE Server/Client 双角色、BLE 网关、UART 调试通道统一整合到一个
 *   OSAL 任务 (mesh_main_task) 中运行。
 *
 *   ──────────── 模块功能 ────────────
 *     1. Mesh 协议栈初始化 (mesh_init_node_addr + 三层 init)
 *        - 节点地址确定（手动 / MAC 自动推导）
 *        - transport / forward / route 三层初始化与回调链注册
 *     2. SLE 双角色启动
 *        - Server 角色：接受其他节点的入向连接
 *        - Client 角色：主动扫描并连出到邻居节点
 *        - P11: Server 启动后立即抑制广播，待 Client 就绪后再恢复
 *     3. BLE 网关 (ble_gateway) 并行启动
 *        - 与手机 BLE 连接，双向桥接 Mesh ↔ 手机
 *     4. UART 调试通道
 *        - 支持 "U:XXXX:" 前缀单播 / 默认广播
 *     5. 定时事件循环（主循环 while(1)）
 *        - 周期 HELLO 广播（P1: 携带邻居列表实现 2-hop 学习）
 *        - 周期扫描新邻居（无邻居时 2s / 有邻居时 MESH_SCAN_INTERVAL_MS）
 *        - 路由表老化 + P3 重连列表清理 + P6/P7 僵尸连接清理
 *        - P3 广播持续化：连接建立/断开后自动重启 SLE 广播
 *        - P1 待发送队列处理 + P2 RREQ 超时重试
 *        - 60s 周期统计打印
 *        - BLE 断线处理 + 拓扑收集 tick + 图片流控 tick
 *
 *   ──────────── 角色判定 ────────────
 *     - ROLE_GATEWAY : g_mesh_node_addr == MESH_ADDR_GATEWAY，作为 BLE-Mesh 桥接
 *     - ROLE_RELAY   : 有 2+ 邻居且不是 Gateway
 *     - ROLE_RECEIVER: 叶子节点，仅被动接收
 *
 *   ──────────── 消息协议 ────────────
 *     8 种消息类型：HELLO / DATA / ACK / RREQ / RREP / RERR / TOPO_REQ / TOPO_RESP
 *     应用层消息由 mesh_app_data_received 统一分发：
 *       - 0xFE 01/02 : 全网拓扑请求/响应
 *       - 0xFD       : O2 Turbo 控制帧
 *       - 0x04~0x08  : 图片传输命令（→ image_receiver）
 *       - 0x85~0x88  : 图片传输响应（→ BLE 上行专用通道）
 *       - 其他        : 通用数据，转发至手机 / 用户回调
 *
 *   ──────────── 全局标志 ────────────
 *     - g_mesh_log_suppress : 日志抑制标志 (mesh_config.h)，高频回调中减少串口输出
 *     - g_need_re_announce  : P3 重启广播标志，由连接/断开事件置位
 *
 * @note 依赖 mesh_config.h 中的 MESH_HELLO_INTERVAL_MS、
 *       MESH_ZOMBIE_CHECK_INTERVAL_MS 等定时参数。
 *       依赖 mesh_types.h 中的 mesh_header_t（src/dst/type/ttl/seq 帧头）。
 *
 * Copyright (c) 2026. All rights reserved.
 *
 * History:
 *   2026-02-07, 创建文件。
 *   2026-02-08, P2: 在主循环中增加 RREQ 超时处理。
 *   2026-02-11, 新增 BLE 网关集成 (BLE+SLE 共存)。
 */

/* ===================================================================
 *  Section 1: 头文件包含
 *
 *  分为四组：
 *    (a) 系统/SDK 基础头文件 (common_def, securec, osal, uart 等)
 *    (b) SLE 协议栈头文件 (sle_common, sle_connection_manager 等)
 *    (c) Mesh 协议栈内部模块 (types, config, api, transport, forward, route)
 *    (d) BLE 网关与图片接收模块
 * =================================================================== */

/* (a) 系统/SDK 基础头文件 */
#include "common_def.h"
#include "securec.h"
#include "soc_osal.h"
#include "app_init.h"
#include "pinctrl.h"
#include "uart.h"

/* (b) SLE 协议栈头文件 */
#include "sle_common.h"
#include "sle_errcode.h"
#include "sle_device_discovery.h"
#include "sle_connection_manager.h"
#include "sle_ssap_server.h"
#include "sle_ssap_client.h"

/* (c) Mesh 协议栈内部模块 */
#include "mesh_types.h"
#include "mesh_config.h"
#include "mesh_api.h"
#include "mesh_transport.h"
#include "mesh_forward.h"
#include "mesh_route.h"

/* (d) BLE 网关与图片接收模块 —— SLE Mesh ↔ BLE 手机桥接 */
#include "ble_gateway.h"
#include "image_receiver.h"

/* ===================================================================
 *  Section 2: 外部函数声明
 *
 *  这些函数定义在 sle_uart_server.c / sle_uart_client.c 中，
 *  已被 Mesh 化改造：支持自定义读写回调注入，以便 mesh_main
 *  拦截 SLE 数据并交给 mesh_transport 层处理。
 * =================================================================== */
extern errcode_t sle_uart_server_init(ssaps_read_request_callback read_cb,
                                       ssaps_write_request_callback write_cb);
extern void sle_uart_client_init(ssapc_notification_callback notification_cb,
                                  ssapc_indication_callback indication_cb);
extern void sle_uart_start_scan(void);
extern errcode_t sle_uart_server_adv_init(void);
extern errcode_t sle_uart_announce_register_cbks(void);
extern void sle_uart_client_reconnect_cleanup(void);  /**< P3: 重连黑名单老化清理 */

/**
 * @brief P3: 重启广播标志（volatile，跨中断/回调使用）
 *
 * SLE 协议栈在连接建立后会自动停止广播。为了让 Server 角色
 * 能持续接受新连接，连接/断开回调中将此标志置 true，
 * 主循环检测到后调用 sle_start_announce() 重新启动广播。
 *
 * @note 必须声明为 volatile，因为写入方为 SLE 回调（中断上下文），
 *       读取方为主循环（任务上下文）。
 */
volatile bool g_need_re_announce = false;

/* ===================================================================
 *  Section 3: 全局变量 —— 运行时节点地址
 *
 *  g_mesh_node_addr 是整个 Mesh 协议栈的核心标识量。
 *  其他模块通过 extern uint16_t g_mesh_node_addr 引用（声明在 mesh_config.h）。
 *  值域: 0x0001~0xFFFD（有效），0x0000 表示未分配（自动模式 pending）。
 * =================================================================== */
uint16_t g_mesh_node_addr = 0;

/**
 * @brief 初始化本节点 Mesh 地址
 *
 * @param  无
 * @return 无
 *
 * @details 处理流程：
 *   1) 检查编译时宏 MESH_NODE_ADDR 是否为非零值
 *   2) 非零 → 手动模式：直接赋值给 g_mesh_node_addr
 *   3) 零   → 自动模式：保持 g_mesh_node_addr = 0 (pending)，
 *      实际推导在 sle_uart_server_adv_init() 中完成
 *      (基于 EFUSE Die-ID 折叠，断电重启后地址不变)
 *
 * @note 自动模式下地址折叠算法见 mesh_config.h 节点地址配置节。
 */
static void mesh_init_node_addr(void)
{
#if (MESH_NODE_ADDR != 0)
    /* 手动模式: 使用编译时配置 */
    g_mesh_node_addr = MESH_NODE_ADDR;
#else
    /* 自动模式: 此时 SLE 尚未 enable, 无法读取 MAC.
     * 保持 0 表示 pending, 由 adv_init 阶段完成推导 */
    g_mesh_node_addr = 0;
#endif
}

/**
 * @brief 获取本节点 Mesh 短地址
 *
 * @return 本节点 16-bit Mesh 地址；若尚未初始化则返回 0
 *
 * @note 供 ble_gateway.c 等外部模块调用，避免直接访问全局变量。
 */
uint16_t mesh_get_my_addr(void)
{
    return g_mesh_node_addr;
}

/* ===================================================================
 *  Section 4: 模块级静态变量
 *
 *  g_node_state      : 节点状态机，跟踪初始化进度（INIT→SERVER_SETUP→CLIENT_SETUP→ACTIVE）
 *  g_user_rx_callback : 用户层数据接收回调，通过 mesh_register_rx_callback() 注册
 * =================================================================== */
static mesh_node_state_t g_node_state = MESH_NODE_STATE_INIT;
static mesh_rx_callback_t g_user_rx_callback = NULL;

/* UART 配置 —— 复用 SDK 原有参数，仅用于调试通道 */
#define SLE_UART_BAUDRATE       115200  /**< UART 波特率 (bps)，与 SDK 默认一致 */
#define SLE_UART_TRANSFER_SIZE  512     /**< UART 接收缓冲区大小 (字节)，需 ≥ 最大单帧长度 */
static uint8_t g_app_uart_rx_buff[SLE_UART_TRANSFER_SIZE] = { 0 };  /**< UART DMA 接收缓冲区 */
static uart_buffer_config_t g_app_uart_buffer_config = {  /**< UART 缓冲区配置描述符 */
    .rx_buffer = g_app_uart_rx_buff,
    .rx_buffer_size = SLE_UART_TRANSFER_SIZE
};

/* ===================================================================
 *  Section 5: UART 初始化
 *
 *  UART 仅用于调试/人工注入数据，引脚和波特率复用 SDK 原有配置。
 *  通过 mesh_uart_rx_handler 接收数据后注入 Mesh 网络。
 * =================================================================== */

/**
 * @brief 初始化 UART 引脚复用
 *
 * @details 根据 CONFIG_SLE_UART_BUS 设置 TXD/RXD 引脚的复用模式。
 *          BUS=0 和 BUS=1 均使用 PIN_MODE_1（UART 功能）。
 */
static void mesh_uart_init_pin(void)
{
    if (CONFIG_SLE_UART_BUS == 0) {
        uapi_pin_set_mode(CONFIG_UART_TXD_PIN, PIN_MODE_1);
        uapi_pin_set_mode(CONFIG_UART_RXD_PIN, PIN_MODE_1);
    } else if (CONFIG_SLE_UART_BUS == 1) {
        uapi_pin_set_mode(CONFIG_UART_TXD_PIN, PIN_MODE_1);
        uapi_pin_set_mode(CONFIG_UART_RXD_PIN, PIN_MODE_1);
    }
}

/**
 * @brief 初始化 UART 通信参数（波特率、数据位、停止位、校验位）
 *
 * @details 处理流程：
 *   1) 配置 UART 属性 (115200, 8N1)
 *   2) deinit 旧实例 → init 新实例
 *
 * @note 必须在 mesh_uart_init_pin() 之后调用。
 */
static void mesh_uart_init_config(void)
{
    uart_attr_t attr = {
        .baud_rate = SLE_UART_BAUDRATE,
        .data_bits = UART_DATA_BIT_8,
        .stop_bits = UART_STOP_BIT_1,
        .parity = UART_PARITY_NONE
    };
    uart_pin_config_t pin_config = {
        .tx_pin = CONFIG_UART_TXD_PIN,
        .rx_pin = CONFIG_UART_RXD_PIN,
        .cts_pin = PIN_NONE,
        .rts_pin = PIN_NONE
    };
    uapi_uart_deinit(CONFIG_SLE_UART_BUS);
    uapi_uart_init(CONFIG_SLE_UART_BUS, &pin_config, &attr, NULL, &g_app_uart_buffer_config);
}

/* ===================================================================
 *  Section 6: SLE Server 回调 (Mesh 版)
 *
 *  当远端 Client 连入本节点并读/写 SSAP 属性时触发。
 *  write_cbk 是核心入口：将 SLE 底层数据交给 mesh_transport 层处理。
 * =================================================================== */

/**
 * @brief Server 读请求回调（当前未使用）
 *
 * @param server_id   SSAP 服务实例 ID
 * @param conn_id     SLE 连接标识
 * @param read_cb_para 读请求参数
 * @param status      操作状态码
 *
 * @note Mesh 协议不使用读请求通道，此回调保留为空实现。
 */
static void mesh_server_read_cbk(uint8_t server_id, uint16_t conn_id,
                                  ssaps_req_read_cb_t *read_cb_para, errcode_t status)
{
    unused(server_id);
    unused(conn_id);
    unused(read_cb_para);
    unused(status);
}

/**
 * @brief Server 写请求回调 —— SLE 底层数据进入 Mesh 协议栈的入口之一
 *
 * @param server_id     SSAP 服务实例 ID（未使用）
 * @param conn_id       SLE 连接标识，用于识别数据来源邻居
 * @param write_cb_para 写请求参数，包含 value 指针和 length
 * @param status        操作状态码（未使用）
 *
 * @details 处理流程：
 *   1) 参数校验 → 2) 拷贝到栈缓冲区（防止底层共享 buffer 被覆盖）
 *   → 3) 交给 mesh_transport_on_data_received() 做帧解析和连接映射
 *
 * @note 必须先拷贝再处理：SLE 底层的 write_cb_para->value 指向共享缓冲区，
 *       下一个 SLE 事件到来时会被覆盖（WHY: SLE 底层内存复用机制）。
 */
static void mesh_server_write_cbk(uint8_t server_id, uint16_t conn_id,
                                   ssaps_req_write_cb_t *write_cb_para, errcode_t status)
{
    unused(server_id);
    unused(status);
    if (write_cb_para == NULL || write_cb_para->length == 0 || write_cb_para->value == NULL) {
        return;
    }

    /* 将数据拷贝到栈上 —— WHY: SLE 底层共享缓冲区会被下一个回调覆盖 */
    uint16_t len = write_cb_para->length;
    if (len > MESH_SLE_MTU_SIZE) {
        len = MESH_SLE_MTU_SIZE;
    }
    uint8_t local_buf[MESH_SLE_MTU_SIZE];
    (void)memcpy_s(local_buf, sizeof(local_buf), write_cb_para->value, len);

    if (!g_mesh_log_suppress) {
    osal_printk("%s server_write_cbk: conn_id=%d len=%d\r\n",
                MESH_LOG_TAG, conn_id, len);
    }

    mesh_transport_on_data_received(conn_id, local_buf, len);
}

/* ===================================================================
 *  Section 7: SLE Client 回调 (Mesh 版)
 *
 *  当本节点作为 Client 主动连出到远端 Server 后，远端通过
 *  notification / indication 推送数据时触发。
 *  处理逻辑与 Server write_cbk 对称：拷贝 → transport 层。
 * =================================================================== */

/**
 * @brief Client Notification 回调 —— 远端 Server 推送数据
 *
 * @param client_id SSAP 客户端实例 ID（未使用）
 * @param conn_id   SLE 连接标识
 * @param data      推送数据（data->data / data->data_len）
 * @param status    操作状态码（未使用）
 *
 * @details 处理流程同 mesh_server_write_cbk：
 *   拷贝到栈缓冲区 → mesh_transport_on_data_received()
 *
 * @note WHY 拷贝：同 Server 回调，SLE 底层共享缓冲区复用。
 */
static void mesh_notification_cbk(uint8_t client_id, uint16_t conn_id,
                                   ssapc_handle_value_t *data, errcode_t status)
{
    unused(client_id);
    unused(status);
    if (data == NULL || data->data_len == 0 || data->data == NULL) {
        return;
    }

    /* 将数据拷贝到栈上 —— WHY: SLE 底层共享缓冲区会被下一个回调覆盖 */
    uint16_t len = data->data_len;
    if (len > MESH_SLE_MTU_SIZE) {
        len = MESH_SLE_MTU_SIZE;
    }
    uint8_t local_buf[MESH_SLE_MTU_SIZE];
    (void)memcpy_s(local_buf, sizeof(local_buf), data->data, len);

    if (!g_mesh_log_suppress) {
    osal_printk("%s notification_cbk: conn_id=%d len=%d\r\n",
                MESH_LOG_TAG, conn_id, len);
    }

    mesh_transport_on_data_received(conn_id, local_buf, len);
}

/**
 * @brief Client Indication 回调 —— 远端 Server 可靠推送数据
 *
 * @param client_id SSAP 客户端实例 ID（未使用）
 * @param conn_id   SLE 连接标识
 * @param data      推送数据
 * @param status    操作状态码（未使用）
 *
 * @note Indication 与 Notification 的区别在于前者需要 ACK 确认，
 *       但 Mesh 层不依赖此机制，处理流程完全相同。
 */
static void mesh_indication_cbk(uint8_t client_id, uint16_t conn_id,
                                  ssapc_handle_value_t *data, errcode_t status)
{
    unused(client_id);
    unused(status);
    if (data == NULL || data->data_len == 0 || data->data == NULL) {
        return;
    }

    /* 将数据拷贝到栈上 —— WHY: SLE 底层共享缓冲区被覆盖风险 */
    uint16_t len = data->data_len;
    if (len > MESH_SLE_MTU_SIZE) {
        len = MESH_SLE_MTU_SIZE;
    }
    uint8_t local_buf[MESH_SLE_MTU_SIZE];
    (void)memcpy_s(local_buf, sizeof(local_buf), data->data, len);

    mesh_transport_on_data_received(conn_id, local_buf, len);
}

/* ===================================================================
 *  Section 8: Hex Dump 辅助工具
 *
 *  将二进制数据格式化为 "HH HH HH | ASCII..." 的可读字符串。
 *  WHY 不逐字节 printk: OSAL printk 每次调用都可能被其他任务日志打断，
 *  拼到 buffer 后一次性输出可保证一条日志的完整性。
 * =================================================================== */
static const char hex_chars[] = "0123456789ABCDEF";  /**< 十六进制字符查找表 */

/**
 * @brief 将二进制数据格式化为 hex+ASCII 字符串
 *
 * @param buf     输出缓冲区
 * @param bufsize 输出缓冲区大小
 * @param data    待格式化的二进制数据
 * @param len     数据长度
 *
 * @details 最多格式化前 64 字节（防止缓冲区溢出），
 *          格式: "HH HH HH ... | ASCII..."
 */
static void mesh_build_hex_dump(char *buf, uint16_t bufsize, const uint8_t *data, uint16_t len)
{
    uint16_t n = (len > 64) ? 64 : len;
    int pos = 0;

    for (uint16_t i = 0; i < n && pos + 4 < bufsize; i++) {
        buf[pos++] = hex_chars[(data[i] >> 4) & 0x0F];
        buf[pos++] = hex_chars[data[i] & 0x0F];
        buf[pos++] = ' ';
    }
    buf[pos++] = '|';
    buf[pos++] = ' ';
    for (uint16_t i = 0; i < n && pos + 1 < bufsize; i++) {
        buf[pos++] = (data[i] >= 0x20 && data[i] <= 0x7E) ? (char)data[i] : '.';
    }
    buf[pos] = '\0';
}

/* ===================================================================
 *  Section 9: 应用层数据接收回调 —— Mesh 解封装后的消息分发中枢
 *
 *  数据流路径: SLE 底层 → transport → forward → 本函数
 *  本函数根据 payload 首字节进行协议分发：
 *    0xFE : 全网拓扑协议 (TOPO_REQ / TOPO_RESP)
 *    0xFD : O2 Turbo 模式控制帧
 *    0x04~0x08 : 图片传输命令 → image_receiver
 *    0x85~0x88 : 图片传输响应 → BLE 上行专用通道
 *    其他 : 通用数据，桥接至手机 / 调用用户回调
 * =================================================================== */

/**
 * @brief 应用层数据接收回调 —— Mesh 解封装后调用
 *
 * @param src_addr 源节点 Mesh 地址
 * @param data     应用层有效载荷指针
 * @param len      有效载荷长度
 *
 * @details 处理流程（按优先级分发）：
 *   1) 0xFE 前缀 → 全网拓扑协议：
 *      - 0xFE 01 (TOPO_REQ): 组建本节点邻居列表并广播 TOPO_RESP
 *      - 0xFE 02 (TOPO_RESP): 喂入 BLE 网关拓扑收集器
 *   2) 0xFD 前缀 → O2 Turbo 控制帧：启用/禁用快速传输模式
 *   3) 0x04~0x08 → 图片传输命令：交给 image_receiver 处理
 *   4) 打印 hex dump 日志
 *   5) 0x85~0x88 → 图片传输响应：通过 BLE 专用上行通道发给手机
 *   6) 通用数据 → 桥接至手机 (ble_gateway_send_to_phone)
 *   7) 调用用户注册的 g_user_rx_callback
 *
 * @note 拓扑协议包和控制帧在处理后直接 return，不转发给手机/用户层。
 */
static void mesh_app_data_received(uint16_t src_addr, const uint8_t *data, uint16_t len)
{
    /* ---- 全网拓扑协议: 0xFE 前缀 (由 BLE 网关发起全网拓扑采集) ---- */
    if (len >= 2 && data[0] == 0xFE) {
        if (data[1] == 0x01) {
            /* TOPO_REQ (0xFE 01): 收到全网拓扑查询请求 → 回复自己的邻居列表 */
            uint16_t my_addr = mesh_get_my_addr();
            uint16_t nbr_addrs[8];
            uint8_t nbr_cnt = mesh_transport_get_all_neighbor_addrs(nbr_addrs, 8);

            /* 组建 TOPO_RESP 报文: FE 02 ADDR_HI ADDR_LO NBR_CNT [NBR_HI NBR_LO]... (大端序) */
            uint8_t resp[32];
            resp[0] = 0xFE;
            resp[1] = 0x02;
            resp[2] = (my_addr >> 8) & 0xFF;
            resp[3] = my_addr & 0xFF;
            resp[4] = nbr_cnt;
            uint8_t pos = 5;
            for (uint8_t i = 0; i < nbr_cnt && pos + 1 < (uint8_t)sizeof(resp); i++) {
                resp[pos++] = (nbr_addrs[i] >> 8) & 0xFF;
                resp[pos++] = nbr_addrs[i] & 0xFF;
            }
            mesh_broadcast(resp, pos);
            osal_printk("%s topo RESP: self=0x%04X, nbrs=%d\r\n",
                        MESH_LOG_TAG, my_addr, nbr_cnt);
            return;  /* 拓扑协议包不转发给手机 / 用户回调 */
        }
        if (data[1] == 0x02 && len >= 5) {
            /* TOPO_RESP: 其他节点的拓扑响应, 喂入 BLE 网关收集器 */
            ble_gateway_on_topo_resp(data, len);
            return;  /* 不转发给手机 */
        }
    }

    /* ---- O2: Turbo 控制帧 (0xFD 前缀) —— 远端下发快速传输模式开关 ---- */
    if (len >= 2 && data[0] == MESH_TURBO_MAGIC) {
        bool turbo_on = (data[1] == MESH_TURBO_ON);
        osal_printk("%s TURBO %s from 0x%04X\r\n",
                    MESH_LOG_TAG, turbo_on ? "ON" : "OFF", src_addr);
        mesh_transport_set_turbo_mode(turbo_on);
        return;  /* 控制帧不转发给用户层 */
    }

    /* ---- 图片传输命令 (0x04~0x08, 0x0A) → image_receiver 模块处理 ---- */
    if (len >= 1 && ((data[0] >= 0x04 && data[0] <= 0x08) || data[0] == 0x0A)) {
        if (image_receiver_on_data(src_addr, data, len)) {
            return;  /* 图片命令已处理，不转发给手机/用户回调 */
        }
    }

    static char dump_buf[300];
    mesh_build_hex_dump(dump_buf, sizeof(dump_buf), data, len);
    osal_printk("%s APP recv from 0x%04X, len=%d: %s\r\n", MESH_LOG_TAG, src_addr, len, dump_buf);

    /* ---- 图片传输响应 (0x85~0x88) → BLE 专用上行通道回传手机 ---- */
    if (len >= 2 && (data[0] == 0x85 || data[0] == 0x86 || data[0] == 0x87 || data[0] == 0x88)) {
        if (ble_gateway_is_connected()) {
            ble_gateway_send_image_response(src_addr, data, len);
        }
        return;  /* 不走通用 AA 81 通道 */
    }

    /* 通用数据上行: 通过 BLE 网关桥接转发至手机 */
    if (ble_gateway_is_connected()) {
        ble_gateway_send_to_phone(src_addr, data, len);
    }

    /* 调用用户注册的回调 */
    if (g_user_rx_callback != NULL) {
        g_user_rx_callback(src_addr, data, len);
    }
}

/* ===================================================================
 *  Section 10: BLE 网关桥接 —— 手机下行数据注入 Mesh 网络
 *
 *  由 ble_gateway.c 解析来自手机的 BLE 下行帧后调用本函数，
 *  根据目标地址选择广播或单播方式注入 Mesh 网络。
 *  数据流: 手机 → BLE → ble_gateway → mesh_gateway_inject → mesh_send/broadcast
 * =================================================================== */

/**
 * @brief 将手机下行数据注入 Mesh 网络
 *
 * @param dst_addr 目标 Mesh 地址（0xFFFF = 广播）
 * @param data     有效载荷指针
 * @param len      有效载荷长度
 *
 * @details 处理流程：
 *   1) 日志打印（受 g_mesh_log_suppress 控制）
 *   2) 图片数据包 (0x05) 只打 seq 号，其他打完整 hex dump
 *   3) dst_addr == MESH_ADDR_BROADCAST → mesh_broadcast()
 *   4) 否则 → mesh_send() 单播
 *
 * @note 此函数由 ble_gateway.c 在 BLE 写回调中调用，
 *       执行上下文为 BLE 协议栈任务，需注意线程安全。
 */
void mesh_gateway_inject(uint16_t dst_addr, const uint8_t *data, uint16_t len)
{
    if (!g_mesh_log_suppress) {
        osal_printk("%s [GW] inject: dst=0x%04X len=%d\r\n",
                    MESH_LOG_TAG, dst_addr, len);
    }

    /* 打印数据内容 (图片数据包只打简要信息, 其他打完整 hex) */
    if (len >= 1 && data[0] == 0x05) {
        /* IMG_DATA: 只打 seq, 不 dump 200 字节 payload */
        uint16_t seq = (len >= 3) ? ((uint16_t)data[1] << 8 | data[2]) : 0;
        if (!g_mesh_log_suppress) {
            osal_printk("%s [GW] IMG_DATA seq=%d\r\n", MESH_LOG_TAG, seq);
        }
    } else {
        static char dump_buf[300];
        mesh_build_hex_dump(dump_buf, sizeof(dump_buf), data, len);
        if (!g_mesh_log_suppress) {
            osal_printk("%s [GW] data: %s\r\n", MESH_LOG_TAG, dump_buf);
        }
    }

    if (dst_addr == MESH_ADDR_BROADCAST) {
        mesh_broadcast(data, len);
    } else {
        mesh_send(dst_addr, data, len);
    }
}

/* ===================================================================
 *  Section 11: UART 接收处理
 *
 *  UART 调试通道的数据注入入口。支持两种模式：
 *    - 单播模式: 以 "U:XXXX:" 开头（XXXX 为 4 位十六进制目标地址）
 *    - 广播模式: 其他任意数据 → 全网广播
 *
 *  设计思路: 供开发者通过串口终端快速测试 Mesh 路由可达性。
 * =================================================================== */

/**
 * @brief UART 接收回调 —— 串口数据注入 Mesh 网络
 *
 * @param buffer 接收数据缓冲区指针
 * @param length 接收数据长度
 * @param error  UART 错误标志（未使用）
 *
 * @details 处理流程：
 *   1) 参数校验
 *   2) 检查是否以 "U:XXXX:" 开头 → 解析 4 位十六进制地址 → mesh_send() 单播
 *   3) 否则 → mesh_broadcast() 广播
 */
static void mesh_uart_rx_handler(const void *buffer, uint16_t length, bool error)
{
    unused(error);

    if (buffer == NULL || length == 0) {
        return;
    }

    const char *buf = (const char *)buffer;

    /* 解析单播前缀 "U:XXXX:" */
    if (length > 7 && buf[0] == 'U' && buf[1] == ':' && buf[6] == ':') {
        char addr_str[5] = { buf[2], buf[3], buf[4], buf[5], '\0' };
        uint16_t dst_addr = (uint16_t)strtol(addr_str, NULL, 16);
        if (dst_addr != 0) {
            osal_printk("%s UART unicast to 0x%04X, len=%d\r\n",
                        MESH_LOG_TAG, dst_addr, length - 7);
            mesh_send(dst_addr, (const uint8_t *)(buf + 7), length - 7);
            return;
        }
    }

    /* 默认广播 */
    osal_printk("%s UART broadcast, len=%d\r\n", MESH_LOG_TAG, length);
    mesh_broadcast((const uint8_t *)buffer, length);
}

/* ===================================================================
 *  Section 12: 主任务 —— Mesh 协议栈的核心调度循环
 *
 *  整个 Mesh 节点的生命周期由本函数管理：
 *    阶段 0: 节点地址初始化
 *    阶段 1: transport / forward / route 三层初始化
 *    阶段 2: SLE Server 角色启动 + P11 广播抑制
 *    阶段 3: BLE 网关并行启动
 *    阶段 4: SLE Client 角色启动 + P11 恢复广播
 *    阶段 5: UART 调试通道初始化
 *    阶段 6: 进入 ACTIVE 状态 → while(1) 事件循环
 *
 *  事件循环中的定时任务：
 *    - HELLO 广播           : 每 MESH_HELLO_INTERVAL_MS（P1: 携带邻居列表 → 2-hop 学习）
 *    - 扫描新邻居            : 无邻居时 2s / 有邻居时 MESH_SCAN_INTERVAL_MS
 *    - 路由清理 + P6/P7 僵尸清理 : 每 10s
 *    - P3 广播持续化          : g_need_re_announce 触发
 *    - P1 待发送队列          : 每 tick 处理
 *    - P2 RREQ 超时重试       : 每 tick 检查
 *    - 统计打印              : 每 60s
 *    - BLE 断线检测           : 每 tick
 *    - 拓扑收集/图片流控 tick   : 每 tick
 *
 *  主循环休眠时间: FC 活跃时使用 MESH_FC_TICK_MS (P4 快速 tick)，
 *                  否则使用 MESH_MAIN_LOOP_INTERVAL_MS。
 * =================================================================== */

/**
 * @brief Mesh 主任务入口函数
 *
 * @param arg OSAL 任务参数（未使用）
 * @return 理论上不返回（while(1) 死循环）
 *
 * @note F16: startDelay 动态调整已集成到底层 transport 初始化中。
 */
static void *mesh_main_task(const char *arg)
{
    unused(arg);

    /* ---- 0. 确定节点地址 (手动/自动) ---- */
    mesh_init_node_addr();

    if (g_mesh_node_addr != 0) {
        osal_printk("%s ===== Mesh node 0x%04X starting =====\r\n",
                    MESH_LOG_TAG, g_mesh_node_addr);
    } else {
        osal_printk("%s ===== Mesh node starting (auto addr pending) =====\r\n",
                    MESH_LOG_TAG);
    }

    /* ---- 1. 初始化各模块 ---- */
    g_node_state = MESH_NODE_STATE_INIT;
    mesh_transport_init();
    mesh_forward_init();
    mesh_route_init();

    /* 注册转发层回调链: transport -> forward -> app */
    mesh_transport_register_rx_callback(mesh_forward_on_data_received);
    mesh_forward_register_app_rx_callback(mesh_app_data_received);

    /* ---- 2. 启动 Server 角色 ---- */
    g_node_state = MESH_NODE_STATE_SERVER_SETUP;
    osal_printk("%s starting Server role...\r\n", MESH_LOG_TAG);

    sle_uart_server_init(mesh_server_read_cbk, mesh_server_write_cbk);

    /* P11: server_init 内部会启动广播 — 立即抑制!
     * 如果在 CLIENT 初始化完成之前就有对端连进来, 会导致:
     *   1. SERVER 占据 conn_id=0
     *   2. 后续 CLIENT 出站连接与 conn_id=0 冲突
     *   3. 所有出站连接超时
     * 解决: 延迟广播到 SERVER + CLIENT 都就绪之后. */
    sle_stop_announce(1);
    osal_msleep(100);
    osal_printk("%s P11: advertising suppressed until CLIENT ready\r\n", MESH_LOG_TAG);

    osal_msleep(MESH_SERVER_INIT_DELAY_MS);

    /* ---- 修改5: SLE mesh 就绪后启动 BLE 网关 (parallel mode) ----
     * WHY 在此处启动: SLE 广播已停止，BLE 初始化不会与 SLE 产生时序冲突 */
    osal_printk("%s starting BLE gateway (parallel mode)...\r\n", MESH_LOG_TAG);
    ble_gateway_init();
    image_receiver_init();
    osal_msleep(800);  /* 给 BLE 协议栈注册回调 + 启动广播的时间 */

    /* ---- 3. 启动 Client 角色 ---- */
    g_node_state = MESH_NODE_STATE_CLIENT_SETUP;
    osal_printk("%s starting Client role...\r\n", MESH_LOG_TAG);

    sle_uart_client_init(mesh_notification_cbk, mesh_indication_cbk);
    osal_msleep(1000);

    /* P11: CLIENT 就绪后才启动 SLE 广播 — 此时 SERVER 和 CLIENT 均已初始化 */
    osal_printk("%s P11: CLIENT ready, starting SLE announce now\r\n", MESH_LOG_TAG);
    sle_start_announce(1);

    /* ---- 4. UART 初始化 ---- */
    mesh_uart_init_pin();
    mesh_uart_init_config();
    uapi_uart_unregister_rx_callback(CONFIG_SLE_UART_BUS);
    errcode_t ret = uapi_uart_register_rx_callback(CONFIG_SLE_UART_BUS,
                                                    UART_RX_CONDITION_FULL_OR_IDLE,
                                                    1, mesh_uart_rx_handler);
    if (ret != ERRCODE_SUCC) {
        osal_printk("%s UART callback register failed: 0x%x\r\n", MESH_LOG_TAG, ret);
    }

    /* ---- 5. 进入活跃状态 ---- */
    g_node_state = MESH_NODE_STATE_ACTIVE;
    osal_printk("%s ===== Mesh node 0x%04X ACTIVE =====\r\n",
                MESH_LOG_TAG, g_mesh_node_addr);

    /* ---- 6. 主循环 ---- */
    uint32_t last_hello_ms = 0;
    uint32_t last_scan_ms = 0;
    uint32_t last_route_cleanup_ms = 0;
    uint32_t last_stats_ms = 0;

    while (1) {
        uint32_t now = osal_get_tick_ms();

        /* 周期性 HELLO */
        if (now - last_hello_ms >= MESH_HELLO_INTERVAL_MS) {
            if (mesh_transport_get_neighbor_count() > 0) {
                mesh_forward_send_hello();
            }
            last_hello_ms = now;
        }

        /* 周期性扫描新邻居 —— 自适应间隔策略:
         * - 无邻居时: 每 2s 扫描一次 (WHY: 加速孤立节点的首次发现)
         * - 有邻居但 Client 未满: 每 MESH_SCAN_INTERVAL_MS 扫描一次 */
        uint32_t scan_interval = (mesh_transport_get_neighbor_count() == 0)
                                 ? 2000 : MESH_SCAN_INTERVAL_MS;
        if (now - last_scan_ms >= scan_interval) {
            if (mesh_transport_get_client_conn_count() < MESH_MAX_CLIENT_CONN) {
                sle_uart_start_scan();
            }
            last_scan_ms = now;
        }

        /* 路由表老化 + P3 重连列表老化 + P6/P7 僵尸连接清理 (每 10s) */
        if (now - last_route_cleanup_ms >= 10000) {
            mesh_route_cleanup();
            sle_uart_client_reconnect_cleanup();
            uint8_t zombie_cleaned = mesh_transport_cleanup_stale();
            if (zombie_cleaned > 0) {
                osal_printk("%s P7 cleanup: removed %d zombie(s)\r\n",
                            MESH_LOG_TAG, zombie_cleaned);
            }
            last_route_cleanup_ms = now;
        }

        /* P3: 广播持续化 — 连接建立/断开后重启广播 */
        if (g_need_re_announce) {
            g_need_re_announce = false;
            osal_msleep(200);  /* 等连接过程稳定 */
            if (mesh_transport_get_server_conn_count() < MESH_MAX_SERVER_CONN) {
                errcode_t adv_ret = sle_start_announce(1); /* SLE_ADV_HANDLE_DEFAULT = 1 */
                osal_printk("%s re-announce %s (server_conn=%d)\r\n", MESH_LOG_TAG,
                            (adv_ret == 0) ? "ok" : "fail",
                            mesh_transport_get_server_conn_count());
            }
        }

        /* P1: 处理待发送队列 */
        mesh_forward_process_pending_queue();

        /* P2: RREQ 超时重试检查 */
        mesh_route_process_rreq_timeouts();

        /* 定期打印统计 (每60秒), 含 BLE 网关状态 */
        if (now - last_stats_ms >= 60000) {
            mesh_forward_print_stats();
            osal_printk("%s neighbors=%d, routes=%d, ble_gw=%s\r\n",
                        MESH_LOG_TAG,
                        mesh_transport_get_neighbor_count(),
                        mesh_route_get_count(),
                        ble_gateway_is_connected() ? "connected" : "idle");
            last_stats_ms = now;
        }

        /* ---- BLE 断线: 不再暂停 SLE, BLE/SLE 并行广播 ---- */
        if (ble_gateway_needs_reconnect() && !ble_gateway_is_connected()) {
            osal_printk("%s BLE phone lost, SLE continues (parallel mode)\r\n", MESH_LOG_TAG);
            ble_gateway_clear_reconnect();  /* 清除重连标志, BLE 广播已在 disconnect 回调中重启 */
        }

        /* ---- 全网拓扑收集: 检查超时 → 发送结果给手机 ---- */
        ble_gateway_topo_tick();
        image_receiver_tick();
        ble_gateway_img_tick();  /* v2: 流控状态机驱动 */

        /* P4: 流控活跃时使用快速 tick (MESH_FC_TICK_MS)，减少主循环空等延迟 */
        osal_msleep(ble_gateway_fc_is_active() ? MESH_FC_TICK_MS : MESH_MAIN_LOOP_INTERVAL_MS);
    }

    return NULL;
}

/* ===================================================================
 *  Section 13: 对外 API 实现
 *
 *  供上层应用调用的 Mesh 网络操作接口，声明在 mesh_api.h 中。
 *  包括：初始化 / 反初始化 / 单播 / 广播 / 回调注册 / 状态查询。
 * =================================================================== */

/**
 * @brief 初始化 Mesh 协议栈（预留接口）
 *
 * @return ERRCODE_SUCC 始终成功
 *
 * @note 实际初始化工作在 mesh_main_task 中执行，此函数为预留扩展点。
 */
errcode_t mesh_init(void)
{
    return ERRCODE_SUCC;
}

/**
 * @brief 反初始化 Mesh 协议栈，释放所有子模块资源
 *
 * @details 依次释放 transport → forward → route 三层资源。
 */
void mesh_deinit(void)
{
    mesh_transport_deinit();
    mesh_forward_deinit();
    mesh_route_deinit();
}

/**
 * @brief 向指定目标节点发送单播数据
 *
 * @param dst_addr 目标 Mesh 地址 (0x0001~0xFFFD)
 * @param data     有效载荷指针
 * @param len      有效载荷长度
 * @return ERRCODE_SUCC 成功入队; 其他表示失败（无路由/队列满）
 */
errcode_t mesh_send(uint16_t dst_addr, const uint8_t *data, uint16_t len)
{
    return mesh_forward_send_unicast(dst_addr, data, len);
}

/**
 * @brief 向全网广播数据
 *
 * @param data 有效载荷指针
 * @param len  有效载荷长度
 * @return ERRCODE_SUCC 成功
 */
errcode_t mesh_broadcast(const uint8_t *data, uint16_t len)
{
    return mesh_forward_send_broadcast(data, len);
}

/**
 * @brief 注册应用层数据接收回调
 *
 * @param cb 回调函数指针，原型: void (*)(uint16_t src, const uint8_t *data, uint16_t len)
 */
void mesh_register_rx_callback(mesh_rx_callback_t cb)
{
    g_user_rx_callback = cb;
}

/**
 * @brief 获取本节点 Mesh 短地址（mesh_api.h 声明的公开 API 版本）
 *
 * @return 本节点 16-bit Mesh 地址
 */
uint16_t mesh_get_local_addr(void)
{
    return g_mesh_node_addr;
}

/**
 * @brief 获取当前直连邻居数量
 *
 * @return 活跃邻居数 (0 ~ MESH_MAX_CONNECTIONS)
 */
uint8_t mesh_get_neighbor_count(void)
{
    return mesh_transport_get_neighbor_count();
}

/**
 * @brief 获取当前路由表条目数
 *
 * @return 有效路由条目数
 */
uint8_t mesh_get_route_count(void)
{
    return mesh_route_get_count();
}

/**
 * @brief 检查目标节点是否可达（直连邻居或路由表中存在）
 *
 * @param dst_addr 目标 Mesh 地址
 * @return true 可达; false 不可达
 *
 * @note 先检查直连邻居（O(n) 扫描连接池），再查路由表。
 */
bool mesh_is_reachable(uint16_t dst_addr)
{
    if (mesh_transport_is_direct_neighbor(dst_addr)) {
        return true;
    }
    return mesh_route_is_reachable(dst_addr);
}

/* ===================================================================
 *  Section 14: 入口函数 —— OSAL 任务创建
 *
 *  通过 app_run() 宏注册为系统启动入口，创建 Mesh 主任务线程。
 *  任务栈大小: MESH_TASK_STACK_SIZE (mesh_config.h)
 *  任务优先级: MESH_TASK_PRIO (mesh_config.h)
 * =================================================================== */

/**
 * @brief Mesh 系统入口 —— 创建主任务 OSAL 线程
 *
 * @details 处理流程：
 *   1) 获取内核线程锁
 *   2) 创建 "SLEMeshTask" 线程，入口为 mesh_main_task
 *   3) 设置线程优先级
 *   4) 释放内核线程锁
 *
 * @note 由 app_run(sle_mesh_entry) 宏自动注册为系统启动回调。
 */
static void sle_mesh_entry(void)
{
    osal_task *task_handle = NULL;
    osal_kthread_lock();
    task_handle = osal_kthread_create((osal_kthread_handler)mesh_main_task,
                                      0, "SLEMeshTask", MESH_TASK_STACK_SIZE);
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, MESH_TASK_PRIO);
    }
    osal_kthread_unlock();
}

/* 注册 Mesh 入口函数为系统启动回调 —— SDK 框架在初始化完成后自动调用 */
app_run(sle_mesh_entry);