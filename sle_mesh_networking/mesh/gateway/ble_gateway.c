/**
 * @file  ble_gateway.c
 * @brief BLE 网关模块 —— SLE Mesh 图片传输系统的手机通信桥梁
 *
 * 本模块实现 BLE GATT Server，作为手机 APP 与 SLE Mesh 网络之间的桥梁。
 * 核心功能包括：
 *   1. BLE GATT 服务端：提供 TX (Notify, 0xFFE1) 和 RX (Write, 0xFFE2) 特征，
 *      手机通过 BLE 连接网关进行双向通信。
 *   2. 流控引擎 (Flow Control, FC)：基于 AIMD (加性增/乘性减) 拥塞控制算法，
 *      实现分段发送 + CHECKPOINT 确认 + 自适应窗口/延时，确保 Mesh 多跳传输可靠性。
 *   3. 图片缓存管理：APP 通过 BLE 将图片数据灌入网关本地缓存 (最大 48KB)，
 *      收到 END 帧后由流控引擎自主注入 Mesh 网络。
 *   4. ACK 代理模式 (v2.2)：网关本地快速回 ACK 给 APP，数据缓存后用流控引擎统一发送，
 *      消除逐包 RTT 累积，2 跳场景从 ~50s 降至 ~10s。
 *   5. Turbo 模式 (O2)：传输期间切换到快速连接间隔，并广播通知中继节点同步切换。
 *   6. 全网拓扑收集：通过广播/响应机制收集邻居和路由节点信息，上报给 APP。
 *
 * 流控状态机 (FC State Machine):
 *   FC_IDLE → FC_START_WAIT → FC_SENDING ⇄ FC_WAIT_CHKPT
 *   → FC_SEND_END → FC_WAIT_RESULT → FC_IDLE (成功)
 *                                   → FC_RETRANSMIT → FC_SEND_END (补包)
 *
 * 版本演进：
 *   v2.0 - 新增自适应流控引擎，DATA 缓存后统一发送，CHECKPOINT 动态窗口/延时
 *   v2.1 - 新增 xfer_mode：0=FAST(流控), 1=ACK(逐包确认)
 *          START 帧新增 XFER 字节: AA 04 DST(2) TOTAL(2) PKT(2) W(2) H(2) MODE(1) XFER(1)
 *   v2.2 - ACK 代理加速：网关本地回 ACK + 缓存 + 流控引擎
 *          APP 感知逐包确认 (本地快速ACK)，Mesh 层走 FAST 模式流控
 *
 * @note 本文件基于 SDK ble_uart_server.c 改写，适配 SLE Mesh 网关场景。
 *       放置于 sle_mesh_networking/mesh/ 目录下。
 */
#include "osal_addr.h"
#include "osal_debug.h"
#include "soc_osal.h"
#include "securec.h"
#include "errcode.h"
#include "bts_def.h"
#include "bts_gatt_stru.h"
#include "bts_gatt_server.h"
#include "bts_le_gap.h"
#include "ble_gateway.h"
#include "mesh_config.h"
#include "mesh_route.h"
#include "mesh_transport.h"
#include "sle_uart_client.h"   /* P3: sle_uart_pause_scan / sle_uart_start_scan */

/* =====================================================================
 *  P1: 日志抑制全局标志
 *  流控传输期间置 true，抑制 Mesh 层冗余日志输出，避免串口带宽被日志占满。
 *  传输完成/中止后恢复为 false。其他模块可直接检查此标志决定是否打印日志。
 * ===================================================================== */
volatile bool g_mesh_log_suppress = false;

/* =====================================================================
 *  基础配置宏定义
 *  包含日志标签、BLE 广播参数、GATT UUID 及通用长度常量。
 *  根据实际硬件和应用场景调整广播间隔、服务数量等参数。
 * ===================================================================== */
#define BLE_GW_LOG              "[ble gw]"   /**< 日志前缀标签，便于过滤网关相关日志 */
#define BLE_GW_SERVER_ID        2             /**< GATT Server ID，SDK 内部标识 */
#define BLE_GW_ADV_HANDLE       0x01          /**< BLE 广播句柄，单广播实例使用 0x01 */
#define BLE_GW_ADV_MIN_INTERVAL 0x60          /**< 广播最小间隔 (单位: 0.625ms)，0x60=60ms。缩小可加快被扫描发现的速度，但增加功耗 */
#define BLE_GW_ADV_MAX_INTERVAL 0xA0          /**< 广播最大间隔 (单位: 0.625ms)，0xA0=100ms。实际间隔在 [min, max] 间随机 */
#define BLE_GW_SERVICE_NUM      3             /**< 注册的 GATT 服务数量 (通用访问 + 通用属性 + 自定义FFE0)，全部启动后才开始广播 */

/* GATT UUID 定义 —— 自定义 16-bit UUID，手机端需匹配相同 UUID 才能通信 */
#define BLE_GW_UUID_SERVICE     0xFFE0        /**< 自定义服务 UUID */
#define BLE_GW_UUID_TX          0xFFE1        /**< TX 特征 UUID (网关→手机, Notify) */
#define BLE_GW_UUID_RX          0xFFE2        /**< RX 特征 UUID (手机→网关, Write/WriteNoRsp) */
#define BLE_GW_UUID_CCC         0x2902        /**< 客户端特征配置描述符 UUID (标准 CCCD) */

#define UART16_LEN              2             /**< 16-bit UUID 字节长度 */
#define BD_ADDR_LEN_LOCAL       6             /**< BLE MAC 地址长度 (6 字节) */
#define MAX_NAME_LENGTH         20            /**< BLE 广播名称最大长度 (字节) */
#define EXT_ADV_DATA_LEN        251           /**< 扩展广播数据最大长度 (BLE 5.0 规范) */

/* =====================================================================
 *  BLE 网关全局状态
 *  管理 GATT Server 连接状态、句柄、MTU 及广播配置。
 *  这些变量在 BLE 回调中更新，由主循环和流控引擎读取。
 * ===================================================================== */
static uint8_t  g_gw_server_id = 0;           /**< GATTS 注册后返回的 Server ID */
static uint16_t g_gw_conn_id = 0;             /**< 当前 BLE 连接的 Connection ID */
static uint8_t  g_gw_connected = 0;           /**< BLE 连接状态: 0=未连接, 1=已连接 */
static uint16_t g_gw_notify_handle = 0;       /**< TX 特征 (0xFFE1) 的值句柄，用于发送 Notify */
static uint8_t  g_gw_service_start_count = 0; /**< 已启动的 GATT 服务计数，达到 BLE_GW_SERVICE_NUM 后开始广播 */
static uint16_t g_gw_mtu = 23;                /**< 当前协商的 BLE MTU 大小 (默认 23, 连接后协商为 247) */
static uint8_t  g_gw_needs_reconnect = 0;     /**< 断连标记: 1=需要重新连接 (供外部模块查询) */

static uint8_t g_gw_adv_name[MAX_NAME_LENGTH] = "sle_gw_test";  /**< BLE 广播设备名称 (初始化时会改为 sle_gw_XXXX) */
static uint8_t g_gw_adv_name_len = 11;        /**< 广播名称当前长度 */
static uint8_t g_gw_ble_addr[] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0xAA }; /**< BLE MAC 地址 (初始化时根据 mesh 地址生成) */
static uint8_t g_gw_app_uuid[] = { 0xFF, 0xE0 }; /**< GATT 应用 UUID，与服务 UUID 对应 */

/* =====================================================================
 *  Mesh 层外部函数前向声明
 *  这些函数在 mesh_route.c / mesh_transport.c 中实现，网关模块通过它们
 *  与 SLE Mesh 网络交互。修改 Mesh 层接口时需同步更新此处声明。
 * ===================================================================== */
extern uint16_t mesh_get_my_addr(void);       /**< 获取本节点的 Mesh 地址 (16-bit) */
extern void mesh_gateway_inject(uint16_t dst_addr, const uint8_t *data, uint16_t len); /**< 向 Mesh 注入数据帧，dst_addr=0xFFFF 为广播 */
extern uint8_t mesh_transport_get_all_neighbor_addrs(uint16_t *addrs, uint8_t max_count); /**< 获取所有直连邻居地址 */
extern uint8_t mesh_route_get_all_destinations(uint16_t *addrs, uint8_t *hops, uint8_t max_count); /**< 获取路由表中所有目的地及其跳数 */
extern errcode_t mesh_broadcast(const uint8_t *data, uint16_t len); /**< 向全网广播数据帧 */
extern void mesh_transport_set_turbo_mode(bool enable); /**< 设置 Turbo 模式 (快速连接间隔) */

/* =====================================================================
 *  全网拓扑收集模块
 *  网关收到手机的拓扑查询命令 (0x03) 后，向全网广播 TOPO_REQ，
 *  各节点响应 TOPO_RESP 报告其邻居列表。网关汇总后通过 BLE Notify 上报给 APP。
 *  超时时间可根据网络规模调整：节点越多、跳数越深，超时应越长。
 * ===================================================================== */
#define TOPO_COLLECT_TIMEOUT_MS   3000   /**< F17: 拓扑收集超时时间 (ms)。
                                             从 2000ms 增大到 3000ms，给单播 TOPO_RESP + AODV 路由发现预留足够时间 */
#define TOPO_MAX_NODES            16     /**< 最大可记录的拓扑节点数。如果网络节点超过此值需扩大 */
#define TOPO_MAGIC                0xFE   /**< 拓扑广播帧的魔术字节，用于区分拓扑命令与普通数据 */
#define TOPO_REQ_CMD              0x01   /**< 拓扑请求命令码 */
#define TOPO_RESP_CMD             0x02   /**< 拓扑响应命令码 */

/**
 * @brief 拓扑收集状态结构体
 *
 * 记录一次拓扑收集的全部状态：是否活跃、开始时间、已发现节点列表、
 * 响应计数等。每次新查询会重置所有字段。
 */
static struct {
    uint8_t  active;                         /**< 收集是否活跃: 0=空闲, 1=正在收集 */
    uint32_t start_time;                     /**< 收集开始时间戳 (ms) */
    uint32_t last_resp_tick;                 /**< O11: 最后一次收到 TOPO_RESP 的时间戳 (ms) */
    uint16_t nodes[TOPO_MAX_NODES];          /**< 已发现节点地址列表 */
    uint8_t  hops[TOPO_MAX_NODES];           /**< 各节点到网关的跳数 */
    uint8_t  count;                          /**< 已发现节点总数 */
    uint16_t resp_src[TOPO_MAX_NODES];       /**< 已收到响应的节点地址 (去重用) */
    uint8_t  resp_count;                     /**< 已收到的响应数量 */
    uint8_t  retry_done;                     /**< F17: 自动重试标记 (0=未重试, 1=已重试过) */
} g_topo_collect = {0};

/* =====================================================================
 *  图片传输: 网关本地缓存
 *  APP 通过 BLE 将图片数据分包发送到网关，缓存在本地 buf 中。
 *  收到 END 帧后，由流控引擎将数据重新分包 (大包负载) 注入 Mesh。
 *  缓存大小取决于目标图片最大尺寸，当前 48KB 可支持 296x128 墨水屏图片。
 * ===================================================================== */
#define IMG_GW_CACHE_SIZE     48000  /**< 图片缓存大小 (字节)。根据目标屏幕分辨率调整，
                                          要确保 >= 图片原始数据总字节数。内存受限时可缩小 */
#define IMG_GW_PKT_PAYLOAD    200    /**< APP 端每包的原始负载大小 (字节)，由 APP 度分包决定 */
#define IMG_GW_MAX_RETRY      5      /**< 流控补包最大轮次。超过此次数后放弃传输。
                                          网络质量差时可适当增大 (7-10) */
#define IMG_GW_BITMAP_BYTES   ((IMG_GW_CACHE_SIZE / IMG_FC_PKT_PAYLOAD + 7) / 8)  /**< O4: 缺包位图字节数，每 bit 表示一个 FC 大包是否缺失 */

/**
 * @brief 图片缓存结构体
 *
 * 保存当前正在传输的图片的全部元数据和原始数据。
 * active 字段指示是否有传输在进行，避免并发传输冲突。
 */
static struct {
    uint8_t  active;              /**< 传输是否活跃: 0=空闲, 1=正在传输 */
    uint16_t dst_addr;            /**< 目标节点 Mesh 地址 */
    uint16_t total_bytes;         /**< 图片数据总字节数 */
    uint16_t pkt_count;           /**< APP 端原始分包数 (每包 200B) */
    uint16_t crc16;               /**< 图片整体 CRC16 校验值，由 APP 在 END 帧中提供 */
    uint8_t  xfer_mode;           /**< v2.1 传输模式: 0=FAST(流控模式), 1=ACK(代理ACK模式)
                                       FAST: 数据全部缓存后流控发送
                                       ACK:  网关本地回 ACK + 缓存 + 流控发送 (v2.2 代理加速) */
    uint8_t  start_payload[12];   /**< START 帧的原始负载备份 (用于重传场景) */
    uint8_t  start_len;           /**< start_payload 的实际长度 */
    uint16_t fc_pkt_count;        /**< O4: Mesh FC 大包分包数 (负载为 IMG_FC_PKT_PAYLOAD，
                                       比 APP 原始包更大，减少包数提高传输效率) */
    /* --- v3: 组播支持 --- */
    uint8_t  is_multicast;        /**< 是否为组播传输: 0=单播, 1=组播 */
    uint8_t  target_count;        /**< 组播目标节点数量 (1~MESH_MCAST_MAX_TARGETS) */
    uint16_t target_addrs[MESH_MCAST_MAX_TARGETS]; /**< 组播目标地址列表 */
    uint8_t  target_done[MESH_MCAST_MAX_TARGETS];  /**< 每个目标的完成状态: 0=待定, 1=OK, 2=CRC_ERR, 3=超时 */
    uint8_t  completed_count;     /**< 已完成（包括成功和失败）的目标数 */
    /* F23: 保存 MCAST_START 帧用于重播，确保后加入的节点也能收到 */
    uint8_t  mcast_start_frame[250]; /**< MCAST_START 帧内容缓存 */
    uint16_t mcast_start_len;        /**< MCAST_START 帧长度 */
    uint8_t  buf[IMG_GW_CACHE_SIZE]; /**< 图片原始数据缓存区 */
} g_img_cache = {0};

/* =====================================================================
 *  流控状态机 (Flow Control State Machine) —— v2 核心模块
 * =====================================================================
 *
 * 架构概述:
 *   APP 通过 BLE 将所有图片数据灌入网关本地缓存 (g_img_cache.buf)，
 *   收到 END 帧后，网关自主进行流控注入 Mesh 网络。
 *   采用 “分段发送 + CHECKPOINT 确认 + AIMD 自适应窗口/延时” 策略，
 *   类似 TCP 拥塞控制，但针对无线 Mesh 少跳、高丢包场景优化。
 *
 * 状态转换图:
 *   FC_IDLE ──(收到 END)──→ FC_START_WAIT ──(延时到期)──→ FC_SENDING
 *   FC_SENDING ──(段发完)──→ FC_WAIT_CHKPT ──(ACK 到达 + 未全部发完)──→ FC_SENDING
 *   FC_WAIT_CHKPT ──(ACK 到达 + 全部发完)──→ FC_SEND_END
 *   FC_WAIT_CHKPT ──(超时 + 重试耗尽)──→ FC_SENDING (重发整段)
 *   FC_SEND_END ──→ FC_WAIT_RESULT
 *   FC_WAIT_RESULT ──(RESULT OK)──→ FC_IDLE (传输完成)
 *   FC_WAIT_RESULT ──(MISSING 位图)──→ FC_RETRANSMIT ──(补发完成)──→ FC_SEND_END
 *   FC_WAIT_RESULT ──(超时)──→ FC_SEND_END (重发 END)
 *
 * 注意: v2.2 后 ACK 代理模式同样使用此流控引擎。
 *       网关代理 ACK 给 APP，目标节点按 FAST 模式接收。
 * ===================================================================== */

/* =====================================================================
 *  流控参数定义 (Flow Control Parameters)
 *
 *  这些参数控制 AIMD 拥塞控制算法的行为。参数按跳数自适应:
 *  - 1 跳 (直连邻居): 使用较快参数，如 window=20, delay=8ms
 *  - 2 跳 (1 个中继): 使用中等保守参数，如 window=15, delay=10ms
 *  - 3+ 跳 (2+ 个中继): 使用最保守参数，如 window=12, delay=15ms
 *
 *  AIMD 算法核心:
 *  - 成功 (0% 丢包): 慢启动阶段窗口翻倍，拥塞避免阶段线性增加 (+ca_inc)
 *  - 轻微丢包 (<15%): 参数保持不变
 *  - 中度拥塞 (15%-40%): 窗口减半 (MD)，进入拥塞避免
 *  - 严重拥塞 (>40%): 窗口重置为最小，延时设为最大，重新慢启动
 *
 *  调优建议:
 *  - 如果传输速度慢: 增大 FC_W_INIT/FC_W_MAX，减小 FC_D_INIT
 *  - 如果丢包严重: 减小 FC_W_INIT，增大 FC_D_INIT，提高 FC_SS_INIT
 *  - 如果 CHKPT 超时频繁: 增大 FC_CHKPT_TIMEOUT_*HOP 和 FC_RTO_MIN_*HOP
 * ===================================================================== */
#define FC_W_INIT           30      /**< 初始窗口大小 (包数)。实际由 fc_start() 根据跳数动态设置:
                                         1跳=20, 2跳=15, 3+跳=12, 未知=10 */
#define FC_W_MIN            8       /**< 最小窗口大小 (包数)。拥塞严重时窗口不会低于此值 */
#define FC_W_MAX            80      /**< 最大窗口大小 (包数)。防止窗口无限增长淡没接收端 */
#define FC_D_INIT           5       /**< 初始包间延时 (ms)。实际由 fc_start() 动态设置:
                                         O8: 1跳=5ms, 2跳=7ms, 3+跳=10ms */
#define FC_D_MIN            0       /**< 最小包间延时 (ms)。0=无延时连发, 适合直连场景 */
#define FC_D_MAX            50      /**< 最大包间延时 (ms)。严重拥塞时延时不超过此值 */
#define FC_SS_INIT          60      /**< 初始慢启动阈值 (ssthresh)。实际由 fc_start() 动态设置:
                                         O8: 1跳=50, 2跳=30, 3+跳=30 */
#define FC_PKTS_PER_TICK    5       /**< O8: 每 tick 批量发送包数。3→5，配合 5ms tick 充分利用 SLE TX 缓冲区 */
#define FC_CHKPT_TIMEOUT_1HOP  1500  /**< O8: 1跳 CHECKPOINT 初始超时 (ms)。从 2500 降至 1500，
                                              配合 Turbo 7.5ms 间隔，实测 RTT 应 <500ms */
#define FC_CHKPT_TIMEOUT_2HOP  3000  /**< O8: 2跳 CHECKPOINT 初始超时 (ms)。从 5000 降至 3000 */
#define FC_CHKPT_TIMEOUT_3HOP  4000  /**< O8: 3+跳 CHECKPOINT 初始超时 (ms)。从 5000 降至 4000 */
#define FC_CHKPT_TIMEOUT_DEF   2500  /**< 路由未知时默认 CHECKPOINT 超时 (ms) */
#define FC_CHKPT_MAX_RETRY  2       /**< CHECKPOINT 最多重发次数。超过后视为严重拥塞，重发整段 */
#define FC_RTO_MIN          500     /**< 自适应超时全局下限 (ms)。具体按跳数见 fc_rto_min_for_hops() */
#define FC_RTO_MIN_1HOP     600     /**< O8: 1跳 RTO 下限 (ms)。从 1000 降至 600，配合 7.5ms Turbo */
#define FC_RTO_MIN_2HOP     1200    /**< O8: 2跳 RTO 下限 (ms)。从 2000 降至 1200 */
#define FC_RTO_MIN_3HOP     2000    /**< O8: 3+跳 RTO 下限 (ms)。从 3000 降至 2000 */
#define FC_RTO_MAX          10000   /**< 自适应超时上限 (ms)。防止指数退避后 RTO 无限增长 */
#define FC_SRTT_ALPHA_SHIFT  3      /**< SRTT 平滑因子 α = 1/8 (RFC 6298)。右移 3 位实现定点除法 */
#define FC_RTTVAR_BETA_SHIFT 2      /**< RTTVAR 平滑因子 β = 1/4 (RFC 6298)。右移 2 位实现定点除法 */
#define FC_RESULT_TIMEOUT_MS 10000  /**< END 发送后等待 RESULT/MISSING 最大超时 (ms) */
#define FC_RETX_DELAY_MS    8       /**< O8: 补包固定包间延时 (ms)。15→8，配合 Turbo 7.5ms 加快重传 */
#define FC_LOSS_NONE        0       /**< 丢包率阈值: 0% (无丢包，加速) */
#define FC_LOSS_LIGHT       15      /**< 丢包率阈值: <15% (轻微丢包，保持不变) */
#define FC_LOSS_MEDIUM      40      /**< 丢包率阈值: <40% (中度拥塞，乘性减半窗口) */

/**
 * @brief  F8: 根据跳数获取 RTO (Retransmission Timeout) 下限
 * @param  hops  目标节点距离网关的跳数
 * @return 对应跳数的 RTO 下限值 (ms)
 * @note   跳数越大，RTT 越长，RTO 下限也应越大，避免假超时导致不必要的重发
 */
static inline uint16_t fc_rto_min_for_hops(uint8_t hops)
{
    if (hops <= 1) return FC_RTO_MIN_1HOP;
    if (hops == 2) return FC_RTO_MIN_2HOP;
    return FC_RTO_MIN_3HOP;
}


/**
 * @brief 流控状态机状态枚举
 *
 * 定义流控引擎的所有可能状态。状态转换由 ble_gateway_img_tick() 驱动。
 * 参见上方状态转换图了解完整流程。
 */
typedef enum {
    FC_IDLE = 0,        /**< 空闲状态，无传输在进行 */
    FC_START_WAIT,      /**< 等待 START 帧在 Mesh 网络中传播 (延时 start_delay ms) */
    FC_SENDING,         /**< 正在发送当前段的数据包 (window 个包为一段) */
    FC_WAIT_CHKPT,      /**< 已发送 CHECKPOINT，等待目标节点的 CHECKPOINT_ACK (0x88) */
    FC_SEND_END,        /**< 所有段发完，发送 END 帧通知目标节点数据传输结束 */
    FC_WAIT_RESULT,     /**< 等待目标节点的 RESULT(0x86) 或 MISSING(0x87) 响应 */
    FC_RETRANSMIT,      /**< 根据 MISSING 位图补发缺失的数据包 */
} fc_state_t;

/**
 * @brief 流控引擎状态结构体
 *
 * 保存流控状态机的全部运行时状态，包括：
 * - AIMD 自适应参数 (window, delay, ssthresh, phase)
 * - 段管理 (seg_id, seg_start, next_seq)
 * - RTT 测量与自适应超时 (srtt, rttvar, chkpt_timeout)
 * - 滑动窗口流水线 (pipeline_seq)
 * - 补包位图与进度跟踪
 *
 * 全局单例 g_fc，同一时间只支持一个传输任务。
 */
static struct {
    fc_state_t state;           /**< 当前流控状态 */

    /* --- AIMD 自适应参数 --- */
    uint16_t window;            /**< 当前段大小/窗口大小 (包数)。每段发送 window 个包后发 CHECKPOINT */
    uint16_t delay_ms;          /**< 包间延时 (ms)。每发送一包后等待此时间再发下一包 */
    uint16_t chkpt_timeout;     /**< P2: 自适应 CHECKPOINT 超时 (ms)，由 RTT 测量动态调整 */
    uint16_t ssthresh;          /**< 慢启动阈值。窗口达到此值后从慢启动切换为拥塞避免 */
    uint8_t  phase;             /**< AIMD 阶段: 0=慢启动 (窗口翻倍), 1=拥塞避免 (窗口线性增) */

    /* --- 段管理 --- */
    uint8_t  seg_id;            /**< 当前段编号 (0 开始递增)，用于 CHECKPOINT 匹配 ACK */
    uint16_t seg_start;         /**< 当前段的起始包序号 (seq)，即本段第一个包的序号 */
    uint16_t next_seq;          /**< 下一个待发送包的 seq，范围 [seg_start, seg_start+window) */
    uint16_t prev_rx;           /**< 本段发送前目标节点已累计收到的包数，用于计算本段丢包率 */

    /* --- 时间戳 --- */
    uint32_t last_pkt_tick;     /**< 上一包发送时刻 (ms)，用于包间延时控制 */
    uint32_t state_tick;        /**< 当前状态进入时刻 (ms)，用于超时判断 */
    uint32_t chkpt_send_tick;   /**< CHECKPOINT 发送时刻 (ms)，用于 RTT 测量 */

    /* --- RTT 自适应超时 (单位 ms, 定点运算) --- */
    int32_t  srtt;              /**< 平滑 RTT (Smoothed RTT)。初始 -1 表示尚未测量，
                                     收到首个 CHKPT_ACK 后直接赋值，之后 EWMA 更新 */
    int32_t  rttvar;            /**< RTT 方差 (RTT Variance)，用于计算 RTO = SRTT + 4*RTTVAR */
    uint8_t  rtt_valid;         /**< 是否已有有效 RTT 样本: 0=未测量, 1=已有样本 */

    /* --- CHECKPOINT_ACK 结果 --- */
    uint8_t  ack_arrived;       /**< 是否收到当前 CHECKPOINT 的 ACK: 0=未收到, 1=已收到 */
    uint16_t ack_rx_count;      /**< 目标节点累计已收到的包数 (来自 ACK 报告) */
    uint8_t  chkpt_retry;       /**< 当前 CHECKPOINT 的重发次数 (最多 FC_CHKPT_MAX_RETRY) */

    /* --- O6: 动态 START 延时 --- */
    uint16_t start_delay;       /**< FC_START_WAIT 状态的等待时长 (ms)。根据跳数自适应:
                                     1跳=300ms, 2跳=500ms, 3+跳=800ms。
                                     多跳需更长时间让 Turbo 连接间隔变更传播到所有中继 */

    /* --- O1: 滑动窗口流水线 --- */
    uint16_t pipeline_seq;      /**< WAIT_CHKPT 期间预发到的最远 seq。
                                     在等待 ACK 期间预发下一段数据，消除空等时间 */

    /* --- 传输元信息 --- */
    uint8_t  transfer_hops;     /**< 传输开始时的跳数 (用于 O7 缓存保存，不受中途路由变化影响) */
    uint8_t  noprogress_count;  /**< 连续无进展 ACK 计数 (Fix B)。ACK 报告的 rx_count 连续未增加时累加，
                                     达到 3 次后视为真实丢包。未达到时用 2s 短超时快速重询 */

    /* --- 补包位图 --- */
    uint8_t  miss_bitmap[IMG_GW_BITMAP_BYTES]; /**< 缺包位图 (从目标节点 0x87 MISSING 复制)，
                                                    bit=1 表示对应 seq 的包缺失 */
    uint16_t miss_count;        /**< 缺失的总包数 */
    uint16_t miss_scan_pos;     /**< 位图扫描位置 (当前扫描到的 seq) */
    uint16_t retx_sent;         /**< 本轮补包已发送的包数 */
    uint8_t  retry_round;       /**< 补包轮次计数 (最多 IMG_GW_MAX_RETRY 轮) */

    /* --- 进度统计 --- */
    uint16_t total_injected;    /**< 网关已注入 Mesh 的总包数 (含所有重传)，用于统计日志 */

    /* --- v3: 组播支持 --- */
    uint8_t  mcast_missing_received; /**< 组播模式：是否有目标发送了 MISSING 位图 */
} g_fc = {0};

/* =====================================================================
 *  工具函数
 * ===================================================================== */

/**
 * @brief  将 16-bit UUID 转换为 SDK 所需的 bt_uuid_t 结构
 * @param  uuid16  16 位 UUID 值 (e.g. 0xFFE0)
 * @param  out     输出的 bt_uuid_t 结构指针
 */
static void uuid16_to_bt_uuid(uint16_t uuid16, bt_uuid_t *out)
{
    out->uuid_len = UART16_LEN;
    out->uuid[0] = (uint8_t)(uuid16 >> 8);
    out->uuid[1] = (uint8_t)(uuid16);
}

/**
 * @brief  判断指定地址是否为当前组播目标之一
 * @param  addr  待检查的 Mesh 地址
 * @return true=该地址在组播目标列表中, false=不在
 */
static bool is_mcast_target(uint16_t addr)
{
    if (!g_img_cache.is_multicast) return false;
    for (uint8_t i = 0; i < g_img_cache.target_count; i++) {
        if (g_img_cache.target_addrs[i] == addr) return true;
    }
    return false;
}

/**
 * @brief  向手机 APP 发送组播进度通知
 * @note   BLE 帧格式: AA 8A COMPLETED(1) TOTAL_TARGETS(1) LATEST_ADDR(2) LATEST_STATUS(1)
 *         APP 据此显示 "X/Y 节点已完成"
 */
static void mcast_notify_progress(uint16_t latest_addr, uint8_t latest_status)
{
    if (!g_gw_connected || g_gw_notify_handle == 0) return;

    uint8_t frame[7];
    frame[0] = 0xAA;
    frame[1] = 0x8A;  /* CMD_IMG_MCAST_PROGRESS */
    frame[2] = g_img_cache.completed_count;
    frame[3] = g_img_cache.target_count;
    frame[4] = (latest_addr >> 8) & 0xFF;
    frame[5] = latest_addr & 0xFF;
    frame[6] = latest_status;

    gatts_ntf_ind_t param = { 0 };
    param.attr_handle = g_gw_notify_handle;
    param.value_len = sizeof(frame);
    param.value = frame;
    gatts_notify_indicate(g_gw_server_id, g_gw_conn_id, &param);

    osal_printk("%s MCAST progress: %d/%d done, latest=0x%04X status=%d\r\n",
                BLE_GW_LOG, g_img_cache.completed_count, g_img_cache.target_count,
                latest_addr, latest_status);
}

/* =====================================================================
 *  BLE 广播模块
 *  构建广播数据、设置广播参数并启动广播。
 *  手机通过扫描广播名称 (sle_gw_XXXX) 发现网关并发起 BLE 连接。
 * ===================================================================== */
/**
 * @brief  构建 BLE 广播数据包
 * @param  buf      输出缓冲区
 * @param  max_len  缓冲区最大长度
 * @return 广播数据实际长度 (字节)，失败返回 0
 * @note   包含标准 Flags (0x06) 和完整本地名称 (0x09)
 */
static uint16_t gw_build_adv_data(uint8_t *buf, uint8_t max_len)
{
    uint8_t idx = 0;
    buf[idx++] = 0x02;   /* Flags 字段长度 */
    buf[idx++] = 0x01;   /* AD Type: Flags */
    buf[idx++] = 0x06;   /* Flags: LE General Discoverable + BR/EDR Not Supported */
    buf[idx++] = g_gw_adv_name_len + 1;
    buf[idx++] = 0x09;   /* AD Type: Complete Local Name */
    if (memcpy_s(&buf[idx], max_len - idx, g_gw_adv_name, g_gw_adv_name_len) != EOK) {
        return 0;
    }
    idx += g_gw_adv_name_len;
    return idx;
}

/**
 * @brief 设置 BLE 广播数据和扫描响应数据
 * @note  广播数据和扫描响应使用相同内容，包含设备名称
 */
static void gw_set_adv_data(void)
{
    uint8_t adv_data[EXT_ADV_DATA_LEN] = { 0 };
    uint8_t scan_rsp[EXT_ADV_DATA_LEN] = { 0 };
    gap_ble_config_adv_data_t cfg = { 0 };
    uint16_t adv_len = gw_build_adv_data(adv_data, EXT_ADV_DATA_LEN);
    uint16_t rsp_len = gw_build_adv_data(scan_rsp, EXT_ADV_DATA_LEN);
    cfg.adv_data = adv_data;
    cfg.adv_length = adv_len;
    cfg.scan_rsp_data = scan_rsp;
    cfg.scan_rsp_length = rsp_len;
    gap_ble_set_adv_data(BLE_GW_ADV_HANDLE, &cfg);
}

/**
 * @brief 配置广播参数并启动 BLE 广播
 * @note  广播类型为可连接无定向广播 (ADV_IND)，duration=0 表示持续广播直到被连接。
 *        channel_map=0x07 表示在所有 3 个广播信道 (37/38/39) 上广播。
 */
static void gw_start_adv(void)
{
    gap_ble_adv_params_t params = { 0 };
    params.min_interval = BLE_GW_ADV_MIN_INTERVAL;
    params.max_interval = BLE_GW_ADV_MAX_INTERVAL;
    params.duration = 0;
    params.channel_map = 0x07;
    params.adv_type = 0x00;
    params.adv_filter_policy = 0x00;
    params.peer_addr.type = 0;
    (void)memset_s(params.peer_addr.addr, BD_ADDR_LEN_LOCAL, 0, BD_ADDR_LEN_LOCAL);
    gap_ble_set_adv_param(BLE_GW_ADV_HANDLE, &params);
    gap_ble_start_adv(BLE_GW_ADV_HANDLE);
    osal_printk("%s BLE advertising started: %s\r\n", BLE_GW_LOG, g_gw_adv_name);
}

/* =====================================================================
 *  GATT 服务回调函数
 *  处理 GATT 服务添加、特征添加、服务启动等事件。
 *  服务结构: 1 个服务 (0xFFE0) + 2 个特征 (TX: 0xFFE1 Notify, RX: 0xFFE2 Write)
 *  + 1 个 CCCD 描述符 (0x2902，订阅/取消 Notify)
 * ===================================================================== */

/**
 * @brief  GATT 服务添加完成回调
 * @param  server_id  GATT Server ID
 * @param  uuid       服务 UUID
 * @param  handle     服务句柄
 * @param  status     操作结果状态码
 * @note   在服务添加成功后，继续添加 TX/RX 特征和 CCCD 描述符，然后启动服务
 */
static void gw_service_add_cbk(uint8_t server_id, bt_uuid_t *uuid,
                                uint16_t handle, errcode_t status)
{
    osal_printk("%s service added srv_id=%d handle=%d status=%d\r\n",
                BLE_GW_LOG, server_id, handle, status);

    bt_uuid_t tx_uuid = { 0 };
    uuid16_to_bt_uuid(BLE_GW_UUID_TX, &tx_uuid);
    uint8_t tx_val[] = { 0x00 };
    gatts_add_chara_info_t tx_char = { 0 };
    tx_char.chara_uuid = tx_uuid;
    tx_char.properties = GATT_CHARACTER_PROPERTY_BIT_NOTIFY | GATT_CHARACTER_PROPERTY_BIT_READ;
    tx_char.permissions = GATT_ATTRIBUTE_PERMISSION_READ;
    tx_char.value_len = sizeof(tx_val);
    tx_char.value = tx_val;
    gatts_add_characteristic(server_id, handle, &tx_char);

    bt_uuid_t cccd_uuid = { 0 };
    uuid16_to_bt_uuid(0x2902, &cccd_uuid);
    uint8_t cccd_val[2] = { 0x00, 0x00 };
    gatts_add_desc_info_t cccd_desc = { 0 };
    cccd_desc.desc_uuid = cccd_uuid;
    cccd_desc.permissions = GATT_ATTRIBUTE_PERMISSION_READ | GATT_ATTRIBUTE_PERMISSION_WRITE;
    cccd_desc.value_len = sizeof(cccd_val);
    cccd_desc.value = cccd_val;
    gatts_add_descriptor(server_id, handle, &cccd_desc);

    bt_uuid_t rx_uuid = { 0 };
    uuid16_to_bt_uuid(BLE_GW_UUID_RX, &rx_uuid);
    uint8_t rx_val[] = { 0x00 };
    gatts_add_chara_info_t rx_char = { 0 };
    rx_char.chara_uuid = rx_uuid;
    rx_char.properties = GATT_CHARACTER_PROPERTY_BIT_WRITE |
                         GATT_CHARACTER_PROPERTY_BIT_WRITE_NO_RSP;
    rx_char.permissions = GATT_ATTRIBUTE_PERMISSION_READ |
                          GATT_ATTRIBUTE_PERMISSION_WRITE;
    rx_char.value_len = sizeof(rx_val);
    rx_char.value = rx_val;
    gatts_add_characteristic(server_id, handle, &rx_char);

    gatts_start_service(server_id, handle);
}

/**
 * @brief  GATT 特征添加完成回调
 * @param  server_id       GATT Server ID
 * @param  uuid            特征 UUID
 * @param  service_handle  所属服务句柄
 * @param  result          添加结果 (包含 value_handle)
 * @param  status          操作结果状态码
 * @note   如果是 TX 特征 (0xFFE1)，保存其 value_handle 用于后续 Notify 发送
 */
static void gw_char_add_cbk(uint8_t server_id, bt_uuid_t *uuid,
                              uint16_t service_handle,
                              gatts_add_character_result_t *result,
                              errcode_t status)
{
    bt_uuid_t tx_uuid = { 0 };
    uuid16_to_bt_uuid(BLE_GW_UUID_TX, &tx_uuid);
    if (uuid->uuid_len == tx_uuid.uuid_len &&
        memcmp(uuid->uuid, tx_uuid.uuid, uuid->uuid_len) == 0) {
        g_gw_notify_handle = result->value_handle;
        osal_printk("%s TX notify handle = %d\r\n", BLE_GW_LOG, g_gw_notify_handle);
    }
    (void)server_id;
    (void)service_handle;
    (void)status;
}

/**
 * @brief GATT 描述符添加完成回调 (当前无额外处理)
 */
static void gw_desc_add_cbk(uint8_t server_id, bt_uuid_t *uuid,
                              uint16_t service_handle, uint16_t handle,
                              errcode_t status)
{
    (void)server_id; (void)uuid; (void)service_handle; (void)handle; (void)status;
}

/**
 * @brief  GATT 服务启动完成回调
 * @param  server_id  GATT Server ID
 * @param  handle     服务句柄
 * @param  status     操作结果状态码
 * @note   所有服务 (BLE_GW_SERVICE_NUM 个) 都启动完成后，设置广播数据并启动广播
 */
static void gw_service_start_cbk(uint8_t server_id, uint16_t handle, errcode_t status)
{
    g_gw_service_start_count++;
    osal_printk("%s service started #%d (srv=%d handle=%d status=%d)\r\n",
                BLE_GW_LOG, g_gw_service_start_count, server_id, handle, status);
    if (g_gw_service_start_count >= BLE_GW_SERVICE_NUM && status == 0) {
        osal_printk("%s all services ready, starting adv\r\n", BLE_GW_LOG);
        gw_set_adv_data();
        gw_start_adv();
    }
}

/* =====================================================================
 *  全网拓扑查询模块
 *  提供拓扑收集、响应处理、结果上报、超时检测等功能。
 *  工作流程: 网关广播 TOPO_REQ → 各节点回复 TOPO_RESP → 网关汇总后 BLE Notify 给 APP
 * ===================================================================== */

/**
 * @brief  向拓扑收集列表中添加节点 (去重 + 取最小跳数)
 * @param  addr  节点 Mesh 地址
 * @param  hop   该节点到网关的跳数
 */
static void topo_collect_add(uint16_t addr, uint8_t hop)
{
    if (addr == 0 || addr == 0xFFFF) return;
    for (uint8_t i = 0; i < g_topo_collect.count; i++) {
        if (g_topo_collect.nodes[i] == addr) {
            if (hop < g_topo_collect.hops[i]) g_topo_collect.hops[i] = hop;
            return;
        }
    }
    if (g_topo_collect.count < TOPO_MAX_NODES) {
        g_topo_collect.nodes[g_topo_collect.count] = addr;
        g_topo_collect.hops[g_topo_collect.count] = hop;
        g_topo_collect.count++;
    }
}

/**
 * @brief 将拓扑收集结果通过 BLE Notify 发送给手机 APP
 * @note  帧格式: AA 83 SELF_ADDR(2) NODE_COUNT(1) [ADDR(2) HOP(1)]...
 */
static void topo_collect_send_result(void)
{
    uint16_t self_addr = mesh_get_my_addr();
    uint8_t frame[200];
    frame[0] = 0xAA;
    frame[1] = 0x83;
    frame[2] = (self_addr >> 8) & 0xFF;
    frame[3] = self_addr & 0xFF;
    uint16_t pos = 5;
    uint8_t total_nodes = 0;
    for (uint8_t i = 0; i < g_topo_collect.count && pos + 3 <= (uint16_t)sizeof(frame); i++) {
        if (g_topo_collect.nodes[i] == self_addr) continue;
        frame[pos++] = (g_topo_collect.nodes[i] >> 8) & 0xFF;
        frame[pos++] = g_topo_collect.nodes[i] & 0xFF;
        frame[pos++] = g_topo_collect.hops[i];
        total_nodes++;
    }
    frame[4] = total_nodes;
    if (g_gw_connected && g_gw_notify_handle != 0) {
        gatts_ntf_ind_t param = { 0 };
        param.attr_handle = g_gw_notify_handle;
        param.value_len = pos;
        param.value = frame;
        gatts_notify_indicate(g_gw_server_id, g_gw_conn_id, &param);
    }
    osal_printk("%s topology: self=0x%04X, nodes=%d\r\n", BLE_GW_LOG, self_addr, total_nodes);
}

/**
 * @brief 处理手机 APP 的拓扑查询请求 (cmd=0x03)
 * @note  收集本地邻居 + 路由表信息，广播 TOPO_REQ，等待响应后汇总上报
 */
static void gw_handle_topology_query(void)
{
    if (g_topo_collect.active) return;
    g_topo_collect.count = 0;
    uint16_t nbr_addrs[16];
    uint8_t nbr_count = mesh_transport_get_all_neighbor_addrs(nbr_addrs, 16);
    for (uint8_t i = 0; i < nbr_count; i++) topo_collect_add(nbr_addrs[i], 1);
    uint16_t rt_addrs[32];
    uint8_t  rt_hops[32];
    uint8_t  rt_count = mesh_route_get_all_destinations(rt_addrs, rt_hops, 32);
    for (uint8_t i = 0; i < rt_count; i++) topo_collect_add(rt_addrs[i], rt_hops[i]);

    /* F17: TOPO_REQ 中携带网关地址，允许远端节点单播回复。
     * 格式: [0xFE, 0x01, GW_ADDR_HI, GW_ADDR_LO]
     * 旧格式: [0xFE, 0x01] (2字节)  新格式: [0xFE, 0x01, HI, LO] (4字节) */
    uint16_t self_addr = mesh_get_my_addr();
    uint8_t topo_req[4] = { TOPO_MAGIC, TOPO_REQ_CMD,
                            (uint8_t)(self_addr >> 8), (uint8_t)(self_addr & 0xFF) };
    mesh_broadcast(topo_req, sizeof(topo_req));
    g_topo_collect.active = 1;
    g_topo_collect.start_time = osal_get_tick_ms();
    g_topo_collect.last_resp_tick = 0;  /* O11: 尚无响应 */
    g_topo_collect.resp_count = 0;
    g_topo_collect.retry_done = 0;
    osal_printk("%s topo REQ broadcast, pre-discovered %d nodes, nbrs=%d, conns=%d\r\n",
                BLE_GW_LOG, g_topo_collect.count, nbr_count,
                mesh_transport_get_neighbor_count());
}

/**
 * @brief  处理其他节点的拓扑响应 (TOPO_RESP)
 * @param  data  响应帧数据: [TOPO_MAGIC, TOPO_RESP_CMD, ADDR_HI, ADDR_LO, NBR_CNT, NBR_ADDR(2)...]
 * @param  len   数据长度
 * @note   不提前结束收集, 始终等待 TOPO_COLLECT_TIMEOUT_MS 超时.
 *         因为 TOPO_REQ 洪泛全网, 所有节点(不仅直连邻居)都会响应,
 *         提前结束会丢失多跳节点的响应.
 */
void ble_gateway_on_topo_resp(const uint8_t *data, uint16_t len)
{
    if (!g_topo_collect.active) return;
    if (len < 5) return;
    uint16_t resp_addr = ((uint16_t)data[2] << 8) | data[3];
    uint8_t  nbr_cnt   = data[4];

    /* 使用路由表/邻居表查询响应者的实际跳数, 而不是硬编码 2 */
    uint8_t resp_hop = mesh_route_get_hop_count(resp_addr);
    if (resp_hop == 0xFF) {
        resp_hop = 2; /* 路由表中无记录时保守估计 */
    }
    topo_collect_add(resp_addr, resp_hop);

    uint16_t idx = 5;
    for (uint8_t i = 0; i < nbr_cnt && idx + 1 < len; i++) {
        uint16_t nbr = ((uint16_t)data[idx] << 8) | data[idx + 1];
        topo_collect_add(nbr, resp_hop + 1); /* 响应者的邻居 = 响应者距离 + 1 */
        idx += 2;
    }

    /* 响应去重记录 (仅用于日志统计) */
    uint8_t already = 0;
    for (uint8_t i = 0; i < g_topo_collect.resp_count; i++) {
        if (g_topo_collect.resp_src[i] == resp_addr) { already = 1; break; }
    }
    if (!already && g_topo_collect.resp_count < TOPO_MAX_NODES) {
        g_topo_collect.resp_src[g_topo_collect.resp_count++] = resp_addr;
    }
    g_topo_collect.last_resp_tick = osal_get_tick_ms();  /* O11: 记录最后响应时间 */
    /* 不提前结束: TOPO_REQ 洪泛全网, 全网节点都会响应 (not just direct neighbors).
     * 基于 expected_resps = nbr_count 的早期终止会在直连邻居响应到达后
     * 立即结束, 丢失 2+ 跳节点的 TOPO_RESP.
     * 改为始终等待超时 (TOPO_COLLECT_TIMEOUT_MS), 确保全网响应有足够时间到达. */
}

/**
 * @brief 拓扑收集超时检测，在主循环中周期性调用
 * @note  超过 TOPO_COLLECT_TIMEOUT_MS 后强制汇总已收集的结果并上报。
 *        F17: 当首次超时结果为 0 节点但连接池非空时，自动重试一次。
 *        这解决了以下场景: 首次 TOPO_REQ 因 mesh_addr 未填充或 SLE 发送
 *        暂时失败导致无响应，短暂延迟后连接已稳定可重试。
 */
void ble_gateway_topo_tick(void)
{
    if (!g_topo_collect.active) return;
    uint32_t elapsed = osal_get_tick_ms() - g_topo_collect.start_time;
    /* O11: 静默期早期终止 —— 已有响应且超过 1s 后 500ms 无新响应，视为全网已回复
     * 避免始终等满 3000ms 浪费时间；多跳节点通常在 1s 内即可响应 */
    if (g_topo_collect.resp_count > 0 && elapsed >= 1000 &&
        g_topo_collect.last_resp_tick > 0 &&
        (osal_get_tick_ms() - g_topo_collect.last_resp_tick) >= 500) {
        osal_printk("%s topo early finish: %d RESPs, quiet 500ms\r\n",
                    BLE_GW_LOG, g_topo_collect.resp_count);
        g_topo_collect.active = 0;
        topo_collect_send_result();
        return;
    }
    if (elapsed >= TOPO_COLLECT_TIMEOUT_MS) {
        /* F17: 自动重试 — 首次 0 结果且有活跃连接时再试一次 */
        if (g_topo_collect.count == 0 &&
            !g_topo_collect.retry_done &&
            mesh_transport_get_neighbor_count() > 0) {
            osal_printk("%s topo retry: 0 nodes but %d conns, retrying...\r\n",
                        BLE_GW_LOG, mesh_transport_get_neighbor_count());
            g_topo_collect.retry_done = 1;
            /* 重新预采集（mesh_addr 可能已通过 F17 快速填充） */
            uint16_t nbr_addrs[16];
            uint8_t nbr_count = mesh_transport_get_all_neighbor_addrs(nbr_addrs, 16);
            for (uint8_t i = 0; i < nbr_count; i++) topo_collect_add(nbr_addrs[i], 1);
            uint16_t rt_addrs[32];
            uint8_t rt_hops[32];
            uint8_t rt_count = mesh_route_get_all_destinations(rt_addrs, rt_hops, 32);
            for (uint8_t i = 0; i < rt_count; i++) topo_collect_add(rt_addrs[i], rt_hops[i]);
            /* 重新广播 TOPO_REQ */
            uint16_t self_addr = mesh_get_my_addr();
            uint8_t topo_req[4] = { 0xFE, 0x01,
                                    (uint8_t)(self_addr >> 8), (uint8_t)(self_addr & 0xFF) };
            mesh_broadcast(topo_req, sizeof(topo_req));
            g_topo_collect.start_time = osal_get_tick_ms();
            osal_printk("%s topo retry broadcast, pre-discovered %d nodes\r\n",
                        BLE_GW_LOG, g_topo_collect.count);
            return;
        }
        osal_printk("%s topo timeout: %d RESPs, %d nodes%s\r\n",
                    BLE_GW_LOG, g_topo_collect.resp_count, g_topo_collect.count,
                    g_topo_collect.retry_done ? " (after retry)" : "");
        g_topo_collect.active = 0;
        topo_collect_send_result();
    }
}

/* ════════════════════════════════════════════════════════════
 *  v2: 流控数据注入引擎
 *  本模块包含流控的所有核心函数:
 *  - fc_inject_data_pkt():  从缓存注入单个数据包到 Mesh
 *  - fc_send_checkpoint():  发送 CHECKPOINT 确认帧到目标节点
 *  - fc_send_end():         发送 END 结束帧到目标节点
 *  - fc_stop_turbo():       关闭 Turbo 模式恢复正常连接间隔
 *  - fc_notify_progress():  向 APP 发送传输进度通知
 *  - fc_adjust():           AIMD 拥塞控制算法 —— 根据丢包率调整窗口和延时
 *  - fc_start():            初始化并启动流控引擎
 * ════════════════════════════════════════════════════════════ */

/**
 * @brief  从网关缓存中提取指定 seq 的数据包并注入 Mesh 网络
 * @param  seq  包序号 (0 开始，对应 FC 大包分包)
 * @note   O4 优化: 使用 IMG_FC_PKT_PAYLOAD 大包负载 + 16-bit 长度字段。
 *         Mesh 负载格式: [0x05(IMG_CMD_DATA), SEQ_HI, SEQ_LO, LEN_HI, LEN_LO, DATA...]
 */
static void fc_inject_data_pkt(uint16_t seq)
{
    if (seq >= g_img_cache.fc_pkt_count) return;

    uint32_t offset = (uint32_t)seq * IMG_FC_PKT_PAYLOAD;
    if (offset >= g_img_cache.total_bytes) return;

    uint16_t remaining = g_img_cache.total_bytes - (uint16_t)offset;
    uint16_t pkt_len = (remaining >= IMG_FC_PKT_PAYLOAD) ?
                       IMG_FC_PKT_PAYLOAD : remaining;

    uint8_t mesh_buf[5 + IMG_FC_PKT_PAYLOAD];
    mesh_buf[0] = 0x05;  /* IMG_CMD_DATA: 图片数据包命令码 */
    mesh_buf[1] = (seq >> 8) & 0xFF;        /* 序号高字节 */
    mesh_buf[2] = seq & 0xFF;               /* 序号低字节 */
    mesh_buf[3] = (pkt_len >> 8) & 0xFF;    /* O4: 16-bit 长度高字节 */
    mesh_buf[4] = pkt_len & 0xFF;           /* O4: 16-bit 长度低字节 */
    (void)memcpy_s(&mesh_buf[5], sizeof(mesh_buf) - 5,
                   &g_img_cache.buf[offset], pkt_len);

    /* v3: 组播模式使用广播发送，单播使用定向注入 */
    if (g_img_cache.is_multicast) {
        mesh_broadcast(mesh_buf, 5 + pkt_len);
    } else {
        mesh_gateway_inject(g_img_cache.dst_addr, mesh_buf, 5 + pkt_len);
    }
    g_fc.total_injected++;
}

/**
 * @brief  发送 CHECKPOINT 帧到目标节点，请求报告已接收包数
 * @note   Mesh 负载: [0x08(IMG_CMD_CHECKPOINT), SEG_ID]
 *         发送时记录 chkpt_send_tick 用于 RTT 测量
 */
static void fc_send_checkpoint(void)
{
    uint8_t mesh_buf[2];
    mesh_buf[0] = 0x08;  /* IMG_CMD_CHECKPOINT: 节点收到后回复已接收包数 */
    mesh_buf[1] = g_fc.seg_id;
    /* v3: 组播模式广播 CHECKPOINT，单播定向发送 */
    if (g_img_cache.is_multicast) {
        mesh_broadcast(mesh_buf, 2);
    } else {
        mesh_gateway_inject(g_img_cache.dst_addr, mesh_buf, 2);
    }
    g_fc.chkpt_send_tick = osal_get_tick_ms();  /* 记录发送时刻，用于计算 RTT 样本 */
    osal_printk("%s FC CHKPT seg=%d sent (W=%d D=%d TO=%d)%s\r\n",
                BLE_GW_LOG, g_fc.seg_id, g_fc.window, g_fc.delay_ms,
                g_fc.chkpt_timeout, g_img_cache.is_multicast ? " [MCAST]" : "");
}

/**
 * @brief 发送 END 帧到目标节点，通知数据传输结束
 * @note  Mesh 负载: [0x06(IMG_CMD_END), CRC16_HI, CRC16_LO]
 *        目标节点收到后验证 CRC，回复 RESULT(0x86) 或 MISSING(0x87) 位图
 */
static void fc_send_end(void)
{
    uint8_t mesh_buf[3];
    mesh_buf[0] = 0x06;  /* IMG_CMD_END: 图片传输结束命令 */
    mesh_buf[1] = (g_img_cache.crc16 >> 8) & 0xFF;
    mesh_buf[2] = g_img_cache.crc16 & 0xFF;
    /* v3: 组播模式广播 END，单播定向发送 */
    if (g_img_cache.is_multicast) {
        mesh_broadcast(mesh_buf, 3);
    } else {
        mesh_gateway_inject(g_img_cache.dst_addr, mesh_buf, 3);
    }
    osal_printk("%s FC END sent crc=0x%04X%s\r\n", BLE_GW_LOG, g_img_cache.crc16,
                g_img_cache.is_multicast ? " [MCAST]" : "");
}

/**
 * @brief  F19: Turbo 控制帧全局序列号
 * @note   每次发送 TURBO ON/OFF 广播时递增，中继节点根据序列号过滤过时帧，
 *         防止旧 TURBO OFF 覆盖新 TURBO ON 导致传输速度骤降。
 */
static uint16_t g_turbo_seq = 0;

/** @brief  O9: 延迟 TURBO OFF 广播的调度时间戳 (0=无待发) */
static uint32_t g_turbo_off_scheduled_tick = 0;
#define TURBO_OFF_COOLDOWN_MS  1000  /* O9: TURBO OFF 冷却期 (ms)，避免连续传输间的快速 ON/OFF 切换 */

/**
 * @brief  O2: 关闭 Turbo 模式，恢复正常 SLE 连接间隔
 * @note   F18: 自发自收 (hops=0) 未开启 Turbo，直接跳过
 *         O9:  正常传输完成时延迟广播 TURBO OFF，给后续传输留出冷却期
 *         紧急路径 (断连/取消/超限) 仍由 fc_stop_turbo_now() 立即关闭
 */
static void fc_stop_turbo(void)
{
    /* F18: 自发自收时未开启 Turbo，无需关闭 */
    if (g_fc.transfer_hops == 0) {
        return;
    }
    /* O9: 本地立即恢复正常间隔，广播 OFF 延迟发出 */
    mesh_transport_set_turbo_mode(false);
    g_turbo_off_scheduled_tick = osal_get_tick_ms();
    if (g_turbo_off_scheduled_tick == 0) g_turbo_off_scheduled_tick = 1; /* 避免 0 误判为"无待发" */
}

/**
 * @brief  紧急关闭 Turbo: 断连/取消/超限时立即广播 TURBO OFF
 */
static void fc_stop_turbo_now(void)
{
    g_turbo_off_scheduled_tick = 0;  /* O9: 取消任何待发的延迟 OFF */
    if (g_fc.transfer_hops == 0) {
        return;  /* F18: 自发自收未开启 Turbo */
    }
    mesh_transport_set_turbo_mode(false);
    g_turbo_seq++;
    uint8_t cmd[4] = { MESH_TURBO_MAGIC, MESH_TURBO_OFF,
                        (uint8_t)(g_turbo_seq >> 8), (uint8_t)(g_turbo_seq & 0xFF) };
    mesh_gateway_inject(0xFFFF, cmd, sizeof(cmd));
    osal_printk("%s O2 turbo OFF (seq=%d)\r\n", BLE_GW_LOG, g_turbo_seq);
}

/**
 * @brief  发送传输进度通知给手机 APP
 * @param  phase     传输阶段: 0=首轮发送, 1=补包阶段
 * @param  rx_count  目标节点已收到的包数
 * @note   BLE 帧格式: AA 89 DST_ADDR(2) PHASE(1) RX_COUNT(2) TOTAL(2)
 *         APP 可据此显示进度条
 */
static void fc_notify_progress(uint8_t phase, uint16_t rx_count)
{
    if (!g_gw_connected || g_gw_notify_handle == 0) return;
    /* v3: 组播模式不发送逐包进度 (0x89)，改用 0x8A 节点完成进度 */
    if (g_img_cache.is_multicast) return;

    uint8_t frame[9];
    frame[0] = 0xAA;
    frame[1] = 0x89;  /* CMD_IMG_PROGRESS: 图片传输进度命令码 */
    frame[2] = (g_img_cache.dst_addr >> 8) & 0xFF;
    frame[3] = g_img_cache.dst_addr & 0xFF;
    frame[4] = phase;  /* 0=首轮发送阶段, 1=补包阶段 */
    frame[5] = (rx_count >> 8) & 0xFF;
    frame[6] = rx_count & 0xFF;
    frame[7] = (g_img_cache.fc_pkt_count >> 8) & 0xFF;
    frame[8] = g_img_cache.fc_pkt_count & 0xFF;

    gatts_ntf_ind_t param = { 0 };
    param.attr_handle = g_gw_notify_handle;
    param.value_len = sizeof(frame);
    param.value = frame;
    gatts_notify_indicate(g_gw_server_id, g_gw_conn_id, &param);
}

/**
 * @brief  根据丢包率调整流控窗口和延时 (AIMD 拥塞控制算法)
 *
 * @param  seg_sent  本段发送的总包数
 * @param  seg_rx    本段目标节点实际收到的包数 (从 CHECKPOINT_ACK 计算)
 *
 * @note   AIMD (Additive Increase / Multiplicative Decrease) 策略:
 *         - 0% 丢包: 慢启动阶段窗口翻倍，拥塞避免阶段线性增加 (ca_inc 按跳数缩放)
 *         - <15% 丢包: 保持参数不变 (容忍轻微丢包)
 *         - 15%-40% 丢包: 窗口减半 (MD)，延时 +10ms，切换到拥塞避免
 *         - >40% 丢包: 窗口重置为 FC_W_MIN，延时重置为 FC_D_MAX，回到慢启动
 */
static void fc_adjust(uint16_t seg_sent, uint16_t seg_rx)
{
    if (seg_sent == 0) return;

    uint16_t loss_pct = ((seg_sent - seg_rx) * 100) / seg_sent;

    osal_printk("%s FC adjust: sent=%d rx=%d loss=%d%% W=%d D=%d phase=%d\r\n",
                BLE_GW_LOG, seg_sent, seg_rx, loss_pct,
                g_fc.window, g_fc.delay_ms, g_fc.phase);

    if (loss_pct == FC_LOSS_NONE) {
        /* 完美传输 (0% 丢包): 加速 —— 增大窗口，降低延时 */
        if (g_fc.phase == 0) {
            /* 慢启动阶段: 窗口翻倍，直到达到 ssthresh 后转入拥塞避免 */
            g_fc.window = g_fc.window * 2;
            if (g_fc.window >= g_fc.ssthresh) {
                g_fc.window = g_fc.ssthresh;
                g_fc.phase = 1;  /* 达到慢启动阈值，转入拥塞避免阶段 */
            }
        } else {
            /* O5+F10: 拥塞避免阶段: 窗口线性增加，增量按跳数缩放
             * 1跳增 8, 2跳增 5, 3+跳增 3 —— 跳数越多増长越保守 */
            uint16_t ca_inc = (g_fc.transfer_hops <= 1) ? 8 :
                              (g_fc.transfer_hops == 2) ? 5 : 3;
            g_fc.window += ca_inc;
        }
        /* 降低包间延时 (每次减 2ms，空间充足时加速发送) */
        if (g_fc.delay_ms > FC_D_MIN + 2) {
            g_fc.delay_ms -= 2;
        } else {
            g_fc.delay_ms = FC_D_MIN;
        }
    } else if (loss_pct < FC_LOSS_LIGHT) {
        /* 轻微丢包 (<15%): 网络状态还行，维持当前参数不变 */
        /* 不做任何调整 */
    } else if (loss_pct < FC_LOSS_MEDIUM) {
        /* 中度拥塞 (15%-40%): 乘性减半窗口 (Multiplicative Decrease) */
        g_fc.ssthresh = g_fc.window / 2;
        if (g_fc.ssthresh < FC_W_MIN) g_fc.ssthresh = FC_W_MIN;
        g_fc.window = g_fc.window / 2;
        g_fc.delay_ms += 10;     /* 增加包间延时 10ms，降低发送速率 */
        g_fc.phase = 1;  /* 切换到拥塞避免阶段 */
    } else {
        /* 严重拥塞 (>40%): 重置为最保守参数，重新开始慢启动 */
        g_fc.ssthresh = g_fc.window / 2;
        if (g_fc.ssthresh < FC_W_MIN) g_fc.ssthresh = FC_W_MIN;
        g_fc.window = FC_W_MIN;
        g_fc.delay_ms = FC_D_MAX;
        g_fc.phase = 0;  /* 回到慢启动阶段，窗口从最小值重新开始增长 */
    }

    /* 参数边界检查: 确保不超出安全范围 */
    if (g_fc.window < FC_W_MIN) g_fc.window = FC_W_MIN;
    if (g_fc.window > FC_W_MAX) g_fc.window = FC_W_MAX;
    if (g_fc.delay_ms > FC_D_MAX) g_fc.delay_ms = FC_D_MAX;

    osal_printk("%s FC adjusted: W=%d D=%d ss=%d phase=%d\r\n",
                BLE_GW_LOG, g_fc.window, g_fc.delay_ms,
                g_fc.ssthresh, g_fc.phase);
}

/**
 * @brief  初始化并启动流控注入引擎
 *
 * 在收到 APP 的 END 帧后调用，FAST 模式和 ACK 代理模式均使用此函数。
 * 核心逻辑:
 *   1. 查询目标节点的跳数 (hop count)
 *   2. 根据跳数自适应设置初始流控参数 (P0 优化)
 *   3. 检测 P3 fallback 场景 (远距离弱信号直连)
 *   4. 启动 Turbo 模式 (O2) 并广播通知中继节点
 *   5. 暂停 SLE 扫描 (P3) 减少射频干扰
 *
 * 跳数对应的初始参数:
 *   0跳 (自发自收): window=全部包, delay=0, ssthresh=99
 *   1跳 (直连):   window=20, delay=8ms, ssthresh=40, timeout=2500ms
 *   2跳 (1个中继): window=15, delay=10ms, ssthresh=20, timeout=5000ms
 *   3+跳 (2+个中继): window=12, delay=15ms, ssthresh=30, timeout=5000ms
 *   未知路由:   window=10, delay=10ms, ssthresh=20, timeout=3000ms
 *   P3 fallback: window=10, delay=30ms, ssthresh=10, timeout=5000ms
 *
 * @note   调用此函数后状态机进入 FC_START_WAIT，等待 start_delay ms 后切换到 FC_SENDING
 */
static void fc_start(void)
{
    (void)memset_s(&g_fc, sizeof(g_fc), 0, sizeof(g_fc));

    /* ---- v3: 组播模式使用保守参数 (广播到多节点，无法精确测跳数) ---- */
    uint8_t hops;
    uint16_t w_init;
    uint16_t d_init;
    uint16_t ss_init;
    uint16_t chkpt_to;

    if (g_img_cache.is_multicast) {
        /* O8: 组播: 使用适中参数 (广播到多节点，Turbo 7.5ms 允许更快) */
        hops = 2;
        w_init = 15;
        d_init = 8;
        ss_init = 25;
        chkpt_to = FC_CHKPT_TIMEOUT_3HOP;
        osal_printk("%s FC MCAST mode: %d targets, using conservative params\r\n",
                    BLE_GW_LOG, g_img_cache.target_count);
    } else {
    /* ---- P0: 根据目标节点跳数自适应初始参数 (单播) ---- */
    hops = mesh_route_get_hop_count(g_img_cache.dst_addr);

    /* F13+F20: 自发自收检测 —— 仅当目标地址等于本节点时才走本地快速路径
     * F20: 移除 hops==0xFF 条件，避免路由丢失时误判远端节点为自发自收 */
    uint16_t my_addr = mesh_get_my_addr();
    if (g_img_cache.dst_addr == my_addr) {
        hops = 0;
        w_init = g_img_cache.fc_pkt_count;  /* 本地处理，一次发完所有包 */
        d_init = 0;
        ss_init = 99;
        chkpt_to = 2000;
        osal_printk("%s FC self-send detected (dst=0x%04X), using fast local params\r\n",
                    BLE_GW_LOG, g_img_cache.dst_addr);
    } else if (hops == 1) {
        /* 直连邻居 —— 需检测 P3 fallback (远距离弱信号直连，应用多跳参数) */
        uint8_t table_hops = mesh_route_get_table_hop_count(g_img_cache.dst_addr);
        bool is_p3 = mesh_transport_is_p3_neighbor(g_img_cache.dst_addr);
        uint8_t p3_hops = is_p3 ? mesh_transport_get_p3_original_hops(g_img_cache.dst_addr) : 0;

        if (table_hops >= 2 && table_hops != 0xFF) {
            /* P3 fallback: 路由表仍显示多跳 (首次检测到直连但路由未更新的情况) */
            hops = table_hops;
            w_init = 10;  d_init = 30;  ss_init = 10;
            chkpt_to = FC_CHKPT_TIMEOUT_3HOP;
            osal_printk("%s FC P3 fallback detected: neighbor hops=1 but table_hops=%d, "
                        "using conservative params\r\n", BLE_GW_LOG, table_hops);
        } else if (is_p3 && p3_hops >= 2) {
            /* P3 fallback: 路由表已被污染为 hops=1, 但邻居标记仍记住原始多跳数 */
            hops = p3_hops;
            w_init = 10;  d_init = 30;  ss_init = 10;
            chkpt_to = FC_CHKPT_TIMEOUT_3HOP;
            osal_printk("%s FC P3 fallback (persistent): neighbor flag p3_hops=%d, "
                        "using conservative params\r\n", BLE_GW_LOG, p3_hops);
        } else {
            /* O8: 真正的近距离邻居 (Turbo 7.5ms，可用更激进参数) */
            w_init = 25;  d_init = 5;   ss_init = 50;
            chkpt_to = FC_CHKPT_TIMEOUT_1HOP;
        }
    } else if (hops == 2) {
        /* O8: 2 跳 (1 个中继): Turbo 7.5ms 允许更高吞吐 */
        w_init = 20;  d_init = 7;   ss_init = 30;
        chkpt_to = FC_CHKPT_TIMEOUT_2HOP;
    } else if (hops >= 3 && hops != 0xFF) {
        /* O8: 3+ 跳 (2+ 个中继): Turbo 7.5ms + 5ms tick 允许减小延时 */
        w_init = 15;  d_init = 10;  ss_init = 30;
        chkpt_to = FC_CHKPT_TIMEOUT_3HOP;
    } else {
        /* O8+F29b: 路由未知 (hops=0xFF): 使用 3+ 跳保守参数
         * 路由丢失后 RREQ 重建通常得到 3+ 跳路径，
         * 用 2-hop 参数 (TO=2500) 极易超时，改用 3-hop 保守值 */
        w_init = 15;  d_init = 10;  ss_init = 30;
        chkpt_to = FC_CHKPT_TIMEOUT_3HOP;
    }
    } /* end of: if (g_img_cache.is_multicast) ... else { ... } */

    /* O8: 根据跳数动态设置 START 延时 (O6+F16 基础上缩短)
     * Turbo 7.5ms 间隔传播更快，减少等待时间
     * 1跳=200ms, 2跳=300ms, 3+跳=500ms */
    uint16_t start_delay = (hops <= 1) ? 200 : (hops == 2) ? 300 : 500;

    g_fc.state        = FC_START_WAIT;
    g_fc.window       = w_init;
    g_fc.delay_ms     = d_init;
    g_fc.chkpt_timeout = chkpt_to;
    g_fc.ssthresh     = ss_init;
    g_fc.phase        = 0;  /* 慢启动阶段: 窗口翻倍增长 */
    g_fc.seg_id       = 0;  /* 段编号从 0 开始 */
    g_fc.seg_start    = 0;
    g_fc.next_seq     = 0;
    g_fc.prev_rx      = 0;
    g_fc.start_delay  = start_delay;  /* O6: 保存到 FC 结构，FC_START_WAIT 读取此值 */
    g_fc.state_tick   = osal_get_tick_ms();
    g_fc.last_pkt_tick = 0;
    g_fc.chkpt_send_tick = 0;
    g_fc.retry_round  = 0;
    g_fc.total_injected = 0;
    g_fc.pipeline_seq = 0;   /* O1: 滑动窗口流水线初始化 */
    g_fc.transfer_hops = hops; /* 记录传输开始时的跳数 (不受中途路由变化影响) */
    /* RTT 测量初始化: srtt=-1 表示尚未有有效样本 */
    g_fc.srtt       = -1;
    g_fc.rttvar     = 0;
    g_fc.rtt_valid  = 0;

    g_mesh_log_suppress = true;     /* P1: 抑制 Mesh 层冗余日志，减少串口占用 */
    sle_uart_pause_scan();           /* P3: 暂停 SLE 扫描，减少射频干扰提高传输可靠性 */

    /* O2: 本节点进入 Turbo 模式 + 广播通知全网中继节点同步切换
     * F18: 自发自收 (hops=0) 无需中继，跳过 Turbo 广播避免乱序竞态
     * F19: 广播帧携带序列号，中继节点据此过滤过时帧
     * O9:  取消前一次传输遗留的延迟 TURBO OFF */
    g_turbo_off_scheduled_tick = 0;  /* O9: 取消待发的延迟 OFF */
    if (hops != 0) {
        mesh_transport_set_turbo_mode(true);
        g_turbo_seq++;
        uint8_t turbo_cmd[4] = { MESH_TURBO_MAGIC, MESH_TURBO_ON,
                                  (uint8_t)(g_turbo_seq >> 8), (uint8_t)(g_turbo_seq & 0xFF) };
        mesh_broadcast(turbo_cmd, 4);
    }

    osal_printk("%s FC start: %d pkts, hops=%d W=%d D=%d SS=%d TO=%d startDelay=%d\r\n",
                BLE_GW_LOG, g_img_cache.fc_pkt_count, hops,
                g_fc.window, g_fc.delay_ms, g_fc.ssthresh, g_fc.chkpt_timeout,
                start_delay);

    /* O10: 自发自收内联快速路径 —— 完全跳过 tick 驱动的状态机
     * 自发自收时所有 mesh 操作均为本地同步回环 (loopback):
     *   fc_inject_data_pkt → image_receiver DATA (本地)
     *   fc_send_checkpoint → image_receiver CHKPT → CHKPT_ACK (同步)
     *   fc_send_end → image_receiver END → CRC → RESULT → send_image_response → BLE 通知手机
     * 全部在一次函数调用中完成，无需状态机 tick 驱动，延迟从 ~2200ms 降至 <1ms */
    if (hops == 0) {
        g_fc.state = FC_SENDING;  /* 非 IDLE，RESULT 处理器需要此状态 */
        for (uint16_t seq = 0; seq < g_img_cache.fc_pkt_count; seq++) {
            fc_inject_data_pkt(seq);
        }
        fc_send_checkpoint();  /* 同步: CHKPT → handle_checkpoint → CHKPT_ACK → fc_on_checkpoint_ack */
        fc_send_end();         /* 同步: END → handle_end → CRC OK → RESULT → BLE notify → FC_IDLE */
        /* send_image_response(0x86) 已在 fc_send_end 同步调用链中完成:
         *   FC_IDLE, g_img_cache.active=0, fc_stop_turbo, resume_scan, BLE notify */
        if (g_fc.state != FC_IDLE) {
            /* 兜底: 若同步路径异常未到达 IDLE，强制清理 */
            g_fc.state = FC_IDLE;
            g_mesh_log_suppress = false;
            sle_uart_resume_scan();
            g_img_cache.active = 0;
        }
        osal_printk("%s FC self-send complete (O10 inline fast path)\r\n", BLE_GW_LOG);
        return;
    }
}

/**
 * @brief  查询流控引擎是否活跃
 * @return true=流控传输进行中, false=空闲
 * @note   P4: 主循环中用于决定是否需要快速 tick (活跃时 1ms tick，空闲时普通 tick)
 */
bool ble_gateway_fc_is_active(void)
{
    return g_fc.state != FC_IDLE;
}

/**
 * @brief  流控状态机 tick 函数 —— 在主循环中周期性调用
 *
 * 驱动整个流控状态机的运行，包括:
 * - FC_START_WAIT: 等待 START 传播延时到期
 * - FC_SENDING:    根据包间延时批量发送数据包，段发完后发 CHECKPOINT
 * - FC_WAIT_CHKPT: 等待 CHECKPOINT_ACK，同时流水线预发下段数据 (O1)
 * - FC_SEND_END:   发送 END 帧
 * - FC_WAIT_RESULT: 等待 RESULT 或 MISSING，超时重发 END
 * - FC_RETRANSMIT: 根据 MISSING 位图补发缺失包
 *
 * @note   建议流控活跃时每 1ms 调用一次 (配合 FC_PKTS_PER_TICK 充分利用带宽)
 */
void ble_gateway_img_tick(void)
{
    /* O9: 延迟 TURBO OFF 广播 —— 在冷却期到期且无新 FC 启动时实际发出 */
    if (g_turbo_off_scheduled_tick != 0 && g_fc.state == FC_IDLE) {
        uint32_t elapsed = osal_get_tick_ms() - g_turbo_off_scheduled_tick;
        if (elapsed >= TURBO_OFF_COOLDOWN_MS) {
            g_turbo_off_scheduled_tick = 0;
            g_turbo_seq++;
            uint8_t cmd[4] = { MESH_TURBO_MAGIC, MESH_TURBO_OFF,
                                (uint8_t)(g_turbo_seq >> 8), (uint8_t)(g_turbo_seq & 0xFF) };
            mesh_gateway_inject(0xFFFF, cmd, sizeof(cmd));
            osal_printk("%s O9 deferred turbo OFF (seq=%d)\r\n", BLE_GW_LOG, g_turbo_seq);
        }
    }

    if (g_fc.state == FC_IDLE) return;

    uint32_t now = osal_get_tick_ms();

    switch (g_fc.state) {

    /* ── 等待 START 传播: 延时 start_delay ms 让目标节点和中继做好准备 ── */
    case FC_START_WAIT:
        if (now - g_fc.state_tick >= g_fc.start_delay) {
            /* F23: 组播模式下，在开始发数据前重播 MCAST_START
             * 网络拓扑可能在缓存期间发生变化（节点断连/重连），
             * 重播确保新上线的节点也进入接收状态 */
            if (g_img_cache.is_multicast && g_img_cache.mcast_start_len > 0) {
                mesh_broadcast(g_img_cache.mcast_start_frame, g_img_cache.mcast_start_len);
                osal_printk("%s FC MCAST_START re-broadcast before data\r\n", BLE_GW_LOG);
            }
            /* F29a: 单播模式下重新注入 START 命令
             * 原始 START (在 IMG_START 缓存时注入) 可能因路由不稳定而丢失，
             * 导致目标节点不在接收状态，后续所有 CHKPT 被 ignore。
             * 在 FC 数据发送前补发一次 START 确保目标已就绪 */
            if (!g_img_cache.is_multicast && g_img_cache.start_len > 0) {
                uint8_t sbuf[16];
                sbuf[0] = 0x04;
                (void)memcpy_s(&sbuf[1], sizeof(sbuf) - 1,
                               g_img_cache.start_payload, g_img_cache.start_len);
                uint8_t slen = 1 + g_img_cache.start_len;
                /* O4: 覆盖 pkt_count 为 FC 大包数 */
                if (slen >= 5) {
                    sbuf[3] = (g_img_cache.fc_pkt_count >> 8) & 0xFF;
                    sbuf[4] = g_img_cache.fc_pkt_count & 0xFF;
                }
                mesh_gateway_inject(g_img_cache.dst_addr, sbuf, slen);
                osal_printk("%s FC F29: re-inject START to 0x%04X\r\n",
                            BLE_GW_LOG, g_img_cache.dst_addr);
            }
            /* START 已传播完成，开始发送首段数据 */
            g_fc.state = FC_SENDING;
            g_fc.seg_start = 0;
            g_fc.next_seq  = 0;
            g_fc.last_pkt_tick = now;
            osal_printk("%s FC → SENDING seg=%d [%d ~ %d)\r\n",
                        BLE_GW_LOG, g_fc.seg_id, g_fc.seg_start,
                        g_fc.seg_start + g_fc.window);
        }
        break;

    /* ── 发送当前段数据包: 按包间延时逐批发送，段发完后发 CHECKPOINT ── */
    case FC_SENDING: {
        /* P4: 包间延时门控 —— 距上次发包未达 delay_ms 则不发送 */
        if (g_fc.delay_ms > 0 && (now - g_fc.last_pkt_tick) < g_fc.delay_ms) {
            break;
        }
        /* O13: SLE QoS 拥塞感知 — 链路 BUSY/FLOWCTRL 时暂停发送, 避免数据积压 */
        if (mesh_transport_is_sle_busy()) {
            break;
        }

        uint16_t seg_end = g_fc.seg_start + g_fc.window;
        if (seg_end > g_img_cache.fc_pkt_count) {
            seg_end = g_img_cache.fc_pkt_count;
        }

        if (g_fc.next_seq < seg_end) {
            /* O3: 每 tick 批量发送多包 (FC_PKTS_PER_TICK)，充分利用 SLE TX 缓冲区，
             * 避免每 tick 只发 1 包时 tick 开销占比过大 */
            uint8_t burst = FC_PKTS_PER_TICK;
            while (burst > 0 && g_fc.next_seq < seg_end) {
                fc_inject_data_pkt(g_fc.next_seq);
                g_fc.next_seq++;
                burst--;
            }
            g_fc.last_pkt_tick = now;
        }

        if (g_fc.next_seq >= seg_end) {
            /* F21: 自发自收时 ACK 可能已在 SENDING 期间到达并暂存
             * 此时无需发 CHECKPOINT，直接以暂存的 ACK 进入处理流程 */
            if (g_fc.ack_arrived && g_fc.transfer_hops == 0) {
                g_fc.chkpt_retry = 0;
                g_fc.state = FC_WAIT_CHKPT;
                g_fc.state_tick = now;
                g_fc.chkpt_send_tick = now;  /* RTT 0ms 表示本地 */
                g_fc.pipeline_seq = g_fc.next_seq;
                /* ack_arrived 保留为 1，tick 函数立即处理 */
            } else {
                /* 本段发完 → 发送 CHECKPOINT 请求目标报告已收包数
                 * 注意: ack_arrived 必须在 fc_send_checkpoint 之前清零，
                 * 因为 1 跳直连时 CHKPT→ACK 可能在 fc_send_checkpoint 内同步完成，
                 * 若在之后清零会丢弃刚到达的 ACK 导致不必要的超时等待 */
                g_fc.ack_arrived = 0;
                fc_send_checkpoint();
                g_fc.chkpt_retry = 0;
                g_fc.state = FC_WAIT_CHKPT;
                g_fc.state_tick = now;
                g_fc.pipeline_seq = g_fc.next_seq;  /* O1: 缓存当前发送位置，启动流水线预发 */
            }
        }
        break;
    }

    /* ── 等待 CHECKPOINT_ACK: O1 流水线模式，等待 ACK 期间继续预发下一段数据 ── */
    case FC_WAIT_CHKPT:
        /* O1: Pipeline —— 在等待 ACK 的同时预发下一段数据，消除空等时间 */
        {
            uint16_t pipe_limit = g_fc.next_seq + g_fc.window;
            if (pipe_limit > g_img_cache.fc_pkt_count) pipe_limit = g_img_cache.fc_pkt_count;
            if (g_fc.pipeline_seq < pipe_limit &&
                (g_fc.delay_ms == 0 || (now - g_fc.last_pkt_tick) >= g_fc.delay_ms)) {
                uint8_t burst = FC_PKTS_PER_TICK;
                while (burst > 0 && g_fc.pipeline_seq < pipe_limit) {
                    fc_inject_data_pkt(g_fc.pipeline_seq);
                    g_fc.pipeline_seq++;
                    burst--;
                }
                g_fc.last_pkt_tick = now;
            }
        }

        if (g_fc.ack_arrived) {
            /* ── RTT 测量 & 自适应超时更新 (RFC 6298 算法) ── */
            if (g_fc.chkpt_retry == 0) {
                /* 仅首次 CHKPT 的 ACK 才是干净的 RTT 样本 (Karn 算法: 重发的包有歧义) */
                int32_t rtt_sample = (int32_t)(now - g_fc.chkpt_send_tick);
                if (rtt_sample < 0) rtt_sample = 0;

                if (!g_fc.rtt_valid) {
                    /* 首个 RTT 样本: 直接赋值为初始 SRTT，方差取半 */
                    g_fc.srtt    = rtt_sample;
                    g_fc.rttvar  = rtt_sample / 2;
                    g_fc.rtt_valid = 1;
                } else {
                    /* RFC 6298 EWMA (Exponentially Weighted Moving Average) 更新:
                     * RTTVAR = (1-β)*RTTVAR + β*|SRTT - sample|
                     * SRTT   = (1-α)*SRTT   + α*sample */
                    int32_t delta = rtt_sample - g_fc.srtt;
                    if (delta < 0) delta = -delta;
                    g_fc.rttvar = g_fc.rttvar
                                  - (g_fc.rttvar >> FC_RTTVAR_BETA_SHIFT)
                                  + (delta >> FC_RTTVAR_BETA_SHIFT);
                    g_fc.srtt = g_fc.srtt
                                - (g_fc.srtt >> FC_SRTT_ALPHA_SHIFT)
                                + (rtt_sample >> FC_SRTT_ALPHA_SHIFT);
                }
                /* F8: RTO = SRTT + 4 * RTTVAR，按跳数设定下限，避免过小导致假超时 */
                int32_t rto = g_fc.srtt + 4 * g_fc.rttvar;
                int32_t rto_min = (int32_t)fc_rto_min_for_hops(g_fc.transfer_hops);
                if (rto < rto_min) rto = rto_min;
                if (rto > FC_RTO_MAX) rto = FC_RTO_MAX;
                g_fc.chkpt_timeout = (uint16_t)rto;

                osal_printk("%s FC RTT: sample=%d srtt=%d var=%d → TO=%d\r\n",
                            BLE_GW_LOG, rtt_sample, g_fc.srtt,
                            g_fc.rttvar, g_fc.chkpt_timeout);
            }

            /* 收到 ACK，计算本段丢包率并调用 AIMD 算法调整参数 */
            uint16_t seg_sent = g_fc.next_seq - g_fc.seg_start;
            uint16_t seg_rx = g_fc.ack_rx_count - g_fc.prev_rx;
            /* seg_rx 可能超过 seg_sent (之前丢的包延迟到达被计入)，必须截断避免负丢包率 */
            if (seg_rx > seg_sent) seg_rx = seg_sent;

            fc_adjust(seg_sent, seg_rx);

            /* 通知 APP 进度 */
            fc_notify_progress(0, g_fc.ack_rx_count);

            /* F24: 接收方已收齐全部数据时，跳过剩余段直接发 END
             * 修复: 之前 window < total 时 (如 W=8, total=23) 即使 rx=23/23 也会
             * 继续发送 seg=1,2,3 的冗余 CHKPT+数据, 每段各触发一次 BLE 进度通知,
             * 导致手机收到多次 "23/23" 且浪费 5~15s */
            if (g_fc.ack_rx_count >= g_img_cache.fc_pkt_count) {
                g_fc.state = FC_SEND_END;
                g_fc.state_tick = now;
                osal_printk("%s FC F24: rx=%d == total, fast-forward to END\r\n",
                            BLE_GW_LOG, g_fc.ack_rx_count);
            } else {
            /* O1: 准备下一段 —— 利用流水线已预发的包，跳过已发过的 seq */
            g_fc.prev_rx = g_fc.ack_rx_count;
            g_fc.seg_start = g_fc.next_seq;
            g_fc.next_seq = g_fc.pipeline_seq;  /* O1: 跳过流水线已预发的包，避免重复发送 */
            g_fc.seg_id++;

            if (g_fc.seg_start >= g_img_cache.fc_pkt_count) {
                /* 全部段已发完 → 发 END 帧通知目标节点传输结束 */
                g_fc.state = FC_SEND_END;
                g_fc.state_tick = now;
                osal_printk("%s FC all segments done, sending END\r\n", BLE_GW_LOG);
            } else {
                uint16_t seg_end = g_fc.seg_start + g_fc.window;
                if (seg_end > g_img_cache.fc_pkt_count) seg_end = g_img_cache.fc_pkt_count;

                if (g_fc.next_seq >= seg_end) {
                    /* O1: 流水线已覆盖整段 → 直接发 CHKPT，无需切换到 SENDING 状态
                     * ack_arrived 在 fc_send_checkpoint 前清零 (同 FC_SENDING 修复) */
                    g_fc.ack_arrived = 0;
                    fc_send_checkpoint();
                    g_fc.chkpt_retry = 0;
                    g_fc.state_tick = now;
                    g_fc.pipeline_seq = g_fc.next_seq;
                    osal_printk("%s FC O1 pipeline: seg=%d pre-sent [%d ~ %d)\r\n",
                                BLE_GW_LOG, g_fc.seg_id, g_fc.seg_start, seg_end);
                } else {
                    /* 新段还有包未发送 → 切换回 SENDING 状态继续发送 */
                    g_fc.state = FC_SENDING;
                    g_fc.last_pkt_tick = now;
                    osal_printk("%s FC → SENDING seg=%d [%d ~ %d)\r\n",
                                BLE_GW_LOG, g_fc.seg_id, g_fc.seg_start,
                                g_fc.seg_start + g_fc.window);
                }
            }
            }  /* end F24 else (not all received) */
        } else if (now - g_fc.state_tick >= g_fc.chkpt_timeout) {
            g_fc.chkpt_retry++;
            osal_printk("%s FC CHKPT timeout seg=%d retry=%d/%d (TO=%d)\r\n",
                        BLE_GW_LOG, g_fc.seg_id, g_fc.chkpt_retry,
                        FC_CHKPT_MAX_RETRY, g_fc.chkpt_timeout);

            if (g_fc.chkpt_retry <= FC_CHKPT_MAX_RETRY) {
                /* 指数退避: 每次超时 RTO 翻倍 (TCP Karn 算法)，避免持续假超时 */
                uint16_t new_to = g_fc.chkpt_timeout * 2;
                if (new_to > FC_RTO_MAX) new_to = FC_RTO_MAX;
                g_fc.chkpt_timeout = new_to;
                /* 重发 CHECKPOINT (可能是 CHKPT 或 ACK 丢失了) */
                fc_send_checkpoint();
                g_fc.state_tick = now;  /* 重置超时计时器 */
            } else {
                /* CHECKPOINT 多次无响应 —— 视为严重拥塞，重置参数并重发整段 */
                g_fc.retry_round++;
                /* F29c: 总重置次数限制防止 FC 无限卡死
                 * 每次 CHKPT 耗尽 (3 次超时) 计为一轮，超过 MAX_RETRY 轮后放弃 */
                if (g_fc.retry_round > IMG_GW_MAX_RETRY) {
                    osal_printk("%s FC give up: %d CHKPT resets without progress\r\n",
                                BLE_GW_LOG, g_fc.retry_round);
                    g_fc.state = FC_IDLE;
                    g_mesh_log_suppress = false;
                    sle_uart_resume_scan();
                    fc_stop_turbo_now();
                    g_img_cache.active = 0;
                    break;
                }
                g_fc.ssthresh = g_fc.window / 2;
                if (g_fc.ssthresh < FC_W_MIN) g_fc.ssthresh = FC_W_MIN;
                g_fc.window = FC_W_MIN;
                g_fc.delay_ms = FC_D_MAX;
                g_fc.phase = 0;
                g_fc.chkpt_retry = 0;  /* F29c: 重置 per-segment 计数器 */

                /* P21: CHKPT 耗尽可能是 START 丢失导致目标不在接收状态，
                 * 补发 START 后再重发段数据，确保目标能正常接收 */
                if (!g_img_cache.is_multicast && g_img_cache.start_len > 0) {
                    uint8_t sbuf[16];
                    sbuf[0] = 0x04;
                    (void)memcpy_s(&sbuf[1], sizeof(sbuf) - 1,
                                   g_img_cache.start_payload, g_img_cache.start_len);
                    uint8_t slen = 1 + g_img_cache.start_len;
                    if (slen >= 5) {
                        sbuf[3] = (g_img_cache.fc_pkt_count >> 8) & 0xFF;
                        sbuf[4] = g_img_cache.fc_pkt_count & 0xFF;
                    }
                    mesh_gateway_inject(g_img_cache.dst_addr, sbuf, slen);
                    osal_printk("%s FC P21: re-inject START (CHKPT exhausted)\r\n", BLE_GW_LOG);
                }

                /* 回到段起点重发 */
                g_fc.next_seq = g_fc.seg_start;
                g_fc.pipeline_seq = g_fc.seg_start;  /* O1: 重置流水线 */
                g_fc.seg_id++;
                g_fc.state = FC_SENDING;
                g_fc.last_pkt_tick = now;

                osal_printk("%s FC resend seg from %d, W=%d D=%d\r\n",
                            BLE_GW_LOG, g_fc.seg_start, g_fc.window, g_fc.delay_ms);
            }
        }
        break;

    /* ── 发送 END 帧: 通知目标节点所有数据已发送完毕 ── */
    case FC_SEND_END:
        fc_send_end();
        g_fc.state = FC_WAIT_RESULT;
        g_fc.state_tick = now;
        osal_printk("%s FC → WAIT_RESULT\r\n", BLE_GW_LOG);
        break;

    /* ── 等待 RESULT(0x86) 或 MISSING(0x87): 目标节点验证后回复结果 ── */
    case FC_WAIT_RESULT:
        if (g_img_cache.is_multicast) {
            /* ── v3: 组播模式 WAIT_RESULT ──
             * 等待所有目标节点回复，收集 RESULT 和 MISSING。
             * 由 ble_gateway_send_image_response() 异步处理每个响应。
             * 此处仅做超时检测和状态迁移。 */
            if (g_img_cache.completed_count >= g_img_cache.target_count) {
                /* 所有目标已完成（在 send_image_response 中已处理清理）*/
                break;
            }
            if (now - g_fc.state_tick >= FC_RESULT_TIMEOUT_MS) {
                if (g_fc.mcast_missing_received && g_fc.miss_count > 0) {
                    /* 有目标报告缺包 → 补包 */
                    osal_printk("%s FC MCAST: retransmit %d missing pkts\r\n",
                                BLE_GW_LOG, g_fc.miss_count);
                    g_fc.mcast_missing_received = 0;
                    g_fc.state = FC_RETRANSMIT;
                    g_fc.retx_sent = 0;
                    g_fc.miss_scan_pos = 0;
                    g_fc.last_pkt_tick = now;
                } else {
                    /* 超时：重发 END 或放弃 */
                    g_fc.retry_round++;
                    osal_printk("%s FC MCAST RESULT timeout, round=%d (%d/%d done)\r\n",
                                BLE_GW_LOG, g_fc.retry_round,
                                g_img_cache.completed_count, g_img_cache.target_count);
                    if (g_fc.retry_round > IMG_GW_MAX_RETRY) {
                        /* 放弃：将未完成的目标标记为超时 */
                        for (uint8_t i = 0; i < g_img_cache.target_count; i++) {
                            if (g_img_cache.target_done[i] == 0) {
                                g_img_cache.target_done[i] = 3; /* 超时 */
                                g_img_cache.completed_count++;
                                mcast_notify_progress(g_img_cache.target_addrs[i], 0x02);
                            }
                        }
                        g_fc.state = FC_IDLE;
                        g_mesh_log_suppress = false;
                        sle_uart_resume_scan();
                        fc_stop_turbo_now();
                        g_img_cache.active = 0;
                    } else if (g_fc.retry_round <= 2) {
                        /* O15: 轻量重试 - 仅重发 END，避免不必要的全量重发
                         * 多数节点可能已收到完整数据但 END 丢失;
                         * 重发 END 后: 数据完整的节点回 RESULT, 数据不完整的回 MISSING */
                        fc_send_end();
                        g_fc.state_tick = now;
                        osal_printk("%s FC MCAST: light retry (END only), round %d\r\n",
                                    BLE_GW_LOG, g_fc.retry_round);
                    } else {
                        /* F23: 重播 MCAST_START + 重发全部数据
                         * 未响应的节点可能从未收到最初的 MCAST_START (网络拓扑变化导致)，
                         * 需要重播 START 后再重发完整数据，让它们从头开始接收 */
                        if (g_img_cache.mcast_start_len > 0) {
                            mesh_broadcast(g_img_cache.mcast_start_frame,
                                           g_img_cache.mcast_start_len);
                            osal_printk("%s FC MCAST_START re-broadcast (retry round %d)\r\n",
                                        BLE_GW_LOG, g_fc.retry_round);
                        }
                        /* 重置 FC 为全量重发: 从 seg=0 开始，使用保守参数 */
                        g_fc.seg_id = 0;
                        g_fc.seg_start = 0;
                        g_fc.next_seq = 0;
                        g_fc.pipeline_seq = 0;
                        g_fc.prev_rx = 0;
                        g_fc.ack_arrived = 0;
                        g_fc.noprogress_count = 0;
                        g_fc.window = 10;
                        g_fc.delay_ms = 10;
                        g_fc.start_delay = 500;
                        g_fc.state = FC_START_WAIT;
                        g_fc.state_tick = now;
                        osal_printk("%s FC MCAST: full resend from seg=0\r\n", BLE_GW_LOG);
                    }
                }
            }
        } else {
            /* ── 单播模式 WAIT_RESULT (原有逻辑) ── */
            if (now - g_fc.state_tick >= FC_RESULT_TIMEOUT_MS) {
                osal_printk("%s FC RESULT timeout, resend END\r\n", BLE_GW_LOG);
                g_fc.retry_round++;
                if (g_fc.retry_round > IMG_GW_MAX_RETRY) {
                    osal_printk("%s FC give up after %d rounds\r\n",
                                BLE_GW_LOG, g_fc.retry_round);
                    g_fc.state = FC_IDLE;
                    g_mesh_log_suppress = false;
                    sle_uart_resume_scan();
                    fc_stop_turbo_now();
                    g_img_cache.active = 0;
                } else {
                    g_fc.state = FC_SEND_END;
                    g_fc.state_tick = now;
                }
            }
        }
        break;

    /* ── 补包模式: 扫描 MISSING 位图，逐个补发缺失的数据包 ── */
    case FC_RETRANSMIT: {
        /* P4: 补包延时门控，使用较保守的延时 (delay_ms 和 FC_RETX_DELAY_MS 取大值) */
        uint16_t retx_delay = (g_fc.delay_ms > FC_RETX_DELAY_MS) ?
                               g_fc.delay_ms : FC_RETX_DELAY_MS;
        if ((now - g_fc.last_pkt_tick) < retx_delay) {
            break;
        }

        /* O3: 补包也采用批量发送 (FC_PKTS_PER_TICK 包/tick) */
        uint8_t retx_burst = FC_PKTS_PER_TICK;
        while (retx_burst > 0 && g_fc.miss_scan_pos < g_img_cache.fc_pkt_count) {
            uint16_t seq = g_fc.miss_scan_pos;
            g_fc.miss_scan_pos++;
            /* miss_bitmap 中 bit=1 表示对应 seq 的包缺失，需要补发 */
            if (g_fc.miss_bitmap[seq >> 3] & (1 << (seq & 7))) {
                fc_inject_data_pkt(seq);
                g_fc.retx_sent++;
                retx_burst--;
                /* 每补发 10 包报告一次进度，避免 APP 无响应 */
                if (g_fc.retx_sent % 10 == 0) {
                    fc_notify_progress(1, g_fc.ack_rx_count + g_fc.retx_sent);
                }
            }
        }
        g_fc.last_pkt_tick = now;

        if (g_fc.miss_scan_pos < g_img_cache.fc_pkt_count) {
            break;  /* 位图未扫完，还有缺包待补发，下次 tick 继续 */
        }

        /* 位图扫描完毕 → 所有缺包已补发，发 END 帧给目标节点重新验证 */
        osal_printk("%s FC retransmit done (%d/%d pkts), sending END\r\n",
                    BLE_GW_LOG, g_fc.retx_sent, g_fc.miss_count);
        fc_notify_progress(1, g_fc.ack_rx_count + g_fc.retx_sent);
        g_fc.state = FC_SEND_END;
        g_fc.state_tick = now;
        break;
    }

    default:
        break;
    }
}

/**
 * @brief  处理目标节点返回的 CHECKPOINT_ACK (0x88)
 * @param  data  报文数据: [0x88, SEG_ID, RX_COUNT_HI, RX_COUNT_LO]
 * @param  len   报文长度
 * @note   该函数实现了以下逻辑:
 *         1. 过滤过期 ACK (seg_id < 当前段)
 *         2. Fix B: 检测无进展 ACK (rx_count 未增加)，用 2s 短超时快速重询
 *         3. 设置 ack_arrived 标志，由 ble_gateway_img_tick() 在下次 tick 中处理
 */
static void fc_on_checkpoint_ack(const uint8_t *data, uint16_t len)
{
    if (len < 4) return;
    /* F21: 自发自收时 D=0 全量发送，ACK 可能在 FC_SENDING 状态到达
     * (image_receiver 同步处理完所有数据后立即回 ACK，此时 FC 尚未转入 WAIT_CHKPT)
     * 此时应接受 ACK 而非丢弃，否则会触发不必要的 CHKPT 超时 */
    if (g_fc.state == FC_SENDING && g_fc.transfer_hops == 0) {
        /* 自发自收: 将 ACK 暂存，切到 WAIT_CHKPT 后由 tick 函数处理 */
        uint8_t  seg_id   = data[1];
        uint16_t rx_count = ((uint16_t)data[2] << 8) | data[3];
        g_fc.ack_arrived  = 1;
        g_fc.ack_rx_count = rx_count;
        osal_printk("%s FC CHKPT_ACK (self-send fast): seg=%d rx=%d/%d\r\n",
                    BLE_GW_LOG, seg_id, rx_count, g_img_cache.fc_pkt_count);
        return;
    }
    if (g_fc.state != FC_WAIT_CHKPT) {
        osal_printk("%s FC CHKPT_ACK ignored (state=%d)\r\n", BLE_GW_LOG, g_fc.state);
        return;
    }

    uint8_t  seg_id   = data[1];
    uint16_t rx_count = ((uint16_t)data[2] << 8) | data[3];

    osal_printk("%s FC CHKPT_ACK: seg=%d rx=%d/%d\r\n",
                BLE_GW_LOG, seg_id, rx_count, g_img_cache.fc_pkt_count);

    /* P21: rx_count == 0xFFFF 表示目标节点不在接收状态 (START 丢失)
     * 立即重发 START，重置到当前段起点，等 START 传播后重发 CHKPT 验证 */
    if (rx_count == 0xFFFF && !g_img_cache.is_multicast) {
        osal_printk("%s FC P21: target NOT_RECEIVING, re-inject START to 0x%04X\r\n",
                    BLE_GW_LOG, g_img_cache.dst_addr);
        if (g_img_cache.start_len > 0) {
            uint8_t sbuf[16];
            sbuf[0] = 0x04;
            (void)memcpy_s(&sbuf[1], sizeof(sbuf) - 1,
                           g_img_cache.start_payload, g_img_cache.start_len);
            uint8_t slen = 1 + g_img_cache.start_len;
            if (slen >= 5) {
                sbuf[3] = (g_img_cache.fc_pkt_count >> 8) & 0xFF;
                sbuf[4] = g_img_cache.fc_pkt_count & 0xFF;
            }
            mesh_gateway_inject(g_img_cache.dst_addr, sbuf, slen);
        }
        /* 重置当前段，留 800ms 等 START 多跳传播后自动重发 CHKPT */
        g_fc.next_seq = g_fc.seg_start;
        g_fc.pipeline_seq = g_fc.seg_start;
        g_fc.chkpt_retry = 0;
        g_fc.noprogress_count = 0;
        g_fc.state_tick = osal_get_tick_ms();
        g_fc.chkpt_timeout = 800;
        return;
    }

    /* Fix: 过滤过期 ACK —— 只接受当前段或比当前段新的 ACK，丢弃旧 ACK */
    if (seg_id < g_fc.seg_id) {
        osal_printk("%s FC CHKPT_ACK stale seg=%d (cur=%d), ignored\r\n",
                    BLE_GW_LOG, seg_id, g_fc.seg_id);
        return;
    }

    /* F22: 若 rx_count 已等于总包数，说明目标已收齐全部数据，直接跳过无进展检测
     * 修复: 之前 prev_rx == rx_count == total 时误判为 "无进展"，
     * 导致已完成的传输反复重传数十秒 */
    if (rx_count >= g_img_cache.fc_pkt_count) {
        g_fc.noprogress_count = 0;
        g_fc.ack_arrived  = 1;
        g_fc.ack_rx_count = rx_count;
        osal_printk("%s FC CHKPT_ACK: rx=%d == total, transfer complete\r\n",
                    BLE_GW_LOG, rx_count);
        return;
    }

    /* Fix B 增强: 检测无进展 ACK
     * 如果 rx_count 未增加，说明数据包可能还在慢路径上
     * (CHKPT 帧走了更快的路径先到达，数据还未到)。
     * 使用 2 秒短超时快速重新轮询，而非被动等待长超时。
     * 连续 3 次无进展 (共 ~6s) 后视为真实丢包，进入正常 AIMD 处理 */
    if (rx_count <= g_fc.prev_rx && g_fc.prev_rx > 0) {
        g_fc.noprogress_count++;
        osal_printk("%s FC CHKPT_ACK seg=%d: no progress #%d (rx=%d prev=%d)\r\n",
                    BLE_GW_LOG, seg_id, g_fc.noprogress_count, rx_count, g_fc.prev_rx);
        if (g_fc.noprogress_count <= 3) {
            /* 数据可能还在慢路径上 —— 设置 2s 短超时，让超时机制快速重发 CHKPT */
            g_fc.state_tick = osal_get_tick_ms();
            g_fc.chkpt_timeout = 2000;
            return;  /* 不设置 ack_arrived，等待下次超时重新轮询 */
        }
        /* 超过 3 次无进展 (共等待 ~6s)，视为真实丢包，继续正常 AIMD 处理 */
        osal_printk("%s FC no-progress limit reached, accepting as loss\r\n", BLE_GW_LOG);
    }

    g_fc.noprogress_count = 0;
    g_fc.ack_arrived  = 1;
    g_fc.ack_rx_count = rx_count;
}

/**
 * @brief  处理目标节点返回的 MISSING 位图 (0x87)
 * @param  data  报文数据: [0x87, MISS_COUNT_HI, MISS_COUNT_LO, BITMAP[IMG_GW_BITMAP_BYTES]]
 * @param  len   报文长度
 * @note   网关复制位图到 g_fc.miss_bitmap，切换到 FC_RETRANSMIT 状态
 *         由 ble_gateway_img_tick() 扫描位图进行补发。
 *         超过 IMG_GW_MAX_RETRY 轮后放弃传输。
 */
static void fc_on_missing_bitmap(const uint8_t *data, uint16_t len)
{
    if (len < 3 + IMG_GW_BITMAP_BYTES) {
        osal_printk("%s FC MISSING too short %d\r\n", BLE_GW_LOG, len);
        return;
    }

    uint16_t miss_count = ((uint16_t)data[1] << 8) | data[2];

    /* 复制缺包位图到流控结构 */
    (void)memcpy_s(g_fc.miss_bitmap, IMG_GW_BITMAP_BYTES,
                   &data[3], IMG_GW_BITMAP_BYTES);
    g_fc.miss_count = miss_count;
    g_fc.miss_scan_pos = 0;
    g_fc.retry_round++;

    osal_printk("%s FC MISSING bitmap: %d missing, round=%d\r\n",
                BLE_GW_LOG, miss_count, g_fc.retry_round);

    if (g_fc.retry_round > IMG_GW_MAX_RETRY) {
        osal_printk("%s FC max retries exceeded, give up\r\n", BLE_GW_LOG);
        g_fc.state = FC_IDLE;
        g_mesh_log_suppress = false;
        sle_uart_resume_scan();  /* P5: 传输失败，恢复 SLE 扫描 */
        fc_stop_turbo_now();     /* O2: 恢复正常连接间隔 (立即) */
        g_img_cache.active = 0;
        return;
    }

    /* 进入补包状态: 扫描位图逐个补发缺失包 */
    g_fc.state = FC_RETRANSMIT;
    g_fc.retx_sent = 0;
    g_fc.last_pkt_tick = osal_get_tick_ms();
}

/* ═══════════════════════════════════════════════════
 *  上行消息路由 (目标节点 → 网关 → APP)
 *  处理目标节点返回的图片传输响应帧，根据命令类型分别处理:
 *  - 流控相关命令 (0x88 CHKPT_ACK, 0x87 MISSING): 驱动流控状态机
 *  - 传输结果 (0x86 RESULT): 停止流控 + 恢复状态
 *  - 所有命令都会转发给 APP (除 0x88 用进度替代)
 * ═══════════════════════════════════════════════════ */

/**
 * @brief  处理目标节点的图片传输上行消息，并转发给手机 APP
 * @param  src_addr  消息来源节点的 Mesh 地址
 * @param  data      消息数据 (首字节为命令码)
 * @param  len       数据长度
 * @return 0=成功, -1=未连接, -2=参数无效, -3=Notify 发送失败
 *
 * @note   命令处理逻辑:
 *         0x88 CHKPT_ACK: 驱动流控 (fc_on_checkpoint_ack)，不转发给 APP，用 0x89 进度替代
 *         0x87 MISSING:   触发补包流程 (fc_on_missing_bitmap)，同时转发给 APP
 *         0x85 ACK:       v2.2 代理模式下静默忽略 (网关已代发ACK)，否则转发
 *         0x86 RESULT:    停止流控引擎，恢复扫描和连接间隔，转发给 APP
 */
int ble_gateway_send_image_response(uint16_t src_addr, const uint8_t *data, uint16_t len)
{
    if (!g_gw_connected || g_gw_notify_handle == 0) return -1;
    if (len < 2 || data == NULL) return -2;

    uint8_t cmd = data[0];

    /* ═══════════════════════════════════════════════════
     *  v3: 组播模式下的上行消息处理
     *  组播使用独立的响应路由逻辑：
     *  - CHKPT_ACK: 接受任意目标节点的首个 ACK 驱动 FC
     *  - MISSING: OR 合并所有目标的缺包位图
     *  - RESULT: 逐节点跟踪完成状态，通过 0x8A 通知 APP
     * ═══════════════════════════════════════════════════ */
    if (g_img_cache.is_multicast && g_img_cache.active) {

        /* ── 0x88 CHECKPOINT_ACK: 接受任意目标节点的 ACK (首个驱动 FC) ── */
        if (cmd == 0x88 && is_mcast_target(src_addr)) {
            fc_on_checkpoint_ack(data, len);
            return 0;
        }

        /* ── 0x87 MISSING: OR 合并所有目标的缺包位图 ── */
        if (cmd == 0x87 && is_mcast_target(src_addr)) {
            if (len >= 3 + IMG_GW_BITMAP_BYTES) {
                /* 首次收到 MISSING 时清零位图，后续 OR 合并 */
                if (!g_fc.mcast_missing_received) {
                    (void)memset_s(g_fc.miss_bitmap, IMG_GW_BITMAP_BYTES, 0, IMG_GW_BITMAP_BYTES);
                }
                /* OR 合并: 将此目标的缺包位图并入全局位图 */
                for (uint8_t i = 0; i < IMG_GW_BITMAP_BYTES; i++) {
                    g_fc.miss_bitmap[i] |= data[3 + i];
                }
                g_fc.mcast_missing_received = 1;
                /* 重新计算合并后的缺包总数 */
                uint16_t total_miss = 0;
                for (uint8_t i = 0; i < IMG_GW_BITMAP_BYTES; i++) {
                    uint8_t b = g_fc.miss_bitmap[i];
                    while (b) { total_miss += (b & 1); b >>= 1; }
                }
                g_fc.miss_count = total_miss;
                osal_printk("%s MCAST MISSING from 0x%04X, merged total=%d\r\n",
                            BLE_GW_LOG, src_addr, total_miss);
            }
            return 0;  /* 不转发给 APP */
        }

        /* ── 0x85 ACK: 组播模式下忽略逐包 ACK ── */
        if (cmd == 0x85 && is_mcast_target(src_addr)) {
            return 0;
        }

        /* ── 0x86 RESULT: 逐节点跟踪完成状态 ── */
        if (cmd == 0x86 && is_mcast_target(src_addr)) {
            uint8_t status = (len >= 2) ? data[1] : 0xFF;
            /* 查找目标并标记完成（去重：已标记的不再重复计数） */
            for (uint8_t i = 0; i < g_img_cache.target_count; i++) {
                if (g_img_cache.target_addrs[i] == src_addr &&
                    g_img_cache.target_done[i] == 0) {
                    g_img_cache.target_done[i] = (status == 0x00) ? 1 : 2;
                    g_img_cache.completed_count++;
                    osal_printk("%s MCAST RESULT from 0x%04X: status=%d (%d/%d done)\r\n",
                                BLE_GW_LOG, src_addr, status,
                                g_img_cache.completed_count, g_img_cache.target_count);
                    /* 通知 APP 组播进度 */
                    mcast_notify_progress(src_addr, status);
                    break;
                }
            }
            /* 如果全部目标完成 → 停止 FC，清理状态 */
            if (g_img_cache.completed_count >= g_img_cache.target_count) {
                osal_printk("%s MCAST ALL DONE: %d/%d targets completed\r\n",
                            BLE_GW_LOG, g_img_cache.completed_count, g_img_cache.target_count);
                if (g_fc.state != FC_IDLE) {
                    g_fc.state = FC_IDLE;
                    g_mesh_log_suppress = false;
                    sle_uart_resume_scan();
                    fc_stop_turbo();
                }
                g_img_cache.active = 0;
            }
            return 0;  /* 不转发原始 RESULT，使用 0x8A 进度帧替代 */
        }

        return 0;  /* 组播模式下其他命令静默忽略 */
    }

    /* ═══════════════════════════════════════════════════
     *  F30: 传输完成后丢弃迟到的控制消息
     *  防止 stale 0x88/0x87/0x85 泄露到手机显示为 [0x0000]AA-88-...
     * ═══════════════════════════════════════════════════ */
    if (!g_img_cache.active && (cmd == 0x88 || cmd == 0x87 || cmd == 0x85)) {
        return 0;
    }

    /* ═══════════════════════════════════════════════════
     *  单播模式下的上行消息处理 (原有逻辑)
     * ═══════════════════════════════════════════════════ */

    /* ── 0x88 CHECKPOINT_ACK: 流控处理 ── */
    if (cmd == 0x88 && g_img_cache.active &&
        src_addr == g_img_cache.dst_addr) {
        fc_on_checkpoint_ack(data, len);
        /* 不转发 0x88 给 APP, 用 0x89 progress 代替 */
        return 0;
    }

    /* ── 0x87 MISSING 位图: 流控补包 ── */
    if (cmd == 0x87 && g_img_cache.active &&
        src_addr == g_img_cache.dst_addr) {
        fc_on_missing_bitmap(data, len);
        /* 转发给 APP (简化版: 只告诉缺包数) */
    }

    /* ── 0x85 ACK: v2.2 代理模式下忽略目标的逐包ACK ── */
    if (cmd == 0x85 && g_img_cache.active &&
        g_img_cache.xfer_mode == 1 &&
        src_addr == g_img_cache.dst_addr) {
        /* ACK代理模式: 目标以FAST模式运行, 不应产生0x85
         * 但如果收到了, 静默忽略 (不转发给APP, 因为网关已发过本地ACK) */
        osal_printk("%s IMG ACK from target ignored (proxy mode)\r\n", BLE_GW_LOG);
        return 0;
    }

    /* ── 0x86 RESULT: 传输完成，停止流控引擎，清理缓存 ── */
    if (cmd == 0x86) {
        uint8_t status = (len >= 2) ? data[1] : 0xFF;
        /* F30: 校验 RESULT 来源: 必须来自当前目标，防止前一目标的迟到 RESULT 误杀当前 FC */
        if (g_img_cache.active && src_addr != g_img_cache.dst_addr) {
            osal_printk("%s IMG RESULT from 0x%04X ignored (dst=0x%04X)\r\n",
                        BLE_GW_LOG, src_addr, g_img_cache.dst_addr);
            return 0;
        }
        if (g_fc.state != FC_IDLE) {
            /* 首次收到 RESULT: 停止流控引擎，恢复所有临时状态 */
            osal_printk("%s IMG DONE: src=0x%04X status=%d fc_rounds=%d injected=%d\r\n",
                        BLE_GW_LOG, src_addr, status,
                        g_fc.retry_round, g_fc.total_injected);

            g_fc.state = FC_IDLE;
            g_mesh_log_suppress = false;
            sle_uart_resume_scan();  /* P5: 传输完成，恢复 SLE 扫描 */
            fc_stop_turbo();         /* O2: 恢复正常连接间隔 */
            g_img_cache.active = 0;
        } else {
            /* F27: 重复 RESULT → 不转发给手机, 防止 APP 收到多次 "传输成功" */
            osal_printk("%s IMG DONE dup ignored (already IDLE)\r\n", BLE_GW_LOG);
            return 0;
        }
    }

    /* ── 将响应帧转发给手机 APP (BLE Notify) ── */
    uint8_t frame[250];
    uint16_t pos = 0;
    frame[pos++] = 0xAA;
    frame[pos++] = cmd;
    frame[pos++] = (src_addr >> 8) & 0xFF;
    frame[pos++] = src_addr & 0xFF;

    uint16_t rest = len - 1;
    if (rest > 0 && pos + rest <= sizeof(frame)) {
        (void)memcpy_s(&frame[pos], sizeof(frame) - pos, &data[1], rest);
        pos += rest;
    }

    gatts_ntf_ind_t param = { 0 };
    param.attr_handle = g_gw_notify_handle;
    param.value_len = pos;
    param.value = frame;

    errcode_t ret = gatts_notify_indicate(g_gw_server_id, g_gw_conn_id, &param);
    if (ret != 0) {
        osal_printk("%s IMG resp notify FAILED ret=%d\r\n", BLE_GW_LOG, ret);
        return -3;
    }
    osal_printk("%s IMG resp: cmd=0x%02X src=0x%04X len=%d\r\n",
                BLE_GW_LOG, cmd, src_addr, pos);
    return 0;
}

/* ════════════════════════════════════════════════════════════
 *  v2.2: ACK 代理模块 —— 网关本地快速回 ACK 给 APP
 *
 *  APP 以为目标节点确认了，实际是网关本地快速回复。
 *  数据缓存在网关，END 帧后用流控引擎统一发送到目标节点。
 *  效果: 消除逐包 BLE→Mesh RTT 累积，2跳场景从 ~50s 降至 ~10s。
 *
 *  模拟 ACK 帧格式 (与目标节点真实 ACK 完全一致):
 *    AA 85 DST_ADDR(2) STATUS(1) SEQ_HI SEQ_LO
 * ════════════════════════════════════════════════════════════ */

/**
 * @brief  发送本地代理 ACK 给手机 APP (v2.2 ACK 代理模式)
 * @param  seq  当前确认的包序号
 * @note   始终返回成功 (IMG_ACK_OK)，APP 自然循环发送直到最后一包
 */
static void gw_send_local_ack(uint16_t seq)
{
    if (!g_gw_connected || g_gw_notify_handle == 0) return;

    uint8_t frame[7];
    frame[0] = 0xAA;
    frame[1] = 0x85;  /* CMD_IMG_ACK: 图片传输 ACK 命令码 */
    frame[2] = (g_img_cache.dst_addr >> 8) & 0xFF;
    frame[3] = g_img_cache.dst_addr & 0xFF;
    frame[4] = 0x00;  /* IMG_ACK_OK: 始终返回成功，APP 自然循环发下一包 */
    frame[5] = (seq >> 8) & 0xFF;
    frame[6] = seq & 0xFF;

    gatts_ntf_ind_t param = { 0 };
    param.attr_handle = g_gw_notify_handle;
    param.value_len = sizeof(frame);
    param.value = frame;
    gatts_notify_indicate(g_gw_server_id, g_gw_conn_id, &param);
}

/* ═══════════════════════════════════════════════════
 *  BLE 写入回调: APP → 网关
 *  处理手机 APP 通过 BLE Write 发送的所有命令，包括:
 *  - 0x03: 拓扑查询
 *  - 0x04-0x07: 图片传输 (START/DATA/END/CANCEL)
 *  - 0x01-0x02: 常规 Mesh 命令转发
 * ═══════════════════════════════════════════════════ */

/**
 * @brief  BLE GATT 写入请求回调 —— 处理手机 APP 发送的所有命令
 * @param  server_id  GATT Server ID
 * @param  conn_id    BLE 连接 ID
 * @param  write_cb   写入请求参数 (包含数据和长度)
 * @param  status     写入状态
 * @note   所有 APP 下行命令都以 0xAA 开头，第二字节为命令码。
 *         图片命令 (0x04-0x07) 格式: AA CMD DST_ADDR(2) PAYLOAD...
 */
static void gw_write_req_cbk(uint8_t server_id, uint16_t conn_id,
                               gatts_req_write_cb_t *write_cb, errcode_t status)
{
    if (write_cb->length < 2 || write_cb->value == NULL) return;
    if (write_cb->value[0] != 0xAA) return;

    uint8_t cmd = write_cb->value[1];

    /* 拓扑查询命令 (0x03) */
    if (cmd == 0x03) {
        gw_handle_topology_query();

    } else if (cmd == 0x0A) {
        /* ═══════════════════════════════════════════════════
         *  v3: 组播图片传输开始命令 (CMD_IMG_MCAST_START = 0x0A)
         *
         *  BLE 帧格式:
         *    AA 0A N(1) ADDR1(2)...ADDRn(2) TOTAL(2) PKT(2) W(2) H(2) MODE(1) [XFER(1)]
         *
         *  处理流程:
         *    1. 解析 N 和目标地址列表
         *    2. 初始化 g_img_cache + 组播字段
         *    3. 构造 Mesh MCAST_START 帧 (含目标列表) 并广播
         *    4. APP 随后继续发 IMG_DATA (0x05) + IMG_END (0x06)，格式不变
         * ═══════════════════════════════════════════════════ */
        if (write_cb->length < 5) return;  /* 至少: AA(1)+CMD(1)+N(1)+ADDR(2) */

        uint8_t n_targets = write_cb->value[2];
        if (n_targets == 0 || n_targets > MESH_MCAST_MAX_TARGETS) {
            osal_printk("%s MCAST: invalid target count %d\r\n", BLE_GW_LOG, n_targets);
            return;
        }

        /* 解析目标地址列表 */
        uint16_t addr_end = 3 + 2 * (uint16_t)n_targets;
        if (write_cb->length < addr_end + 9) {
            osal_printk("%s MCAST: frame too short %d (need %d)\r\n",
                        BLE_GW_LOG, write_cb->length, addr_end + 9);
            return;
        }

        /* 初始化缓存 */
        (void)memset_s(&g_img_cache, sizeof(g_img_cache), 0, sizeof(g_img_cache));
        g_img_cache.is_multicast = 1;
        g_img_cache.target_count = n_targets;
        g_img_cache.dst_addr = MESH_ADDR_BROADCAST; /* 组播使用广播地址 */
        g_img_cache.completed_count = 0;

        for (uint8_t i = 0; i < n_targets; i++) {
            g_img_cache.target_addrs[i] = ((uint16_t)write_cb->value[3 + 2 * i] << 8)
                                         | write_cb->value[4 + 2 * i];
            g_img_cache.target_done[i] = 0;
            osal_printk("%s MCAST target[%d] = 0x%04X\r\n",
                        BLE_GW_LOG, i, g_img_cache.target_addrs[i]);
        }

        /* 解析图片元信息 (偏移 = 3 + 2*N) */
        uint16_t off = addr_end;
        g_img_cache.total_bytes = ((uint16_t)write_cb->value[off] << 8) | write_cb->value[off + 1];
        g_img_cache.pkt_count   = ((uint16_t)write_cb->value[off + 2] << 8) | write_cb->value[off + 3];
        g_img_cache.fc_pkt_count = (g_img_cache.total_bytes + IMG_FC_PKT_PAYLOAD - 1) / IMG_FC_PKT_PAYLOAD;

        g_img_cache.xfer_mode = 0;  /* 组播固定使用 FAST 模式 */
        g_img_cache.active = 1;
        g_fc.state = FC_IDLE;
        g_mesh_log_suppress = false;
        sle_uart_resume_scan();

        osal_printk("%s MCAST START: %d targets, total=%dB pkts=%d→%d\r\n",
                    BLE_GW_LOG, n_targets, g_img_cache.total_bytes,
                    g_img_cache.pkt_count, g_img_cache.fc_pkt_count);

        /* 构造 Mesh MCAST_START 帧并广播:
         * [0x0A, N, ADDR1(2)...ADDRn(2), TOTAL(2), PKT(2), W(2), H(2), MODE, XFER]
         * PKT 用 FC 大包数覆盖 (与单播 START 一致) */
        uint8_t mesh_buf[250];
        uint16_t mpos = 0;
        mesh_buf[mpos++] = 0x0A;  /* IMG_CMD_MCAST_START */
        mesh_buf[mpos++] = n_targets;
        for (uint8_t i = 0; i < n_targets; i++) {
            mesh_buf[mpos++] = (g_img_cache.target_addrs[i] >> 8) & 0xFF;
            mesh_buf[mpos++] = g_img_cache.target_addrs[i] & 0xFF;
        }
        /* 复制图片元信息 (TOTAL, PKT, W, H, MODE, [XFER]) */
        uint16_t meta_len = write_cb->length - addr_end;
        if (meta_len > sizeof(mesh_buf) - mpos) meta_len = sizeof(mesh_buf) - mpos;
        (void)memcpy_s(&mesh_buf[mpos], sizeof(mesh_buf) - mpos,
                       &write_cb->value[addr_end], meta_len);
        /* 覆盖 PKT 为 FC 大包数 */
        mesh_buf[mpos + 2] = (g_img_cache.fc_pkt_count >> 8) & 0xFF;
        mesh_buf[mpos + 3] = g_img_cache.fc_pkt_count & 0xFF;
        /* 强制 FAST 模式 (组播不支持 ACK 模式) */
        if (meta_len >= 10) {
            mesh_buf[mpos + 9] = 0x00;  /* IMG_XFER_FAST */
        }
        mpos += meta_len;

        /* F23: 保存 MCAST_START 帧，用于后续重播给迟到节点 */
        (void)memcpy_s(g_img_cache.mcast_start_frame, sizeof(g_img_cache.mcast_start_frame),
                       mesh_buf, mpos);
        g_img_cache.mcast_start_len = mpos;

        /* 广播到全网，所有目标节点将检查自己是否在列表中 */
        mesh_broadcast(mesh_buf, mpos);

        /* 网关自身若在目标列表中，直接标记完成 (网关既是发送方也是接收方，
         * 数据已在 g_img_cache.buf 中，无需等待自己给自己发 RESULT) */
        for (uint8_t i = 0; i < n_targets; i++) {
            if (g_img_cache.target_addrs[i] == g_mesh_node_addr) {
                g_img_cache.target_done[i] = 1;  /* 成功 */
                g_img_cache.completed_count++;
                osal_printk("%s MCAST: gateway self 0x%04X auto-done (%d/%d)\r\n",
                            BLE_GW_LOG, g_mesh_node_addr,
                            g_img_cache.completed_count, g_img_cache.target_count);
                mcast_notify_progress(g_mesh_node_addr, 0x00);
                break;
            }
        }

    } else if (cmd >= 0x04 && cmd <= 0x07) {
        /* ── 图片传输命令 (0x04-0x07) ── */
        if (write_cb->length < 4) return;
        uint16_t dst = ((uint16_t)write_cb->value[2] << 8) | write_cb->value[3];
        uint16_t rest_len = write_cb->length - 4;

        /* 构造 Mesh 负载: [CMD(1)] + [REST_DATA(N)]
         * 将 BLE 帧的指令码和负载复制到 mesh_buf 中统一注入 */
        uint8_t mesh_buf[250];
        mesh_buf[0] = cmd;
        if (rest_len > 0 && rest_len < sizeof(mesh_buf) - 1) {
            (void)memcpy_s(&mesh_buf[1], sizeof(mesh_buf) - 1,
                           &write_cb->value[4], rest_len);
        }
        uint16_t mesh_len = 1 + rest_len;

        /* ── 缓存与流控逻辑 ── */
        if (cmd == 0x04) {
            /* IMG_START (0x04): 初始化缓存 + 立即注入 Mesh (让目标节点先做准备) */
            (void)memset_s(&g_img_cache, sizeof(g_img_cache), 0, sizeof(g_img_cache));
            g_img_cache.dst_addr = dst;
            if (rest_len >= 4) {
                g_img_cache.total_bytes = ((uint16_t)write_cb->value[4] << 8) | write_cb->value[5];
                g_img_cache.pkt_count   = ((uint16_t)write_cb->value[6] << 8) | write_cb->value[7];
            }
            /* O4: 计算 FC 大包分包数 (大包负载 vs APP 原始 200B 小包) */
            g_img_cache.fc_pkt_count = (g_img_cache.total_bytes + IMG_FC_PKT_PAYLOAD - 1) / IMG_FC_PKT_PAYLOAD;
            /* v2.1: 解析传输模式 (xfer_mode)
             * APP 帧的 IMG_START 完整格式:
             *   [AA] [04] [DST_HI DST_LO] [TOTAL_HI TOTAL_LO] [PKT_HI PKT_LO]
             *   [W_HI W_LO] [H_HI H_LO] [MODE] [XFER]
             * rest 从 value[4] 开始, xfer 在 rest[9] = value[13]
             * 当 rest_len >= 10 时才包含 xfer_mode 字段
             * xfer_mode: 0=FAST(默认), 1=ACK代理模式 */
            if (rest_len >= 10) {
                g_img_cache.xfer_mode = write_cb->value[13];
            } else {
                g_img_cache.xfer_mode = 0;  /* 默认 FAST 模式 (无逽包ACK, 网关流控注入) */
            }
            if (rest_len <= sizeof(g_img_cache.start_payload)) {
                (void)memcpy_s(g_img_cache.start_payload, sizeof(g_img_cache.start_payload),
                               &write_cb->value[4], rest_len);
                g_img_cache.start_len = (uint8_t)rest_len;
            }
            g_img_cache.active = 1;
            g_fc.state = FC_IDLE;  /* 重置流控状态，确保上次传输的残留状态不干扰 */
            g_mesh_log_suppress = false;
            sle_uart_resume_scan();  /* P5: START 重置时确保扫描已恢复 (上次传输可能未正常结束) */

            osal_printk("%s IMG START cached: dst=0x%04X total=%dB pkts=%d→%d xfer=%s\r\n",
                        BLE_GW_LOG, dst, g_img_cache.total_bytes,
                        g_img_cache.pkt_count, g_img_cache.fc_pkt_count,
                        g_img_cache.xfer_mode ? "ACK" : "FAST");

            /* v2.2: ACK代理模式 —— 告诉目标节点使用 FAST 模式接收
             * (网关代理ACK给APP，Mesh层走流控引擎)
             * mesh_buf 字节位置:
             * [0x04, TOTAL(2), PKT(2), W(2), H(2), MODE(1), XFER(1)]
             *  [0]   [1-2]     [3-4]   [5-6] [7-8] [9]      [10]     */
            if (g_img_cache.xfer_mode == 1 && mesh_len >= 11) {
                mesh_buf[10] = 0x00;  /* IMG_XFER_FAST: 目标节点以 FAST 模式接收 (无逐包ACK) */
                osal_printk("%s IMG START: ACK-proxy mode, target will use FAST\r\n", BLE_GW_LOG);
            }

            /* O4: 覆盖 START 帧中的 pkt_count 为 FC 大包数，目标节点按大包分配接收缓冲 */
            if (mesh_len >= 5) {
                mesh_buf[3] = (g_img_cache.fc_pkt_count >> 8) & 0xFF;
                mesh_buf[4] = g_img_cache.fc_pkt_count & 0xFF;
            }
            /* START 立即注入 Mesh，让目标节点提前初始化接收缓冲区 */
            mesh_gateway_inject(dst, mesh_buf, mesh_len);

        } else if (cmd == 0x05 && g_img_cache.active) {
            /* IMG_DATA (0x05): 将图片数据包缓存到本地 buf，不立即注入 Mesh */
            if (rest_len >= 3) {
                uint16_t seq = ((uint16_t)write_cb->value[4] << 8) | write_cb->value[5];
                uint8_t  dlen = write_cb->value[6];
                uint32_t offset = (uint32_t)seq * IMG_GW_PKT_PAYLOAD;
                if (offset + dlen <= IMG_GW_CACHE_SIZE && dlen + 3 <= rest_len) {
                    (void)memcpy_s(&g_img_cache.buf[offset], IMG_GW_CACHE_SIZE - offset,
                                   &write_cb->value[7], dlen);
                }
            }

            if (g_img_cache.xfer_mode == 1) {
                /* v2.2: ACK代理模式 —— 不注入 Mesh，发本地 ACK 给 APP 让其继续发下一包 */
                uint16_t seq = (rest_len >= 2) ?
                    ((uint16_t)write_cb->value[4] << 8 | write_cb->value[5]) : 0;
                gw_send_local_ack(seq);
                /* 数据缓存在网关 buf 中，END 后由流控引擎统一发送到目标 */
            }
            /* FAST 模式: 同样不注入 —— 所有数据都缓存，等 END 后流控发送 */

        } else if (cmd == 0x06 && g_img_cache.active) {
            /* IMG_END (0x06): 提取 CRC16，启动流控引擎 */
            if (rest_len >= 2) {
                g_img_cache.crc16 = ((uint16_t)write_cb->value[4] << 8) | write_cb->value[5];
            }

            if (g_img_cache.xfer_mode == 1) {
                /* v2.2: ACK代理模式 —— 数据已全部缓存，启动流控引擎
                 * 目标节点以 FAST 模式接收，与 FAST 模式相同的流控路径 */
                osal_printk("%s IMG END: ACK-proxy mode, cached %d pkts → starting flow control\r\n",
                            BLE_GW_LOG, g_img_cache.pkt_count);
                fc_start();
                /* 不直接注入 END —— 等流控完成后再发 END 帧到目标 */
            } else {
                /* FAST 模式: 缓存 CRC 并启动流控注入引擎 */
                osal_printk("%s IMG END cached: crc=0x%04X → starting flow control\r\n",
                            BLE_GW_LOG, g_img_cache.crc16);
                fc_start();
                /* 不注入 END —— 流控完成后再发 END 帧到目标 */
            }

        } else if (cmd == 0x07) {
            /* IMG_CANCEL (0x07): 中止传输，停止流控 + 清理状态 + 将取消命令注入 Mesh */
            g_fc.state = FC_IDLE;                /* 重置流控状态机 */
            g_mesh_log_suppress = false;           /* 恢复 Mesh 日志输出 */
            sle_uart_resume_scan();                /* P5: 恢复 SLE 扫描 */
            fc_stop_turbo_now();                   /* O2: 恢复默认连接间隔 (立即) */
            g_img_cache.active = 0;                /* 清除缓存标记 */
            osal_printk("%s IMG CANCEL: FC stopped, cache cleared\r\n", BLE_GW_LOG);
            mesh_gateway_inject(dst, mesh_buf, mesh_len);
        }

        /* 日志输出: 仅在未被 P1 抑制时打印 IMG_DATA 序列号及模式 */
        if (cmd == 0x05) {
            uint16_t seq = (rest_len >= 2) ?
                ((uint16_t)write_cb->value[4] << 8 | write_cb->value[5]) : 0;
            if (!g_mesh_log_suppress) {
                osal_printk("%s IMG_DATA %s seq=%d\r\n", BLE_GW_LOG,
                            (g_img_cache.xfer_mode == 1) ? "cached+localACK" : "cached", seq);
            }
        }

    } else if (write_cb->length >= 5) {
        /* 常规命令: 非图片传输的普通 Mesh 注入 (如 0x01 显示文本、 0x02 LED 控制等) */
        uint16_t dst = ((uint16_t)write_cb->value[2] << 8) | write_cb->value[3];
        uint8_t  payload_len = write_cb->value[4];
        const uint8_t *payload = &write_cb->value[5];
        if (payload_len + 5 > write_cb->length) return;

        osal_printk("%s cmd=0x%02X dst=0x%04X len=%d\r\n", BLE_GW_LOG, cmd, dst, payload_len);
        switch (cmd) {
            case 0x01:
            case 0x02:
                mesh_gateway_inject(dst, payload, payload_len);
                break;
            default:
                break;
        }
    }

    (void)server_id; (void)conn_id; (void)status;
}

/* ===========================================================================
 * 其他 GATT / GAP 回调
 * ---------------------------------------------------------------------------
 * 包含: 读请求(空实现)、MTU 协商、广播状态、连接/断连、配对结果。
 * =========================================================================== */

/**
 * @brief 读请求回调 (空实现)
 *
 * 当前未注册可读特征, 不会触发。留作将来扩展。
 */
static void gw_read_req_cbk(uint8_t server_id, uint16_t conn_id,
                              gatts_req_read_cb_t *read_cb, errcode_t status)
{
    (void)server_id; (void)conn_id; (void)read_cb; (void)status;
}

/**
 * @brief MTU 协商完成回调
 *
 * BLE 连接后由 Gateway 主动请求 MTU=247, 协商结果在此回调中更新。
 * g_gw_mtu 影响 ble_gateway_send_to_phone() 的最大 Notify 负载大小。
 *
 * @param server_id  GATT 服务器 ID
 * @param conn_id    连接句柄
 * @param mtu_size   协商后的 MTU 大小 (字节)
 * @param status     协商结果, 0=成功
 */
static void gw_mtu_changed_cbk(uint8_t server_id, uint16_t conn_id,
                                 uint16_t mtu_size, errcode_t status)
{
    osal_printk("%s MTU changed: %d\r\n", BLE_GW_LOG, mtu_size);
    if (status == 0 && mtu_size > 0) g_gw_mtu = mtu_size;
    (void)server_id;
}

/** @brief 广播启用回调 — 日志记录广播 ID 和状态 */
static void gw_adv_enable_cbk(uint8_t adv_id, adv_status_t status)
{
    osal_printk("%s adv enable adv_id=%d status=%d\r\n", BLE_GW_LOG, adv_id, status);
}

/** @brief 广播停止回调 — 日志记录广播 ID 和状态 */
static void gw_adv_disable_cbk(uint8_t adv_id, adv_status_t status)
{
    osal_printk("%s adv disable adv_id=%d status=%d\r\n", BLE_GW_LOG, adv_id, status);
}

/**
 * @brief BLE 连接状态变化回调
 *
 * 处理两种关键事件:
 * 1. 连接 (GAP_BLE_STATE_CONNECTED):
 *    - 记录 conn_id, 标记已连接
 *    - 主动发起 MTU 协商 (247B)
 * 2. 断连 (GAP_BLE_STATE_DISCONNECTED):
 *    - 中止流控引擎, 恢复扫描与连接间隔
 *    - 清除图片缓存, 重启广播等待下一次连接
 *
 * @param conn_id      连接句柄
 * @param addr         对端设备地址
 * @param conn_state   新的连接状态 (连接 / 断连)
 * @param pair_state   配对状态 (此处未使用)
 * @param disc_reason  断连原因码 (此处未使用)
 */
static void gw_conn_state_change_cbk(uint16_t conn_id, bd_addr_t *addr,
                                      gap_ble_conn_state_t conn_state,
                                      gap_ble_pair_state_t pair_state,
                                      gap_ble_disc_reason_t disc_reason)
{
    osal_printk("%s BLE conn change: conn_id=%d state=%d\r\n", BLE_GW_LOG, conn_id, conn_state);
    if (conn_state == GAP_BLE_STATE_CONNECTED) {
        g_gw_conn_id = conn_id;
        g_gw_connected = 1;
        osal_printk("%s ====== PHONE CONNECTED ======\r\n", BLE_GW_LOG);
        gatts_set_mtu_size(conn_id, 247);  /* 主动请求最大 MTU */
    } else if (conn_state == GAP_BLE_STATE_DISCONNECTED) {
        g_gw_connected = 0;
        g_gw_needs_reconnect = 1;
        /* 断连时: 全面清理传输状态 */
        g_fc.state = FC_IDLE;              /* 重置流控状态机 */
        g_mesh_log_suppress = false;       /* 恢复 Mesh 日志 */
        sle_uart_resume_scan();            /* P5: 恢复 SLE 扫描 */
        fc_stop_turbo_now();               /* O2: 恢复默认连接间隔 (立即) */
        g_img_cache.active = 0;            /* 清除图片缓存标记 */
        osal_printk("%s phone disconnected, restart adv\r\n", BLE_GW_LOG);
        gw_set_adv_data();                 /* 重新设置广播数据 */
        gw_start_adv();                    /* 重启广播等待新连接 */
    }
    (void)addr; (void)pair_state; (void)disc_reason;
}

/** @brief 配对结果回调 — 当前无安全要求, 空实现 */
static void gw_pair_result_cbk(uint16_t conn_id, const bd_addr_t *addr, errcode_t status)
{
    (void)conn_id; (void)addr; (void)status;
}

/* ===========================================================================
 * BLE 网关初始化
 * ---------------------------------------------------------------------------
 * 完整初始化序列:
 *   1. 生成设备名称  sle_gw_XXXX (基于 Mesh 地址)
 *   2. 注册 GAP 回调 (广播、连接、配对)
 *   3. 注册 GATTS 回调 (Service/Char/Desc 添加、读写请求、MTU)
 *   4. 使能 BLE 协议栈
 *   5. 设置 BLE MAC 地址 (CC:BB:AA:00:低:高, 基于 Mesh 地址)
 *   6. 注册 GATT Server → 添加 Service (UUID 0xFFE0)
 *      后续通过回调链自动添加 TX/RX 特征值和描述符
 * 调用时机: 系统启动后、SLE Mesh 初始化完成后
 * =========================================================================== */
void ble_gateway_init(void)
{
    errcode_t ret;
    osal_printk("%s === BLE Gateway init start ===\r\n", BLE_GW_LOG);

    /* Step 1: 生成 BLE 设备名称 sle_gw_XXXX (基于 Mesh 地址确保唯一) */
    uint16_t my_addr = mesh_get_my_addr();
    int name_len = snprintf_s((char *)g_gw_adv_name, MAX_NAME_LENGTH, MAX_NAME_LENGTH - 1,
                               "sle_gw_%04X", my_addr);
    if (name_len > 0) g_gw_adv_name_len = (uint8_t)name_len;
    osal_printk("%s BLE name: %s\r\n", BLE_GW_LOG, g_gw_adv_name);

    /* Step 2: 注册 GAP 回调 (广播启停、连接状态、配对结果) */
    gap_ble_callbacks_t gap_cb = { 0 };
    gap_cb.start_adv_cb = gw_adv_enable_cbk;
    gap_cb.stop_adv_cb = gw_adv_disable_cbk;
    gap_cb.conn_state_change_cb = gw_conn_state_change_cbk;
    gap_cb.pair_result_cb = gw_pair_result_cbk;
    ret = gap_ble_register_callbacks(&gap_cb);

    /* Step 3: 注册 GATTS 回调 (Service/Char/Desc 添加、读写请求、MTU 协商) */
    gatts_callbacks_t svc_cb = { 0 };
    svc_cb.add_service_cb = gw_service_add_cbk;
    svc_cb.add_characteristic_cb = gw_char_add_cbk;
    svc_cb.add_descriptor_cb = gw_desc_add_cbk;
    svc_cb.start_service_cb = gw_service_start_cbk;
    svc_cb.read_request_cb = gw_read_req_cbk;
    svc_cb.write_request_cb = gw_write_req_cbk;
    svc_cb.mtu_changed_cb = gw_mtu_changed_cbk;
    ret = gatts_register_callbacks(&svc_cb);

    /* Step 4: 使能 BLE 协议栈 (需等待 500ms 等硬件就绪) */
    ret = enable_ble();
    osal_msleep(500);

    /* Step 5: 设置 BLE MAC 地址 CC:BB:AA:00:低:高 (基于 Mesh 地址生成) */
    g_gw_ble_addr[0] = 0xCC;
    g_gw_ble_addr[1] = 0xBB;
    g_gw_ble_addr[2] = 0xAA;
    g_gw_ble_addr[3] = 0x00;
    g_gw_ble_addr[4] = (uint8_t)(my_addr & 0xFF);
    g_gw_ble_addr[5] = (uint8_t)(my_addr >> 8);
    osal_printk("%s BLE addr: %02X:%02X:%02X:%02X:%02X:%02X\r\n", BLE_GW_LOG,
        g_gw_ble_addr[5], g_gw_ble_addr[4], g_gw_ble_addr[3],
        g_gw_ble_addr[2], g_gw_ble_addr[1], g_gw_ble_addr[0]);

    /* Step 5b: 应用地址和名称到 BLE 协议栈 */
    bd_addr_t ble_addr = { 0 };
    ble_addr.type = 0;  /* 公共地址类型 */
    (void)memcpy_s(ble_addr.addr, BD_ADDR_LEN_LOCAL, g_gw_ble_addr, sizeof(g_gw_ble_addr));
    gap_ble_set_local_name(g_gw_adv_name, g_gw_adv_name_len);
    gap_ble_set_local_addr(&ble_addr);

    /* Step 6: 注册 GATT Server 并添加服务 (UUID 0xFFE0)
     * 服务添加后会触发 gw_service_add_cbk → 自动添加 TX/RX 特征值 */
    bt_uuid_t app_uuid = { 0 };
    app_uuid.uuid_len = sizeof(g_gw_app_uuid);
    (void)memcpy_s(app_uuid.uuid, app_uuid.uuid_len, g_gw_app_uuid, sizeof(g_gw_app_uuid));
    ret = gatts_register_server(&app_uuid, &g_gw_server_id);

    bt_uuid_t svc_uuid = { 0 };
    uuid16_to_bt_uuid(BLE_GW_UUID_SERVICE, &svc_uuid);
    gatts_add_service(g_gw_server_id, &svc_uuid, true);

    osal_printk("%s === BLE Gateway init done ===\r\n", BLE_GW_LOG);
    (void)ret;
}

/* ===========================================================================
 * 数据接口 (供外部模块调用)
 * ---------------------------------------------------------------------------
 * ble_gateway_is_connected()    - 查询 BLE 是否已连接手机
 * ble_gateway_needs_reconnect() - 查询断连后是否需要重连通知
 * ble_gateway_clear_reconnect() - 清除重连标记
 * ble_gateway_send_to_phone()   - 向手机发送 Mesh 上报数据
 * =========================================================================== */

/** @brief 查询 BLE 连接状态 */
bool ble_gateway_is_connected(void) { return g_gw_connected != 0; }
/** @brief 查询是否需要重连通知 (断连后置位, 由上层清除) */
bool ble_gateway_needs_reconnect(void) { return g_gw_needs_reconnect != 0; }
/** @brief 清除重连标记 */
void ble_gateway_clear_reconnect(void) { g_gw_needs_reconnect = 0; }

/**
 * @brief 向手机发送 Mesh 上报数据
 *
 * 将 Mesh 节点上报的数据封装为统一帧格式通过 BLE Notify 发送给手机 APP。
 *
 * 帧格式: [0xAA][CMD=0x81][SRC_HI][SRC_LO][LEN][PAYLOAD...]
 *  - 0xAA:     帧头标识
 *  - 0x81:     上行数据命令码 (MESH_TO_PHONE)
 *  - SRC:      源节点 Mesh 地址 (大端)
 *  - LEN:      有效载荷长度
 *  - PAYLOAD:  原始数据内容
 *
 * @param src_addr 源节点 Mesh 地址
 * @param data     待发送的有效载荷
 * @param len      有效载荷长度 (字节)
 * @return  0: 成功, -1: 未连接或句柄无效, -3: Notify 发送失败
 *
 * @note 自动按 MTU 裁剪: 最大负载 = MTU - 3(ATT头) - 5(帧头)
 */
int ble_gateway_send_to_phone(uint16_t src_addr, const uint8_t *data, uint16_t len)
{
    if (!g_gw_connected || g_gw_notify_handle == 0) return -1;

    uint16_t max_payload = (g_gw_mtu > 3) ? (g_gw_mtu - 3) : 20;  /* ATT 协议预留 3B */
    uint16_t header_size = 5;                                       /* 帧头固定 5 字节 */
    uint16_t max_data = max_payload - header_size;                  /* 可用载荷空间 */
    if (len > max_data) len = max_data;                             /* 超长自动裁剪 */

    /* 组装上行帧: [0xAA][0x81][SRC_HI][SRC_LO][LEN][DATA...] */
    uint8_t frame[250];
    frame[0] = 0xAA;                              /* 帧头标识 */
    frame[1] = 0x81;                              /* 命令码: MESH_TO_PHONE */
    frame[2] = (src_addr >> 8) & 0xFF;            /* 源地址高字节 */
    frame[3] = src_addr & 0xFF;                   /* 源地址低字节 */
    frame[4] = (uint8_t)len;                      /* 有效载荷长度 */
    (void)memcpy_s(&frame[5], sizeof(frame) - 5, data, len);

    /* 通过 GATT Notify 发送到手机 APP */
    gatts_ntf_ind_t param = { 0 };
    param.attr_handle = g_gw_notify_handle;
    param.value_len = header_size + len;
    param.value = frame;

    errcode_t ret = gatts_notify_indicate(g_gw_server_id, g_gw_conn_id, &param);
    if (ret != 0) return -3;
    return 0;
}