/**
 * @file  sle_uart_server_adv.c
 * @brief SLE UART Server 广播（Advertising）模块实现
 *
 * @details 本文件实现 SLE 广播的完整配置与管理，包括：
 *          1. 广播数据组装: discovery_level + access_mode (adv_data)
 *          2. 扫描响应数据: TX_power + local_name   (scan_rsp_data)
 *          3. 广播参数设置: 间隔、通道、TX 功率、连接参数
 *          4. 本地地址与广播名生成（Mesh 模式下从 g_mesh_node_addr 派生）
 *          5. 广播回调注册: enable / disable / terminal
 *
 *          Mesh 核心要点:
 *          ┌───────────────────────────────────────────────────────────────┐
 *          │ g_local_addr[4..5] = g_mesh_node_addr 的低/高字节（小端序）      │
 *          │ 对端通过 mesh_transport_addr_from_sle() 读取这两字节推导 mesh_addr │
 *          │ 广播名 = "sle_m_XXXX"，XXXX 为 mesh_addr 的 hex 表示              │
 *          │ 每次有新连接建立后 SLE 自动停止广播，需通过 g_need_re_announce     │
 *          │ 标志在主循环中重启，以允许后续节点继续连入                          │
 *          └───────────────────────────────────────────────────────────────┘
 *
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * History: \n
 * 2023-07-17, Create file. \n
 * 2026-02-07, Modified for Mesh support: 动态地址/广播名、terminal 回调重启. \n
 */
#include "securec.h"
#include "errcode.h"
#include "osal_addr.h"
#include "product.h"
#include "sle_common.h"
#include "sle_uart_server.h"
#include "sle_device_discovery.h"
#include "sle_errcode.h"
#include "osal_debug.h"
#include "osal_task.h"
#include "string.h"
#include "sle_uart_server_adv.h"

#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
#include "mesh_config.h"   /* g_mesh_node_addr, MESH_ADV_NAME_PREFIX, SLE_ADDR_INDEX */
#include "efuse.h"         /* uapi_efuse_init, uapi_efuse_get_die_id */
#endif

/* ============================================================
 *  常量宏定义 —— 广播与连接参数
 *
 *  注意: SLE 间隔/超时的单位与 BLE 不同:
 *        - 连接间隔单位: 125μs (0x64 = 100 × 125μs = 12.5ms)
 *        - 广播间隔单位: 125μs (0xC8 = 200 × 125μs = 25ms)
 *        - 超时单位:     10ms  (0x1F4 = 500 × 10ms = 5000ms)
 * ============================================================ */
#define NAME_MAX_LENGTH 16                                  /**< 广播本地名称最大长度（含 '\0'） */
#define SLE_CONN_INTV_MIN_DEFAULT                 0x64      /**< 连接间隔下限: 0x64 × 125μs = 12.5ms */
#define SLE_CONN_INTV_MAX_DEFAULT                 0x64      /**< 连接间隔上限: 0x64 × 125μs = 12.5ms (与 min 相同则固定间隔) */
#define SLE_ADV_INTERVAL_MIN_DEFAULT              0xC8      /**< 广播间隔下限: 0xC8 × 125μs = 25ms; 减小可加快被发现速度但增加功耗 */
#define SLE_ADV_INTERVAL_MAX_DEFAULT              0xC8      /**< 广播间隔上限: 与 min 相同则固定 25ms */
#define SLE_CONN_SUPERVISION_TIMEOUT_DEFAULT      0x1F4     /**< 连接监督超时: 0x1F4 × 10ms = 5000ms; 超时未收到包则断连 */
#define SLE_CONN_MAX_LATENCY                      0x1F3     /**< 最大连接延迟: 从设备可跳过的连接间隔数; 0x1F3 ≈ 499 (极限省电) */
#define SLE_ADV_TX_POWER  10                                /**< 广播 TX 功率 (dBm); 扫描响应中以 TLV 形式携带 */
#define SLE_ADV_HANDLE_DEFAULT                    1         /**< 广播句柄 ID; SLE 协议栈支持多组广播, 本模块仅使用 1 组 */
#define SLE_ADV_DATA_LEN_MAX                      251       /**< 广播/扫描响应数据最大长度 (字节) */

/* ============================================================
 *  广播名称与本地地址
 *
 *  Mesh 模式:
 *    - sle_local_name = "sle_m_XXXX" (XXXX = hex(g_mesh_node_addr))
 *      在 sle_uart_server_adv_init() 中由 snprintf 动态生成
 *    - g_local_addr[4..5] = g_mesh_node_addr 小端序
 *      对端 Client 通过 mesh_transport_addr_from_sle() 读后两字节推导地址
 *
 *  非 Mesh 模式: 使用固定字符串和固定地址，无动态生成
 * ============================================================ */
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
static uint8_t sle_local_name[NAME_MAX_LENGTH] = "sle_mesh_0001";  /**< 运行时被 adv_init 覆写 */
#else
static uint8_t sle_local_name[NAME_MAX_LENGTH] = "sle_uart_server";
#endif

#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
static unsigned char g_local_addr[SLE_ADDR_LEN] = {
    0x01, 0x02, 0x03, 0x04, 0x00, 0x00  /**< [4],[5] 在 adv_init 时从 g_mesh_node_addr 填充 */
};
#else
static unsigned char g_local_addr[SLE_ADDR_LEN] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x01 };
#endif

#define SLE_SERVER_INIT_DELAY_MS    1000                        /**< Server 初始化延迟 (ms) */
#define sample_at_log_print(fmt, args...) osal_printk(fmt, ##args)  /**< 日志输出宏 */
#define SLE_UART_SERVER_LOG "[sle uart server]"                 /**< 日志前缀标签 */

/**
 * @brief  向广播/扫描响应数据缓冲区中写入本地名称 TLV 字段
 *
 * @param  adv_data  目标缓冲区指针（当前写入位置）
 * @param  max_len   缓冲区剩余可用长度
 * @return 写入的字节数（含 length + type + name）；失败返回 0
 *
 * @note   名称来源: sle_local_name[]，Mesh 模式下已由 adv_init 填充为 "sle_m_XXXX"
 */
static uint16_t sle_set_adv_local_name(uint8_t *adv_data, uint16_t max_len)
{
    errno_t ret;
    uint8_t index = 0;

    uint8_t *local_name = sle_local_name;
    uint8_t local_name_len = (uint8_t)strlen((const char *)sle_local_name);
    sample_at_log_print("%s local_name_len = %d\r\n", SLE_UART_SERVER_LOG, local_name_len);
    sample_at_log_print("%s local_name: %s\r\n", SLE_UART_SERVER_LOG, local_name);

    adv_data[index++] = local_name_len + 1;
    adv_data[index++] = SLE_ADV_DATA_TYPE_COMPLETE_LOCAL_NAME;
    ret = memcpy_s(&adv_data[index], max_len - index, local_name, local_name_len);
    if (ret != EOK) {
        sample_at_log_print("%s memcpy fail\r\n", SLE_UART_SERVER_LOG);
        return 0;
    }
    return (uint16_t)index + local_name_len;
}

/**
 * @brief  组装广播数据（adv_data）—— 填充 discovery_level + access_mode
 *
 * @param  adv_data  广播数据缓冲区指针，至少 SLE_ADV_DATA_LEN_MAX 字节
 * @return 已写入的字节数；出错返回 0
 *
 * @note   广播数据主要用于被扫描端设备过滤和识别:  
 *         - discovery_level = NORMAL: 允许常规扫描发现  
 *         - access_mode = 0: 无特殊接入层限制
 */
static uint16_t sle_set_adv_data(uint8_t *adv_data)
{
    size_t len = 0;
    uint16_t idx = 0;
    errno_t  ret = 0;

    len = sizeof(struct sle_adv_common_value);
    struct sle_adv_common_value adv_disc_level = {
        .length = len - 1,
        .type = SLE_ADV_DATA_TYPE_DISCOVERY_LEVEL,
        .value = SLE_ANNOUNCE_LEVEL_NORMAL,
    };
    ret = memcpy_s(&adv_data[idx], SLE_ADV_DATA_LEN_MAX - idx, &adv_disc_level, len);
    if (ret != EOK) {
        sample_at_log_print("%s adv_disc_level memcpy fail\r\n", SLE_UART_SERVER_LOG);
        return 0;
    }
    idx += len;

    len = sizeof(struct sle_adv_common_value);
    struct sle_adv_common_value adv_access_mode = {
        .length = len - 1,
        .type = SLE_ADV_DATA_TYPE_ACCESS_MODE,
        .value = 0,
    };
    ret = memcpy_s(&adv_data[idx], SLE_ADV_DATA_LEN_MAX - idx, &adv_access_mode, len);
    if (ret != EOK) {
        sample_at_log_print("%s adv_access_mode memcpy fail\r\n", SLE_UART_SERVER_LOG);
        return 0;
    }
    idx += len;

    return idx;
}

/**
 * @brief  组装扫描响应数据（scan_rsp_data）—— TX power + local_name
 *
 * @param  scan_rsp_data  扫描响应缓冲区指针，至少 SLE_ADV_DATA_LEN_MAX 字节
 * @return 已写入的字节数
 *
 * @note   扫描响应在 Client 主动请求时返回，包含:
 *         1. TX 功率等级（供对端估算距离/信号强度）
 *         2. 完整本地名称（Mesh 模式下含节点地址后缀）
 */
static uint16_t sle_set_scan_response_data(uint8_t *scan_rsp_data)
{
    uint16_t idx = 0;
    errno_t ret;
    size_t scan_rsp_data_len = sizeof(struct sle_adv_common_value);

    struct sle_adv_common_value tx_power_level = {
        .length = scan_rsp_data_len - 1,
        .type = SLE_ADV_DATA_TYPE_TX_POWER_LEVEL,
        .value = SLE_ADV_TX_POWER,
    };
    ret = memcpy_s(scan_rsp_data, SLE_ADV_DATA_LEN_MAX, &tx_power_level, scan_rsp_data_len);
    if (ret != EOK) {
        sample_at_log_print("%s scan response data memcpy fail\r\n", SLE_UART_SERVER_LOG);
        return 0;
    }
    idx += scan_rsp_data_len;

    /* set local name */
    idx += sle_set_adv_local_name(&scan_rsp_data[idx], SLE_ADV_DATA_LEN_MAX - idx);
    return idx;
}

/* ============================================================
 *  广播参数与数据配置
 * ============================================================ */

/**
 * @brief  设置默认广播参数: 模式、间隔、通道、功率、连接参数、本地地址
 *
 * @return ERRCODE_SLE_SUCCESS 成功
 *
 * @note   参数说明:
 *         - announce_mode = CONNECTABLE_SCANABLE: 可被连接且可被扫描
 *         - announce_tx_power = 18 dBm: 实际发射功率（与 adv_data 中的
 *           TX_POWER_LEVEL 字段独立，后者仅为信息通告）
 *         - own_addr: 从 g_local_addr 拷贝，Mesh 模式下后两字节已= mesh_addr
 *
 *         调优建议:
 *         - 增大 announce_interval → 降低功耗但被发现变慢
 *         - 增大 announce_tx_power → 增大覆盖范围但增加干扰
 *         - 减小 conn_interval → 降低传输延迟但增加功耗
 */
static int sle_set_default_announce_param(void)
{
    errno_t ret;
    sle_announce_param_t param = {0};
    uint8_t index;

    param.announce_mode = SLE_ANNOUNCE_MODE_CONNECTABLE_SCANABLE;
    param.announce_handle = SLE_ADV_HANDLE_DEFAULT;
    param.announce_gt_role = SLE_ANNOUNCE_ROLE_T_CAN_NEGO;
    param.announce_level = SLE_ANNOUNCE_LEVEL_NORMAL;
    param.announce_channel_map = SLE_ADV_CHANNEL_MAP_DEFAULT;
    param.announce_interval_min = SLE_ADV_INTERVAL_MIN_DEFAULT;
    param.announce_interval_max = SLE_ADV_INTERVAL_MAX_DEFAULT;
    param.conn_interval_min = SLE_CONN_INTV_MIN_DEFAULT;
    param.conn_interval_max = SLE_CONN_INTV_MAX_DEFAULT;
    param.conn_max_latency = SLE_CONN_MAX_LATENCY;
    param.conn_supervision_timeout = SLE_CONN_SUPERVISION_TIMEOUT_DEFAULT;
    param.announce_tx_power = 18;
    param.own_addr.type = 0;

    ret = memcpy_s(param.own_addr.addr, SLE_ADDR_LEN, g_local_addr, SLE_ADDR_LEN);
    if (ret != EOK) {
        sample_at_log_print("%s announce_param memcpy fail\r\n", SLE_UART_SERVER_LOG);
        return 0;
    }

    sample_at_log_print("%s local addr: ", SLE_UART_SERVER_LOG);
    for (index = 0; index < SLE_ADDR_LEN; index++) {
        sample_at_log_print("0x%02x ", param.own_addr.addr[index]);
    }
    sample_at_log_print("\r\n");

    return sle_set_announce_param(param.announce_handle, &param);
}

/**
 * @brief  设置默认广播数据: 组装 adv_data + scan_rsp_data 后注册到协议栈
 *
 * @return ERRCODE_SLE_SUCCESS 成功
 *
 * @note   广播数据结构:
 *         adv_data     = [discovery_level TLV] + [access_mode TLV]
 *         scan_rsp_data = [TX_power TLV] + [local_name TLV]
 */
static int sle_set_default_announce_data(void)
{
    errcode_t ret;
    uint8_t announce_data_len = 0;
    uint8_t seek_data_len = 0;
    sle_announce_data_t data = {0};
    uint8_t adv_handle = SLE_ADV_HANDLE_DEFAULT;
    uint8_t announce_data[SLE_ADV_DATA_LEN_MAX] = {0};
    uint8_t seek_rsp_data[SLE_ADV_DATA_LEN_MAX] = {0};

    announce_data_len = sle_set_adv_data(announce_data);
    data.announce_data = announce_data;
    data.announce_data_len = announce_data_len;

    seek_data_len = sle_set_scan_response_data(seek_rsp_data);
    data.seek_rsp_data = seek_rsp_data;
    data.seek_rsp_data_len = seek_data_len;

    ret = sle_set_announce_data(adv_handle, &data);
    if (ret == ERRCODE_SLE_SUCCESS) {
        sample_at_log_print("%s set announce data success.\r\n", SLE_UART_SERVER_LOG);
    } else {
        sample_at_log_print("%s set adv param fail.\r\n", SLE_UART_SERVER_LOG);
    }
    return ERRCODE_SLE_SUCCESS;
}

/* ============================================================
 *  广播事件回调
 *
 *  SLE 协议栈在广播状态变化时触发这些回调:
 *  - enable:   广播成功启动
 *  - disable:  广播成功停止
 *  - terminal: 广播被协议栈终止（如有新连接建立时自动停止）
 *
 *  Mesh 关键: terminal 回调中设置 g_need_re_announce = true，
 *  通知主循环重启广播以继续接受新节点连入。
 * ============================================================ */

/** @brief 广播启动成功回调 */
static void sle_announce_enable_cbk(uint32_t announce_id, errcode_t status)
{
    sample_at_log_print("%s announce enable id:%02x, state:%x\r\n",
        SLE_UART_SERVER_LOG, announce_id, status);
}

/** @brief 广播停止成功回调 */
static void sle_announce_disable_cbk(uint32_t announce_id, errcode_t status)
{
    sample_at_log_print("%s announce disable id:%02x, state:%x\r\n",
        SLE_UART_SERVER_LOG, announce_id, status);
}

/**
 * @brief  广播终止回调 —— SLE 协议栈因建连而自动停止广播时触发
 *
 * @param  announce_id  被终止的广播句柄 ID
 *
 * @note   Mesh 模式下不在回调中直接重启广播（避免 SLE 回调上下文中进行复杂操作），
 *         而是设置 g_need_re_announce 标志，由 mesh_main 主循环安全处理。
 */
static void sle_announce_terminal_cbk(uint32_t announce_id)
{
    sample_at_log_print("%s announce terminal id:%02x\r\n", SLE_UART_SERVER_LOG, announce_id);

#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
    /* P3: Mesh 需要持续广播以接受多个连接.
     * SLE 在有新连接时自动停止广播, 我们需要重启它.
     * 不在回调中直接操作, 设标志让主循环处理 (更安全) */
    extern volatile bool g_need_re_announce;
    g_need_re_announce = true;
#endif
}

/**
 * @brief  注册广播事件回调: enable / disable / terminal
 * @return ERRCODE_SLE_SUCCESS 成功
 */
errcode_t sle_uart_announce_register_cbks(void)
{
    errcode_t ret = 0;
    sle_announce_seek_callbacks_t seek_cbks = {0};
    seek_cbks.announce_enable_cb = sle_announce_enable_cbk;
    seek_cbks.announce_disable_cb = sle_announce_disable_cbk;
    seek_cbks.announce_terminal_cb = sle_announce_terminal_cbk;
    ret = sle_announce_seek_register_callbacks(&seek_cbks);
    if (ret != ERRCODE_SLE_SUCCESS) {
        sample_at_log_print("%s register_callbacks fail :%x\r\n", SLE_UART_SERVER_LOG, ret);
        return ret;
    }
    return ERRCODE_SLE_SUCCESS;
}

/* ============================================================
 *  广播初始化入口
 *
 *  调用链: sle_enable → enable_cbk → sle_uart_server_adv_init()
 *          或 mesh_main 断连后 → g_need_re_announce → adv_init()
 *
 *  Mesh 模式流程:
 *  1. 若 g_mesh_node_addr == 0（自动地址模式），从 EFUSE Die-ID 推导 16-bit 地址
 *  2. 将 mesh_addr 写入 g_local_addr[4..5]（小端序）
 *  3. 生成广播名 "sle_m_XXXX"
 *  4. 配置广播参数 → 配置广播数据 → 启动广播
 * ============================================================ */
errcode_t sle_uart_server_adv_init(void)
{
    errcode_t ret;

#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
    /* 自动地址模式: g_mesh_node_addr == 0 表示需要从芯片唯一标识推导.
     * 使用 EFUSE Die-ID（出厂烧写, 每颗芯片唯一, 重启不变）而非 SLE MAC,
     * 确保同一份固件烧录到不同板子 → 不同地址，且断电/reboot 后地址不变 */
    if (g_mesh_node_addr == 0) {
        uint8_t die_id[16] = {0};
        uint16_t addr = 0;

        /* 初始化 EFUSE 驱动（bootloader 已初始化, 此处为安全起见再调一次, 幂等操作） */
        (void)uapi_efuse_init();
        errcode_t efuse_ret = uapi_efuse_get_die_id(die_id, sizeof(die_id));

        if (efuse_ret == ERRCODE_SUCC) {
            /* 将 16 字节 Die-ID 折叠为 16-bit:
             * 偶数下标字节 XOR → 低 8 位, 奇数下标字节 XOR → 高 8 位
             * 16 字节参与折叠，冲突概率约 1/65533，满足小规模 Mesh 需求 */
            uint8_t lo = 0, hi = 0;
            for (uint8_t i = 0; i < 16; i += 2) {
                lo ^= die_id[i];
                hi ^= die_id[i + 1];
            }
            addr = (uint16_t)lo | ((uint16_t)hi << 8);
            sample_at_log_print("%s addr from EFUSE Die-ID -> 0x%04X\r\n",
                        SLE_UART_SERVER_LOG, addr);
        } else {
            /* Die-ID 读取失败 → 降级到 SLE MAC 方式（地址可能随重启变化） */
            sample_at_log_print("%s die_id read fail: 0x%x, fallback to MAC\r\n",
                        SLE_UART_SERVER_LOG, efuse_ret);
            sle_addr_t real_addr = {0};
            sle_get_local_addr(&real_addr);
            uint8_t *mac = real_addr.addr;
            addr = ((uint16_t)mac[0] ^ mac[2] ^ mac[4]) |
                   (((uint16_t)mac[1] ^ mac[3] ^ mac[5]) << 8);
        }

        /* 确保有效范围 0x0001 ~ 0xFFFD
         * WHY: 0x0000 = MESH_ADDR_UNASSIGNED, 0xFFFE/0xFFFF = 广播/保留地址 */
        if (addr == 0x0000) addr = 0x0001;
        if (addr >= 0xFFFE) addr = 0xFFFD;

        g_mesh_node_addr = addr;
        sample_at_log_print("%s mesh node addr = 0x%04X (persistent)\r\n",
                    SLE_UART_SERVER_LOG, g_mesh_node_addr);
    }

    /* Mesh模式: 运行时填充 SLE 地址与广播名称
     * g_local_addr[4] = low byte, [5] = high byte (小端序)
     * 对端 Client 通过 mesh_transport_addr_from_sle() 读取这两字节推导 mesh_addr */
    g_local_addr[4] = (uint8_t)(g_mesh_node_addr & 0xFF);
    g_local_addr[5] = (uint8_t)((g_mesh_node_addr >> 8) & 0xFF);

    snprintf((char *)sle_local_name, NAME_MAX_LENGTH,
             "%s%04X", MESH_ADV_NAME_PREFIX, g_mesh_node_addr);
    sample_at_log_print("%s mesh adv name: %s\r\n", SLE_UART_SERVER_LOG, sle_local_name);
#endif

    sle_set_default_announce_param();
    sle_set_default_announce_data();
    ret = sle_start_announce(SLE_ADV_HANDLE_DEFAULT);
    if (ret != ERRCODE_SLE_SUCCESS) {
        sample_at_log_print("%s sle_start_announce fail :%x\r\n", SLE_UART_SERVER_LOG, ret);
        return ret;
    }
    return ERRCODE_SLE_SUCCESS;
}