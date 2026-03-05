/**
 * @file  sle_uart_server.c
 * @brief SLE UART Server 模块实现（BearPi-Pico H3863 SLE Mesh 图传）
 * @details
 *   实现 SLE 服务端（Server 角色）的核心逻辑，包括：
 *   - SSAP Service / Property / Descriptor 注册
 *     （add_service → add_property(read+notify) → add_descriptor → start_service）
 *   - SSAP 回调注册（read / write / MTU 变更等）
 *   - SLE 连接/断开状态管理（含 Mesh 多连接场景下的方向判定与地址校验）
 *   - 配对完成回调处理
 *   - 通过 Notify（ssaps_notify_indicate）向指定 Client 发送数据
 *   - Mesh 模式下与 mesh_transport / sle_uart_client 的联动
 *
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * History:
 *   2023-07-17, Create file.
 *   2026-02-07, Modified for Mesh support.
 *   2026-02-08, Fix: unified connection callback with outgoing tracking.
 */
#include "common_def.h"
#include "securec.h"
#include "soc_osal.h"
#include "sle_errcode.h"
#include "sle_connection_manager.h"
#include "sle_device_discovery.h"
#include "sle_uart_server_adv.h"
#include "sle_uart_server.h"

/* === 常量宏定义 === */
#define OCTET_BIT_LEN           8       /**< 单字节比特长度，也用作 Property 初始值数组大小 */
#define UUID_LEN_2              2       /**< 16-bit UUID 编码后的字节长度 */
#define UUID_INDEX              14      /**< 128-bit UUID 中写入 16-bit 短 UUID 的偏移位置 */
#define BT_INDEX_4              4       /**< SLE 地址数组第 4 字节索引（日志打印用） */
#define BT_INDEX_0              0       /**< SLE 地址数组第 0 字节索引（日志打印用） */
#define UART_BUFF_LENGTH        0x100   /**< Notify 发送缓冲区长度（256 字节） */

#define SLE_ADV_HANDLE_DEFAULT  1       /**< 默认 SLE 广播句柄 ID */

/* === 全局 / 静态变量 === */
static char g_sle_uuid_app_uuid[UUID_LEN_2] = { 0x12, 0x34 };    /**< 应用层 UUID（用于 ssaps_register_server） */
static char g_sle_property_value[OCTET_BIT_LEN] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }; /**< Property 初始值 */

/* ============================================================
 *  Mesh 模式：多连接支持
 *  芯片限制 SSAP Server 实例为 1，但可同时持有多条连接。
 *  g_sle_conn_hdl_list[] 记录所有作为 Server 被对端 Client 连入的 conn_id。
 * ============================================================ */
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
/* P8-B fix: 使用统一配置, 不再硬编码 (芯片限制 server=1) */
#include "mesh_config.h"
#define MAX_SERVER_CONN  MESH_MAX_SERVER_CONN          /**< Server 最大并发连接数（取自 mesh_config） */
static uint16_t g_sle_conn_hdl_list[MAX_SERVER_CONN] = {0}; /**< Server 已连接 conn_id 列表 */
static uint8_t  g_sle_conn_count = 0;                 /**< 当前 Server 已连接数 */
static uint16_t g_sle_conn_hdl = 0;                   /**< 最近一次活动连接的 conn_id（兼容原有逻辑） */
#else
static uint16_t g_sle_conn_hdl = 0;                   /**< 当前连接的 conn_id */
#endif

static uint8_t g_server_id = 0;         /**< SSAP Server 实例 ID（ssaps_register_server 返回） */
static uint16_t g_service_handle = 0;   /**< SSAP Service Handle（ssaps_add_service_sync 返回） */
static uint16_t g_property_handle = 0;  /**< SSAP Property Handle（Notify 发送时使用） */
uint16_t g_sle_pair_hdl;                /**< 配对句柄（conn_id + 1），非零表示已完成配对 */

#define UUID_16BIT_LEN 2                                          /**< 16-bit UUID 长度 */
#define UUID_128BIT_LEN 16                                        /**< 128-bit UUID 长度 */
#define sample_at_log_print(fmt, args...) osal_printk(fmt, ##args) /**< 日志输出宏（映射到 osal_printk） */
#define SLE_UART_SERVER_LOG "[sle uart server]"                   /**< 日志前缀标签 */
#define SLE_SERVER_INIT_DELAY_MS    1000                          /**< Server 初始化延时（毫秒，预留协议栈就绪） */

static sle_uart_server_msg_queue g_sle_uart_server_msg_queue = NULL; /**< 上层消息队列回调（断连事件通知） */
/** @brief 128-bit UUID 基础模板，16-bit 短 UUID 写入 [14..15] 字节组成完整 UUID */
static uint8_t g_sle_uart_base[] = { 0x37, 0xBE, 0xA8, 0x80, 0xFC, 0x70, 0x11, 0xEA, \
    0xB7, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* === Mesh 依赖头文件 === */
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
#include "sle_uart_client.h"
#include "mesh_types.h"
#include "mesh_transport.h"
#endif

/**
 * @brief  获取当前活动的 SLE 连接 ID
 * @return g_sle_conn_hdl（当前连接 ID，无连接时为 0）
 */
uint16_t get_connect_id(void)
{
    return g_sle_conn_hdl;
}

/**
 * @brief  将 16-bit 数据以小端序写入字节数组
 * @param  _ptr [out] 目标缓冲区（至少 2 字节）
 * @param  data [in]  待编码的 16-bit 数据
 */
static void encode2byte_little(uint8_t *_ptr, uint16_t data)
{
    *(uint8_t *)((_ptr) + 1) = (uint8_t)((data) >> 0x8);
    *(uint8_t *)(_ptr) = (uint8_t)(data);
}

/**
 * @brief  用基础 UUID 模板填充 sle_uuid_t 结构
 * @param  out [out] 待填充的 UUID 结构体
 * @note   将 g_sle_uart_base 拷贝到 out->uuid，并设置长度为 UUID_LEN_2
 */
static void sle_uuid_set_base(sle_uuid_t *out)
{
    errcode_t ret;
    ret = memcpy_s(out->uuid, SLE_UUID_LEN, g_sle_uart_base, SLE_UUID_LEN);
    if (ret != EOK) {
        sample_at_log_print("%s sle_uuid_set_base memcpy fail\n", SLE_UART_SERVER_LOG);
        out->len = 0;
        return ;
    }
    out->len = UUID_LEN_2;
}

/**
 * @brief  用 16-bit 短 UUID 构造完整的 128-bit UUID
 * @param  u2  [in]  16-bit UUID 值（如 0x2222）
 * @param  out [out] 输出的 UUID 结构体
 * @note   先填入基础模板，再将 u2 写入 uuid[14..15]（小端序）
 */
static void sle_uuid_setu2(uint16_t u2, sle_uuid_t *out)
{
    sle_uuid_set_base(out);
    out->len = UUID_LEN_2;
    encode2byte_little(&out->uuid[UUID_INDEX], u2);
}

/**
 * @brief  调试用：打印 UUID 内容（支持 16-bit 和 128-bit 两种格式）
 * @param  uuid [in] 待打印的 UUID 结构体指针
 */
static void sle_uart_uuid_print(sle_uuid_t *uuid)
{
    if (uuid == NULL) {
        sample_at_log_print("%s uuid_print,uuid is null\r\n", SLE_UART_SERVER_LOG);
        return;
    }
    if (uuid->len == UUID_16BIT_LEN) {
        sample_at_log_print("%s uuid: %02x %02x.\n", SLE_UART_SERVER_LOG,
            uuid->uuid[14], uuid->uuid[15]);
    } else if (uuid->len == UUID_128BIT_LEN) {
        sample_at_log_print("%s uuid: \n", SLE_UART_SERVER_LOG);
        sample_at_log_print("%s 0x%02x 0x%02x 0x%02x \n", SLE_UART_SERVER_LOG, uuid->uuid[0], uuid->uuid[1],
            uuid->uuid[2], uuid->uuid[3]);
        sample_at_log_print("%s 0x%02x 0x%02x 0x%02x \n", SLE_UART_SERVER_LOG, uuid->uuid[4], uuid->uuid[5],
            uuid->uuid[6], uuid->uuid[7]);
        sample_at_log_print("%s 0x%02x 0x%02x 0x%02x \n", SLE_UART_SERVER_LOG, uuid->uuid[8], uuid->uuid[9],
            uuid->uuid[10], uuid->uuid[11]);
        sample_at_log_print("%s 0x%02x 0x%02x 0x%02x \n", SLE_UART_SERVER_LOG, uuid->uuid[12], uuid->uuid[13],
            uuid->uuid[14], uuid->uuid[15]);
    }
}

/* ============================================================
 *  SSAP 协议回调函数
 *  这些回调在 SSAP 协议事件（服务添加、MTU 变更等）发生时被协议栈调用。
 * ============================================================ */

/**
 * @brief  MTU 协商完成回调
 * @param  server_id [in] SSAP Server ID
 * @param  conn_id   [in] 连接 ID
 * @param  mtu_size  [in] 协商后的 MTU 信息
 * @param  status    [in] 操作状态码
 * @note   首次连接时设置 g_sle_pair_hdl 为 conn_id+1，标记已有配对连接
 */
static void ssaps_mtu_changed_cbk(uint8_t server_id, uint16_t conn_id, ssap_exchange_info_t *mtu_size,
    errcode_t status)
{
    sample_at_log_print("%s ssaps mtu_changed_cbk server_id:%x, conn_id:%x, mtu_size:%x, status:%x\r\n",
        SLE_UART_SERVER_LOG, server_id, conn_id, mtu_size->mtu_size, status);
    if (g_sle_pair_hdl == 0) {
        g_sle_pair_hdl = conn_id + 1;
    }
}

/**
 * @brief  服务启动完成回调
 * @param  server_id [in] SSAP Server ID
 * @param  handle    [in] 服务句柄
 * @param  status    [in] 操作状态码
 */
static void ssaps_start_service_cbk(uint8_t server_id, uint16_t handle, errcode_t status)
{
    sample_at_log_print("%s start service cbk server_id:%d, handle:%x, status:%x\r\n",
        SLE_UART_SERVER_LOG, server_id, handle, status);
}

/**
 * @brief  添加服务完成回调
 * @param  server_id [in] SSAP Server ID
 * @param  uuid      [in] 服务 UUID
 * @param  handle    [in] 分配到的服务句柄
 * @param  status    [in] 操作状态码
 */
static void ssaps_add_service_cbk(uint8_t server_id, sle_uuid_t *uuid, uint16_t handle, errcode_t status)
{
    sample_at_log_print("%s add service cbk server_id:%x, handle:%x, status:%x\r\n",
        SLE_UART_SERVER_LOG, server_id, handle, status);
    sle_uart_uuid_print(uuid);
}

/**
 * @brief  添加 Property（特征值）完成回调
 * @param  server_id      [in] SSAP Server ID
 * @param  uuid           [in] Property UUID
 * @param  service_handle [in] 所属服务句柄
 * @param  handle         [in] 分配到的 Property 句柄
 * @param  status         [in] 操作状态码
 */
static void ssaps_add_property_cbk(uint8_t server_id, sle_uuid_t *uuid, uint16_t service_handle,
    uint16_t handle, errcode_t status)
{
    sample_at_log_print("%s add property cbk server_id:%x, service_handle:%x, handle:%x, status:%x\r\n",
        SLE_UART_SERVER_LOG, server_id, service_handle, handle, status);
    sle_uart_uuid_print(uuid);
}

/**
 * @brief  添加 Descriptor（描述符）完成回调
 * @param  server_id       [in] SSAP Server ID
 * @param  uuid            [in] Descriptor UUID
 * @param  service_handle  [in] 所属服务句柄
 * @param  property_handle [in] 所属 Property 句柄
 * @param  status          [in] 操作状态码
 */
static void ssaps_add_descriptor_cbk(uint8_t server_id, sle_uuid_t *uuid, uint16_t service_handle,
    uint16_t property_handle, errcode_t status)
{
    sample_at_log_print("%s add descriptor cbk server_id:%x, service_handle:%x, property_handle:%x, \
        status:%x\r\n", SLE_UART_SERVER_LOG, server_id, service_handle, property_handle, status);
    sle_uart_uuid_print(uuid);
}

/**
 * @brief  删除所有服务完成回调
 * @param  server_id [in] SSAP Server ID
 * @param  status    [in] 操作状态码
 */
static void ssaps_delete_all_service_cbk(uint8_t server_id, errcode_t status)
{
    sample_at_log_print("%s delete all service server_id:%x, status:%x\r\n",
        SLE_UART_SERVER_LOG, server_id, status);
}

/**
 * @brief  注册所有 SSAP 回调函数
 * @param  ssaps_read_callback  [in] 来自上层的读请求处理函数
 * @param  ssaps_write_callback [in] 来自上层的写请求处理函数
 * @return ERRCODE_SLE_SUCCESS 成功，其他值失败
 * @note   内部将 add_service / add_property / add_descriptor / start_service /
 *         delete_all_service / mtu_changed 等回调统一注册
 */
static errcode_t sle_ssaps_register_cbks(ssaps_read_request_callback ssaps_read_callback,
    ssaps_write_request_callback ssaps_write_callback)
{
    errcode_t ret;
    ssaps_callbacks_t ssaps_cbk = {0};
    ssaps_cbk.add_service_cb = ssaps_add_service_cbk;
    ssaps_cbk.add_property_cb = ssaps_add_property_cbk;
    ssaps_cbk.add_descriptor_cb = ssaps_add_descriptor_cbk;
    ssaps_cbk.start_service_cb = ssaps_start_service_cbk;
    ssaps_cbk.delete_all_service_cb = ssaps_delete_all_service_cbk;
    ssaps_cbk.mtu_changed_cb = ssaps_mtu_changed_cbk;
    ssaps_cbk.read_request_cb = ssaps_read_callback;
    ssaps_cbk.write_request_cb = ssaps_write_callback;
    ret = ssaps_register_callbacks(&ssaps_cbk);
    if (ret != ERRCODE_SLE_SUCCESS) {
        sample_at_log_print("%s ssaps_register_callbacks fail :%x\r\n", SLE_UART_SERVER_LOG, ret);
        return ret;
    }
    return ERRCODE_SLE_SUCCESS;
}

/**
 * @brief  向 SSAP 协议栈添加一个服务（Service）
 * @return ERRCODE_SLE_SUCCESS 成功，ERRCODE_SLE_FAIL 失败
 * @note   使用 SLE_UUID_SERVER_SERVICE (0x2222) 作为服务 UUID，
 *         成功后 g_service_handle 被赋值
 */
static errcode_t sle_uuid_server_service_add(void)
{
    errcode_t ret;
    sle_uuid_t service_uuid = {0};
    sle_uuid_setu2(SLE_UUID_SERVER_SERVICE, &service_uuid);
    ret = ssaps_add_service_sync(g_server_id, &service_uuid, 1, &g_service_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        sample_at_log_print("%s add service fail, ret:%x\r\n", SLE_UART_SERVER_LOG, ret);
        return ERRCODE_SLE_FAIL;
    }
    return ERRCODE_SLE_SUCCESS;
}

/**
 * @brief  向已添加的服务中注册 Property 和 Descriptor
 * @return ERRCODE_SLE_SUCCESS 成功，ERRCODE_SLE_FAIL 失败
 * @note   处理步骤：
 *         1. 构造 Property（READ + NOTIFY 权限），UUID = 0x2323
 *         2. 调用 ssaps_add_property_sync 注册，获得 g_property_handle
 *         3. 构造 Descriptor（READ + WRITE），类型为 USER_DESCRIPTION
 *         4. 调用 ssaps_add_descriptor_sync 注册到该 Property 下
 */
static errcode_t sle_uuid_server_property_add(void)
{
    errcode_t ret;
    ssaps_property_info_t property = {0};
    ssaps_desc_info_t descriptor = {0};
    uint8_t ntf_value[] = {0x01, 0x0};

    property.permissions = SLE_UUID_TEST_PROPERTIES;
    property.operate_indication = SSAP_OPERATE_INDICATION_BIT_READ | SSAP_OPERATE_INDICATION_BIT_NOTIFY;
    sle_uuid_setu2(SLE_UUID_SERVER_NTF_REPORT, &property.uuid);
    property.value = (uint8_t *)osal_vmalloc(sizeof(g_sle_property_value));
    if (property.value == NULL) {
        return ERRCODE_SLE_FAIL;
    }
    if (memcpy_s(property.value, sizeof(g_sle_property_value), g_sle_property_value,
        sizeof(g_sle_property_value)) != EOK) {
        osal_vfree(property.value);
        return ERRCODE_SLE_FAIL;
    }
    ret = ssaps_add_property_sync(g_server_id, g_service_handle, &property, &g_property_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        sample_at_log_print("%s add property fail, ret:%x\r\n", SLE_UART_SERVER_LOG, ret);
        osal_vfree(property.value);
        return ERRCODE_SLE_FAIL;
    }
    descriptor.permissions = SLE_UUID_TEST_DESCRIPTOR;
    descriptor.type = SSAP_DESCRIPTOR_USER_DESCRIPTION;
    descriptor.operate_indication = SSAP_OPERATE_INDICATION_BIT_READ | SSAP_OPERATE_INDICATION_BIT_WRITE;
    descriptor.value = ntf_value;
    descriptor.value_len = sizeof(ntf_value);

    ret = ssaps_add_descriptor_sync(g_server_id, g_service_handle, g_property_handle, &descriptor);
    if (ret != ERRCODE_SLE_SUCCESS) {
        sample_at_log_print("%s add descriptor fail, ret:%x\r\n", SLE_UART_SERVER_LOG, ret);
        osal_vfree(property.value);
        osal_vfree(descriptor.value);
        return ERRCODE_SLE_FAIL;
    }
    osal_vfree(property.value);
    return ERRCODE_SLE_SUCCESS;
}

/**
 * @brief  完整的 SSAP 服务注册流程
 * @return ERRCODE_SLE_SUCCESS 成功，ERRCODE_SLE_FAIL 失败
 * @note   流程：register_server → set_info(MTU=520) → add_service → add_property → start_service
 *         MTU 设为 520 以匹配客户端，保证大帧 Notify 不被截断
 */
static errcode_t sle_uart_server_add(void)
{
    errcode_t ret;
    sle_uuid_t app_uuid = {0};

    sample_at_log_print("%s sle uart add service in\r\n", SLE_UART_SERVER_LOG);
    app_uuid.len = sizeof(g_sle_uuid_app_uuid);
    if (memcpy_s(app_uuid.uuid, app_uuid.len, g_sle_uuid_app_uuid, sizeof(g_sle_uuid_app_uuid)) != EOK) {
        return ERRCODE_SLE_FAIL;
    }
    ssaps_register_server(&app_uuid, &g_server_id);

    /* O4: 设置服务端 MTU=520, 匹配客户端 MTU, 允许大帧 notify
     * 默认值 208 太小, 导致 >208B 的 mesh 帧 notify 失败 */
    ssap_exchange_info_t server_info = {0};
    server_info.mtu_size = 520;
    server_info.version  = 1;
    ret = ssaps_set_info(g_server_id, &server_info);
    sample_at_log_print("%s server MTU set to 520, ret:0x%x\r\n", SLE_UART_SERVER_LOG, ret);

    if (sle_uuid_server_service_add() != ERRCODE_SLE_SUCCESS) {
        ssaps_unregister_server(g_server_id);
        return ERRCODE_SLE_FAIL;
    }
    if (sle_uuid_server_property_add() != ERRCODE_SLE_SUCCESS) {
        ssaps_unregister_server(g_server_id);
        return ERRCODE_SLE_FAIL;
    }
    sample_at_log_print("%s add service, server_id:%x, service_handle:%x, property_handle:%x\r\n",
        SLE_UART_SERVER_LOG, g_server_id, g_service_handle, g_property_handle);
    ret = ssaps_start_service(g_server_id, g_service_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        sample_at_log_print("%s start service fail, ret:%x\r\n", SLE_UART_SERVER_LOG, ret);
        return ERRCODE_SLE_FAIL;
    }
    sample_at_log_print("%s add service out\r\n", SLE_UART_SERVER_LOG);
    return ERRCODE_SLE_SUCCESS;
}

/**
 * @brief  通过 UUID 匹配方式向当前连接发送 Notify
 * @param  data [in] 待发送的数据指针
 * @param  len  [in] 数据长度（字节）
 * @return ERRCODE_SLE_SUCCESS 成功，其他值失败
 * @note   内部通过 ssaps_notify_indicate_by_uuid 发送，
 *         使用 g_sle_conn_hdl 作为目标连接 ID
 */
errcode_t sle_uart_server_send_report_by_uuid(const uint8_t *data, uint8_t len)
{
    errcode_t ret;
    ssaps_ntf_ind_by_uuid_t param = {0};
    param.type = SSAP_PROPERTY_TYPE_VALUE;
    param.start_handle = g_service_handle;
    param.end_handle = g_property_handle;
    param.value_len = len;
    param.value = (uint8_t *)osal_vmalloc(len);
    if (param.value == NULL) {
        sample_at_log_print("%s send report new fail\r\n", SLE_UART_SERVER_LOG);
        return ERRCODE_SLE_FAIL;
    }
    if (memcpy_s(param.value, param.value_len, data, len) != EOK) {
        sample_at_log_print("%s send input report memcpy fail\r\n", SLE_UART_SERVER_LOG);
        osal_vfree(param.value);
        return ERRCODE_SLE_FAIL;
    }
    sle_uuid_setu2(SLE_UUID_SERVER_NTF_REPORT, &param.uuid);
    ret = ssaps_notify_indicate_by_uuid(g_server_id, g_sle_conn_hdl, &param);
    if (ret != ERRCODE_SLE_SUCCESS) {
        sample_at_log_print("%s ssaps_notify_indicate_by_uuid fail :%x\r\n", SLE_UART_SERVER_LOG, ret);
        osal_vfree(param.value);
        return ret;
    }
    osal_vfree(param.value);
    return ERRCODE_SLE_SUCCESS;
}

/**
 * @brief  通过 Handle 方式向当前连接发送 Notify
 * @param  data [in] 待发送的数据指针
 * @param  len  [in] 数据长度（字节，不超过 UART_BUFF_LENGTH）
 * @return ERRCODE_SLE_SUCCESS 成功，其他值失败
 * @note   使用栈上缓冲区 receive_buf 拷贝数据后发送，适用于较小数据帧
 */
errcode_t sle_uart_server_send_report_by_handle(const uint8_t *data, uint16_t len)
{
    ssaps_ntf_ind_t param = {0};
    uint8_t receive_buf[UART_BUFF_LENGTH] = { 0 };
    param.handle = g_property_handle;
    param.type = SSAP_PROPERTY_TYPE_VALUE;
    param.value = receive_buf;
    param.value_len = len;
    if (memcpy_s(param.value, param.value_len, data, len) != EOK) {
        return ERRCODE_SLE_FAIL;
    }
    return ssaps_notify_indicate(g_server_id, g_sle_conn_hdl, &param);
}

/* ============================================================
 *  Mesh 新增接口（多连接 Notify 与状态查询）
 * ============================================================ */
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
/**
 * @brief  向指定 conn_id 的 Client 发送 Notify
 * @param  conn_id [in] 目标 SLE 连接 ID
 * @param  data    [in] 待发送数据指针（零拷贝，直接传递给协议栈）
 * @param  len     [in] 数据长度（字节）
 * @return ERRCODE_SUCC 成功，其他值失败
 * @note   与 send_report_by_handle 不同，此函数不使用 g_sle_conn_hdl，
 *         而是直接指定 conn_id，供 mesh_transport 层按需转发
 */
errcode_t sle_uart_server_send_to_conn(uint16_t conn_id, const uint8_t *data, uint16_t len)
{
    ssaps_ntf_ind_t param = {0};
    param.handle = g_property_handle;
    param.type = SSAP_PROPERTY_TYPE_VALUE;
    param.value = (uint8_t *)data;
    param.value_len = len;

    errcode_t ret = ssaps_notify_indicate(g_server_id, conn_id, &param);
    if (ret != ERRCODE_SUCC) {
        sample_at_log_print("%s notify_indicate FAIL: server=%d conn=%d len=%d ret=0x%x\r\n",
                            SLE_UART_SERVER_LOG, g_server_id, conn_id, len, ret);
    }
    return ret;
}

/**
 * @brief  获取 SSAP Server ID
 * @return g_server_id
 */
uint8_t sle_uart_server_get_server_id(void)
{
    return g_server_id;
}

/**
 * @brief  获取 Notify Property Handle
 * @return g_property_handle
 */
uint16_t sle_uart_server_get_property_handle(void)
{
    return g_property_handle;
}

/**
 * @brief  获取 Server 角色当前已连接数
 * @return g_sle_conn_count（0 ~ MAX_SERVER_CONN）
 */
uint8_t sle_uart_server_get_conn_count(void)
{
    return g_sle_conn_count;
}
#endif /* CONFIG_SAMPLE_SUPPORT_SLE_MESH */

/* ============================================================
 *  连接状态回调
 *
 *  Mesh 模式: 统一回调, 通过出站连接追踪区分方向.
 *
 *  原理:
 *    Client 端在 seek_disable_cbk 中调用 sle_connect_remote_device 前
 *    会设置 g_outgoing_pending=true 并记录目标地址.
 *    当连接回调触发时, 用 sle_uart_client_check_outgoing(addr) 检查
 *    是否匹配正在等待的出站连接. 匹配 → CLIENT, 不匹配 → SERVER.
 * ============================================================ */
/**
 * @brief  SLE 连接状态变更回调（协议栈触发）
 * @param  conn_id      [in] 连接 ID
 * @param  addr         [in] 对端 SLE 地址
 * @param  conn_state   [in] 连接状态（CONNECTED / DISCONNECTED）
 * @param  pair_state   [in] 配对状态
 * @param  disc_reason  [in] 断连原因码
 * @note   Mesh 模式下的核心判定逻辑：
 *         - 连接时：通过 sle_uart_client_check_outgoing(addr) 判断方向
 *           · 匹配出站 → CLIENT 方向，调用 sle_uart_client_on_connected
 *           · 不匹配  → SERVER 方向，校验 mesh 地址有效性后加入列表，
 *             调用 mesh_transport_on_server_connected
 *         - 断开时：遍历 g_sle_conn_hdl_list 确认是否为 Server 连接，
 *           并校验地址防止 conn_id 复用导致误删
 */
static void sle_connect_state_changed_cbk(uint16_t conn_id, const sle_addr_t *addr,
    sle_acb_state_t conn_state, sle_pair_state_t pair_state, sle_disc_reason_t disc_reason)
{
    uint8_t sle_connect_state[] = "sle_dis_connect";
    sample_at_log_print("%s connect state changed conn_id:0x%02x, conn_state:0x%x, pair_state:0x%x, \
        disc_reason:0x%x\r\n", SLE_UART_SERVER_LOG, conn_id, conn_state, pair_state, disc_reason);
    sample_at_log_print("%s addr:%02x:**:**:**:%02x:%02x\r\n", SLE_UART_SERVER_LOG,
        addr->addr[BT_INDEX_0], addr->addr[BT_INDEX_4], addr->addr[5]);

    if (conn_state == SLE_ACB_STATE_CONNECTED) {
        g_sle_conn_hdl = conn_id;

#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
        /* ---- Mesh 模式: 通过出站追踪区分方向 ---- */
        if (sle_uart_client_check_outgoing(addr)) {
            /* 匹配出站连接 → 这是我们作为 Client 发起的连接 */
            sample_at_log_print("%s [MESH] CLIENT conn established, conn_id=%d\r\n",
                                SLE_UART_SERVER_LOG, conn_id);
            sle_uart_client_on_connected(conn_id, addr);
        } else {
            /* 不匹配 → 这是对方连入我们的 Server
             * Fix: 验证 mesh 地址是否有效, 过滤幽灵设备 */
            uint16_t remote_mesh = (uint16_t)((addr->addr[5] << 8) | addr->addr[4]);
            if (remote_mesh == 0 || remote_mesh >= 0xFFFE) {
                /* 非 mesh 设备 (幽灵设备: 地址为0, 广播地址0xFFFF, 或网关预留0xFFFE) */
                sample_at_log_print("%s [MESH] reject ghost device 0x%04X, conn_id=%d\r\n",
                                    SLE_UART_SERVER_LOG, remote_mesh, conn_id);
                sle_disconnect_remote_device(addr);
                return;
            }

            sample_at_log_print("%s [MESH] SERVER conn accepted, conn_id=%d, mesh=0x%04X\r\n",
                                SLE_UART_SERVER_LOG, conn_id, remote_mesh);

            if (g_sle_conn_count < MAX_SERVER_CONN) {
                g_sle_conn_hdl_list[g_sle_conn_count] = conn_id;
                g_sle_conn_count++;
            }
            mesh_transport_on_server_connected(conn_id, addr);

            /* P8-B fix: 通知 client 逻辑: 该地址已通过 server 连入,
             * 无需再尝试 outgoing 连接, 清除相关退避记录 */
            sle_uart_client_on_server_neighbor(addr);
        }
#endif

    } else if (conn_state == SLE_ACB_STATE_DISCONNECTED) {
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
        /* ============================================================
         *  P4 修复: 断开时必须校验地址
         *
         *  SLE 协议栈对失败的出站连接尝试报的 disconnect 可能
         *  复用已有连接的 conn_id. 必须比对地址, 避免误删活着的连接.
         * ============================================================ */

        /* 1) 检查是否为 Server 连接 — 同时校验地址 */
        bool was_server_conn = false;
        for (uint8_t i = 0; i < g_sle_conn_count; i++) {
            if (g_sle_conn_hdl_list[i] == conn_id) {
                /* conn_id 匹配, 再校验 SLE 地址 */
                mesh_conn_entry_t *srv_entry = mesh_transport_find_by_conn_id(conn_id);
                if (srv_entry != NULL &&
                    memcmp(srv_entry->sle_addr, addr->addr, SLE_ADDR_LEN) != 0) {
                    /* 地址不匹配: 这不是真正的 server 断开, 跳过 */
                    sample_at_log_print("%s [MESH] conn_id=%d addr mismatch in server list, "
                                        "skip server removal\r\n",
                                        SLE_UART_SERVER_LOG, conn_id);
                    break;
                }
                g_sle_conn_hdl_list[i] = g_sle_conn_hdl_list[g_sle_conn_count - 1];
                g_sle_conn_count--;
                was_server_conn = true;
                break;
            }
        }

        if (was_server_conn) {
            sample_at_log_print("%s [MESH] SERVER conn lost, conn_id=%d\r\n",
                                SLE_UART_SERVER_LOG, conn_id);
            mesh_transport_on_server_disconnected(conn_id);

            /* P3: Server 连接断开, 腾出位置, 重启广播 */
            extern volatile bool g_need_re_announce;
            g_need_re_announce = true;
        } else {
            sample_at_log_print("%s [MESH] CLIENT conn lost, conn_id=%d\r\n",
                                SLE_UART_SERVER_LOG, conn_id);
            sle_uart_client_on_disconnected(conn_id, addr);
        }

        g_sle_conn_hdl = (g_sle_conn_count > 0) ? g_sle_conn_hdl_list[0] : 0;
#else
        g_sle_conn_hdl = 0;
#endif
        g_sle_pair_hdl = 0;
        if (g_sle_uart_server_msg_queue != NULL) {
            g_sle_uart_server_msg_queue(sle_connect_state, sizeof(sle_connect_state));
        }
    }
}

/**
 * @brief  SLE 配对完成回调
 * @param  conn_id [in] 连接 ID
 * @param  addr    [in] 对端 SLE 地址
 * @param  status  [in] 配对结果状态码
 * @note   配对成功后重新设置 MTU=520，确保后续 Notify 可承载大帧数据；
 *         Mesh 模式下同时通知 Client 逻辑处理配对完成事件
 */
static void sle_pair_complete_cbk(uint16_t conn_id, const sle_addr_t *addr, errcode_t status)
{
    sample_at_log_print("%s pair complete conn_id:%02x, status:%x\r\n",
        SLE_UART_SERVER_LOG, conn_id, status);
    sample_at_log_print("%s pair complete addr:%02x:**:**:**:%02x:%02x\r\n",
        SLE_UART_SERVER_LOG, addr->addr[BT_INDEX_0], addr->addr[BT_INDEX_4]);

    g_sle_pair_hdl = conn_id + 1;
    ssap_exchange_info_t parameter = { 0 };
    parameter.mtu_size = 520;
    parameter.version = 1;
    ssaps_set_info(g_server_id, &parameter);

#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH)
    /* Client 连接的配对也需要处理 */
    sle_uart_client_on_pair_complete(conn_id, addr, status);
#endif
}

/**
 * @brief  注册 SLE 连接管理回调（连接状态变更 + 配对完成）
 * @return ERRCODE_SLE_SUCCESS 成功，其他值失败
 */
static errcode_t sle_conn_register_cbks(void)
{
    errcode_t ret;
    sle_connection_callbacks_t conn_cbks = {0};
    conn_cbks.connect_state_changed_cb = sle_connect_state_changed_cbk;
    conn_cbks.pair_complete_cb = sle_pair_complete_cbk;
    ret = sle_connection_register_callbacks(&conn_cbks);
    if (ret != ERRCODE_SLE_SUCCESS) {
        sample_at_log_print("%s sle_connection_register_callbacks fail :%x\r\n",
        SLE_UART_SERVER_LOG, ret);
        return ret;
    }
    return ERRCODE_SLE_SUCCESS;
}

/**
 * @brief  查询当前是否有 Client 已完成配对
 * @return 非零（conn_id+1）表示已配对，0 表示未配对
 */
uint16_t sle_uart_client_is_connected(void)
{
    return g_sle_pair_hdl;
}

/**
 * @brief  SLE UART Server 模块初始化入口
 * @param  ssaps_read_callback  [in] 上层提供的 SSAP 读请求处理回调
 * @param  ssaps_write_callback [in] 上层提供的 SSAP 写请求处理回调
 * @return ERRCODE_SLE_SUCCESS 成功，其他值失败
 * @note   初始化步骤：
 *         1. enable_sle() — 使能 SLE 协议栈
 *         2. 注册广播回调（sle_uart_announce_register_cbks）
 *         3. 注册连接管理回调（sle_conn_register_cbks）
 *         4. 注册 SSAP 回调（sle_ssaps_register_cbks）
 *         5. 注册服务并启动（sle_uart_server_add）
 *         6. 初始化并启动广播（sle_uart_server_adv_init）
 */
errcode_t sle_uart_server_init(ssaps_read_request_callback ssaps_read_callback,
    ssaps_write_request_callback ssaps_write_callback)
{
    errcode_t ret;

    /* 使能SLE */
    if (enable_sle() != ERRCODE_SUCC) {
        sample_at_log_print("[SLE Server] sle enable fail !\r\n");
        return -1;
    }

    ret = sle_uart_announce_register_cbks();
    if (ret != ERRCODE_SLE_SUCCESS) {
        sample_at_log_print("%s announce_register_cbks fail :%x\r\n", SLE_UART_SERVER_LOG, ret);
        return ret;
    }
    ret = sle_conn_register_cbks();
    if (ret != ERRCODE_SLE_SUCCESS) {
        sample_at_log_print("%s conn_register_cbks fail :%x\r\n", SLE_UART_SERVER_LOG, ret);
        return ret;
    }
    ret = sle_ssaps_register_cbks(ssaps_read_callback, ssaps_write_callback);
    if (ret != ERRCODE_SLE_SUCCESS) {
        sample_at_log_print("%s ssaps_register_cbks fail :%x\r\n", SLE_UART_SERVER_LOG, ret);
        return ret;
    }
    ret = sle_uart_server_add();
    if (ret != ERRCODE_SLE_SUCCESS) {
        sample_at_log_print("%s server_add fail :%x\r\n", SLE_UART_SERVER_LOG, ret);
        return ret;
    }
    ret = sle_uart_server_adv_init();
    if (ret != ERRCODE_SLE_SUCCESS) {
        sample_at_log_print("%s adv_init fail :%x\r\n", SLE_UART_SERVER_LOG, ret);
        return ret;
    }
    sample_at_log_print("%s init ok\r\n", SLE_UART_SERVER_LOG);
    return ERRCODE_SLE_SUCCESS;
}

/**
 * @brief  注册上层消息队列回调
 * @param  sle_uart_server_msg [in] 回调函数指针（NULL 则不通知）
 * @note   断连等事件发生时，通过此回调向上层发送消息字符串
 */
void sle_uart_server_register_msg(sle_uart_server_msg_queue sle_uart_server_msg)
{
    g_sle_uart_server_msg_queue = sle_uart_server_msg;
}