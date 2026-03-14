/**
 * @file    sle_uart.c
 * @brief   SLE UART 桥接层入口文件
 * @details 本文件是 BearPi-Pico H3863 SLE Mesh 图传项目中 SLE UART 桥接层的核心入口。
 *          基于 SDK 原始示例精简而来，负责：
 *          1. UART 引脚复用与串口参数初始化；
 *          2. SLE Server / Client 回调注册及消息队列管理；
 *          3. 创建内核线程运行 Server 或 Client 主任务。
 *          上层 mesh_main.c 通过 app_run 宏间接调用 sle_uart_entry() 启动整条链路。
 *
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * History:
 * 2023-07-17, Create file.
 */

/* ============================= 头文件包含 ============================= */
#include "common_def.h"
#include "soc_osal.h"
#include "app_init.h"
#include "pinctrl.h"
#include "uart.h"
// #include "pm_clock.h"
#include "sle_low_latency.h"

/* ---------- 根据编译宏选择 Server 或 Client 头文件 ---------- */
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH_SERVER)
#include "securec.h"
#include "sle_uart_server.h"
#include "sle_uart_server_adv.h"
#include "sle_device_discovery.h"
#include "sle_errcode.h"
#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH_CLIENT)
#define SLE_UART_TASK_STACK_SIZE            0x600   /**< Client 任务栈大小，1536 字节 */
#include "sle_connection_manager.h"
#include "sle_ssap_client.h"
#include "sle_uart_client.h"
#endif  /* CONFIG_SAMPLE_SUPPORT_SLE_MESH_CLIENT */

/* ============================= 公共宏定义 ============================= */
#define SLE_UART_TASK_PRIO                  28      /**< 内核线程优先级，数值越小优先级越高，取值 0-63 */
#define SLE_UART_TASK_DURATION_MS           2000    /**< Server 主循环轮询间隔，单位 ms */
#define SLE_UART_BAUDRATE                   115200  /**< UART 波特率，单位 bps */
#define SLE_UART_TRANSFER_SIZE              512     /**< UART 单次接收缓冲区大小，单位字节 */

/* ============================= 全局/静态变量 ============================= */

/** UART 接收环形缓冲区（静态分配） */
static uint8_t g_app_uart_rx_buff[SLE_UART_TRANSFER_SIZE] = { 0 };

/** UART 缓冲区配置结构体，绑定接收缓冲区及其大小 */
static uart_buffer_config_t g_app_uart_buffer_config = {
    .rx_buffer = g_app_uart_rx_buff,
    .rx_buffer_size = SLE_UART_TRANSFER_SIZE
};

/* ============================= UART 底层初始化 ============================= */

/**
 * @brief  初始化 UART 引脚复用
 * @note   根据 CONFIG_SLE_UART_BUS 选择对应 UART 控制器，
 *         将 TXD/RXD 引脚设置为 PIN_MODE_1（UART 功能复用模式）。
 *         当前 UART0 与 UART1 使用相同引脚配置，由 menuconfig 统一指定。
 */
static void uart_init_pin(void)
{
    if (CONFIG_SLE_UART_BUS == 0) {
        uapi_pin_set_mode(CONFIG_UART_TXD_PIN, PIN_MODE_1);
        uapi_pin_set_mode(CONFIG_UART_RXD_PIN, PIN_MODE_1);       
    }else if (CONFIG_SLE_UART_BUS == 1) {
        uapi_pin_set_mode(CONFIG_UART_TXD_PIN, PIN_MODE_1);
        uapi_pin_set_mode(CONFIG_UART_RXD_PIN, PIN_MODE_1);       
    }
}

/**
 * @brief  配置并初始化 UART 外设
 * @note   处理步骤：
 *         1. 填充串口属性（波特率 115200、8N1）；
 *         2. 填充引脚配置（无硬件流控，CTS/RTS 置 PIN_NONE）；
 *         3. 先 deinit 再 init，确保重复调用时状态干净；
 *         4. 传入全局接收缓冲区配置 g_app_uart_buffer_config。
 */
static void uart_init_config(void)
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
        .cts_pin = PIN_NONE,            /* 不使用硬件流控 */
        .rts_pin = PIN_NONE
    };
    /* 先反初始化，防止重复 init 导致资源泄漏 */
    uapi_uart_deinit(CONFIG_SLE_UART_BUS);
    uapi_uart_init(CONFIG_SLE_UART_BUS, &pin_config, &attr, NULL, &g_app_uart_buffer_config);

}

/* ============================= SLE Server 段 ============================= */
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH_SERVER)

/* ---------- Server 专用宏定义 ---------- */
#define SLE_UART_SERVER_DELAY_COUNT         5           /**< Server 启动前延迟计数（暂未使用） */
#define SLE_UART_TASK_STACK_SIZE            0x2400      /**< Server 任务栈大小，9216 字节（回退至原始值，节省堆支持 96KB 缓冲区）*/
#define SLE_ADV_HANDLE_DEFAULT              1           /**< 默认广播句柄编号，取值 >= 1 */
#define SLE_UART_SERVER_MSG_QUEUE_LEN       5           /**< 消息队列最大消息条数 */
#define SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE  32          /**< 单条消息最大字节数 */
#define SLE_UART_SERVER_QUEUE_DELAY         0xFFFFFFFF  /**< 消息队列读取超时，0xFFFFFFFF 表示永久阻塞 */
#define SLE_UART_SERVER_BUFF_MAX_SIZE       800         /**< Server 数据缓冲区上限，单位字节（预留） */

/** Server 消息队列 ID（全局，供回调写入消息） */
unsigned long g_sle_uart_server_msgqueue_id;

#define SLE_UART_SERVER_LOG                 "[sle uart server]"  /**< 日志前缀标签 */
/**
 * @brief  SSAP Server 读请求回调
 * @param  server_id    服务端实例 ID
 * @param  conn_id      连接 ID
 * @param  read_cb_para 读请求参数（含属性句柄等）
 * @param  status       底层返回的错误码
 * @note   当前仅打印日志，未向对端返回实际数据。
 */
static void ssaps_server_read_request_cbk(uint8_t server_id, uint16_t conn_id, ssaps_req_read_cb_t *read_cb_para,
    errcode_t status)
{
    osal_printk("%s ssaps read request cbk callback server_id:%x, conn_id:%x, handle:%x, status:%x\r\n",
        SLE_UART_SERVER_LOG, server_id, conn_id, read_cb_para->handle, status);
}
/**
 * @brief  SSAP Server 写请求回调（Client → Server 方向数据入口）
 * @param  server_id     服务端实例 ID
 * @param  conn_id       连接 ID
 * @param  write_cb_para 写请求参数（含数据指针和长度）
 * @param  status        底层返回的错误码
 * @note   收到 Client 发来的数据后，通过 UART 输出到外设（如电子墨水屏控制器）。
 */
static void ssaps_server_write_request_cbk(uint8_t server_id, uint16_t conn_id, ssaps_req_write_cb_t *write_cb_para,
    errcode_t status)
{
    osal_printk("%s ssaps write request callback cbk server_id:%x, conn_id:%x, handle:%x, status:%x\r\n",
        SLE_UART_SERVER_LOG, server_id, conn_id, write_cb_para->handle, status);
    if ((write_cb_para->length > 0) && write_cb_para->value) {
        osal_printk("\n sle uart recived data : %s\r\n", write_cb_para->value);
        /* 将 SLE 收到的数据直接透传到 UART 总线 */
        uapi_uart_write(CONFIG_SLE_UART_BUS, (uint8_t *)write_cb_para->value, write_cb_para->length, 0);
    }
}

/**
 * @brief  Server 端 UART 接收中断处理函数
 * @param  buffer 接收到的数据缓冲区指针
 * @param  length 接收到的数据字节数
 * @param  error  UART 接收错误标志（此处忽略）
 * @note   UART 收到数据后，判断 SLE 是否已连接：
 *         - 已连接：通过 SLE Notify 将数据上报给 Client；
 *         - 未连接：仅打印告警日志。
 */
static void sle_uart_server_read_int_handler(const void *buffer, uint16_t length, bool error)
{
    unused(error);
    if (sle_uart_client_is_connected()) {
    sle_uart_server_send_report_by_handle(buffer, length);
    } else {
        osal_printk("%s sle client is not connected! \r\n", SLE_UART_SERVER_LOG);
    }
}


/* ---------- Server 消息队列管理 ---------- */

/**
 * @brief  创建 Server 消息队列
 * @note   消息队列用于在 SLE 连接状态变化回调与主循环之间传递事件。
 */
static void sle_uart_server_create_msgqueue(void)
{
    if (osal_msg_queue_create("sle_uart_server_msgqueue", SLE_UART_SERVER_MSG_QUEUE_LEN, \
        (unsigned long *)&g_sle_uart_server_msgqueue_id, 0, SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE) != OSAL_SUCCESS) {
        osal_printk("^%s sle_uart_server_create_msgqueue message queue create failed!\n", SLE_UART_SERVER_LOG);
    }
}

/**
 * @brief  删除 Server 消息队列，释放资源
 */
static void sle_uart_server_delete_msgqueue(void)
{
    osal_msg_queue_delete(g_sle_uart_server_msgqueue_id);
}

/**
 * @brief  向 Server 消息队列写入一条消息（拷贝方式）
 * @param  buffer_addr 待写入数据的起始地址
 * @param  buffer_size 待写入数据的字节数
 * @note   此函数作为回调注册给 sle_uart_server 模块，在连接状态变化时被调用。
 */
static void sle_uart_server_write_msgqueue(uint8_t *buffer_addr, uint16_t buffer_size)
{
    osal_msg_queue_write_copy(g_sle_uart_server_msgqueue_id, (void *)buffer_addr, \
                              (uint32_t)buffer_size, 0);
}

/**
 * @brief  从 Server 消息队列读取一条消息（阻塞方式）
 * @param  buffer_addr  读取数据的目标缓冲区
 * @param  buffer_size  [in/out] 输入缓冲区大小，输出实际读取字节数
 * @return 0 成功；非 0 失败
 * @note   超时时间为 SLE_UART_SERVER_QUEUE_DELAY（永久阻塞），
 *         线程会在此处挂起直到有新消息到来。
 */
static int32_t sle_uart_server_receive_msgqueue(uint8_t *buffer_addr, uint32_t *buffer_size)
{
    return osal_msg_queue_read_copy(g_sle_uart_server_msgqueue_id, (void *)buffer_addr, \
                                    buffer_size, SLE_UART_SERVER_QUEUE_DELAY);
}
/**
 * @brief  重置 Server 接收缓冲区
 * @param  buffer_addr 缓冲区起始地址
 * @param  buffer_size [out] 重置后的缓冲区大小
 * @note   每次读取消息队列前调用，清零缓冲区防止残留数据干扰。
 */
static void sle_uart_server_rx_buf_init(uint8_t *buffer_addr, uint32_t *buffer_size)
{
    *buffer_size = SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE;
    (void)memset_s(buffer_addr, *buffer_size, 0, *buffer_size);
}

/* ---------- Server 主任务 ---------- */

/**
 * @brief  SLE UART Server 主任务函数
 * @param  arg 线程参数（未使用）
 * @return 始终返回 NULL（实际不会退出 while 循环）
 * @note   处理步骤：
 *         1. 创建消息队列并注册写入回调；
 *         2. 初始化 SLE Server（注册读 / 写请求回调）；
 *         3. 初始化 UART 引脚与配置，注册 UART 接收回调；
 *         4. 进入主循环：阻塞读取消息队列，若收到 "sle_dis_connect" 则
 *            重新启动 SLE 广播，等待 Client 重连。
 */
static void *sle_uart_server_task(const char *arg)
{
    unused(arg);
    uint8_t rx_buf[SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE] = {0};
    uint32_t rx_length = SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE;
    uint8_t sle_connect_state[] = "sle_dis_connect";

    /* 步骤 1：创建消息队列 & 注册消息写入回调 */
    sle_uart_server_create_msgqueue();
    sle_uart_server_register_msg(sle_uart_server_write_msgqueue);
    /* 步骤 2：初始化 SLE Server，注册 SSAP 读/写回调 */
    sle_uart_server_init(ssaps_server_read_request_cbk, ssaps_server_write_request_cbk);


    /* 步骤 3：UART 引脚复用 & 串口参数初始化 */
    uart_init_pin();
    uart_init_config();

    /* 先注销旧回调，再注册新的 UART 接收回调，避免重复注册 */
    uapi_uart_unregister_rx_callback(CONFIG_SLE_UART_BUS);
    errcode_t ret = uapi_uart_register_rx_callback(CONFIG_SLE_UART_BUS,
                                                   UART_RX_CONDITION_FULL_OR_IDLE,
                                                   1, sle_uart_server_read_int_handler);
    if (ret != ERRCODE_SUCC) {
        osal_printk("%s Register uart callback fail.[%x]\r\n", SLE_UART_SERVER_LOG, ret);
        return NULL;
    }

    /* 步骤 4：主循环——监听连接断开事件并重启广播 */
    while (1) {
        sle_uart_server_rx_buf_init(rx_buf, &rx_length);
        sle_uart_server_receive_msgqueue(rx_buf, &rx_length);
        /* 收到 "sle_dis_connect" 表示远端断开，需要重新发起广播 */
        if (strncmp((const char *)rx_buf, (const char *)sle_connect_state, sizeof(sle_connect_state)) == 0) {
            ret = sle_start_announce(SLE_ADV_HANDLE_DEFAULT);
            if (ret != ERRCODE_SLE_SUCCESS) {
                osal_printk("%s sle_connect_state_changed_cbk,sle_start_announce fail :%02x\r\n",
                    SLE_UART_SERVER_LOG, ret);
            }
        }
        osal_msleep(SLE_UART_TASK_DURATION_MS);
    }
    sle_uart_server_delete_msgqueue();
    return NULL;
}
/* ============================= SLE Client 段 ============================= */
#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH_CLIENT)

/**
 * @brief  SLE Client Notification 回调（Server → Client 推送）
 * @param  client_id 客户端实例 ID（未使用）
 * @param  conn_id   连接 ID（未使用）
 * @param  data      收到的 Notification 数据（含指针和长度）
 * @param  status    错误码（未使用）
 * @note   将 Server 推送的数据直接透传到 UART，实现 SLE→串口 方向的桥接。
 */
void sle_uart_notification_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data,
    errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);
    osal_printk("\n sle uart recived data : %s\r\n", data->data);
    uapi_uart_write(CONFIG_SLE_UART_BUS, (uint8_t *)(data->data), data->data_len, 0);
}

/**
 * @brief  SLE Client Indication 回调（Server → Client 可靠推送）
 * @param  client_id 客户端实例 ID（未使用）
 * @param  conn_id   连接 ID（未使用）
 * @param  data      收到的 Indication 数据
 * @param  status    错误码（未使用）
 * @note   与 Notification 回调逻辑一致；Indication 需要 Client 应答确认，
 *         底层协议栈会自动完成 ACK，此处只负责数据透传。
 */
void sle_uart_indication_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data,
    errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);
    osal_printk("\n sle uart recived data : %s\r\n", data->data);
    uapi_uart_write(CONFIG_SLE_UART_BUS, (uint8_t *)(data->data), data->data_len, 0);
}

/**
 * @brief  Client 端 UART 接收中断处理函数
 * @param  buffer 接收到的原始数据缓冲区
 * @param  length 接收到的数据总字节数
 * @param  error  UART 接收错误标志（此处忽略）
 * @note   协议约定：buffer[0] 为目标 conn_id 的 ASCII 数字字符，
 *         buffer[1..] 为实际负载。此设计使 1-vs-8 场景下 Client 可通过
 *         首字节选择将数据发往哪条 SLE 连接。
 *         处理步骤：
 *         1. 提取首字节转为 conn_id；
 *         2. 获取全局发送参数结构体；
 *         3. 跳过首字节，将剩余数据通过 SSAP 写请求发送给 Server。
 */
static void sle_uart_client_read_int_handler(const void *buffer, uint16_t length, bool error)
{
    unused(error);

    uint8_t *buff = (uint8_t *)buffer;
    /* 首字节为 conn_id 的 ASCII 数字，例如 '0'-'7' 对应连接 0-7 */
    char num_str[2] = {buff[0], '\0'};
    ssapc_write_param_t *sle_uart_send_param = get_g_sle_uart_send_param();
    uint16_t g_sle_uart_conn_id = atoi(num_str);                            

    osal_printk("\n sle_uart_client_read_int_handler: %d\r\n", g_sle_uart_conn_id);
    sle_uart_send_param->data_len = length - 1;
    /* 跳过首字节（conn_id），将实际负载指针传给 SLE 发送接口 */
    sle_uart_send_param->data = (uint8_t *)buffer+1;
    ssapc_write_req(0, g_sle_uart_conn_id, sle_uart_send_param);
}

/* ---------- Client 主任务 ---------- */

/**
 * @brief  SLE UART Client 主任务函数
 * @param  arg 线程参数（未使用）
 * @return NULL
 * @note   处理步骤：
 *         1. 初始化 UART 引脚与配置；
 *         2. 注册 UART 接收回调（用于将串口数据转发到 SLE）；
 *         3. 初始化 SLE Client（注册 Notification / Indication 回调）；
 *         4. 任务完成后即退出（Client 靠回调驱动，无需主循环）。
 */
static void *sle_uart_client_task(const char *arg)
{
    unused(arg);
    /* 步骤 1：UART 引脚复用 & 串口参数初始化 */
    uart_init_pin();
    uart_init_config();

    /* 步骤 2：注册 UART 接收回调 */
    uapi_uart_unregister_rx_callback(CONFIG_SLE_UART_BUS);
    errcode_t ret = uapi_uart_register_rx_callback(CONFIG_SLE_UART_BUS,
                                                   UART_RX_CONDITION_FULL_OR_IDLE,
                                                   1, sle_uart_client_read_int_handler);
    /* 步骤 3：初始化 SLE Client，传入数据接收回调 */
    sle_uart_client_init(sle_uart_notification_cb, sle_uart_indication_cb);
    
    if (ret != ERRCODE_SUCC) {
        osal_printk("Register uart callback fail.");
        return NULL;
    }

    return NULL;
}
#endif  /* CONFIG_SAMPLE_SUPPORT_SLE_MESH_CLIENT */

/* ============================= 入口函数 ============================= */

/**
 * @brief  SLE UART 模块入口函数
 * @note   处理步骤：
 *         1. 获取内核线程锁（防止任务调度并发）；
 *         2. 根据编译宏创建 Server 或 Client 内核线程；
 *         3. 设置线程优先级为 SLE_UART_TASK_PRIO；
 *         4. 释放内核线程锁，允许新创建的线程被调度执行。
 *         本函数由 app_run 宏在系统启动阶段自动调用。
 */
static void sle_uart_entry(void)
{
    osal_task *task_handle = NULL;
    osal_kthread_lock();
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH_SERVER)
    task_handle = osal_kthread_create((osal_kthread_handler)sle_uart_server_task, 0, "SLEUartServerTask",
                                      SLE_UART_TASK_STACK_SIZE);
#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_MESH_CLIENT)
    task_handle = osal_kthread_create((osal_kthread_handler)sle_uart_client_task, 0, "SLEUartDongleTask",
                                      SLE_UART_TASK_STACK_SIZE);
#endif /* CONFIG_SAMPLE_SUPPORT_SLE_MESH_CLIENT */
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, SLE_UART_TASK_PRIO);
    }
    osal_kthread_unlock();
}

/* app_run 宏：将 sle_uart_entry 注册到系统启动序列，内核初始化完成后自动执行 */
app_run(sle_uart_entry);