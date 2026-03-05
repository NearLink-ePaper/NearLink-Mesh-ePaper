/**
 * @file  sle_uart_server_adv.h
 * @brief SLE UART Server 广播（Advertising）模块头文件
 * @details 定义 SLE 广播相关的数据结构、枚举与接口声明。
 *          本模块负责：
 *          - 广播通用 TLV 数据结构定义（sle_adv_common_value）
 *          - 广播通道映射枚举（sle_adv_channel_map_t）
 *          - 广播数据类型枚举（sle_adv_data_type）
 *          - 对外提供设备注册、广播初始化、回调注册接口
 *
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * History: \n
 * 2023-07-17, Create file. \n
 */

#ifndef SLE_SERVER_ADV_H
#define SLE_SERVER_ADV_H

/* ============================= 数据结构定义 ============================= */

/**
 * @brief SLE 广播通用 TLV（Type-Length-Value）数据结构
 * @note  用于组装广播数据包中的各字段（如 discovery level、access mode、TX power 等）
 */
typedef struct sle_adv_common_value {
    uint8_t type;   /**< 数据类型，取值参见 sle_adv_data_type 枚举 */
    uint8_t length; /**< 数据长度（不含 length 字段本身），单位：字节 */
    uint8_t value;  /**< 数据值（单字节载荷） */
} le_adv_common_t;

/* ============================= 广播通道枚举 ============================= */

/**
 * @brief SLE 广播通道位图枚举
 * @note  可按位组合，DEFAULT (0x07) 表示同时在 77/78/79 三个通道上广播
 */
typedef enum sle_adv_channel {
    SLE_ADV_CHANNEL_MAP_77                 = 0x01, /**< 通道 77，bit0 */
    SLE_ADV_CHANNEL_MAP_78                 = 0x02, /**< 通道 78，bit1 */
    SLE_ADV_CHANNEL_MAP_79                 = 0x04, /**< 通道 79，bit2 */
    SLE_ADV_CHANNEL_MAP_DEFAULT            = 0x07  /**< 默认：三通道全开（0x01|0x02|0x04） */
} sle_adv_channel_map_t;

/* ============================= 广播数据类型枚举 ============================= */

/**
 * @brief SLE 广播数据类型（AD Type）枚举
 * @note  定义广播包 / 扫描响应包中各 TLV 字段的 type 值，
 *        取值与 SLE 规范保持一致
 */
typedef enum sle_adv_data {
    SLE_ADV_DATA_TYPE_DISCOVERY_LEVEL                              = 0x01,   /**< 发现等级 */
    SLE_ADV_DATA_TYPE_ACCESS_MODE                                  = 0x02,   /**< 接入层能力 */
    SLE_ADV_DATA_TYPE_SERVICE_DATA_16BIT_UUID                      = 0x03,   /**< 标准服务数据信息 */
    SLE_ADV_DATA_TYPE_SERVICE_DATA_128BIT_UUID                     = 0x04,   /**< 自定义服务数据信息 */
    SLE_ADV_DATA_TYPE_COMPLETE_LIST_OF_16BIT_SERVICE_UUIDS         = 0x05,   /**< 完整标准服务标识列表 */
    SLE_ADV_DATA_TYPE_COMPLETE_LIST_OF_128BIT_SERVICE_UUIDS        = 0x06,   /**< 完整自定义服务标识列表 */
    SLE_ADV_DATA_TYPE_INCOMPLETE_LIST_OF_16BIT_SERVICE_UUIDS       = 0x07,   /**< 部分标准服务标识列表 */
    SLE_ADV_DATA_TYPE_INCOMPLETE_LIST_OF_128BIT_SERVICE_UUIDS      = 0x08,   /**< 部分自定义服务标识列表 */
    SLE_ADV_DATA_TYPE_SERVICE_STRUCTURE_HASH_VALUE                 = 0x09,   /**< 服务结构散列值 */
    SLE_ADV_DATA_TYPE_SHORTENED_LOCAL_NAME                         = 0x0A,   /**< 设备缩写本地名称 */
    SLE_ADV_DATA_TYPE_COMPLETE_LOCAL_NAME                          = 0x0B,   /**< 设备完整本地名称 */
    SLE_ADV_DATA_TYPE_TX_POWER_LEVEL                               = 0x0C,   /**< 广播发送功率 */
    SLE_ADV_DATA_TYPE_SLB_COMMUNICATION_DOMAIN                     = 0x0D,   /**< SLB 通信域域名 */
    SLE_ADV_DATA_TYPE_SLB_MEDIA_ACCESS_LAYER_ID                    = 0x0E,   /**< SLB 媒体接入层标识 */
    SLE_ADV_DATA_TYPE_EXTENDED                                     = 0xFE,   /**< 数据类型扩展 */
    SLE_ADV_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA                   = 0xFF    /**< 厂商自定义信息 */
} sle_adv_data_type;

/* ============================= 接口声明 ============================= */

/**
 * @brief  注册 SLE 设备管理相关回调函数
 * @return ERRCODE_SLE_SUCCESS 成功；其他值表示失败错误码
 */
errcode_t sle_dev_register_cbks(void);

/**
 * @brief  SLE 广播初始化（配置参数 + 设置数据 + 启动广播）
 * @return ERRCODE_SLE_SUCCESS 成功；其他值表示失败错误码
 * @note   Mesh 模式下会根据 g_mesh_node_addr 动态生成 SLE 地址和广播名称；
 *         被 sle_uart_server.c enable_cbk 及 mesh_main.c 重连逻辑调用
 */
errcode_t sle_uart_server_adv_init(void);

/**
 * @brief  注册广播使能 / 去使能 / 终止回调
 * @return ERRCODE_SLE_SUCCESS 成功；其他值表示失败错误码
 * @note   Mesh 模式下 announce_terminal_cb 会设置 g_need_re_announce 标志以触发重新广播
 */
errcode_t sle_uart_announce_register_cbks(void);

#endif /* SLE_SERVER_ADV_H */