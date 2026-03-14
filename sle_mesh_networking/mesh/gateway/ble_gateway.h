/**
 * @file   ble_gateway.h
 * @brief  BLE Gateway — BLE+SLE 共存手机桥接模块
 *
 * @details
 *   本模块实现了 BLE Gateway 功能，作为手机 APP 与 SLE Mesh 网络之间的桥梁：
 *
 *   架构：Phone ←BLE GATT→ Gateway ←SLE Mesh→ Relay(s) ←SLE→ Receiver
 *
 *   核心功能：
 *     1. BLE GATT Server：提供自定义 Service/Characteristic 对，
 *        手机通过 Write 下发命令，Gateway 通过 Notify 上报响应。
 *     2. 图片流控引擎 (FC)：基于 AIMD 拥塞控制算法，将手机下发的
 *        图片数据通过 Mesh 网络可靠传输到目标节点（电子墨水屏）。
 *        支持 CHECKPOINT 机制、丢包位图重传、跳数自适应参数。
 *     3. 拓扑收集：通过广播请求/单播响应收集全网拓扑信息。
 *     4. ACK 代理 (v2.2)：网关先本地缓存所有数据，向手机回复 ACK，
 *        然后通过 FC 引擎可靠注入 Mesh。
 *
 *   流控 (FC) 状态机：
 *     FC_IDLE → FC_START_WAIT → FC_SENDING → FC_WAIT_CHKPT →
 *     FC_SEND_END → FC_WAIT_RESULT → FC_RETRANSMIT → [循环或完成]
 *
 * @note 本模块同时使用 BLE 和 SLE 双射频协议栈，初始化时需确保
 *       enable_ble() 和 enable_sle() 均已成功调用。
 */
#ifndef BLE_GATEWAY_H
#define BLE_GATEWAY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化 BLE Gateway 模块
 *
 * 注册 GAP/GATTS 回调、使能 BLE、创建 GATT Service/Characteristic、
 * 设置 BLE 广播参数并启动广播。调用此函数前 SLE 部分需已初始化完成
 * （因为 BLE 广播名称包含 Mesh 地址）。
 */
void ble_gateway_init(void);

/**
 * @brief  通过 BLE GATT Notify 向手机发送数据
 *
 * @param src_addr  数据来源的 Mesh 地址（用于组装上行帧头）
 * @param data      有效载荷指针
 * @param len       有效载荷长度（自动截断到 MTU 限制内）
 * @return 0 成功, -1 未连接, -3 notify 失败
 */
int ble_gateway_send_to_phone(uint16_t src_addr, const uint8_t *data, uint16_t len);

/**
 * @brief  查询手机是否已通过 BLE 连接到 Gateway
 * @return true 已连接, false 未连接
 */
bool ble_gateway_is_connected(void);

/**
 * @brief  查询 Gateway 是否需要重新启动 BLE 广播等待手机重连
 * @return true 需要重连, false 不需要
 */
bool ble_gateway_needs_reconnect(void);

/**
 * @brief  清除重连标志（主循环处理完重连逻辑后调用）
 */
void ble_gateway_clear_reconnect(void);

/**
 * @brief  接收拓扑响应数据（由 mesh_main.c 在收到 0xFE 帧时调用）
 * @param data  拓扑响应 payload
 * @param len   payload 长度
 */
void ble_gateway_on_topo_resp(const uint8_t *data, uint16_t len);

/**
 * @brief  拓扑收集超时检查（在主循环中周期调用）
 *
 * 当拓扑收集窗口超时后，将所有已收集的响应打包通过 BLE 发送给手机。
 */
void ble_gateway_topo_tick(void);

/**
 * @brief  图片流控状态机驱动（在主循环中周期调用）
 *
 * 根据 FC 当前状态执行相应操作：
 *   - FC_START_WAIT : 等待 startDelay 后发送 Turbo ON + IMG_START
 *   - FC_SENDING    : 按窗口批量发送数据包，窗口用尽后发 CHECKPOINT
 *   - FC_WAIT_CHKPT : 等待接收端 CHKPT_ACK 或超时重传
 *   - FC_SEND_END   : 发送 IMG_END 帧
 *   - FC_WAIT_RESULT: 等待接收端 RESULT 或超时重传
 *   - FC_RETRANSMIT : 根据 MISSING 位图补发丢失的数据包
 */
void ble_gateway_img_tick(void);

/**
 * @brief  查询流控引擎是否处于活跃传输状态
 *
 * 用于主循环动态调整 tick 间隔：
 *   FC 活跃时使用 MESH_FC_TICK_MS (10ms)，空闲时使用
 *   MESH_MAIN_LOOP_INTERVAL_MS (100ms)。
 *
 * @return true FC 正在传输, false FC 空闲 (FC_IDLE)
 */
bool ble_gateway_fc_is_active(void);

/**
 * @brief  处理图片传输的上行响应消息
 *
 * 由 mesh_main.c 在收到来自接收端的图片协议帧 (0x85~0x88) 时调用。
 * 根据命令字节执行不同操作：
 *   - 0x85 (ACK)       : FAST 模式下转发给手机
 *   - 0x86 (RESULT)    : 停止 FC 引擎 + 恢复扫描/Turbo + 转发给手机
 *   - 0x87 (MISSING)   : 解析丢包位图，填充重传列表 + 触发 FC_RETRANSMIT
 *   - 0x88 (CHKPT_ACK) : 更新 FC 状态机（AIMD 窗口调整 + 继续发送）
 *
 * @param src_addr  发送端 Mesh 地址（即接收端节点地址）
 * @param data      响应帧 payload（已去除 mesh 帧头）
 * @param len       payload 长度
 * @return 0 处理成功
 */
int ble_gateway_send_image_response(uint16_t src_addr, const uint8_t *data, uint16_t len);

/**
 * @brief  启动 WiFi TOPO 查询（广播 TOPO_REQ，收集响应）
 */
void ble_gateway_start_wifi_topo(void);

/**
 * @brief  读取 WiFi TOPO 查询结果，返回节点数
 * @param  addrs  输出: 节点地址数组
 * @param  hops   输出: 对应节点的跳数
 * @param  max    数组最大容量
 * @return 实际节点数
 */
uint8_t ble_gateway_read_wifi_topo(uint16_t *addrs, uint8_t *hops, uint8_t max);

/**
 * @brief  获取图片缓存区 (g_img_cache.buf)，供 WiFi 任务直接写入图片数据
 */
uint8_t *ble_gateway_get_wifi_img_buf(void);

/**
 * @brief  返回 WiFi 图片缓存区大小 (字节)，供外部作大小上限检查
 */
uint32_t ble_gateway_wifi_img_buf_size(void);

/**
 * @brief  设置 WiFi FC 转发元数据并触发传输
 *         图片数据必须已通过 ble_gateway_get_wifi_img_buf() 写入缓存
 * @return 0=成功, 负=错误(FC忙/OOM/阶载带)
 */
int ble_gateway_wifi_setup_fc(uint16_t dst_addr, uint32_t data_size,
                              uint16_t w, uint16_t h, uint8_t mode);

/**
 * @brief  获取 WiFi FC 传输结果
 * @return 0=成功, 1=失败, -1=进行中
 */
int8_t ble_gateway_wifi_get_fc_result(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_GATEWAY_H */