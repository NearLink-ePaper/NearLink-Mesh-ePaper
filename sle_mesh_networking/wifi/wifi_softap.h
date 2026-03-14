/**
 * @file  wifi_softap.h
 * @brief WiFi SoftAP 热点初始化 — 供 PC 端通过 Socket 传输图片
 */
#ifndef WIFI_SOFTAP_H
#define WIFI_SOFTAP_H

#include <stdint.h>
#include <stdbool.h>

/** SoftAP 固定参数 */
#define WIFI_SOFTAP_PWD         "12345678"
#define WIFI_SOFTAP_IP          "192.168.43.1"
#define WIFI_SOFTAP_CHANNEL     6

/**
 * @brief  启动 SoftAP 热点 (阻塞直到成功)
 * @param  ssid  热点名称，建议与 BLE 广播名称一致 (如 sle_gw_XXXX)
 * @return 0=成功, -1=失败
 */
int wifi_softap_start(const char *ssid);

#endif /* WIFI_SOFTAP_H */
