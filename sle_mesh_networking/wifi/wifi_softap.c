/**
 * @file  wifi_softap.c
 * @brief WiFi SoftAP 热点初始化 (基于 HiSilicon WiFi API)
 */

#include "wifi_softap.h"
#include "lwip/netifapi.h"
#include "wifi_hotspot.h"
#include "wifi_hotspot_config.h"
#include "td_base.h"
#include "td_type.h"
#include "securec.h"
#include "cmsis_os2.h"
#include "soc_osal.h"
#include <stdio.h>

#define SOFTAP_LOG  "[wifi softap]"

#define WIFI_IFNAME_MAX_SIZE    16
#define WIFI_MAX_KEY_LEN        65
#define WIFI_MAX_SSID_LEN       33

int wifi_softap_start(const char *ssid)
{
    td_char ifname[WIFI_IFNAME_MAX_SIZE + 1] = "ap0";
    struct netif *netif_p = TD_NULL;
    ip4_addr_t st_gw, st_ipaddr, st_netmask;

    /* 等待 WiFi 子系统初始化完成 */
    while (wifi_is_wifi_inited() == 0) {
        (void)osDelay(10);
    }
    printf("%s wifi init done\r\n", SOFTAP_LOG);

    /* 配置 SoftAP 基本参数 */
    softap_config_stru hapd_conf = {0};
    (void)memcpy_s(hapd_conf.ssid, sizeof(hapd_conf.ssid),
                   ssid, strlen(ssid));
    (void)memcpy_s(hapd_conf.pre_shared_key, WIFI_MAX_KEY_LEN,
                   WIFI_SOFTAP_PWD, strlen(WIFI_SOFTAP_PWD));
    hapd_conf.security_type = 3;  /* WPA_WPA2_PSK */
    hapd_conf.channel_num   = WIFI_SOFTAP_CHANNEL;
    hapd_conf.wifi_psk_type = 0;

    /* 配置高级参数 */
    softap_config_advance_stru config = {0};
    config.beacon_interval  = 100;
    config.dtim_period      = 2;
    config.gi               = 0;
    config.group_rekey      = 86400;
    config.protocol_mode    = 4;  /* 802.11b/g/n/ax */
    config.hidden_ssid_flag = 1;  /* 不隐藏 */
    if (wifi_set_softap_config_advance(&config) != 0) {
        printf("%s set advance config failed\r\n", SOFTAP_LOG);
        return -1;
    }

    /* 启动 SoftAP */
    if (wifi_softap_enable(&hapd_conf) != 0) {
        printf("%s softap enable failed\r\n", SOFTAP_LOG);
        return -1;
    }

    /* 配置网络接口 IP 地址 */
    IP4_ADDR(&st_ipaddr,  192, 168, 43, 1);
    IP4_ADDR(&st_netmask, 255, 255, 255, 0);
    IP4_ADDR(&st_gw,      192, 168, 43, 1);

    netif_p = netif_find(ifname);
    if (netif_p == TD_NULL) {
        printf("%s netif_find(%s) failed\r\n", SOFTAP_LOG, ifname);
        (void)wifi_softap_disable();
        return -1;
    }
    if (netifapi_netif_set_addr(netif_p, &st_ipaddr, &st_netmask, &st_gw) != 0) {
        printf("%s set addr failed\r\n", SOFTAP_LOG);
        (void)wifi_softap_disable();
        return -1;
    }
    if (netifapi_dhcps_start(netif_p, NULL, 0) != 0) {
        printf("%s DHCP server start failed\r\n", SOFTAP_LOG);
        (void)wifi_softap_disable();
        return -1;
    }

    printf("%s SoftAP started: SSID=%s IP=192.168.43.1\r\n",
           SOFTAP_LOG, ssid);
    return 0;
}
