/**
 * @file  wifi_socket_server.c
 * @brief TCP Socket 服务器 — 接收 PC 端图片并触发 ePaper 显示
 */

#include "wifi_socket_server.h"
#include "image_receiver.h"
#include "epaper.h"
#include "watchdog.h"
#include "ble_gateway.h"
#include "soc_osal.h"
#include "cmsis_os2.h"
#include "securec.h"
#include "lwip/sockets.h"
#include <stdio.h>
#include <string.h>

#define SOCK_LOG    "[wifi sock]"

static char g_gw_name[20] = "sle_gw_0000";  /* 网关名称, 由 wifi_socket_server_set_name 设置 */

void wifi_socket_server_set_name(const char *name)
{
    if (name) {
        (void)strncpy(g_gw_name, name, sizeof(g_gw_name) - 1);
        g_gw_name[sizeof(g_gw_name) - 1] = '\0';
    }
}

#define SERVER_TASK_STACK   0x1800  /* 6KB: waterline=0x498(1.2KB), lwip recv/send 需要额外栈空间。释放 2KB 堆给 WiFi+BT OAL */
#define SERVER_TASK_PRIO    (osPriority_t)(15)

/**
 * @brief 从 socket 精确读取 len 字节
 * @return 0=成功, -1=失败/断开
 */
static int recv_exact(int fd, uint8_t *buf, uint32_t len)
{
    uint32_t got = 0;
    while (got < len) {
        int n = lwip_recv(fd, buf + got, (int)(len - got), 0);
        if (n <= 0) return -1;
        got += (uint32_t)n;
        uapi_watchdog_kick();
    }
    return 0;
}

/**
 * @brief 处理一个客户端连接
 */
static void handle_client(int client_fd)
{
    uint8_t header[WIFI_IMG_HEADER_SIZE];
    uint8_t resp;
    uint16_t width, height;
    uint8_t  mode;
    uint32_t data_size;
    uint8_t *img_buf;

    /* 读第一字节: 判断是探测(0xFE)、命令(0xFD)还是图片传输(0xAA) */
    {
        int n = lwip_recv(client_fd, &header[0], 1, 0);
        if (n <= 0) {
            printf("%s header recv failed\r\n", SOCK_LOG);
            return;
        }
    }

    /* ── 0xFE: 探测，回复网关名称 ── */
    if (header[0] == WIFI_PROBE_MAGIC) {
        lwip_send(client_fd, g_gw_name, (int)strlen(g_gw_name), 0);
        printf("%s probe: name='%s'\r\n", SOCK_LOG, g_gw_name);
        return;
    }

    /* ── 0xFD: 命令帧，读取子命令 ── */
    if (header[0] == WIFI_CMD_MAGIC) {
        uint8_t subcmd = 0;
        if (lwip_recv(client_fd, &subcmd, 1, 0) <= 0) return;
        if (subcmd == WIFI_CMD_TOPO_QUERY) {
            /* 启动 TOPO 查询，等待收集完成 */
            ble_gateway_start_wifi_topo();
            osDelay(3500);  /* 等待 3.5s 收集响应 */
            uint16_t addrs[16];
            uint8_t  hops_arr[16];
            uint8_t  count = ble_gateway_read_wifi_topo(addrs, hops_arr, 16);
            /* 回复: [count(1)] [addr_hi addr_lo hops(1)] ... */
            uint8_t resp_buf[1 + 16 * 3];
            resp_buf[0] = count;
            for (uint8_t i = 0; i < count; i++) {
                resp_buf[1 + i * 3 + 0] = (uint8_t)(addrs[i] >> 8);
                resp_buf[1 + i * 3 + 1] = (uint8_t)(addrs[i] & 0xFF);
                resp_buf[1 + i * 3 + 2] = hops_arr[i];
            }
            lwip_send(client_fd, resp_buf, (int)(1 + count * 3), 0);
            printf("%s TOPO: %d nodes\r\n", SOCK_LOG, count);
        }
        return;
    }

    /* ── 0xAA: 图片传输 ── */
    if (header[0] != WIFI_IMG_MAGIC_0) {
        printf("%s bad magic[0]: %02X\r\n", SOCK_LOG, header[0]);
        resp = WIFI_IMG_RESP_FAIL;
        lwip_send(client_fd, &resp, 1, 0);
        return;
    }
    /* 读剩余 13 字节头部 (新协议头共 14B) */
    if (recv_exact(client_fd, &header[1], WIFI_IMG_HEADER_SIZE - 1) != 0) {
        printf("%s header recv failed\r\n", SOCK_LOG);
        return;
    }
    if (header[1] != WIFI_IMG_MAGIC_1) {
        printf("%s bad magic[1]: %02X\r\n", SOCK_LOG, header[1]);
        resp = WIFI_IMG_RESP_FAIL;
        lwip_send(client_fd, &resp, 1, 0);
        return;
    }

    /* 解析 14 字节协议头:
     * [0-1] magic  [2-3] width  [4-5] height  [6] mode
     * [7] dst_hi   [8] dst_lo   [9] reserved   [10-13] data_size */
    width    = ((uint16_t)header[2] << 8) | header[3];
    height   = ((uint16_t)header[4] << 8) | header[5];
    mode     = header[6];
    uint16_t dst_addr = ((uint16_t)header[7] << 8) | header[8];
    /* header[9] reserved */
    data_size = ((uint32_t)header[10] << 24) | ((uint32_t)header[11] << 16) |
                ((uint32_t)header[12] << 8)  | header[13];

    printf("%s header: %ux%u mode=%u dst=0x%04X size=%lu\r\n",
           SOCK_LOG, width, height, mode, dst_addr, (unsigned long)data_size);

    /* 互斥检查: BLE 已连接时拒绝 WiFi 传图 */
    if (ble_gateway_is_connected()) {
        printf("%s BLE active, reject WiFi transfer\r\n", SOCK_LOG);
        resp = WIFI_IMG_RESP_BUSY;
        lwip_send(client_fd, &resp, 1, 0);
        return;
    }

    /* ── dst_addr != 0xFFFF: Mesh 转发路径 ── */
    if (dst_addr != WIFI_IMG_DST_LOCAL) {
        if (data_size > ble_gateway_wifi_img_buf_size()) {
            printf("%s size %lu > cache OOM\r\n", SOCK_LOG, (unsigned long)data_size);
            resp = WIFI_IMG_RESP_OOM;
            lwip_send(client_fd, &resp, 1, 0);
            return;
        }
        /* 直接接收到 g_img_cache.buf */
        img_buf = ble_gateway_get_wifi_img_buf();
        if (recv_exact(client_fd, img_buf, data_size) != 0) {
            printf("%s mesh data recv failed\r\n", SOCK_LOG);
            resp = WIFI_IMG_RESP_FAIL;
            lwip_send(client_fd, &resp, 1, 0);
            return;
        }
        /* 设置 FC 元数据并触发传输 */
        if (ble_gateway_wifi_setup_fc(dst_addr, data_size, width, height, mode) != 0) {
            printf("%s FC busy/fail\r\n", SOCK_LOG);
            resp = WIFI_IMG_RESP_FAIL;
            lwip_send(client_fd, &resp, 1, 0);
            return;
        }
        /* 等待 FC 完成 (轮询, 最长 120s) */
        uint32_t deadline = osKernelGetTickCount() + 120000U;
        while (ble_gateway_wifi_get_fc_result() < 0) {
            osDelay(200);
            if (osKernelGetTickCount() > deadline) {
                printf("%s mesh FC timeout\r\n", SOCK_LOG);
                resp = WIFI_IMG_RESP_FAIL;
                lwip_send(client_fd, &resp, 1, 0);
                return;
            }
            uapi_watchdog_kick();
        }
        resp = (ble_gateway_wifi_get_fc_result() == 0) ? WIFI_IMG_RESP_OK : WIFI_IMG_RESP_FAIL;
        printf("%s mesh FC done: %s\r\n", SOCK_LOG, resp == WIFI_IMG_RESP_OK ? "OK" : "FAIL");
        lwip_send(client_fd, &resp, 1, 0);
        return;
    }

    /* ── dst_addr == 0xFFFF: 本地 ePaper 显示路径 ── */
    if (data_size > IMG_RX_BUF_SIZE) {
        printf("%s data_size %lu > buf %d, OOM\r\n",
               SOCK_LOG, (unsigned long)data_size, IMG_RX_BUF_SIZE);
        resp = WIFI_IMG_RESP_OOM;
        lwip_send(client_fd, &resp, 1, 0);
        return;
    }
    {
        const img_rx_info_t *info = image_receiver_get_info();
        if (info->state == IMG_STATE_RECEIVING) {
            printf("%s busy (BLE receiving), reject\r\n", SOCK_LOG);
            resp = WIFI_IMG_RESP_FAIL;
            lwip_send(client_fd, &resp, 1, 0);
            return;
        }
    }
    img_buf = image_receiver_get_buffer_writable();
    if (recv_exact(client_fd, img_buf, data_size) != 0) {
        printf("%s data recv failed\r\n", SOCK_LOG);
        resp = WIFI_IMG_RESP_FAIL;
        lwip_send(client_fd, &resp, 1, 0);
        return;
    }
    printf("%s received %lu bytes OK\r\n", SOCK_LOG, (unsigned long)data_size);
    epaper_trigger_mesh_image(img_buf, IMG_RX_BUF_SIZE, width, height, mode, (uint16_t)data_size);
    resp = WIFI_IMG_RESP_OK;
    lwip_send(client_fd, &resp, 1, 0);
    printf("%s triggered ePaper\r\n", SOCK_LOG);
}

/**
 * @brief TCP 服务器主循环
 */
static void *server_task(const char *arg)
{
    (void)arg;
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len;
    int opt = 1;

    /* 创建 socket */
    server_fd = lwip_socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        printf("%s socket() failed\r\n", SOCK_LOG);
        return NULL;
    }

    lwip_setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    /* 绑定地址 */
    (void)memset_s(&server_addr, sizeof(server_addr), 0, sizeof(server_addr));
    server_addr.sin_family      = AF_INET;
    server_addr.sin_port        = htons(WIFI_IMG_SERVER_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (lwip_bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        printf("%s bind() failed\r\n", SOCK_LOG);
        lwip_close(server_fd);
        return NULL;
    }

    if (lwip_listen(server_fd, 1) < 0) {
        printf("%s listen() failed\r\n", SOCK_LOG);
        lwip_close(server_fd);
        return NULL;
    }

    printf("%s listening on port %d\r\n", SOCK_LOG, WIFI_IMG_SERVER_PORT);

    /* 主循环: 接受连接并处理 */
    while (1) {
        addr_len = sizeof(client_addr);
        client_fd = lwip_accept(server_fd, (struct sockaddr *)&client_addr, &addr_len);
        if (client_fd < 0) {
            osal_msleep(100);
            continue;
        }
        printf("%s client connected\r\n", SOCK_LOG);
        /* 设置 recv 超时: 防止探测连接或死连接长期阻塞 accept 循环 */
        {
            struct timeval tv = { .tv_sec = 10, .tv_usec = 0 };
            lwip_setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        }
        handle_client(client_fd);
        lwip_close(client_fd);
        printf("%s client disconnected\r\n", SOCK_LOG);
    }

    lwip_close(server_fd);
    return NULL;
}

void wifi_socket_server_start(void)
{
    osThreadAttr_t attr = {0};
    attr.name       = "wifi_img_server";
    attr.stack_size = SERVER_TASK_STACK;
    attr.priority   = SERVER_TASK_PRIO;

    if (osThreadNew((osThreadFunc_t)server_task, NULL, &attr) == NULL) {
        printf("%s create task failed\r\n", SOCK_LOG);
    } else {
        printf("%s task created\r\n", SOCK_LOG);
    }
}
