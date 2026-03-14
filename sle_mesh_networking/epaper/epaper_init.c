#include "app_init.h"
#include "cmsis_os2.h"
#include "gpio.h"
#include "osal_debug.h"
#include "pinctrl.h"
#include "soc_osal.h"
#include "spi.h"
#include "stdio.h"

#include "epaper.h"
#include "image_receiver.h"
#include "jpeg_decoder.h"
#include "securec.h"

#define SPI_WAIT_CYCLES 0x10

#define SPI_TASK_STACK_SIZE 0x1000  /* 4KB: waterline=0x498(1.2KB), 4KB 留 3x 余量。释放 4KB 堆给 WiFi+BT OAL */
#define SPI_TASK_DURATION_MS 1000
#define SPI_TASK_PRIO (osPriority_t)(17)

static osal_semaphore       g_epaper_sem;
static const uint8_t       *g_epaper_mesh_buf    = NULL;
static volatile uint32_t    g_epaper_mesh_buf_size = 0;
static volatile uint16_t    g_epaper_mesh_width  = 0;
static volatile uint16_t    g_epaper_mesh_height = 0;
static volatile uint8_t     g_epaper_mesh_mode   = 0;
static volatile uint16_t    g_epaper_mesh_data_size = 0;

void epaper_trigger_mesh_image(const uint8_t *buf, uint32_t buf_size,
                               uint16_t width, uint16_t height,
                               uint8_t mode, uint16_t data_size) {
    g_epaper_mesh_buf       = buf;
    g_epaper_mesh_buf_size  = buf_size;
    g_epaper_mesh_width     = width;
    g_epaper_mesh_height    = height;
    g_epaper_mesh_mode      = mode;
    g_epaper_mesh_data_size = data_size;
    osal_sem_up(&g_epaper_sem);
}

static void app_spi_init_pin(void) {
    uapi_pin_set_mode(CONFIG_EPD_RST_PIN, PIN_MODE_0);
    uapi_gpio_set_dir(CONFIG_EPD_RST_PIN, GPIO_DIRECTION_OUTPUT);

    uapi_pin_set_mode(CONFIG_EPD_DC_PIN, PIN_MODE_0);
    uapi_gpio_set_dir(CONFIG_EPD_DC_PIN, GPIO_DIRECTION_OUTPUT);

    uapi_pin_set_mode(CONFIG_EPD_CS_PIN, PIN_MODE_0);
    uapi_gpio_set_dir(CONFIG_EPD_CS_PIN, GPIO_DIRECTION_OUTPUT);

    uapi_pin_set_mode(CONFIG_EPD_PWR_PIN, PIN_MODE_3);
    uapi_gpio_set_dir(CONFIG_EPD_PWR_PIN, GPIO_DIRECTION_OUTPUT);

    uapi_pin_set_mode(CONFIG_EPD_BUSY_PIN, PIN_MODE_0);
    uapi_gpio_set_dir(CONFIG_EPD_BUSY_PIN, GPIO_DIRECTION_INPUT);

    uapi_pin_set_mode(CONFIG_EPD_DIN_PIN, PIN_MODE_3);
    uapi_pin_set_mode(CONFIG_EPD_CLK_PIN, PIN_MODE_3);
}

static void app_spi_master_init_config(void) {
    spi_attr_t config = {0};
    spi_extra_attr_t ext_config = {0};

    config.is_slave = false;
    config.slave_num = 1;
    config.bus_clk = 2000000;
    config.freq_mhz = 2;
    config.clk_polarity = SPI_CFG_CLK_CPOL_0;
    config.clk_phase = SPI_CFG_CLK_CPHA_0;
    config.frame_format = SPI_CFG_FRAME_FORMAT_MOTOROLA_SPI;
    config.spi_frame_format = HAL_SPI_FRAME_FORMAT_STANDARD;
    config.frame_size = HAL_SPI_FRAME_SIZE_8;
    config.tmod = HAL_SPI_TRANS_MODE_TXRX;
    config.sste = SPI_CFG_SSTE_DISABLE;

    ext_config.qspi_param.inst_len = HAL_SPI_INST_LEN_8;
    ext_config.qspi_param.addr_len = HAL_SPI_ADDR_LEN_16;
    ext_config.qspi_param.wait_cycles = SPI_WAIT_CYCLES;
    uapi_spi_init(CONFIG_EPD_MASTER_BUS_ID, &config, &ext_config);
}

static void Sample_Enter(void) {
    uapi_gpio_set_val(CONFIG_EPD_DC_PIN, GPIO_LEVEL_LOW);
    uapi_gpio_set_val(CONFIG_EPD_CS_PIN, GPIO_LEVEL_LOW);
    uapi_gpio_set_val(CONFIG_EPD_PWR_PIN, GPIO_LEVEL_HIGH);
    uapi_gpio_set_val(CONFIG_EPD_RST_PIN, GPIO_LEVEL_HIGH);
}

static void *epaper_init_task(const char *arg) {
    unused(arg);
    uapi_spi_deinit(CONFIG_EPD_MASTER_BUS_ID);

    app_spi_init_pin();
    app_spi_master_init_config();

    osal_mdelay(200);

    Sample_Enter();

    /* 首次上电：白屏 */
    EPD_Init();
    EPD_Clear();
    EPD_sleep();

    /* 循环等待 mesh 图像触发信号 */
    while (1) {
        if (osal_sem_down(&g_epaper_sem) == OSAL_SUCCESS) {
            const uint8_t *buf    = g_epaper_mesh_buf;
            uint32_t       buf_sz = g_epaper_mesh_buf_size;
            uint16_t       width  = g_epaper_mesh_width;
            uint16_t       height = g_epaper_mesh_height;
            uint8_t  mode      = g_epaper_mesh_mode;
            uint16_t data_size = g_epaper_mesh_data_size;
            if (buf != NULL) {
                printf("ePaper: displaying mesh image %ux%u mode=%d size=%u\r\n",
                       width, height, mode, data_size);
                EPD_Init();
                if (mode == IMG_MODE_JPEG) {
                    /* 流式解码: JPEG → 抖动 → 直接 SPI 输出, scratch 复用缓冲区末尾，零堆分配
                     * buf_sz 由调用方传入实际缓冲区大小，避免 gateway cache (50KB) 被
                     * 当作 image_receiver buf (96KB) 计算 scratch 导致越界 */
                    uint32_t scratch_avail = (data_size < buf_sz) ?
                        (buf_sz - data_size) : 0;
                    const uint8_t *decode_buf = buf;
                    /* 若原缓冲区 scratch 不足 (如 gateway 50KB cache)，
                     * 拷贝数据到 image_receiver 的 96KB 缓冲区再解码 */
                    if (scratch_avail < JPEG_SCRATCH_SIZE &&
                        data_size <= IMG_RX_BUF_SIZE - JPEG_SCRATCH_SIZE) {
                        uint8_t *rx_buf = image_receiver_get_buffer_writable();
                        (void)memcpy_s(rx_buf, IMG_RX_BUF_SIZE, buf, data_size);
                        decode_buf = rx_buf;
                        scratch_avail = IMG_RX_BUF_SIZE - data_size;
                        printf("ePaper: JPEG copied to rx_buf (%u→%lu scratch)\r\n",
                               data_size, (unsigned long)scratch_avail);
                    }
                    if (!jpeg_decode_stream_epd(decode_buf, (uint32_t)data_size,
                                               (uint8_t *)decode_buf + data_size, scratch_avail)) {
                        printf("ePaper: JPEG stream decode failed!\r\n");
                    }
                    image_receiver_reset(); /* 缓冲区数据已发送到 EPD, 可安全复用 */
                    EPD_refresh();
                } else if (mode == IMG_MODE_RAW) {
                    /* 4bpp nibble (converterTo6color 格式): portrait → 固件旋转 90° CW */
                    EPD_display_4bpp(buf, width, height);
                    image_receiver_reset(); /* EPD_display_4bpp 内含 refresh */
                } else {
                    EPD_display_1bpp(buf, width, height);
                    image_receiver_reset(); /* EPD_display_1bpp 内含 refresh, 数据已用完 */
                }
                EPD_sleep();
                printf("ePaper: display done\r\n");
            }
        }
    }

    return NULL;
}

static void epaper_init_entry(void) {
    osThreadAttr_t attr;

    if (osal_sem_binary_sem_init(&g_epaper_sem, 0) != OSAL_SUCCESS) {
        printf("Create ePaper semaphore fail.\r\n");
        return;
    }

    attr.name = "ePaperInitTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = SPI_TASK_STACK_SIZE;
    attr.priority = SPI_TASK_PRIO;

    if (osThreadNew((osThreadFunc_t)epaper_init_task, NULL, &attr) == NULL) {
        printf("Create task ePaperInit fail.\r\n");
    }
}

app_run(epaper_init_entry);
