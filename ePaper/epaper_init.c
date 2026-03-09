#include "app_init.h"
#include "cmsis_os2.h"
#include "gpio.h"
#include "osal_debug.h"
#include "pinctrl.h"
#include "soc_osal.h"
#include "spi.h"
#include "stdio.h"

#include "epaper.h"

#define SPI_WAIT_CYCLES 0x10

#define SPI_TASK_STACK_SIZE 0x1000
#define SPI_TASK_DURATION_MS 1000
#define SPI_TASK_PRIO (osPriority_t)(17)

static osal_semaphore       g_epaper_sem;
static const uint8_t       *g_epaper_mesh_buf    = NULL;
static volatile uint16_t    g_epaper_mesh_width  = 0;
static volatile uint16_t    g_epaper_mesh_height = 0;

void epaper_trigger_mesh_image(const uint8_t *buf, uint16_t width, uint16_t height) {
    g_epaper_mesh_buf    = buf;
    g_epaper_mesh_width  = width;
    g_epaper_mesh_height = height;
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
            uint16_t       width  = g_epaper_mesh_width;
            uint16_t       height = g_epaper_mesh_height;
            if (buf != NULL) {
                printf("ePaper: displaying mesh image %ux%u\r\n", width, height);
                EPD_Init();
                EPD_display_1bpp(buf, width, height);
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
