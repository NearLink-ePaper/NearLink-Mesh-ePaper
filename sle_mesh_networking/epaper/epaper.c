#include "epaper.h"
#include "gpio.h"
#include "pinctrl.h"
#include "soc_osal.h"
#include "spi.h"
#include "stdio.h"
#include "watchdog.h"

unsigned char EPD_Flag = 0;

/// @brief 硬件复位
void EPD_Reset(void) {
    uapi_gpio_set_val(CONFIG_EPD_RST_PIN, GPIO_LEVEL_HIGH);
    osal_mdelay(200);
    uapi_gpio_set_val(CONFIG_EPD_RST_PIN, GPIO_LEVEL_LOW);
    osal_mdelay(2);
    uapi_gpio_set_val(CONFIG_EPD_RST_PIN, GPIO_LEVEL_HIGH);
    osal_mdelay(200);
}

static void uapi_spi_master_write_byte(uint8_t byte) {
    spi_xfer_data_t data = {0};
    uint8_t buff[] = {byte};
    data.tx_buff = buff;
    data.tx_bytes = sizeof(buff);
    uint32_t ret = uapi_spi_master_write(CONFIG_EPD_MASTER_BUS_ID, &data, 1000);
    if (ret != 0) {
        printf("spi = %0x master send failed\r\n", ret);
    }
}

/// @brief 发送指令
/// @param Reg 指令字节
void EPD_SendCommand(uint8_t Reg) {
    uapi_gpio_set_val(CONFIG_EPD_DC_PIN, GPIO_LEVEL_LOW);
    uapi_gpio_set_val(CONFIG_EPD_CS_PIN, GPIO_LEVEL_LOW);
    uapi_spi_master_write_byte(Reg);
    uapi_gpio_set_val(CONFIG_EPD_CS_PIN, GPIO_LEVEL_HIGH);
}

/// @brief 发送数据
/// @param Data 数据字节
void EPD_SendData(uint8_t Data) {
    uapi_gpio_set_val(CONFIG_EPD_DC_PIN, GPIO_LEVEL_HIGH);
    uapi_gpio_set_val(CONFIG_EPD_CS_PIN, GPIO_LEVEL_LOW);
    uapi_spi_master_write_byte(Data);
    uapi_gpio_set_val(CONFIG_EPD_CS_PIN, GPIO_LEVEL_HIGH);
}

/// @brief 等待 BUSY 引脚变高（屏幕就绪），最长等待 40 秒
static void EPD_ReadBusy(void) {
    printf("e-Paper busy\r\n");
    uint8_t busy;
    uint32_t timeout = 40000; /* 40s = 40000 × 1ms */
    do {
        busy = uapi_gpio_get_val(CONFIG_EPD_BUSY_PIN);
        uapi_watchdog_kick();
        osal_mdelay(1);
        timeout--;
        if (timeout == 0) {
            printf("e-Paper busy TIMEOUT! Check BUSY pin (GPIO%d).\r\n",
                   CONFIG_EPD_BUSY_PIN);
            break;
        }
    } while (!busy);
    osal_mdelay(200);
    printf("e-Paper busy release (remaining=%lu)\r\n", (unsigned long)timeout);
}

/// @brief TurnOnDisplay: Booster 二次设置 → DISPLAY_REFRESH → POWER_OFF
void EPD_refresh(void) {
    EPD_SendCommand(0x04); /* POWER ON */
    EPD_ReadBusy();

    /* Booster soft start second setting */
    EPD_SendCommand(0x06);
    EPD_SendData(0x6F);
    EPD_SendData(0x1F);
    EPD_SendData(0x17);
    EPD_SendData(0x49);

    EPD_SendCommand(0x12); /* DISPLAY REFRESH */
    EPD_SendData(0x00);
    EPD_ReadBusy();

    EPD_SendCommand(0x02); /* POWER OFF */
    EPD_SendData(0x00);
    EPD_ReadBusy();
}

/// @brief LUT_GC 占位（保留接口兼容性）
void EPD_lut_GC(void) {
}

/// @brief LUT_DU 占位（保留接口兼容性）
void EPD_lut_DU(void) {
}

/// @brief 初始化 7.3inch e-Paper (E) 寄存器序列
void EPD_Init(void) {
    EPD_Flag = 0;
    EPD_Reset();
    EPD_ReadBusy();
    osal_mdelay(30);

    EPD_SendCommand(0xAA); /* CMDH */
    EPD_SendData(0x49);
    EPD_SendData(0x55);
    EPD_SendData(0x20);
    EPD_SendData(0x08);
    EPD_SendData(0x09);
    EPD_SendData(0x18);

    EPD_SendCommand(0x01);
    EPD_SendData(0x3F);

    EPD_SendCommand(0x00);
    EPD_SendData(0x5F);
    EPD_SendData(0x69);

    EPD_SendCommand(0x03);
    EPD_SendData(0x00);
    EPD_SendData(0x54);
    EPD_SendData(0x00);
    EPD_SendData(0x44);

    EPD_SendCommand(0x05);
    EPD_SendData(0x40);
    EPD_SendData(0x1F);
    EPD_SendData(0x1F);
    EPD_SendData(0x2C);

    EPD_SendCommand(0x06); /* Booster soft start first setting */
    EPD_SendData(0x6F);
    EPD_SendData(0x1F);
    EPD_SendData(0x17);
    EPD_SendData(0x49);

    EPD_SendCommand(0x08);
    EPD_SendData(0x6F);
    EPD_SendData(0x1F);
    EPD_SendData(0x1F);
    EPD_SendData(0x22);

    EPD_SendCommand(0x30);
    EPD_SendData(0x03);

    EPD_SendCommand(0x50);
    EPD_SendData(0x3F);

    EPD_SendCommand(0x60);
    EPD_SendData(0x02);
    EPD_SendData(0x00);

    EPD_SendCommand(0x61); /* 分辨率: 800 x 480 */
    EPD_SendData(0x03);    /* 0x0320 = 800 */
    EPD_SendData(0x20);
    EPD_SendData(0x01);    /* 0x01E0 = 480 */
    EPD_SendData(0xE0);

    EPD_SendCommand(0x84);
    EPD_SendData(0x01);

    EPD_SendCommand(0xE3);
    EPD_SendData(0x2F);

    EPD_SendCommand(0x04); /* POWER ON */
    EPD_ReadBusy();
}

/// @brief 发送整帧图像数据并刷新
/// @param picData 图像缓冲区，格式：4bpp packed，每字节含2像素，
///                大小 = EPD_WIDTH/2 * EPD_HEIGHT = 192000 字节
void EPD_display(uint8_t *picData) {
    uint32_t i, j;
    EPD_SendCommand(0x10);
    for (i = 0; i < EPD_HEIGHT; i++) {
        uapi_watchdog_kick();
        for (j = 0; j < EPD_WIDTH / 2; j++) {
            EPD_SendData(picData[j + (EPD_WIDTH / 2) * i]);
        }
    }
    EPD_refresh();
}

/// @brief 以指定颜色填充整屏并刷新
/// @param color 颜色索引（EPD_BLACK / EPD_WHITE / EPD_RED 等 0x0~0x6）
void EPD_display_NUM(uint8_t color) {
    uint32_t i, j;
    uint8_t byte = (uint8_t)((color << 4) | color);
    EPD_SendCommand(0x10);
    for (i = 0; i < EPD_HEIGHT; i++) {
        uapi_watchdog_kick();
        for (j = 0; j < EPD_WIDTH / 2; j++) {
            EPD_SendData(byte);
        }
    }
    EPD_refresh();
}

/// @brief 在指定区域显示局部图像，其余区域填充白色
/// @param picData      局部图像缓冲区，4bpp packed，行步长 = image_width/2
/// @param xstart       起始列（像素，须为偶数）
/// @param ystart       起始行（像素）
/// @param image_width  图像宽度（像素，须为偶数）
/// @param image_height 图像高度（像素）
void EPD_display_part(const uint8_t *picData, uint16_t xstart, uint16_t ystart,
                      uint16_t image_width, uint16_t image_height) {
    uint32_t i, j;
    uint16_t x_byte_start = xstart / 2;
    uint16_t x_byte_end   = x_byte_start + image_width / 2;
    EPD_SendCommand(0x10);
    for (i = 0; i < EPD_HEIGHT; i++) {
        uapi_watchdog_kick();
        for (j = 0; j < EPD_WIDTH / 2; j++) {
            if (i >= ystart && i < (uint32_t)(ystart + image_height) &&
                j >= x_byte_start && j < x_byte_end) {
                EPD_SendData(picData[(j - x_byte_start) + (image_width / 2) * (i - ystart)]);
            } else {
                EPD_SendData((EPD_WHITE << 4) | EPD_WHITE);
            }
        }
    }
    EPD_refresh();
}

/// @brief 显示 6 色色块演示（每色横跨整屏宽，共 6 个等高色带）
void EPD_Show7Block(void) {
    uint32_t i, j;
    const uint8_t colors[6] = {
        EPD_BLACK, EPD_YELLOW, EPD_RED, EPD_BLUE, EPD_GREEN, EPD_WHITE
    };
    EPD_SendCommand(0x10);
    for (i = 0; i < EPD_HEIGHT; i++) {
        uapi_watchdog_kick();
        uint8_t color = colors[(i * 6) / EPD_HEIGHT];
        for (j = 0; j < EPD_WIDTH / 2; j++) {
            EPD_SendData((color << 4) | color);
        }
    }
    EPD_refresh();
}

/// @brief 清屏（填充白色并刷新）
void EPD_Clear(void) {
    uint32_t i, j;
    EPD_SendCommand(0x10);
    for (i = 0; i < EPD_HEIGHT; i++) {
        uapi_watchdog_kick();
        for (j = 0; j < EPD_WIDTH / 2; j++) {
            EPD_SendData((EPD_WHITE << 4) | EPD_WHITE);
        }
    }
    EPD_refresh();
}

/// @brief 将 1bpp 黑白图像流式转换为 4bpp 并显示（无需额外缓冲区）
/// @param buf    1bpp 数据：高位先，每行 ceil(width/8) 字节，bit=1→黑，bit=0→白
/// @param width  图像宽度（像素），超出 EPD_WIDTH 时右侧补白
/// @param height 图像高度（像素），超出 EPD_HEIGHT 时下方补白
void EPD_display_1bpp(const uint8_t *buf, uint16_t width, uint16_t height) {
    uint32_t erow, ecol;
    uint16_t bpr = (uint16_t)((width + 7) / 8); /* bytes per source row */
    EPD_SendCommand(0x10);
    /* 90° CW rotation: portrait(width×height) → landscape(EPD_WIDTH×EPD_HEIGHT)
     * Formula: ePaper(erow, ecol) ← source(height-1-ecol, erow)
     * Valid for width==EPD_HEIGHT(480) && height==EPD_WIDTH(800) */
    for (erow = 0; erow < EPD_HEIGHT; erow++) {
        uapi_watchdog_kick();
        for (ecol = 0; ecol < EPD_WIDTH / 2; ecol++) {
            uint32_t lx = ecol * 2;      /* left pixel x on ePaper */
            uint32_t rx = ecol * 2 + 1;  /* right pixel x on ePaper */
            uint8_t left, right;
            /* left pixel: source row = height-1-lx, source col = erow */
            if (lx < height && erow < width) {
                uint32_t sr = height - 1 - lx;
                left = (buf[sr * bpr + erow / 8] >> (7 - (erow & 7))) & 1
                       ? EPD_BLACK : EPD_WHITE;
            } else {
                left = EPD_WHITE;
            }
            /* right pixel: source row = height-1-rx, source col = erow */
            if (rx < height && erow < width) {
                uint32_t sr = height - 1 - rx;
                right = (buf[sr * bpr + erow / 8] >> (7 - (erow & 7))) & 1
                        ? EPD_BLACK : EPD_WHITE;
            } else {
                right = EPD_WHITE;
            }
            EPD_SendData((uint8_t)((left << 4) | right));
        }
    }
    EPD_refresh();
}

/// @brief 显示 4bpp packed 图像数据并刷新（含 90° CW 旋转）
/// @param buf    4bpp packed 数据：每字节 2 像素 (高nibble=左, 低nibble=右)
///               行步长 = width/2 字节, 总大小 = (width/2)*height
/// @param width  图像宽度（像素，须为偶数），对应 portrait 方向
/// @param height 图像高度（像素），对应 portrait 方向
void EPD_display_4bpp(const uint8_t *buf, uint16_t width, uint16_t height) {
    uint32_t erow, ecol;
    EPD_SendCommand(0x10);
    /* 90° CW rotation: portrait(width×height) → landscape(EPD_WIDTH×EPD_HEIGHT)
     * ePaper(erow, ecol) ← source(height-1-ecol, erow)
     * Valid for width==EPD_HEIGHT(480) && height==EPD_WIDTH(800) */
    for (erow = 0; erow < EPD_HEIGHT; erow++) {
        uapi_watchdog_kick();
        for (ecol = 0; ecol < EPD_WIDTH / 2; ecol++) {
            uint32_t lx = ecol * 2;      /* left pixel x on ePaper */
            uint32_t rx = ecol * 2 + 1;  /* right pixel x on ePaper */
            uint8_t left, right;
            /* left pixel: source row = height-1-lx, source col = erow */
            if (lx < height && erow < width) {
                uint32_t sr = height - 1 - lx;
                uint32_t sc = erow;
                uint8_t byte = buf[sr * (width / 2) + sc / 2];
                left = (sc & 1) ? (byte & 0x0F) : (byte >> 4);
            } else {
                left = EPD_WHITE;
            }
            /* right pixel: source row = height-1-rx, source col = erow */
            if (rx < height && erow < width) {
                uint32_t sr = height - 1 - rx;
                uint32_t sc = erow;
                uint8_t byte = buf[sr * (width / 2) + sc / 2];
                right = (sc & 1) ? (byte & 0x0F) : (byte >> 4);
            } else {
                right = EPD_WHITE;
            }
            EPD_SendData((uint8_t)((left << 4) | right));
        }
    }
    EPD_refresh();
}

/// @brief 进入深度睡眠模式（先 POWER_OFF 再 DEEP_SLEEP）
void EPD_sleep(void) {
    EPD_SendCommand(0x02); /* POWER OFF */
    EPD_SendData(0x00);
    EPD_ReadBusy();

    EPD_SendCommand(0x07); /* DEEP SLEEP */
    EPD_SendData(0xA5);
}
