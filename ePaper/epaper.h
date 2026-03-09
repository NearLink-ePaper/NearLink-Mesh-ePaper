#ifndef EPD_H
#define EPD_H

#include <stddef.h>
#include <stdint.h>

#define EPD_WIDTH  800
#define EPD_HEIGHT 480

/* 7.3inch e-Paper (E) 4-bit pixel color index */
#define EPD_BLACK  0x0
#define EPD_WHITE  0x1
#define EPD_YELLOW 0x2
#define EPD_RED    0x3
/* 0x4 reserved */
#define EPD_BLUE   0x5
#define EPD_GREEN  0x6

/* Pattern codes for EPD_display_NUM (solid color fill) */
#define EPD_Source_Line            EPD_WHITE
#define EPD_Gate_Line              EPD_BLACK
#define EPD_UP_BLACK_DOWN_WHITE    EPD_BLACK
#define EPD_LEFT_BLACK_RIGHT_WHITE EPD_BLACK
#define EPD_Frame                  EPD_WHITE
#define EPD_Crosstalk              EPD_WHITE
#define EPD_Chessboard             EPD_WHITE
#define EPD_Image                  EPD_WHITE

extern unsigned char EPD_Flag;

void EPD_Reset(void);
void EPD_SendCommand(uint8_t Reg);
void EPD_SendData(uint8_t Data);
void EPD_refresh(void);
void EPD_lut_GC(void);
void EPD_lut_DU(void);
void EPD_Init(void);
void EPD_display(uint8_t *picData);
void EPD_display_NUM(uint8_t color);
void EPD_display_part(const uint8_t *picData, uint16_t xstart, uint16_t ystart,
                      uint16_t image_width, uint16_t image_height);
void EPD_Show7Block(void);
void EPD_Clear(void);
void EPD_sleep(void);
void EPD_display_1bpp(const uint8_t *buf, uint16_t width, uint16_t height);

void epaper_trigger_mesh_image(const uint8_t *buf, uint16_t width, uint16_t height);

#endif
