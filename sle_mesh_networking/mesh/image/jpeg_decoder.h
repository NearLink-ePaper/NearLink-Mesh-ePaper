/**
 * @file  jpeg_decoder.h
 * @brief JPEG 流式解码 + Floyd-Steinberg 6色抖动 → 直接输出到 ePaper SPI
 *
 * @details
 *   PC 端将图像预旋转为横屏 (landscape, 800×480) 后 JPEG 压缩，
 *   设备端逐 MCU 行流式解码+抖动，直接通过 EPD_SendData() 发送到
 *   ePaper 控制器 SPI，无需 192KB 中间缓冲区。
 *
 *   工作内存 (~52KB for 800px wide)，由调用方提供 scratch 缓冲区，零堆分配:
 *     - TJpgDec 工作池: 4KB
 *     - MCU 行条带 RGB: width × 16 × 3 ≈ 38KB
 *     - 误差缓冲: 2 × width × 3 × 2 ≈ 10KB
 *
 * @def JPEG_SCRATCH_SIZE  调用方需提供的 scratch 最小字节数 (52096)
 */
#ifndef JPEG_DECODER_H
#define JPEG_DECODER_H

#define JPEG_SCRATCH_SIZE  52096U  /* 4096 + 800*16*3 + 2*(800*3*2) */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  流式 JPEG 解码 + 抖动 → 直接写入 ePaper SPI
 *
 * @param  jpeg_data     JPEG 压缩数据指针 (landscape 图像)
 * @param  jpeg_size     JPEG 数据大小（字节）
 * @param  scratch       调用方提供的工作缓冲区（≥ JPEG_SCRATCH_SIZE 字节）
 * @param  scratch_size  scratch 缓冲区大小（字节）
 * @return true=成功, false=解码失败或缓冲区不足
 *
 * @note   调用前必须 EPD_Init()；调用后需 EPD_refresh() + EPD_sleep()。
 *         scratch 可直接使用接收缓冲区末尾（jpeg_data + jpeg_size），零堆分配。
 */
bool jpeg_decode_stream_epd(const uint8_t *jpeg_data, uint32_t jpeg_size,
                             uint8_t *scratch, uint32_t scratch_size);

#ifdef __cplusplus
}
#endif

#endif /* JPEG_DECODER_H */
