/**
 * @file  image_rle.h
 * @brief 二值图像 RLE 压缩/解压算法
 *
 * @details
 *   针对 1bpp 墨水屏图像设计的游程编码 (Run-Length Encoding)。
 *   利用墨水屏图像大面积连续黑/白像素的特点，实现高压缩率。
 *
 *   编码格式:
 *     1) Literal (连续相同像素 < 8 个):
 *        |0|x|x|x|x|x|x|x|
 *        bit7 = 0, bit6~bit0 = 接下来的 7 个像素值
 *
 *     2) RLE (连续相同像素 >= 8 个):
 *        |1|val|has_next|count_low5|  [续传字节...]
 *        bit7 = 1 (RLE 标记)
 *        bit6 = 像素值 (0=黑, 1=白)
 *        bit5 = 是否有后续字节继续编码长度
 *        bit4~0 = (实际长度 - 8) 的低 5 位
 *        后续字节: 类 Protobuf Varint, 每字节 7 位有效数据, 最高位为续传标记
 *
 *   解码器采用流式设计 (rle_ctx_t), 支持分包输入:
 *     - 从 Mesh 网络逐包收到压缩数据时直接喂入, 无需等全部到齐
 *     - 内部维护状态机, 跨包边界时自动续接
 *     - 解码输出通过回调函数分段送出, 适合内存受限的嵌入式场景
 */
#ifndef IMAGE_RLE_H
#define IMAGE_RLE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 流式解码内部缓冲区大小 (字节), 攒满一批再回调 */
#define RLE_DECODE_BUF_SIZE  256

/** 解码输出回调: 每攒满 RLE_DECODE_BUF_SIZE 字节或全部解码完成时触发 */
typedef void (*rle_on_data_cb)(const uint8_t *data, uint32_t len, void *user_data);

/** 解码器状态机枚举 */
typedef enum {
    RLE_STATE_OPCODE,       /**< 等待下一个操作码字节 */
    RLE_STATE_READ_COUNT,   /**< 正在读取多字节 RLE 长度 */
    RLE_STATE_EMIT,         /**< 正在输出 RLE 重复像素 */
} rle_state_t;

/** 流式 RLE 解码器上下文 */
typedef struct {
    uint32_t     expected_len;   /**< 期望的总像素数 (width × height) */
    uint32_t     decoded_len;    /**< 已输出的像素数 */
    rle_on_data_cb callback;     /**< 输出回调 */
    void        *user_data;      /**< 回调用户数据 */

    rle_state_t  state;          /**< 当前状态 */
    uint8_t      rle_pixel_val;  /**< 当前 RLE 段的像素值 (0 或 1) */
    uint32_t     rle_count;      /**< 当前 RLE 段剩余像素数 */
    uint8_t      rle_shift;      /**< Varint 解码的位偏移 */

    uint8_t      current_byte;   /**< 正在组装的输出字节 (逐 bit 拼装) */
    uint8_t      bit_count;      /**< current_byte 中已填的 bit 数 */
    uint8_t      buffer[RLE_DECODE_BUF_SIZE]; /**< 输出缓冲区 */
    uint32_t     buf_pos;        /**< 缓冲区当前写位置 */
} rle_ctx_t;

/**
 * @brief  初始化 RLE 流式解码器
 * @param  ctx           解码器上下文 (调用方分配)
 * @param  expected_len  期望的总像素数 (e.g. 400*300 = 120000)
 * @param  callback      输出回调, 每段完整字节数据通过此回调送出
 * @param  user_data     回调的用户数据指针
 */
void rle_init(rle_ctx_t *ctx, uint32_t expected_len,
              rle_on_data_cb callback, void *user_data);

/**
 * @brief  向解码器喂入一段压缩数据
 * @param  ctx   解码器上下文
 * @param  data  压缩数据指针
 * @param  len   压缩数据长度 (字节)
 * @note   可多次调用, 每次喂一个 Mesh 分包的数据。
 *         解码完成后 (decoded_len == expected_len) 会自动 flush 剩余 bits。
 */
void rle_decode(rle_ctx_t *ctx, const uint8_t *data, uint32_t len);

/**
 * @brief  查询解码是否已完成
 * @param  ctx  解码器上下文
 * @return 1=已完成, 0=未完成
 */
static inline int rle_is_done(const rle_ctx_t *ctx)
{
    return ctx->decoded_len >= ctx->expected_len;
}

/**
 * @brief  RLE 编码 (一次性, 非流式)
 * @param  raw          原始 1bpp 像素数据
 * @param  raw_len      raw 字节长度
 * @param  total_pixels 总像素数 (width × height)
 * @param  out          输出缓冲区
 * @param  out_cap      输出缓冲区容量
 * @return 压缩后字节数; 0 表示失败 (输出缓冲区不足)
 */
uint32_t rle_encode(const uint8_t *raw, uint32_t raw_len,
                    uint32_t total_pixels,
                    uint8_t *out, uint32_t out_cap);

#ifdef __cplusplus
}
#endif

#endif /* IMAGE_RLE_H */
