/**
 * @file  image_rle.c
 * @brief 二值图像 RLE 流式解码器实现
 *
 * @details
 *   本文件实现 image_rle.h 中声明的流式 RLE 解码器。
 *   编码格式兼容 compress.py 的 rle_encode() 输出。
 *
 *   解码器状态机:
 *     OPCODE ──(Literal 0xxxxxxx)──→ 输出 7 像素 ──→ OPCODE
 *     OPCODE ──(RLE 1xx0xxxxx)──→ 直接 EMIT
 *     OPCODE ──(RLE 1xx1xxxxx)──→ READ_COUNT ──(末续传)──→ EMIT
 *     EMIT   ──(count 耗尽)──→ OPCODE
 *
 *   像素以 bit 为单位逐个输出, 8 bit 凑成 1 字节写入缓冲区,
 *   缓冲区满或全部解码完成时通过回调送出。
 */

#include "image_rle.h"
#include <string.h>

void rle_init(rle_ctx_t *ctx, uint32_t expected_len,
              rle_on_data_cb callback, void *user_data)
{
    (void)memset(ctx, 0, sizeof(rle_ctx_t));
    ctx->expected_len = expected_len;
    ctx->callback     = callback;
    ctx->user_data    = user_data;
    ctx->state        = RLE_STATE_OPCODE;
}

/**
 * @brief  输出一个像素 bit, 内部逐 bit 拼装为字节
 * @param  ctx  解码器上下文
 * @param  val  像素值 (0 或 1)
 */
static void emit_pixel(rle_ctx_t *ctx, uint8_t val)
{
    if (val) {
        ctx->current_byte |= (1 << (7 - ctx->bit_count));
    }
    ctx->bit_count++;

    if (ctx->bit_count == 8) {
        ctx->buffer[ctx->buf_pos++] = ctx->current_byte;
        ctx->current_byte = 0;
        ctx->bit_count = 0;

        if (ctx->buf_pos == RLE_DECODE_BUF_SIZE) {
            ctx->callback(ctx->buffer, ctx->buf_pos, ctx->user_data);
            ctx->buf_pos = 0;
        }
    }
    ctx->decoded_len++;
}

void rle_decode(rle_ctx_t *ctx, const uint8_t *data, uint32_t len)
{
    uint32_t i = 0;

    while (ctx->decoded_len < ctx->expected_len) {
        /* 优先处理正在输出的 RLE 段 */
        if (ctx->state == RLE_STATE_EMIT) {
            while (ctx->rle_count > 0) {
                emit_pixel(ctx, ctx->rle_pixel_val);
                ctx->rle_count--;
                if (ctx->decoded_len >= ctx->expected_len) break;
            }
            if (ctx->rle_count == 0) {
                ctx->state = RLE_STATE_OPCODE;
            }
            continue;
        }

        /* 需要更多输入数据 */
        if (i >= len) break;
        uint8_t byte = data[i++];

        if (ctx->state == RLE_STATE_OPCODE) {
            if ((byte & 0x80) == 0) {
                /* Literal: 0xxxxxxx → 7 个像素 */
                for (int k = 0; k < 7; k++) {
                    if (ctx->decoded_len >= ctx->expected_len) break;
                    uint8_t pixel = (byte >> (6 - k)) & 1;
                    emit_pixel(ctx, pixel);
                }
            } else {
                /* RLE: 1|val|has_next|count_low5 */
                ctx->rle_pixel_val = (byte >> 6) & 1;
                uint8_t has_next = (byte >> 5) & 1;
                ctx->rle_count = byte & 0x1F;

                if (has_next) {
                    ctx->state = RLE_STATE_READ_COUNT;
                    ctx->rle_shift = 5;
                } else {
                    ctx->rle_count += 8;
                    ctx->state = RLE_STATE_EMIT;
                }
            }
        } else if (ctx->state == RLE_STATE_READ_COUNT) {
            /* Varint 续传: 最高位=续传标记, 低 7 位为数据 */
            uint8_t has_next = (byte & 0x80) >> 7;
            uint8_t part = byte & 0x7F;
            ctx->rle_count |= ((uint32_t)part << ctx->rle_shift);
            ctx->rle_shift += 7;

            if (!has_next) {
                ctx->rle_count += 8;
                ctx->state = RLE_STATE_EMIT;
            }
        }
    }

    /* 解码完成: flush 残余 bits */
    if (ctx->decoded_len >= ctx->expected_len) {
        if (ctx->bit_count > 0) {
            ctx->buffer[ctx->buf_pos++] = ctx->current_byte;
            ctx->bit_count = 0;
            ctx->current_byte = 0;
        }
        if (ctx->buf_pos > 0) {
            ctx->callback(ctx->buffer, ctx->buf_pos, ctx->user_data);
            ctx->buf_pos = 0;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  RLE 编码器 (一次性, 非流式)
 *  与上方解码器格式完全对称, 可用于固件端自测 round-trip。
 * ═══════════════════════════════════════════════════════════════ */

/** 从原始 1bpp 字节数据中读取第 pixel_idx 个像素 (MSB first) */
static inline uint8_t get_pixel(const uint8_t *raw, uint32_t pixel_idx)
{
    return (raw[pixel_idx / 8] >> (7 - (pixel_idx % 8))) & 1;
}

uint32_t rle_encode(const uint8_t *raw, uint32_t raw_len,
                    uint32_t total_pixels,
                    uint8_t *out, uint32_t out_cap)
{
    (void)raw_len;  /* raw_len 可用于边界校验, 此处信任调用方 */
    uint32_t pos = 0;
    uint32_t i = 0;

    while (i < total_pixels) {
        uint8_t val = get_pixel(raw, i);

        /* 统计连续相同像素 */
        uint32_t run = 1;
        while (i + run < total_pixels && get_pixel(raw, i + run) == val) {
            run++;
        }

        if (run >= 8) {
            /* RLE: |1|val|has_next|count_low5| + varint 续传 */
            uint32_t count = run - 8;
            uint8_t low5 = count & 0x1F;
            uint8_t has_more = (count >> 5) > 0 ? 1 : 0;

            if (pos >= out_cap) return 0;
            out[pos++] = 0x80 | (val << 6) | (has_more << 5) | low5;

            if (has_more) {
                uint32_t remaining = count >> 5;
                while (remaining > 0) {
                    if (pos >= out_cap) return 0;
                    uint8_t part = remaining & 0x7F;
                    remaining >>= 7;
                    out[pos++] = (remaining > 0) ? (part | 0x80) : part;
                }
            }
            i += run;
        } else {
            /* Literal: 打包接下来 7 个像素 */
            uint8_t literal = 0;
            for (int k = 0; k < 7; k++) {
                if (i + (uint32_t)k < total_pixels) {
                    literal |= (get_pixel(raw, i + (uint32_t)k) << (6 - k));
                }
            }
            if (pos >= out_cap) return 0;
            out[pos++] = literal;
            i += 7;
        }
    }

    return pos;
}
