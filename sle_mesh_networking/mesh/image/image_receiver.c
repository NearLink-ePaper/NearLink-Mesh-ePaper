/**
 * @file  image_receiver.c
 * @brief SLE Mesh 图传接收端 —— 图片接收状态机实现
 *
 * @details
 *   本文件实现了 Receiver 节点上的完整图片接收逻辑，核心工作流：
 *
 *   1. 【START】网关发送 IMG_CMD_START，携带图片尺寸/包数/模式/传输方式，
 *      接收端据此初始化缓冲区和位图，进入 RECEIVING 状态；
 *   2. 【DATA】逐包写入 s_img_buf[]，使用 rx_bitmap 进行去重与到达跟踪，
 *      ACK 模式下每包回复确认；
 *   3. 【CHECKPOINT】(v2) 网关分段发包后发送检查点，接收端回复当前进度，
 *      同时重置无活动超时——避免大图传输被误判超时；
 *   4. 【END】网关发送 CRC16 校验值，接收端先做缺包位图检查：
 *      - 若有缺包 → 发送 MISSING 位图，进入补包等待；
 *      - 若全部收齐 → CRC16 校验 → 成功则通知上层刷屏；
 *   5. 【RESULT 重发】为确保网关收到最终结果，RESULT 会在 tick() 中
 *      定时重发若干次。
 *
 *   v2 改动：
 *     - 新增 CHECKPOINT (0x08) → 回复 CHKPT_ACK (0x88)
 *     - MISSING 改为 30 字节位图格式，一次覆盖 240 个 seq
 *     - CHECKPOINT / DATA 均重置无活动超时计时器
 */

#include "osal_addr.h"
#include "osal_debug.h"
#include "securec.h"
#include "errcode.h"
#include "image_receiver.h"
#include "image_rle.h"
#include "jpeg_decoder.h"
#include "mesh_config.h"
#include "soc_osal.h"

/** 日志标签前缀，所有 osal_printk 输出均以此开头便于过滤 */
#define IMG_LOG  "[img rx]"

/* === RESULT 重发参数 =====================================================
 *  网关可能因 mesh 丢包未收到最终 RESULT，因此在 DONE/ERROR 状态下
 *  周期性重发若干次以提高可靠性。
 * ====================================================================== */
/** F28: RESULT 最大重发次数。3次已足够 (F27 保证网关只转发首条给手机)
 *  额外重试仅用于覆盖偶发丢包, 不需过多 */
#define IMG_RESULT_RETRY_COUNT  3
/** F28: RESULT 重发间隔（毫秒）。1s 间隔 × 3次 = 3s 覆盖窗口
 *  配合网关侧 FC_WAIT_RESULT 超时 (5s), 足以完成交互 */
#define IMG_RESULT_RETRY_MS     1000

/* === 位图辅助宏 ==========================================================
 *  使用 1-bit-per-seq 的紧凑位图跟踪每个分包是否到达。
 *  seq 号右移 3 位定位字节索引，低 3 位定位位偏移（小端序）。
 * ====================================================================== */
/** 将位图 bm 中第 seq 位置 1（标记已收到） */
#define BITMAP_SET(bm, seq)   ((bm)[(seq) >> 3] |= (1 << ((seq) & 7)))
/** 读取位图 bm 中第 seq 位，返回 0 或 1 */
#define BITMAP_GET(bm, seq)   (((bm)[(seq) >> 3] >> ((seq) & 7)) & 1)
/** 清零位图 bm 的前 n 个字节 */
#define BITMAP_CLR_ALL(bm, n) (void)memset_s((bm), (n), 0, (n))

/* === 静态缓冲区 ==========================================================
 *  s_img_buf: 图片像素数据缓冲区，按 seq × IMG_PKT_PAYLOAD 偏移写入；
 *  s_info   : 唯一的接收会话上下文（单例），保存所有运行时状态。
 *  两者均为全局 static，避免堆分配，生命周期与模块相同。
 * ====================================================================== */
static uint8_t s_img_buf[IMG_RX_BUF_SIZE];  /**< 图片数据缓冲区 */
static img_rx_info_t s_info;                /**< 接收会话上下文 */

/* === 组播目标列表 =========================================================
 *  当接收到组播 START (0x0A) 时，目标列表记录在此处。
 *  节点先检查自己的地址是否在列表中，若是则接受传输，否则忽略。
 * ====================================================================== */
static uint16_t s_mcast_targets[MESH_MCAST_MAX_TARGETS]; /**< 组播目标地址列表 */
static uint8_t  s_mcast_count = 0;           /**< 当前组播目标数量，0=非组播/单播 */

/* === 外部依赖 ============================================================
 *  mesh_send() 由 mesh 组网模块提供，用于向指定 mesh 地址发送原始数据。
 *  此处使用 extern 前向声明而非头文件引入，因为 mesh 模块头文件可能
 *  存在循环依赖。
 * ====================================================================== */
extern errcode_t mesh_send(uint16_t dst_addr, const uint8_t *data, uint16_t len);

/* === CRC16-CCITT 校验 ====================================================
 *  用于验证完整图片数据的正确性。
 *  多项式: 0x1021，初始值: 0x0000，无反转，与网关端算法一致。
 * ====================================================================== */

/**
 * @brief  计算 CRC16-CCITT 校验值
 * @param  data  待校验数据指针
 * @param  len   数据长度（字节）
 * @return 16 位 CRC 校验值
 * @note   采用逐位移位法，适合嵌入式场景（无需查表，节省 ROM）。
 *         若性能敏感可改为 256 项查表法，约快 8 倍。
 */
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0x0000;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* === RLE 解压辅助 ========================================================
 *  CRC 校验通过后，若 mode == IMG_MODE_RLE，将 s_img_buf 中的 RLE
 *  压缩数据解压为原始 1bpp 像素。
 *  步骤：
 *    1. osal_vmalloc 申请临时缓冲区，拷贝压缩数据；
 *    2. 使用流式 RLE 解码器解压，回调函数依次写入 s_img_buf；
 *    3. 释放临时缓冲区，更新 total_bytes 为解压后大小。
 * ====================================================================== */

/** RLE 解码回调写入上下文 */
typedef struct {
    uint32_t write_pos;   /**< 当前写入偏移 */
} rle_write_ctx_t;

/**
 * @brief  RLE 解码输出回调 —— 将解码字节写入 s_img_buf
 */
static void rle_write_to_buf(const uint8_t *data, uint32_t len, void *user_data)
{
    rle_write_ctx_t *wctx = (rle_write_ctx_t *)user_data;
    if (wctx->write_pos + len <= IMG_RX_BUF_SIZE) {
        (void)memcpy_s(&s_img_buf[wctx->write_pos],
                       IMG_RX_BUF_SIZE - wctx->write_pos, data, len);
        wctx->write_pos += len;
    }
}

/**
 * @brief  对 s_img_buf 中的 RLE 压缩数据进行解压
 * @return true=解压成功，false=OOM 或解码异常
 */
static bool rle_decompress_image(void)
{
    uint16_t compressed_size = s_info.total_bytes;
    uint32_t total_pixels = (uint32_t)s_info.width * s_info.height;
    uint32_t decompressed_size = (total_pixels + 7) / 8;

    if (decompressed_size > IMG_RX_BUF_SIZE) {
        osal_printk("%s RLE: decompressed %lu > buf %d, OOM\r\n",
                    IMG_LOG, decompressed_size, IMG_RX_BUF_SIZE);
        return false;
    }

    /* 申请临时缓冲区保存压缩数据 */
    uint8_t *tmp = (uint8_t *)osal_vmalloc(compressed_size);
    if (tmp == NULL) {
        osal_printk("%s RLE: vmalloc %d failed\r\n", IMG_LOG, compressed_size);
        return false;
    }

    (void)memcpy_s(tmp, compressed_size, s_img_buf, compressed_size);
    (void)memset_s(s_img_buf, IMG_RX_BUF_SIZE, 0, decompressed_size);

    /* 流式解码 */
    rle_ctx_t rle;
    rle_write_ctx_t wctx = { .write_pos = 0 };
    rle_init(&rle, total_pixels, rle_write_to_buf, &wctx);
    rle_decode(&rle, tmp, compressed_size);

    osal_vfree(tmp);

    /* 日志：压缩率 (用整数除法避免浮点) */
    uint32_t ratio_pct = (decompressed_size > 0)
        ? ((uint32_t)compressed_size * 100) / decompressed_size : 0;
    osal_printk("%s RLE decode: %dB → %luB (%d%%)\r\n",
                IMG_LOG, compressed_size, wctx.write_pos, ratio_pct);

    /* 更新 total_bytes 为解压后大小，供上层显示代码使用 */
    s_info.total_bytes = (uint16_t)decompressed_size;
    return true;
}

/* === 发送回复 ============================================================
 *  以下函数封装各类上行响应报文的组帧与发送。
 *  报文格式参见 image_receiver.h 中各 IMG_CMD_* 的说明。
 * ====================================================================== */

/**
 * @brief  发送单包 ACK 响应
 * @param  gw_addr  网关 mesh 地址
 * @param  status   ACK 状态码：IMG_ACK_OK / IMG_ACK_RESEND / IMG_ACK_DONE
 * @param  seq      被确认的分包序号
 * @note   报文格式: [0x85, status, seq_hi, seq_lo]，共 4 字节
 */
static void send_ack(uint16_t gw_addr, uint8_t status, uint16_t seq)
{
    uint8_t resp[4];
    resp[0] = IMG_CMD_ACK;
    resp[1] = status;
    resp[2] = (seq >> 8) & 0xFF;
    resp[3] = seq & 0xFF;
    mesh_send(gw_addr, resp, 4);
}

/**
 * @brief  发送最终结果 RESULT 报文
 * @param  gw_addr  网关 mesh 地址
 * @param  status   结果状态码：IMG_RESULT_OK / OOM / TIMEOUT / CANCEL / CRC_ERR
 * @note   处理流程：
 *         1. 组帧 [0x86, status] 并立即发送一次；
 *         2. 将 status 和重发参数保存到 s_info，后续 tick() 中继续重发。
 *         为什么要重发？因为 mesh 网络可能丢包，单次发送不保证网关收到。
 */
static void send_result(uint16_t gw_addr, uint8_t status)
{
    uint8_t resp[2];
    resp[0] = IMG_CMD_RESULT;
    resp[1] = status;
    mesh_send(gw_addr, resp, 2);
    osal_printk("%s RESULT status=%d → gw=0x%04X\r\n", IMG_LOG, status, gw_addr);

    s_info.result_status    = status;
    s_info.result_retries   = IMG_RESULT_RETRY_COUNT;
    s_info.result_next_tick = osal_get_tick_ms() + IMG_RESULT_RETRY_MS;
}

/**
 * @brief  发送 CHECKPOINT 应答报文（v2 新增）
 * @param  gw_addr  网关 mesh 地址
 * @param  seg_id   检查点段号，由网关指定
 * @note   报文格式: [0x88, seg_id, rx_count_hi, rx_count_lo]，共 4 字节。
 *         网关收到后可判断是否需要补发该段的缺包。
 */
static void send_checkpoint_ack(uint16_t gw_addr, uint8_t seg_id)
{
    uint8_t resp[4];
    resp[0] = IMG_CMD_CHKPT_ACK;
    resp[1] = seg_id;
    resp[2] = (s_info.rx_count >> 8) & 0xFF;
    resp[3] = s_info.rx_count & 0xFF;
    mesh_send(gw_addr, resp, 4);
    osal_printk("%s CHKPT_ACK: seg=%d rx=%d/%d → gw=0x%04X\r\n",
                IMG_LOG, seg_id, s_info.rx_count, s_info.pkt_count, gw_addr);
}

/**
 * @brief  发送缺包位图 MISSING 报文（v2 新增）
 * @param  gw_addr  网关 mesh 地址
 * @return 缺包总数，0 表示全部收齐
 *
 * @note   处理流程：
 *         1. 遍历 rx_bitmap，找出所有 bit=0（未收到）的 seq；
 *         2. 在 miss_bitmap 中将对应 bit 置 1（表示"缺失"）；
 *         3. 组帧 [0x87, total_missing_hi, total_missing_lo, bitmap[30]]；
 *         4. 30 字节位图覆盖 240 个 seq，彻底解决旧版逐 seq 上报"报不完"问题。
 *
 *         为什么 rx_bitmap 中 bit=1 表示"已收到"而 miss_bitmap 中 bit=1 表示"缺失"？
 *         → 两个位图语义相反，是为了让网关直接用 miss_bitmap 确定重传列表，
 *           无需再做取反运算。
 */
static uint16_t send_missing_bitmap(uint16_t gw_addr)
{
    uint8_t miss_bitmap[IMG_BITMAP_BYTES];
    BITMAP_CLR_ALL(miss_bitmap, IMG_BITMAP_BYTES);
    uint16_t total_missing = 0;

    for (uint16_t seq = 0; seq < s_info.pkt_count; seq++) {
        if (!BITMAP_GET(s_info.rx_bitmap, seq)) {
            /* 在 miss_bitmap 中标记为缺失 (bit=1) */
            miss_bitmap[seq >> 3] |= (1 << (seq & 7));
            total_missing++;
        }
    }

    if (total_missing == 0) return 0;

    /* 组帧: [0x87, TOTAL_HI, TOTAL_LO, BITMAP[30]] = 33 字节
     * 为什么固定 30 字节位图而不根据 pkt_count 裁剪？
     * → 保持报文定长便于网关解析，多余 bit 为 0 不影响语义 */
    uint8_t msg[3 + IMG_BITMAP_BYTES];
    msg[0] = IMG_CMD_MISSING;
    msg[1] = (total_missing >> 8) & 0xFF;
    msg[2] = total_missing & 0xFF;
    (void)memcpy_s(&msg[3], IMG_BITMAP_BYTES, miss_bitmap, IMG_BITMAP_BYTES);

    mesh_send(gw_addr, msg, 3 + IMG_BITMAP_BYTES);
    osal_printk("%s MISSING bitmap: %d missing → gw=0x%04X (round %d)\r\n",
                IMG_LOG, total_missing, gw_addr, s_info.retry_round);
    return total_missing;
}

/* === 命令处理函数 ========================================================
 *  每个 handle_xxx() 对应一种下行命令字的处理逻辑。
 *  由 image_receiver_on_data() 根据首字节分发调用。
 * ====================================================================== */

/**
 * @brief  处理 IMG_CMD_START (0x04) —— 开始传输命令
 * @param  src_addr  发送方 mesh 地址
 * @param  data      完整报文（含 cmd 字节）
 * @param  len       报文长度
 *
 * @note   处理流程：
 *         1. 校验报文最小长度（10 字节）；
 *         2. 解析图片元信息：total_bytes / pkt_count / width / height / mode / xfer_mode；
 *         3. 检查 total_bytes 是否超出缓冲区上限，超出则回复 OOM；
 *         4. 清零接收上下文和缓冲区，初始化位图，进入 RECEIVING 状态；
 *         5. 若此前正在接收另一张图，会被新 START 覆盖（设计决策：不排队）。
 */
static void handle_start(uint16_t src_addr, const uint8_t *data, uint16_t len)
{
    if (len < 10) {
        osal_printk("%s START: too short %d\r\n", IMG_LOG, len);
        return;
    }

    if (s_info.state == IMG_STATE_RECEIVING) {
        osal_printk("%s START: cancel prev transfer\r\n", IMG_LOG);
    }

    uint16_t total  = ((uint16_t)data[1] << 8) | data[2];
    uint16_t pkt    = ((uint16_t)data[3] << 8) | data[4];
    uint16_t width  = ((uint16_t)data[5] << 8) | data[6];
    uint16_t height = ((uint16_t)data[7] << 8) | data[8];
    uint8_t  mode   = data[9];
    uint8_t  xfer   = (len >= 11) ? data[10] : IMG_XFER_FAST;

    osal_printk("%s START: %dx%d total=%dB pkt=%d mode=%d xfer=%s from gw=0x%04X\r\n",
                IMG_LOG, width, height, total, pkt, mode,
                (xfer == IMG_XFER_ACK) ? "ACK" : "FAST", src_addr);

    if (total > IMG_RX_BUF_SIZE) {
        osal_printk("%s START: total %d > buf %d, OOM\r\n", IMG_LOG, total, IMG_RX_BUF_SIZE);
        send_result(src_addr, IMG_RESULT_OOM);
        return;
    }

    (void)memset_s(&s_info, sizeof(s_info), 0, sizeof(s_info));
    (void)memset_s(s_img_buf, sizeof(s_img_buf), 0, total);
    BITMAP_CLR_ALL(s_info.rx_bitmap, IMG_BITMAP_BYTES);

    s_info.state       = IMG_STATE_RECEIVING;
    s_info.width       = width;
    s_info.height      = height;
    s_info.mode        = mode;
    s_info.xfer_mode   = xfer;
    s_info.total_bytes = total;
    s_info.pkt_count   = pkt;
    s_info.rx_count    = 0;
    s_info.expect_seq  = 0;
    s_info.gw_addr     = src_addr;
    s_info.start_tick  = osal_get_tick_ms();
    s_info.retry_round = 0;
    s_info.last_missing_cnt = pkt;
}

/**
 * @brief  处理 IMG_CMD_DATA (0x05) —— 数据分包
 * @param  src_addr  发送方 mesh 地址
 * @param  data      完整报文（含 cmd 字节）
 * @param  len       报文长度
 *
 * @note   处理流程：
 *         1. 解析 5 字节头部: [cmd, seq_hi, seq_lo, len_hi, len_lo]；
 *         2. 校验边界：payload 不越界、offset + data_len ≤ total_bytes；
 *         3. memcpy 写入 s_img_buf[seq * IMG_PKT_PAYLOAD] 处；
 *         4. 位图去重：仅首次收到时 rx_count++；
 *         5. 重置无活动超时计时器（防止大图传输中途误判超时）；
 *         6. ACK 模式下回复 ACK（全部收齐则回复 DONE）。
 */
static void handle_data(uint16_t src_addr, const uint8_t *data, uint16_t len)
{
    if (s_info.state != IMG_STATE_RECEIVING) return;
    if (len < 5) return;  /* O4 优化：5 字节头 (cmd + seq×2 + len×2)，比 v1 的 3 字节头多了 16-bit 长度字段 */

    uint16_t seq      = ((uint16_t)data[1] << 8) | data[2];
    uint16_t data_len = ((uint16_t)data[3] << 8) | data[4];  /* O4 优化：16-bit 长度，支持最大 65535 字节载荷 */
    const uint8_t *payload = &data[5];

    if ((uint16_t)(5 + data_len) > len) return;

    uint32_t offset = (uint32_t)seq * IMG_PKT_PAYLOAD;
    if (offset + data_len > s_info.total_bytes) return;

    (void)memcpy_s(&s_img_buf[offset], IMG_RX_BUF_SIZE - offset, payload, data_len);

    if (!BITMAP_GET(s_info.rx_bitmap, seq)) {
        /* 首次收到该 seq：标记位图并递增计数。
         * 为什么要做去重？因为补包阶段网关可能重发已收到的包 */
        BITMAP_SET(s_info.rx_bitmap, seq);
        s_info.rx_count++;
    }
    s_info.expect_seq = seq + 1;

    /* 重置无活动超时——每收到一个 DATA 包都延长计时，
     * 避免慢速 mesh 网络下大图传输被误判为超时 */
    s_info.start_tick = osal_get_tick_ms();

    /* 每 10 包打一次进度 */
    if (s_info.rx_count % 10 == 0 || s_info.rx_count == s_info.pkt_count) {
        osal_printk("%s DATA: seq=%d rx=%d/%d\r\n",
                    IMG_LOG, seq, s_info.rx_count, s_info.pkt_count);
    }

    /* ACK 模式 */
    if (s_info.xfer_mode == IMG_XFER_ACK) {
        if (s_info.rx_count >= s_info.pkt_count) {
            send_ack(s_info.gw_addr, IMG_ACK_DONE, seq);
        } else {
            send_ack(s_info.gw_addr, IMG_ACK_OK, seq);
        }
    }

    (void)src_addr;
}

/**
 * @brief  处理 IMG_CMD_CHECKPOINT (0x08) —— 网关分段检查点（v2 新增）
 * @param  src_addr  发送方 mesh 地址
 * @param  data      完整报文: [0x08, seg_id]
 * @param  len       报文长度（至少 2 字节）
 *
 * @note   处理流程：
 *         1. 读取 seg_id（网关分段编号）；
 *         2. 回复 CHKPT_ACK 携带当前 rx_count；
 *         3. 重置无活动超时（CHECKPOINT 本身即是一种心跳）。
 *         为什么需要 CHECKPOINT？大图 100 包传输耗时长，
 *         中途没有 DATA 到达时可能触发超时，CHECKPOINT 作为心跳解决此问题。
 */
static void handle_checkpoint(uint16_t src_addr, const uint8_t *data, uint16_t len)
{
    if (s_info.state != IMG_STATE_RECEIVING) {
        /* P21: 回复特殊 CHKPT_ACK (rx=0xFFFF) 通知网关本节点未在接收状态，
         * 可能是 START 丢包。网关收到后会立即重发 START，而非被动等 CHKPT 超时 */
        osal_printk("%s CHKPT: not receiving, reply NOT_RX to 0x%04X\r\n", IMG_LOG, src_addr);
        if (len >= 2) {
            uint8_t resp[4];
            resp[0] = IMG_CMD_CHKPT_ACK;
            resp[1] = data[1];  /* seg_id echo */
            resp[2] = 0xFF;
            resp[3] = 0xFF;     /* 0xFFFF = 未在接收状态 */
            mesh_send(src_addr, resp, 4);
        }
        return;
    }
    if (len < 2) return;

    uint8_t seg_id = data[1];

    /* 回复 CHECKPOINT_ACK，让网关知道当前进度 */
    send_checkpoint_ack(s_info.gw_addr, seg_id);

    /* 重置无活动超时——CHECKPOINT 本身充当心跳信号 */
    s_info.start_tick = osal_get_tick_ms();

    (void)src_addr;
}

/**
 * @brief  处理 IMG_CMD_END (0x06) —— 传输结束命令
 * @param  src_addr  发送方 mesh 地址
 * @param  data      完整报文: [0x06, crc_hi, crc_lo]
 * @param  len       报文长度（至少 3 字节）
 *
 * @note   处理流程：
 *         1. 位图检查缺包 → 若有缺包则发送 MISSING 位图并等待补包；
 *         2. 补包轮次 +1，若超过 IMG_MAX_RETRY_ROUNDS 则放弃并报错；
 *         3. 若全部收齐 → 计算 CRC16 并与 expected_crc 比对；
 *         4. CRC 匹配 → 状态迁移至 DONE，上层可读取缓冲区刷屏；
 *         5. CRC 不匹配 → 状态迁移至 ERROR，上报 CRC_ERR。
 */
static void handle_end(uint16_t src_addr, const uint8_t *data, uint16_t len)
{
    if (s_info.state != IMG_STATE_RECEIVING) {
        osal_printk("%s END: not receiving, ignore\r\n", IMG_LOG);
        return;
    }
    if (len < 3) return;

    uint16_t expected_crc = ((uint16_t)data[1] << 8) | data[2];

    osal_printk("%s END: rx=%d/%d round=%d\r\n",
                IMG_LOG, s_info.rx_count, s_info.pkt_count, s_info.retry_round);

    /* ── 位图检查缺包 ──
     * 先上报缺包位图，网关据此重传后会再次发送 END */
    uint16_t missing = send_missing_bitmap(s_info.gw_addr);

    if (missing > 0) {
        s_info.retry_round++;
        s_info.last_missing_cnt = missing;
        s_info.start_tick = osal_get_tick_ms();

        osal_printk("%s END: %d missing, wait retry (round %d/%d)\r\n",
                    IMG_LOG, missing, s_info.retry_round, IMG_MAX_RETRY_ROUNDS);

        if (s_info.retry_round > IMG_MAX_RETRY_ROUNDS) {
            osal_printk("%s END: max retries exceeded, giving up\r\n", IMG_LOG);
            s_info.state = IMG_STATE_ERROR;
            send_result(s_info.gw_addr, IMG_RESULT_CRC_ERR);
        }
        return;
    }

    /* ── 全部收齐，做 CRC 校验 ──
     * CRC 覆盖整个 s_img_buf[0..total_bytes-1]，与网关端算法一致 */
    uint16_t actual_crc = crc16_ccitt(s_img_buf, s_info.total_bytes);

    osal_printk("%s END: all received! crc expect=0x%04X actual=0x%04X\r\n",
                IMG_LOG, expected_crc, actual_crc);

    if (actual_crc != expected_crc) {
        osal_printk("%s END: CRC MISMATCH!\r\n", IMG_LOG);
        s_info.state = IMG_STATE_ERROR;
        send_result(s_info.gw_addr, IMG_RESULT_CRC_ERR);
        return;
    }

    /* 先发送 RESULT 告知网关 CRC 通过 (不设 DONE, 等后处理完成) */
    send_result(s_info.gw_addr, IMG_RESULT_OK);

    /* ── 后处理: 根据 mode 做解压/解码 ── */
    if (s_info.mode == IMG_MODE_RLE) {
        /* RLE 解压 */
        if (!rle_decompress_image()) {
            osal_printk("%s END: RLE decompress failed!\r\n", IMG_LOG);
            s_info.state = IMG_STATE_ERROR;
            return;
        }
    } else if (s_info.mode == IMG_MODE_JPEG) {
        /* JPEG 数据留在 s_img_buf, ePaper 任务流式解码+显示 */
        osal_printk("%s JPEG: %d bytes, defer to ePaper task\r\n",
                    IMG_LOG, s_info.total_bytes);
    }

    /* 后处理完成，缓冲区数据就绪，现在才标记 DONE */
    s_info.state = IMG_STATE_DONE;

    osal_printk("%s DONE: %dx%d %dB rounds=%d → buffer ready\r\n",
                IMG_LOG, s_info.width, s_info.height, s_info.total_bytes,
                s_info.retry_round);

    (void)src_addr;
}

/**
 * @brief  处理 IMG_CMD_CANCEL (0x07) —— 取消传输命令
 * @param  src_addr  发送方 mesh 地址
 * @param  data      完整报文（仅 cmd 字节有效）
 * @param  len       报文长度
 *
 * @note   无论当前处于何种状态均接受取消，正在接收时额外发送 CANCEL 结果通知。
 */
static void handle_cancel(uint16_t src_addr, const uint8_t *data, uint16_t len)
{
    osal_printk("%s CANCEL from 0x%04X (state=%d)\r\n", IMG_LOG, src_addr, s_info.state);

    if (s_info.state == IMG_STATE_RECEIVING) {
        send_result(s_info.gw_addr, IMG_RESULT_CANCEL);
    }

    s_info.state = IMG_STATE_IDLE;
    s_mcast_count = 0;  /* 取消时清除组播状态 */
    (void)data;
    (void)len;
}

/**
 * @brief  处理 IMG_CMD_MCAST_START (0x0A) —— 组播开始传输命令（v3 新增）
 * @param  src_addr  发送方 mesh 地址（网关）
 * @param  data      完整报文：[0x0A, N, ADDR1_HI, ADDR1_LO, ..., TOTAL(2), PKT(2), W(2), H(2), MODE, XFER]
 * @param  len       报文长度
 *
 * @note   处理流程：
 *         1. 解析 N（目标节点数量）和目标地址列表；
 *         2. 检查本节点是否在目标列表中，不在则忽略本次传输；
 *         3. 若在列表中，按照与普通 START 相同的流程初始化接收缓冲区；
 *         4. 将目标列表保存到 s_mcast_targets 供后续使用。
 *
 *         帧格式（Mesh 负载）：
 *         [0x0A] [N(1)] [ADDR1_HI ADDR1_LO] ... [ADDRn_HI ADDRn_LO]
 *         [TOTAL_HI TOTAL_LO] [PKT_HI PKT_LO] [W_HI W_LO] [H_HI H_LO] [MODE] [XFER]
 */
static void handle_mcast_start(uint16_t src_addr, const uint8_t *data, uint16_t len)
{
    if (len < 3) {
        osal_printk("%s MCAST_START: too short %d\r\n", IMG_LOG, len);
        return;
    }

    uint8_t n_targets = data[1];
    if (n_targets == 0 || n_targets > MESH_MCAST_MAX_TARGETS) {
        osal_printk("%s MCAST_START: invalid target count %d\r\n", IMG_LOG, n_targets);
        return;
    }

    /* 最小长度: 1(cmd) + 1(N) + 2*N(addrs) + 9(TOTAL+PKT+W+H+MODE) = 11 + 2*N */
    uint16_t min_len = 2 + 2 * (uint16_t)n_targets + 9;
    if (len < min_len) {
        osal_printk("%s MCAST_START: too short %d (need %d for %d targets)\r\n",
                    IMG_LOG, len, min_len, n_targets);
        return;
    }

    /* 解析目标地址列表，检查本节点是否在列表中 */
    uint16_t my_addr = g_mesh_node_addr;
    bool is_target = false;
    s_mcast_count = n_targets;

    for (uint8_t i = 0; i < n_targets; i++) {
        uint16_t addr = ((uint16_t)data[2 + 2 * i] << 8) | data[3 + 2 * i];
        s_mcast_targets[i] = addr;
        if (addr == my_addr) {
            is_target = true;
        }
    }

    osal_printk("%s MCAST_START: %d targets, my_addr=0x%04X, is_target=%d\r\n",
                IMG_LOG, n_targets, my_addr, is_target);

    if (!is_target) {
        osal_printk("%s MCAST_START: not a target, ignoring\r\n", IMG_LOG);
        s_mcast_count = 0;
        return;
    }

    /* 本节点是组播目标 → 解析图片元信息（偏移 = 2 + 2*N） */
    uint16_t off = 2 + 2 * (uint16_t)n_targets;
    uint16_t total  = ((uint16_t)data[off]     << 8) | data[off + 1];
    uint16_t pkt    = ((uint16_t)data[off + 2] << 8) | data[off + 3];
    uint16_t width  = ((uint16_t)data[off + 4] << 8) | data[off + 5];
    uint16_t height = ((uint16_t)data[off + 6] << 8) | data[off + 7];
    uint8_t  mode   = data[off + 8];
    uint8_t  xfer   = (len > off + 9) ? data[off + 9] : IMG_XFER_FAST;

    osal_printk("%s MCAST accepted: %dx%d total=%dB pkt=%d mode=%d xfer=%s from gw=0x%04X\r\n",
                IMG_LOG, width, height, total, pkt, mode,
                (xfer == IMG_XFER_ACK) ? "ACK" : "FAST", src_addr);

    if (total > IMG_RX_BUF_SIZE) {
        osal_printk("%s MCAST_START: total %d > buf %d, OOM\r\n", IMG_LOG, total, IMG_RX_BUF_SIZE);
        send_result(src_addr, IMG_RESULT_OOM);
        return;
    }

    /* F26: 防止重复 MCAST_START 广播清空已接收数据
     * 组播 START 通过 mesh_broadcast 发送，可能经多条路径到达同一节点，
     * 导致同一 START 被 handle 两次。若已在接收同一传输，则跳过重初始化。
     * 注: F23 重试时节点通常已进入 DONE/ERROR 状态，不会命中此检查。 */
    if (s_info.state == IMG_STATE_RECEIVING &&
        s_info.gw_addr == src_addr &&
        s_info.total_bytes == total) {
        osal_printk("%s MCAST_START: dup (already receiving from 0x%04X), ignored\r\n",
                    IMG_LOG, src_addr);
        return;
    }

    /* 初始化接收上下文（与普通 START 完全相同） */
    (void)memset_s(&s_info, sizeof(s_info), 0, sizeof(s_info));
    (void)memset_s(s_img_buf, sizeof(s_img_buf), 0, total);
    BITMAP_CLR_ALL(s_info.rx_bitmap, IMG_BITMAP_BYTES);

    s_info.state       = IMG_STATE_RECEIVING;
    s_info.width       = width;
    s_info.height      = height;
    s_info.mode        = mode;
    s_info.xfer_mode   = xfer;
    s_info.total_bytes = total;
    s_info.pkt_count   = pkt;
    s_info.rx_count    = 0;
    s_info.expect_seq  = 0;
    s_info.gw_addr     = src_addr;
    s_info.start_tick  = osal_get_tick_ms();
    s_info.retry_round = 0;
    s_info.last_missing_cnt = pkt;
}

/* === 公共接口 ============================================================
 *  以下函数供上层（mesh 回调、主循环、E-Paper 驱动）调用。
 *  所有函数线程安全性假设：单线程环境（LiteOS 任务内顺序调用）。
 * ====================================================================== */

/**
 * @brief  初始化图片接收模块
 * @note   清零接收上下文，将状态机置为 IDLE。应在系统启动时调用一次。
 */
void image_receiver_init(void)
{
    (void)memset_s(&s_info, sizeof(s_info), 0, sizeof(s_info));
    s_info.state = IMG_STATE_IDLE;
    osal_printk("%s init OK, buf=%d bytes\r\n", IMG_LOG, IMG_RX_BUF_SIZE);
}

/**
 * @brief  处理来自 mesh 网络的图片协议报文（统一入口）
 * @param  src_addr  发送方 mesh 地址
 * @param  data      原始报文数据，首字节为命令字
 * @param  len       报文长度（字节）
 * @return true 表示报文已被本模块消费，false 表示未识别的命令字
 *
 * @note   此函数通常在 mesh 接收回调中被调用，根据 data[0] 分发至具体 handler。
 */
bool image_receiver_on_data(uint16_t src_addr, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len < 1) return false;

    uint8_t cmd = data[0];
    switch (cmd) {
        case IMG_CMD_START:
            handle_start(src_addr, data, len);
            return true;
        case IMG_CMD_DATA:
            handle_data(src_addr, data, len);
            return true;
        case IMG_CMD_END:
            handle_end(src_addr, data, len);
            return true;
        case IMG_CMD_CANCEL:
            handle_cancel(src_addr, data, len);
            return true;
        case IMG_CMD_CHECKPOINT:
            handle_checkpoint(src_addr, data, len);
            return true;
        case IMG_CMD_MCAST_START:
            handle_mcast_start(src_addr, data, len);
            return true;
        default:
            return false;
    }
}

/**
 * @brief  周期性 tick 处理（建议 100~500ms 调用一次）
 * @note   职责：
 *         1. DONE / ERROR 状态下周期重发 RESULT 报文，确保网关收到最终结果；
 *         2. RECEIVING 状态下检测无活动超时，超过 IMG_RX_TIMEOUT_MS 则上报 TIMEOUT。
 */
void image_receiver_tick(void)
{
    /* ── RESULT 重发 ──
     * 为什么在 tick 中重发而不在 send_result 中开定时器？
     * → 避免引入额外定时器资源，tick 驱动模型更简单可靠 */
    if ((s_info.state == IMG_STATE_DONE || s_info.state == IMG_STATE_ERROR)
        && s_info.result_retries > 0) {
        uint32_t now = osal_get_tick_ms();
        if (now >= s_info.result_next_tick) {
            uint8_t resp[2];
            resp[0] = IMG_CMD_RESULT;
            resp[1] = s_info.result_status;
            mesh_send(s_info.gw_addr, resp, 2);
            s_info.result_retries--;
            s_info.result_next_tick = now + IMG_RESULT_RETRY_MS;
            osal_printk("%s RESULT retry status=%d remain=%d\r\n",
                        IMG_LOG, s_info.result_status, s_info.result_retries);
        }
    }

    /* ── 无活动超时检测 ──
     * 只在 RECEIVING 状态下检查；DONE/ERROR 状态不需要（已有 RESULT 重发机制） */
    if (s_info.state != IMG_STATE_RECEIVING) return;

    uint32_t now = osal_get_tick_ms();
    uint32_t elapsed = now - s_info.start_tick;

    if (elapsed > IMG_RX_TIMEOUT_MS) {
        osal_printk("%s TIMEOUT: rx=%d/%d elapsed=%lums\r\n",
                    IMG_LOG, s_info.rx_count, s_info.pkt_count, elapsed);
        send_result(s_info.gw_addr, IMG_RESULT_TIMEOUT);
        s_info.state = IMG_STATE_ERROR;
    }
}

/**
 * @brief  获取当前接收上下文（只读）
 * @return 指向内部 s_info 的 const 指针，上层可查询进度/状态
 */
const img_rx_info_t *image_receiver_get_info(void)
{
    return &s_info;
}

/**
 * @brief  获取图片数据缓冲区（只读）
 * @return 指向 s_img_buf 的 const 指针
 * @note   仅当 state == IMG_STATE_DONE 时数据有效，可直接送 E-Paper 驱动刷屏。
 */
const uint8_t *image_receiver_get_buffer(void)
{
    return s_img_buf;
}

uint8_t *image_receiver_get_buffer_writable(void)
{
    return s_img_buf;
}

/**
 * @brief  强制重置接收模块至 IDLE 状态
 * @note   清除接收进度和 RESULT 重发计数器。
 *         典型用法：E-Paper 刷屏完成后调用，为下一次图传做准备。
 */
void image_receiver_reset(void)
{
    s_info.state = IMG_STATE_IDLE;
    s_info.rx_count = 0;
    s_info.result_retries = 0;
    s_mcast_count = 0;
    osal_printk("%s reset → IDLE\r\n", IMG_LOG);
}

/**
 * @brief  自发自收直接加载: 跳过逐包注入回环，直接 memcpy + CRC 校验
 *
 * 流程:
 *   1. 长度校验 (total_bytes <= IMG_RX_BUF_SIZE)
 *   2. CRC16 预校验 src_buf，不匹配则立即返回 CRC_ERR
 *   3. memcpy src_buf → s_img_buf
 *   4. 设置 s_info 为 DONE 状态 (供 ePaper 任务检测并刷屏)
 *   5. 若 mode == JPEG，留待 ePaper 任务流式解码
 */
uint8_t image_receiver_load_direct(const uint8_t *src_buf, uint16_t total_bytes,
                                   uint16_t pkt_count, uint16_t width, uint16_t height,
                                   uint8_t mode, uint16_t expected_crc)
{
    if (src_buf == NULL || total_bytes == 0 || total_bytes > IMG_RX_BUF_SIZE) {
        osal_printk("%s DIRECT: invalid params total=%d\r\n", IMG_LOG, total_bytes);
        return IMG_RESULT_OOM;
    }

    /* CRC 预校验: 在源缓冲区上直接计算，避免无意义的 memcpy */
    uint16_t actual_crc = crc16_ccitt(src_buf, total_bytes);
    if (actual_crc != expected_crc) {
        osal_printk("%s DIRECT: cache CRC MISMATCH expect=0x%04X actual=0x%04X\r\n",
                    IMG_LOG, expected_crc, actual_crc);
        return IMG_RESULT_CRC_ERR;
    }

    /* 取消可能正在进行的接收 */
    if (s_info.state == IMG_STATE_RECEIVING) {
        osal_printk("%s DIRECT: cancel prev transfer\r\n", IMG_LOG);
    }

    /* 直接拷贝: 单次 memcpy 替代 90 次逐包回环注入 */
    (void)memcpy_s(s_img_buf, IMG_RX_BUF_SIZE, src_buf, total_bytes);

    /* 设置接收完成状态 */
    (void)memset_s(&s_info, sizeof(s_info), 0, sizeof(s_info));
    s_info.state       = IMG_STATE_DONE;
    s_info.width       = width;
    s_info.height      = height;
    s_info.mode        = mode;
    s_info.xfer_mode   = IMG_XFER_FAST;
    s_info.total_bytes = total_bytes;
    s_info.pkt_count   = pkt_count;
    s_info.rx_count    = pkt_count;
    s_info.gw_addr     = 0;  /* 自发自收无需回复 */

    /* JPEG: 留待 ePaper 任务流式解码 */
    if (mode == IMG_MODE_JPEG) {
        osal_printk("%s DIRECT: JPEG %d bytes, defer to ePaper task\r\n",
                    IMG_LOG, total_bytes);
    }

    osal_printk("%s DIRECT: %dx%d %dB CRC OK → buffer ready\r\n",
                IMG_LOG, width, height, total_bytes);
    return IMG_RESULT_OK;
}