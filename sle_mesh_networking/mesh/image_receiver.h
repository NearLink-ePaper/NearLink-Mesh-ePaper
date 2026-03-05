/**
 * @file  image_receiver.h
 * @brief SLE Mesh 图传接收端 —— 图片接收状态机公共头文件
 *
 * @details
 *   本模块运行在 BearPi-Pico H3863 的 Receiver（目标）节点上，负责：
 *     1. 从 SLE Mesh 网络接收网关/源端发送的图片分包（IMG_CMD_DATA）；
 *     2. 使用位图（rx_bitmap）跟踪每个分包的到达状态；
 *     3. 收到 CHECKPOINT 时回报进度，收到 END 时做 CRC16 校验；
 *     4. 校验通过后上层可读取 image_receiver_get_buffer() 送往电子墨水屏刷新。
 *
 *   协议版本 v2 改进：
 *     - 新增 CHECKPOINT（0x08）分段检查点机制，网关可在传输中途查询进度；
 *     - MISSING 报文改为 30 字节位图格式，一次覆盖 240 个 seq，彻底解决报不完问题；
 *     - 收到 DATA / CHECKPOINT 均会重置无活动超时计时器，防止误判超时。
 *
 *   典型数据流：
 *     网关 → START → DATA×N → (CHECKPOINT) → END
 *     节点 → (ACK / CHKPT_ACK / MISSING) → RESULT
 */
#ifndef IMAGE_RECEIVER_H
#define IMAGE_RECEIVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* === 缓冲区与分包参数 ===================================================
 *  本段定义图片接收缓冲区大小、单包载荷、位图尺寸及超时/重试上限。
 *  设计思路：缓冲区按最大支持分辨率（480×800 / 1bpp = 48000 字节）静态分配，
 *  避免运行时动态申请内存；位图以 1-bit-per-seq 记录到达状态，空间开销极小。
 * ====================================================================== */

/** 图片接收缓冲区大小（字节）。
 *  计算方式：480×800 二值化 → (480/8)×800 = 48000 字节。
 *  调优建议：若实际分辨率更小，可适当缩减以节省 RAM。 */
#define IMG_RX_BUF_SIZE         48000

/** 单个分包最大载荷（字节）。
 *  O4 优化：从 200 提升到 480，与 mesh_config.h 中 IMG_FC_PKT_PAYLOAD 保持一致。
 *  取值范围：不能超过 mesh 底层 MTU 减去 5 字节头部开销。 */
#define IMG_PKT_PAYLOAD         480

/** 分包总数上限，向上取整：48000 / 480 = 100 包（当前实际值） */
#define IMG_MAX_PKTS            ((IMG_RX_BUF_SIZE + IMG_PKT_PAYLOAD - 1) / IMG_PKT_PAYLOAD)  /* 240 */
/** 位图字节数，每 bit 对应一个 seq：ceil(240/8) = 30 字节 */
#define IMG_BITMAP_BYTES        ((IMG_MAX_PKTS + 7) / 8)   /* 30 bytes */

/** 补包最大重试轮次。超过此轮次后放弃接收并上报 CRC_ERR。
 *  取值范围：1~10，建议 3~5；过大会延长失败恢复时间。 */
#define IMG_MAX_RETRY_ROUNDS    5

/** 无活动超时（毫秒）。收到 DATA / CHECKPOINT 都会重置计时器。
 *  取值范围：5000~60000；30s 适合大多数 mesh 拓扑。 */
#define IMG_RX_TIMEOUT_MS       30000   /* 30 秒 */

/* === 协议命令字 ===========================================================
 *  下行（网关 → 目标节点）命令字范围 0x01~0x7F；
 *  上行（目标节点 → 网关）命令字范围 0x80~0xFF，最高位置 1 以示区分。
 * ====================================================================== */

/* —— 下行命令（网关 → 接收节点） —— */
#define IMG_CMD_START           0x04    /**< 开始传输：携带图片元信息（尺寸/包数/模式） */
#define IMG_CMD_DATA            0x05    /**< 数据分包：[cmd, seq_hi, seq_lo, len_hi, len_lo, payload...] */
#define IMG_CMD_END             0x06    /**< 传输结束：携带 CRC16 校验值，触发缺包检查 */
#define IMG_CMD_CANCEL          0x07    /**< 取消传输：源端主动终止本次图传 */
#define IMG_CMD_CHECKPOINT      0x08    /**< 分段检查点（v2）：网关分段发完后查询接收进度 */

/* —— 上行命令（接收节点 → 网关） —— */
#define IMG_CMD_ACK             0x85    /**< 单包确认（ACK 模式下使用） */
#define IMG_CMD_RESULT          0x86    /**< 最终结果：OK / OOM / TIMEOUT / CRC_ERR 等 */
#define IMG_CMD_MISSING         0x87    /**< 缺包位图（v2）：[cmd, cnt_hi, cnt_lo, bitmap[30]] */
#define IMG_CMD_CHKPT_ACK       0x88    /**< 检查点应答（v2）：回报当前已收包数 */

/* === ACK 状态码 ===========================================================
 *  随 IMG_CMD_ACK 报文发送，用于逐包确认模式下指示单包接收结果。
 * ====================================================================== */
#define IMG_ACK_OK              0x00    /**< 该包接收成功 */
#define IMG_ACK_RESEND          0x01    /**< 请求重发该包 */
#define IMG_ACK_DONE            0xFF    /**< 所有包已收齐，传输完成 */

/* === 最终结果状态码 =======================================================
 *  随 IMG_CMD_RESULT 报文发送，指示整张图片传输的最终状态。
 *  网关根据此状态决定是否重传或通知用户。
 * ====================================================================== */
#define IMG_RESULT_OK           0x00    /**< 传输成功，CRC 校验通过 */
#define IMG_RESULT_OOM          0x01    /**< 内存不足：图片 total_bytes > IMG_RX_BUF_SIZE */
#define IMG_RESULT_TIMEOUT      0x02    /**< 无活动超时：超过 IMG_RX_TIMEOUT_MS 未收到数据 */
#define IMG_RESULT_CANCEL       0x03    /**< 源端主动取消 */
#define IMG_RESULT_CRC_ERR      0x04    /**< CRC 校验失败或补包轮次超限 */

/* === 传输模式 =============================================================
 *  在 START 报文中由源端指定，影响接收端是否逐包回 ACK。
 *  FAST 模式吞吐更高但依赖 END 阶段补包；ACK 模式更可靠但速度较慢。
 * ====================================================================== */
#define IMG_XFER_FAST           0x00    /**< 快速模式：不逐包确认，仅在 END 阶段上报缺包 */
#define IMG_XFER_ACK            0x01    /**< 确认模式：每包回复 ACK，丢包立即重传 */

/* === 接收状态机枚举 =======================================================
 *  状态迁移路径：
 *    IDLE ──START──▶ RECEIVING ──END(OK)──▶ DONE
 *                       │                     │
 *                       ├──TIMEOUT/CRC_ERR──▶ ERROR
 *                       └──CANCEL───────────▶ IDLE
 *  DONE / ERROR 状态下仍会重发 RESULT 报文若干轮以确保网关收到。
 * ====================================================================== */
typedef enum {
    IMG_STATE_IDLE = 0,     /**< 空闲，等待 START 命令 */
    IMG_STATE_RECEIVING,    /**< 接收中，持续写入缓冲区并更新位图 */
    IMG_STATE_DONE,         /**< 接收完成，CRC 校验通过，缓冲区数据可用 */
    IMG_STATE_ERROR,        /**< 出错（CRC 不匹配 / 超时 / 超过重试上限） */
} img_rx_state_t;

/* === 接收上下文结构体 =====================================================
 *  单例运行（模块内 static），保存当前图传会话的全部状态。
 *  接收端同一时刻只处理一张图片；若收到新的 START，会覆盖旧会话。
 * ====================================================================== */
typedef struct {
    img_rx_state_t state;       /**< 当前状态机状态 */
    uint16_t width;             /**< 图片宽度（像素），由 START 携带 */
    uint16_t height;            /**< 图片高度（像素），由 START 携带 */
    uint8_t  mode;              /**< 图片模式：0=1bpp 黑白，其他保留 */
    uint8_t  xfer_mode;         /**< 传输模式：IMG_XFER_FAST / IMG_XFER_ACK */
    uint16_t total_bytes;       /**< 图片总字节数，不超过 IMG_RX_BUF_SIZE */
    uint16_t pkt_count;         /**< 分包总数，由 START 携带 */
    uint16_t rx_count;          /**< 已成功接收的分包数量（去重后） */
    uint16_t expect_seq;        /**< 下一个期望的 seq，用于顺序检测 */
    uint16_t gw_addr;           /**< 源端（网关）mesh 地址，用于回复 */
    uint32_t start_tick;        /**< 上次活动时间戳（ms），用于超时检测 */
    uint8_t  result_status;     /**< 最终结果状态码，用于 RESULT 重发 */
    uint8_t  result_retries;    /**< RESULT 报文剩余重发次数 */
    uint32_t result_next_tick;  /**< 下一次 RESULT 重发的时间戳（ms） */
    uint8_t  rx_bitmap[IMG_BITMAP_BYTES]; /**< 分包到达位图，bit=1 表示该 seq 已收到 */
    uint8_t  retry_round;       /**< 当前补包轮次，0=首轮，上限 IMG_MAX_RETRY_ROUNDS */
    uint16_t last_missing_cnt;  /**< 上一轮缺包数量，用于检测补包是否有进展 */
} img_rx_info_t;

/* === 公共 API =============================================================
 *  本模块对外仅暴露以下 6 个函数，所有内部状态不可直接访问。
 *  调用顺序：init() → 循环 { on_data() + tick() } → get_buffer()
 * ====================================================================== */

/**
 * @brief  初始化图片接收模块
 * @note   清零接收上下文，将状态置为 IDLE。应在系统启动时调用一次。
 */
void image_receiver_init(void);

/**
 * @brief  处理从 mesh 网络收到的图片协议报文
 * @param  src_addr  发送方的 mesh 网络地址
 * @param  data      报文原始数据指针（首字节为命令字）
 * @param  len       报文长度（字节）
 * @return true  —— 该报文已被本模块消费（命令字匹配）
 * @return false —— 未识别的命令字，调用者可继续分发给其他模块
 */
bool image_receiver_on_data(uint16_t src_addr, const uint8_t *data, uint16_t len);

/**
 * @brief  周期性 tick 处理（建议 100~500ms 调用一次）
 * @note   负责两件事：
 *         1. RESULT 报文超时重发（确保网关收到最终结果）；
 *         2. 无活动超时检测（超过 IMG_RX_TIMEOUT_MS 则上报 TIMEOUT）。
 */
void image_receiver_tick(void);

/**
 * @brief  获取当前接收上下文（只读）
 * @return 指向内部 img_rx_info_t 的 const 指针，生命周期与模块相同
 */
const img_rx_info_t *image_receiver_get_info(void);

/**
 * @brief  获取图片接收缓冲区（只读）
 * @return 指向 s_img_buf[IMG_RX_BUF_SIZE] 的 const 指针
 * @note   仅在 state == IMG_STATE_DONE 时数据有效
 */
const uint8_t *image_receiver_get_buffer(void);

/**
 * @brief  强制重置接收模块至 IDLE 状态
 * @note   清除进度并停止 RESULT 重发，可在刷屏完成后调用以准备下次传输。
 */
void image_receiver_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* IMAGE_RECEIVER_H */