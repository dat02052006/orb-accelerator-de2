#include "orb_driver.h"
#include <io.h>
#include "sys/alt_irq.h"

static void (*g_user_irq_cb)(orb_status_t status, uint32_t kp_count) = 0;

// Ham phuc vu ngat (ISR) khi ORB_0 kich hoat IRQ 0
static void orb_isr_handler(void *context) {
    uint32_t st;
    uint32_t kp_cnt;
    orb_status_t result = ORB_OK;

    (void)context;

    st = IORD(ORB_0_BASE, ORB_REG_STATUS);
    kp_cnt = IORD(ORB_0_BASE, ORB_REG_OUT_COUNT);

    // Xoa co sticky de ha tin hieu ngat irq xuong muc 0
    IOWR(ORB_0_BASE, ORB_REG_CTRL, ORB_CTRL_CLR_STICKY);

    if (st & ORB_STAT_ERR_MASK) {
        result = ORB_ERR_HARDWARE;
    } else if (st & (ORB_STAT_KP_OVF_MASK | ORB_STAT_PKT_OVF_MASK)) {
        result = ORB_ERR_OVERFLOW;
    }

    if (g_user_irq_cb) {
        g_user_irq_cb(result, kp_cnt);
    }
}

orb_status_t orb_init(void) {
    uint32_t sig = IORD(ORB_0_BASE, ORB_REG_SIGNATURE);
    if (sig != ORB_IP_SIGNATURE) {
        return ORB_ERR_SIGNATURE;
    }
    // Tat ngat va xoa toan bo co trang thai cu
    IOWR(ORB_0_BASE, ORB_REG_CTRL, ORB_CTRL_CLR_STICKY);
    return ORB_OK;
}

uint32_t orb_get_status(void) {
    return IORD(ORB_0_BASE, ORB_REG_STATUS);
}

bool orb_is_image_ready(void) {
    uint32_t st = IORD(ORB_0_BASE, ORB_REG_STATUS);
    return ((st & ORB_STAT_IMG_LOADED) != 0) && ((st & ORB_STAT_LOAD_MODE) == 0);
}

void orb_clear_flags(void) {
    IOWR(ORB_0_BASE, ORB_REG_CTRL, ORB_CTRL_CLR_STICKY);
}

orb_status_t orb_run_polling(uint32_t dma_sdram_addr, uint8_t fast_threshold, uint32_t *out_kp_count) {
    uint32_t st = IORD(ORB_0_BASE, ORB_REG_STATUS);
    uint32_t timeout = 50000000u; // ~1 giay o 50MHz

    if (st & ORB_STAT_BUSY_MASK) {
        return ORB_ERR_BUSY;
    }
    if (!orb_is_image_ready()) {
        return ORB_ERR_NOT_READY;
    }

    // 1. Cau hinh dia chi DMA tren SDRAM va nguong FAST
    IOWR(ORB_0_BASE, ORB_REG_DMA_BASE, dma_sdram_addr);
    IOWR(ORB_0_BASE, ORB_REG_THRESHOLD, (uint32_t)fast_threshold);

    // 2. Phat xung Start (bit 0 = 1, irq_en = 0)
    IOWR(ORB_0_BASE, ORB_REG_CTRL, ORB_CTRL_START_MASK);

    // 3. Cho co done_sticky hoac err_sticky
    while (timeout--) {
        st = IORD(ORB_0_BASE, ORB_REG_STATUS);
        if (st & (ORB_STAT_DONE_MASK | ORB_STAT_ERR_MASK)) {
            break;
        }
    }

    if (timeout == 0) {
        return ORB_ERR_TIMEOUT;
    }

    if (out_kp_count) {
        *out_kp_count = IORD(ORB_0_BASE, ORB_REG_OUT_COUNT);
    }

    if (st & ORB_STAT_ERR_MASK) {
        return ORB_ERR_HARDWARE;
    }
    if (st & (ORB_STAT_KP_OVF_MASK | ORB_STAT_PKT_OVF_MASK)) {
        return ORB_ERR_OVERFLOW;
    }

    return ORB_OK;
}

orb_status_t orb_register_irq(void (*user_callback)(orb_status_t status, uint32_t kp_count)) {
    int rc;
    g_user_irq_cb = user_callback;
    rc = alt_ic_isr_register(
        ORB_0_IRQ_INTERRUPT_CONTROLLER_ID,
        ORB_0_IRQ,
        orb_isr_handler,
        0,
        0
    );
    return (rc == 0) ? ORB_OK : ORB_ERR_HARDWARE;
}

orb_status_t orb_start_async(uint32_t dma_sdram_addr, uint8_t fast_threshold) {
    uint32_t st = IORD(ORB_0_BASE, ORB_REG_STATUS);
    if (st & ORB_STAT_BUSY_MASK) {
        return ORB_ERR_BUSY;
    }
    if (!orb_is_image_ready()) {
        return ORB_ERR_NOT_READY;
    }

    IOWR(ORB_0_BASE, ORB_REG_DMA_BASE, dma_sdram_addr);
    IOWR(ORB_0_BASE, ORB_REG_THRESHOLD, (uint32_t)fast_threshold);

    // Kich hoat Start (bit 0) dong thoi bat IRQ_EN (bit 1)
    IOWR(ORB_0_BASE, ORB_REG_CTRL, ORB_CTRL_START_MASK | ORB_CTRL_IRQ_EN_MASK);
    return ORB_OK;
}

uint32_t orb_fetch_keypoints(uint32_t dma_sdram_addr, orb_keypoint_t *dst_array, uint32_t max_items) {
    uint32_t hw_kp_count   = IORD(ORB_0_BASE, ORB_REG_OUT_COUNT);
    uint32_t words_written = IORD(ORB_0_BASE, ORB_REG_WORDS_WRITTEN);
    uint32_t dma_kp_count  = words_written / ORB_WORDS_PER_KP;
    uint32_t valid_count   = (hw_kp_count < dma_kp_count) ? hw_kp_count : dma_kp_count;
    const volatile orb_raw_kp_t *raw_buf = (const volatile orb_raw_kp_t *)dma_sdram_addr;
    uint32_t i;
    int w;

    if (valid_count > max_items) {
        valid_count = max_items;
    }

    for (i = 0; i < valid_count; i++) {
        dst_array[i].x         = raw_buf[i].word0_x & 0x03FFu;
        dst_array[i].y         = raw_buf[i].word1_y & 0x01FFu;
        dst_array[i].score     = (uint8_t)(raw_buf[i].word2_meta & 0x00FFu);
        dst_array[i].angle_bin = (uint8_t)((raw_buf[i].word2_meta >> 8) & 0x001Fu);

        for (w = 0; w < 8; w++) {
            uint32_t lo = (uint32_t)raw_buf[i].desc_w[2 * w];
            uint32_t hi = (uint32_t)raw_buf[i].desc_w[2 * w + 1];
            dst_array[i].desc[w] = (hi << 16) | lo;
        }
    }

    return valid_count;
}

uint32_t orb_hamming_distance(const uint32_t desc_a[8], const uint32_t desc_b[8]) {
    uint32_t dist = 0;
    int i;
    for (i = 0; i < 8; i++) {
        dist += (uint32_t)__builtin_popcount(desc_a[i] ^ desc_b[i]);
    }
    return dist;
}
