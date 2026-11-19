#ifndef ORB_DRIVER_H_
#define ORB_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include "system.h"

// ============================================================================
// 1. Hang so & Register Map (Word Offset cho IOWR / IORD 32-bit)
// ============================================================================
#define ORB_IP_SIGNATURE        0x0B202026u
#define ORB_MAX_KEYPOINTS       2048u
#define ORB_WORDS_PER_KP        19u         // 19 x 16-bit = 38 bytes / keypoint

// Word offsets cua cong avs_csr (ORB_0_BASE = 0x2003000)
#define ORB_REG_CTRL            0           // [W] Bit 0: Start, Bit 1: IRQ_EN, Bit 2: Clear Sticky
#define ORB_REG_STATUS          1           // [R] Cac co trang thai va loi
#define ORB_REG_DMA_BASE        2           // [R/W] Dia chi byte tren SDRAM luu ket qua
#define ORB_REG_THRESHOLD       3           // [R/W] Nguong FAST-9 (8-bit, mac dinh 20)
#define ORB_REG_OUT_COUNT       4           // [R] So keypoint da trich xuat
#define ORB_REG_WORDS_WRITTEN   5           // [R] Tong so word 16-bit DMA da ghi
#define ORB_REG_SIGNATURE       6           // [R] Ma nhan dien IP: 0x0B202026

// Bitmasks cho thanh ghi CTRL (Offset 0)
#define ORB_CTRL_START_MASK     (1u << 0)
#define ORB_CTRL_IRQ_EN_MASK    (1u << 1)
#define ORB_CTRL_CLR_STICKY     (1u << 2)

// Bitmasks cho thanh ghi STATUS (Offset 1)
#define ORB_STAT_BUSY_MASK      (1u << 0)
#define ORB_STAT_DONE_MASK      (1u << 1)
#define ORB_STAT_ERR_MASK       (1u << 2)
#define ORB_STAT_KP_OVF_MASK    (1u << 3)
#define ORB_STAT_PKT_OVF_MASK   (1u << 4)
#define ORB_STAT_IMG_LOADED     (1u << 5)
#define ORB_STAT_LOAD_MODE      (1u << 6)

// Ma tra ve cua Driver
typedef enum {
    ORB_OK = 0,
    ORB_ERR_SIGNATURE,      // Sai IP signature (loi ket noi bus/Qsys)
    ORB_ERR_NOT_READY,      // Chua nap anh vao SRAM hoac dang o che do Load Mode
    ORB_ERR_BUSY,           // Khoi ORB dang ban xu ly frame truoc
    ORB_ERR_HARDWARE,       // Co loi line buffer / image reader (err_sticky)
    ORB_ERR_OVERFLOW,       // Tran bo dem keypoint (>2048) hoac tran packer
    ORB_ERR_TIMEOUT         // Qua thoi gian cho
} orb_status_t;

// ============================================================================
// 2. Cau truc du lieu tho 38-byte tren SDRAM (khop 100% orb_result_packer.v)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint16_t word0_x;       // [9:0]  = X (0..639)
    uint16_t word1_y;       // [8:0]  = Y (0..479)
    uint16_t word2_meta;    // [7:0]  = Score, [12:8] = Angle (0..31)
    uint16_t desc_w[16];    // Word 3..18: 256-bit rBRIEF (desc_w[0] = [15:0], desc_w[15] = [255:240])
} orb_raw_kp_t;

// Cau truc da giai ma gon gang de xu ly tren phan mem
typedef struct {
    uint16_t x;
    uint16_t y;
    uint8_t  score;
    uint8_t  angle_bin;     // 0..31 (moi bin = 11.25 do)
    uint32_t desc[8];       // 256-bit rBRIEF (desc[0] = LSB [31:0], desc[7] = MSB [255:224])
} orb_keypoint_t;

// ============================================================================
// 3. Khai bao ham API
// ============================================================================
orb_status_t orb_init(void);
uint32_t     orb_get_status(void);
bool         orb_is_image_ready(void);
void         orb_clear_flags(void);

// Chay ORB theo che do Polling (de debug va do thoi gian)
orb_status_t orb_run_polling(uint32_t dma_sdram_addr, uint8_t fast_threshold, uint32_t *out_kp_count);

// Chay ORB theo che do Ngat (IRQ 0)
orb_status_t orb_register_irq(void (*user_callback)(orb_status_t status, uint32_t kp_count));
orb_status_t orb_start_async(uint32_t dma_sdram_addr, uint8_t fast_threshold);

// Giai ma ket qua tu SDRAM sang mang struct C
uint32_t     orb_fetch_keypoints(uint32_t dma_sdram_addr, orb_keypoint_t *dst_array, uint32_t max_items);

// Tinh khoang cach Hamming giua 2 descriptor 256-bit
uint32_t     orb_hamming_distance(const uint32_t desc_a[8], const uint32_t desc_b[8]);

#endif /* ORB_DRIVER_H_ */
