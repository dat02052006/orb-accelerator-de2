#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <io.h>
#include "system.h"
#include "orb_driver.h"

// Dat vung dem ket qua ORB o nua sau SDRAM (offset +4MB) de tranh de len code/stack
#define ORB_RESULT_SDRAM_ADDR   (NEW_SDRAM_CONTROLLER_0_BASE + 0x00400000u)
#define DEFAULT_FAST_THRESHOLD  20u

static orb_keypoint_t g_keypoints[ORB_MAX_KEYPOINTS];

int main(void) {
    orb_status_t st;
    uint32_t kp_count = 0;
    uint32_t words_written;
    uint32_t fetched;
    uint32_t print_limit;
    uint32_t i;
    int w;

    printf("=== ORB HARDWARE ACCELERATOR TEST (NIOS II) ===\n");

    // 1. Kiem tra ket noi voi IP Core
    st = orb_init();
    if (st != ORB_OK) {
        printf("[ERROR] Khong tim thay ORB IP (Signature doc duoc: 0x%08lX)\n",
               (unsigned long)IORD(ORB_0_BASE, ORB_REG_SIGNATURE));
        return -1;
    }
    printf("[OK] Da ket noi ORB IP (Signature = 0x%08X)\n", ORB_IP_SIGNATURE);

    // 2. Cho PC nap anh 640x480 vao SRAM qua JTAG-to-Avalon Master
    if (!orb_is_image_ready()) {
        printf("[WAIT] Dang cho nap anh 640x480 vao SRAM qua JTAG...\n");
        while (!orb_is_image_ready()) {
            usleep(100000); // Kiem tra moi 100ms
        }
    }
    printf("[OK] Anh da san sang trong SRAM!\n");

    // 3. Kich hoat xu ly ORB (Polling)
    printf("[RUN] Bat dau trich xuat dac trung ORB (Threshold = %u)...\n", DEFAULT_FAST_THRESHOLD);

    st = orb_run_polling(ORB_RESULT_SDRAM_ADDR, DEFAULT_FAST_THRESHOLD, &kp_count);
    if (st != ORB_OK) {
        printf("[ERROR] Loi khi chay ORB! Ma loi: %d | STATUS = 0x%08lX\n",
               st, (unsigned long)orb_get_status());
        return -1;
    }

    words_written = IORD(ORB_0_BASE, ORB_REG_WORDS_WRITTEN);
    printf("[DONE] Hoan tat! Tim thay %lu keypoints (%lu words 16-bit da ghi DMA)\n",
           (unsigned long)kp_count, (unsigned long)words_written);

    // 4. Doc va giai ma du lieu tu SDRAM
    fetched = orb_fetch_keypoints(ORB_RESULT_SDRAM_ADDR, g_keypoints, ORB_MAX_KEYPOINTS);

    // 5. In ra Console de doi chieu bit-exact voi OpenCV
    print_limit = (fetched < 20) ? fetched : 20;
    printf("\n--- DANH SACH KEYPOINTS (Hien thi %lu / %lu) ---\n",
           (unsigned long)print_limit, (unsigned long)fetched);

    for (i = 0; i < print_limit; i++) {
        printf("KP[%04lu]: X=%3u, Y=%3u, Score=%3u, AngleBin=%2u | Desc=0x",
               (unsigned long)i,
               g_keypoints[i].x,
               g_keypoints[i].y,
               g_keypoints[i].score,
               g_keypoints[i].angle_bin);

        // In 256-bit descriptor tu MSB (desc[7]) den LSB (desc[0])
        for (w = 7; w >= 0; w--) {
            printf("%08lX", (unsigned long)g_keypoints[i].desc[w]);
        }
        printf("\n");
    }

    return 0;
}
