#ifdef CONFIG_HUAWEI_DSM
#include "mtk_camera_dsm.h"

//these bits should be the same to LM3642's spec exactly
#define LED_TIMEOUT                  (1<<0)
#define LED_THERMAL_SHUTDOWN         (1<<1)
#define LED_VOUT_ERR                 (1<<2)

//thermal engine shut down the flash
#define LED_POWER_SAVING             (1<<3)

struct flash_dsm_report
{
    int report_mask;
    const char *report_msg;
    int dsm_error_no;
};

void camera_report_flash_dsm_err(int flash_flag);
#endif
