#include "fp_vendor.h"
#include<linux/types.h>
#include "../tz_driver/include/nt_smc_call.h"
#include<linux/kernel.h>
#include <linux/mutex.h>


#define FPC_VENDOR_ID           0x12
#define GOODIX_VENDOR_ID        0x22
#define GOODIX_VENDOR_ID_3268   0x0C
#define GOODIX_VENDOR_ID_3258   0x0D
#define LEADCORE_VENDOR_ID      0x71
#define CHIPSAIL_VENDOR_ID      0x31

int fp_vendor_active = 0;
int fp_vendor = FP_VENDOR_INVALID;
static DEFINE_MUTEX(fp_vendor_lock);

int get_fp_vendor(void)
{
    uint64_t fp_vendor_id_64 = 0;
    uint32_t *p_temp = NULL;
    uint32_t fp_vendor_id_32 = 0;
    uint32_t fp_vendor_id_32_Low8=0;

    fp_vendor = FP_VENDOR_INVALID;
    mutex_lock(&fp_vendor_lock);

    if (fp_vendor_active) {
    mutex_unlock(&fp_vendor_lock);
    return fp_vendor;
    }

    get_t_device_id(&fp_vendor_id_64);
    printk("in %s, line:%d     fp_vendor_id_64:0x%016llx\n",__func__,__LINE__,fp_vendor_id_64);
    p_temp = (uint32_t *)&fp_vendor_id_64;
    fp_vendor_id_32 = *p_temp;
    fp_vendor_id_32_Low8 = (fp_vendor_id_32 >> 0) & 0xff;
    fp_vendor_id_32 = (fp_vendor_id_32 >> 8) & 0xff;

    printk("%s:%d->0x%x\n", __func__, __LINE__, fp_vendor_id_32);

    switch (fp_vendor_id_32) {
    case FPC_VENDOR_ID:
        fp_vendor = FPC_VENDOR;
        break;

    case GOODIX_VENDOR_ID:
        if(fp_vendor_id_32_Low8 == GOODIX_VENDOR_ID_3268)
            fp_vendor = GOODIX_VENDOR_3268;
        else if(fp_vendor_id_32_Low8 == GOODIX_VENDOR_ID_3258)
            fp_vendor = GOODIX_VENDOR_3258;
        break;

    case LEADCORE_VENDOR_ID :
        fp_vendor = LEADCORE_VENDOR;
        break;
    case CHIPSAIL_VENDOR_ID :
        fp_vendor = CHIPSAIL_VENDOR;
        break;

    default:
        fp_vendor = FP_VENDOR_INVALID;
        break;
    }

    fp_vendor_active = 1;
    mutex_unlock(&fp_vendor_lock);
    fp_vendor_active = 0;
    return fp_vendor;
}
