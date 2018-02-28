/*
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/completion.h>
#include <linux/gpio.h>

#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include "fp_vendor.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/*#include <mach/hardware.h>*/
/*#include <mach/mt_irq.h>*/
#include <mt_spi.h>
#include <mt_spi_hal.h>
//#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
//#include <mach/upmu_common.h>
//#include <mach/eint.h>

//#include <mach/memory.h>
#include <mach/mt_clkmgr.h>
#include <net/sock.h>

//#include <cust_eint.h>
//#include "cust_gpio_usage.h"
#include <dt-bindings/pinctrl/mt65xx.h>
#include "fpsensor_spi_tee.h"

/*spi device name*/
#define FPSENSOR_DEV_NAME   "fpsensor"
/*device name after register in charater*/
// #define DEV_NAME "sensor_fp"

#define FPSENSOR_CLASS_NAME            "fpsensor"
#define FPSENSOR_MAJOR            0
#define N_SPI_MINORS            32    /* ... up to 256 */

#define FPSENSOR_SPI_VERSION "fpsensor_spi_tee_v0.1"
#define FPSENSOR_INPUT_NAME  "fpsensor_keys"
extern int get_fp_vendor(void);

/*
#ifdef CONFIG_OF

static struct of_device_id fpsensor_of_match[] = {
{ .compatible = "leadcore,leadcore-fp", },
{}
};
MODULE_DEVICE_TABLE(of, fpsensor_of_match);

#endif
 */

/************************GPIO setting port layer*************************/
/* customer hardware port layer, please change according to customer's hardware */
/*
#ifndef GPIO_FP_INT_PIN
#define GPIO_FP_INT_PIN         (GPIO96 | 0x80000000)//(GPIO5 | 0x80000000)
#define GPIO_FP_INT_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_FP_INT_PIN_M_EINT  GPIO_FP_INT_PIN_M_GPIO
#endif
*/
/*
#define GPIO_FP_SPICLK_PIN         (GPIO66 | 0x80000000)
#define GPIO_FP_SPICLK_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_FP_SPICLK_PIN_M_PWM  GPIO_MODE_03
#define GPIO_FP_SPICLK_PIN_M_SPI_CK   GPIO_MODE_01

#define GPIO_FP_SPIMISO_PIN         (GPIO67 | 0x80000000)
#define GPIO_FP_SPIMISO_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_FP_SPIMISO_PIN_M_PWM  GPIO_MODE_03
#define GPIO_FP_SPIMISO_PIN_M_SPI_MI   GPIO_MODE_01

#define GPIO_FP_SPIMOSI_PIN         (GPIO68 | 0x80000000)
#define GPIO_FP_SPIMOSI_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_FP_SPIMOSI_PIN_M_MDEINT  GPIO_MODE_02
#define GPIO_FP_SPIMOSI_PIN_M_PWM  GPIO_MODE_03
#define GPIO_FP_SPIMOSI_PIN_M_SPI_MO   GPIO_MODE_01

#define GPIO_FP_SPICS_PIN         (GPIO65 | 0x80000000)
#define GPIO_FP_SPICS_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_FP_SPICS_PIN_M_MDEINT  GPIO_MODE_02
#define GPIO_FP_SPICS_PIN_M_PWM  GPIO_MODE_03
#define GPIO_FP_SPICS_PIN_M_SPI_CS   GPIO_MODE_01
*/
#if 0
#define GPIO_FP_RESET_PIN       (GPIO122 | 0x80000000)
#define GPIO_FP_RESET_PIN         (GPIO97 | 0x80000000)  //(GPIO8 | 0x80000000)
#define GPIO_FP_RESET_PIN_M_GPIO  GPIO_MODE_00

#define GPIO_FP_SPI_BYPASS_PIN         (GPIO133 | 0x80000000)
#define GPIO_FP_SPI_BYPASS_M_GPIO  GPIO_MODE_00

#ifndef CUST_EINT_FP_EINT_NUM
#define CUST_EINT_FP_EINT_NUM              289//5
#define CUST_EINT_FP_EINT_DEBOUNCE_CN      0
#define CUST_EINT_FP_EINT_TYPE             EINTF_TRIGGER_RISING /* EINTF_TRIGGER_LOW */
#define CUST_EINT_FP_EINT_DEBOUNCE_EN      0  /* CUST_EINT_DEBOUNCE_DISABLE */
#endif
#endif

/**************************feature control******************************/
#define FPSENSOR_NETLINK     0 /* If support netlink mechanism */
#define FPSENSOR_IOCTL      1
#define FPSENSOR_SYSFS    0 
#define  NETLINK_FP    26   /* for FPSENSOR test only, need defined in include/uapi/linux/netlink.h */
#define MAX_NL_MSG_LEN 16

/***********************input *************************/
#ifndef FPSENSOR_INPUT_HOME_KEY
/* on MTK EVB board, home key has been redefine to KEY_HOMEPAGE! */
/* double check the define on customer board!!! */
#define FPSENSOR_INPUT_HOME_KEY KEY_HOMEPAGE /* KEY_HOME */

#define FPSENSOR_INPUT_MENU_KEY  KEY_MENU
#define FPSENSOR_INPUT_BACK_KEY  KEY_BACK

#define FPSENSOR_INPUT_FF_KEY  KEY_POWER
#define FPSENSOR_INPUT_CAMERA_KEY  KEY_CAMERA


#define FPSENSOR_INPUT_OTHER_KEY KEY_VOLUMEDOWN  /* temporary key value for capture use */
#endif
#define FPSENSOR_NAV_UP_KEY     19//KEY_UP
#define FPSENSOR_NAV_DOWN_KEY   20//KEY_DOWN
#define FPSENSOR_NAV_LEFT_KEY   21//KEY_LEFT
#define FPSENSOR_NAV_RIGHT_KEY  22//KEY_RIGHT
#define FPSENSOR_NAV_TAP_KEY    23
/***********************GPIO setting port layer*************************/
/* customer hardware port layer, please change according to customer's hardware */
// #ifndef GPIO_FP_INT_PIN
// #define GPIO_FP_INT_PIN            (GPIO86 | 0x80000000)
// #define GPIO_FP_INT_PIN_M_GPIO    GPIO_MODE_00
// #define GPIO_FP_INT_PIN_M_EINT    GPIO_FP_INT_PIN_M_GPIO
// #endif
#define GPIO_PIN_IRQ   86

// #define GPIO_FP_RESET_PIN          (GPIO43 | 0x80000000)
// #define GPIO_FP_RESET_PIN_M_GPIO  GPIO_MODE_00


/*************************************************************/
fpsensor_data_t *g_fpsensor = NULL;
struct mt_spi_t *fpsensor_ms;
EXPORT_SYMBOL(g_fpsensor);
#define ROUND_UP(x, align)        ((x+(align-1))&~(align-1))

/**************************debug******************************/
#define ERR_LOG  (0)
#define INFO_LOG (1)
#define DEBUG_LOG (2)

/* debug log setting */
u8 fpsensor_debug_level = ERR_LOG;

#define fpsensor_debug(level, fmt, args...) do { \
    if (fpsensor_debug_level >= level) {\
        printk( "[fpsensor] " fmt, ##args); \
    } \
} while (0)

#define FUNC_ENTRY()  fpsensor_debug(DEBUG_LOG, "%s, %d, entry\n", __func__, __LINE__)
#define FUNC_EXIT()  fpsensor_debug(DEBUG_LOG, "%s, %d, exit\n", __func__, __LINE__)

/*************************************************************/
//static LIST_HEAD(device_list);
//static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = (150 * 150);

module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

/* -------------------------------------------------------------------- */
/* global variables                         */
/* -------------------------------------------------------------------- */
//static int fpsensor_device_count;
/* -------------------------------------------------------------------- */
/* timer function                                */
/* -------------------------------------------------------------------- */
#define TIME_START       0
#define TIME_STOP       1
/*
   static long int prev_time, cur_time;

   long int kernel_time(unsigned int step)
   {
   cur_time = ktime_to_us(ktime_get());
   if (step == TIME_START) {
   prev_time = cur_time;
   return 0;
   } else if (step == TIME_STOP) {
   fpsensor_debug(DEBUG_LOG, "%s, use: %ld us\n", __func__, (cur_time - prev_time));
   return cur_time - prev_time;
   } else {
   prev_time = cur_time;
   return -1;
   }
   }
 */
//void mt_spi_enable_clk(struct mt_spi_t *ms);
//void mt_spi_disable_clk(struct mt_spi_t *ms);
//dts

//+add by hzb for ontim debug
#include <ontim/ontim_dev_dgb.h>
static char version[]="lc1550_mtk_1.0";
static char vendor_name[20]="KaiEr-LC1550";
    DEV_ATTR_DECLARE(fpsensor)
    DEV_ATTR_DEFINE("version",version)
    DEV_ATTR_DEFINE("vendor",vendor_name)
    DEV_ATTR_DECLARE_END;
    ONTIM_DEBUG_DECLARE_AND_INIT(fpsensor,fpsensor,8);
//-add by hzb for ontim debug

static DEFINE_MUTEX(spidev_set_gpio_mutex);
static void spidev_gpio_as_int(fpsensor_data_t *fpsensor)
{
    fpsensor_trace( "[fpsensor]----%s---\n", __func__);
    mutex_lock(&spidev_set_gpio_mutex);
    printk("[fpsensor]spidev_gpio_as_int\n");
    pinctrl_select_state(fpsensor->pinctrl1, fpsensor->eint_as_int);
    mutex_unlock(&spidev_set_gpio_mutex);
}
void fpsensor_gpio_output_dts(int gpio,int level)
{
    mutex_lock(&spidev_set_gpio_mutex);
    printk("[fpsensor]fpsensor_gpio_output_dts: gpio= %d, level = %d\n",gpio,level);
    if(gpio == FPSENSOR_RST_PIN)
    {
        if (level)
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_rst_high);
        else
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_rst_low);
    }
    else if(gpio == FPSENSOR_SPI_CS_PIN)
    {
        if (level)
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_cs_high);
        else
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_cs_low);
    }
    else if(gpio == FPSENSOR_SPI_MO_PIN)
    {
        if (level)
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_mo_high);
        else
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_mo_low);
    }
    else if(gpio == FPSENSOR_SPI_CK_PIN)
    {
        if (level)
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_ck_high);
        else
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_ck_low);
    }
    else if(gpio == FPSENSOR_SPI_MI_PIN)
    {
        if (level)
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_mi_high);
        else
            pinctrl_select_state(g_fpsensor->pinctrl1, g_fpsensor->fp_mi_low);
    }
    mutex_unlock(&spidev_set_gpio_mutex);
}

static void fpsensor_spi_gpio_cfg(void)
{
/*
    mt_set_gpio_mode(FPSENSOR_SPI_CK_PIN, FPSENSOR_SPI_SCK_PIN_M_SCK);
    mt_set_gpio_mode(FPSENSOR_SPI_MI_PIN, FPSENSOR_SPI_SCK_PIN_M_MISO);
    mt_set_gpio_mode(FPSENSOR_SPI_MO_PIN, FPSENSOR_SPI_SCK_PIN_M_MOSI);
    mt_set_gpio_mode(FPSENSOR_SPI_CS_PIN, FPSENSOR_SPI_SCK_PIN_M_CS);
*/
}

int fpsensor_gpio_wirte(int gpio, int value)
{
    //    gpio_set_value(gpio, value);
    //gpio_direction_output(gpio, value);

    fpsensor_gpio_output_dts(gpio,value);

    return 0;
}
int fpsensor_gpio_read(int gpio)
{
    return gpio_get_value(gpio);
}

int fpsensor_spidev_dts_init(fpsensor_data_t *fpsensor)
{
    struct device_node *node;
    int ret = 0;
    fpsensor_printk( "%s\n", __func__);
    node = of_find_compatible_node(NULL, NULL, "mediatek,mtk_finger");
    if (node) {
        fpsensor->fp_rst_low = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_finger_rst_low");
        if (IS_ERR(fpsensor->fp_rst_low)) {
            ret = PTR_ERR(fpsensor->fp_rst_low);
            fpsensor_error("fpensor Cannot find fp pinctrl fp_rst_low!\n");
            return ret;
        }
        fpsensor->fp_rst_high = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_finger_rst_high");
        if (IS_ERR(fpsensor->fp_rst_high)) {
            ret = PTR_ERR(fpsensor->fp_rst_high);
            fpsensor_error( "fpsensor Cannot find fp pinctrl fp_rst_high!\n");
            return ret;
        }

        fpsensor->eint_as_int = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_eint_in_low"); //eint_in_low; eint
        if (IS_ERR(fpsensor->eint_as_int)) {
            ret = PTR_ERR(fpsensor->eint_as_int);
            fpsensor_error( "fpsensor Cannot find fp pinctrl eint_as_int!\n");
            return ret;
        }
        fpsensor->fp_cs_low = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_spi_cs_low");
        if (IS_ERR(fpsensor->fp_cs_low)) {
            ret = PTR_ERR(fpsensor->fp_cs_low);
            fpsensor_error("fpensor Cannot find fp pinctrl fp_cs_low!\n");
            return ret;
        }
        fpsensor->fp_cs_high = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_spi_cs_high");
        if (IS_ERR(fpsensor->fp_cs_high)) {
            ret = PTR_ERR(fpsensor->fp_cs_high);
            fpsensor_error( "fpsensor Cannot find fp pinctrl fp_cs_high!\n");
            return ret;
        }

        fpsensor->fp_mo_high = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_spi_mo_high");
        if (IS_ERR(fpsensor->fp_mo_high)) {
            ret = PTR_ERR(fpsensor->fp_mo_high);
            fpsensor_error( "fpsensor Cannot find fp pinctrl fp_mo_high!\n");
            return ret;
        }
        fpsensor->fp_mo_low = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_spi_mo_low");
        if (IS_ERR(fpsensor->fp_mo_low)) {
            ret = PTR_ERR(fpsensor->fp_mo_low);
            fpsensor_error("fpensor Cannot find fp pinctrl fp_mo_low!\n");
            return ret;
        }

        fpsensor->fp_mi_high = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_spi_mi_high");
        if (IS_ERR(fpsensor->fp_mi_high)) {
            ret = PTR_ERR(fpsensor->fp_mi_high);
            fpsensor_error( "fpsensor Cannot find fp pinctrl fp_mi_high!\n");
            return ret;
        }
        fpsensor->fp_mi_low = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_spi_mi_low");
        if (IS_ERR(fpsensor->fp_mi_low)) {
            ret = PTR_ERR(fpsensor->fp_mi_low);
            fpsensor_error("fpensor Cannot find fp pinctrl fp_mi_low!\n");
            return ret;
        }

        fpsensor->fp_ck_high = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_spi_mclk_high");
        if (IS_ERR(fpsensor->fp_ck_high)) {
            ret = PTR_ERR(fpsensor->fp_ck_high);
            fpsensor_error( "fpsensor Cannot find fp pinctrl fp_ck_high!\n");
            return ret;
        }
        fpsensor->fp_ck_low = pinctrl_lookup_state(fpsensor->pinctrl1, "fpsensor_spi_mclk_low");
        if (IS_ERR(fpsensor->fp_ck_low)) {
            ret = PTR_ERR(fpsensor->fp_ck_low);
            fpsensor_error("fpensor Cannot find fp pinctrl fp_ck_low!\n");
            return ret;
        }

        fpsensor_gpio_output_dts(FPSENSOR_SPI_MO_PIN, 0);
        fpsensor_gpio_output_dts(FPSENSOR_SPI_MI_PIN, 0);
        fpsensor_gpio_output_dts(FPSENSOR_SPI_CK_PIN, 0);
        fpsensor_gpio_output_dts(FPSENSOR_SPI_CS_PIN, 0);


    }
    else{
        fpsensor_error("fpensor Cannot find node!\n");
    }
    return 0;
}



irqreturn_t fpsensor_interrupt(int irq, void *_fpsensor)
{
        fpsensor_data_t *fpsensor = _fpsensor;
        //fpsensor_trace("*************%s, 0x%x, %d\n", __func__, (int)fpsensor, fpsensor->fpsensor_init_done);
        //wake up cpu for syber
        //wake_lock_timeout(&fpsensor_timeout_wakelock, msecs_to_jiffies(200));

        //if(fpsensor->fpsensor_init_done == 0)
                //return 0;

        if (fpsensor_gpio_read(fpsensor->irq_gpio)) {
                //fpsensor->interrupt_done = true;
                //wake_up_interruptible(&fpsensor->wq_irq_return);
                return 0;
        }
        return 0;
}

/* -------------------------------------------------------------------- */
/* fingerprint chip hardware configuration                                  */
/* -------------------------------------------------------------------- */


static int fpsensor_irq_gpio_cfg(void)
{
    int error = 0;
    //     int ret = 0;
    struct device_node *node;
    fpsensor_data_t *fpsensor;
    u32 ints[2] = {0, 0};
    fpsensor_printk("%s\n", __func__);

    fpsensor = g_fpsensor;

    spidev_gpio_as_int(fpsensor);

    node = of_find_compatible_node(NULL, NULL, "mediatek,mtk_finger");
    if ( node)
    {
        of_property_read_u32_array( node, "debounce", ints, ARRAY_SIZE(ints));
        gpio_request(ints[0], "fpsensor-irq");  
        gpio_set_debounce(ints[0], ints[1]);
        fpsensor_printk("[fpsensor]ints[0] = %d,is irq_gpio , ints[1] = %d!!\n", ints[0], ints[1]);
        fpsensor->irq_gpio = ints[0];
        fpsensor->irq = irq_of_parse_and_map(node, 0);  // get irq number
        if (!fpsensor->irq)
        {
            printk("fpsensor irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }
        fpsensor_printk(" [fpsensor]fpsensor->irq= %d,fpsensor>irq_gpio = %d\n", fpsensor->irq,fpsensor->irq_gpio);
    }
    else
    {
        printk("fpsensor null irq node!!\n");
        return -EINVAL;
    }
    /*
    error = request_irq(fpsensor->irq, fpsensor_interrupt,
            IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND, FPSENSOR_DEV_NAME, fpsensor);

    if (error) {
        dev_err(&fpsensor->spi->dev,
                "request_irq %i failed.\n",
                fpsensor->irq);
        fpsensor->irq = -EINVAL;
    }
*/
    //fpsensor_irq_disable(fpsensor);       
    return error;

}



/* delay ms after reset */
static void fpsensor_hw_reset(int delay)
{
    FUNC_ENTRY();

    //mt_set_gpio_out(GPIO_FP_RESET_PIN, GPIO_OUT_ONE);
    //fpsensor_gpio_wirte(g_fpsensor->reset_gpio, 1);
    fpsensor_gpio_output_dts(FPSENSOR_RST_PIN,    1);
    udelay(100);
    //mt_set_gpio_out(GPIO_FP_RESET_PIN, GPIO_OUT_ZERO);
    //fpsensor_gpio_wirte(g_fpsensor->reset_gpio, 0);
    fpsensor_gpio_output_dts(FPSENSOR_RST_PIN,  0);
    //mdelay(5);
    udelay(1000);
    //mt_set_gpio_out(GPIO_FP_RESET_PIN, GPIO_OUT_ONE);
    //fpsensor_gpio_wirte(g_fpsensor->reset_gpio, 1);
    fpsensor_gpio_output_dts(FPSENSOR_RST_PIN,  1);
    if (delay) {
        /* delay is configurable */
        //mdelay(delay);
        udelay(delay);
    }

    FUNC_EXIT();
}
//extern void finger_set_spi_clk(u8 bonoff);
static void fpsensor_spi_clk_enable(u8 bonoff)
{
#if 0
    //finger_set_spi_clk(bonoff);

    static int count = 0;
    if (bonoff && (count == 0))
    {
        enable_clock(MT_CG_PERI_SPI0, "spi");
        count = 1;
    } else if ((count > 0) && (bonoff == 0)) {
        disable_clock(MT_CG_PERI_SPI0, "spi");
        count = 0;
    }   
#endif
#if 1
    if(bonoff == 0)
        	mt_spi_disable_master_clk(g_fpsensor->spi);
    else
			mt_spi_enable_master_clk(g_fpsensor->spi);
#endif
}
static void fpsensor_hw_power_enable(u8 onoff)
{
    #if 0
    static int enable = 1;
    if (onoff && enable)
    {
        //pinctrl_select_state(g_fpsensor->pinctrl_gpios, g_fpsensor->pins_power_on);
        enable = 0;
    }
    else if (!onoff && !enable)
    {
        //pinctrl_select_state(g_fpsensor->pinctrl_gpios, g_fpsensor->pins_power_off);
        enable = 1;
    }
    #endif
}
/*
   extern int g_irq_num;
   static void fpsensor_irq_gpio_cfg(void)
   {

   if (g_irq_num == 0)
   {
   fpsensor_debug(ERR_LOG, "irq num is 0\n");
   }
   fpsensor_debug(ERR_LOG, "irq num is %d \n", g_irq_num);
   g_fpsensor->spi->irq = g_irq_num;
   g_fpsensor->irq = g_irq_num;

   }
 */
static void fpsensor_enable_irq(fpsensor_data_t *fpsensor_dev)
{
    FUNC_ENTRY();
#if 1 
    setRcvIRQ(0);
    if (0 == fpsensor_dev->device_available) {
        fpsensor_debug(ERR_LOG, "%s, devices not available\n", __func__);
    } else {
        if (1 == fpsensor_dev->irq_count) {
            fpsensor_debug(ERR_LOG, "%s, irq already enabled\n", __func__);
        } else {
            enable_irq(fpsensor_dev->irq);
            fpsensor_dev->irq_count = 1;
            fpsensor_debug(INFO_LOG, "%s enable interrupt!\n", __func__);
        }
    }
#endif
    FUNC_EXIT();
}

static void fpsensor_disable_irq(fpsensor_data_t *fpsensor_dev)
{
    FUNC_ENTRY();
#if 1
    if (0 == fpsensor_dev->device_available) {
        fpsensor_debug(ERR_LOG, "%s, devices not available\n", __func__);
    } else {
        if (0 == fpsensor_dev->irq_count) {
            fpsensor_debug(ERR_LOG, "%s, irq already disabled\n", __func__);
        } else {
            disable_irq(fpsensor_dev->irq);
            fpsensor_dev->irq_count = 0;
            fpsensor_debug(DEBUG_LOG, "%s disable interrupt!\n", __func__);
        }
    }
    setRcvIRQ(0);
#endif
    FUNC_EXIT();
}
/* -------------------------------------------------------------------- */
/* sysfs                 */
/* -------------------------------------------------------------------- */
#if FPSENSOR_SYSFS
static ssize_t irq_ack(struct device *device,
        struct device_attribute *attribute,
        const char *buffer, size_t count)
{
    FUNC_ENTRY();
    FUNC_EXIT();
    return count;
}

static DEVICE_ATTR(irq,S_IWUSR, NULL, irq_ack);


static struct attribute *attributes[] = {
    &dev_attr_irq.attr,
    NULL
};
static const struct attribute_group attribute_group = {
    .attrs = attributes,
};
#endif
/* -------------------------------------------------------------------- */
/* netlink functions                 */
/* -------------------------------------------------------------------- */
#if FPSENSOR_NETLINK 

void fpsensor_netlink_send(const int command)
{
    struct nlmsghdr *nlh;
    struct sk_buff *skb;
    int ret;
    // char *msg = "Hello from kernel";
    // int msg_size;
    fpsensor_debug(INFO_LOG, "[%s] : enter, send command %d\n", __func__, command);
    if (NULL == g_fpsensor->nl_sk) {
        fpsensor_debug(ERR_LOG, "[%s] : invalid socket\n", __func__);
        return;
    }

    if (0 == g_fpsensor->pid) {
        fpsensor_debug(ERR_LOG, "[%s] : invalid native process pid\n", __func__);
        return;
    }

    /*alloc data buffer for sending to native*/
    /*malloc data space at least 1500 bytes, which is ethernet data length*/
    skb = alloc_skb(NLMSG_SPACE(MAX_NL_MSG_LEN), GFP_ATOMIC);
    if (skb == NULL) {
        fpsensor_debug(ERR_LOG, "[%s] : allocate skb failed\n", __func__);
        return;
    }

    nlh = nlmsg_put(skb, 0, 0, 0, MAX_NL_MSG_LEN, 0);

    NETLINK_CB(skb).portid = 0;
    NETLINK_CB(skb).dst_group = 0;

    *(char *)NLMSG_DATA(nlh) = command;
    ret = netlink_unicast(g_fpsensor->nl_sk, skb, g_fpsensor->pid, MSG_DONTWAIT);
    // msg_size = strlen(msg);
    // skb = nlmsg_new(msg_size,0);
    // if (!skb)
    // {
    //     fpsensor_debug(ERR_LOG,"netlink get skb erro\n");
    //     return ;
    // }
    // nlh=nlmsg_put(skb,0,0,NLMSG_DONE,msg_size,0);
    // NETLINK_CB(skb).dst_group = 0;
    // strncpy(nlmsg_data(nlh),msg,msg_size);
    // ret=nlmsg_unicast(g_fpsensor->nl_sk,skb,993);

    if (ret == 0) {
        fpsensor_debug(ERR_LOG, "[%s] : send failed\n", __func__);
        return;
    } else {
        fpsensor_debug(INFO_LOG, "[%s] : send done, data length is %d\n", __func__, ret);
        return;
    }
}

static void fpsensor_netlink_recv(struct sk_buff *__skb)
{
    struct sk_buff *skb;
    struct nlmsghdr *nlh;
    char str[128];

    fpsensor_debug(INFO_LOG, "[%s] : enter\n", __func__);

    skb = skb_get(__skb);
    if (skb == NULL) {
        fpsensor_debug(ERR_LOG, "[%s] : skb_get return NULL\n", __func__);
        return;
    }

    /*presume there is 5byte payload at leaset*/
    if (skb->len >= NLMSG_SPACE(0)) {
        nlh = nlmsg_hdr(skb);
        memcpy(str, NLMSG_DATA(nlh), sizeof(str));
        g_fpsensor->pid = nlh->nlmsg_pid;
        fpsensor_debug(INFO_LOG, "[%s] : pid: %d, msg: %d\n", __func__, g_fpsensor->pid, *str);

    } else {
        fpsensor_debug(ERR_LOG, "[%s] : not enough data length\n", __func__);
    }

    kfree_skb(skb);
    return;
}
static int fpsensor_netlink_init(void)
{
    struct netlink_kernel_cfg cfg;
    memset(&cfg, 0, sizeof(struct netlink_kernel_cfg));
    cfg.input = fpsensor_netlink_recv;

    g_fpsensor->nl_sk = netlink_kernel_create(&init_net, NETLINK_FP, &cfg);
    if (g_fpsensor->nl_sk == NULL) {
        fpsensor_debug(ERR_LOG, "[%s] : netlink create failed\n", __func__);
        return -1;
    } else {
        fpsensor_debug(INFO_LOG, "[%s] : netlink create success\n", __func__);
        return 0;
    }
}
static int fpsensor_netlink_destory(void)
{
    if (g_fpsensor->nl_sk != NULL) {
        /* sock_release(g_fpsensor->nl_sk->sk_socket); */
        netlink_kernel_release(g_fpsensor->nl_sk);
        return 0;
    } else {
        fpsensor_debug(ERR_LOG, "[%s] : no netlink socket yet\n", __func__);
        return -1;
    }
}

#endif

/* -------------------------------------------------------------------- */
/* REE SPI read /write                                                                                       */
/* -------------------------------------------------------------------- */
/* fpsensor_spi_setup_conf_ree, configure spi speed and transfer mode in REE mode
 *
 * speed: 1, 4, 6, 8 unit:MHz
 * mode: DMA mode or FIFO mode
 */
#if SUPPORT_REE_SPI
void fpsensor_spi_setup_conf_ree(u32 speed, enum spi_transfer_mode mode)
{
    struct mt_chip_conf *mcc = &g_fpsensor->spi_mcc;

    switch (speed) {
        case 1:
            /* set to 1MHz clock */
            mcc->high_time = 50;
            mcc->low_time = 50;
            break;
        case 4:
            /* set to 4MHz clock */
            mcc->high_time = 15;
            mcc->low_time = 15;
            break;
        case 6:
            /* set to 6MHz clock */
            mcc->high_time = 10;
            mcc->low_time = 10;
            break;
        case 8:
            /* set to 8MHz clock */
            mcc->high_time = 8;
            mcc->low_time = 8;
            break;
        default:
            /* default set to 1MHz clock */
            mcc->high_time = 50;
            mcc->low_time = 50;
    }

    if ((mode == DMA_TRANSFER) || (mode == FIFO_TRANSFER)) {
        mcc->com_mod = mode;
    } else {
        /* default set to FIFO mode */
        mcc->com_mod = FIFO_TRANSFER;
    }

    if (spi_setup(g_fpsensor->spi))
        fpsensor_debug(ERR_LOG, "%s, failed to setup spi conf\n", __func__);
}

int fpsensor_spi_read_bytes_ree(u16 addr, u32 data_len, u8 *rx_buf)
{
    struct spi_message msg;
    struct spi_transfer *xfer;
    u8 *tmp_buf;
    u32 package, reminder, retry;

    package = (data_len + 2) / 1024;
    reminder = (data_len + 2) % 1024;

    if ((package > 0) && (reminder != 0)) {
        xfer = kzalloc(sizeof(*xfer) * 4, GFP_KERNEL);
        retry = 1;
    } else {
        xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
        retry = 0;
    }
    if (xfer == NULL) {
        fpsensor_debug(ERR_LOG, "%s, no memory for SPI transfer\n", __func__);
        return -ENOMEM;
    }

    tmp_buf = g_fpsensor->spi_buffer;

    /* switch to DMA mode if transfer length larger than 32 bytes */
    if ((data_len + 1) > 32) {
        g_fpsensor->spi_mcc.com_mod = DMA_TRANSFER;
        spi_setup(g_fpsensor->spi);
    }
    spi_message_init(&msg);
    *tmp_buf = 0xF0;
    *(tmp_buf + 1) = (u8)((addr >> 8) & 0xFF);
    *(tmp_buf + 2) = (u8)(addr & 0xFF);
    xfer[0].tx_buf = tmp_buf;
    xfer[0].len = 3;
    xfer[0].delay_usecs = 5;
    spi_message_add_tail(&xfer[0], &msg);
    spi_sync(g_fpsensor->spi, &msg);

    spi_message_init(&msg);
    /* memset((tmp_buf + 4), 0x00, data_len + 1); */
    /* 4 bytes align */
    *(tmp_buf + 4) = 0xF1;
    xfer[1].tx_buf = tmp_buf + 4;
    xfer[1].rx_buf = tmp_buf + 4;

    if (retry)
        xfer[1].len = package * 1024;
    else
        xfer[1].len = data_len + 1;

    xfer[1].delay_usecs = 5;
    spi_message_add_tail(&xfer[1], &msg);
    spi_sync(g_fpsensor->spi, &msg);

    /* copy received data */
    if (retry)
        memcpy(rx_buf, (tmp_buf + 5), (package * 1024 - 1));
    else
        memcpy(rx_buf, (tmp_buf + 5), data_len);

    /* send reminder SPI data */
    if (retry) {
        addr = addr + package * 1024 - 2;
        spi_message_init(&msg);

        *tmp_buf = 0xF0;
        *(tmp_buf + 1) = (u8)((addr >> 8) & 0xFF);
        *(tmp_buf + 2) = (u8)(addr & 0xFF);
        xfer[2].tx_buf = tmp_buf;
        xfer[2].len = 3;
        xfer[2].delay_usecs = 5;
        spi_message_add_tail(&xfer[2], &msg);
        spi_sync(g_fpsensor->spi, &msg);

        spi_message_init(&msg);
        *(tmp_buf + 4) = 0xF1;
        xfer[3].tx_buf = tmp_buf + 4;
        xfer[3].rx_buf = tmp_buf + 4;
        xfer[3].len = reminder + 1;
        xfer[3].delay_usecs = 5;
        spi_message_add_tail(&xfer[3], &msg);
        spi_sync(g_fpsensor->spi, &msg);

        memcpy((rx_buf + package * 1024 - 1), (tmp_buf + 6), (reminder - 1));
    }

    /* restore to FIFO mode if has used DMA */
    if ((data_len + 1) > 32) {
        g_fpsensor->spi_mcc.com_mod = FIFO_TRANSFER;
        spi_setup(g_fpsensor->spi);
    }
    kfree(xfer);
    if (xfer != NULL)
        xfer = NULL;

    return 0;
}
int fpsensor_spi_read_byte_ree(u16 addr, u16 *value)
{
    struct spi_message msg;
    struct spi_transfer *xfer;

    xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
    if (xfer == NULL) {
        fpsensor_debug(ERR_LOG, "%s, no memory for SPI transfer\n", __func__);
        return -ENOMEM;
    }

    spi_message_init(&msg);
    *g_fpsensor->spi_buffer = 0x00;
    *(g_fpsensor->spi_buffer + 1) = (u8)((addr >> 8) & 0xFF);
    *(g_fpsensor->spi_buffer + 2) = (u8)(addr & 0xFF);

    xfer[0].tx_buf = g_fpsensor->spi_buffer;
    xfer[0].rx_buf = g_fpsensor->spi_buffer + 12;
    xfer[0].len = 3;
    xfer[0].delay_usecs = 5;
    spi_message_add_tail(&xfer[0], &msg);
    spi_sync(g_fpsensor->spi, &msg);

    spi_message_init(&msg);
    /* 4 bytes align */
    *(g_fpsensor->spi_buffer + 4) = 0xF1;
    xfer[1].tx_buf = g_fpsensor->spi_buffer + 4;
    xfer[1].rx_buf = g_fpsensor->spi_buffer + 4;
    xfer[1].len = 2;
    xfer[1].delay_usecs = 5;
    spi_message_add_tail(&xfer[1], &msg);
    spi_sync(g_fpsensor->spi, &msg);

    *value = *(u16*)(g_fpsensor->spi_buffer + 12);

    kfree(xfer);
    if (xfer != NULL)
        xfer = NULL;

    return 0;
}
#endif
/* -------------------------------------------------------------------- */
/* file operation function                                                                                       */
/* -------------------------------------------------------------------- */

static ssize_t fpsensor_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    fpsensor_debug(ERR_LOG, "Not support read opertion in TEE version\n");
    return -EFAULT;
}

static ssize_t fpsensor_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *f_pos)
{
    fpsensor_debug(ERR_LOG, "Not support write opertion in TEE version\n");
    return -EFAULT;
}

static irqreturn_t fpsensor_irq(int irq, void *handle)
    // void fpsensor_irq(void)
{
    fpsensor_data_t *fpsensor_dev = (fpsensor_data_t *)handle;
    // fpsensor_data_t *fpsensor_dev = g_fpsensor;

    FUNC_ENTRY();
#if FPSENSOR_SYSFS
    sysfs_notify(&fpsensor_dev->device->kobj, NULL, dev_attr_irq.attr.name);
#endif
#if FPSENSOR_NETLINK
    fpsensor_netlink_send(FPSENSOR_NETLINK_IRQ);
#endif
#if FPSENSOR_IOCTL
    setRcvIRQ(1);
#endif
    wake_up_interruptible(&fpsensor_dev->wq_irq_return);
    fpsensor_dev->sig_count++;

    FUNC_EXIT();
    return IRQ_WAKE_THREAD;
}
void setRcvIRQ( int  val )
{
    fpsensor_data_t *fpsensor_dev = g_fpsensor;
    // fpsensor_debug(INFO_LOG, "[rickon]: %s befor val :  %d ; set val : %d   \n", __func__, fpsensor_dev-> RcvIRQ, val);
    fpsensor_dev-> RcvIRQ = val;
}

//#include <linux/dev_info.h>
//static struct devinfo_struct *s_DEVINFO_fpsensor;
static long fpsensor_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    fpsensor_data_t *fpsensor_dev = NULL;
    struct fpsensor_key fpsensor_key;
    uint32_t key_event;
    int retval = 0;
    unsigned int val = 0;

    FUNC_ENTRY();
    fpsensor_debug(INFO_LOG, "[rickon]: 2222 fpsensor ioctl cmd : 0x%x \n", cmd );
    fpsensor_dev = (fpsensor_data_t *)filp->private_data;
    //clear cancel flag
    fpsensor_dev->cancel = 0 ;
    switch (cmd) {
        case FPSENSOR_IOC_INIT:
            fpsensor_debug(INFO_LOG, "%s: fpsensor init started======\n", __func__);
            //fpsensor_irq_gpio_cfg();
#if 0
            mt_eint_registration(fpsensor_dev->spi->irq, EINTF_TRIGGER_RISING, fpsensor_irq, 1);        // 0:auto mask is no
#endif
            retval = request_threaded_irq(fpsensor_dev->irq, fpsensor_irq, NULL,
                    IRQF_TRIGGER_RISING | IRQF_ONESHOT,dev_name(&(fpsensor_dev->spi->dev)), fpsensor_dev);

            if (retval == 0)
            {
                fpsensor_debug(ERR_LOG, " irq thread reqquest success!\n");
            }
            else
            {
                fpsensor_debug(ERR_LOG, " irq thread request failed , retval =%d \n",retval);
            }
            fpsensor_dev->device_available = 1;
            fpsensor_dev->irq_count = 1;
            fpsensor_disable_irq(fpsensor_dev);
            // mt_eint_unmask(fpsensor_dev->spi->irq);

            fpsensor_dev->sig_count = 0;
            /*
               s_DEVINFO_fpsensor =(struct devinfo_struct*) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);
               s_DEVINFO_fpsensor->device_type = "FP";
               s_DEVINFO_fpsensor->device_module = "oflim";
               s_DEVINFO_fpsensor->device_vendor = "NULL";
               s_DEVINFO_fpsensor->device_ic = "LC1550";
               s_DEVINFO_fpsensor->device_version = "NULL";
               s_DEVINFO_fpsensor->device_info = "fingerprint";
               s_DEVINFO_fpsensor->device_used = DEVINFO_USED;
               devinfo_check_add_device(s_DEVINFO_fpsensor);
             */
            fpsensor_debug(INFO_LOG, "%s: fpsensor init finished======\n", __func__);
            break;

        case FPSENSOR_IOC_EXIT:
            fpsensor_disable_irq(fpsensor_dev);
            if (fpsensor_dev->spi->irq) {
                free_irq(fpsensor_dev->spi->irq, fpsensor_dev);
                fpsensor_dev->irq_count = 0;
            }
            fpsensor_dev->device_available = 0;
            fpsensor_debug(INFO_LOG, "%s: fpsensor exit finished======\n", __func__);
            break;

        case FPSENSOR_IOC_RESET:
            fpsensor_debug(INFO_LOG, "%s: chip reset command\n", __func__);
            fpsensor_hw_reset(1250);
            break;

        case FPSENSOR_IOC_ENABLE_IRQ:
            fpsensor_debug(INFO_LOG, "%s: chip ENable IRQ command\n", __func__);
            fpsensor_enable_irq(fpsensor_dev);
            break;

        case FPSENSOR_IOC_DISABLE_IRQ:
            fpsensor_debug(INFO_LOG, "%s: chip disable IRQ command\n", __func__);
            fpsensor_disable_irq(fpsensor_dev);
            break;
        case FPSENSOR_IOC_GET_INT_VAL:
            // retval = mt_get_gpio_in(GPIO_FP_INT_PIN);
            // val = fpsensor_dev -> RcvIRQ ;

            fpsensor_debug(INFO_LOG, "g_fpsensor->irq: %d\n", g_fpsensor->irq);

            //val = mt_get_gpio_in(GPIO_FP_INT_PIN);//gpio_get_value(GPIO_PIN_IRQ);
            val = fpsensor_gpio_read(g_fpsensor->irq_gpio);
            if (copy_to_user((void __user *)arg, (void *)&val, sizeof(unsigned int))) {
                fpsensor_debug(ERR_LOG, "Failed to copy data to user\n");
                retval = -EFAULT;
                break;
            }
            retval = 0;
            break;
        case FPSENSOR_IOC_ENABLE_SPI_CLK:
            fpsensor_debug(INFO_LOG, "%s: ENABLE_SPI_CLK ======\n", __func__);
            fpsensor_spi_clk_enable(1);
            break;
        case FPSENSOR_IOC_DISABLE_SPI_CLK:
            fpsensor_debug(INFO_LOG, "%s: DISABLE_SPI_CLK ======\n", __func__);
            fpsensor_spi_clk_enable(0);
            break;

        case FPSENSOR_IOC_ENABLE_POWER:
            fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_ENABLE_POWER ======\n", __func__);
            fpsensor_hw_power_enable(1);
            break;

        case FPSENSOR_IOC_DISABLE_POWER:
            fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_DISABLE_POWER ======\n", __func__);
            fpsensor_hw_power_enable(0);
            break;


        case FPSENSOR_IOC_INPUT_KEY_EVENT:
            if (copy_from_user(&fpsensor_key, (struct fpsensor_key *)arg, sizeof(struct fpsensor_key))) {
                fpsensor_debug(ERR_LOG, "Failed to copy input key event from user to kernel\n");
                retval = -EFAULT;
                break;
            }
            if (FPSENSOR_KEY_HOME == fpsensor_key.key) {
                key_event = FPSENSOR_INPUT_HOME_KEY;
            } else if (FPSENSOR_KEY_POWER == fpsensor_key.key) {
                key_event = FPSENSOR_INPUT_FF_KEY;
            } else if (FPSENSOR_KEY_CAPTURE == fpsensor_key.key) {
                key_event = FPSENSOR_INPUT_CAMERA_KEY;
            } else {
                /* add special key define */
                key_event = FPSENSOR_INPUT_OTHER_KEY;
            }
            fpsensor_debug(INFO_LOG, "%s: received key event[%d], key=%d, value=%d\n",
                    __func__, key_event, fpsensor_key.key, fpsensor_key.value);
            if ((FPSENSOR_KEY_POWER == fpsensor_key.key || FPSENSOR_KEY_CAPTURE == fpsensor_key.key) && (fpsensor_key.value == 1)) {
                input_report_key(fpsensor_dev->input, key_event, 1);
                input_sync(fpsensor_dev->input);
                input_report_key(fpsensor_dev->input, key_event, 0);
                input_sync(fpsensor_dev->input);
            } else if (FPSENSOR_KEY_UP == fpsensor_key.key) {
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_UP_KEY, 1);
                input_sync(fpsensor_dev->input);
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_UP_KEY, 0);
                input_sync(fpsensor_dev->input);
            } else if (FPSENSOR_KEY_DOWN == fpsensor_key.key) {
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_DOWN_KEY, 1);
                input_sync(fpsensor_dev->input);
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_DOWN_KEY, 0);
                input_sync(fpsensor_dev->input);
            } else if (FPSENSOR_KEY_RIGHT == fpsensor_key.key) {
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_RIGHT_KEY, 1);
                input_sync(fpsensor_dev->input);
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_RIGHT_KEY, 0);
                input_sync(fpsensor_dev->input);
            } else if (FPSENSOR_KEY_LEFT == fpsensor_key.key) {
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_LEFT_KEY, 1);
                input_sync(fpsensor_dev->input);
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_LEFT_KEY, 0);
                input_sync(fpsensor_dev->input);
            }else  if(FPSENSOR_KEY_TAP == fpsensor_key.key){
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_TAP_KEY, 1);
                input_sync(fpsensor_dev->input);
                input_report_key(fpsensor_dev->input, FPSENSOR_NAV_TAP_KEY, 0);
                input_sync(fpsensor_dev->input);
            } else if ((FPSENSOR_KEY_POWER != fpsensor_key.key) && (FPSENSOR_KEY_CAPTURE != fpsensor_key.key)) {
                input_report_key(fpsensor_dev->input, key_event, fpsensor_key.value);
                input_sync(fpsensor_dev->input);
            }
            break;

        case FPSENSOR_IOC_ENTER_SLEEP_MODE:
            fpsensor_dev->is_sleep_mode = 1;
            break;
        case FPSENSOR_IOC_REMOVE:

#if FPSENSOR_NETLINK 
            fpsensor_netlink_destory();
#endif
            if (fpsensor_dev->input != NULL)
            {
                input_unregister_device(fpsensor_dev->input);
                //input_free_device(fpsensor_dev->input);
            }
            // fpsensor_dev->spi = NULL;
            // spi_set_drvdata(fpsensor_dev->spi, NULL);

            /* prevent new opens */
            //mutex_lock(&device_list_lock);
            // sysfs_remove_group(&spi->dev.kobj, &fpsensor_debug_attr_group);
            //list_del(&fpsensor_dev->device_entry);
            // if (fpsensor_dev->device != NULL)
            // {
            //    device_destroy(fpsensor_dev->class, fpsensor_dev->devno);
            // }
            // if (fpsensor_dev->class != NULL )
            // {
            //    unregister_chrdev_region(fpsensor_dev->devno, 1);
            //    class_destroy(fpsensor_dev->class);
            // }
            // if (fpsensor_dev->users == 0) {
            // if (fpsensor_dev->input != NULL)
            // input_unregister_device(fpsensor_dev->input);

            // if (fpsensor_dev->buffer != NULL)
            // kfree(fpsensor_dev->buffer);

            // kfree(fpsensor_dev);
            // }
            //mutex_unlock(&device_list_lock);
            /*
               if (s_DEVINFO_fpsensor != NULL)
               {
               s_DEVINFO_fpsensor->device_type = "FP";
               s_DEVINFO_fpsensor->device_module = "oflim";
               s_DEVINFO_fpsensor->device_vendor = "NULL";
               s_DEVINFO_fpsensor->device_ic = "LC1550";
               s_DEVINFO_fpsensor->device_version = "NULL";
               s_DEVINFO_fpsensor->device_info = "fingerprint";
               s_DEVINFO_fpsensor->device_used = DEVINFO_UNUSED;
               devinfo_check_add_device(s_DEVINFO_fpsensor);
               }
             */
            // fpsensor_hw_power_enable(0);
            // fpsensor_spi_clk_enable(0);
            fpsensor_debug(INFO_LOG, "%s remove finished\n", __func__);
            break;

        case FPSENSOR_IOC_CANCEL_WAIT:
            fpsensor_debug(INFO_LOG, "%s: FPSENSOR CANCEL WAIT\n", __func__);
            wake_up_interruptible(&fpsensor_dev->wq_irq_return);
            fpsensor_dev->cancel = 1;
            break;
        default:
            fpsensor_debug(ERR_LOG, "fpsensor doesn't support this command(%d)\n", cmd);
            break;
    }

    FUNC_EXIT();
    return retval;
}
static long fpsensor_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return fpsensor_ioctl(filp, cmd, (unsigned long)(arg));
}

static unsigned int fpsensor_poll(struct file *filp, struct poll_table_struct *wait)
{
    unsigned int ret = 0;
    fpsensor_debug(ERR_LOG, " support poll opertion  in   version\n");
    ret |= POLLIN;
    poll_wait(filp,&g_fpsensor->wq_irq_return,wait); 
    if (g_fpsensor->cancel == 1 )
    {
        fpsensor_debug(ERR_LOG, " cancle\n");
        ret =  POLLERR;
        g_fpsensor->cancel = 0;
        return ret;
    }
    if ( g_fpsensor->RcvIRQ)
    {
        fpsensor_debug(ERR_LOG, " get irq\n");
        ret |= POLLRDNORM;
    }
    else
    {
        ret = 0;
    }
    return ret;
}


/* -------------------------------------------------------------------- */
/* device function                                  */
/* -------------------------------------------------------------------- */
static int fpsensor_open(struct inode *inode, struct file *filp)
{
    fpsensor_data_t *fpsensor_dev;
    // int status = -ENXIO;

    FUNC_ENTRY();
    fpsensor_dev = container_of(inode->i_cdev, fpsensor_data_t, cdev);
    fpsensor_dev->users++;
    filp->private_data = fpsensor_dev;
    FUNC_EXIT();
    return 0;
}



static int fpsensor_release(struct inode *inode, struct file *filp)
{
    fpsensor_data_t *fpsensor_dev;
    int    status = 0;

    FUNC_ENTRY();
    //mutex_lock(&device_list_lock);
    fpsensor_dev = filp->private_data;
    filp->private_data = NULL;

    /*last close??*/
    fpsensor_dev->users--;
    if (!fpsensor_dev->users) {
        fpsensor_debug(INFO_LOG, "%s, disble_irq. irq = %d\n", __func__, fpsensor_dev->spi->irq);
        fpsensor_disable_irq(fpsensor_dev);
    }
    //mutex_unlock(&device_list_lock);
    fpsensor_dev->device_available = 0;
    FUNC_EXIT();
    return status;
}



static const struct file_operations fpsensor_fops = {
    .owner =    THIS_MODULE,

    .write =    fpsensor_write,
    .read =        fpsensor_read,
    .unlocked_ioctl = fpsensor_ioctl,
#ifdef CONFIG_COMPAT    
    .compat_ioctl   = fpsensor_compat_ioctl,
#endif    
    .open =        fpsensor_open,
    .release =    fpsensor_release,
    .poll    = fpsensor_poll,

};

/* -------------------------------------------------------------------- */
static int fpsensor_create_class(fpsensor_data_t *fpsensor)
{
    int error = 0;

    fpsensor->class = class_create(THIS_MODULE, FPSENSOR_CLASS_NAME);
    if (IS_ERR(fpsensor->class)) {
        fpsensor_debug(ERR_LOG, "%s, Failed to create class.\n", __func__);
        error = PTR_ERR(fpsensor->class);
    }

    return error;
}



/* -------------------------------------------------------------------- */
static int fpsensor_create_device(fpsensor_data_t *fpsensor)
{
    int error = 0;


    if (FPSENSOR_MAJOR > 0) {
        // fpsensor->devno = MKDEV(FPSENSOR_MAJOR, fpsensor_device_count++);

        // error = register_chrdev_region(fpsensor->devno,
        //                 1,
        //                 FPSENSOR_DEV_NAME);
    } else {
        error = alloc_chrdev_region(&fpsensor->devno,
                fpsensor->device_count++,
                1,
                FPSENSOR_DEV_NAME);
    }

    if (error < 0) {
        fpsensor_debug(ERR_LOG,
                "%s: FAILED %d.\n", __func__, error);
        goto out;

    } else {
        fpsensor_debug(INFO_LOG, "%s: major=%d, minor=%d\n",
                __func__,
                MAJOR(fpsensor->devno),
                MINOR(fpsensor->devno));
    }

    fpsensor->device = device_create(fpsensor->class, &(fpsensor->spi->dev), fpsensor->devno,
            fpsensor, FPSENSOR_DEV_NAME);

    if (IS_ERR(fpsensor->device)) {
        fpsensor_debug(ERR_LOG, "device_create failed.\n");
        error = PTR_ERR(fpsensor->device);
    }
out:
    return error;
}
#if SUPPORT_REE_SPI
const struct mt_chip_conf spi_ctrdata = {
    .setuptime = 10,
    .holdtime = 10,
    .high_time = 50, /* 1MHz */
    .low_time = 50,
    .cs_idletime = 10,
    .ulthgh_thrsh = 0,

    .cpol = SPI_CPOL_0,
    .cpha = SPI_CPHA_0,

    .rx_mlsb = SPI_MSB,
    .tx_mlsb = SPI_MSB,

    .tx_endian = SPI_LENDIAN,
    .rx_endian = SPI_LENDIAN,

    .com_mod = FIFO_TRANSFER,
    /* .com_mod = DMA_TRANSFER, */

    .pause = 0,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 0,
};
#endif
/*-------------------------------------------------------------------------*/

static int fpsensor_probe(struct spi_device *spi)
{
    struct device *dev = &spi->dev;
    fpsensor_data_t *fpsensor_dev = NULL;
    int error = 0;
    // u16 i = 0;
    // unsigned long minor;f
    int status = -EINVAL;
//+add by hzb for ontim debug
        if(CHECK_THIS_DEV_DEBUG_AREADY_EXIT()==0)
        {
           return -EIO;
        }
//-add by hzb for ontim debug
    FUNC_ENTRY();
    

    /* Allocate driver data */
    fpsensor_dev = kzalloc(sizeof(*fpsensor_dev), GFP_KERNEL);
    if (!fpsensor_dev) {
        fpsensor_debug(ERR_LOG, "%s, Failed to alloc memory for fpsensor device.\n", __func__);
        FUNC_EXIT();
        return -ENOMEM;
    }
    fpsensor_dev->device = dev ;
#if SUPPORT_REE_SPI
    /* allocate buffer for SPI transfer */
    fpsensor_dev->spi_buffer = kzalloc(20, GFP_KERNEL);
    if (fpsensor_dev->spi_buffer == NULL) {
        status = -ENOMEM;
        goto err3;
    }
#endif
    g_fpsensor = fpsensor_dev;
    /* Initialize the driver data */
    mutex_init(&fpsensor_dev->buf_lock);
#if SUPPORT_REE_SPI
    /* setup SPI parameters */
    /* CPOL=CPHA=0, speed 1MHz */
    fpsensor_dev->spi->mode            = SPI_MODE_0;
    fpsensor_dev->spi->bits_per_word   = 8;
    fpsensor_dev->spi->max_speed_hz    = 1 * 1000 * 1000;
    memcpy(&fpsensor_dev->spi_mcc, &spi_ctrdata, sizeof(struct mt_chip_conf));
    fpsensor_dev->spi->controller_data = (void *)&fpsensor_dev->spi_mcc;

    spi_setup(fpsensor_dev->spi);
#endif
    spi_set_drvdata(spi, fpsensor_dev);
    fpsensor_dev->spi = spi;
    fpsensor_ms = spi_master_get_devdata(spi->master);


    // INIT_LIST_HEAD(&fpsensor_dev->device_entry);
    fpsensor_dev->device_available = 0;
    fpsensor_dev->spi->irq = 0;
    fpsensor_dev->probe_finish = 0;
    fpsensor_dev->device_count     = 0;
    fpsensor_dev->users = 0;
    /*setup fpsensor configurations.*/
    fpsensor_debug(INFO_LOG, "%s, Setting fpsensor device configuration.\n", __func__);
     //fpsensor_reset_gpio_cfg();
    fpsensor_spi_gpio_cfg();
    /* gpio function setting */ 

    //fpsensor_reset_gpio_cfg();

    //fpsensor_get_gpio_dts_info();

    // dts read
    spi->dev.of_node=of_find_compatible_node(NULL, NULL, "mediatek,mtk_finger");

    fpsensor_dev->pinctrl1 = devm_pinctrl_get(&spi->dev);
    if (IS_ERR(fpsensor_dev->pinctrl1)) {
        error = PTR_ERR(fpsensor_dev->pinctrl1);
        fpsensor_error("fpsensor Cannot find fp pinctrl1.\n");
        goto err1;
    }
    fpsensor_spidev_dts_init(fpsensor_dev);
    // end dts read

    fpsensor_irq_gpio_cfg();

    // fpsensor_dev->spi->irq = mt_gpio_to_irq(CUST_EINT_FP_NUM);
    // fpsensor_dev->spi->irq = CUST_EINT_FP_NUM;

    error = fpsensor_create_class(fpsensor_dev);
    if (error)
        goto err2;
    error = fpsensor_create_device(fpsensor_dev);
    if (error)
        goto err2;
    cdev_init(&fpsensor_dev->cdev, &fpsensor_fops);
    fpsensor_dev->cdev.owner = THIS_MODULE;
    error = cdev_add(&fpsensor_dev->cdev, fpsensor_dev->devno, 1);
    if (error)
        goto err2;
    // mt_eint_registration(fpsensor_dev->spi->irq, EINTF_TRIGGER_RISING, fpsensor_irq, 1);        // 0:auto mask is no
    //register input device
    fpsensor_dev->input = input_allocate_device();
    if (fpsensor_dev->input == NULL)
    {
        fpsensor_debug(ERR_LOG, "%s, Failed to allocate input device.\n", __func__);
        error = -ENOMEM;
        goto err2;
    }
    __set_bit(EV_KEY, fpsensor_dev->input->evbit);
    __set_bit(FPSENSOR_INPUT_HOME_KEY, fpsensor_dev->input->keybit);

    __set_bit(FPSENSOR_INPUT_MENU_KEY, fpsensor_dev->input->keybit);
    __set_bit(FPSENSOR_INPUT_BACK_KEY, fpsensor_dev->input->keybit);
    __set_bit(FPSENSOR_INPUT_FF_KEY, fpsensor_dev->input->keybit);

    __set_bit(FPSENSOR_NAV_TAP_KEY, fpsensor_dev->input->keybit);
    __set_bit(FPSENSOR_NAV_UP_KEY, fpsensor_dev->input->keybit);
    __set_bit(FPSENSOR_NAV_DOWN_KEY, fpsensor_dev->input->keybit);
    __set_bit(FPSENSOR_NAV_RIGHT_KEY, fpsensor_dev->input->keybit);
    __set_bit(FPSENSOR_NAV_LEFT_KEY, fpsensor_dev->input->keybit);
    __set_bit(FPSENSOR_INPUT_CAMERA_KEY, fpsensor_dev->input->keybit);
    fpsensor_dev->input->name = FPSENSOR_INPUT_NAME;
    if (input_register_device(fpsensor_dev->input))
    {
        fpsensor_debug(ERR_LOG, "%s, Failed to register input device.\n", __func__);
        error = -ENODEV;
        goto err1;
    }
    fpsensor_dev->device_available = 1;
    fpsensor_dev->irq_count = 1;
    // mt_eint_unmask(fpsensor_dev->spi->irq);

    fpsensor_dev->sig_count = 0;

    fpsensor_debug(INFO_LOG, "%s: fpsensor init finished======\n", __func__);
#if FPSENSOR_SYSFS
    error = sysfs_create_group(&dev->kobj, &attribute_group);
    if (error) {
        fpsensor_debug(ERR_LOG, "%s, could not create sysfs\n", __func__);
        goto err1;
    }
#endif
    /* netlink interface init */
#if FPSENSOR_NETLINK
    fpsensor_netlink_init();
#endif
    fpsensor_dev->probe_finish = 1;
    fpsensor_dev->is_sleep_mode = 0;
    fpsensor_hw_power_enable(1);
    fpsensor_spi_clk_enable(1);
#if SUPPORT_REE_SPI
    fpsensor_spi_setup_conf_ree(6, DMA_TRANSFER);
    fpsensor_spi_read_byte_ree(0x1234, &i);
    fpsensor_debug(INFO_LOG, "%s: HWID ; %d======\n", __func__, i);
#endif
    //init wait queue
    init_waitqueue_head(&fpsensor_dev->wq_irq_return); 

    fpsensor_debug(INFO_LOG, "%s probe finished, normal driver version: %s\n", __func__, FPSENSOR_SPI_VERSION);
    //test
    // for ( i = 0; i < 10; ++i)
    // {
    fpsensor_hw_reset(20);
    // }
    FUNC_EXIT();
//+add by hzb for ontim debug
        REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();
//-add by hzb for ontim debug
    return 0;
err1:
    input_free_device(fpsensor_dev->input);
#if SUPPORT_REE_SPI
    kfree(fpsensor_dev->spi_buffer);
#endif
err2:
    device_destroy(fpsensor_dev->class, fpsensor_dev->devno);
    fpsensor_hw_power_enable(0);
    fpsensor_spi_clk_enable(0);
#if SUPPORT_REE_SPI
err3:
#endif
    kfree(fpsensor_dev);
    FUNC_EXIT();
    return status;
}

static int fpsensor_remove(struct spi_device *spi)
{
    fpsensor_data_t *fpsensor_dev = spi_get_drvdata(spi);
    FUNC_ENTRY();

    /* make sure ops on existing fds can abort cleanly */
    if (fpsensor_dev->spi->irq)
        free_irq(fpsensor_dev->spi->irq, fpsensor_dev);
#if FPSENSOR_NETLINK
    fpsensor_netlink_destory();
#endif
    fpsensor_dev->spi = NULL;
    spi_set_drvdata(spi, NULL);
#if FPSENSOR_SYSFS
    sysfs_remove_group(&spi->dev.kobj, &attribute_group);
#endif

    /* prevent new opens */
    //mutex_lock(&device_list_lock);
    // sysfs_remove_group(&spi->dev.kobj, &fpsensor_debug_attr_group);
    //list_del(&fpsensor_dev->device_entry);
    device_destroy(fpsensor_dev->class, fpsensor_dev->devno);
    unregister_chrdev_region(fpsensor_dev->devno, 1);
    class_destroy(fpsensor_dev->class);
    if (fpsensor_dev->users == 0) {
        if (fpsensor_dev->input != NULL)
            input_unregister_device(fpsensor_dev->input);

        if (fpsensor_dev->buffer != NULL)
            kfree(fpsensor_dev->buffer);
    }
    fpsensor_hw_power_enable(0);
    //mutex_unlock(&device_list_lock);
#if SUPPORT_REE_SPI
    kfree(fpsensor_dev->spi_buffer);
#endif
    fpsensor_debug(INFO_LOG, "%s remove finished\n", __func__);
    kfree(fpsensor_dev);
    /*
       if (s_DEVINFO_fpsensor != NULL)
       {
       kfree(s_DEVINFO_fpsensor);
       }
     */
    FUNC_EXIT();
    return 0;
}
struct spi_device_id fpsensor_spi_id_table = {FPSENSOR_DEV_NAME, 0};

static struct spi_driver fpsensor_spi_driver = {
    .driver = {
        .name   = FPSENSOR_DEV_NAME,
        //.bus    = &spi_bus_type,
        .owner  = THIS_MODULE,
    },
    .id_table = &fpsensor_spi_id_table,
    .probe  = fpsensor_probe,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
    .remove = __devexit_p(fpsensor_remove),
#else
    .remove = fpsensor_remove,
#endif
};

// struct mt_chip_conf spi_ctrdata = {
//     .setuptime = 10,
//     .holdtime = 10,
//     .high_time = 50, /* 1MHz */
//     .low_time = 50,
//     .cs_idletime = 10,
//     .ulthgh_thrsh = 0,

//     .cpol = SPI_CPOL_0,
//     .cpha = SPI_CPHA_0,

//     .rx_mlsb = SPI_MSB,
//     .tx_mlsb = SPI_MSB,

//     .tx_endian = SPI_LENDIAN,
//     .rx_endian = SPI_LENDIAN,

//     .com_mod = FIFO_TRANSFER,
//     /* .com_mod = DMA_TRANSFER, */

//     .pause = 0,
//     .finish_intr = 1,
//     .deassert = 0,
//     .ulthigh = 0,
//     .tckdly = 0,
// };

// static struct spi_board_info spi_fp_board_info[] __initdata = {
//     [0] = {
//         .modalias        = FPSENSOR_DEV_NAME,
//         .platform_data        = NULL,
//         .chip_select = 0,
//         .bus_num        = 0,
//         .mode = SPI_MODE_0,
//         // .controller_data    = &spi_ctrdata,
//     },
// };


const struct mt_chip_conf spi_ctrdata = {
    .setuptime = 10,
    .holdtime = 10,
    .high_time = 50, /* 1MHz */
    .low_time = 50,
    .cs_idletime = 10,
    .ulthgh_thrsh = 0,

    .cpol = SPI_CPOL_0,
    .cpha = SPI_CPHA_0,

    .rx_mlsb = SPI_MSB,
    .tx_mlsb = SPI_MSB,

    .tx_endian = SPI_LENDIAN,
    .rx_endian = SPI_LENDIAN,

    .com_mod = FIFO_TRANSFER,
    /* .com_mod = DMA_TRANSFER, */

    .pause = 0,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 0,
};
struct mt_chip_conf fpsensor_spi_conf_mt65xx = {
    .setuptime = 20,
    .holdtime = 20,
    .high_time = 50,
    .low_time = 50,
    .cs_idletime = 5,
    .rx_mlsb = 1,
    .tx_mlsb = 1,
    .tx_endian = 0,
    .rx_endian = 0,
    .cpol = 0,
    .cpha = 0,
    .com_mod = FIFO_TRANSFER,
    .pause = 1,
    .finish_intr = 1,
    .deassert = 0,
    .tckdly = 0,
};

static struct spi_board_info spi_fp_board_info[] __initdata = {
    [0] = {
        .modalias = FPSENSOR_DEV_NAME,
        .bus_num = 0,
        .chip_select = 0,
        .mode = SPI_MODE_0,
        .controller_data = &fpsensor_spi_conf_mt65xx, //&spi_conf
    },
};

/*-------------------------------------------------------------------------*/
static int __init fpsensor_init(void)
{
    int status;
    FUNC_ENTRY();
   if (get_fp_vendor() !=3 ){
	 printk("[leadcore]not find  leancore fingerprint sensor, get_fp_vendor=%d!\n",get_fp_vendor());
	 return -1;       
	 }
    spi_register_board_info(spi_fp_board_info, ARRAY_SIZE(spi_fp_board_info));
    status = spi_register_driver(&fpsensor_spi_driver);
    if (status < 0) {
        fpsensor_debug(ERR_LOG, "%s, Failed to register SPI driver.\n", __func__);
    }

    FUNC_EXIT();
    return status;
}
module_init(fpsensor_init);

static void __exit fpsensor_exit(void)
{
    FUNC_ENTRY();
    spi_unregister_driver(&fpsensor_spi_driver);
    FUNC_EXIT();
}
module_exit(fpsensor_exit);


MODULE_AUTHOR("xhli");

MODULE_DESCRIPTION(" Fingerprint chip TEE driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:fpsensor_spi");



