/*************************************************************************
  版权： ShenZhen ChipSailing Technology Co., Ltd. All rights reserved.
  文件名称: chips_main.c
  文件描述: spi字符设备驱动
  作者: zwp    ID:58    版本:2.0   日期:2016/10/16
  其他:
  历史:
      1. 日期:           作者:          ID:
         修改说明:

      2.
**************************************************************************/

/*********************************头文件***********************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <asm/uaccess.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/irqreturn.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <mach/mt_clkmgr.h>
#include <mt_spi_hal.h>
#include "./inc/chips_main.h"
#include "./inc/chips_platform.h"
#include "./inc/chips_common.h"
#include "./inc/fp_vendor.h"

#if defined(MTK_PLAT)
#include "./inc/mt_spi.h"
#elif defined(QCOM_PLAT)

#include <linux/of_gpio.h>

#endif


#if defined (CONFIG_HUAWEI_DSM)
#include <dsm/dsm_pub.h>
struct dsm_dev dsm_fingerprint =
{
    .name = "dsm_fingerprint",
    .device_name = "chipsailing",
    .ic_name = "NNN",
    .module_name = "NNN",
    .fops = NULL,
    .buff_size = 1024,
};
struct dsm_client *fingerprint_dclient = NULL;
#endif



/**********************************宏定义************************************/
/*设备信息*/
#define N_SPI_MINORS 33                   //次设备号总数
#define CHIPS_FP_MAJOR 155                //主设备号

#define CHIPS_DEV_NAME    "cs_spi"        //字符设备名称
#define CHIPS_CLASS_NAME  "chips_fp"      //类名
#define CHIPS_CHRD_NAME   "chips_fp"      //字符设备名称
#define FB_EVENT_NOTIFIER

/*按键信息*/
#define CHIPS_IOC_MAGIC                      'k'
#define CHIPS_IOC_INPUT_KEY_EVENT            _IOW(CHIPS_IOC_MAGIC, 7, struct chips_key_event)
#define CHIPS_IOC_SET_RESET_GPIO             _IOW(CHIPS_IOC_MAGIC, 8, unsigned char)
#define CHIPS_IOC_SPI_MESSAGE                _IOWR(CHIPS_IOC_MAGIC, 9, struct chips_ioc_transfer)
#define CHIPS_IOC_SPI_SEND_CMD               _IOW(CHIPS_IOC_MAGIC, 10, unsigned char)
#define CHIPS_IOC_ENABLE_IRQ                 _IO(CHIPS_IOC_MAGIC, 11)
#define CHIPS_IOC_DISABLE_IRQ                _IO(CHIPS_IOC_MAGIC, 12)
#define CHIPS_IOC_SENSOR_CONFIG              _IOW(CHIPS_IOC_MAGIC, 13, void*)
#define CHIPS_FP_IOC_GET_IRQ_STATE           _IOR(CHIPS_IOC_MAGIC, 16, unsigned int)
#define CHIPS_FP_IOC_GET_AUTO_TEST_RESULT    _IOR(CHIPS_IOC_MAGIC, 17, unsigned int)
#define CHIPS_INPUT_KEY_FINGERDOWN        KEY_F11
#define CHIPS_INPUT_KEY_SINGLECLICK       601//106 //KEY_F19
#define CHIPS_INPUT_KEY_DOUBLECLICK       501//107 //KEY_F6
#define CHIPS_INPUT_KEY_LONGTOUCH         502//108 //KEY_F5
#define CHIPS_INPUT_KEY_UP                511//102 //KEY_F7
#define CHIPS_INPUT_KEY_DOWN              512//103 //KEY_F8
#define CHIPS_INPUT_KEY_LEFT              513//104 //KEY_F9
#define CHIPS_INPUT_KEY_RIGHT             514//105 //KEY_F10

#define CHIPS_INPUT_KEY_POWER             KEY_POWER

/*********************************全局变量定义*********************************/
const static unsigned bufsiz = 10240*2;
static int display_blank_flag = -1;
static struct wake_lock wakelock;
struct chips_data *chips_spidev;
int autotest_input_result =-1;

static DECLARE_BITMAP(minors,N_SPI_MINORS);    //定义一个位图
static LIST_HEAD(device_list);                 //创建一个全局链表,用于保存chips_data
static DEFINE_MUTEX(device_list_lock);         //定义一个全局互斥锁
extern int get_fp_vendor(void);
struct platform_device*   pf_dev;
int g_irq_happend = -1;                        //供中断测试使用的g_irq_happend为一全局变量，在irq_handler()中置位
/********************************结构体定义************************************/
/**
 *  struct chips_key_event 描述按键事件
 *  @code  键码
 *  @value 键值 0:抬起  1:按下  2:1个完整的按键事件
 */
struct chips_key_event {
    int code;
    int value;
};

/**
 *  struct chips_ioc_transfer 描述芯片通过ioctl传递的数据
 *  @cmd          命令值
 *  @addr         地址
 *  @buf          数据缓存区
 *  @actual_len   要传送的数据的字节数
 */
struct chips_ioc_transfer {
    unsigned char cmd;
    unsigned short addr;
    unsigned char *buf;
    unsigned short actual_len;
};
//+add by hzb for ontim debug
#include <ontim/ontim_dev_dgb.h>
static char version[]="cs3511_mtk_1.0";
static char vendor_name[20]="Truly-CS3511";
    DEV_ATTR_DECLARE(fpsensor)
    DEV_ATTR_DEFINE("version",version)
    DEV_ATTR_DEFINE("vendor",vendor_name)
    DEV_ATTR_DECLARE_END;
    ONTIM_DEBUG_DECLARE_AND_INIT(fpsensor,fpsensor,8);
//-add by hzb for ontim debug

/***********************************函数定义***************************************/


//add for fingerprint autotest
/*--------------------- devices/platform/fingerprint/result  -------------------------*/
/*--------------------- devices/platform/fingerprint/fingerprint_chip_info  -------------------------*/

static ssize_t result_show(struct device* device,struct device_attribute* attribute,char* buffer)
{
    //struct chips_data *chips_data=dev_get_drvdata(device);
    //struct fp_data* fingerprint = dev_get_drvdata(device);
    //return scnprintf(buffer, PAGE_SIZE, "%i\n", fingerprint->autotest_input);
    return scnprintf(buffer, PAGE_SIZE, "%i\n", autotest_input_result);
}

static ssize_t result_store(struct device* device,struct device_attribute* attribute,const char* buffer, size_t count)
{
    //struct fp_data* fingerprint = dev_get_drvdata(device);
    //fingerprint->autotest_input = simple_strtoul(buffer, NULL, 10);
    //sysfs_notify(&fingerprint->pf_dev->dev.kobj, NULL, "result");
    return count;
}

static DEVICE_ATTR(result, S_IRUSR | S_IWUSR, result_show, result_store);

//end of add for fingerprint autotest


static ssize_t fingerprint_chip_info_show(struct device *device, struct device_attribute *attribute, char *buf)
{

    char sensor_id[10]={0};
    strcpy(sensor_id, "0x3106");
    return scnprintf(buf, 50, "CHIPSAILING Fingerprint chip id: %s\n", sensor_id);
}

static DEVICE_ATTR(fingerprint_chip_info, S_IRUSR  | S_IRGRP | S_IROTH, fingerprint_chip_info_show, NULL);

static struct attribute* attributes[] =
{
    &dev_attr_fingerprint_chip_info.attr,
    &dev_attr_result.attr,
    NULL
};

static const struct attribute_group attribute_group =
{
    .attrs = attributes,
};

/**
 *  @brief chips_enable_irq 使能中断
 *
 *  @param [in] chips_data 芯片数据的结构体指针
 *
 *  @return 无返回值
 */
static void chips_enable_irq(struct chips_data *chips_data)
{
    if(!chips_data->irq_enabled){
        enable_irq(chips_data->irq);
        chips_data->irq_enabled = true;
    }
}


/**
 *  @brief chips_disable_irq 关闭中断
 *
 *  @param [in] chips_data 芯片数据的结构体指针
 *
 *  @return 无返回值
 */
static void chips_disable_irq(struct chips_data *chips_data)
{
    if(chips_data->irq_enabled){
        disable_irq_nosync(chips_data->irq);
        chips_data->irq_enabled = false;
    }
}


 /**
 *  @brief chips_register_input 注册输入设备
 *
 *  @param [in] chips_data 芯片数据的结构体指针
 *
 *  @return 成功返回0，失败返回负数
 */
static int chips_register_input(struct chips_data *chips_data)
{
    chips_data->input = input_allocate_device();
    if(NULL == chips_data->input){
        chips_dbg("allocate input device error\n");
        return -ENOMEM;
    }

    __set_bit(EV_KEY,chips_data->input->evbit);
    __set_bit(CHIPS_INPUT_KEY_FINGERDOWN,chips_data->input->keybit);
    __set_bit(CHIPS_INPUT_KEY_SINGLECLICK,chips_data->input->keybit);
    __set_bit(CHIPS_INPUT_KEY_DOUBLECLICK,chips_data->input->keybit);
    __set_bit(CHIPS_INPUT_KEY_LONGTOUCH,chips_data->input->keybit);
    __set_bit(CHIPS_INPUT_KEY_UP,chips_data->input->keybit);
    __set_bit(CHIPS_INPUT_KEY_DOWN,chips_data->input->keybit);
    __set_bit(CHIPS_INPUT_KEY_LEFT,chips_data->input->keybit);
    __set_bit(CHIPS_INPUT_KEY_RIGHT,chips_data->input->keybit);
    __set_bit(CHIPS_INPUT_KEY_POWER,chips_data->input->keybit);
    __set_bit(KEY_F19,chips_data->input->keybit);

    chips_data->input->id.bustype = BUS_HOST;
    chips_data->input->name = "fingerprint";
    chips_data->input->id.version = 1;

    if(input_register_device(chips_data->input) != 0){
        chips_dbg("input_register_device error\n");
        input_free_device(chips_data->input);
        return -ENOMEM;
    }

    return 0;
}


 /**
 *  @brief chips_irq_handler 中断处理函数
 *
 *  @param [in] irq 中断号
 *  @param [in] dev 芯片数据的结构体指针
 *
 *  @return 总是返回IRQ_HANDLED
 */
static irqreturn_t chips_irq_handler(int irq,void *dev)
{
    struct chips_data *chips_data = (struct chips_data *)dev;
    g_irq_happend = 1;
    wake_lock_timeout(&wakelock, 6*HZ);

    if(chips_data->async != NULL){
        kill_fasync(&chips_data->async,SIGIO,POLL_IN);
    }else{
        chips_dbg("struct *async is null\n");
    }

    return IRQ_HANDLED;
}
static int cs_kill_fasync(struct chips_data *chips_data)
{

    if(chips_data->async != NULL){
        kill_fasync(&chips_data->async,SIGIO,POLL_IN);
    }else{
        chips_dbg("struct *async is null\n");
    }
    return 0;
}



 /**
 *  @brief chips_spi_setup 设置spi参数
 *
 *  @param [in] spi 指向spi_device结构体的指针
 *
 *  @return 成功返回0，失败返回负数
 */
static int chips_spi_setup(struct spi_device *spi)
{
    spi->mode = SPI_MODE_0;
    spi->bits_per_word = 8;
    //spi->max_speed_hz = 7000000;
    spi->chip_select = 0;

    return spi_setup(spi);
}


/**
 *  @brief chips_read 芯片读操作,从fp读取count字节数据到buf
 *
 *  @param [in] fp      file结构体指针,表示打开的一个文件
 *  @param [in] buf     用户空间的buffer指针
 *  @param [in] count   要读取的字节数
 *  @param [in] offset  文件偏移指针
 *
 *  @return 成功返回读取的字节数,失败返回负数
 */
static ssize_t chips_read(struct file *fp, char __user *buf, size_t count, loff_t *offset)
{
    struct chips_data *chips_data;
    unsigned long missing;
    int status = -1;

    if(NULL == buf)
        return -EINVAL;

    if(count > bufsiz)
        return -EMSGSIZE;

    chips_data = (struct chips_data *)fp->private_data;

    mutex_lock(&chips_data->buf_lock);
    missing = copy_to_user(buf,&display_blank_flag,count);
        if(missing == count ){
            status = -EFAULT;
        }else{
            status = count - missing;
    }
    mutex_unlock(&chips_data->buf_lock);

    return status;
}


/**
 *  @brief chips_write 芯片写操作,将buf的count个字节的数据写入fp
 *
 *  @param [in] fp      file结构体指针,表示打开的一个文件
 *  @param [in] buf     用户空间的buffer指针,表示要写入的数据
 *  @param [in] count   要写入的字节数,注意不能超过128个字节
 *  @param [in] offset  文件偏移
 *
 *  @return 成功返回写入的字节数,失败返回负数
 */
static ssize_t chips_write(struct file *fp, const char __user *buf, size_t count, loff_t *offset)
{
    int status = -1;

    struct chips_data *chips_data;
    unsigned long missing;
    unsigned char buffer[128] = {0};

    if(count > 128)
        return -EMSGSIZE;

    chips_data = (struct chips_data *)fp->private_data;

    missing = copy_from_user(buffer,buf,count);
    if(missing != 0){
        status = -EFAULT;
    }else{
        if(strncmp(buffer,"config",strlen("config"))==0){
            chips_dbg("sensor init begin\n");
            chips_sensor_config();
            chips_dbg("sensor init success\n");

        }else if(strncmp(buffer,"info",strlen("info"))==0){
            uint16_t reg_0xFC2E = 0;
            uint16_t reg_0xFC30 = 0;
            uint16_t sensorid = 0;
            uint16_t reg_0xFC80 = 0;
            uint16_t reg_0xFC88 = 0;
            uint16_t reg_0xFC8A = 0;
            uint16_t reg_0xFC00 = 0;
            uint8_t mode = -1;
            uint8_t flag = -1;

            read_SFR(0x46,&mode);
            read_SFR(0x50,&flag);
            chips_probe_sensorID(&sensorid);
            read_SRAM(0xFC00,&reg_0xFC00);
            read_SRAM(0xFC2E,&reg_0xFC2E);
            read_SRAM(0xFC30,&reg_0xFC30);
            read_SRAM(0xFC80,&reg_0xFC80);
            read_SRAM(0xFC88,&reg_0xFC88);
            read_SRAM(0xFC8A,&reg_0xFC8A);

            printk("flag = %x\n",flag);
            printk("mode = %x\n",mode);
            printk("sensorid = %x\n",sensorid);
            printk("reg_0xFC00 = %x\n",reg_0xFC00);
            printk("reg_0xFC2E = %x\n",reg_0xFC2E);
            printk("reg_0xFC30 = %x\n",reg_0xFC30);
            printk("reg_0xFC80 = %x\n",reg_0xFC80);
            printk("reg_0xFC88 = %x\n",reg_0xFC88);
            printk("reg_0xFC8A = %x\n",reg_0xFC8A);

        }else if(strncmp(buffer,"irq",strlen("irq"))==0){
            chips_dbg("irq_num = %d\n",chips_data->irq);

        }else if(strncmp(buffer,"reset",strlen("reset"))==0){
            status = chips_hw_reset(chips_data,5);
            chips_dbg("reset status = %d\n", status);

        }else if(strncmp(buffer,"image",strlen("image"))==0){
            int i = 0;
            uint8_t mode = -1;
            uint8_t flag = -1;
            uint16_t test_0xFC00 = 0;

            read_SFR(0x46,&mode);
            read_SFR(0x50, &flag);
            chips_dbg("flag = %x\n",flag);
            chips_dbg("mode = %x\n",mode);
            memset(chips_data->buffer,0,bufsiz);
            status = chips_scan_one_image(0XFFFF,chips_data->buffer,112*88);
            if(status != 0)
                chips_dbg("chips_scan_one_image err,status = %d\n",status);
            for(i = 0;i < 50; i++){
                chips_dbg("image[%d] = 0x%x\n",i,chips_data->buffer[i]);
            }
            for(i = 112*88; i > 112*88-50; i--){
                chips_dbg("image[%d] = 0x%x\n",i,chips_data->buffer[i]);
            }
            read_SRAM(0xFC00,&test_0xFC00);
            chips_dbg("test_0xFC00 = %x\n",test_0xFC00);

        }else if(strncmp(buffer,"mode:0",strlen("mode:0"))==0){
            status = chips_set_sensor_mode(IDLE);
            chips_dbg("set idle mode status = %d\n",status);

        }else if(strncmp(buffer,"mode:1",strlen("mode:1"))==0){
            status = chips_set_sensor_mode(NORMAL);
            chips_dbg("set normal mode status = %d\n",status);

        }else if(strncmp(buffer,"mode:2",strlen("mode:2"))==0){
            status = chips_set_sensor_mode(SLEEP);
            chips_dbg("set sleep mode status = %d\n",status);

        }else if(strncmp(buffer,"mode:6",strlen("mode:6"))==0){
            status = chips_set_sensor_mode(DEEP_SLEEP);
            chips_dbg("set deep_sleep mode status = %d\n",status);
        }

    status = count - missing;
    }

    return status;
}


/**
 *  @brief chips_open  芯片打开操作
 *
 *  @param [in] inode  inode结构体指针
 *  @param [in] fp     file结构体指针
 *
 *  @return 成功返回0 失败返回负数
 */
static int chips_open(struct inode *inode, struct file *fp)
{
    int status = -ENXIO;
    struct chips_data *chips_data;

    mutex_lock(&device_list_lock);

    /*遍历全局链表,找出chips_data*/
    list_for_each_entry(chips_data,&device_list,device_entry){
        if(chips_data->devt == inode->i_rdev){
            status = 0;
            break;
        }
    }

    /*初始化chips_data->buffer,即:为chips_data->buffer申请内存*/
    if (status == 0) {
        if (NULL == chips_data->buffer){
            chips_data->buffer = kmalloc(bufsiz, GFP_KERNEL);
            if (NULL == chips_data->buffer) {
                chips_dbg("kmalloc/ENOMEM\n");
                status = -ENOMEM;
            }
        }

        /*打开设备文件的用户计数加1*/
        if (status == 0){
            chips_data->users++;
            fp->private_data = chips_data;
            nonseekable_open(inode, fp); //禁止lseek操作
        }
    }else{
        chips_dbg("no device found with minor = %d\n", iminor(inode));
    }

    mutex_unlock(&device_list_lock);
    return status;
}


/**
 *  @brief chips_release 芯片关闭操作,close的时候调用
 *
 *  @param [in] inode  inode结构体指针
 *  @param [in] fp     file结构体指针
 *
 *  @return 总是返回0
 */
static int chips_release(struct inode *inode, struct file *fp)
{
    struct chips_data *chips_data;

    mutex_lock(&device_list_lock);
    chips_data = (struct chips_data*)fp->private_data;
    fp->private_data = NULL;

    /*打开设备文件的用户计数减1*/
    chips_data->users--;

    /*当打开设备文件的用户计数为0时,释放chips_data->buffer*/
    if (chips_data->users == 0){
        int     dofree;

        kfree(chips_data->buffer);
        chips_data->buffer = NULL;

        spin_lock_irq(&chips_data->spin_lock);
        dofree = (chips_data->spi == NULL);
        spin_unlock_irq(&chips_data->spin_lock);

        if (dofree){
            kfree(chips_data);
            chips_spidev = NULL;
        }

    }

    mutex_unlock(&device_list_lock);
    return 0;
}


/**
 *  @brief chips_ioctl 芯片IO控制函数
 *
 *  @param [in] fp   file结构体指针
 *  @param [in] cmd  命令
 *  @param [in] arg  参数,通常是一个指针
 *
 *  @return 成功返回0,失败返回负数
 */
static long chips_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int err = 0;
    int retval = 0;
    int missing = 0;
    uint8_t command = 0;
    int param_num;
    int param_len;

    struct param *p_param = NULL;
    struct chips_key_event *key = NULL;
    struct chips_ioc_transfer *ioc = NULL;
    struct config *p_config = NULL;
    struct chips_data *chips_data;
    struct spi_device *spi;

    /*命令合法性判断,magic码为'K'*/
    if(_IOC_TYPE(cmd)!=CHIPS_IOC_MAGIC)
        return -ENOTTY;

    /*内核交互环境判断是否可以读写*/
    if(_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE,(void __user*)arg,_IOC_SIZE(cmd));
    if(err==0 && _IOC_DIR(cmd)&_IOC_WRITE)
        err = !access_ok(VERIFY_READ,(void __user*)arg,_IOC_SIZE(cmd));
    if(err)
        return -EFAULT;

    /*获取spi_device*/
    chips_data = (struct chips_data*)fp->private_data;
    spin_lock_irq(&chips_data->spin_lock);
    spi = spi_dev_get(chips_data->spi); //为什么还需要这一步，直接判断chips_data->spi_dev不行吗?
    spin_unlock_irq(&chips_data->spin_lock);

    if(spi == NULL)
        return -ESHUTDOWN;

    /* use the buffer lock here for triple duty:
     *  - prevent I/O (from us) so calling spi_setup() is safe;
     *  - prevent concurrent SPI_IOC_WR_* from morphing
     *    data fields while SPI_IOC_RD_* reads them;
     *  - CHIPS_IOC_SPI_MESSAGE needs the buffer locked "normally".
     */
    mutex_lock(&chips_data->buf_lock);

    switch(cmd)
    {
        /*check finger status*/
        case CHIPS_IOC_SET_RESET_GPIO:
            //chips_dbg("reset sensor\n");
            retval = __get_user(command,(u8 __user*)arg);
            if(retval == 0){
                retval = chips_set_reset_gpio(chips_data,command);
                if(retval < 0){
                    chips_dbg("Failed to send spi cmd,cmd = 0x%x",command);
                    retval = -EFAULT;
                    break;
                }
            }
            break;
        /*使能中断*/
        case CHIPS_IOC_ENABLE_IRQ:
            chips_enable_irq(chips_data);
            chips_dbg("enable irq\n");
            break;

        /*关闭中断*/
        case CHIPS_IOC_DISABLE_IRQ:
            chips_disable_irq(chips_data);
            chips_dbg("disable irq\n");
            break;

        /*键值事件处理*/
        case CHIPS_IOC_INPUT_KEY_EVENT:
            chips_dbg("report key event\n");

            /*为key申请内存*/
            key = kzalloc(sizeof(*key),GFP_KERNEL);
            if(NULL == key){
                chips_dbg("Failed to allocate mem for key event\n");
                retval = -ENOMEM;
                break;
            }

            /*获取key数据*/
            missing = copy_from_user(key,(u8 __user *)arg,sizeof(*key));
            if(missing != 0){
                chips_dbg("Failed to copy key from user space\n");
                retval = -EFAULT;
                break;
            }

            /*按键值类型判断处理：抬起、按下、一个完整的键值*/
            /*上报一个完整的键值(2)*/
            if(key->value == 2){
                chips_dbg("a key event reported,key->code = 0x%x,key->value = 0x%x\n",key->code,key->value);
                //__set_bit(key->code,chips_data->input->keybit);
                input_report_key(chips_data->input,key->code,1);
                input_sync(chips_data->input);
                input_report_key(chips_data->input,key->code,0);
                input_sync(chips_data->input);

            /*上报按下（1）或抬起（0）*/
            }else{
                chips_dbg("a key event reported,key->code = 0x%x,key->value = 0x%x\n",key->code,key->value);
                //__set_bit(key->code,chips_data->input->keybit);
                input_report_key(chips_data->input,key->code,key->value);
                input_sync(chips_data->input);
            }

            break;

        /*spi数据处理*/
        case CHIPS_IOC_SPI_MESSAGE:
            chips_dbg("spi message\n");

            /*为chips_ioc_transfer 申请内存*/
            ioc = kzalloc(sizeof(*ioc),GFP_KERNEL);
            if(NULL == ioc){
                chips_dbg("Failed to allocat mem for ioc\n");
                retval = -EFAULT;
                break;
            }

            /*获取用户空间的参数*/
            missing = copy_from_user(ioc,(u8 __user*)arg,sizeof(*ioc));
            if(missing != 0){
                chips_dbg("Failed to copy from userspace to kernel space\n");
                retval = -EFAULT;
                break;
            }

            /*判断参数合法性*/
            if((ioc->actual_len > bufsiz) || (ioc->actual_len == 0)){
                chips_dbg("the length of bytes:%d transferred not suported\n",ioc->actual_len);
                retval = -EMSGSIZE;
                break;
            }

            chips_dbg("ioc->cmd = %x\n",ioc->cmd);

            /*写sram*/
            if(ioc->cmd == CHIPS_W_SRAM){
                chips_dbg("write sram\n");
                missing = copy_from_user(chips_data->buffer,(u8 __user*)ioc->buf,ioc->actual_len);
                if(missing != 0){
                    chips_dbg("Failed to copy from userspace to kernel\n");
                    retval = -EFAULT;
                    break;
                }

                chips_dbg("chips_data->buffer[0] = 0x%x,chips_data->buffer[1] = 0x%x,ioc->actual_len = %d\n",chips_data->buffer[0],chips_data->buffer[1],ioc->actual_len);
                retval = chips_sram_write(ioc->addr,chips_data->buffer,ioc->actual_len);
                if(retval < 0){
                    chips_dbg("write sram error,retval = %d\n",retval);
                    retval = -EFAULT;
                    break;
                };

            /*读sram*/
            }else if(ioc->cmd == CHIPS_R_SRAM){
                chips_dbg("read sram\n");
                retval = chips_sram_read(ioc->addr,chips_data->buffer,ioc->actual_len);
                if(retval < 0){
                    chips_dbg("read sram error,retval = %d\n",retval);
                    retval = -EFAULT;
                    break;
                };
                chips_dbg("chips_data->buffer[0] = 0x%x,chips_data->buffer[1] = 0x%x,ioc->actual_len = %d\n",chips_data->buffer[0],chips_data->buffer[1],ioc->actual_len);

                missing = copy_to_user((u8 __user*)ioc->buf,chips_data->buffer,ioc->actual_len);
                if(missing != 0){
                    chips_dbg("Failed to copy from kernel to userspace\n");
                    retval = -EFAULT;
                    break;
                }

            /*写寄存器*/
            }else if(ioc->cmd == CHIPS_W_SFR){
                chips_dbg("write sfr\n");
                missing = copy_from_user(chips_data->buffer,(u8 __user*)ioc->buf,ioc->actual_len);
                if(missing != 0){
                    chips_dbg("Failed to copy from userspace to kernel\n");
                    retval = -EFAULT;
                    break;
                }

                chips_dbg("chips_data->buffer[0] = 0x%x,ioc->actual_len = %d\n",chips_data->buffer[0],ioc->actual_len);
                retval = chips_sfr_write(ioc->addr,chips_data->buffer,ioc->actual_len);
                if(retval < 0){
                    chips_dbg("write sfr error,retval = %d\n",retval);
                    retval = -EFAULT;
                    break;
                };

            /*读寄存器*/
            }else if(ioc->cmd == CHIPS_R_SFR){
                chips_dbg("read sfr\n");
                retval = chips_sfr_read(ioc->addr,chips_data->buffer,ioc->actual_len);
                if(retval < 0){
                    chips_dbg("read sfr error,retval = %d\n",retval);
                    retval = -EFAULT;
                    break;
                };
                chips_dbg("chips_data->buffer[0] =0x%x,ioc->actual_len = %d\n",chips_data->buffer[0],ioc->actual_len);

                missing = copy_to_user((u8 __user*)ioc->buf,chips_data->buffer,ioc->actual_len);
                if(missing != 0){
                    chips_dbg("Failed to copy from kernel to user\n");
                    retval = -EFAULT;
                    break;
                }
            }else{
                chips_dbg("error cmd for ioc\n");
                retval = -EFAULT;
            }
            break;

        /*发送spi命令*/
        case CHIPS_IOC_SPI_SEND_CMD:
            chips_dbg("spi send cmd\n");
            retval = __get_user(command,(u8 __user*)arg);
            if(retval == 0){
                chips_dbg("spi cmd from userspace is:0x%x\n",command);
                retval = chips_spi_send_cmd(&command,1);
                if(retval < 0){
                    chips_dbg("Failed to send spi cmd,cmd = 0x%x",command);
                    retval = -EFAULT;
                    break;
                }
            }
            break;

        /*sensor参数配置*/
        case CHIPS_IOC_SENSOR_CONFIG:
            chips_dbg("write config\n");

            /*为 p_config 申请内存,存放struct config*/
            p_config = kzalloc(sizeof(*p_config),GFP_KERNEL);
            if(NULL == p_config){
                chips_dbg("Failed to allocate mem for p_config\n");
                retval = -ENOMEM;
                break;
            }

            /*获取用户空间传下来的struct config的信息*/
            missing = copy_from_user(p_config,(u8 __user*)arg,sizeof(*p_config));
            if(missing != 0){
                retval = -EFAULT;
                break;
            }

            /*为p_param申请内存存放参数信息*/
            param_num = p_config->num;
            param_len = param_num*sizeof(struct param);
            if(param_len > 0){
                p_param = kzalloc(param_len,GFP_KERNEL);
                if(NULL == p_param){
                    chips_dbg("Failed to allocate mem for p_config\n");
                    retval = -ENOMEM;
                    break;
                }

            }else{
                chips_dbg("param_len is negative\n");
                retval = -EFAULT;
                break;
            }

            /*获取参数信息*/
            missing = copy_from_user(p_param,(u8 __user*)p_config->p_param,param_len);
            if(missing != 0){
                retval = EFAULT;
                break;
            }

            /*将参数写入芯片*/
            retval = chips_write_configs(p_param,param_num);
            if(retval < 0){
                chips_dbg("Failed to write configs to sensor\n");
            }
            break;

        case CHIPS_FP_IOC_GET_IRQ_STATE:
                        chips_dbg("irq state: %d\n",g_irq_happend);
            missing = copy_to_user((u8 __user *)arg,&g_irq_happend,4);
            if(missing != 0){
                retval = -EFAULT;
            }
            g_irq_happend = 0;
            break;
        case  CHIPS_FP_IOC_GET_AUTO_TEST_RESULT:

            missing = copy_to_user((u8 __user *)arg,&autotest_input_result,4);
            if(missing != 0){
                retval = -EFAULT;
            }
            chips_dbg(" CHIPS_FP_IOC_GET_AUTO_TEST_RESULT =%d\n",autotest_input_result);
            break;
        default:
            break;
    }
    if(NULL != key){
        kfree(key);
        key = NULL;
    }

    if(NULL != ioc){
        kfree(ioc);
        ioc = NULL;
    }

    if(NULL != p_config){
        kfree(p_config);
        p_config = NULL;
    }

    if(NULL != p_param){
        kfree(p_param);
        p_param = NULL;
    }

    mutex_unlock(&chips_data->buf_lock);
    spi_dev_put(spi);

    return retval;
}

#ifdef CONFIG_COMPAT
/**
 *  @brief chips_compat_ioctl  作兼容用的芯片IO控制函数
 *
 *  @param [in] fp  file结构体指针
 *  @param [in] cmd 命令
 *  @param [in] arg 参数,通常是一个指针
 *
 *  @return 成功返回0,失败返回负数
 */
static long chips_compat_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    return chips_ioctl(fp, cmd, arg);
}
#else
#define chips_compat_ioctl NULL
#endif


static int chips_fasync(int fd, struct file *fp, int mode)
{
    struct chips_data *chips_data;

    chips_data = (struct chips_data*)fp->private_data;

    return fasync_helper(fd,fp,mode,&chips_data->async);
}


static const struct file_operations chips_fops = {
    .owner = THIS_MODULE,

    .unlocked_ioctl = chips_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = chips_compat_ioctl,
#endif
    .open = chips_open,
    .release = chips_release,

    .read = chips_read,
    .write = chips_write,
    .fasync = chips_fasync,
};

#if defined(FB_EVENT_NOTIFIER)
static int stk3x1x_fb_event_notify(struct notifier_block *self,unsigned long action, void *data)
 {
    struct fb_event *event = data;
    int blank_mode = *((int *)event->data);
    if(action == FB_EVENT_BLANK){
        switch (blank_mode)
        {
            case FB_BLANK_UNBLANK:
                display_blank_flag = 0;
              cs_kill_fasync(chips_spidev);
                chips_dbg("display_blank_flag = %d\n",display_blank_flag);
                break;
            case FB_BLANK_POWERDOWN:
                display_blank_flag = 1;
          cs_kill_fasync(chips_spidev);
                chips_dbg("display_blank_flag = %d\n",display_blank_flag);
                break;
            default:
                break;
        }
    }
       return NOTIFY_OK;
}

static struct notifier_block stk3x1x_fb_notifier = {
        .notifier_call = stk3x1x_fb_event_notify,
};
#endif


static void sensor_spi_clk_enable(struct spi_device *spi,u8 bonoff)
{
#if 1
    if(bonoff == 0)
            mt_spi_disable_master_clk(spi);
    else
            mt_spi_enable_master_clk(spi);
#endif
}

static int chips_probe(struct spi_device *spi)
{
    int status;
    //uint16_t sensorid;
    unsigned long minor;
    struct chips_data *chips_data;

    printk("[chipsailing]:entry chips_probe\n");
    
    //+add by hzb for ontim debug
    if (CHECK_THIS_DEV_DEBUG_AREADY_EXIT() == 0) {
        return -EIO;
    }
    //-add by hzb for ontim debug

    chips_data = kzalloc(sizeof(struct chips_data),GFP_KERNEL);
    if(NULL == chips_data){
        chips_dbg("kzalloc mem for chips_data error\n");
        return -ENOMEM;
    }


    #if defined (CONFIG_HUAWEI_DSM)
        if(!fingerprint_dclient)
        {
            fingerprint_dclient = dsm_register_client(&dsm_fingerprint);
        }
    #endif
    chips_spidev = chips_data;

    INIT_LIST_HEAD(&chips_data->device_entry);
    spin_lock_init(&chips_data->spin_lock);
    mutex_init(&chips_data->buf_lock);

    /*初始化SPI参数*/
    sensor_spi_clk_enable(spi,1);
    chips_data->spi = spi;
    if(chips_spi_setup(spi) < 0){
        chips_dbg("spi_setup error\n");
        return -ENODEV;
    }

#if defined(MTK_PLAT)
    /*解析dts文件*/
    if(chips_parse_dts(chips_data) != 0){
        chips_dbg("Failed to parse dts\n");
        return -ENODEV;
    }

    /*设置MI、MO*/
    //chips_set_spi_mode(chips_data);

#elif defined(QCOM_PLAT)

    chips_data->reset_gpio = of_get_named_gpio(spi->dev.of_node, "cs,gpio-reset", 0);
    chips_data->irq_gpio   = of_get_named_gpio(spi->dev.of_node, "cs,gpio-irq", 0);
    chips_data->pwr_gpio   = of_get_named_gpio(spi->dev.of_node, "cs,gpio-pwr", 0);

    if(gpio_request(chips_data->reset_gpio, "chips_data reset")<0)
    chips_dbg(KERN_INFO "cs_fp_init - gpio_request (reset) failed.");

    if(gpio_direction_output(chips_data->reset_gpio,1) < 0)
    chips_dbg(KERN_INFO "cs_fp_init - gpio_direction_output (reset) failed.");

    if(gpio_request(chips_data->irq_gpio,"chips_data irq")<0)
    chips_dbg(KERN_INFO "cs_fp_init - gpio_request (irq) failed.");

    if(gpio_direction_input(chips_data->irq_gpio) < 0)
    chips_dbg(KERN_INFO "cs_fp_init - gpio_direction_input (irq) failed.");

    if(gpio_request(chips_data->pwr_gpio,"chips_data pwr")<0)
    chips_dbg(KERN_INFO "cs_fp_init - gpio_request (pwr) failed.");

    if(gpio_direction_output(chips_data->pwr_gpio,1) < 0)
    chips_dbg(KERN_INFO "cs_fp_init - gpio_direction_input (pwr) failed.");

    chips_data->irq = gpio_to_irq(chips_data->irq_gpio);

    if(chips_data->irq < 0)
    chips_dbg(KERN_INFO "cs_fp_init - gpio_to_irq failed.");


#endif

    //power on
    //gpio_set_value(chips_data->pwr_gpio, 1);
    /*硬件复位芯片*/
    chips_hw_reset(chips_data,5);

    /*读取芯片ID,作兼容用*/
    /*chips_probe_sensorID(&sensorid);
    if(sensorid != 0x3106){
        chips_dbg("sensor is not cs3511\n");
        //return -ENODEV;
    }*/

    /*********************************************
    给芯片下载参数,已移除！！！！！
    chips_sensor_config();
    *********************************************/

    /*分配主设备号,关联file_operation*/
    BUILD_BUG_ON(N_SPI_MINORS > 256);
    status = register_chrdev(CHIPS_FP_MAJOR,CHIPS_CHRD_NAME,&chips_fops);
    if(status < 0){
        chips_dbg("register chrdev error\n");
        return status;
    }

    /*创建设备类*/
    chips_data->cls = class_create(THIS_MODULE,CHIPS_CLASS_NAME);
    if(IS_ERR(chips_data->cls)){
        chips_dbg("class_create error\n");
        unregister_chrdev(CHIPS_FP_MAJOR,CHIPS_CHRD_NAME);
        return PTR_ERR(chips_data->cls);
    }

    /*创建设备节点，并将设备加入到一个链表中*/
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors,N_SPI_MINORS);
    if(minor < N_SPI_MINORS){
        struct device *dev;

        chips_data->devt = MKDEV(CHIPS_FP_MAJOR,minor);
        dev = device_create(chips_data->cls,NULL,chips_data->devt,chips_data,CHIPS_DEV_NAME);
        status = IS_ERR(dev) ? PTR_ERR(dev):0;

    }else{
        chips_dbg("no minor number available,monir = %lu\n",minor);
        status = -ENODEV;
    }

    if(status == 0){
        set_bit(minor,minors);
        list_add(&chips_data->device_entry,&device_list);
    }else{
        chips_data->devt = 0;
    }

    mutex_unlock(&device_list_lock);

    /*设置共享数据*/
    spi_set_drvdata(spi,chips_data);
    wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "chips_wakelock");
    /*注册中断处理函数，可以查看/proc/interrupts目录确认是否注册成功*/

#if defined(MTK_PLAT)
    chips_data->irq = chips_get_irqno(chips_data);
#endif

    status = request_irq(chips_data->irq,chips_irq_handler,IRQF_TRIGGER_RISING | IRQF_ONESHOT,"fngr_irq",chips_data);
    if(status != 0){
        chips_dbg("request_irq error\n");
        goto err;
    }

    enable_irq_wake(chips_data->irq);
    chips_data->irq_enabled = true;
/*
    //使能irq唤醒
    status = enable_irq_wake(chips_data->irq);
    if(status != 0){
        free_irq(chips_data->irq,chips_data);
        chips_dbg("enable_irq_wake error\n");
        goto err;
    }
*/

    /*注册输入设备，用于上报按键事件*/
    chips_register_input(chips_data);


    pf_dev= platform_device_alloc("fingerprint", -1);
    status = platform_device_add(pf_dev);
    if (status != 0)
    {
        chips_dbg("platform_device_add failed, error = %d\n", status);
        platform_device_del(pf_dev);
        goto err;
    }
    status = sysfs_create_group(&(pf_dev->dev.kobj), &attribute_group);
    if (status != 0)
    {
        chips_dbg("sysfs_create_group failed, error = %d\n", status);
        goto err;
    }
#if defined(FB_EVENT_NOTIFIER)
    fb_register_client(&stk3x1x_fb_notifier);
#endif
//+add by hzb for ontim debug
        REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();
//-add by hzb for ontim debug
    return status;

err:
    mutex_lock(&device_list_lock);
    device_destroy(chips_data->cls,chips_data->devt);
    list_del(&chips_data->device_entry);
    clear_bit(MINOR(chips_data->devt),minors);
    mutex_unlock(&device_list_lock);
    sysfs_remove_group(&pf_dev->dev.kobj, &attribute_group);
    class_destroy(chips_data->cls);
    unregister_chrdev(CHIPS_FP_MAJOR,CHIPS_CHRD_NAME);
    chips_release_gpio(chips_data);

    #if defined (CONFIG_HUAWEI_DSM)
        if (status && !dsm_client_ocuppy(fingerprint_dclient))
        {
            dsm_client_record(fingerprint_dclient,"chipsailing fingerprint_probe failed, status = %d\n", status);
            dsm_client_notify(fingerprint_dclient, DSM_FINGERPRINT_PROBE_FAIL_ERROR_NO);
        }
    #endif
    return -1;
}


static int chips_remove(struct spi_device *spi)
{
    struct chips_data* chips_data = spi_get_drvdata(spi);
    /* make sure ops on existing fds can abort cleanly */

    /*注销共享数据*/
    spin_lock_irq(&chips_data->spin_lock);
    chips_data->spi = NULL;
    spi_set_drvdata(spi, NULL);
    spin_unlock_irq(&chips_data->spin_lock);

    /*释放中断号*/
    if (chips_data->irq !=0)
        free_irq(chips_data->irq, chips_data);

    wake_lock_destroy(&wakelock);
    /*注销输入设备*/
    if (chips_data->input != NULL){
        input_unregister_device(chips_data->input);
        //Once device has been successfully registered it can be unregistered with
        //input_unregister_device(); input_free_device() should not be called in this case.
        //input_free_device(chips_data->input);
    }

    /*注销设备*/
    mutex_lock(&device_list_lock);
    device_destroy(chips_data->cls,chips_data->devt);
    list_del(&chips_data->device_entry);
    clear_bit(MINOR(chips_data->devt),minors);
    mutex_unlock(&device_list_lock);

    /*注销类*/
    class_destroy(chips_data->cls);
    unregister_chrdev(CHIPS_FP_MAJOR,CHIPS_DEV_NAME);
    chips_release_gpio(chips_data);

    /*注销互斥锁*/
    mutex_destroy(&chips_data->buf_lock);
    mutex_destroy(&device_list_lock);
    if(chips_data != NULL){
        kfree(chips_data);
        chips_spidev = NULL;
    }

    return 0;
}


/*系统挂起时调用*/
static int chips_suspend(struct spi_device *spi, pm_message_t mesg)
{
    return 0;
}


/*系统恢复时调用*/
static int chips_resume(struct spi_device *spi)
{
    return 0;
}

#if defined(MTK_PLAT)

static struct of_device_id chips_of_match_table[] = {
    {.compatible = "mediatek,cs_finger"},
    {},
}

#elif defined(QCOM_PLAT)

static struct of_device_id chips_of_match_table[] = {
    {.compatible = "cs,cs_spi",},
    {.compatible = "qcom,cs_finger"},                   //must stay the same with dtsi
    {},
}

#endif


MODULE_DEVICE_TABLE(of,chips_of_match_table);


static struct spi_driver chips_spi_driver = {
    .driver = {
        .name = CHIPS_DEV_NAME,
        .owner = THIS_MODULE,
        .of_match_table = chips_of_match_table,
    },

    .probe = chips_probe,
    .remove = chips_remove,
    .suspend = chips_suspend,
    .resume = chips_resume,
};

#if defined(MTK_PLAT)

/**
 *  struct mt_chip_conf spi_conf 描述芯片spi通信的配置信息
 */
static struct mt_chip_conf spi_conf = {
    .setuptime = 7,//15,cs
    .holdtime = 7,//15, cs
    .high_time = 25,//6 sck
    .low_time =  25,//6  sck
    .cs_idletime = 3,//20,
    //.ulthgh_thrsh = 0,
    .cpol = 0,
    .cpha = 0,
    .rx_mlsb = 1,
    .tx_mlsb = 1,

    .tx_endian = 0,
    .rx_endian = 0,
    .com_mod = DMA_TRANSFER,
    //.com_mod = FIFO_TRANSFER,
    .pause = 0,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 0,
};



static struct spi_board_info chips_spi_board_info[] __initdata = {
    [0] = {
        .modalias = CHIPS_DEV_NAME,
        .bus_num = 0,
        .chip_select=0,
        .mode = SPI_MODE_0,
        .controller_data = &spi_conf,
    },
};

#endif


static int __init chips_init(void)
{
    int status;
   if (get_fp_vendor() !=4 ){
    chips_dbg("not find chipsailing fingerprint sensor, get_fp_vendor=%d\n",get_fp_vendor());
    return -1;
    }
#if defined(MTK_PLAT)

    status = spi_register_board_info(chips_spi_board_info,ARRAY_SIZE(chips_spi_board_info));
    if(status < 0){
        chips_dbg("Failed to register spi_board_info\n");
        return status;
    }
#endif
    printk(" chips_init\n");
    status = spi_register_driver(&chips_spi_driver);
    if(status<0){
        printk(" spi_driver_register error\n");
        chips_dbg("spi_driver_register error\n");
        return status;
    }

    return 0;
}

static void __exit chips_exit(void)
{
    spi_unregister_driver(&chips_spi_driver);
}

module_init(chips_init);
module_exit(chips_exit);

MODULE_AUTHOR("ChipSailing Tech");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:cs15xx");
