
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/reboot.h>
#include <linux/string.h>
#include <video/mmp_disp.h>
#include "../misc/mediatek/lcm/inc/lcm_drv.h"
//#include "kd_imgsensor.h"

//+add by hzb
#include <../misc/mediatek/include/mt-plat/mt_gpio.h>
#include "../misc/mediatek/include/mt-plat/mt6755/include/mach/gpio_const.h"
//#include "upmu_common.h"
//#include <mach/upmu_sw.h>
#include <linux/delay.h>
//-add by hzb

//#include "sec_boot_lib.h"

//extern LCM_DRIVER *lcm_kernel_detect_drv;//add by liuwei
//extern ssize_t modem_show(struct kobject *kobj, struct kobj_attribute *attr, char* buf);

extern int FL_Enable(void);
extern int FL_Disable(void);

static struct kobject *bootinfo_kobj = NULL;
//static u8 lcd_array[]="lcd not found!";
static u8 back_camera[]="back_camera not found!";
static u8 back_camera_array[]="SanYingXing_ov13855";
static u8 back_camera_array1[]="QiuTai_s5k3l8xx";
static u8 back_camera_array2[]="XinLi_hi1333";
static u8 front_camera[]="front_camera not found!";
static u8 front_camera_array[]="SanYingXing_ov8856";
static u8 front_camera_array1[]="OuFeiGuang_gc8024";
static u8 front_camera_array2[]="XinLi_gc8034";
//static u8 back_camera_array2[]="ov13850of_mipiraw";
//static u8 back_camera_array3[]="ov13850qt_mipiraw";

//static u8 front_camera_array2[]="ov5670ofmipiraw";
//static u8 front_camera_array3[]="ov5670semipiraw";

int back_camera_find_success=0;
int front_camera_find_success=0;
int touch_flash_enable=0;

//int lcd_find_success=0;
#if 1
bool tp_probe_ok;//bit0
bool camera_front_probe_ok;//bit1
bool camera_back_probe_ok;//bit2
bool gsensor_probe_ok;//bit3
bool proximity_probe_ok;//bit4
bool charger_probe_ok;//bit5
bool pmu_probe_ok=1;//bit6
bool compass_probe_ok;//bit7
bool fingerprint_probe_ok;//bit31
#endif
extern char *mtkfb_find_lcm_driver(void);
#define GPIO_HW_VER_PIN0    (GPIO81 | 0x80000000)
#define GPIO_HW_SD_TRAY_PIN0    (GPIO3 | 0x80000000)
static ssize_t lcd_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
#if 0
	if(lcd_find_success)
		s += sprintf(s, "%s\n",lcm_kernel_detect_drv->name);
	else
		s += sprintf(s, "%s\n",lcd_array);
#endif
	s += sprintf(s, "%s\n",mtkfb_find_lcm_driver());
	return (s - buf);
}

static ssize_t lcd_info_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}
static struct kobj_attribute lcd_info_attr = {
	.attr = {
		.name = "lcd_info",
		.mode = 0644,
	},
	.show =&lcd_info_show,
	.store= &lcd_info_store,
};

extern int sec_schip_enabled(void);
static ssize_t efuse_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	return sprintf(buf, "%d\n", sec_schip_enabled()? 1:0);
}

static struct kobj_attribute sboot_efuse_info_attr = {
	.attr = {
		.name = "hw_efuse_info",
		.mode = 0444,
	},
	.show =&efuse_info_show,
};
static ssize_t back_camera_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

        if(back_camera_find_success==1)
                  s += sprintf(s, "%s\n",back_camera_array);
        else if(back_camera_find_success==2)
                 s += sprintf(s, "%s\n",back_camera_array1);
		else if(back_camera_find_success==3)
			s += sprintf(s, "%s\n",back_camera_array2);
#if 0
        else if(back_camera_find_success==3)
                 s += sprintf(s, "%s\n",back_camera_array2);
        else if(back_camera_find_success==4)
                 s += sprintf(s, "%s\n",back_camera_array3);
#endif
        else
		s += sprintf(s, "%s\n",back_camera);

	return (s - buf);
}

static ssize_t back_camera_info_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}
static struct kobj_attribute back_camera_info_attr = {
	.attr = {
		.name = "back_camera",
		.mode = 0644,
	},
	.show =&back_camera_info_show,
	.store= &back_camera_info_store,
};

static ssize_t front_camera_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

	if(front_camera_find_success==1)
		s += sprintf(s, "%s\n",front_camera_array);
    	else if(front_camera_find_success==2)
		s += sprintf(s, "%s\n",front_camera_array1);
	else if (front_camera_find_success==3)
		s += sprintf(s, "%s\n",front_camera_array2);
#if 0
        	else if(front_camera_find_success==3)
		s += sprintf(s, "%s\n",front_camera_array2);
    	else if(front_camera_find_success==4)
		s += sprintf(s, "%s\n",front_camera_array3);
#endif
	else
		s += sprintf(s, "%s\n",front_camera);

	return (s - buf);
}

static ssize_t front_camera_info_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}
static struct kobj_attribute front_camera_info_attr = {
	.attr = {
		.name = "front_camera",
		.mode = 0644,
	},
	.show =&front_camera_info_show,
	.store= &front_camera_info_store,
};
#if 1
static ssize_t touch_flash_onoff_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	return sprintf(buf, "%d\n", touch_flash_enable ? 1:0);
}
static ssize_t touch_flash_onoff_info_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

    int res=0;
    res = kstrtouint(buf, 10, &touch_flash_enable);

    if(touch_flash_enable==1)
    {
                FL_Enable();
    }
    else
    {
                FL_Disable();
    }
	return n;
}
static struct kobj_attribute touch_flash_onoff_info_attr = {
	.attr = {
		.name = "touch_flash",
		.mode = 0644,
	},
	.show =&touch_flash_onoff_info_show,
	.store= &touch_flash_onoff_info_store,
};
#endif

#if 0
static struct kobj_attribute modem_info_attr = {
	.attr = {
		.name = "modem_info",
		.mode = 0444,
	},
	.show = &modem_show,
};
#endif
//end
#if 0
static ssize_t lcd_driving_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
       unsigned int val;
       int res=0;

	 res = kstrtouint(buf, 10, &val);

	kernel_restart("nff_test");
	//lcm_kernel_detect_drv->esd_check();
	//if(lcd_detect_mipi_info.lcd_set_driving_mode)
	//	lcd_detect_mipi_info.lcd_set_driving_mode(&lcd_detect_mipi_info,val);
	//else
		//printk(KERN_ERR "[kernel]:lcd_set_driving_mode not found!.\n");
	return n;
}

static struct kobj_attribute lcd_driving_mode_set_attr = {
	.attr = {
		.name = "lcd_driving_mode_set_info",
		.mode = 0644,
	},
	.store = &lcd_driving_mode_store,
};
#endif
#if 1
static ssize_t i2c_devices_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
//	u8 string[5]={'\0'};
	int tmp=0;
	tmp|= (tp_probe_ok<<0);
	tmp|= (camera_front_probe_ok<<1);
	tmp|= (camera_back_probe_ok<<2);
	tmp|= (gsensor_probe_ok<<3);
	tmp|= (proximity_probe_ok<<4);
	tmp|= (charger_probe_ok<<5);
	tmp|= (pmu_probe_ok<<6);
	tmp|= (compass_probe_ok<<7);
	tmp|= (fingerprint_probe_ok<<31);
	//itoa((int)tmp,string);
	s += sprintf(s, "0x%x\n",tmp);

	return (s - buf);
}
static struct kobj_attribute i2c_devices_info_attr = {
	.attr = {
		.name = "i2c_devices_probe_info",
		.mode = 0444,
	},
	.show =&i2c_devices_info_show,
};
#endif
//+add by hzb

#if 0
#if  defined (CONFIG_ARM64)  //Titan_TL PRJ
int get_pa_num(void)
{
     //add by pare for modem gpio check
#define GPIO_VERSION_PIN1    (GPIO95 | 0x80000000)
#define GPIO_VERSION_PIN2    (GPIO96 | 0x80000000)
#define GPIO_VERSION_PIN3    (GPIO93 | 0x80000000)
#define GPIO_VERSION_PIN4    (GPIO94 | 0x80000000)

	int pin1_val = 0, pin2_val = 0, pin3_val = 0, pin4_val = 0;

      mt_set_gpio_pull_select(GPIO_VERSION_PIN1, GPIO_PULL_UP);
      mt_set_gpio_pull_enable(GPIO_VERSION_PIN1, GPIO_PULL_ENABLE);
      mt_set_gpio_mode(GPIO_VERSION_PIN1, GPIO_MODE_00);
      mt_set_gpio_dir(GPIO_VERSION_PIN1, GPIO_DIR_IN);

      mt_set_gpio_pull_select(GPIO_VERSION_PIN2, GPIO_PULL_UP);
      mt_set_gpio_pull_enable(GPIO_VERSION_PIN2, GPIO_PULL_ENABLE);
      mt_set_gpio_mode(GPIO_VERSION_PIN2, GPIO_MODE_00);
      mt_set_gpio_dir(GPIO_VERSION_PIN2, GPIO_DIR_IN);

      mt_set_gpio_pull_select(GPIO_VERSION_PIN3, GPIO_PULL_UP);
      mt_set_gpio_pull_enable(GPIO_VERSION_PIN3, GPIO_PULL_ENABLE);
      mt_set_gpio_mode(GPIO_VERSION_PIN3, GPIO_MODE_00);
      mt_set_gpio_dir(GPIO_VERSION_PIN3, GPIO_DIR_IN);

      mt_set_gpio_pull_select(GPIO_VERSION_PIN4, GPIO_PULL_UP);
      mt_set_gpio_pull_enable(GPIO_VERSION_PIN4, GPIO_PULL_ENABLE);
      mt_set_gpio_mode(GPIO_VERSION_PIN4, GPIO_MODE_00);
      mt_set_gpio_dir(GPIO_VERSION_PIN4, GPIO_DIR_IN);

	mdelay(20);

	pin1_val = mt_get_gpio_in(GPIO_VERSION_PIN1);
	pin2_val = mt_get_gpio_in(GPIO_VERSION_PIN2);
	pin3_val = mt_get_gpio_in(GPIO_VERSION_PIN3);
	pin4_val = mt_get_gpio_in(GPIO_VERSION_PIN4);

	printk(KERN_ERR "%s:  pin1 is %d, pin2 is %d, pin3 is %d, pin4 is %d\n",__func__, pin1_val, pin2_val,pin3_val,pin4_val);

      mt_set_gpio_pull_enable(GPIO_VERSION_PIN1, GPIO_PULL_DISABLE);
      mt_set_gpio_pull_enable(GPIO_VERSION_PIN2, GPIO_PULL_DISABLE);
      mt_set_gpio_pull_enable(GPIO_VERSION_PIN3, GPIO_PULL_DISABLE);
      mt_set_gpio_pull_enable(GPIO_VERSION_PIN4, GPIO_PULL_DISABLE);


#if  !defined (CONFIG_MTK_C2K_SUPPORT)  //Titan_TL
     if (pin1_val && pin4_val )
     {
         return 1;
     }
     else
     {
         return 0;
     }
#else
     if (0) //( pin2_val && ( !pin3_val) && ( !pin4_val))
     {
         return 1;
     }
     else
     {
         return 0;
     }
#endif
}
#else
int get_pa_num(void)
{
//baixue add for disable second PA
      return 0;

     //add by pare for modem gpio check
#define GPIO_VERSION_PIN1    (GPIO96 | 0x80000000)

	int pin1_val = 0, pin2_val = 0;

      mt_set_gpio_pull_select(GPIO_VERSION_PIN1, GPIO_PULL_UP);
      mt_set_gpio_pull_enable(GPIO_VERSION_PIN1, GPIO_PULL_ENABLE);
      mt_set_gpio_mode(GPIO_VERSION_PIN1, GPIO_MODE_00);
      mt_set_gpio_dir(GPIO_VERSION_PIN1, GPIO_DIR_IN);

	mdelay(10);

	pin1_val = mt_get_gpio_in(GPIO_VERSION_PIN1);

	printk(KERN_ERR "%s:  pin1 is %d\n",__func__, pin1_val);

      mt_set_gpio_pull_enable(GPIO_VERSION_PIN1, GPIO_PULL_DISABLE);


      return !pin1_val;

}
#endif

static ssize_t RF_PA_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
//	u8 string[5]={'\0'};
	int tmp=0;

	tmp = get_pa_num();

	s += sprintf(s, "%d\n",tmp);

	return (s - buf);
}

static struct kobj_attribute RF_PA_info_attr = {
	.attr = {
		.name = "RF_PA_Type",
		.mode = 0444,
	},
	.show =&RF_PA_info_show,
};
#endif
#if 0
//+add by liujingchuan for check cust prj ver
int get_hw_prj(void)
{
#define GPIO_HW_PRJ_PIN0    (GPIO93 | 0x80000000)
#define GPIO_HW_PRJ_PIN1    (GPIO96 | 0x80000000)
	int pin0_val = 0;
	int  pin1_val = 0;
	int hw_prj=0;

      mt_set_gpio_pull_select(GPIO_HW_PRJ_PIN0, GPIO_PULL_UP);
      mt_set_gpio_pull_enable(GPIO_HW_PRJ_PIN0, GPIO_PULL_ENABLE);
      mt_set_gpio_mode(GPIO_HW_PRJ_PIN0, GPIO_MODE_00);
      mt_set_gpio_dir(GPIO_HW_PRJ_PIN0, GPIO_DIR_IN);

      mt_set_gpio_pull_select(GPIO_HW_PRJ_PIN1, GPIO_PULL_UP);
      mt_set_gpio_pull_enable(GPIO_HW_PRJ_PIN1, GPIO_PULL_ENABLE);
      mt_set_gpio_mode(GPIO_HW_PRJ_PIN1, GPIO_MODE_00);
      mt_set_gpio_dir(GPIO_HW_PRJ_PIN1, GPIO_DIR_IN);

	mdelay(20);

	pin0_val = mt_get_gpio_in(GPIO_HW_PRJ_PIN0);
	pin1_val = mt_get_gpio_in(GPIO_HW_PRJ_PIN1);
	hw_prj = ( pin0_val?1:0);
	hw_prj |= (pin1_val?(1<<1):0);

	printk(KERN_ERR "%s: pin0 is %d , pin1 is %d, hw_prj=%d\n",__func__, pin0_val, pin1_val,hw_prj);

      mt_set_gpio_pull_enable(GPIO_HW_PRJ_PIN0, GPIO_PULL_DISABLE);
      mt_set_gpio_pull_enable(GPIO_HW_PRJ_PIN1, GPIO_PULL_DISABLE);

	return  hw_prj;

}
static ssize_t get_hw_prj_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

      s += sprintf(s, "%d\n",get_hw_prj());

	return (s - buf);

}

static struct kobj_attribute get_hw_prj_attr = {
	.attr = {
		.name = "hw_prj",
		.mode = 0644,
	},
	.show =&get_hw_prj_show,
};
#endif
//-add by liujingchuan for check cust prj ver

#if 1
struct hw_ver {
        const char *name;
        const char *name1;
        const char *ver;
        int hw_type;//0 for vcdt old, 1 vcdt modified
        int voltage_max;
        int voltage_min;
};
static struct hw_ver hw_ver_array[] = {
        {
                .name = "TL00",
                .name1 = "TL10",
                .ver = "EP",
                .hw_type = 0,
                .voltage_max = 1500,
                .voltage_min = 1351,
        },
        {
                .name = "TL00",
                .name1 = "TL10",
                .ver = "EP",
                .hw_type = 1,
                .voltage_max = 1349,
                .voltage_min = 1051,
        },
	    {
                .name = "AL00",
                .name1 = "AL10",
                .ver = "EP",
                .hw_type = 0,
                .voltage_max = 149,
                .voltage_min = 0,
        },
        {
                .name = "AL00",
                .name1 = "AL10",
                .ver = "EP",
                .hw_type = 1,
                .voltage_max = 749,
                .voltage_min = 451,
        },
};
#define BOARD_ID_CHANNEL_NUM 12
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int *rawdata);
struct hw_ver * get_hw_ver_info(void)
{
	int data[4], ret_value = 0, hw_ver=0x0;
	int temp_vol;
	int i = 0;
	int hw_ver_num=(sizeof(hw_ver_array))/(sizeof(hw_ver_array[0]));

	ret_value = IMM_GetOneChannelValue(BOARD_ID_CHANNEL_NUM, data, &hw_ver);
        if (ret_value == -1) {/* AUXADC is busy */
		printk(KERN_ERR "kernel IMM_GetOneChannelValua get raw value error \n");
        }
	temp_vol = hw_ver * 1500 / 4096;
    printk(KERN_ERR "kernel IMM_GetOneChannelValua get vol %d\n", temp_vol);
	for(i = 0;i<hw_ver_num;i++) {
                if((temp_vol>= hw_ver_array[i].voltage_min)&&(temp_vol<= hw_ver_array[i].voltage_max))
                {
                        break;
                }
        }
        if(i>=hw_ver_num)
        {
            return NULL;
        }

	return &hw_ver_array[i];

}
EXPORT_SYMBOL(get_hw_ver_info);
extern unsigned long Ontim_get_mem_size(void);
bool mt_get_mem_ver(void)
{

   mt_set_gpio_pull_select(GPIO_HW_VER_PIN0, GPIO_PULL_UP);
   mt_set_gpio_pull_enable(GPIO_HW_VER_PIN0, GPIO_PULL_ENABLE);
   mt_set_gpio_mode(GPIO_HW_VER_PIN0, GPIO_MODE_00);
   mt_set_gpio_dir(GPIO_HW_VER_PIN0, GPIO_DIR_IN);
   mdelay(20);
   return (mt_get_gpio_in(GPIO_HW_VER_PIN0) == 0);
}
static ssize_t get_hw_ver_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char *s = buf;
    struct hw_ver * hw_ver_info=get_hw_ver_info();
    if( hw_ver_info==NULL)
    {
        s += sprintf(s, "%s\n","invalid hw_ver");
    }
    else
    {
        s += sprintf(s, "%s-JMM-%s\n",hw_ver_info->ver, \
            mt_get_mem_ver() ? hw_ver_info->name : hw_ver_info->name1);
    }
    return (s - buf);
}


static struct kobj_attribute get_hw_ver_attr = {
	.attr = {
		.name = "hw_ver",
		.mode = 0644,
	},
	.show =&get_hw_ver_show,
};
//-add by hzb
#endif

static ssize_t get_equip_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char *s = buf;
#if 0  //modify by hzb
    #define GPIO_HW_VER_PIN0    (GPIO81 | 0x80000000)

    mt_set_gpio_pull_select(GPIO_HW_VER_PIN0, GPIO_PULL_UP);
    mt_set_gpio_pull_enable(GPIO_HW_VER_PIN0, GPIO_PULL_ENABLE);
    mt_set_gpio_mode(GPIO_HW_VER_PIN0, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_HW_VER_PIN0, GPIO_DIR_IN);
    mdelay(20);
    s += sprintf(s, "%s\n",mt_get_gpio_in(GPIO_HW_VER_PIN0) ? "C10" : "C00");
    mt_set_gpio_pull_enable(GPIO_HW_VER_PIN0, GPIO_PULL_DISABLE);
    printk(KERN_ERR "%s: MEM size is %luK \n",__func__,Ontim_get_mem_size());
    s += sprintf(s, "%s\n",(Ontim_get_mem_size() >3145728) ? "C10" : "C00");  //mem size > 3G is C10
#endif
    s += sprintf(s, "%luK\n", Ontim_get_mem_size());
    return (s - buf);

}
/*--------------------------------------------------------------------------------------*/

static struct kobj_attribute get_equip_attr = {
        .attr = {
                .name = "equip",
                .mode = 0644,
        },
        .show =&get_equip_show,
};
//+add by hzb for check cust prj ver
static void check_cust_ver(void){
    int rb_flag = 0;;
    struct hw_ver * hw_ver_info=get_hw_ver_info();
    if ( hw_ver_info == NULL )
    {
    	printk(KERN_ERR "%s: Get PRJ info Error!!!\n",__func__);
        return;
    }
#ifdef PRJ_NAME
    printk(KERN_ERR "%s: Build PRJ is %s, This PRJ is %s!!\n",__func__, PRJ_NAME, hw_ver_info->name);
    if((!strcmp(hw_ver_info->name, PRJ_NAME)) || (!strcmp(hw_ver_info->name, "NULL"))){
    	printk(KERN_ERR "%s: Version Pass!!\n",__func__);
    }
    else
    {
        printk(KERN_ERR "%s: Version Error!!!\n",__func__);
        kernel_restart("prjerr");
    }
#endif
}
//-add by hzb for check cust prj ver
bool mt_get_mem_sd_tray(void)
{

   mt_set_gpio_pull_select(GPIO_HW_SD_TRAY_PIN0, GPIO_PULL_UP);
   mt_set_gpio_pull_enable(GPIO_HW_SD_TRAY_PIN0, GPIO_PULL_ENABLE);
   mt_set_gpio_mode(GPIO_HW_SD_TRAY_PIN0, GPIO_MODE_00);
   mt_set_gpio_dir(GPIO_HW_SD_TRAY_PIN0, GPIO_DIR_IN);
   mdelay(20);
   return (mt_get_gpio_in(GPIO_HW_SD_TRAY_PIN0) == 0);
}
static ssize_t get_hw_sd_tray_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char *s = buf;

    if (mt_get_mem_sd_tray())
        s += sprintf(s, "0 \n");
    else
        s += sprintf(s, "1 \n");
    return (s - buf);
}

static struct kobj_attribute get_hw_sd_tray_attr = {
    .attr = {
        .name = "hw_sd_tray",
        .mode = 0644,
    },
    .show =&get_hw_sd_tray_show,
};

static struct attribute * g[] = {
	//&get_hw_prj_attr.attr,//add by liujingchuan
	&get_hw_ver_attr.attr,//+add by hzb
	&get_equip_attr.attr,
	&lcd_info_attr.attr,//+add by liuwei
	//&modem_info_attr,//+add by youjiangong
	//&lcd_driving_mode_set_attr,//+add by liuwei
	&i2c_devices_info_attr,//+add by liuwei
	&back_camera_info_attr.attr,
	&front_camera_info_attr.attr,
	&touch_flash_onoff_info_attr.attr,
	//&enemmd_attr.attr,//add by youjiangong
	//&RF_PA_info_attr.attr,    //add by hzb
    &get_hw_sd_tray_attr.attr,
    &sboot_efuse_info_attr,//add by yzw
	NULL,
};
static struct attribute_group attr_group = {
	.attrs = g,
};
#if 0
int touchscreen_has_steel_film=0;
static int __init touchscreen_film_setup(char *str)
{
	int en;
	if(!get_option(&str, &en))
		return 0;
	touchscreen_has_steel_film = en;
	return 1;
}

__setup("tp_film=", touchscreen_film_setup);

int get_touchscreen_film_state(void)
{
	printk("[kernel]:touchscreen_has_steel_film=%d.\n",touchscreen_has_steel_film);
	return touchscreen_has_steel_film;
}
#endif
#if 0
int lcm_id=0x83;
static int __init lcm_id_setup(char *str)
{
        int en;
        if(!get_option(&str, &en))
                return 0;
        lcm_id = en;
        return 1;
}
int get_lcm_id(void)
{
        printk("[kernel]:get_lcm_id=%x.\n",lcm_id);
        return lcm_id;
}
__setup("lcm_id=", lcm_id_setup);
#endif
static int __init bootinfo_init(void)
{
	int ret = -ENOMEM;

	//printk("%s,line=%d\n",__func__,__LINE__);

	bootinfo_kobj = kobject_create_and_add("ontim_bootinfo", NULL);

	if (bootinfo_kobj == NULL) {
		printk("bootinfo_init: kobject_create_and_add failed\n");
		goto fail;
	}

	ret = sysfs_create_group(bootinfo_kobj, &attr_group);
	if (ret) {
		printk("bootinfo_init: sysfs_create_group failed\n");
		goto sys_fail;
	}

	return ret;
sys_fail:
	kobject_del(bootinfo_kobj);
fail:
	return ret;

}
static int __init prjinfo_init(void)
{
    printk("bootinfo_init: check_hw_ver Start!\n");
    check_cust_ver();
    return 0;
}
static void __exit bootinfo_exit(void)
{

	if (bootinfo_kobj) {
		sysfs_remove_group(bootinfo_kobj, &attr_group);
		kobject_del(bootinfo_kobj);
	}
}

arch_initcall(bootinfo_init);
device_initcall(prjinfo_init);
module_exit(bootinfo_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Boot information collector");
