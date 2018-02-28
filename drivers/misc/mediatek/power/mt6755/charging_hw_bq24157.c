#include <linux/types.h>
#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <mt-plat/mt_boot.h>
#include <mt-plat/battery_common.h>
#include <mach/mt_charging.h>
#include <mach/mt_pmic.h>
#include "bq24157.h"
//+add by hzb for dsm
#ifdef CONFIG_ONTIM_DSM	

#include <ontim/ontim_dsm.h>

struct dsm_dev bq24157_dsm_dev=
{
	.type=OMTIM_DSM_DEV_TYPE_POWER,
	.id=OMTIM_DSM_DEV_ID_CHARGER,
	.name="Fan5405_charger",
	.buff_size=1024,
};
struct dsm_client *bq24157_dsm_client=NULL;
#endif
//-add by hzb for dsm

/* ============================================================ // */
/* Define */
/* ============================================================ // */
#define STATUS_OK	0
#define STATUS_FAIL	1
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))


/* ============================================================ // */
/* Global variable */
/* ============================================================ // */

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
#define WIRELESS_CHARGER_EXIST_STATE 0
int wireless_charger_gpio_number = (168 | 0x80000000);
#endif

const u32 BQ24157_VBAT_CV_VTH[] = {
	BATTERY_VOLT_03_500000_V, BATTERY_VOLT_03_520000_V, BATTERY_VOLT_03_540000_V,
	    BATTERY_VOLT_03_560000_V,
	BATTERY_VOLT_03_580000_V, BATTERY_VOLT_03_600000_V, BATTERY_VOLT_03_620000_V,
	    BATTERY_VOLT_03_640000_V,
	BATTERY_VOLT_03_660000_V, BATTERY_VOLT_03_680000_V, BATTERY_VOLT_03_700000_V,
	    BATTERY_VOLT_03_720000_V,
	BATTERY_VOLT_03_740000_V, BATTERY_VOLT_03_760000_V, BATTERY_VOLT_03_780000_V,
	    BATTERY_VOLT_03_800000_V,
	BATTERY_VOLT_03_820000_V, BATTERY_VOLT_03_840000_V, BATTERY_VOLT_03_860000_V,
	    BATTERY_VOLT_03_880000_V,
	BATTERY_VOLT_03_900000_V, BATTERY_VOLT_03_920000_V, BATTERY_VOLT_03_940000_V,
	    BATTERY_VOLT_03_960000_V,
	BATTERY_VOLT_03_980000_V, BATTERY_VOLT_04_000000_V, BATTERY_VOLT_04_020000_V,
	    BATTERY_VOLT_04_040000_V,
	BATTERY_VOLT_04_060000_V, BATTERY_VOLT_04_080000_V, BATTERY_VOLT_04_100000_V,
	    BATTERY_VOLT_04_120000_V,
	BATTERY_VOLT_04_140000_V, BATTERY_VOLT_04_160000_V, BATTERY_VOLT_04_180000_V,
	    BATTERY_VOLT_04_200000_V,
	BATTERY_VOLT_04_220000_V, BATTERY_VOLT_04_240000_V, BATTERY_VOLT_04_260000_V,
	    BATTERY_VOLT_04_280000_V,
	BATTERY_VOLT_04_300000_V, BATTERY_VOLT_04_320000_V, BATTERY_VOLT_04_340000_V,
	    BATTERY_VOLT_04_360000_V,
	BATTERY_VOLT_04_380000_V, BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_420000_V,
	    BATTERY_VOLT_04_440000_V
};

const u32 BQ24157_CS_VTH[] = {
	CHARGE_CURRENT_550_00_MA, CHARGE_CURRENT_650_00_MA, CHARGE_CURRENT_750_00_MA,
	    CHARGE_CURRENT_850_00_MA,
	CHARGE_CURRENT_950_00_MA, CHARGE_CURRENT_1050_00_MA, CHARGE_CURRENT_1150_00_MA,
	    CHARGE_CURRENT_1250_00_MA
};

const u32 BQ24157_INPUT_CS_VTH[] = {
	CHARGE_CURRENT_100_00_MA, CHARGE_CURRENT_500_00_MA, CHARGE_CURRENT_800_00_MA,
	    CHARGE_CURRENT_MAX
};

const u32 BQ24157_VCDT_HV_VTH[] = {
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V, BATTERY_VOLT_04_300000_V,
	    BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V, BATTERY_VOLT_04_500000_V,
	    BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V, BATTERY_VOLT_06_500000_V,
	    BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V, BATTERY_VOLT_09_500000_V,
	    BATTERY_VOLT_10_500000_V
};

static u32 charging_value_to_parameter(const u32 *parameter, const u32 array_size, const u32 val)
{
	if (val < array_size)
		return parameter[val];
	battery_log(BAT_LOG_CRTI, "Can't find the parameter \r\n");
	return parameter[0];
}

static u32 charging_parameter_to_value(const u32 *parameter, const u32 array_size, const u32 val)
{
	u32 i;

	for (i = 0; i < array_size; i++)
		if (val == *(parameter + i))
			return i;

	battery_log(BAT_LOG_CRTI, "NO register value match \r\n");

	return 0;
}


static u32 bmt_find_closest_level(const u32 *pList, u32 number, u32 level)
{
	u32 i;
	u32 max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = KAL_TRUE;
	else
		max_value_in_last_element = KAL_FALSE;

	if (max_value_in_last_element == KAL_TRUE) {
		for (i = (number - 1); i != 0; i--)	/* max value in the last element */
			if (pList[i] <= level)
				return pList[i];

		battery_log(BAT_LOG_CRTI, "Can't find closest level, small value first \r\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		for (i = 0; i < number; i++)	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];

		battery_log(BAT_LOG_CRTI, "Can't find closest level, large value first \r\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}
extern char * ontim_get_g_Ansemi_charger_ic(void);

static int charging_hw_init(void *data)
{
	u32 status = STATUS_OK;
	static bool charging_init_flag = KAL_FALSE;

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	mt_set_gpio_mode(wireless_charger_gpio_number, 0);	/* 0:GPIO mode */
	mt_set_gpio_dir(wireless_charger_gpio_number, 0);	/* 0: input, 1: output */
#endif
//battery_log(BAT_LOG_CRTI, "bq24157  charging_hw_init  line=%d\n", __LINE__);

	#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	bq24157_reg_config_interface(0x06,0x57); // ISAFE = 1050mA, VSAFE = 4.34V
	#else
	bq24157_reg_config_interface(0x06,0x70);
	#endif
	    
	bq24157_reg_config_interface(0x00,0xC0);	//kick chip watch dog
	if(ontim_get_g_Ansemi_charger_ic())
	{
		bq24157_reg_config_interface(0x01,0xf8);	//TE=1, CE=0, HZ_MODE=0, OPA_MODE=0
	}
	else
	{
		bq24157_reg_config_interface(0x01,0xb8);	//TE=1, CE=0, HZ_MODE=0, OPA_MODE=0
	}
	if(ontim_get_g_Ansemi_charger_ic())
		bq24157_reg_config_interface(0x05,0x05);
	else
		bq24157_reg_config_interface(0x05,0x03);
	if ( !charging_init_flag ) {   
		bq24157_reg_config_interface(0x04,0x59); //97mA
		charging_init_flag = KAL_TRUE;
	}
#ifdef CONFIG_ONTIM_DSM	
	bq24157_dsm_client=dsm_register_client (&bq24157_dsm_dev);
#endif
	return status;
}


static int charging_dump_register(void *data)
{
	u32 status = STATUS_OK;

	bq24157_dump_register();

	return status;
}


static int charging_enable(void *data)
{
	u32 status = STATUS_OK;
	u32 enable = *(u32 *) (data);

	if (KAL_TRUE == enable) {
		bq24157_set_ce(0);
		bq24157_set_hz_mode(0);
		bq24157_set_opa_mode(0);
	} else {

#if defined(CONFIG_USB_MTK_HDRC_HCD)
		if (mt_usb_is_device())
#endif

			bq24157_set_ce(1);
	}

	return status;
}


static int charging_set_cv_voltage(void *data)
{
	u32 status = STATUS_OK;
	u16 register_value;

	register_value =
	    charging_parameter_to_value(BQ24157_VBAT_CV_VTH, GETARRAYNUM(BQ24157_VBAT_CV_VTH), *(u32 *) (data));
	bq24157_set_oreg(register_value);

	return status;
}


static int charging_get_current(void *data)
{
	u32 status = STATUS_OK;
	u32 array_size;
	u8 reg_value;

	/* Get current level */
	array_size = GETARRAYNUM(BQ24157_CS_VTH);
	bq24157_read_interface(0x1, &reg_value, 0x3, 0x6);	/* IINLIM */
	*(u32 *) data = charging_value_to_parameter(BQ24157_CS_VTH, array_size, reg_value);

	return status;
}



static int charging_set_current(void *data)
{
	u32 status = STATUS_OK;
	u32 set_chr_current;
	u32 array_size;
	u32 register_value;
	u32 current_value = *(u32 *) data;
	if((ontim_get_g_Ansemi_charger_ic())&&(current_value>CHARGE_CURRENT_1000_00_MA))
	{
	      current_value=CHARGE_CURRENT_1000_00_MA;
	}
	if (current_value <= CHARGE_CURRENT_350_00_MA) {
		bq24157_set_io_level(1);
	} else {
		bq24157_set_io_level(0);
		array_size = GETARRAYNUM(BQ24157_CS_VTH);
		set_chr_current = bmt_find_closest_level(BQ24157_CS_VTH, array_size, current_value);
        
	//battery_log(BAT_LOG_CRTI, "charging_set_current  set_chr_current=%d\n", set_chr_current);
        
		register_value = charging_parameter_to_value(BQ24157_CS_VTH, array_size, set_chr_current);
	//battery_log(BAT_LOG_CRTI, "charging_set_current  register_value=%d\n", register_value);
		bq24157_set_iocharge(register_value);
	}
	return status;
}


static int charging_set_input_current(void *data)
{
	u32 status = STATUS_OK;
	u32 set_chr_current;
	u32 array_size;
	u32 register_value;

	if (*(u32 *) data > CHARGE_CURRENT_500_00_MA) {
		register_value = 0x3;
	//battery_log(BAT_LOG_CRTI, "charging_set_input_current  register_value=%d\n", register_value);
	} else {
		array_size = GETARRAYNUM(BQ24157_INPUT_CS_VTH);
		set_chr_current = bmt_find_closest_level(BQ24157_INPUT_CS_VTH, array_size, *(u32 *) data);
	//battery_log(BAT_LOG_CRTI, "charging_set_input_current  set_chr_current=%d\n", set_chr_current);
		register_value =
		    charging_parameter_to_value(BQ24157_INPUT_CS_VTH, array_size, set_chr_current);
	//battery_log(BAT_LOG_CRTI, "charging_set_input_current  register_value=%d\n", register_value);
	}
	//if(ontim_get_g_Ansemi_charger_ic()==NULL)
		bq24157_set_input_charging_current(register_value);

	return status;
}


static int charging_get_charging_status(void *data)
{
	u32 status = STATUS_OK;
	u32 ret_val;

	ret_val = bq24157_get_chip_status();

	if (ret_val == 0x2)
		*(u32 *) data = KAL_TRUE;
	else
		*(u32 *) data = KAL_FALSE;

	return status;
}


static int charging_reset_watch_dog_timer(void *data)
{
	u32 status = STATUS_OK;

#ifdef CONFIG_ONTIM_DSM	
	 if (bq24157_dsm_client && (bq24157_get_chip_status()==0x03))
	 {
	 	u8 reg[8];
		int i;
		for (i=0;i<7;i++)
		{
		    bq24157_read_interface(i, &reg[i],0xFF,0);
		}
		bq24157_read_interface(0x10,&reg[i],0xFF,0);
	 	if ( (bq24157_dsm_client ) && dsm_client_ocuppy(bq24157_dsm_client))
	 	{
			int error=OMTIM_DSM_CHARGER_ERROR;
	 		if ((bq24157_dsm_client->dump_buff) && (bq24157_dsm_client->buff_size)&&(bq24157_dsm_client->buff_flag == OMTIM_DSM_BUFF_OK))
	 		{
				bq24157_dsm_client->used_size = sprintf(bq24157_dsm_client->dump_buff,"Type=%d; ID=%d; error_id=%d;  Charger info:%s; ",bq24157_dsm_client->client_type,bq24157_dsm_client->client_id,error,bq24157_dsm_client->client_name);
				for (i=0;i<7;i++)
				{
					bq24157_dsm_client->used_size += sprintf(bq24157_dsm_client->dump_buff+bq24157_dsm_client->used_size,"Reg[0x%x]=0x%x; ",i,reg[i]);
				}
				bq24157_dsm_client->used_size += sprintf(bq24157_dsm_client->dump_buff+bq24157_dsm_client->used_size,"Reg[0x10]=0x%x; ",reg[i]);
				bq24157_dsm_client->used_size += sprintf(bq24157_dsm_client->dump_buff+bq24157_dsm_client->used_size,"\n");
				dsm_client_notify(bq24157_dsm_client,error);
	 		}
	 	}
		else
		{
			printk(KERN_ERR "%s: dsm ocuppy error!!!",__func__);
		}
		
		
	 }
#endif
 
	bq24157_set_tmr_rst(1);

	return status;
}


static int charging_set_hv_threshold(void *data)
{
	u32 status = STATUS_OK;

	u32 set_hv_voltage;
	u32 array_size;
	u16 register_value;
	u32 voltage = *(u32 *) (data);

	array_size = GETARRAYNUM(BQ24157_VCDT_HV_VTH);
	set_hv_voltage = bmt_find_closest_level(BQ24157_VCDT_HV_VTH, array_size, voltage);
	register_value = charging_parameter_to_value(BQ24157_VCDT_HV_VTH, array_size, set_hv_voltage);
	pmic_set_register_value(PMIC_RG_VCDT_HV_VTH, register_value);
	return status;
}


static int charging_get_hv_status(void *data)
{
	u32 status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 0;
#else
	*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_VCDT_HV_DET);
#endif
	return status;
}


static int charging_get_battery_status(void *data)
{
	unsigned int status = STATUS_OK;

#if 1 //defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 0;	/* battery exist */
	//battery_log(BAT_LOG_CRTI, "[charging_get_battery_status] battery exist for bring up.\n");
#else
	unsigned int val = 0;

	val = pmic_get_register_value(PMIC_BATON_TDET_EN);
	battery_log(BAT_LOG_FULL, "[charging_get_battery_status] BATON_TDET_EN = %d\n", val);
	if (val) {
		pmic_set_register_value(PMIC_BATON_TDET_EN, 1);
		pmic_set_register_value(PMIC_RG_BATON_EN, 1);
		*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_BATON_UNDET);
	} else {
		*(kal_bool *) (data) = KAL_FALSE;
	}
#endif

	return status;
}


static int charging_get_charger_det_status(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int val = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	val = 1;
	battery_log(BAT_LOG_CRTI, "[charging_get_charger_det_status] chr exist for fpga.\n");
#else
	val = pmic_get_register_value(PMIC_RGS_CHRDET);
#endif

	*(kal_bool *) (data) = val;

	return status;
}

static int charging_get_charger_type(void *data)
{
	u32 status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(CHARGER_TYPE *) (data) = STANDARD_HOST;
#else
	*(CHARGER_TYPE *) (data) = hw_charging_get_charger_type();
#endif

	return status;
}

static int charging_get_is_pcm_timer_trigger(void *data)
{
	u32 status = STATUS_OK;
/* M migration
	if (slp_get_wake_reason() == WR_PCM_TIMER)
		*(kal_bool *) (data) = KAL_TRUE;
	else
		*(kal_bool *) (data) = KAL_FALSE;
	battery_log(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n", slp_get_wake_reason());
*/
	*(kal_bool *)(data) = KAL_FALSE;
	return status;
}

static int charging_set_platform_reset(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	battery_log(BAT_LOG_CRTI, "charging_set_platform_reset\n");

	kernel_restart("battery service reboot system");
	/* arch_reset(0,NULL); */
#endif

	return status;
}

static int charging_get_platform_boot_mode(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	*(unsigned int *) (data) = get_boot_mode();

	battery_log(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());
#endif

	return status;
}

static int charging_set_power_off(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	battery_log(BAT_LOG_CRTI, "charging_set_power_off\n");
	kernel_power_off();
#endif

	return status;
}

static int charging_get_power_source(void *data)
{
	u32 status = STATUS_UNSUPPORTED;

	return status;
}

static int charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;
}

static int charging_set_ta_current_pattern(void *data)
{
	return STATUS_UNSUPPORTED;
}

static int charging_set_error_state(void *data)
{
	return STATUS_UNSUPPORTED;
}

static int charging_set_hz_mode(void *data)
{
	unsigned int status = STATUS_OK;
        unsigned int enable = *(unsigned int *) (data);

        bq24157_set_hz_mode(enable);

        return status;
}

static int (* charging_func[CHARGING_CMD_NUMBER]) (void *data) = {};

int chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
    static int frist_run=1;
    s32 status;
    if (frist_run)
    {
        frist_run = 0;
        memset((void*)charging_func,0,sizeof(charging_func));
        charging_func[CHARGING_CMD_INIT]=charging_hw_init;
        charging_func[CHARGING_CMD_DUMP_REGISTER]=charging_dump_register;
        charging_func[CHARGING_CMD_ENABLE]=charging_enable;
        charging_func[CHARGING_CMD_SET_CV_VOLTAGE]=charging_set_cv_voltage;
        charging_func[CHARGING_CMD_GET_CURRENT]=charging_get_current;
        charging_func[CHARGING_CMD_SET_CURRENT]=charging_set_current;
        charging_func[CHARGING_CMD_SET_INPUT_CURRENT]=charging_set_input_current;
        charging_func[CHARGING_CMD_GET_CHARGING_STATUS]=charging_get_charging_status;
        charging_func[CHARGING_CMD_RESET_WATCH_DOG_TIMER]=charging_reset_watch_dog_timer;
        charging_func[CHARGING_CMD_SET_HV_THRESHOLD]=charging_set_hv_threshold;
        charging_func[CHARGING_CMD_GET_HV_STATUS]=charging_get_hv_status;
        charging_func[CHARGING_CMD_GET_BATTERY_STATUS]=charging_get_battery_status;
        charging_func[CHARGING_CMD_GET_CHARGER_DET_STATUS]=charging_get_charger_det_status;
        charging_func[CHARGING_CMD_GET_CHARGER_TYPE]=charging_get_charger_type;
        charging_func[CHARGING_CMD_GET_IS_PCM_TIMER_TRIGGER]=charging_get_is_pcm_timer_trigger;
        charging_func[CHARGING_CMD_SET_PLATFORM_RESET]=charging_set_platform_reset;
        charging_func[CHARGING_CMD_GET_PLATFORM_BOOT_MODE]=charging_get_platform_boot_mode;
        charging_func[CHARGING_CMD_SET_POWER_OFF]=charging_set_power_off;
        charging_func[CHARGING_CMD_GET_POWER_SOURCE]=charging_get_power_source;
        charging_func[CHARGING_CMD_GET_CSDAC_FALL_FLAG]=charging_get_csdac_full_flag;
        charging_func[CHARGING_CMD_SET_TA_CURRENT_PATTERN]=charging_set_ta_current_pattern;
        charging_func[CHARGING_CMD_SET_ERROR_STATE]=charging_set_error_state;
        charging_func[CHARGING_CMD_SET_HZ_MODE]=charging_set_hz_mode;
    }
	if (cmd < CHARGING_CMD_NUMBER)
		if (charging_func[cmd] != NULL)
                        status = charging_func[cmd](data);
                else {
                        battery_log(BAT_LOG_CRTI, "[chr_control_interface]cmd:%d not supported\n", cmd);
                        status = STATUS_UNSUPPORTED;
		}
	else
		return STATUS_UNSUPPORTED;

	return status;
}
