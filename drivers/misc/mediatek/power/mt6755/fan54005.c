#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <mt-plat/charging.h>
#include "fan54005.h"


/*added by luhongjiang ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#if 1
#include <ontim/ontim_dev_dgb.h>
static  char charge_ic_vendor_name[50]="FAN54005";
static  char g_Second_charger_ic_name[50]="HL7005D";
DEV_ATTR_DECLARE(charge_ic)
DEV_ATTR_DEFINE("vendor",charge_ic_vendor_name)
DEV_ATTR_DECLARE_END;
ONTIM_DEBUG_DECLARE_AND_INIT(charge_ic,charge_ic,8);
#endif


#define fan54005_SLAVE_ADDR_WRITE_ERROR   0xDA
#define fan54005_SLAVE_ADDR_READ    0xD5
#define fan54005_SLAVE_ADDR_WRITE   0xD4

static struct i2c_client *new_client;
static const struct i2c_device_id fan54005_i2c_id[] = { {"fan54005", 0}, {} };

kal_bool chargin_hw_init_done = KAL_FALSE;
//extern kal_bool chargin_hw_init_done;
static int fan54005_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

#ifdef CONFIG_OF
static const struct of_device_id fan54005_of_match[] = {
	{.compatible = "fan54005",},
	{},
};

MODULE_DEVICE_TABLE(of, fan54005_of_match);
#endif

static struct i2c_driver fan54005_driver = {
	.driver = {
		   .name = "fan54005",
#ifdef CONFIG_OF
		   .of_match_table = fan54005_of_match,
#endif
	},
	.probe = fan54005_driver_probe,
	.id_table = fan54005_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char fan54005_reg[fan54005_REG_NUM] = { 0 };

static DEFINE_MUTEX(fan54005_i2c_access);

int g_fan54005_hw_exist = -1;

/**********************************************************
  *
  *   [I2C Function For Read/Write fan54005]
  *
  *********************************************************/
int fan54005_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char cmd_buf[1] = { 0x00 };
	char readData = 0;
	int ret = 0;

	mutex_lock(&fan54005_i2c_access);

	new_client->ext_flag =
	    ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

	cmd_buf[0] = cmd;
	ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		new_client->ext_flag = 0;

		mutex_unlock(&fan54005_i2c_access);
		return 0;
	}

	readData = cmd_buf[0];
	*returnData = readData;

	new_client->ext_flag = 0;

	mutex_unlock(&fan54005_i2c_access);
	return 1;
}

int fan54005_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;

	mutex_lock(&fan54005_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;

	printk(KERN_ERR "%s: write Reg[0x%x]=0x%x\n",__func__, cmd, writeData);

	new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;

	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {

		new_client->ext_flag = 0;
		mutex_unlock(&fan54005_i2c_access);
		return 0;
	}

	new_client->ext_flag = 0;
	mutex_unlock(&fan54005_i2c_access);
	return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int fan54005_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char fan54005_reg = 0;
	int ret = 0;

	ret = fan54005_read_byte(RegNum, &fan54005_reg);

	battery_log(BAT_LOG_FULL, "[fan54005_read_interface] Reg[%x]=0x%x\n", RegNum, fan54005_reg);

	fan54005_reg &= (MASK << SHIFT);
	*val = (fan54005_reg >> SHIFT);

	//battery_log(BAT_LOG_CRTI, "[fan54005_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int fan54005_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char fan54005_reg = 0;
	int ret = 0;

	ret = fan54005_read_byte(RegNum, &fan54005_reg);
	battery_log(BAT_LOG_FULL, "[fan54005_config_interface] Reg[%x]=0x%x\n", RegNum, fan54005_reg);

	fan54005_reg &= ~(MASK << SHIFT);
	fan54005_reg |= (val << SHIFT);

	if (RegNum == fan54005_CON4 && val == 1 && MASK == CON4_RESET_MASK
	    && SHIFT == CON4_RESET_SHIFT) {
		/* RESET bit */
	} else if (RegNum == fan54005_CON4) {
		fan54005_reg &= ~0x80;	/* RESET bit read returs 1, so clear it */
        //battery_log(BAT_LOG_CRTI, "[fan54005_config_interface] fan54005_reg:0x%x; line:%d\n",fan54005_reg,__LINE__);

	}

	ret = fan54005_write_byte(RegNum, fan54005_reg);
	battery_log(BAT_LOG_FULL, "[fan54005_config_interface] write Reg[%x]=0x%x\n", RegNum,
		    fan54005_reg);

	return ret;
}

/* write one register directly */
unsigned int fan54005_reg_config_interface(unsigned char RegNum, unsigned char val)
{
	int ret = 0;

	ret = fan54005_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0 */

void fan54005_set_tmr_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_TMR_RST_MASK),
				       (unsigned char) (CON0_TMR_RST_SHIFT)
	    );
}

unsigned int fan54005_get_otg_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON0),
				     (&val), (unsigned char) (CON0_OTG_MASK),
				     (unsigned char) (CON0_OTG_SHIFT)
	    );
	return val;
}

void fan54005_set_en_stat(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_STAT_MASK),
				       (unsigned char) (CON0_EN_STAT_SHIFT)
	    );
}

unsigned int fan54005_get_chip_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON0),
				     (&val), (unsigned char) (CON0_STAT_MASK),
				     (unsigned char) (CON0_STAT_SHIFT)
	    );
	return val;
}

unsigned int fan54005_get_boost_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON0),
				     (&val), (unsigned char) (CON0_BOOST_MASK),
				     (unsigned char) (CON0_BOOST_SHIFT)
	    );
	return val;
}

unsigned int fan54005_get_fault_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON0),
				     (&val), (unsigned char) (CON0_FAULT_MASK),
				     (unsigned char) (CON0_FAULT_SHIFT)
	    );
	return val;
}

/* CON1 */

void fan54005_set_input_charging_current(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_LIN_LIMIT_MASK),
				       (unsigned char) (CON1_LIN_LIMIT_SHIFT)
	    );
}

void fan54005_set_v_low(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_LOW_V_MASK),
				       (unsigned char) (CON1_LOW_V_SHIFT)
	    );
}

void fan54005_set_te(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_TE_MASK),
				       (unsigned char) (CON1_TE_SHIFT)
	    );
}

void fan54005_set_ce(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_CE_MASK),
				       (unsigned char) (CON1_CE_SHIFT)
	    );
}

void fan54005_set_hz_mode(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_HZ_MODE_MASK),
				       (unsigned char) (CON1_HZ_MODE_SHIFT)
	    );
}

void fan54005_set_opa_mode(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_OPA_MODE_MASK),
				       (unsigned char) (CON1_OPA_MODE_SHIFT)
	    );
}

/* CON2 */

void fan54005_set_oreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OREG_MASK),
				       (unsigned char) (CON2_OREG_SHIFT)
	    );
}

void fan54005_set_otg_pl(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OTG_PL_MASK),
				       (unsigned char) (CON2_OTG_PL_SHIFT)
	    );
}

void fan54005_set_otg_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OTG_EN_MASK),
				       (unsigned char) (CON2_OTG_EN_SHIFT)
	    );
}

/* CON3 */

unsigned int fan54005_get_vender_code(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON3),
				     (&val), (unsigned char) (CON3_VENDER_CODE_MASK),
				     (unsigned char) (CON3_VENDER_CODE_SHIFT)
	    );
	return val;
}

unsigned int fan54005_get_pn(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON3),
				     (&val), (unsigned char) (CON3_PIN_MASK),
				     (unsigned char) (CON3_PIN_SHIFT)
	    );
	return val;
}

unsigned int fan54005_get_revision(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON3),
				     (&val), (unsigned char) (CON3_REVISION_MASK),
				     (unsigned char) (CON3_REVISION_SHIFT)
	    );
	return val;
}

/* CON4 */

void fan54005_set_reset(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_RESET_MASK),
				       (unsigned char) (CON4_RESET_SHIFT)
	    );
}

void fan54005_set_iocharge(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_I_CHR_MASK),
				       (unsigned char) (CON4_I_CHR_SHIFT)
	    );
}

void fan54005_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_I_TERM_MASK),
				       (unsigned char) (CON4_I_TERM_SHIFT)
	    );
}

/* CON5 */

void fan54005_set_dis_vreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_DIS_VREG_MASK),
				       (unsigned char) (CON5_DIS_VREG_SHIFT)
	    );
}

void fan54005_set_io_level(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_IO_LEVEL_MASK),
				       (unsigned char) (CON5_IO_LEVEL_SHIFT)
	    );
}

unsigned int fan54005_get_sp_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON5),
				     (&val), (unsigned char) (CON5_SP_STATUS_MASK),
				     (unsigned char) (CON5_SP_STATUS_SHIFT)
	    );
	return val;
}

unsigned int fan54005_get_en_level(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON5),
				     (&val), (unsigned char) (CON5_EN_LEVEL_MASK),
				     (unsigned char) (CON5_EN_LEVEL_SHIFT)
	    );
	return val;
}
int fan54005_set_vsp(unsigned int val)
{
    int ret = 0;
    ret = fan54005_config_interface((unsigned char) (fan54005_CON5),
                        (unsigned char) (val),
                        (unsigned char) (CON5_VSP_MASK),
                        (unsigned char) (CON5_VSP_SHIFT)
                        );
    return ret;
}
/* CON6 */

void fan54005_set_i_safe(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_ISAFE_MASK),
				       (unsigned char) (CON6_ISAFE_SHIFT)
	    );
}

void fan54005_set_v_safe(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VSAFE_MASK),
				       (unsigned char) (CON6_VSAFE_SHIFT)
	    );
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
  static char *g_Second_charger_ic=NULL;
void fan54005_hw_component_detect(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface(0x03, &val, 0xFF, 0x0);
	if (val == 0x41)
	{
           g_Second_charger_ic=g_Second_charger_ic_name;
	}
	if (val == 0x94 ||val == 0x41)  //0x94--FAN54005; 0x41--HL7005D
		g_fan54005_hw_exist = 0;
	else
		g_fan54005_hw_exist = 1;

	battery_log(BAT_LOG_CRTI, "[fan54005_hw_component_detect] exist=%d, Reg[03]=0x%x\n", g_fan54005_hw_exist, val);
}

int is_fan54005_exist(void)
{
	battery_log(BAT_LOG_CRTI, "[is_fan54005_exist] g_fan54005_hw_exist=%d\n", g_fan54005_hw_exist);

	return g_fan54005_hw_exist;
}

void fan54005_dump_register(void)
{
	int i = 0;

    printk("[fan54005] ");
	for (i = 0; i < fan54005_REG_NUM; i++) {
		fan54005_read_byte(i, &fan54005_reg[i]);
        printk("[0x%x]=0x%x ", i, fan54005_reg[i]);
	}
    printk("\n");
}
char * ontim_get_g_Second_charger_ic(void)
{
	return g_Second_charger_ic;
}
static int fan54005_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
        int ret = 0;
	new_client = client;
        new_client->addr = fan54005_SLAVE_ADDR_WRITE;
        new_client->addr =  new_client->addr >>1;

        if(CHECK_THIS_DEV_DEBUG_AREADY_EXIT()==0)
        { 
            return -EIO;
        }
	fan54005_hw_component_detect();
         fan54005_dump_register();
        if(is_fan54005_exist()==0)
        {
            if(g_Second_charger_ic)
            {
                strcpy(charge_ic_vendor_name,g_Second_charger_ic);
            }
            REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();
             chargin_hw_init_done = KAL_TRUE;
        }
        else
        {
	    chargin_hw_init_done = KAL_TRUE;
            ret =0;//is_fan54005_exist = 1 //ret = -1;
        }
        battery_log(BAT_LOG_CRTI, "fan54005_driver_probe  line=%d\n", __LINE__);

	return ret;
}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_fan54005 = 0;
static ssize_t show_fan54005_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_log(BAT_LOG_CRTI, "[show_fan54005_access] 0x%x\n", g_reg_value_fan54005);
	return sprintf(buf, "%u\n", g_reg_value_fan54005);
}

static ssize_t store_fan54005_access(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	battery_log(BAT_LOG_CRTI, "[store_fan54005_access]\n");

	if (buf != NULL && size != 0) {

		pvalue = (char *)buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			ret = kstrtou32(addr, 16, (unsigned int *)&reg_address);
		} else
			ret = kstrtou32(pvalue, 16, (unsigned int *)&reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			ret = kstrtou32(val, 16, (unsigned int *)&reg_value);

			battery_log(BAT_LOG_CRTI,
			    "[store_fan54005_access] write fan54005 reg 0x%x with value 0x%x !\n",
			     reg_address, reg_value);
			ret = fan54005_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = fan54005_read_interface(reg_address, &g_reg_value_fan54005, 0xFF, 0x0);
			battery_log(BAT_LOG_CRTI,
			    "[store_fan54005_access] read fan54005 reg 0x%x with value 0x%x !\n",
			     reg_address, g_reg_value_fan54005);
			battery_log(BAT_LOG_CRTI,
			    "[store_fan54005_access] Please use \"cat fan54005_access\" to get value\r\n");
		}
	}
	return size;
}

static DEVICE_ATTR(fan54005_access, 0664, show_fan54005_access, store_fan54005_access);	/* 664 */

static int fan54005_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	battery_log(BAT_LOG_CRTI, "******** fan54005_user_space_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_fan54005_access);

	return 0;
}

struct platform_device fan54005_user_space_device = {
	.name = "fan54005-user",
	.id = -1,
};

static struct platform_driver fan54005_user_space_driver = {
	.probe = fan54005_user_space_probe,
	.driver = {
		   .name = "fan54005-user",
	},
};

#define FAN54005_BUSNUM 1
static struct i2c_board_info __initdata i2c_fan54005 = { I2C_BOARD_INFO("fan54005", (fan54005_SLAVE_ADDR_WRITE_ERROR>>1))};
static int __init fan54005_init(void)
{
	int ret = 0;
	struct device_node *node = of_find_compatible_node(NULL, NULL, "fan54005");

	battery_log(BAT_LOG_CRTI,"[fan54005_init] init start\n");

	if (!node)  i2c_register_board_info(FAN54005_BUSNUM, &i2c_fan54005, 1);

	if (i2c_add_driver(&fan54005_driver) != 0) {
		battery_log(BAT_LOG_CRTI,
			    "[fan54005_init] failed to register fan54005 i2c driver.\n");
	} else {
		battery_log(BAT_LOG_CRTI,
			    "[fan54005_init] Success to register fan54005 i2c driver.\n");
	}

	/* fan54005 user space access interface */
	ret = platform_device_register(&fan54005_user_space_device);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[fan54005_init] Unable to device register(%d)\n",
			    ret);
		return ret;
	}
	ret = platform_driver_register(&fan54005_user_space_driver);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[fan54005_init] Unable to register driver (%d)\n",
			    ret);
		return ret;
	}

	return 0;
}

static void __exit fan54005_exit(void)
{
	i2c_del_driver(&fan54005_driver);
}

subsys_initcall(fan54005_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C fan54005 Driver");
MODULE_AUTHOR("James Lo<james.lo@mediatek.com>");
