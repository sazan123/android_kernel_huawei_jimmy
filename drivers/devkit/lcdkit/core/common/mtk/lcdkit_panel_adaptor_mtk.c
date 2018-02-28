#include "lcdkit_panel.h"
#include "lcdkit_dbg.h"
#include "lcdkit_parse.h"
#include "ddp_dsi.h"
#include "primary_display.h"

/*
*name:lcdkit_info_init
*function:lcd_panel_info init
*@pinfo:lcd panel info
*/
void lcdkit_info_init(void* pdata)
{
   return;
}

int lcdkit_cmds_to_dsi_cmds(struct lcdkit_dsi_panel_cmds* lcdkit_cmds, struct dsi_cmd_desc* cmd)
{
    return 0;
}

/*
 *  dsi send cmds
*/
void lcdkit_dsi_tx(unsigned int address, unsigned char value)
{
    DSI_set_cmdq_V2_DSI0(NULL,address,1,&value,1);
    LCDKIT_INFO("dsi write:address=0x%0x,value=0x%0x\n",address,value);
}

void lcdkit_dsi_rx(unsigned int address,unsigned char *value)
{
    unsigned char buffer[2]={};
    fbconfig_get_esd_check_test(0,address,buffer,1);
    *value = buffer[0];
    LCDKIT_INFO("dsi read:address=0x%0x,value=0x%0x\n",address,*value);
}
void lcdkit_dsi_rx_block_data(unsigned int address,unsigned char *value,unsigned int num)
{
    fbconfig_get_esd_check_test(0,address,value,num);
    LCDKIT_INFO("dsi read:address=0x%0x,value=0x%0x\n",address,*value);
}

/*switch lp to hs or hs to lp*/
void lcdkit_switch_hs_lp(void* pdata, bool enable)
{
    return;
}

void lcdkit_hs_lp_switch(void* pdata, int mode)
{
    return;
}

void lcdkit_mipi_dsi_max_return_packet_size(void* pdata, struct lcdkit_dsi_cmd_desc* cm)
{
    return;
}

#if 0
int lcdkit_mipi_dsi_read_compare(struct mipi_dsi_read_compare_data* data,
                                 void* pdata)
{
    int ret = 0;
    return ret;
}
#endif

bool lcdkit_is_cmd_panel(void)
{
    return true;
}

void lcdkit_updt_porch(struct platform_device* pdev, int scence)
{
    return;
}

void lcdkit_lp2hs_mipi_test(void* pdata)
{
    return;
}

void lcdkit_effect_switch_ctrl(void* pdata, bool ctrl)
{
    return;
}

int adc_get_value(int channel)
{
    // hisi_adc_get_value(channel);
    return 0;
}

int lcdkit_fake_update_bl(void *pdata, uint32_t bl_level)
{
    int ret = 0;
    return ret;
}

ssize_t lcdkit_jdi_nt35696_5p5_gram_check_show(void* pdata, char* buf)
{
    return 0;
}

ssize_t lcdkit_jdi_nt35696_5p5_reg_read_show(void* pdata, char* buf)
{
    return 0;
}

int buf_trans(const char* inbuf, int inlen, char** outbuf, int* outlen)
{
    int ret = 0;
    return ret;
}

int lcdkit_check_mipi_fifo_empty(char __iomem *dsi_base)
{
    int ret = 0;
    return ret;
}

void lcdkit_fps_scence_adaptor_handle(struct platform_device* pdev, uint32_t scence)
{
    return;
}

void lcdkit_fps_scence_switch_immediately(struct platform_device *pdev, uint32_t scence)
{
    return;
}

#if 0
static void lcdkit_fps_work_handler(struct work_struct *data)
{
    return;
}
#endif

void lcdkit_fps_timer_adaptor_handler(unsigned long data)
{
    return;
}

void lcdkit_fps_timer_adaptor_init(void)
{
    return;
}

void lcdkit_fps_adaptor_ts_callback(void)
{
    return;
}

void lcdkit_fps_updt_adaptor_handle(void* pdata)
{
    return;
}
int lcdkit_lread_reg(void *pdata, uint32_t *out, struct lcdkit_dsi_cmd_desc* cmds, uint32_t len)
{
    int ret = 0;
    return ret;
}

ssize_t host_panel_oem_info_show(void* pdata, char *buf)
{
	return 0;
}

ssize_t host_panel_oem_info_store(void* pdata, char *buf)
{
	return 0;
}

ssize_t lcdkit_set_bl_normal_mode_reg(void* pdata)
{
	return 0;
}

ssize_t lcdkit_set_bl_enhance_mode_reg(void* pdata)
{
	return 0;
}
