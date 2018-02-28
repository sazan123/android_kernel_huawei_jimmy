#include "lcdkit_panel.h"
#include "lcdkit_dbg.h"
//#include "hisi_display_effect.h"

unsigned int g_curr_color_temp_lv = 0;

typedef struct {
	unsigned int color_level;
	unsigned int coef[3][3];
} LCDKIT_COLOR_TEMP_T;

extern void set_color_temp_interface(unsigned int ccorr_coef_ref[3][3], void *handle);

LCDKIT_COLOR_TEMP_T g_color_temp_params[] =
{
    {0,     {   {1024,0,0}, {0,965,0},  {0,0,891}   }},/*coldest*/
    {1,     {   {1024,0,0}, {0,856,0},  {0,0,703}   }},
    {2,     {   {1024,0,0}, {0,750,0},  {0,0,517}   }},
    {3,     {   {1024,0,0}, {0,644,0},  {0,0,331}   }},/*warmest*/
};

static int color_enhancement_mode = 2;
extern void set_lcd_ic_color_enhancement_mode(int mode);

ssize_t get_lcdkit_support(void)
{
    return true;
}

#if 0
struct hisi_fb_data_type* get_fb_data(struct device* dev)
{
    struct hisi_fb_data_type* hisifd = NULL;
    struct fb_info* fbi = NULL;

    if (NULL == dev)
    {
        LCDKIT_ERR("dev is NULL Point!\n");
        return NULL;
    }

    fbi = dev_get_drvdata(dev);

    if (NULL == fbi)
    {
        LCDKIT_ERR("fbi is NULL Point!\n");
        return NULL;
    }

    hisifd = (struct hisi_fb_data_type*)fbi->par;
    return hisifd;
}
#endif

ssize_t lcd_cabc_mode_store(struct device* dev, struct lcdkit_panel_data* lcdkit_info, const char* buf)
{
    ssize_t ret = 0;
    void *pdata = NULL;
    ret = lcdkit_info->lcdkit_cabc_mode_store(pdata,buf);
    return ret;
}

ssize_t lcd_inversion_mode_store(struct device* dev, struct lcdkit_panel_data* lcdkit_info, const char* buf)
{
    ssize_t ret = 0;
    void *pdata = NULL;
    ret = lcdkit_info->lcdkit_inversion_mode_store(pdata,buf);
    return ret;
}

ssize_t lcd_scan_mode_store(struct device* dev, struct lcdkit_panel_data* lcdkit_info, const char* buf)
{
    ssize_t ret = 0;
    void *pdata = NULL;
    ret = lcdkit_info->lcdkit_scan_mode_store(pdata,buf);
    return ret;
}

ssize_t lcd_check_reg_show(struct device* dev, struct lcdkit_panel_data* lcdkit_info, char* buf)
{
    ssize_t ret = 0;
    void *pdata = NULL;
    ret = lcdkit_info->lcdkit_check_reg_show(pdata,buf);
    return ret;
}

ssize_t lcd_gram_check_show(struct device* dev, struct lcdkit_panel_data* lcdkit_info, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_gram_check_store(struct device* dev, struct lcdkit_panel_data* lcdkit_info, const char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_dynamic_sram_check_show(struct device* dev, struct lcdkit_panel_data* lcdkit_info, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_dynamic_sram_check_store(struct device* dev, struct lcdkit_panel_data* lcdkit_info, const char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_ic_color_enhancement_mode_show(struct device* dev, struct lcdkit_panel_data* lcdkit_info, const char* buf)
{
    return sprintf(buf, "%d\n", color_enhancement_mode);
}

ssize_t lcd_ic_color_enhancement_mode_store(struct device* dev, struct lcdkit_panel_data* lcdkit_info, const char* buf)
{
    ssize_t ret = 0;
    int mode=0;

    ret = kstrtoint(buf, 10, &mode);
    if(mode < 0 || mode >2)
        return ret;
    color_enhancement_mode = mode;
    set_lcd_ic_color_enhancement_mode(mode);
    return ret;
}

ssize_t lcd_sleep_ctrl_store(struct device* dev, struct lcdkit_panel_data* lcdkit_info, const char* buf)
{
    ssize_t ret = 0;

    lcdkit_info->lcdkit_sleep_ctrl_store(buf);

    return ret;
}

ssize_t lcd_lp2hs_mipi_check_show(struct device* dev, struct lcdkit_panel_data* lcdkit_info, char* buf)
{
    ssize_t ret = 0;
    unsigned char esd_value;
    lcdkit_dsi_rx(0xD9,&esd_value);
    if( esd_value == 0x80){
         ret = snprintf(buf, PAGE_SIZE, "OK\n");
    }else{
         ret = snprintf(buf, PAGE_SIZE, "FAIL \n");
    }
    return ret;
}

ssize_t lcd_lp2hs_mipi_check_store(struct device* dev, struct lcdkit_panel_data* lcdkit_info, const char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_bist_check(struct device* dev, struct lcdkit_panel_data* lcdkit_info, char* buf)
{
    ssize_t ret = 0;
    return ret;
}


ssize_t lcd_mipi_detect_show(struct device* dev, struct lcdkit_panel_data* lcdkit_info, char* buf)
{
    ssize_t ret = 0;
    unsigned char value;
    lcdkit_dsi_rx(0xD9,&value);
    if( value == 0x80){
         ret = snprintf(buf, PAGE_SIZE, "OK\n");
    }else{
         ret = snprintf(buf, PAGE_SIZE, "FAIL \n");
    }
    return ret;
}

ssize_t lcd_voltage_mode_enable_store(struct device* dev, struct lcdkit_panel_data* lcdkit_info, const char* buf)
{
    ssize_t ret = 0;

    return ret;
}

ssize_t lcd_acl_ctrl_show(struct device* dev, struct lcdkit_panel_data* lcdkit_info, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_acl_ctrl_store(struct device* dev, struct lcdkit_panel_data* lcdkit_info, const char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_amoled_vr_mode_show(struct device* dev, struct lcdkit_panel_data* lcdkit_info, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_amoled_vr_mode_store(struct device* dev, struct lcdkit_panel_data* lcdkit_info, const char* buf)
{
    ssize_t ret = 0;  
    return ret;
}

ssize_t lcd_hbm_ctrl_store(struct device* dev, struct lcdkit_panel_data* lcdkit_info, const char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_support_mode_show(struct device* dev, struct lcdkit_panel_data* lcdkit_info, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_support_mode_store(struct device* dev, struct lcdkit_panel_data* lcdkit_info, const char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_comform_mode_show(struct device* dev,
                              struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_comform_mode_store(struct device* dev,
                               struct device_attribute* attr, const char* buf, size_t count)
{
    ssize_t ret = 0;
    return count;
}

ssize_t lcd_cinema_mode_show(struct device* dev,
                             struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_cinema_mode_store(struct device* dev,
                              struct device_attribute* attr, const char* buf, size_t count)
{
    ssize_t ret = 0;

    return count;
}

ssize_t lcd_support_checkmode_show(struct device* dev, struct lcdkit_panel_data* lcdkit_info, char* buf)
{
    ssize_t ret = 0;
    if (NULL == dev)
    {
        LCDKIT_ERR("dev is NULL Point!\n");
        return -ENXIO;
    }

    if (NULL == lcdkit_info)
    {
        LCDKIT_ERR("lcdkit_info is NULL Point!\n");
        return -EINVAL;
    }

    if (lcdkit_info->lcdkit_support_checkmode_show)
    { 
        ret = lcdkit_info->lcdkit_support_checkmode_show(buf); 
    }
    return ret;
}

ssize_t led_rg_lcd_color_temperature_show(struct device* dev,
        struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t led_rg_lcd_color_temperature_store(struct device* dev,
        struct device_attribute* attr, const char* buf, size_t count)
{
    return count;
}

ssize_t lcd_ce_mode_show(struct device* dev,
                         struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_ce_mode_store(struct device* dev,
                          struct device_attribute* attr, const char* buf, size_t count)
{
    ssize_t ret = 0;
    return count;
}

ssize_t effect_al_show(struct device* dev,
                       struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t effect_al_store(struct device* dev,
                        struct device_attribute* attr, const char* buf, size_t count)
{
    ssize_t ret = 0;
    return count;
}
ssize_t effect_ce_show(struct device* dev,
                       struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t effect_ce_store(struct device* dev,
                        struct device_attribute* attr, const char* buf, size_t count)
{
    ssize_t ret = 0;
    return count;
}

ssize_t lcd_effect_sre_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_effect_sre_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    ssize_t ret = 0;
    return count;
}

ssize_t effect_bl_show(struct device* dev,
                       struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t effect_bl_enable_show(struct device* dev,
                              struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t effect_bl_enable_store(struct device* dev,
                               struct device_attribute* attr, const char* buf, size_t count)
{
    ssize_t ret = 0;
    return count;
}

ssize_t effect_metadata_show(struct device* dev,
                             struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t effect_metadata_store(struct device* dev,
                              struct device_attribute* attr, const char* buf, size_t count)
{
    ssize_t ret = 0;
    return count;
}

ssize_t effect_available_show(struct device* dev,
                              struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t gamma_dynamic_store(struct device* dev,
                            struct device_attribute* attr, const char* buf, size_t count)
{
    ssize_t ret = 0;
    return count;
}

ssize_t  lcd_2d_sharpness_show(struct device* dev,
                               struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;  
    return ret;
}

ssize_t lcd_2d_sharpness_store(struct device* dev,
                               struct device_attribute* attr, const char* buf, size_t count)
{
    return count;
}

ssize_t lcd_acm_state_show(struct device* dev,
                           struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
  
    return ret;
}

ssize_t lcd_acm_state_store(struct device* dev,
                            struct device_attribute* attr, const char* buf, size_t count)
{
    return count;
}

ssize_t lcd_gmp_state_show(struct device* dev,
                           struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_gmp_state_store(struct device* dev,
                            struct device_attribute* attr, const char* buf, size_t count)
{
    return count;
}

ssize_t sbl_ctrl_show(struct device* dev,
                      struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t sbl_ctrl_store(struct device* dev,
                       struct device_attribute* attr, const char* buf, size_t count)
{
    ssize_t ret = 0;
    return count;
}

ssize_t lcd_color_temperature_show(struct device* dev,
                                   struct device_attribute* attr, char* buf)
{
    return sprintf(buf, "%u\n", g_curr_color_temp_lv);
}

ssize_t lcd_color_temperature_store(struct device* dev,
                                    struct device_attribute* attr, const char* buf, size_t count)
{
    int ret = 0;
    int index;
    int i,j,k;
    unsigned int temp_param[3][3];
    unsigned int max_level = 0;

    ret = kstrtoint(buf, 10, &index);
    LCDKIT_DEBUG("[%s]:index = %d\n",__func__,index);

    max_level = sizeof(g_color_temp_params)/sizeof(LCDKIT_COLOR_TEMP_T);
    if(index<0||index>=max_level) {
        LCDKIT_ERR("*****   Input color temprature level not supported!    ***** The available range is [0-%d]\n",(max_level-1));
        return;
    }

    /* save current temprature level */
    g_curr_color_temp_lv = index;

    /* reset color data struct */
    for(i=0;i<3;i++)
        for(j=0;j<3;j++)
            temp_param[i][j] = (unsigned int)0;

    /* get color temp params by index */
    for(k=0;k<max_level;k++) {
        if(index == g_color_temp_params[k].color_level)
        {
            for(i=0;i<3;i++) 
                for(j=0;j<3;j++) 
                    temp_param[i][j] = g_color_temp_params[k].coef[i][j];
        }
    }

    /*debug info*/
    LCDKIT_DEBUG("[%s]====== CCORR Coefficient ======\n",__func__);
    LCDKIT_DEBUG("%4d %4d %4d\n", temp_param[0][0], temp_param[0][1], temp_param[0][2]);
    LCDKIT_DEBUG("%4d %4d %4d\n", temp_param[1][0], temp_param[1][1], temp_param[1][2]);
    LCDKIT_DEBUG("%4d %4d %4d\n", temp_param[2][0], temp_param[2][1], temp_param[2][2]);

    set_color_temp_interface(temp_param, NULL);

    return ret;
}

ssize_t lcd_frame_count_show(struct device* dev,
                             struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_frame_update_show(struct device* dev,
                              struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_frame_update_store(struct device* dev,
                               struct device_attribute* attr, const char* buf, size_t count)
{
    return count;
}

ssize_t mipi_dsi_bit_clk_upt_show(struct device* dev,
                                  struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}


ssize_t mipi_dsi_bit_clk_upt_store(struct device* dev,
                                   struct device_attribute* attr, const char* buf, size_t count)
{
    return count;
}

ssize_t lcd_fps_scence_show(struct device* dev,
                            struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_fps_scence_store(struct device* dev, struct device_attribute* attr,
                             const char* buf, size_t count)
{
    return count;
}

ssize_t alpm_function_show(struct device* dev,
                           struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t alpm_function_store(struct device* dev,
                            struct device_attribute* attr, const char* buf, size_t count)
{
    return count;
}

ssize_t alpm_setting_store(struct device* dev,
                           struct device_attribute* attr, const char* buf, size_t count)
{
    return count;
}

ssize_t lcd_func_switch_show(struct device* dev,
                             struct device_attribute* attr, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_func_switch_store(struct device* dev,
                              struct device_attribute* attr, const char* buf, size_t count)
{
    return count;
}

ssize_t lcd_test_config_show(struct device* dev, struct lcdkit_panel_data* lcdkit_info, char* buf)
{
    ssize_t ret = 0; 

    if (NULL == dev)
    {
        LCDKIT_ERR("dev is NULL Point!\n");
        return -ENXIO;
    }

    if (NULL == lcdkit_info)
    {
        LCDKIT_ERR("lcdkit_info is NULL Point!\n");
        return -EINVAL;
    }

     if (lcdkit_info->lcdkit_test_config_show)
    { 
        ret = lcdkit_info->lcdkit_test_config_show(NULL, buf); 
    }
    return ret;
}

ssize_t lcd_test_config_store(struct device* dev, struct lcdkit_panel_data* lcdkit_info, const char* buf)
{
    ssize_t ret = 0; 

    if (NULL == dev)
    {
        LCDKIT_ERR("dev is NULL Point!\n");
        return -ENXIO;
    }

    if (NULL == lcdkit_info)
    {
        LCDKIT_ERR("lcdkit_info is NULL Point!\n");
        return -EINVAL;
    } 

    if (lcdkit_info->lcdkit_test_config_store)
    { 
        ret = lcdkit_info->lcdkit_test_config_store(buf); 
    } 
    return ret;
}

ssize_t lv_detect_show(struct device* dev, struct lcdkit_panel_data* lcdkit_info, char* buf)
{
    ssize_t ret = 0;
    if (NULL == dev)
    {
        LCDKIT_ERR("dev is NULL Point!\n");
        return -ENXIO;
    }

    if (NULL == lcdkit_info)
    {
        LCDKIT_ERR("lcdkit_info is NULL Point!\n");
        return -EINVAL;
    }

     if (lcdkit_info->lcdkit_lv_detect)
    { 
        ret = lcdkit_info->lcdkit_lv_detect(buf); 
    }
    return sprintf(buf, "%u\n", ret);
}

ssize_t current_detect_show(struct device* dev, struct lcdkit_panel_data* lcdkit_info, char* buf)
{
    ssize_t ret = 0;
    if (NULL == dev)
    {
        LCDKIT_ERR("dev is NULL Point!\n");
        return -ENXIO;
    }

    if (NULL == lcdkit_info)
    {
        LCDKIT_ERR("lcdkit_info is NULL Point!\n");
        return -EINVAL;
    }

    if (lcdkit_info->lcdkit_current_detect)
    {
        ret = lcdkit_info->lcdkit_current_detect(buf);
    }
    return sprintf(buf, "%u\n", ret);
}

ssize_t lcd_reg_read_show(struct device* dev, struct lcdkit_panel_data* lcdkit_info, char* buf)
{
    ssize_t ret = 0;
    return ret;
}
ssize_t lcd_reg_read_store(struct device* dev, struct lcdkit_panel_data* lcdkit_info, const char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_ddic_oem_info_show(struct device* dev, struct lcdkit_panel_data* lcdkit_inf, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_ddic_oem_info_store(struct device* dev, struct lcdkit_panel_data* lcdkit_inf, const char* buf)
{
    ssize_t ret = 0;
    return ret;
}
ssize_t lcd_bl_mode_show(struct device* dev, struct lcdkit_panel_data* lcdkit_inf, char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_bl_mode_store(struct device* dev, struct lcdkit_panel_data* lcdkit_inf, const char* buf)
{
    ssize_t ret = 0;
    return ret;
}

ssize_t lcd_support_bl_mode_show(struct device* dev, struct lcdkit_panel_data* lcdkit_inf, char* buf)
{
    ssize_t ret = 0;
    return ret;
}
