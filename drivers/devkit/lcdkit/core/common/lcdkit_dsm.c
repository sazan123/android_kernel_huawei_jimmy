#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>

#include<linux/init.h>
#include<linux/module.h>

#include <linux/of.h>

#ifdef CONFIG_HUAWEI_DSM
#include "lcdkit_dsm.h"
#include <dsm/dsm_pub.h>
#include <linux/sched.h>
#include "lcdkit_dbg.h"

static int lcd_mdp_error_debug = 0;
struct dsm_client* lcd_dclient = NULL;

int lcdkit_report_dsm_err(int type, char* reg_name, int read_value, int expect_value)
{
    char* reg_string = (reg_name == NULL) ? " " : reg_name;;
    /* we will ignore lcd error 20100 for 0x51 */
    if ((DSM_LCD_MIPI_ERROR_NO == type && 0x51 == expect_value)
        || DSM_LCD_MDSS_DSI_ISR_ERROR_NO == type)
    {
        return 0;
    }

    LCDKIT_INFO(": entry! type:%d\n", type);


    if ( NULL == lcd_dclient )
    {
        LCDKIT_ERR(": there is not lcd_dclient!\n");
        return -1;
    }

    /* try to get permission to use the buffer */
    if (dsm_client_ocuppy(lcd_dclient))
    {
        /* buffer is busy */
        LCDKIT_ERR(": buffer is busy!\n");
        return -1;
    }

    /* lcd report err according to err type */
    switch (type)
    {
        case DSM_LCD_MDSS_UNDERRUN_ERROR_NO:
            dsm_client_record(lcd_dclient, "ldi underflow!\n");
            break;

        case DSM_LCD_TE_TIME_OUT_ERROR_NO:
            dsm_client_record(lcd_dclient, "TE time out!\n");
            break;

        case DSM_LCD_STATUS_ERROR_NO:
            dsm_client_record(lcd_dclient,
                "lcd register %s status wrong, read value :%x, "
                "but expect value: %x\n", reg_string, read_value, expect_value);
            break;

        case DSM_LCD_POWER_STATUS_ERROR_NO:
            dsm_client_record(lcd_dclient, "lcd power status error!\n");
            break;

        /* add for lcd esd */
        case DSM_LCD_ESD_STATUS_ERROR_NO:
            dsm_client_record(lcd_dclient, "lcd esd register status error!\n");
            break;

        case DSM_LCD_ESD_RECOVERY_NO:
            dsm_client_record(lcd_dclient, "lcd esd recover happend!\n");
            break;

        case DSM_LCD_ESD_RESET_ERROR_NO:
            dsm_client_record(lcd_dclient,
                "LCD RESET register %x read data =%x\n", expect_value, read_value );
            break;

        case DSM_LCD_ESD_OCP_RECOVERY_NO:
            dsm_client_record(lcd_dclient,
                "esd ocp happend, register %x status wrong, read value :%x, "
                "but expect value: %x\n", reg_string, read_value, expect_value);
            break;

        case DSM_LCD_MIPI_ERROR_NO:
            dsm_client_record(lcd_dclient,
                "mipi transmit register %x time out ,err number :%x\n",
                expect_value, read_value );
            break;

        case DSM_LCD_MDSS_IOMMU_ERROR_NO:
            dsm_client_record(lcd_dclient,
                "mdss iommu attach/detach or map memory fail (%d)\n", read_value);

            if (!lcd_mdp_error_debug)
            {
                lcd_mdp_error_debug = 1;
                dsm_client_record(lcd_dclient,
                                  "cmd:@echo 1 >  /sys/kernel/debug/mdp/xlog/enable");
                dsm_client_record(lcd_dclient,
                                  "@echo 1 >  /sys/kernel/debug/mdp/xlog/reg_dump");
                dsm_client_record(lcd_dclient,
                                  "echo 1 >  /sys/kernel/debug/mdp/xlog/panic");
            }
            else
            {
                dsm_client_record(lcd_dclient,
                                  "cmd:@rm -rf /data/hwzd_logs/dmd_log/dmd_spcial_log/*");
                dsm_client_record(lcd_dclient,
                                  "cp /sys/kernel/debug/mdp/xlog/dump /data/hwzd_logs/dmd_spcial_log/debug_log/xlog_and_mdss_register.txt");
            }

            break;

        case DSM_LCD_MDSS_PIPE_ERROR_NO:
            dsm_client_record(lcd_dclient,
                "mdss pipe status error (%d)\n",read_value);

            if (!lcd_mdp_error_debug)
            {
                lcd_mdp_error_debug = 1;
                dsm_client_record(lcd_dclient,
                                  "cmd:@echo 1 >  /sys/kernel/debug/mdp/xlog/enable");
                dsm_client_record(lcd_dclient,
                                  "@echo 1 >  /sys/kernel/debug/mdp/xlog/reg_dump");
                dsm_client_record(lcd_dclient,
                                  "echo 1 >  /sys/kernel/debug/mdp/xlog/panic");
            }
            else
            {
                dsm_client_record(lcd_dclient,
                                  "cmd:@rm -rf /data/hwzd_logs/dmd_log/dmd_spcial_log/*");
                dsm_client_record(lcd_dclient,
                                  "cp /sys/kernel/debug/mdp/xlog/dump /data/hwzd_logs/dmd_spcial_log/debug_log/xlog_and_mdss_register.txt");
            }

            break;

        case DSM_LCD_MDSS_PINGPONG_ERROR_NO:
            dsm_client_record(lcd_dclient,
                "mdss wait pingpong time out (%d)\n",read_value);

            if (!lcd_mdp_error_debug)
            {
                lcd_mdp_error_debug = 1;
                dsm_client_record(lcd_dclient,
                                  "cmd:@echo 1 >  /sys/kernel/debug/mdp/xlog/enable");
                dsm_client_record(lcd_dclient,
                                  "@echo 1 >  /sys/kernel/debug/mdp/xlog/reg_dump");
                dsm_client_record(lcd_dclient,
                                  "echo 1 >  /sys/kernel/debug/mdp/xlog/panic");
            }
            else
            {
                dsm_client_record(lcd_dclient,
                                  "cmd:@rm -rf /data/hwzd_logs/dmd_log/dmd_spcial_log/*");
                dsm_client_record(lcd_dclient,
                                  "cp /sys/kernel/debug/mdp/xlog/dump /data/hwzd_logs/dmd_spcial_log/debug_log/xlog_and_mdss_register.txt");
            }

            break;

        case DSM_LCD_MDSS_VSP_VSN_ERROR_NO:
            dsm_client_record(lcd_dclient,
                "get vsp/vsn(%d) register fail (%d) \n", expect_value, read_value);
            break;

        case DSM_LCD_MDSS_ROTATOR_ERROR_NO:
            dsm_client_record(lcd_dclient,
                              "mdss rotator queue fail (%d) \n", read_value);

            if (!lcd_mdp_error_debug)
            {
                lcd_mdp_error_debug = 1;
                dsm_client_record(lcd_dclient,
                                  "cmd:@echo 1 >  /sys/kernel/debug/mdp/xlog/enable");
                dsm_client_record(lcd_dclient,
                                  "@echo 1 >  /sys/kernel/debug/mdp/xlog/reg_dump");
                dsm_client_record(lcd_dclient,
                                  "echo 1 >  /sys/kernel/debug/mdp/xlog/panic");
            }
            else
            {
                dsm_client_record(lcd_dclient,
                                  "cmd:@rm -rf /data/hwzd_logs/dmd_log/dmd_spcial_log/*");
                dsm_client_record(lcd_dclient,
                                  "cp /sys/kernel/debug/mdp/xlog/dump /data/hwzd_logs/dmd_log/dmd_spcial_log/xlog_and_mdss_register.txt");
            }

            break;

        case DSM_LCD_MDSS_FENCE_ERROR_NO:
            dsm_client_record(lcd_dclient,
                              "mdss sync_fence_wait fail (%d) \n", read_value);

            if (!lcd_mdp_error_debug)
            {
                lcd_mdp_error_debug = 1;
                dsm_client_record(lcd_dclient,
                                  "cmd:@echo 1 >  /sys/kernel/debug/mdp/xlog/enable");
                dsm_client_record(lcd_dclient,
                                  "@echo 1 >  /sys/kernel/debug/mdp/xlog/reg_dump");
                dsm_client_record(lcd_dclient,
                                  "echo 1 >  /sys/kernel/debug/mdp/xlog/panic");
            }
            else
            {
                dsm_client_record(lcd_dclient,
                                  "cmd:@rm -rf /data/hwzd_logs/dmd_log/dmd_spcial_log/*");
                dsm_client_record(lcd_dclient,
                                  "cp /sys/kernel/debug/mdp/xlog/dump /data/hwzd_logs/dmd_log/dmd_spcial_log/xlog_and_mdss_register.txt");
            }

            break;

        case DSM_LCD_MDSS_CMD_STOP_ERROR_NO:
            dsm_client_record(lcd_dclient,
                              "mdss stop cmd time out (%d) \n", read_value);

            if (!lcd_mdp_error_debug)
            {
                lcd_mdp_error_debug = 1;
                dsm_client_record(lcd_dclient,
                                  "cmd:@echo 1 >  /sys/kernel/debug/mdp/xlog/enable");
                dsm_client_record(lcd_dclient,
                                  "@echo 1 >  /sys/kernel/debug/mdp/xlog/reg_dump");
                dsm_client_record(lcd_dclient,
                                  "echo 1 >  /sys/kernel/debug/mdp/xlog/panic");
            }
            else
            {
                dsm_client_record(lcd_dclient,
                                  "cmd:@rm -rf /data/hwzd_logs/dmd_log/dmd_spcial_log/*");
                dsm_client_record(lcd_dclient,
                                  "cp /sys/kernel/debug/mdp/xlog/dump /data/hwzd_logs/dmd_log/dmd_spcial_log/xlog_and_mdss_register.txt");
            }

            break;

        case DSM_LCD_MDSS_VIDEO_DISPLAY_ERROR_NO:
            dsm_client_record(lcd_dclient,
                              "mdss commit without wait! ctl=%d", read_value);

            if (!lcd_mdp_error_debug)
            {
                lcd_mdp_error_debug = 1;
                dsm_client_record(lcd_dclient,
                                  "cmd:@echo 1 >  /sys/kernel/debug/mdp/xlog/enable");
                dsm_client_record(lcd_dclient,
                                  "@echo 1 >  /sys/kernel/debug/mdp/xlog/reg_dump");
                dsm_client_record(lcd_dclient,
                                  "echo 1 >  /sys/kernel/debug/mdp/xlog/panic");
            }
            else
            {
                dsm_client_record(lcd_dclient,
                                  "cmd:@rm -rf /data/hwzd_logs/dmd_log/dmd_spcial_log/*");
                dsm_client_record(lcd_dclient,
                                  "cp /sys/kernel/debug/mdp/xlog/dump /data/hwzd_logs/dmd_log/dmd_spcial_log/xlog_and_mdss_register.txt");
            }

            break;

        case DSM_LCD_MDSS_MDP_CLK_ERROR_NO:
            dsm_client_record(lcd_dclient,
                              "mdss mdp clk can't be turned off\n", read_value);

            if (!lcd_mdp_error_debug)
            {
                lcd_mdp_error_debug = 1;
                dsm_client_record(lcd_dclient,
                                  "cmd:@echo 1 >  /sys/kernel/debug/mdp/xlog/enable");
                dsm_client_record(lcd_dclient,
                                  "@echo 1 >  /sys/kernel/debug/mdp/xlog/reg_dump");
                dsm_client_record(lcd_dclient,
                                  "echo 1 >  /sys/kernel/debug/mdp/xlog/panic");
            }
            else
            {
                dsm_client_record(lcd_dclient,
                                  "cmd:@rm -rf /data/hwzd_logs/dmd_log/dmd_spcial_log/*");
                dsm_client_record(lcd_dclient,
                                  "cp /sys/kernel/debug/mdp/xlog/dump /data/hwzd_logs/dmd_log/dmd_spcial_log/xlog_and_mdss_register.txt");
            }

            break;

        case DSM_LCD_MDSS_MDP_BUSY_ERROR_NO:
            dsm_client_record(lcd_dclient,
                              "mdss mdp dma tx time out (%d)\n", read_value);

            if (!lcd_mdp_error_debug)
            {
                lcd_mdp_error_debug = 1;
                dsm_client_record(lcd_dclient,
                                  "cmd:@echo 1 >  /sys/kernel/debug/mdp/xlog/enable");
                dsm_client_record(lcd_dclient,
                                  "@echo 1 >  /sys/kernel/debug/mdp/xlog/reg_dump");
                dsm_client_record(lcd_dclient,
                                  "echo 1 >  /sys/kernel/debug/mdp/xlog/panic");
            }
            else
            {
                dsm_client_record(lcd_dclient,
                                  "cmd:@rm -rf /data/hwzd_logs/dmd_log/dmd_spcial_log/*");
                dsm_client_record(lcd_dclient,
                                  "cp /sys/kernel/debug/mdp/xlog/dump /data/hwzd_logs/dmd_log/dmd_spcial_log/xlog_and_mdss_register.txt");
            }

            break;

        default:
            break;
    }

    dsm_client_notify(lcd_dclient, type);

    return 0;
}
#endif

