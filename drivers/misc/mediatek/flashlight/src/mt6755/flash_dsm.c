#ifdef CONFIG_HUAWEI_DSM
#include "flash_dsm.h"

//number of flash errors to report
#define MAX_ERR_REPORT_CNT                  4

//specific errors to report
static const struct flash_dsm_report flash_reports[MAX_ERR_REPORT_CNT] =
{
    {LED_TIMEOUT,
        "[mtk_camera]Flash led timeout.\n",
        DSM_CAMERA_LED_FLASH_CIRCUIT_ERR},
    {LED_THERMAL_SHUTDOWN,
        "[mtk_camera]Flash temperature reached thermal shutdown value.\n",
        DSM_CAMERA_LED_FLASH_CIRCUIT_ERR},
    {LED_VOUT_ERR,
        "[mtk_camera]Flash gpio set error.\n",
        DSM_CAMERA_LED_FLASH_CIRCUIT_ERR},
    {LED_POWER_SAVING,
        "[mtk_camera]power saving shutdown the flash.\n",
        DSM_CAMERA_LED_FLASH_CIRCUIT_ERR},
};

/*==========================================================
 * FUNCTION   : camera_report_flash_dsm_err
 *
 * DESCRIPTION: report error msg to dsm
 *
 * PARAMETERS :
 *   @flash_flag  : error flags
 *
 * RETURN     : void
 *=========================================================*/

void camera_report_flash_dsm_err(int flash_flag)
{
	ssize_t len = 0;
    int i = 0;

	if (0 == flash_flag){
		return;
	}

	memset(camera_dsm_log_buff, 0, MTK_CAMERA_DSM_BUFFER_SIZE);

	/* camera record error info according to err type */
    for (i=0; i<MAX_ERR_REPORT_CNT; i++)
    {
		len = 0;
        if (flash_reports[i].report_mask & flash_flag)
        {
			len += snprintf(camera_dsm_log_buff+len, MTK_CAMERA_DSM_BUFFER_SIZE-len, flash_reports[i].report_msg);
			if ((len < 0) || (len >= MTK_CAMERA_DSM_BUFFER_SIZE -1))
			{
				pr_err("%s %d. write camera_dsm_log_buff error\n",__func__, __LINE__);
				return;
			}
			camera_report_dsm_err(flash_reports[i].dsm_error_no, flash_flag, camera_dsm_log_buff);
			break;
		}
	}
	return;
}
#endif
