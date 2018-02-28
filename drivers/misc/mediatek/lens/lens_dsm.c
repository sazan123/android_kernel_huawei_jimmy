#ifdef CONFIG_HUAWEI_DSM
#include "lens_dsm.h"

static char camera_lens_dsm_log_buff[MTK_CAMERA_DSM_BUFFER_SIZE] = {0};

void camera_report_lens_dsm_err(char *lensName, unsigned long curPos, unsigned long tarPos, int type, int err_num)
{
	ssize_t len = 0;

	memset(camera_lens_dsm_log_buff, 0, MTK_CAMERA_DSM_BUFFER_SIZE);

	/* camera record error info according to err type */
	switch(type)
	{
		case DSM_CAMERA_ACTUATOR_MOVE_FAIL:
			/* report move lens fail */
			len += snprintf(camera_lens_dsm_log_buff+len, MTK_CAMERA_DSM_BUFFER_SIZE-len, "[mtk_camera]move lens fail.\n");
			if ((len < 0) || (len >= MTK_CAMERA_DSM_BUFFER_SIZE -1))
			{
				pr_err("%s %d. write camera_lens_dsm_log_buff error\n",__func__, __LINE__);
				return ;
			}
			len += snprintf(camera_lens_dsm_log_buff+len, MTK_CAMERA_DSM_BUFFER_SIZE-len,
			        "lens name:%s target_pos:%ld  current_pos:%ld\n",
			        lensName, tarPos, curPos);
			if ((len < 0) || (len >= MTK_CAMERA_DSM_BUFFER_SIZE -1))
			{
				pr_err("%s %d. write camera_lens_dsm_log_buff error\n",__func__, __LINE__);
				return ;
			}
			break;

		default:
			break;
	}

	if ( len >= MTK_CAMERA_DSM_BUFFER_SIZE -1 )
	{
		pr_err("write camera_lens_dsm_log_buff overflow.\n");
		return;
	}
	camera_report_dsm_err(type, err_num, camera_lens_dsm_log_buff);
    return;
}
#endif
