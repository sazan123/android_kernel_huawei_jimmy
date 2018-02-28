/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <sound/soc.h>
#include <linux/regulator/consumer.h>
#include <sound/hw_audio_info.h>
#ifdef CONFIG_HUAWEI_DSM
static struct dsm_dev audio_dsm_info = {
	.name = DSM_AUDIO_MOD_NAME,
	.fops = NULL,
	.buff_size = DSM_AUDIO_BUF_SIZE,
};
static struct dsm_client *audio_dclient = NULL;

int audio_dsm_register(void)
{
	if (NULL != audio_dclient)
		return 0;

	audio_dclient = dsm_register_client(&audio_dsm_info);
	if (NULL == audio_dclient)
		pr_err("audio_dclient register failed!\n");

	return 0;
}

int audio_dsm_report_num(int error_no, unsigned int mesg_no)
{
	int err = 0;

	if (NULL == audio_dclient) {
		pr_err("%s: audio_dclient did not register!\n", __func__);
		return 0;
	}

	err = dsm_client_ocuppy(audio_dclient);
	if (0 != err) {
		pr_err("%s: user buffer is busy!\n", __func__);
		return 0;
	}

	pr_info("%s: after dsm_client_ocuppy, error_no=0x%x, mesg_no=0x%x!\n",
		__func__, error_no, mesg_no);
	dsm_client_record(audio_dclient, "Message code = 0x%x.\n", mesg_no);
	dsm_client_notify(audio_dclient, error_no);

	return 0;
}

int audio_dsm_report_info(int error_no, char *fmt, ...)
{
	int err = 0;
	int ret = 0;
	char dsm_report_buffer[DSM_REPORT_BUF_SIZE] = {0};
	va_list args;

	if (NULL == audio_dclient) {
		pr_err("%s: audio_dclient did not register!\n", __func__);
		return 0;
	}

	if (error_no < DSM_AUDIO_ERROR_NUM) {
		pr_err("%s: input error_no err!\n", __func__);
		return 0;
	}

	va_start(args, fmt);
	ret = vsnprintf(dsm_report_buffer, DSM_REPORT_BUF_SIZE, fmt, args);
	va_end(args);

	err = dsm_client_ocuppy(audio_dclient);
	if (0 != err) {
		pr_err("%s: user buffer is busy!\n", __func__);
		return 0;
	}

	pr_info("%s: after dsm_client_ocuppy, dsm_error_no = %d, %s\n",
			__func__, error_no, dsm_report_buffer);
	dsm_client_record(audio_dclient, "%s\n", dsm_report_buffer);
	dsm_client_notify(audio_dclient, error_no);

	return 0;
}
#else
int audio_dsm_register(void)
{
	return 0;
}
int audio_dsm_report_num(int error_no, unsigned int mesg_no)
{
	return 0;
}

int audio_dsm_report_info(int error_no, char *fmt, ...)
{
	return 0;
}
#endif