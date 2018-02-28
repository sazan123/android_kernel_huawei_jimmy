#ifndef __LCDKIT_DSM_H_
#define __LCDKIT_DSM_H_


#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
extern struct dsm_client *lcd_dclient;
int lcdkit_report_dsm_err(int type, char* reg_name, int read_value,int expect_value);

#endif

#endif