/**
* @file    mt_hotplug_strategy_external.h
* @brief   hotplug strategy(hps) - external header file
*/

#ifndef __MT_HOTPLUG_STRATEGY_EXTERNAL_H__
#define __MT_HOTPLUG_STRATEGY_EXTERNAL_H__

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/sched/rt.h>

void __attribute__((weak)) hps_dump_task_info(void)
{

}

extern void hps_dump_task_info(void);

#endif
