#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

#
# Makefile for misc devices that really don't fit anywhere else.
#

obj-y += ontim_hd720_ili9881c_tcl_5209.o

ccflags-$(CONFIG_MTK_LCM) += -I$(srctree)/drivers/misc/mediatek/lcm/inc
ifeq ($(CONFIG_LCDKIT_DRIVER),y)
EXTRA_CFLAGS += -Idrivers/devkit/lcdkit/include
EXTRA_CFLAGS += -Idrivers/devkit/lcdkit/core/common/mtk
endif
