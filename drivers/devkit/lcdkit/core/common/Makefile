obj-$(CONFIG_LCDKIT_DRIVER)	+= lcdkit_panel.o
obj-$(CONFIG_LCDKIT_DRIVER)	+= lcdkit_fb_util.o
obj-$(CONFIG_LCDKIT_DRIVER)	+= lcdkit_debug.o
obj-$(CONFIG_LCDKIT_DRIVER)	+= lcdkit_dsm.o


mkfile_path := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
ifeq ($(mkfile_path)/hisi/Makefile, $(wildcard $(mkfile_path)/hisi/Makefile))
obj-$(CONFIG_LCDKIT_DRIVER) += hisi/
$(info find lcdkit hisi folder, include it now!)
endif

ifeq ($(mkfile_path)/qcom/Makefile, $(wildcard $(mkfile_path)/qcom/Makefile))
obj-$(CONFIG_LCDKIT_DRIVER) += qcom/
$(info find lcdkit qcom folder, include it now! )
endif

ifeq ($(mkfile_path)/mtk/Makefile, $(wildcard $(mkfile_path)/mtk/Makefile))
obj-$(CONFIG_LCDKIT_DRIVER) += mtk/
$(info find lcdkit mtk folder, include it now! )
endif

#for debug!!
#$(error stop! $(mkfile_path))
EXTRA_CFLAGS += -Idrivers/devkit/lcdkit/include                 \
                -Idrivers/devkit/tpkit                          \
                -Idrivers/misc/mediatek/video/mt6755            \
                -Idrivers/misc/mediatek/lcm/inc                 \
                -Idrivers/misc/mediatek/video/include           \
                -Idrivers/misc/mediatek/cmdq/v2/                \
                -Idrivers/misc/mediatek/cmdq/v2/$(MTK_PLATFORM)
