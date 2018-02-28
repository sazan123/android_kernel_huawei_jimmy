obj-$(CONFIG_LCDKIT_DRIVER)	+= lcdkit_fb_util_mtk.o
obj-$(CONFIG_LCDKIT_DRIVER)	+= lcdkit_debug_mtk.o
obj-$(CONFIG_LCDKIT_DRIVER)	+= lcdkit_panel_adaptor_mtk.o

EXTRA_CFLAGS += -Idrivers/devkit/lcdkit/include                 \
                -Idrivers/devkit/lcdkit/                        \
                -Idrivers/misc/mediatek/video/$(MTK_PLATFORM)/  \
                -Idrivers/misc/mediatek/lcm/inc/                \
                -Idrivers/misc/mediatek/video/include           \
                -Idrivers/misc/mediatek/cmdq/v2/                \
                -Idrivers/misc/mediatek/cmdq/v2/$(MTK_PLATFORM) \
                -Idrivers/devkit/tpkit
