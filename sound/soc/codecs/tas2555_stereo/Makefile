tas2555-core-objs := tas2555-core.o tiload.o
tas2555-objs-$(CONFIG_TAS2555_CODEC_STEREO) += tas2555-codec.o
tas2555-objs-$(CONFIG_TAS2555_I2C_STEREO) += tas2555-i2c.o
tas2555-objs-$(CONFIG_TAS2555_MISC_STEREO) += tas2555-misc.o

snd-soc-tas2555-objs := $(tas2555-core-objs) $(tas2555-objs-y) $(tas2555-objs-m)
obj-$(CONFIG_SND_SOC_TAS2555_STEREO) += snd-soc-tas2555.o
