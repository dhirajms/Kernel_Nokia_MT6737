# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/GT5663/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/GT5663/include/$(CONFIG_GTP_FIRMWARE_GOODIX)/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/GT5663/include/$(CONFIG_GTP_CONFIG_GOODIX)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

#obj-y	+=  gt1x_extents.o
#obj-y	+=  gt1x_generic.o
#obj-y	+=  gt1x_tools.o
#obj-y	+=  gt1x_tpd.o
#obj-y	+=  gt1x_update.o
gt5663-objs := gt1x_extents.o gt1x_generic.o gt1x_tools.o gt1x_tpd.o gt1x_update.o
obj-$(CONFIG_TOUCHSCREEN_MTK_GT5663) += gt5663.o
