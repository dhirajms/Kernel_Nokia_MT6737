ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/alsps/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include

ifeq ($(TARGET_BUILD_VARIANT),user)
ccflags-y += -D_USER_DEBUG_
endif
obj-y	:=  ltr559.o
