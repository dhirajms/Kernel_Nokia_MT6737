#
# Makefile for misc devices that really don't fit anywhere else.
#
ifeq ($(TARGET_BUILD_VARIANT),user)
#$(error yujinyun $(TARGET_BUILD_VARIANT))
ccflags-y += -D_LOG_USER_
endif
obj-y				+= leds_strobe.o
