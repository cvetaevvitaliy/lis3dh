#
# Makefile for the input misc STM acc lis3dh driver
#
obj-m := lis3dh.o
lis3dh-objs := lis3dh_core.o lis3dh_i2c.o
KERNELDIR := /home/linux/openwrt/gateway/linux_3.14/
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- 	
