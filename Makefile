obj-m := soft_uart.o
soft_uart-objs := softuart.o	
ccflags-y := -O2 -Wno-unused-function -Wno-unused-variable	
PWD := $(shell pwd)

KERNEL_SRC_DIR := /root/raspberry/linux
GCC_PREFIX := arm-bcm2708-linux-gnueabi-

all:
	$(MAKE) -C $(KERNEL_SRC_DIR) M=$(PWD) modules ARCH=arm CROSS_COMPILE=$(GCC_PREFIX)

load:
	insmod softuart.ko

unload:
	rmmod softuart
