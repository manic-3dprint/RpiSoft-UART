obj-m := softuart.o

LINUX = /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(LINUX) M=$(PWD) modules

clean:
	$(MAKE) -C $(LINUX) M=$(PWD) clean

install:
	$(MAKE) -C $(LINUX) M=$(PWD) modules_install
