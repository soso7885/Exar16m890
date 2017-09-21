XR_SRC			= 8250_exar_16m890.c
OF_SRC			= of_exar_16m890.c
CROSS_COMPILE	= arm-linux-gnueabihf-
MODULESOPS		= ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE)
KERNELDIR		?= ../../
CC				= $(CROSS_COMPILE)gcc
PWD				:= $(shell pwd)

obj-m = $(XR_SRC).o $(OF_SRC).o

all: modules

modules:
	$(MAKE) $(MODULESOPS) -C $(KDIR) M=$(PWD) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko .tmp_versions modules.order *.mod.c Module.symvers
