ARCH ?= arm
LINUX_SRC ?= ${HOME}/git/linux-xlnx 
CROSS_COMPILE ?= aarch64-linux-gnu-

obj-m += src/ti-dacx0508.o
obj-m += src/ti-tmp275.o

#BR_OUTPUT_DIR = $(BR_DIR)/output
#LINUX_VER = $(shell grep "^BR2_LINUX_KERNEL_VERSION" $(BR_DIR)/.config 2>/dev/null | cut -d \" -f 2)
#LINUX_SRC = $(BR_OUTPUT_DIR)/build/linux-$(LINUX_VER)

PWD := $(shell pwd)
MODULES := $(wildcard obj-m *.ko)

all:
	$(MAKE) -C $(LINUX_SRC) \
		KBUILD_EXTMOD=$(PWD) modules ARCH=$(ARCH) \
			CROSS_COMPILE=$(CROSS_COMPILE)-

clean:
	$(RM) *.order *.symvers
	$(RM) src/*.o src/*.ko src/*.mod src/*.mod.c
