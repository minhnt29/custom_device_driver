KERNEL_SRC := $(HOME)/kernel_development/kernel-bbb

# Cross compiler
CROSS_COMPILE := arm-linux-gnueabihf-

# Architecture
ARCH := arm

# Module name
obj-m += ssd1306.o

# Target
all:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_SRC) M=$(PWD) modules

clean:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_SRC) M=$(PWD) clean

.PHONY: all clean