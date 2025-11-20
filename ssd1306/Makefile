KERNEL_SRC := $(HOME)/kernel_development/kernel-bbb

# Cross compiler
CROSS_COMPILE := arm-linux-gnueabihf-

# Architecture
ARCH := arm

# Module name
obj-m += ssd1306.o

# Build directory
BUILD_DIR := $(PWD)/build

# Target
all:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_SRC) M=$(PWD) modules
	@mkdir -p $(BUILD_DIR)
	@mv -f *.o *.ko *.mod* .*.cmd *.order *.symvers .tmp_versions Module.markers 2>/dev/null || true
	@mv -f *.o *.ko *.mod* .*.cmd *.order *.symvers .tmp_versions 2>/dev/null $(BUILD_DIR)/ || true
	@echo "Build artifacts moved to $(BUILD_DIR)"

clean:
	@rm -rf $(BUILD_DIR)
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_SRC) M=$(PWD) clean

.PHONY: all clean