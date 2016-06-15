obj-y +=  quark_se.o
obj-y +=  usb_setup.o
obj-y +=  soc_flash.o
obj-$(CONFIG_PANIC_DUMP) += panic_boot.o
obj-y +=  boot_x86.o
obj-y +=  cos.o

# CFLAGS_dfu_desc.o += -Wno-error=format -Wno-error -w
obj-$(CONFIG_USB_DFU) += dfu_desc.o

# TODO: REMOVEME
cflags-y += -Wno-error=format
cflags-y += -Wno-error -w

