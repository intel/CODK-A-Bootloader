#TODO: REMOVEME
cflags-$(CONFIG_USB) 	+= -Wno-error=format
cflags-$(CONFIG_USB) 	+= -Wno-error -w
#cflags-$(CONFIG_USB) 	+= -DUSB_DEBUG
#cflags-$(CONFIG_USB) 	+= -DDEBUG_EP0

# Use one of the following flags to compile the software in host-only or
# device-only mode.
cflags-$(CONFIG_USB)	+= -DDWC_DEVICE_ONLY

cflags-$(CONFIG_USB)	+= -I$(T)/arduino101_firmware/bsp/bootable/bootloader/drivers/usb/dwc_3.10b/dwc_common_port/
cflags-$(CONFIG_USB)	+= -I$(T)/arduino101_firmware/bsp/bootable/bootloader/include/usb/

obj-$(CONFIG_USB)	+= dwc_otg_cil.o dwc_otg_cil_intr.o
obj-$(CONFIG_USB)	+= dwc_otg_pcd.o dwc_otg_pcd_intr.o
obj-$(CONFIG_USB)	+= dwc_usb_device_driver.o
obj-$(CONFIG_USB)	+= dwc_otg_adp_stub.o
