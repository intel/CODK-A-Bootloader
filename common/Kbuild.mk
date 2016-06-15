obj-y += main.o
obj-y += version.o
obj-y += bootlogic.o
obj-y += balloc.o
obj-y += printk.o
obj-y += utils.o

obj-$(CONFIG_PANIC_DUMP) += panic_dump.o

# FIXME: change name
obj-$(CONFIG_OTA) += ota.o

# FIXME: change name
obj-$(CONFIG_SIGN) += sign.o
