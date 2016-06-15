#
# Copyright (c) 2016, Intel Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

BOOTLOADER_ROOT ?= .
OUT ?= $(BOOTLOADER_ROOT)/out/current
BOOTLOADER_KBUILD_OUT  ?= $(OUT)/kbuild
BOOTLOADER_KCONFIG_FILE ?= $(OUT)/.config
ifndef COS_BLOB
$(error "COS not defined")
endif
# arduino101_firmware bsp includes
TD_BSP_INCS = $(T)/arduino101_firmware/bsp/include

# FIXME: board defconfig file shall not be hard-coded
BOOTLOADER_DEFCONFIG ?= $(BOOTLOADER_ROOT)/board/intel/configs/$(BOARD)_defconfig

# FIXME: linker script shall not be hard-coded
BOOTLOADER_LDS ?= $(BOOTLOADER_ROOT)/core/x86/quark/quark.lds

CROSS_COMPILE ?= $(T)/arduino101_firmware/external/gcc-i586-pc-elf/bin/i586-pc-elf-

include $(BOOTLOADER_ROOT)/build/Makefile.version
include $(BOOTLOADER_ROOT)/build/Makefile.vars
include $(BOOTLOADER_ROOT)/build/Makefile.kconfig

BOOTLOADER_KCONFIG_HEADER := $(BOOTLOADER_KBUILD_OUT)/config.h

$(BOOTLOADER_KCONFIG_HEADER): $(BOOTLOADER_KCONFIG_FILE)
	@echo "Creating Kconfig header:" $(@:$(T)/%=%)
	$(AT)mkdir -p $(BOOTLOADER_KBUILD_OUT)
	$(AT)$(SED) $< -e 's/#.*//' > $@
	$(AT)$(SED) -i $@ -e 's/\(CONFIG_.*\)=/#define \1 /'


$(OUT) $(BOOTLOADER_KBUILD_OUT):
	@mkdir -p $@

$(OUT)/.config:  $(BOOTLOADER_DEFCONFIG) | $(OUT)
	@cp $< $@


$(OUT)/bootloader.lds: $(BOOTLOADER_LDS) | $(OUT)
	$(CROSS_COMPILE)gcc -E -P -o $@ -ansi -D__ASSEMBLY__  -x assembler-with-cpp -P $< -I$(BOOTLOADER_ROOT)/include/ -I$(TD_BSP_INCS)


CFLAGS := -Os -m32 -Wall -Wextra -Werror -nostdlib  -ffreestanding -nostartfiles -nodefaultlibs -g3

# FIXME: relax warnings for now
CFLAGS += -Wno-unused-parameter
CFLAGS += -Wno-sign-compare
CFLAGS += -Wno-unused-value
CFLAGS += -Wno-unused-variable
CFLAGS += -fno-exceptions -fno-asynchronous-unwind-tables
CFLAGS += -fdata-sections -ffunction-sections -Wl,--gc-sections

CFLAGS += -include $(BOOTLOADER_KCONFIG_HEADER)

CFLAGS += -I$(TD_BSP_INCS)

$(BOOTLOADER_KBUILD_OUT)/built-in.o: $(OUT)/.config $(BOOTLOADER_KCONFIG_HEADER) FORCE
	$(AT)$(MAKE) -C $(BOOTLOADER_ROOT)/ -f $(BOOTLOADER_ROOT)/build/Makefile.build \
		SRC=. \
		INC_ROOT=include \
		OUT=$(BOOTLOADER_KBUILD_OUT) \
		KCONFIG=$(BOOTLOADER_KCONFIG_FILE) \
		KCONFIG_AUTOHEADER=$(BOOTLOADER_KBUILD_OUT) \
		CFLAGS="$(CFLAGS) $(EXTRA_BUILD_CFLAGS)" \
		CC=$(CC) \
		AR=$(AR) \
		LD=$(LD)

$(BOOTLOADER_KBUILD_OUT)/cos.o: $(COS_BLOB) $(BOOTLOADER_KCONFIG_HEADER)
	$(AT)$(CC) $(CFLAGS) $(EXTRA_BUILD_CFLAGS) -o $@ -c $<

$(OUT)/bootloader.bin: $(OUT)/bootloader.elf
	@echo $(ANSI_RED)"[bX]"$(ANSI_OFF) $(@:$(T)/%=%)
	$(AT)$(OBJCOPY) -O binary $< $@
	$(AT)$(BINARY_VERSION_HEADER) \
		--major $(VERSION_MAJOR) \
		--minor $(VERSION_MINOR) \
		--patch $(VERSION_PATCH) \
		--version_string ARD1QRKBLR-$(VERSION_STRING_SUFFIX) \
		$@ $(DEV_NULL)

$(OUT)/bootloader.elf: $(BOOTLOADER_KBUILD_OUT)/built-in.o $(BOOTLOADER_KBUILD_OUT)/cos.o $(OUT)/bootloader.lds
	@echo $(ANSI_RED)"[bLD]"$(ANSI_OFF) $(@:$(T)/%=%)
	$(AT)$(LD) $(LDFLAGS) $(LDFLAGS_bootloader) -o $@ \
      -T $(OUT)/bootloader.lds \
      --start-group $(BOOTLOADER_KBUILD_OUT)/built-in.o $(BOOTLOADER_KBUILD_OUT)/cos.o --end-group \
      -Map $(OUT)/bootloader.map --gc-sections

.PHONY: bootloader
bootloader: $(OUT)/bootloader.bin

all: kconfig bootloader

clean:
	@rm -rf $(OUT)

FORCE:
