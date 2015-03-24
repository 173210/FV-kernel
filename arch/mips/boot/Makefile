#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# Copyright (C) 1995, 1998, 2001, 2002 by Ralf Baechle
# Copyright (C) 2004  Maciej W. Rozycki
#

#
# Some DECstations need all possible sections of an ECOFF executable
#
ifdef CONFIG_MACH_DECSTATION
  E2EFLAGS = -a
else
  E2EFLAGS =
endif

#
# Drop some uninteresting sections in the kernel.
# This is only relevant for ELF kernels but doesn't hurt a.out
#
drop-sections	= .reginfo .mdebug .comment .note .pdr .options .MIPS.options
strip-flags	= $(addprefix --remove-section=,$(drop-sections))

VMLINUX = vmlinux

all: vmlinux.ecoff vmlinux.srec addinitrd vmlinux.elf

vmlinux.ecoff: $(obj)/elf2ecoff $(VMLINUX)
	$(obj)/elf2ecoff $(VMLINUX) vmlinux.ecoff $(E2EFLAGS)

$(obj)/elf2ecoff: $(obj)/elf2ecoff.c
	$(HOSTCC) -o $@ $^

vmlinux.elf:	$(VMLINUX)
	$(STRIP) -o $(obj)/vmlinux.elf $(VMLINUX)

vmlinux.bin: $(VMLINUX)
	$(OBJCOPY) -O binary $(strip-flags) $(VMLINUX) $(obj)/vmlinux.bin

vmlinux.srec: $(VMLINUX)
	$(OBJCOPY) -S -O srec $(strip-flags) $(VMLINUX) $(obj)/vmlinux.srec

$(obj)/addinitrd: $(obj)/addinitrd.c
	$(HOSTCC) -o $@ $^

#
# Create XIP image
#
ifeq ($(CONFIG_XIP_KERNEL),y)
xipImage: $(VMLINUX)
	$(OBJCOPY) -O binary -R .data $(strip-flags) vmlinux vmlinux-text.bin
	$(OBJCOPY) -O binary -j .data vmlinux vmlinux-data.bin
	cat vmlinux-text.bin vmlinux-data.bin > $@
	rm -f vmlinux-text.bin vmlinux-data.bin
	@echo '  kernel: $@ is ready'
endif

archhelp:
	@echo	'* vmlinux.ecoff	- ECOFF boot image'
	@echo	'* vmlinux.srec		- SREC boot image'
	@echo	'* vmlinux.elf		- ELF boot image (stripped)'

clean-files += addinitrd \
	       elf2ecoff \
	       vmlinux.bin \
	       vmlinux.ecoff \
	       vmlinux.srec \
	       vmlinux.elf \
	       xipImage
