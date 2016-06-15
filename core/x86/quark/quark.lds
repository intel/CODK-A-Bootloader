#include <partition.h>
#include "quark_se/boot_config.h"
#include "machine/soc/quark_se/cos_interface.h"

ENTRY(main)
OUTPUT_ARCH(i386)
OUTPUT_FORMAT("elf32-i386")

MEMORY
{
    flash (r)  : ORIGIN = BASE_FLASH_ADDR     , LENGTH = BOOT_FLASH_SIZE
    esram (rw) : ORIGIN = BOOT_RAM_START_ADDR , LENGTH = COS_STARTUP - BOOT_RAM_START_ADDR
}

SECTIONS
{
   . = ORIGIN(flash);
   _image_start = .;

  .version_header_section () :
  {
    *(.version_header)
    KEEP(*(.version_header*))
  } >flash

  . = ALIGN(4);

   _text_base = .;
  .text :
  {
   *(.text.entry)
   KEEP(*(.text.entry*))
   *(.text)
   *(.text.last)
   *(.text.*)
  } >flash

  
  . = ALIGN(4);
  .rdata : ALIGN(4)
  {
   *(.rdata)
   *(.rdata.*)
   _init_data = .;
  } >flash

  . = ORIGIN(esram);
  . = ALIGN(4);

  /* Data load address is here, but appears in the image after .rdata */
  .data : AT ( _init_data )
  {
   __data_lma = LOADADDR(.data); __data_vma = .;
   KEEP(*(.usb_shared_if))
   *(.data)
   *(.data.*)
   . = ALIGN(4);
  } > esram

  __data_size = SIZEOF( .data );

  . = ALIGN(4);
  .bss :
  {
   __bss_start = .;
   *(.bss)
   *(.bss.*)
   *(COMMON)
   __bss_end = .;
  } > esram

   __bss_num_words = (__bss_end - __bss_start) >> 2;

  _image_end = .;

 .comment 0 : { *(.comment) }

  ASSERT( !(_image_end % 4), "image not 4 bytes aligned")

  /* Reserve panic section at end of RAM in order to
   * avoid overwriting panic dumps with stack */
  .panic_section ORIGIN(esram) + LENGTH(esram) - PANIC_DUMP_MAX_SIZE:
  {
   __panic_start__ = .;
   . += PANIC_DUMP_MAX_SIZE;
   __panic_end__ = .;
  } > esram

  /* Compute new stack top address */
  __StackTop = ORIGIN(esram) + LENGTH(esram) - SIZEOF(.panic_section);
}
