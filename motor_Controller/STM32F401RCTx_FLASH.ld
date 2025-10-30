/* STM32F401RCTx Flash linker script */

ENTRY(Reset_Handler)

/* Memory regions */
MEMORY
{
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 256K
  RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 64K
}

/* Symbols provided to the application */
_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size  = 0x200;  /* 512 bytes */
_Min_Stack_Size = 0x400;  /* 1 KB */

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text) *(.text*)
    *(.glue_7) *(.glue_7t)
    *(.gnu.linkonce.t*)
    . = ALIGN(4);
    KEEP(*(.init))
    KEEP(*(.fini))
  } > FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata) *(.rodata*)
    . = ALIGN(4);
  } > FLASH

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > FLASH

  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
  } > FLASH

  .eh_frame_hdr : { *(.eh_frame_hdr) } > FLASH
  .eh_frame     : { KEEP(*(.eh_frame)) } > FLASH

  _etext  = .;
  _sidata = _etext;

  .data : AT ( _sidata )
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data) *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss) *(.bss*) *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } > RAM

  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = ALIGN(8);
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } > RAM

  /* Discard unwanted sections */
  /DISCARD/ :
  {
    *(.comment)
    *(.note*)
    *(.gnu*)
  }
}
