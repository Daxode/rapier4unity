/* android_linker.ld */
SECTIONS {
  . = ALIGN(16384); /* 16KB alignment */
  .text : { *(.text .text.*) }
  .data : { *(.data .data.*) }
  .bss : { *(.bss .bss.*) }
  /* Add other sections as needed */
}