MEMORY
{
  FLASH (rwx) : ORIGIN = 0x8000000, LENGTH = 1024K
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 192K
}

SECTIONS
{
  .text :
  {
    *(vectors)
    *(.text)        /* Program code */
    *(.rodata)      /* Read only data */
  } > FLASH

  _end_text = .;
  .data :
  {
    _start_idata = .;
    *(.data)        /* Data memory */
    _end_idata = .;
  } > RAM AT > FLASH

  .bss :
  {
    _start_bss = .;
    *(.bss)         /* Zero-filled run time allocate data memory */
    _end_bss = .;
  } > RAM
}  
