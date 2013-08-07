/******************************************************************************
 *
 * project0.ld - Linker configuration file for project0.
 *
 * Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
 * Software License Agreement
 * 
 * Texas Instruments (TI) is supplying this software for use solely and
 * exclusively on TI's microcontroller products. The software is owned by
 * TI and/or its suppliers, and is protected under applicable copyright
 * laws. You may not combine this software with "viral" open-source
 * software in order to form a larger program.
 * 
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 * NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 * NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 * CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 * 
 * This is part of revision 9453 of the EK-LM4F120XL Firmware Package.
 *
 * MODIFIED By MARK Roy 2013 for use with the Stellarap 3D Printer Firmware Project.
 * 
 *****************************************************************************/

/* Highest address of the user mode stack */
_estack = 0x20008000;    /* end of 32K RAM */

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size  = 0;      /* required amount of heap  */
_Min_Stack_Size = 0x100;  /* required amount of stack */


MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x00040000
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00008000
}

SECTIONS
{
    .text :
    {
        _text = .;
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
        _etext = .;
    } > FLASH

    .data : AT(ADDR(.text) + SIZEOF(.text))
    {
        _data = .;
        *(vtable)
        *(.data*)
        _edata = .;
    } > SRAM

    .bss :
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM
}
