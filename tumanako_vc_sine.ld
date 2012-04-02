/*
 * This file is part of the tumanako project.
 *
 * Copyright (C) 2009 Philip Court (philip@greenstage.co.nz)
 * Copyright (C) 2012 Bernard Mentink <bmentink@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Linker script for Olimex STM32-H103 (STM32F103RBT6, 128K flash, 20K RAM). */

OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(reset_handler) 

/* Define memory regions. */
MEMORY
{
	rom (rx)    : ORIGIN = 0x08000000, LENGTH = 128K
	ram (rwx)   : ORIGIN = 0x20000000, LENGTH = 20K
}

/*
SECTIONS {
	/DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) }
} */

/* Include the common ld script from libopenstm32. */
INCLUDE libopencm3_stm32f1.ld

