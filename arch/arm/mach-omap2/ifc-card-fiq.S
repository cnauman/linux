/*
 *  linux/arch/arm/mach-omap2/ifc-card-fiq.S
 *
 *  Based on linux/arch/arm/mach-omap1/ams-delta-fiq-handler.S
 *  Copyright (C) 1995, 1996 Russell King
 *  Copyright (C) 2004 Pete Trapps
 *  Copyright (C) 2006 Matt Callow
 *  Copyright (C) 2010 Janusz Krzysztofik
 *  Copyright (C) 2012 Craig Nauman
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#include <linux/linkage.h>

#include <plat/io.h>

/*
 * C function is contained in banked register r10
 */
	.text
	.global fiq_handler_end

ENTRY(fiq_handler_start)
        movs r10, r10       @ test for null
        moveq pc, lr        @ skip if C-function is not set

        mov ip, sp
        stmfd sp!, {r0-r7, ip, lr}

        @ call C function in r10 register
        mov lr, pc          @ save the return address
        mov pc, r10         @ call C-function

        ldmfd sp, {r0-r7, sp, lr}
        subs    pc, lr, #4              @ return from FIQ

fiq_handler_end:
	
/*****************************************************************/

/*
 * Check the size of the FIQ,
 * it cannot go beyond 0xffff0200, and is copied to 0xffff001c
 */
.if (fiq_handler_end - fiq_handler_start) > (0x200 - 0x1c)
	.err
.endif
