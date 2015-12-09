/*
 * mml_nvic_regs.h --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2014, Maxim Integrated Products
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY MAXIM INTEGRATED PRODUCTS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL MAXIM INTEGRATED PRODUCTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* [INTERNAL] ------------------------------------------------------------------
 * Created on: June 11, 2014
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision: 4487 $:  Revision of last commit
 * $Author: jeremy.kongs $:  Author of last commit
 * $Date: 2015-01-14 15:43:29 -0600 (Wed, 14 Jan 2015) $:  Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _NVIC_REGS_H_
#define _NVIC_REGS_H_

/** Global includes */
/** Other includes */
/** Local includes */


#define MML_NVIC_ICTR_OFST			0x004

#define MML_NVIC_ISER0_OFST			0x100
#define MML_NVIC_ISER1_OFST			0x104
#define MML_NVIC_ISER2_OFST			0x108
#define MML_NVIC_ISER3_OFST			0x10c
#define MML_NVIC_ISER4_OFST			0x110
#define MML_NVIC_ISER5_OFST			0x114
#define MML_NVIC_ISER6_OFST			0x118
#define MML_NVIC_ISER7_OFST			0x11c

#define MML_NVIC_ICER0_OFST			0x180
#define MML_NVIC_ICER1_OFST			0x184
#define MML_NVIC_ICER2_OFST			0x188
#define MML_NVIC_ICER3_OFST			0x18c
#define MML_NVIC_ICER4_OFST			0x190
#define MML_NVIC_ICER5_OFST			0x194
#define MML_NVIC_ICER6_OFST			0x198
#define MML_NVIC_ICER7_OFST			0x19c

#define MML_NVIC_ISPR0_OFST			0x200
#define MML_NVIC_ISPR1_OFST			0x204
#define MML_NVIC_ISPR2_OFST			0x208
#define MML_NVIC_ISPR3_OFST			0x20c
#define MML_NVIC_ISPR4_OFST			0x210
#define MML_NVIC_ISPR5_OFST			0x214
#define MML_NVIC_ISPR6_OFST			0x218
#define MML_NVIC_ISPR7_OFST			0x21c

#define MML_NVIC_ICPR0_OFST			0x280
#define MML_NVIC_ICPR1_OFST			0x284
#define MML_NVIC_ICPR2_OFST			0x288
#define MML_NVIC_ICPR3_OFST			0x28c
#define MML_NVIC_ICPR4_OFST			0x290
#define MML_NVIC_ICPR5_OFST			0x294
#define MML_NVIC_ICPR6_OFST			0x298
#define MML_NVIC_ICPR7_OFST			0x29c

#define MML_NVIC_IABR0_OFST			0x300
#define MML_NVIC_IABR1_OFST			0x304
#define MML_NVIC_IABR2_OFST			0x308
#define MML_NVIC_IABR3_OFST			0x30c
#define MML_NVIC_IABR4_OFST			0x310
#define MML_NVIC_IABR5_OFST			0x314
#define MML_NVIC_IABR6_OFST			0x318
#define MML_NVIC_IABR7_OFST			0x31c

#define MML_NVIC_VTOR_OFST			0xd08

#ifndef __ASSEMBLER__
/******************************************************************************/

/** INTC Registers.
 */
typedef volatile struct
{
    unsigned int				space00; // +0x000
    unsigned int				ictr; // +0x004
    unsigned char				space01[0x100 - 0x008]; // +0x008
    unsigned int				enable_set[8]; // +0x100
    unsigned char				space02[0x180 - 0x120]; // +0x120
    unsigned int				enable_clr[8]; // +0x180
    unsigned char				space03[0x200 - 0x1a0]; // +0x1a0
    unsigned int				pend_set[8]; // +0x200
    unsigned char				space04[0x280 - 0x220]; // +0x220
    unsigned int				pend_clr[8]; // +0x280
    unsigned char				space05[0x300 - 0x2a0]; // +0x2a0
    unsigned int				active[8]; // +0x300
    unsigned char				space06[0x400 - 0x320]; // +0x320
    unsigned char				priority[256]; // +0x400
    unsigned char				space07[0xd08 - 0x500]; // +0x500
    unsigned int				vtor; // +0xd08

} nvic_regs_t;

/** Interruption Priorities
 */
typedef unsigned char nvic_prio_t;

#endif /* __ASSEMBLER__ */

#endif /* _NVIC_REGS_H_ */

/******************************************************************************/
/* EOF */
