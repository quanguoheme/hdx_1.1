/*
 * mml_icache_regs.h --
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

#ifndef _MML_ICACHE_REGS_H_
#define _MML_ICACHE_REGS_H_

/** @file mml_icache_regs.h ICACHE Registers Header */

/** @defgroup MML_ICACHE ICACHE Driver
 * @ingroup MML_DRIVER
 */

#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */
/** ICACHE Registers.
 *
 */
typedef volatile struct
{
    unsigned int				id; //!< 0x0000 Cache ID
    unsigned int				memcfg; //!< 0x0004 Memory Configuration
    unsigned int				rfu0[62]; //!< 0x0008-0x00FC
    unsigned int				ctrlstat; //!< 0x0100 Cache Control and Status
    unsigned int				rfu1[63]; //!< 0x0104-0x01FC
    unsigned int				rfu2[64]; //!< 0x0200-0x02FC
    unsigned int				rfu3[64]; //!< 0x0300-0x03FC
    unsigned int				rfu4[64]; //!< 0x0400-0x04FC
    unsigned int				rfu5[64]; //!< 0x0500-0x05FC
    unsigned int				rfu6[64]; //!< 0x0600-0x06FC
    unsigned int				invalidate; //!< 0x0700 Invalidate All

} mml_icache_regs_t;

#endif /* __ASSEMBLER__ */

/** @} *//* @defgroup MML_ICACHE_REGS */

#endif /* _MML_ICACHE_REGS_H_ */

/******************************************************************************/
/* EOF */
