/*
 * mml_wdt_regs.h --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2013, Maxim Integrated
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
 * THIS SOFTWARE IS PROVIDED BY MAXIM INTEGRATED ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* [INTERNAL] ------------------------------------------------------------------
 * Created on: Oct 10, 2013
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_WDT_REGS_H_
#define _MML_WDT_REGS_H_

/** @file mml_wdt_regs.h WDT Registers Header */

/** @defgroup MML_WDT WDT Driver
 * @ingroup MML_DRIVER
 */

/** @defgroup MML_WDT_REGS WDT Registers
 *
 * @note WDT IP or Specification version number
 *
 * @ingroup MML_WDT
 * @{
 */

/** @defgroup MML_WDT_REGS_CTRL WDT Control Register
 *
 * This register contains WDT control bits
 *
 * @{
 */

#define MML_WDT_CTRL_OFST						0x0 //!< CTRL Register offset
#define MML_WDT_CTRL_DFLT						0x0 //!< CTRL Register default value

#define MML_WDT_CTRL_INTPERIOD_OFST				0 //!< Interrupt period offset
#define MML_WDT_CTRL_INTPERIOD_MASK_NOOFST		0xf //!< Interrupt period mask no offset
#define MML_WDT_CTRL_INTPERIOD_MASK				( MML_WDT_CTRL_INTPERIOD_MASK_NOOFST << MML_WDT_CTRL_INTPERIOD_OFST ) //!< Interrupt period mask

#define MML_WDT_CTRL_PERIOD_2POW31				0 //!< 2^31 cycles
#define MML_WDT_CTRL_PERIOD_2POW30				1 //!< 2^30 cycles
#define MML_WDT_CTRL_PERIOD_2POW29				2 //!< 2^29 cycles
#define MML_WDT_CTRL_PERIOD_2POW28				3 //!< 2^28 cycles
#define MML_WDT_CTRL_PERIOD_2POW27				4 //!< 2^27 cycles
#define MML_WDT_CTRL_PERIOD_2POW26				5 //!< 2^26 cycles
#define MML_WDT_CTRL_PERIOD_2POW25				6 //!< 2^25 cycles
#define MML_WDT_CTRL_PERIOD_2POW24				7 //!< 2^24 cycles
#define MML_WDT_CTRL_PERIOD_2POW23				8 //!< 2^23 cycles
#define MML_WDT_CTRL_PERIOD_2POW22				9 //!< 2^22 cycles
#define MML_WDT_CTRL_PERIOD_2POW21				10 //!< 2^21 cycles
#define MML_WDT_CTRL_PERIOD_2POW20				11 //!< 2^20 cycles
#define MML_WDT_CTRL_PERIOD_2POW19				12 //!< 2^19 cycles
#define MML_WDT_CTRL_PERIOD_2POW18				13 //!< 2^18 cycles
#define MML_WDT_CTRL_PERIOD_2POW17				14 //!< 2^17 cycles
#define MML_WDT_CTRL_PERIOD_2POW16				15 //!< 2^16 cycles

#define MML_WDT_CTRL_RSTPERIOD_OFST				4 //!< Reset period offset
#define MML_WDT_CTRL_RSTPERIOD_MASK_NOOFST		0xf //!< Reset period mask no offset
#define MML_WDT_CTRL_RSTPERIOD_MASK				( MML_WDT_CTRL_RSTPERIOD_MASK_NOOFST << MML_WDT_CTRL_RSTPERIOD_OFST ) //!< Reset period mask

#define MML_WDT_CTRL_WDEN_OFST					8 //!< WDT Enable offset
#define MML_WDT_CTRL_WDEN_MASK_NOOFST			0x1 //!< WDT Enable mask no offset
#define MML_WDT_CTRL_WDEN_MASK					( MML_WDT_CTRL_WDEN_MASK_NOOFST << MML_WDT_CTRL_WDEN_OFST ) //!< WDT Enable mask

#define MML_WDT_CTRL_INTFLAG_OFST				9 //!< WDT Interrupt flag offset
#define MML_WDT_CTRL_INTFLAG_MASK_NOOFST		0x1 //!< WDT Interrupt flag mask no offset
#define MML_WDT_CTRL_INTFLAG_MASK				( MML_WDT_CTRL_INTFLAG_MASK_NOOFST << MML_WDT_CTRL_INTFLAG_OFST ) //!< WDT Interrupt flag mask

#define MML_WDT_CTRL_INTEN_OFST					10 //!< WDT Interrupt Enable offset
#define MML_WDT_CTRL_INTEN_MASK_NOOFST			0x1 //!< WDT Interrupt Enable mask no offset
#define MML_WDT_CTRL_INTEN_MASK					( MML_WDT_CTRL_INTEN_MASK_NOOFST << MML_WDT_CTRL_INTEN_OFST ) //!< WDT Interrupt flag mask

#define MML_WDT_CTRL_RSTEN_OFST					11 //!< WDT Reset Enable offset
#define MML_WDT_CTRL_RSTEN_MASK_NOOFST			0x1 //!< WDT Reset Enable mask no offset
#define MML_WDT_CTRL_RSTEN_MASK					( MML_WDT_CTRL_RSTEN_MASK_NOOFST << MML_WDT_CTRL_RSTEN_OFST ) //!< WDT Reset Enable mask

#define	MML_WDT_CTRL_WDRSTF_OFST				31 //!< WDT Reset Flag offset
#define	MML_WDT_CTRL_WDRSTF_MASK_NOOFST			0x1u //!< WDT Reset Flag mask no offset
#define	MML_WDT_CTRL_WDRSTF_MASK				( MML_WDT_CTRL_WDRSTF_MASK_NOOFST << MML_WDT_CTRL_WDRSTF_OFST ) //!< WDT Reset Flag mask

/** @} */ /* @defgroup MML_WDT_REGS_CTRL */

/** @defgroup MML_WDT_REGS_RST WDT Reset Register
 *
 * @{
 */

#define MML_WDT_RST_OFST						0x4 //!<  Reset Register offset
#define MML_WDT_RST_DFLT						0x0 //!< Reset Register default value

#define MML_WDT_RST_TRST_OFST					0 //!<  WDT TRST offset
#define MML_WDT_RST_TRST_MASK_NOOFST			0xff //!<  WDT TRST offset no offset
#define MML_WDT_RST_TRST_MASK					( MML_WDT_RST_TRST_MASK_NOOFST << MML_WDT_RST_TRST_OFST ) //!<  WDT TRST offset

#define	MML_WDT_RST_TRST_1ST_WORD				0x5a
#define	MML_WDT_RST_TRST_2ND_WORD				0xa5

/** @} */ /* @defgroup MML_WDT_REGS_RST */


#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */
/** WDT Registers.
 *
 */
typedef volatile struct
{
	/** control register */
	volatile unsigned int						control;
	/** reset register */
	volatile unsigned int						reset;

} mml_wdt_regs_t;

#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_WDT_REGS */

#endif /* _MML_WDT_REGS_H_ */

/******************************************************************************/
/* EOF */
