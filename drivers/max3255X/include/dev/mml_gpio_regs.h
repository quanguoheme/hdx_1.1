/*
 * mml_gpio_regs.h --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2014, Maxim Integrated
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
 * Created on: Jun 13, 2014
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_GPIO_REGS_H_
#define _MML_GPIO_REGS_H_

/** @file mml_gpio_regs.h GPIO Registers Header */

/** @defgroup MML_GPIO GPIO Driver
 * @ingroup MML_DRIVER
 *
 * @{
 */

/** @defgroup MML_GPIO_REGS GPIO Registers
 *
 * @note GPIO IP or Specification version number
 *
 * @ingroup MML_GPIO
 * @{
 */

/** @defgroup MML_GPIO_REGS_GPIOx_ER0	GPIO Function Enable Register (GPIOxER0)
 *
 * @li GPIO [n]	-	R/W: GPIOx.n Function Enable
 *
 * @{
 */
#define MML_GPIOx_EN_OFST						0x00000000 //!< Register offset
#define MML_GPIOx_EN_DFLT						0xffffffff //!< Register default value

/* Bits Fields */
#define MML_GPIOx_EN_BIT_OFST(n)				n //!<GPIOx.n offset
#define MML_GPIOx_EN_BIT_MASK(n)				( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOxER0 */

/** @defgroup MML_GPIO_REGS_GPIOxDIR	GPIO Output Enable Register (GPIOxDIR)
 *
 * @li GPIO [n]	-	R/W: GPIOx.n Direction *
 * @{
 */
#define MML_GPIOx_OUT_EN_OFST					0x0000000c //!< Register offset
#define MML_GPIOx_OUT_EN_DFLT					0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_OUT_EN_BIT_OFST(n)			n //!<GPIOx.n offset
#define MML_GPIOx_OUT_EN_BIT_MASK(n)			( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_DIR */

/** @defgroup MML_GPIO_REGS_GPIOx_DRO	GPIO Output Register (GPIOx_DRO)
 *
 * @li GPIO [n]	-	R/W: GPIOx.n Output Data Register
 *
 * @{
 */
#define MML_GPIOx_OUT_OFST						0x00000018 //!< Register offset
#define MML_GPIOx_OUT_DFLT						0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_OUT_BIT_OFST(n)				n //!<GPIOx.n offset
#define MML_GPIOx_OUT_BIT_MASK(n)				( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_DRO */

/** @defgroup MML_GPIO_REGS_GPIOx__OUT_SET	GPIO Output Set Register (GPIOx_SET)
 *
 * @li GPIO [n]	-	W: GPIOx.n Set
 *
 * @{
 */
#define MML_GPIOx_OUT_SET_OFST					0x0000001c //!< Register offset
#define MML_GPIOx_OUT_SET_DFLT					0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_OUT_SET_BIT_OFST(n)			n //!<GPIOx.n offset
#define MML_GPIOx_OUT_SET_BIT_MASK(n)			( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx__OUT_SET */


/** @defgroup MML_GPIO_REGS_GPIOx_CLR	GPIO Output Clear Register (GPIOx_CLR)
 *
 * @li GPIO [n]	-	W: GPIOx.n
 *
 * @{
 */
#define MML_GPIOx_OUT_CLR_OFST					0x00000020 //!< Register offset
#define MML_GPIOx_OUT_CLR_DFLT					0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_OUT_CLR_BIT_OFST(n)			n //!<GPIOx.n offset
#define MML_GPIOx_OUT_CLR_BIT_MASK(n)			( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_CLR */


/** @defgroup MML_GPIO_REGS_GPIOx_DRI	GPIO Input Data Register (GPIOx_DRI)
 *
 * @li GPIO [n]	-	R: GPIOx.n
 *
 * @{
 */
#define MML_GPIOx_IN_OFST						0x00000024 //!< Register offset
#define MML_GPIOx_IN_DFLT						0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_IN_BIT_OFST(n)				n //!<GPIOx.n offset
#define MML_GPIOx_IN_BIT_MASK(n)				( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_DRI */


/** @defgroup MML_GPIO_REGS_GPIOx_IMR0	GPIO Interrupt Mode Register 0 (GPIOx_IMR0)
 *
 * @li GPIO [n]	-	R/W: GPIOx.n
 *
 * @{
 */

#define MML_GPIOx_INT_MOD_OFST					0x00000028 //!< Register offset
#define MML_GPIOx_INT_MOD_DFLT					0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_INT_MOD_BIT_OFST(n)			n //!<GPIOx.n offset
#define MML_GPIOx_INT_MOD_BIT_MASK(n)			( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_IMR0 */


/** @defgroup MML_GPIO_REGS_GPIOx_IMR1	GPIO Interrupt Mode Register 1 (GPIOx_IMR1)
 *
 * @li GPIO [n]	-	R/W: GPIOx.n
 *
 * @{
 */

#define MML_GPIOx_INT_POL_OFST					0x0000002c //!< Register offset
#define MML_GPIOx_INT_POL_DFLT					0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_INT_POL_BIT_OFST(n)			n //!<GPIOx.n offset
#define MML_GPIOx_INT_POL_BIT_MASK(n)			( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_IMR1 */


/** @defgroup MML_GPIO_REGS_GPIOx_IER	GPIO Interrupt Enable Register (GPIOx_IER)
 *
 * @li GPIO [n]	-	R/W: GPIOx.n
 *
 * @{
 */

#define MML_GPIOx_INT_EN_OFST					0x00000034 //!< Register offset
#define MML_GPIOx_INT_EN_DFLT					0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_INT_EN_BIT_OFST(n)			n //!<GPIOx.n offset
#define MML_GPIOx_INT_EN_BIT_MASK(n)			( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_IER */


/** @defgroup MML_GPIO_REGS_GPIOx_IESET	GPIO Interrupt Enable Set Register (GPIOx_IESET)
 *
 * @li GPIO [n]	-	W: GPIOx.n
 *
 * @{
 */

#define MML_GPIOx_INT_EN_SET_OFST				0x00000038 //!< Register offset
#define MML_GPIOx_INT_EN_SET_DFLT				0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_INT_EN_SET_BIT_OFST(n)		n //!<GPIOx.n offset
#define MML_GPIOx_INT_EN_SET_BIT_MASK(n)		( 1 << n )//!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_IESET */


/** @defgroup MML_GPIO_REGS_GPIOx_IECLR		GPIO Interrupt Enable Clear Register (GPIOx_IECLR)
 *
 * @li GPIO [n]	-	W: GPIOx.n
 *
 * @{
 */

#define MML_GPIOx_INT_EN_CLR_OFST				0x0000003c //!< Register offset
#define MML_GPIOx_INT_EN_CLR_DFLT				0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_INT_EN_CLR_BIT_OFST(n)		n //!<GPIOx.n offset
#define MML_GPIOx_INT_EN_CLR_BIT_MASK(n)		( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_IECLR */


/** @defgroup MML_GPIO_REGS_GPIOx_ISR	GPIO Interrupt Status Register (GPIOx_ISR)
 *
 * @li GPIO [n]	-	R: GPIOx.n
 *
 * @{
 */

#define MML_GPIOx_INT_STAT_OFST					0x00000040 //!< Register offset
#define MML_GPIOx_INT_STAT_DFLT					0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_INT_STAT_BIT_OFST(n)			n //!<GPIOx.n offset
#define MML_GPIOx_INT_STAT_BIT_MASK(n)			( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_ISR */


/** @defgroup MML_GPIO_REGS_GPIOx_ISCLR	GPIO Interrupt Status Clear Register (GPIOx_ISCLR)
 *
 * @li GPIO [n]	-	W: GPIOx.n
 *
 * @{
 */

#define MML_GPIOx_INT_CLR_OFST					0x00000048 //!< Register offset
#define MML_GPIOx_INT_CLR_DFLT					0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_INT_CLR_BIT_OFST(n)			n //!<GPIOx.n offset
#define MML_GPIOx_INT_CLR_BIT_MASK(n)			( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_ISCLR */


/** @defgroup MML_GPIO_REGS_GPIOx_WAKE_EN	GPIO Power Management Enable Register (GPIOx_WAKE_EN)
 *
 * @li GPIO [n]	-	R/W: GPIOx.n
 *
 * @{
 */

#define MML_GPIOx_WAKE_EN_OFST					0x0000004c //!< Register offset
#define MML_GPIOx_WAKE_EN_DFLT					0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_WAKE_EN_BIT_OFST(n)			n //!<GPIOx.n offset
#define MML_GPIOx_WAKE_EN_BIT_MASK(n)			( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_PMER */


/** @defgroup MML_GPIO_REGS_GPIOx_INT_DUAL_EDGE	GPIO Interrupt Dual Edge Mode Register (GPIOx_INT_DUAL_EDGE)
 *
 * @li GPIO [n]	-	R/W: GPIOx.n
 *
 * @{
 */

#define MML_GPIOx_INT_DUAL_EDGE_OFST			0x0000005c //!< Register offset
#define MML_GPIOx_INT_DUAL_EDGE_DFLT			0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_INT_DUAL_EDGE_BIT_OFST(n)		n //!<GPIOx.n offset
#define MML_GPIOx_INT_DUAL_EDGE_BIT_MASK(n)		( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_INT_DUAL_EDGE */


/** @defgroup MML_GPIO_REGS_GPIOx_PAD_CFG1	GPIO Pull Up Enable Register (GPIOx_PAD_CFG1)
 *
 * @li GPIO [n]	-	R/W: GPIOx.n
 *
 * @{
 */

#define MML_GPIOx_PAD_CFG1_OFST					0x00000060 //!< Register offset
#define MML_GPIOx_PAD_CFG1_DFLT					0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_PAD_CFG1_BIT_OFST(n)			n //!<GPIOx.n offset
#define MML_GPIOx_PAD_CFG1_BIT_MASK(n)			( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_PAD_CFG1 */


/** @defgroup MML_GPIO_REGS_GPIOx_PAD_CFG2	GPIO Pull Down Enable Register (GPIOx_PAD_CFG2)
 *
 * @li GPIO [n]	-	R/W: GPIOx.n
 *
 * @{
 */

#define MML_GPIOx_PAD_CFG2_OFST					0x00000064 //!< Register offset
#define MML_GPIOx_PAD_CFG2_DFLT					0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_PAD_CFG2_BIT_OFST(n)			n //!<GPIOx.n offset
#define MML_GPIOx_PAD_CFG2_BIT_MASK(n)			( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_PAD_CFG2 */


/** @defgroup MML_GPIO_REGS_GPIOx_EN1	GPIO Alternate Function Enable Register (GPIOx_EN1)
 *
 * @li GPIO [n]	-	R/W: GPIOx.n
 *
 * @{
 */
#define MML_GPIOx_EN1_OFST						0x00000068 //!< Register offset
#define MML_GPIOx_EN1_DFLT						0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_EN1_BIT_OFST(n)				n //!<GPIOx.n offset
#define MML_GPIOx_EN1_BIT_MASK(n)				( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_EN1 */


/** @defgroup MML_GPIO_REGS_GPIOx_DS	GPIO Alternate Function Enable Register (GPIOx_DS)
 *
 * @li GPIO [n]	-	R/W: GPIOx.n
 *
 * @{
 */

#define MML_GPIOx_DS_OFST						0x000000b0 //!< Register offset
#define MML_GPIOx_DS_DFLT						0x00000000 //!< Register default value

/* Bits Fields */
#define MML_GPIOx_DS_BIT_OFST(n)				n //!<GPIOx.n offset
#define MML_GPIOx_DS_BIT_MASK(n)				( 1 << n ) //!<GPIOx.n mask
/** @} */ /* defgroup MML_GPIO_REGS_GPIOx_DS */

#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */
/** GPIO Registers.
 *
 */
typedef volatile struct
{
	/** 0x00000000 */
	unsigned int								en;
	/** 0x00000004 - 8 */
	unsigned int								rfu0[2];
	/** 0x0000000c */
	unsigned int								out_en;
	/** 0x00000010 - 14 */
	unsigned int								rfu1[2];
	/** 0x00000018 */
	unsigned int								out;
	/** 0x0000001c */
	unsigned int								out_set;
	/** 0x00000020 */
	unsigned int								out_clr;
	/** 0x00000024 */
	unsigned int								in;
	/** 0x00000028 */
	unsigned int								int_mod;
	/** 0x0000002c */
	unsigned int								int_pol;
	/** 0x00000030 */
	unsigned int								rfu2;
	/** 0x00000034 */
	unsigned int								int_en;
	/** 0x00000038 */
	unsigned int								int_en_set;
	/** 0x0000003c */
	unsigned int								int_en_clr;
	/** 0x00000040 */
	unsigned int								int_stat;
	/** 0x00000044 */
	unsigned int								rfu3;
	/** 0x00000048 */
	unsigned int								int_clr;
	/** 0x0000004c */
	unsigned int								wake_en;
	/** 0x00000050 - 58 */
	unsigned int								rfu4[3];
	/** 0x0000005c */
	unsigned int								int_dual_edge;
	/** 0x00000060 */
	unsigned int								pad_cfg1;
	/** 0x00000064 */
	unsigned int								pad_cfg2;
	/** 0x00000068 */
	unsigned int								en1;
	/** 0x0000006c - ac */
	unsigned int								rfu5[17];
	/** 0x000000b0 */
	unsigned int								ds;

} mml_gpio_regs_t;

#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_GPIO_REGS */

/** @} */ /* @defgroup MML_GPIO */

#endif /* _MML_GPIO_REGS_H_ */

/******************************************************************************/
/* EOF */
