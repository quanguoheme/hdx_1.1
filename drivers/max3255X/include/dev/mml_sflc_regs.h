/*
 * mml_sflc_regs.h --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2012, Maxim Integrated Products
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
 * Created on: June 5, 2012
 * Author: Jeremy B. <jeremy.brodt@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision: 4579 $:  Revision of last commit
 * $Author: robert.muchsel $:  Author of last commit
 * $Date: 2015-01-16 22:20:55 -0600 (Fri, 16 Jan 2015) $:  Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_SFLC_REGS_H_
#define _MML_SFLC_REGS_H_

/** @file mml_sflc_regs.h SFLC Registers Header */

/** @defgroup MML_SFLC SFLC Driver
 * @ingroup MML_DRIVER
 */

#define MML_SFLC_ADDR_OFST						0x00000000 //!< Address register address
#define MML_SFLC_ADDR_DFLT						0x00000000 //!< Address register default value

/******************************************************************************/

#define MML_SFLC_CLKDIV_OFST					0x00000004 //!< Clock Divider register address
#define MML_SFLC_CLKDIV_DFLT					0x0000003c //!< Clock Divider default value

#define	MML_SFLC_CLKDIV_CLKDIV_OFST				0
#define	MML_SFLC_CLKDIV_CLKDIV_MASK_NOOFST		0xff
#define	MML_SFLC_CLKDIV_CLKDIV_MASK				( MML_SFLC_CLKDIV_CLKDIV_MASK_NOOFST << MML_SFLC_CLKDIV_CLKDIV_OFST )

/******************************************************************************/

#define MML_SFLC_CN_OFST						0x00000008 //!< Control register address
#define MML_SFLC_CN_DFLT						0x00000000 //!< Control register default value

#define	MML_SFLC_CN_WR_OFST						0
#define	MML_SFLC_CN_WR_MASK_NOOFST				0x1
#define	MML_SFLC_CN_WR_MASK						( MML_SFLC_CN_WR_MASK_NOOFST << MML_SFLC_CN_WR_OFST )

#define	MML_SFLC_CN_ME_OFST						1
#define	MML_SFLC_CN_ME_MASK_NOOFST				0x1
#define	MML_SFLC_CN_ME_MASK						( MML_SFLC_CN_ME_MASK_NOOFST << MML_SFLC_CN_ME_OFST )

#define	MML_SFLC_CN_PGE_OFST					2
#define	MML_SFLC_CN_PGE_MASK_NOOFST				0x1
#define	MML_SFLC_CN_PGE_MASK					( MML_SFLC_CN_PGE_MASK_NOOFST << MML_SFLC_CN_PGE_OFST )

#define	MML_SFLC_CN_ERASE_CODE_OFST				8
#define	MML_SFLC_CN_ERASE_CODE_MASK_NOOFST		0xff
#define	MML_SFLC_CN_ERASE_CODE_MASK				( MML_SFLC_CN_ERASE_CODE_MASK_NOOFST << MML_SFLC_CN_ERASE_CODE_OFST )

#define	MML_SFLC_CN_PEND_OFST					24
#define	MML_SFLC_CN_PEND_MASK_NOOFST			0x1
#define	MML_SFLC_CN_PEND_MASK					( MML_SFLC_CN_PEND_MASK_NOOFST << MML_SFLC_CN_PEND_OFST )

#define	MML_SFLC_CN_UNLOCK_OFST					28
#define	MML_SFLC_CN_UNLOCK_MASK_NOOFST			0xfu
#define	MML_SFLC_CN_UNLOCK_MASK					( MML_SFLC_CN_UNLOCK_MASK_NOOFST << MML_SFLC_CN_UNLOCK_OFST )

#define	MML_SFLC_CN_UNLOCK_VALUE				0x2

#define	MML_SFLC_CN_WIDTH_OFST					4
#define	MML_SFLC_CN_WIDTH_MASK_NOOFST			0x1
#define	MML_SFLC_CN_WIDTH_MASK					( MML_SFLC_CN_WIDTH_MASK_NOOFST << MML_SFLC_CN_WIDTH_OFST )

#define	MML_SFLC_CN_ERASE_CODE_NOP				0x00
#define	MML_SFLC_CN_ERASE_CODE_PAGE				0x55
#define	MML_SFLC_CN_ERASE_CODE_MASS				0xaa

#define	MML_SFLC_CN_ALL_CMDS_MASK				( MML_SFLC_CN_WR_MASK | \
													MML_SFLC_CN_ME_MASK | \
													MML_SFLC_CN_PGE_MASK )

/******************************************************************************/

#define MML_SFLC_INT_OFST						0x00000040 //!< Interrupt register address
#define MML_SFLC_INT_DFLT						0x00000000 //!< Interrupt Control default value

#define	MML_SFLC_INT_DONE_OFST					0
#define	MML_SFLC_INT_DONE_MASK_NOOFST			0x1
#define	MML_SFLC_INT_DONE_MASK					( MML_SFLC_INT_DONE_MASK_NOOFST << MML_SFLC_INT_DONE_OFST )

#define	MML_SFLC_INT_AF_OFST				    1
#define	MML_SFLC_INT_AF_MASK_NOOFST				0x1
#define	MML_SFLC_INT_AF_MASK					( MML_SFLC_INT_AF_MASK_NOOFST << MML_SFLC_INT_AF_OFST )

#define	MML_SFLC_INT_MASK						( MML_SFLC_INT_DONE_MASK | MML_SFLC_INT_AF_MASK )

#define	MML_SFLC_INT_DONEIE_OFST				8
#define	MML_SFLC_INT_DONEIE_MASK_NOOFST			0x1
#define	MML_SFLC_INT_DONEIE_MASK				( MML_SFLC_INT_DONEIE_MASK_NOOFST << MML_SFLC_INT_DONEIE_OFST )

#define	MML_SFLC_INT_AFIE_OFST					9
#define	MML_SFLC_INT_AFIE_MASK_NOOFST			0x1
#define	MML_SFLC_INT_AFIE_MASK					( MML_SFLC_INT_AFIE_MASK_NOOFST << MML_SFLC_INT_AFIE_OFST )

#define	MML_SFLC_INT_IE_MASK					( MML_SFLC_INT_DONEIE_MASK | MML_SFLC_INT_AFIE_MASK )

/******************************************************************************/

#define MML_SFLC_ACNTL_OFST						0x00000040 //!< Access Control register address
#define MML_SFLC_ACNTL_DFLT						0x00000000 //!< Access Control default value

#define	MML_SFLC_ACNTL_ADATA_OFST				0
#define	MML_SFLC_ACNTL_ADATA_MASK_NOOFST		0xffffffff
#define	MML_SFLC_ACNTL_ADATA_MASK				( MML_SFLC_ACNTL_ADATA_MASK_NOOFST << MML_SFLC_ACNTL_ADATA_OFST )

#define	MML_SFLC_ACNTL_MAGIC_WORD1				0x3a7f5ca3
#define	MML_SFLC_ACNTL_MAGIC_WORD2				0xa1e34f20
#define	MML_SFLC_ACNTL_MAGIC_WORD3				0x9608b2c1

#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */
/** SFLC Registers.
 *
 */
typedef volatile struct
{
	/** !< 0x00 Address */
	unsigned int							faddr;
	/** !< 0x04 Clock Divide */
	unsigned int							fckdiv;
	/** !< 0x08 Control */
	unsigned int							fcntl;
	/** !< 0x0C Serial FTM control */
	unsigned int							sftmc;
	/** !< 0x10 Serial FTM read data0 */
	unsigned int							sftmd0;
	/** !< 0x14 Serial FTM read data1 */
	unsigned int							sftmd1;
	/** !< 0x18 Serial FTM read data2 */
	unsigned int							sftmd2;
	/** !< 0x1C Serial FTM read data3 */
	unsigned int							sftmd3;
	/** !< 0x20 RFU*/
	unsigned int							rfu0;
	/** !< 0x24 Interrupt */
	unsigned int							fint;
	/** !< 0x28 Reserved*/
	unsigned int							rfu1[2];
	/** !< 0x30 Data Reg 0 */
	unsigned int							fdata0;
	/** !< 0x34 Data Reg 1 */
	unsigned int							rfu2[3];
	/** !< 0x40 Access control (unlock flash here!) */
	unsigned int							acntl;

} mml_sflc_regs_t;

#endif /* __ASSEMBLER__ */

/** @} *//* @defgroup MML_SFLC_REGS */

#endif /* _MML_SFLC_REGS_H_ */

/******************************************************************************/
/* EOF */
