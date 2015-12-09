/*
 * mml_i2c_regs.h --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2012, Maxim Integrated
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
 * Created on: Oct 24, 2012
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_I2C_REGS_H_
#define _MML_I2C_REGS_H_

/** @file mml_i2c_regs.h I2C Registers Header */

/** @defgroup MML_I2C I2C Driver
 * @ingroup MML_DRIVER
 */

/** @defgroup MML_I2C_REGS I2C Registers
 *
 * @note I2C IP or Specification version number
 *
 * @ingroup MML_I2C
 * @{
 */

/** @defgroup MML_I2C_REGS_CR I2C CR Register
 *
 *
 * @{
 */

#define MML_I2C_CR_OFST					0x0 //!< Control Register offset
#define MML_I2C_CR_DFLT					0x0 //!< Control Register default value

/** Bits Fields */
#define MML_I2C_CR_RXTHD_OFST				0 //!< RXTHD offset
#define MML_I2C_CR_RXTHD_MASK_NOOFST		0xf //!< RXTHD mask
#define MML_I2C_CR_RXTHD_MASK				( MML_I2C_CR_RXTHD_MASK_NOOFST << MML_I2C_CR_RXTHD_OFST ) //!< RXTHD mask

#define MML_I2C_CR_AUTOSTT_OFST			4 //!< AUTOSTT offset
#define MML_I2C_CR_AUTOSTT_MASK_NOOFST		0x1 //!< BF2 mask
#define MML_I2C_CR_AUTOSTT_MASK			( MML_I2C_CR_AUTOSTT_MASK_NOOFST << MML_I2C_CR_AUTOSTT_OFST ) //!< BF2 mask

#define MML_I2C_CR_START_OFST				5 //!< START offset
#define MML_I2C_CR_START_MASK_NOOFST		0x1 //!< BF3 mask
#define MML_I2C_CR_START_MASK				( MML_I2C_CR_START_MASK_NOOFST << MML_I2C_CR_START_OFST ) //!< BF3 mask

#define MML_I2C_CR_RESTART_OFST 			6 //!< RESTART offset
#define MML_I2C_CR_RESTART_MASK_NOOFST 	0x1 //!< RESTART MASK
#define MML_I2C_CR_RESTART_MASK 			( MML_I2C_CR_RESTART_MASK_NOOFST << MML_I2C_CR_RESTART_OFST ) //!< RESTART MASK

#define MML_I2C_CR_STOP_OFST 				7 //!< STOP offset
#define MML_I2C_CR_STOP_MASK_NOOFST 		0x1 //!< STOP MASK
#define MML_I2C_CR_STOP_MASK 				( MML_I2C_CR_STOP_MASK_NOOFST << MML_I2C_CR_STOP_OFST ) //!< STOP MASK

#define MML_I2C_CR_TXFLUSH_OFST 			8 //!< TXFLUSH offset
#define MML_I2C_CR_TXFLUSH_MASK_NOOFST 	0x1 //!< TXFLUSH MASK
#define MML_I2C_CR_TXFLUSH_MASK 			( MML_I2C_CR_TXFLUSH_MASK_NOOFST << MML_I2C_CR_TXFLUSH_OFST ) //!< TXFLUSH MASK

#define MML_I2C_CR_RXFLUSH_OFST 			9 //!< RXFLUSH offset
#define MML_I2C_CR_RXFLUSH_MASK_NOOFST 	0x1 //!< RXFLUSH MASK
#define MML_I2C_CR_RXFLUSH_MASK 			( MML_I2C_CR_RXFLUSH_MASK_NOOFST << MML_I2C_CR_RXFLUSH_OFST ) //!< RXFLUSH MASK

#define MML_I2C_CR_AFREAD_OFST 			10 //!< AFREAD offset
#define MML_I2C_CR_AFREAD_MASK_NOOFST 		0x1 //!< AFREAD MASK
#define MML_I2C_CR_AFREAD_MASK 			( MML_I2C_CR_AFREAD_MASK_NOOFST << MML_I2C_CR_AFREAD_OFST ) //!< AFREAD MASK

#define MML_I2C_CR_FREEZE_OFST 			11 //!< FREEZE offset
#define MML_I2C_CR_FREEZE_MASK_NOOFST 		0x1 //!< FREEZE MASK
#define MML_I2C_CR_FREEZE_MASK 			( MML_I2C_CR_FREEZE_MASK_NOOFST << MML_I2C_CR_FREEZE_OFST ) //!< FREEZE MASK

/** @} */ /* @defgroup MML_I2C_REGS_CR I2C CR Register */

/** @defgroup MML_I2C_REGS_SR I2C SR Register
 * @{
 */

#define MML_I2C_SR_OFST					0x4 //!< SR Register offset
#define MML_I2C_SR_DFLT					0x0 //!< SR Register default value

/** Bits Fields */
#define MML_I2C_SR_BUSY_OFST				0 //!< BUSY offset
#define MML_I2C_SR_BUSY_MASK_NOOFST		0x1 //!< BUSY mask no offset
#define MML_I2C_SR_BUSY_MASK				( MML_I2C_SR_BUSY_MASK_NOOFST << MML_I2C_SR_BUSY_OFST ) //!< BUSY mask

#define MML_I2C_SR_FROZEN_OFST				1 //!< FROZEN offset
#define MML_I2C_SR_FROZEN_MASK_NOOFST		0x1 //!< FROZEN mask no offset
#define MML_I2C_SR_FROZEN_MASK				( MML_I2C_SR_FROZEN_MASK_NOOFST << MML_I2C_SR_FROZEN_OFST ) //!< FROZEN mask

#define MML_I2C_SR_RSRVD_OFST				2 //!< RSRVD offset
#define MML_I2C_SR_RSRVD_MASK_NOOFST		0x3 //!< RSRVD mask no offset
#define MML_I2C_SR_RSRVD_MASK				( MML_I2C_SR_RSRVD_MASK_NOOFST << MML_I2C_SR_RSRVD_OFST ) //!< RSRVD mask

#define MML_I2C_SR_RXEMPTY_OFST			4 //!< RXEMPTY offset
#define MML_I2C_SR_RXEMPTY_MASK_NOOFST		0x1 //!< RXEMPTY mask no offset
#define MML_I2C_SR_RXEMPTY_MASK			( MML_I2C_SR_RXEMPTY_MASK_NOOFST << MML_I2C_SR_RXEMPTY_OFST ) //!< RXEMPTY mask

#define MML_I2C_SR_RXFULL_OFST				5 //!< RXFULL offset
#define MML_I2C_SR_RXFULL_MASK_NOOFST		0x1 //!< RXFULL mask no offset
#define MML_I2C_SR_RXFULL_MASK				( MML_I2C_SR_RXFULL_MASK_NOOFST << MML_I2C_SR_RXFULL_OFST ) //!< RXFULL mask

#define MML_I2C_SR_TXEMPTY_OFST			6 //!< TXEMPTY offset
#define MML_I2C_SR_TXEMPTY_MASK_NOOFST		0x1 //!< TXEMPTY mask no offset
#define MML_I2C_SR_TXEMPTY_MASK			( MML_I2C_SR_TXEMPTY_MASK_NOOFST << MML_I2C_SR_TXEMPTY_OFST ) //!< TXEMPTY mask

#define MML_I2C_SR_TXFULL_OFST				7 //!< TXFULL offset
#define MML_I2C_SR_TXFULL_MASK_NOOFST		0x1 //!< TXFULL mask no offset
#define MML_I2C_SR_TXFULL_MASK				( MML_I2C_SR_TXFULL_MASK_NOOFST << MML_I2C_SR_TXFULL_OFST ) //!< TXFULL mask

#define MML_I2C_SR_RXELT_OFST				8 //!< TXELT offset
#define MML_I2C_SR_RXELT_MASK_NOOFST		0xf //!< TXELT mask no offset
#define MML_I2C_SR_RXELT_MASK				( MML_I2C_SR_RXELT_MASK_NOOFST << MML_I2C_SR_RXELT_OFST ) //!< TXELT mask

#define MML_I2C_SR_TXELT_OFST				12 //!< RXELT offset
#define MML_I2C_SR_TXELT_MASK_NOOFST		0xf //!< RXELT mask no offset
#define MML_I2C_SR_TXELT_MASK				( MML_I2C_SR_TXELT_MASK_NOOFST << MML_I2C_SR_TXELT_OFST ) //!< RXELT mask

#define MML_I2C_SR_RSRVD1_OFST				16 //!< RSRVD1
#define MML_I2C_SR_RSRVD1_MASK_NOOFST		0xffff //!< RSRVD1 no offset
#define MML_I2C_SR_RSRVD1_MASK				( MML_I2C_SR_RSRVD1_MASK_NOOFST << MML_I2C_SR_RSRVD1_OFST ) //!< RSRVD1

/** @} */ /* @defgroup MML_I2C_REGS_SR */

/** @defgroup MML_I2C_REGS_IER I2C IER Register
 * @{
 */

#define MML_I2C_IER_OFST					0x8 //!< IER Register offset
#define MML_I2C_IER_DFLT					0x0 //!< IER Register default value


/* Bits Fields */

#define MML_I2C_IER_LOST_OFST				0 //!< Enable TR LOSS interrupt offset
#define MML_I2C_IER_LOST_MASK_NOOFST		0x1 //!< Enable TR LOSS mask no offset
#define MML_I2C_IER_LOST_MASK				( MML_I2C_IER_LOST_MASK_NOOFST << MML_I2C_IER_LOST_OFST ) //!< Enable TR LOSS mask

#define MML_I2C_IER_NOANS_OFST				1 //!< Enable NACK interrupt offset
#define MML_I2C_IER_NOANS_MASK_NOOFST		0x1 //!< Enable NACK mask no offset
#define MML_I2C_IER_NOANS_MASK				( MML_I2C_IER_NOANS_MASK_NOOFST << MML_I2C_IER_NOANS_OFST ) //!< Enable NACK mask

#define MML_I2C_IER_COMEN_OFST				2 //!< Enable tx completion interrupt offset
#define MML_I2C_IER_COMEN_MASK_NOOFST		0x1 //!< Enable tx completion mask no offset
#define MML_I2C_IER_COMEN_MASK				( MML_I2C_IER_COMEN_MASK_NOOFST << MML_I2C_IER_COMEN_OFST ) //!< Enable tx completion mask

#define MML_I2C_IER_RDYRD_OFST				3 //!< Enable next read ready interrupt offset
#define MML_I2C_IER_RDYRD_MASK_NOOFST		0x1 //!< Enable next read ready mask no offset
#define MML_I2C_IER_RDYRD_MASK				( MML_I2C_IER_RDYRD_MASK_NOOFST << MML_I2C_IER_RDYRD_OFST ) //!< Enable next read ready mask

#define MML_I2C_IER_FFRX_OFST				4 //!< Enable FIFO RX interrupt offset
#define MML_I2C_IER_FFRX_MASK_NOOFST		0x1 //!< Enable FIFO RX mask no offset
#define MML_I2C_IER_FFRX_MASK				( MML_I2C_IER_FFRX_MASK_NOOFST << MML_I2C_IER_FFRX_OFST ) //!< Enable FIFO RX mask

#define MML_I2C_IER_FFTX_OFST				5 //!< Enable FIFO TX interrupt offset
#define MML_I2C_IER_FFTX_MASK_NOOFST		0x1 //!< Enable FIFO TX mask no offset
#define MML_I2C_IER_FFTX_MASK				( MML_I2C_IER_FFTX_MASK_NOOFST << MML_I2C_IER_FFTX_OFST ) //!< Enable FIFO TX mask

#define MML_I2C_IER_FFTXH_OFST				6 //!< Enable FIFO TX Half empty interrupt offset
#define MML_I2C_IER_FFTXH_MASK_NOOFST		0x1 //!< Enable FIFO TX Half empty mask no offset
#define MML_I2C_IER_FFTXH_MASK				( MML_I2C_IER_FFTXH_MASK_NOOFST << MML_I2C_IER_FFTXH_OFST ) //!< Enable FIFO TX Half empty mask

/** @} */ /* @defgroup MML_I2C_REGS_IER */

/** @defgroup MML_I2C_REGS_ISR I2C ISR Register
 * @{
 */

#define MML_I2C_ISR_OFST					0xc //!< ISR Register offset
#define MML_I2C_ISR_DFLT					0x0 //!< ISR Register default value

/** Bits Fields */
#define MML_I2C_ISR_LOST_OFST				0 //!<  TR LOSS interrupt status offset
#define MML_I2C_ISR_LOST_MASK_NOOFST		0x1 //!<  TR LOSS  status mask
#define MML_I2C_ISR_LOST_MASK				( MML_I2C_ISR_LOST_MASK_NOOFST << MML_I2C_ISR_LOST_OFST ) //!<  TR LOSS  status mask

#define MML_I2C_ISR_NOANS_OFST				1 //!<  NACK interrupt status offset
#define MML_I2C_ISR_NOANS_MASK_NOOFST		0x2 //!<  NACK status mask
#define MML_I2C_ISR_NOANS_MASK				( MML_I2C_ISR_NOANS_MASK_NOOFST << MML_I2C_ISR_NOANS_OFST ) //!<  NACK status mask

#define MML_I2C_ISR_COMEN_OFST				2 //!<  tx completion interrupt status offset
#define MML_I2C_ISR_COMEN_MASK_NOOFST		0x1 //!<  tx completion status mask
#define MML_I2C_ISR_COMEN_MASK				( MML_I2C_ISR_COMEN_MASK_NOOFST << MML_I2C_ISR_COMEN_OFST )//!<  tx completion status mask

#define MML_I2C_ISR_RDYRD_OFST				3 //!<  next read ready interrupt status offset
#define MML_I2C_ISR_RDYRD_MASK_NOOFST		0x1 //!<  next read ready status mask
#define MML_I2C_ISR_RDYRD_MASK				( MML_I2C_ISR_RDYRD_MASK_NOOFST << MML_I2C_ISR_RDYRD_OFST )//!<  next read ready status mask

#define MML_I2C_ISR_FFRX_OFST				4 //!<  FIFO RX interrupt status offset
#define MML_I2C_ISR_FFRX_MASK_NOOFST		0x1 //!<  FIFO RX status mask
#define MML_I2C_ISR_FFRX_MASK				( MML_I2C_ISR_FFRX_MASK_NOOFST << MML_I2C_ISR_FFRX_OFST ) //!<  next read ready status mask

#define MML_I2C_ISR_FFTX_OFST				5 //!< FIFO TX interrupt status offset
#define MML_I2C_ISR_FFTX_MASK_NOOFST		0x1 //!< FIFO TX status mask
#define MML_I2C_ISR_FFTX_MASK				( MML_I2C_ISR_FFTX_MASK_NOOFST << MML_I2C_ISR_FFTX_OFST ) //!< FIFO TX status mask

#define MML_I2C_ISR_FFTXH_OFST				6 //!< FIFO TX Half empty interrupt status offset
#define MML_I2C_ISR_FFTXH_MASK_NOOFST		0x1 //!< FIFO TX Half empty status mask
#define MML_I2C_ISR_FFTXH_MASK				( MML_I2C_ISR_FFTXH_MASK_NOOFST << MML_I2C_ISR_FFTXH_OFST ) //!< FIFO TX Half empty status mask

/** @} */ /* @defgroup MML_I2C_REGS_ISR */

/** @defgroup MML_I2C_REGS_BRR I2C BRR Register
 * @{
 */

#define MML_I2C_BRR_OFST					0x10 //!< BRR Register offset
#define MML_I2C_BRR_DFLT					0x0 //!< BRR Register default value

/** Bits Fields */
#define MML_I2C_BRR_DIV_OFST				0 //!<  Divisor value offset
#define MML_I2C_BRR_DIV_MASK_NOOFST		0x7f //!<  Divisor value mask
#define MML_I2C_BRR_DIV_MASK				( MML_I2C_BRR_DIV_MASK_NOOFST << MML_I2C_BRR_DIV_OFST ) //!<  Divisor value mask

/** @} */ /* @defgroup MML_I2C_REGS_BRR */

/** @defgroup MML_I2C_REGS_RCR I2C RCR Register
 * @{
 */

#define MML_I2C_RCR_OFST					0x14 //!< RCR Register offset
#define MML_I2C_RCR_DFLT					0x0 //!< RCR Register default value

/** Bits Fields */
#define MML_I2C_RCR_RXCNT_OFST				0 //!<  Number of Bytes to Read offset
#define MML_I2C_RCR_RXCNT_MASK_NOOFST		0xffff //!<  Number of Bytes to Read mask no offset
#define MML_I2C_RCR_RXCNT_MASK				( MML_I2C_RCR_RXCNT_MASK_NOOFST << MML_I2C_RCR_RXCNT_OFST )//!<  Number of Bytes to Read mask

/** @} */ /* @defgroup MML_I2C_REGS_RCR */

/** @defgroup MML_I2C_REGS_DR I2C DR Register
 * @{
 */
//Changing with MML_I2C_TXR_OFST
#define MML_I2C_DR_OFST					0x1c //!< DR Register offset
#define MML_I2C_DR_DFLT					0x0 //!< DR Register default value

/** Bits Fields */
#define MML_I2C_DR_DATA_OFST				0 //!<  DATA offset
#define MML_I2C_DR_DATA_MASK_NOOFST		0xff //!<  DATA mask no offset
#define MML_I2C_DR_DATA_MASK				( MML_I2C_DR_DATA_MASK_NOOFST << MML_I2C_DR_DATA_OFST ) //!<  DATA mask

/** @} */ /* @defgroup MML_I2C_REGS_DR */

/** @defgroup MML_I2C_REGS_TXR I2C TXR Register
 * @{
 */

#define MML_I2C_TXR_OFST					0x18 //!< TXR Register offset
#define MML_I2C_TXR_DFLT					0x0 //!< TXR Register default value

/** Bits Fields */
#define MML_I2C_TXR_DATA_OFST				0 //!<  DATA offset
#define MML_I2C_TXR_DATA_MASK_NOOFST		0xff //!<  DATA mask no offset
#define MML_I2C_TXR_DATA_MASK				( MML_I2C_TXR_DATA_MASK_NOOFST << MML_I2C_TXR_DATA_OFST )//!<  DATA mask

/** @} */ /* @defgroup MML_I2C_REGS_DR */

#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */
/** I2C Registers.
 *
 */
typedef volatile struct
{
	/** Control register */
	volatile unsigned 						cr;
	/** Status register */
	volatile unsigned 						sr;
	/** Interrupt Enable register */
	volatile unsigned 						ier;
	/** Interrupt Status register */
	volatile unsigned 						isr;
	/** Baud Rate register */
	volatile unsigned 						brr;
	/** RX Count register */
	volatile unsigned 						rcr;
	/** TX FIFO register */
	volatile unsigned 						txr;
	/** Data Register */
	volatile unsigned 						dr;

} mml_i2c_regs_t;

#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_I2C_REGS */

#endif /* _MML_I2C_REGS_H_ */

/******************************************************************************/
/* EOF */
