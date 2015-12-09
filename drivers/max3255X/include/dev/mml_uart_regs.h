/*
 * mml_uart_regs.h --
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

#ifndef _MML_UART_REGS_H_
#define _MML_UART_REGS_H_

/** @file mml_uart_regs.h UART Registers Header */

/** @defgroup MML_UART UART Driver */

/** @defgroup MML_UART_REGS UART Registers
 *
 * @note UART IP or Specification version number
 *
 * @ingroup MML_UART
 * @{
 */

/** @defgroup MML_UART_REGS_CR Control Register (CR)
 *
 * @li RXTHD [3:0]	- R/W: receive FIFO threshold
 * @li PAREN [4] 	- R/W: parity enable
 * @li PAREO [5]	- R/W: parity mode (odd or even)
 * @li PARMD [6]	- R/W: parity mode
 * @li RFU1 [7]		- R: reserved
 * @li TXFLUSH [8]	- R/W: flush transmit FIFO, clear by hw
 * @li RXFLUSH [9]	- R/W: flush receive FIFO , clear by hw
 * @li SIZE [11:10] - R/W: data format, number of bits per data
 * @li STOP [12]	- R/W: number of stop bits
 * @li RTSCTS [13]	- R/W: hardware flow control enable
 * @li RFU2 [31:14]	- R: reserved
 *
 * @{
 */
#define MML_UART_CR_OFST						0x00000000 //!< Register offset
#define MML_UART_CR_DFLT						0x00000000 //!< Register default value

/** Bits Fields */
#define MML_UART_CR_RXTHD_OFST					0 //!< RXTHD offset
#define MML_UART_CR_RXTHD_MASK_NOOSFT			0xf //!< RXTHD mask
#define MML_UART_CR_RXTHD_MASK					( MML_UART_CR_RXTHD_MASK_NOOSFT << MML_UART_CR_RXTHD_OFST ) //!< RXTHD mask

#define MML_UART_CR_PAREN_OFST					4 //!< PAREN offset
#define MML_UART_CR_PAREN_MASK_NOOFST			0x1 //!< PAREN mask no offset
#define MML_UART_CR_PAREN_MASK					( MML_UART_CR_PAREN_MASK_NOOFST << MML_UART_CR_PAREN_OFST ) //!< PAREN mask

#define MML_UART_CR_PAREO_OFST					5 //!< PAREO offset
#define MML_UART_CR_PAREO_MASK_NOOFST			0x1 //!< PAREO mask no offset
#define MML_UART_CR_PAREO_MASK					( MML_UART_CR_PAREO_MASK_NOOFST << MML_UART_CR_PAREO_OFST ) //!< PAREO mask

#define MML_UART_CR_PARMD_OFST					6 //!< PARMD offset
#define MML_UART_CR_PARMD_MASK_NOOFST			0x1 //!< PARMD mask no offset
#define MML_UART_CR_PARMD_MASK					( MML_UART_CR_PARMD_MASK_NOOFST << MML_UART_CR_PARMD_OFST ) //!< PARMD mask

#define MML_UART_CR_RFU1_OFST					7 //!< RFU1 offset
#define MML_UART_CR_RFU1_MASK_NOOFST			0x1 //!< RFU1 mask no offset
#define MML_UART_CR_RFU1_MASK					( MML_UART_CR_RFU1_MASK_NOOFST << MML_UART_CR_RFU1_OFST )0x80 //!< RFU1 mask

#define MML_UART_CR_TXFLUSH_OFST				8 //!< TXFLUSH offset
#define MML_UART_CR_TXFLUSH_MASK_NOOFST			0x1 //!< TXFLUSH mask no offset
#define MML_UART_CR_TXFLUSH_MASK				( MML_UART_CR_TXFLUSH_MASK_NOOFST << MML_UART_CR_TXFLUSH_OFST ) //!< TXFLUSH mask

#define MML_UART_CR_RXFLUSH_OFST				9 //!< RXFLUSH offset
#define MML_UART_CR_RXFLUSH_MASK_NOOFST			0x1 //!< RXFLUSH mask no offset
#define MML_UART_CR_RXFLUSH_MASK				( MML_UART_CR_RXFLUSH_MASK_NOOFST << MML_UART_CR_RXFLUSH_OFST ) //!< RXFLUSH mask

#define MML_UART_CR_SIZE_OFST					10 //!< SIZE offset
#define MML_UART_CR_SIZE_MASK_NOOFST			0x3 //!< SIZE mask no offset
#define MML_UART_CR_SIZE_MASK					( MML_UART_CR_SIZE_MASK_NOOFST << MML_UART_CR_SIZE_OFST ) //!< SIZE mask

#define MML_UART_CR_STOP_OFST					12 //!< STOP offset
#define MML_UART_CR_STOP_MASK_NOOFST			0x1 //!< STOP mask no offset
#define MML_UART_CR_STOP_MASK					( MML_UART_CR_STOP_MASK_NOOFST << MML_UART_CR_STOP_OFST ) //!< STOP mask

#define MML_UART_CR_RTSCTS_OFST					13 //!< CTSRTS offset
#define MML_UART_CR_RTSCTS_MASK_NOOFST			0x1 //!< CTSRTS mask no offset
#define MML_UART_CR_RTSCTS_MASK					( MML_UART_CR_RTSCTS_MASK_NOOFST << MML_UART_CR_RTSCTS_OFST ) //!< CTSRTS mask

#define MML_UART_CR_RFU2_OFST					14 //!< RFU2 offset
#define MML_UART_CR_RFU2_MASK_NOOFST			0x3ffff //!< RFU2 mask no offset
#define MML_UART_CR_RFU2_MASK					( MML_UART_CR_RFU2_MASK_NOOFST << MML_UART_CR_RFU2_OFST ) //!< RFU2 mask

/** Enumeration of the different value for a bits field . We recommend to use human readable value. */
#define MML_UART_CR_PAREN_DISABLE				0 //!< no parity bit
#define MML_UART_CR_PAREN_ENABLE				1 //!< one parity bit

#define MML_UART_CR_PAREO_ODD					1 //!< odd parity
#define MML_UART_CR_PAREO_EVEN					0 //!< even parity

#define MML_UART_CR_PARMD_O						1 //!< parity based on the number of 0s
#define MML_UART_CR_PARMD_1						0 //!< parity based on the number of 1s

#define MML_UART_CR_TXFLUSH_FLUSH				1 //!< flush TX FIFO

#define MML_UART_CR_RXFLUSH_FLUSH				1 //!< flush RX FIFO

#define MML_UART_CR_SIZE_5						0 //!< 5-bit data format
#define MML_UART_CR_SIZE_6						1 //!< 6-bit data format
#define MML_UART_CR_SIZE_7						2 //!< 7-bit data format
#define MML_UART_CR_SIZE_8						3 //!< 8-bit data format

#define MML_UART_CR_RTSCTS_DISABLE				0 //!< hardware flow control disable
#define MML_UART_CR_RTSCTS_ENABLE				1 //!< hardware flow control enable

/** @} */ /* @defgroup MML_UART_REGS_CR */


/** @defgroup MML_UART_REGS_SR Status Register (SR)
 *
 * @li TXBUSY [0]	- R: UART is transmitting data
 * @li RXBUSY [1] 	- R: UART is receiving data
 * @li RFU1 [2]		- R: reserved
 * @li RXEMPTY [6]	- R: RX FIFO empty flag
 * @li RXFULL [6]	- R: TX FIFO full flag
 * @li TXEMPTY [7]	- R: TX FIFO empty flag
 * @li TXFULL [6]	- R: TX FIFO full flag
 * @li RXELT [7]	- R: number of element in RX FIFO
 * @li TXELT [6]	- R: number of element in TX FIFO
 * @li RFU2 [31:16]	- R: reserved
 *
 * @{
 */
#define MML_UART_SR_OFST						0x00000004 //!< Register offset
#define MML_UART_SR_DFLT						0x00000050 //!< Register default value

/** Bits Fields */
#define MML_UART_SR_TXBUSY_OFST					0 //!< TXBUSY offset
#define MML_UART_SR_TXBUSY_MASK_NOOFST			0x1 //!< TXBUSY mask no offset
#define MML_UART_SR_TXBUSY_MASK					( MML_UART_SR_TXBUSY_MASK_NOOFST << MML_UART_SR_TXBUSY_OFST ) //!< TXBUSY mask

#define MML_UART_SR_RXBUSY_OFST					1 //!< RXBUSY offset
#define MML_UART_SR_RXBUSY_MASK_NOOFST			0x1 //!< RXBUSY mask no offset
#define MML_UART_SR_RXBUSY_MASK					( MML_UART_SR_RXBUSY_MASK_NOOFST << MML_UART_SR_RXBUSY_OFST ) //!< RXBUSY mask

#define MML_UART_SR_RFU1_OFST					2 //!< RFU1 offset
#define MML_UART_SR_RFU1_MASK_NOOFST			0x3 //!< RFU1 mask no offset
#define MML_UART_SR_RFU1_MASK					( MML_UART_SR_RFU1_MASK_NOOFST << MML_UART_SR_RFU1_OFST ) //!< RFU1 mask

#define MML_UART_SR_RXEMPTY_OFST				4 //!< RXEMPTY offset
#define MML_UART_SR_RXEMPTY_MASK_NOOFST			0x1 //!< RXEMPTY mask no offset
#define MML_UART_SR_RXEMPTY_MASK				( MML_UART_SR_RXEMPTY_MASK_NOOFST << MML_UART_SR_RXEMPTY_OFST ) //!< RXEMPTY mask

#define MML_UART_SR_RXFULL_OFST					5 //!< RXFULL offset
#define MML_UART_SR_RXFULL_MASK_NOOFST			0x1 //!< RXFULL mask no offset
#define MML_UART_SR_RXFULL_MASK					( MML_UART_SR_RXFULL_MASK_NOOFST << MML_UART_SR_RXFULL_OFST ) //!< RXFULL mask

#define MML_UART_SR_TXEMPTY_OFST				6 //!< TXEMPTY offset
#define MML_UART_SR_TXEMPTY_MASK_NOOFST			0x1 //!< TXEMPTY mask no offset
#define MML_UART_SR_TXEMPTY_MASK				( MML_UART_SR_TXEMPTY_MASK_NOOFST << MML_UART_SR_TXEMPTY_OFST ) //!< TXEMPTY mask

#define MML_UART_SR_TXFULL_OFST					7 //!< TXFULL offset
#define MML_UART_SR_TXFULL_MASK_NOOFST			0x1 //!< TXFULL mask no offset
#define MML_UART_SR_TXFULL_MASK					( MML_UART_SR_TXFULL_MASK_NOOFST << MML_UART_SR_TXFULL_OFST ) //!< TXFULL mask

#define MML_UART_SR_RXELT_OFST					8 //!< RXELT offset
#define MML_UART_SR_RXELT_MASK_NOOFST			0xf //!< RXELT mask no offset
#define MML_UART_SR_RXELT_MASK					( MML_UART_SR_RXELT_MASK_NOOFST << MML_UART_SR_RXELT_OFST ) //!< RXELT mask

#define MML_UART_SR_TXELT_OFST					12 //!< TXELT offset
#define MML_UART_SR_TXELT_MASK_NOOFST			0xf //!< TXELT mask no offset
#define MML_UART_SR_TXELT_MASK					( MML_UART_SR_TXELT_MASK_NOOFST << MML_UART_SR_TXELT_OFST ) //!< TXELT mask

#define MML_UART_SR_RFU2_OFST					16 //!< RFU2 offset
#define MML_UART_SR_RFU2_MASK_NOOFST			0xffff //!< RFU2 mask no offset
#define MML_UART_SR_RFU2_MASK					( MML_UART_SR_RFU2_MASK_NOOFST << MML_UART_SR_RFU2_OFST ) //!< RFU2 mask

#define MML_UART_SR_TX_BUSY						1 //!< UART is transmitting data
#define MML_UART_SR_RX_BUSY						1 //!< UART is receiving data
#define MML_UART_SR_RX_EMPTY					1 //!< RX FIFO is empty
#define MML_UART_SR_RX_FULL						1 //!< RX FIFO is full
#define MML_UART_SR_TX_EMPTY					1 //!< TX FIFO is empty
#define MML_UART_SR_TX_FULL						1 //!< TX FIFO is full

#define MML_UART_STATUS_TX_BUSY					( MML_UART_SR_TX_BUSY << MML_UART_SR_TXBUSY_OFST ) /** UART is transmitting data */
#define MML_UART_STATUS_RX_BUSY					( MML_UART_SR_RX_BUSY << MML_UART_SR_RXBUSY_OFST ) /** UART is receiving data */
#define MML_UART_STATUS_RX_EMPTY				( MML_UART_SR_RX_EMPTY << MML_UART_SR_RXEMPTY_OFST ) /** RX FIFO is empty */
#define MML_UART_STATUS_RX_FULL					( MML_UART_SR_RX_FULL << MML_UART_SR_RXFULL_OFST ) /** RX FIFO is full */
#define MML_UART_STATUS_TX_EMPTY				( MML_UART_SR_TX_EMPTY << MML_UART_SR_TXEMPTY_OFST ) /** TX FIFO is empty */
#define MML_UART_STATUS_TX_FULL					( MML_UART_SR_TX_FULL << MML_UART_SR_TXFULL_OFST ) /** TX FIFO is full */
#define MML_UART_DATA_RX_PARITY					( 0x01 << MML_UART_SR_RXELT_OFST ) /** parity error occurred during the byte reception */

#define	MML_UART_BRR_DIV_FACTOR					128
#define	MML_UART_RCR_DDIV_FACTOR				128

#define MML_UART_BAUD_RATE_MASK_IDIV			0x00000fff
#define MML_UART_BAUD_RATE_MASK_DDIV			0x0000007f
#define MML_UART_CTL_TRANSFER_SIZE_MASK			( 0x03 << MML_UART_CR_SIZE_OFST )

#define	MML_UARTx_GPIO_CFG_MASK_NOOFST			0xf

#define	MML_UART0_GPIO_CFG_OFFSET				8
#define	MML_UART0_GPIO_CFG_MASK					( MML_UARTx_GPIO_CFG_MASK_NOOFST << MML_UART0_GPIO_CFG_OFFSET )

#define	MML_UART1_GPIO_CFG_OFFSET				12
#define	MML_UART1_GPIO_CFG_MASK					( MML_UARTx_GPIO_CFG_MASK_NOOFST << MML_UART1_GPIO_CFG_OFFSET )

#define	MML_UART2_GPIO_CFG_OFFSET				14
#define	MML_UART2_GPIO_CFG_MASK					( MML_UARTx_GPIO_CFG_MASK_NOOFST << MML_UART2_GPIO_CFG_OFFSET )

#define	MML_UARTx_GPIO_RXD						0x1
#define	MML_UARTx_GPIO_TXD						0x2
#define	MML_UARTx_GPIO_RTS						0x4
#define	MML_UARTx_GPIO_CTS						0x8


/** @} */ /* @defgroup MML_UART_REGS_SR */


/** @defgroup MML_UART_REGS_IER Interrupt Enable Register (IER)
 *
 * @li FRAMIE [0]	- R/W: framing error interrupt flag
 * @li PARIE [1] 	- R/W: parity error interrupt flag
 * @li SIGIE [2]	- R/W: CTS line toggling interrupt flag
 * @li OVERIE [3]	- R/W: overrun on receiving interrupt flag
 * @li FFRXIE [4]	- R/W: RX FIFO threshold interrupt flag
 * @li FFTXOIE [5]	- R/W: TX FIFO last byte interrupt flag
 * @li FFTXHIE [6]	- R/W: TX FIFO half level interrupt flag
 * @li RFU [31:7]	- R: reserved
 *
 * @{
 */
#define MML_UART_IER_OFST						0x00000008 //!< Register offset
#define MML_UART_IER_DFLT						0x00000000 //!< Register default value

/** Bits Fields */
#define MML_UART_IER_FRAMIE_OFST				0 //!< FRAMIE offset
#define MML_UART_IER_FRAMIE_MASK_NOOFST			0x1 //!< FRAMIE mask no offset
#define MML_UART_IER_FRAMIE_MASK				( MML_UART_IER_FRAMIE_MASK_NOOFST << MML_UART_IER_FRAMIE_OFST ) //!< FRAMIE mask

#define MML_UART_IER_PARIE_OFST					1 //!< PARIE offset
#define MML_UART_IER_PARIE_MASK_NOOFST			0x1 //!< PARIE mask no offset
#define MML_UART_IER_PARIE_MASK					( MML_UART_IER_PARIE_MASK_NOOFST << MML_UART_IER_PARIE_OFST ) //!< PARIE mask

#define MML_UART_IER_SIGIE_OFST					2 //!< SIGIE offset
#define MML_UART_IER_SIGIE_MASK_NOOFST			0x1 //!< SIGIE mask no offset
#define MML_UART_IER_SIGIE_MASK					( MML_UART_IER_SIGIE_MASK_NOOFST << MML_UART_IER_SIGIE_OFST ) //!< SIGIE mask

#define MML_UART_IER_OVERIE_OFST				3 //!< OVERIE offset
#define MML_UART_IER_OVERIE_MASK_NOOFST			0x1 //!< OVERIE mask no offset
#define MML_UART_IER_OVERIE_MASK				( MML_UART_IER_OVERIE_MASK_NOOFST << MML_UART_IER_OVERIE_OFST ) //!< OVERIE mask

#define MML_UART_IER_FFRXIE_OFST				4 //!< FFRXIE offset
#define MML_UART_IER_FFRXIE_MASK_NOOFST			0x1 //!< FFRXIE mask no offset
#define MML_UART_IER_FFRXIE_MASK				( MML_UART_IER_FFRXIE_MASK_NOOFST << MML_UART_IER_FFRXIE_OFST ) //!< FFRXIE mask

#define MML_UART_IER_FFTXOIE_OFST				5 //!< FFTXOIE offset
#define MML_UART_IER_FFTXOIE_MASK_NOOFST		0x1 //!< FFTXOIE mask no offset
#define MML_UART_IER_FFTXOIE_MASK				( MML_UART_IER_FFTXOIE_MASK_NOOFST << MML_UART_IER_FFTXOIE_OFST ) //!< FFTXOIE mask

#define MML_UART_IER_FFTXHIE_OFST				6 //!< FFTXHIE offset
#define MML_UART_IER_FFTXHIE_MASK_NOOFST		0x1 //!< FFTXHIE mask no offset
#define MML_UART_IER_FFTXHIE_MASK				( MML_UART_IER_FFTXHIE_MASK_NOOFST << MML_UART_IER_FFTXHIE_OFST ) //!< FFTXHIE mask

#define MML_UART_IER_RFU_OFST					7//!< RFU offset
#define MML_UART_IER_RFU_MASK_NOOFST			0x1ffffff //!< RFU mask no offset
#define MML_UART_IER_RFU_MASK					( MML_UART_IER_RFU_MASK_NOOFST << MML_UART_IER_RFU_OFST ) //!< RFU mask

#define MML_UART_IER_FRAMIE_DISABLE				0 //!< framing error interrupt disable
#define MML_UART_IER_FRAMIE_ENABLE				1 //!< framing error interrupt enable

#define MML_UART_IER_PARIE_DISABLE				1 //!< parity error interrupt disable
#define MML_UART_IER_PARIE_ENABLE				1 //!< parity error interrupt enable

#define MML_UART_IER_SIGIE_DISABlE				0 //!< CTS changed interrupt disable
#define MML_UART_IER_SIGIE_ENABlE				1 //!< CTS changed interrupt enable

#define MML_UART_IER_OVERIE_DISABLE				0 //!< overrun interrupt disable
#define MML_UART_IER_OVERIE_ENABLE				1 //!< overrun interrupt enable

#define MML_UART_IER_FFRXIE_DISABLE				0 //!< RX FIFO threshold interrupt disable
#define MML_UART_IER_FFRXIE_ENABLE				1 //!< RX FIFO threshold interrupt enable

#define MML_UART_IER_FFTXOIE_DISABLE			0 //!< TX FIFO last byte interrupt disable
#define MML_UART_IER_FFTXOIE_ENABLE				1 //!< TX FIFO last byte interrupt enable

#define MML_UART_IER_FFTXHIE_DISABLE			0 //!< TX FIFO half filled interrupt disable
#define MML_UART_IER_FFTXHIE_ENABLE				1 //!< TX FIFO half filled interrupt enable

/** @} */ /* @defgroup MML_UART_REGS_IER */


/** @defgroup MML_UART_REGS_ISR Interrupt Status Register (ISR)
 *
 * @li FRAMIS [0]	- R/C: framing error interrupt flag
 * @li PARIS [1] 	- R/C: parity error interrupt flag
 * @li SIGIS [2]	- R/C: CTS line toggling interrupt flag
 * @li OVERIS [3]	- R/C: overrun on receiving interrupt flag
 * @li FFRXIS [4]	- R/C: RX FIFO threshold interrupt flag
 * @li FFTXOIS [5]	- R/C: TX FIFO last byte interrupt flag
 * @li FFTXHIS [6]	- R/C: TX FIFO half level interrupt flag
 * @li RFU [31:7]	- R: reserved
 *
 * @{
 */
#define MML_UART_ISR_OFST						0x0000000c //!< Register offset
#define MML_UART_ISR_DFLT						0x00000000 //!< Register default value

/** Bits Fields */
#define MML_UART_ISR_FRAMIS_OFST				0 //!< FRAMIS offset
#define MML_UART_ISR_FRAMIS_MASK_NOOFST			0x1 //!< FRAMIS mask no offset
#define MML_UART_ISR_FRAMIS_MASK				( MML_UART_ISR_FRAMIS_MASK_NOOFST << MML_UART_ISR_FRAMIS_OFST ) //!< FRAMIS mask

#define MML_UART_ISR_PARIS_OFST					1 //!< PARIS offset
#define MML_UART_ISR_PARIS_MASK_NOOFST			0x1 //!< PARIS mask no offset
#define MML_UART_ISR_PARIS_MASK					( MML_UART_ISR_PARIS_MASK_NOOFST << MML_UART_ISR_PARIS_OFST ) //!< PARIS mask

#define MML_UART_ISR_SIGIS_OFST					2 //!< SIGIS offset
#define MML_UART_ISR_SIGIS_MASK_NOOFST			0x1 //!< SIGIS mask no offset
#define MML_UART_ISR_SIGIS_MASK					( MML_UART_ISR_SIGIS_MASK_NOOFST << MML_UART_ISR_SIGIS_OFST ) //!< SIGIS mask

#define MML_UART_ISR_OVERIS_OFST				3 //!< OVERIS offset
#define MML_UART_ISR_OVERIS_MASK_NOOFST			0x1 //!< OVERIS mask no offset
#define MML_UART_ISR_OVERIS_MASK				( MML_UART_ISR_OVERIS_MASK_NOOFST << MML_UART_ISR_OVERIS_OFST ) //!< OVERIS mask

#define MML_UART_ISR_FFRXIS_OFST				4 //!< FFRXIS offset
#define MML_UART_ISR_FFRXIS_MASK_NOOFST			0x1 //!< FFRXIS mask no offset
#define MML_UART_ISR_FFRXIS_MASK				( MML_UART_ISR_FFRXIS_MASK_NOOFST << MML_UART_ISR_FFRXIS_OFST ) //!< FFRXIS mask

#define MML_UART_ISR_FFTXOIS_OFST				5 //!< FFTXOIS offset
#define MML_UART_ISR_FFTXOIS_MASK_NOOFST		0x1 //!< FFTXOIS mask no offset
#define MML_UART_ISR_FFTXOIS_MASK				( MML_UART_ISR_FFTXOIS_MASK_NOOFST << MML_UART_ISR_FFTXOIS_OFST ) //!< FFTXOIS mask

#define MML_UART_ISR_FFTXHIS_OFST				6 //!< FFTXHIS offset
#define MML_UART_ISR_FFTXHIS_MASK_NOOFST		0x1 //!< FFTXHIS mask no offset
#define MML_UART_ISR_FFTXHIS_MASK				( MML_UART_ISR_FFTXHIS_MASK_NOOFST << MML_UART_ISR_FFTXHIS_OFST ) //!< FFTXHIS mask

#define MML_UART_ISR_RFU_OFST					7 //!< RFU offset
#define MML_UART_ISR_RFU_MASK_NOOFST			0x1ffffff //!< RFU mask no offset
#define MML_UART_ISR_RFU_MASK					( MML_UART_ISR_RFU_MASK_NOOFST << MML_UART_ISR_RFU_OFST ) //!< RFU mask

#define MML_UART_ISR_FRAMIS_ERROR				1 //!< framing error
#define MML_UART_ISR_PARIS_ERROR				1 //!< parity error
#define MML_UART_ISR_SIGIS_TOGGLING				1 //!< CTS changed
#define MML_UART_ISR_OVERIS_ERROR				1 //!< OVERIS error
#define MML_UART_ISR_FFRXIS_THRESHOLD			1 //!< RX FIFO threshold reached
#define MML_UART_ISR_FFTXOIS_ONE				1 //!< TX FIFO last byte
#define MML_UART_ISR_FFTXHIS_HALF				1 //!< TX FIFO half filled

/** @} */ /* @defgroup MML_UART_REGS_ISR */


/** @defgroup MML_UART_REGS_BRR0 Baud Rate Register 0 (BRR0)
 *
 * @li IDIV [11:0]	- R/W: integer divider
 * @li RFU [31:12]	- R: reserved
 *
 * @{
 */
#define MML_UART_BRR_OFST						0x00000010 //!< Register offset
#define MML_UART_BRR_DFLT						0x00000000 //!< Register default value

/** Bits Fields */
#define MML_UART_BRR_IDIV_OFST					0 //!< IDIV offset
#define MML_UART_BRR_IDIV_MASK_NOOFST			0xfff //!< IDIV mask no offset
#define MML_UART_BRR_IDIV_MASK					( MML_UART_BRR_IDIV_MASK_NOOFST << MML_UART_BRR_IDIV_OFST ) //!< IDIV mask

#define MML_UART_BRR_RFU_OFST					12 //!< RFU offset
#define MML_UART_BRR_RFU_MASK_NOOFST			0xfffff //!< RFU mask no offset
#define MML_UART_BRR_RFU_MASK					( MML_UART_BRR_RFU_MASK_NOOFST << MML_UART_BRR_RFU_OFST ) //!< RFU mask

/** @} */ /* @defgroup MML_UART_REGS_BRR0 */


/** @defgroup MML_UART_REGS_BBR1 Baud Rate Register 1 (BRR1)
 *
 * @li DDIV [6:0]	- R/W: decimal divider
 * @li RFU [31:7]	- R: reserved
 *
 * @{
 */
#define MML_UART_BRR1_OFST						0x00000014 //!< Register offset
#define MML_UART_BRR1_DFLT						0x00000000 //!< Register default value

/** Bits Fields */
#define MML_UART_BRR1_IDIV_OFST					0 //!< DDIV offset
#define MML_UART_BRR1_IDIV_MASK_NOOFST			0x7f //!< DDIV mask no offset
#define MML_UART_BRR1_IDIV_MASK					( MML_UART_BRR1_IDIV_MASK_NOOFST << MML_UART_BRR1_IDIV_OFST ) //!< DDIV mask

#define MML_UART_BRR1_RFU_OFST					7 //!< RFU offset
#define MML_UART_BRR1_RFU_MASK_NOOFST			0x1ffffff //!< RFU mask no offset
#define MML_UART_BRR1_RFU_MASK					( MML_UART_BRR1_RFU_MASK_NOOFST << MML_UART_BRR1_RFU_OFST ) //!< RFU mask

/** @} */ /* @defgroup MML_UART_REGS_BRR1 */


/** @defgroup MML_UART_REGS_TXR Transmit Register (TXR)
 *
 * @li DATA [7:0]	- R/W: TX data read out (for debug purpose)
 * @li RFU [31:8]	- R: reserved
 *
 * @{
 */
#define MML_UART_TXR_OFST						0x18 //!< Register offset
#define MML_UART_TXR_DFLT						0x0 //!< Register default value

/** Bits Fields */
#define MML_UART_TXR_DATA_OFST					0 //!< DATA offset
#define MML_UART_TXR_DATA_MASK_NOOFST			0xff //!< DATA mask no offset
#define MML_UART_TXR_DATA_MASK					( MML_UART_TXR_DATA_MASK_NOOFST << MML_UART_TXR_DATA_OFST ) //!< DATA mask

#define MML_UART_TXR_RFU_OFST					8 //!< RFU offset
#define MML_UART_TXR_RFU_MASK_NOOFST			0xffffff //!< RFU mask no offset
#define MML_UART_TXR_RFU_MASK					( MML_UART_TXR_RFU_MASK_NOOFST << MML_UART_TXR_RFU_OFST ) //!< RFU mask

/** @} */ /* @defgroup MML_UART_REGS_TXR */


/** @defgroup MML_UART_REGS_PR Pins Register (PR)
 *
 * @li CTS [0]		- R: CTS pin status
 * @li CTS [1]		- R/W: RTS pin control
 * @li RFU [31:2]	- R: reserved
 *
 * @{
 */
#define MML_UART_PR_OFST						0x1c //!< Register offset
#define MML_UART_PR_DFLT						0x3 //!< Register default value

/** Bits Fields */
#define MML_UART_PR_CTS_OFST					0 //!< CTS offset
#define MML_UART_PR_CTS_MASK_NOOFST				0x1 //!< CTS mask no offset
#define MML_UART_PR_CTS_MASK					( MML_UART_PR_CTS_MASK_NOOFST << MML_UART_PR_CTS_OFST ) //!< CTS mask

#define MML_UART_PR_RTS_OFST					1 //!< RTS offset
#define MML_UART_PR_RTS_MASK_NOOFST				0x1 //!< RTS mask no offset
#define MML_UART_PR_RTS_MASK					( MML_UART_PR_RTS_MASK_NOOFST << MML_UART_PR_RTS_OFST ) //!< RTS mask

#define MML_UART_PR_RFU_OFST					2 //!< RFU offset
#define MML_UART_PR_RFU_MASK_NOOFST				0x3fffffff //!< RFU mask no offset
#define MML_UART_PR_RFU_MASK					( MML_UART_PR_RFU_MASK_NOOFST << MML_UART_PR_RFU_OFST ) //!< RFU mask

/** @} */ /* @defgroup MML_UART_REGS_PR */


/** @defgroup MML_UART_REGS_DR Data Register (DR)
 *
 * @li DATA [7:0]	- R/W: RX/TX data
 * @li PARITY [8]	- R/W: RX data parity
 * @li RFU [31:9]	- R: reserved
 *
 * @{
 */
#define MML_UART_DR_OFST						0x20 //!< Register offset
#define MML_UART_DR_DFLT						0x0 //!< Register default value

/** Bits Fields */
#define MML_UART_DR_DATA_OFST					0 //!< DATA offset
#define MML_UART_DR_DATA_MASK_NOOFST			0xff //!< DATA mask no offset
#define MML_UART_DR_DATA_MASK					( MML_UART_DR_DATA_MASK_NOOFST << MML_UART_DR_DATA_OFST ) //!< DATA mask

#define MML_UART_DR_PARITY_OFST					8 //!< PARITY offset
#define MML_UART_DR_PARITY_MASK_NOOFST			0x1 //!< PARITY mask no offset
#define MML_UART_DR_PARITY_MASK					( MML_UART_DR_PARITY_MASK_NOOFST << MML_UART_DR_PARITY_OFST ) //!< PARITY mask

#define MML_UART_DR_RFU_OFST					2 //!< RFU offset
#define MML_UART_DR_RFU_MASK_NOOFST				0x7fffff//!< RFU mask no offset
#define MML_UART_DR_RFU_MASK					( MML_UART_DR_RFU_MASK_NOOFST << MML_UART_DR_RFU_OFST )//!< RFU mask

/** @} */ /* @defgroup MML_UART_REGS_DR */

/** @defgroup MML_UART_REGS_DMA DMA Register (DMA)
 *
 * @li TXCNT [3:0]	- R/W: TX threshold
 * @li TXEN  [4]        - R/W: TX channel enable
 * @li RXCNT [8:5]	- R/W: RX threshold
 * @li RXEN  [9]        - R/W: RX channel enable
 * @li RFU   [31:10]	- R: reserved
 *
 * @{
 */

#define MML_UART_DMA_OFST						0x30 //!< Register offset
#define MML_UART_DMA_DFLT						0x0 //!< Register default value

/* Bits Fields */

#define MML_UART_DMA_TXCNT_OFST					0 //!< TXCNT offset
#define MML_UART_DMA_TXCNT_MASK_NOOFST			0xf //!< TXCNT mask no offset
#define MML_UART_DMA_TXCNT_MASK					( MML_UART_DMA_TXCNT_MASK_NOOFST << MML_UART_DMA_TXCNT_OFST ) //!< TXCNT mask


#define MML_UART_DMA_TXEN_OFST					4 //!< TXEN offset
#define MML_UART_DMA_TXEN_MASK_NOOFST			0x1 //!< TXEN mask no offset
#define MML_UART_DMA_TXEN_MASK					( MML_UART_DMA_TXEN_MASK_NOOFST << MML_UART_DMA_TXEN_OFST ) //!< TXEN mask

#define MML_UART_DMA_RXCNT_OFST					5 //!< RXCNT offset
#define MML_UART_DMA_RXCNT_MASK_NOOFST			0xf //!< RXCNT mask no offset
#define MML_UART_DMA_RXCNT_MASK					( MML_UART_DMA_RXCNT_MASK_NOOFST << MML_UART_DMA_RXCNT_OFST ) //!< RXCNT mask

#define MML_UART_DMA_RXEN_OFST					9	//!< RXEN offset
#define MML_UART_DMA_RXEN_MASK_NOOFST			0x1 //!< RXEN mask no offset
#define MML_UART_DMA_RXEN_MASK					( MML_UART_DMA_RXEN_MASK_NOOFST << MML_UART_DMA_RXEN_OFST ) //!< RXEN mask

#define MML_UART_DMA_RFU_MASK					0xfffffc00 //!< RFU mask

/** @} */ /* @defgroup MML_UART_REGS_DMA */


#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */
/** UART Registers.
 *
 */
typedef volatile struct
{
	 /** Control register */
	volatile unsigned int						cr;
	/** Status register */
	volatile unsigned int						sr;
	/** Interrupt enable register */
	volatile unsigned int						ier;
	/** Interrupt status register */
	volatile unsigned int						isr;
	/** Baudrate register 0 */
	volatile unsigned int						brr;
	/** Baudrate register 1 */
	volatile unsigned int						rcr;
	/** TX register */
	volatile unsigned int						txr;
	/** Pin register */
	volatile unsigned int						pnr;
	/** Data register */
	volatile unsigned int						dr;
	/**  */
	volatile unsigned int						rfu[3];
	/** DMA */
	volatile unsigned int						dma;

} mml_uart_regs_t;

#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_UART_REGS */

#endif /* _MML_UART_REGS_H_ */

/******************************************************************************/
/* EOF */
