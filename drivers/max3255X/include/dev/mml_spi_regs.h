/*
 * mml_spi_regs.h --
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
 * Created on: Oct 21, 2013
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_SPI_REGS_H_
#define _MML_SPI_REGS_H_

/** @file mml_spi_regs.h SPI Registers Header */

/** @defgroup MML_SPI SPI Driver */

/** @defgroup MML_SPI_REGS SPI Registers
 *
 * @note SPI IP or Specification version number
 *
 * @ingroup MML_SPI
 * @{
 */
/** @defgroup MML_SPI_REGS_DR Data Register (DR)
 *
 * @li DATA [15:0]	- R/W: data
 * @li RFU [31:16]	- R: reserved
 * @{
 */
#define MML_SPI_DATA_OFST						0x00000000 //!< Register offset
#define MML_SPI_DATA_DFLT						0x00000000 //!< Register default value
/* Bits Fields */
#define MML_SPI_DATA_DATA_OFST					0 //!< DATA offset
#define MML_SPI_DATA_DATA_MASK_NOOFST			0xffff //!< DATA mask no offset
#define MML_SPI_DATA_DATA_MASK					( MML_SPI_DATA_DATA_MASK_NOOFST << MML_SPI_DATA_DATA_OFST ) //!< DATA mask
#define MML_SPI_DATA_RFU_MASK					~MML_SPI_DATA_DATA_MASK //!< RFU mask
/** @} */ /* @defgroup MML_SPI_REGS_DR */

/** @defgroup MML_SPI_REGS_CR Control Register (CR)
 *
 * @li SPIEN [0]	- R/W: SPI enable
 * @li MMEN [1]		- R/W: SPI master mode enable
 * @li WOR [2]		- R/W: wire-OR
 * @li CLKPOL [3]	- R/W: clock polarity
 * @li PHASE [4]	- R/W: phase select
 * @li BIRQ [5]		- R/W: BRG timer interrupt request
 * @li STR [6]		- R/W: start an SPI interrupt request
 * @li IRQE [7]		- R/W: interrupt request enable
 * @li RFU [31:8]	- R: reserved
 *
 * @{
 */
#define MML_SPI_CR_OFST							0x00000004 //!< Register offset
#define MML_SPI_CR_DFLT							0x00000000 //!< Register default value
/* Bits Fields */
#define MML_SPI_CR_SPIEN_OFST					0 //!< SPIEN offset
#define MML_SPI_CR_SPIEN_MASK_NOOFST			0x1 //!< SPIEN mask no offset
#define MML_SPI_CR_SPIEN_MASK					( MML_SPI_CR_SPIEN_MASK_NOOFST << MML_SPI_CR_SPIEN_OFST ) //!< SPIEN mask

#define MML_SPI_CR_MMEN_OFST					1 //!< MMEN offset
#define MML_SPI_CR_MMEN_MASK_NOOFST				0x1 //!< MMEN mask
#define MML_SPI_CR_MMEN_MASK					( MML_SPI_CR_MMEN_MASK_NOOFST << MML_SPI_CR_MMEN_OFST ) //!< MMEN mask

#define MML_SPI_CR_WOR_OFST						2 //!< WOR offset
#define MML_SPI_CR_WOR_MASK_NOOFST				0x1 //!< WOR mask no offset
#define MML_SPI_CR_WOR_MASK						( MML_SPI_CR_WOR_MASK_NOOFST << MML_SPI_CR_WOR_OFST ) //!< WOR mask

#define MML_SPI_CR_CLKPOL_OFST					3 //!< CLKPOL offset
#define MML_SPI_CR_CLKPOL_MASK_NOOFST			0x1 //!< CLKPOL mask no offset
#define MML_SPI_CR_CLKPOL_MASK					( MML_SPI_CR_CLKPOL_MASK_NOOFST << MML_SPI_CR_CLKPOL_OFST ) //!< CLKPOL mask

#define MML_SPI_CR_PHASE_OFST					4 //!< PHASE offset
#define MML_SPI_CR_PHASE_MASK_NOOFST			0x1 //!< PHASE mask no offset
#define MML_SPI_CR_PHASE_MASK					( MML_SPI_CR_PHASE_MASK_NOOFST << MML_SPI_CR_PHASE_OFST ) //!< PHASE mask

#define MML_SPI_CR_BIRQ_OFST					5 //!< BIRQ offset
#define MML_SPI_CR_BIRQ_MASK_NOOFST				0x1 //!< BIRQ mask no offset
#define MML_SPI_CR_BIRQ_MASK					( MML_SPI_CR_BIRQ_MASK_NOOFST << MML_SPI_CR_BIRQ_OFST ) //!< BIRQ mask

#define MML_SPI_CR_STR_OFST						6 //!< STR offset
#define MML_SPI_CR_STR_MASK_NOOFST				0x1 //!< STR mask no offset
#define MML_SPI_CR_STR_MASK						( MML_SPI_CR_STR_MASK_NOOFST << MML_SPI_CR_STR_OFST ) //!< STR mask

#define MML_SPI_CR_IRQE_OFST					7 //!< IRQE offset
#define MML_SPI_CR_IRQE_MASK_NOOFST				0x1 //!< IRQE mask no offset
#define MML_SPI_CR_IRQE_MASK					( MML_SPI_CR_IRQE_MASK_NOOFST << MML_SPI_CR_IRQE_OFST ) //!< IRQE mask

#define MML_SPI_CR_RFU_OFST						8 //!< RFU offset
#define MML_SPI_CR_RFU_MASK						0xffffff00 //!< RFU mask

/* Enumeration of the different value for a bits field . We recommend to use
 * human readable value. */

#define MML_SPI_CR_SPIEN_DISABLE				0 //!< SPI disabled
#define MML_SPI_CR_SPIEN_ENABLE					1 //!< SPI enabled

#define MML_SPI_CR_MODE_SLAVE					0 //!< Slave mode
#define MML_SPI_CR_MODE_MASTER					1 //!< Master mode

#define MML_SPI_IO_OPEN_DRAIN_DISABLE			0 //!< SPI signals not configured
#define MML_SPI_IO_OPEN_DRAIN_ENABLE			1 //!< SPI signals configured

#define MML_SPI_CLOCK_POL_LOW					0 //!< polarity low
#define MML_SPI_CLOCK_POL_HIGH					1 //!< polarity high

#define MML_SPI_CLOCK_PHA_LOW					0 //!< sets the phase of the data to the clock
#define MML_SPI_CLOCK_PHA_HIGH					1

#define MML_SPI_CR_BIRQ_DISABLE					0 //!< SPIEN ='0' disable the baud rate generation else nothing
#define MML_SPI_CR_BIRQ_ENABLE					1 //!< SPIEN ='0' enable the baud rate generation else nothing

#define MML_SPI_CR_STR_0						0
#define MML_SPI_CR_STR_1						1 //!< Sets IRQ='1'in the SPI_STAT

#define MML_SPI_CR_IRQE_DISABLE					0 //!< interrupts disabled
#define MML_SPI_CR_IRQE_ENABLE					1 //!< interrupts enable



/** @} */ /* @defgroup MML_SPI_REGS_CR */

/** @defgroup MML_SPI_REGS_SR Status Register (SR)
 *
 * @li SLAS [0]		- R: slave select
 * @li TXST [1]		- R: transmit status
 * @li TUND [2]		- R/W1C: transmit underrun
 * @li ROVR [3]		- R/W1C: receive overrun
 * @li ABT [4]		- R/W1C: slave mode transaction abort
 * @li COL [5]		- R/W1C: collision
 * @li TOVR [6]		- R/W1C: transmit overrun
 * @li IRQ [7]		- R/W1C: interrupt request
 * @li RFU [31:8]	- R/W1C: reserved
 *
 * @{
 */
#define MML_SPI_SR_OFST							0x00000008 //!< Register offset
#define MML_SPI_SR_DFLT							0x00000001 //!< Register default value

/* Bits Fields */
#define MML_SPI_SR_SLAS_OFST					0 //!< SLAS offset
#define MML_SPI_SR_SLAS_MASK_NOOFST				0x1 //!< SLAS mask no offset
#define MML_SPI_SR_SLAS_MASK					( MML_SPI_SR_SLAS_MASK_NOOFST << MML_SPI_SR_SLAS_OFST ) //!< SLAS mask

#define MML_SPI_SR_TXST_OFST					1 //!< TXST offset
#define MML_SPI_SR_TXST_MASK_NOOFST				0x1 //!< TXST mask
#define MML_SPI_SR_TXST_MASK					( MML_SPI_SR_TXST_MASK_NOOFST << MML_SPI_SR_TXST_OFST ) //!< TXST mask

#define MML_SPI_SR_TUND_OFST					2 //!< TUND offset
#define MML_SPI_SR_TUND_MASK_NOOFST				0x1 //!< TUND mask no offset
#define MML_SPI_SR_TUND_MASK					( MML_SPI_SR_TUND_MASK_NOOFST << MML_SPI_SR_TUND_OFST ) //!< TUND mask

#define MML_SPI_SR_ROVR_OFST					3 //!< ROVR offset
#define MML_SPI_SR_ROVR_MASK_NOOFST				0x1 //!< ROVR mask no offset
#define MML_SPI_SR_ROVR_MASK					( MML_SPI_SR_ROVR_MASK_NOOFST << MML_SPI_SR_ROVR_OFST ) //!< ROVR mask

#define MML_SPI_SR_ABT_OFST						4 //!< ABT offset
#define MML_SPI_SR_ABT_MASK_NOOFST				0x1 //!< ABT mask no offset
#define MML_SPI_SR_ABT_MASK						( MML_SPI_SR_ABT_MASK_NOOFST << MML_SPI_SR_ABT_OFST ) //!< ABT mask

#define MML_SPI_SR_COL_OFST						5 //!< COL offset
#define MML_SPI_SR_COL_MASK_NOOFST				0x1 //!< COL mask
#define MML_SPI_SR_COL_MASK						( MML_SPI_SR_COL_MASK_NOOFST << MML_SPI_SR_COL_OFST ) //!< COL mask

#define MML_SPI_SR_TOVR_OFST					6 //!< TOVR offset
#define MML_SPI_SR_TOVR_MASK_NOOFST				0x1 //!< TOVR mask no offset
#define MML_SPI_SR_TOVR_MASK					( MML_SPI_SR_TOVR_MASK_NOOFST << MML_SPI_SR_TOVR_OFST ) //!< TOVR mask

#define MML_SPI_SR_IRQ_OFST						7 //!< IRQ offset
#define MML_SPI_SR_IRQ_MASK_NOOFST				0x1 //!< IRQ mask no offset
#define MML_SPI_SR_IRQ_MASK						( MML_SPI_SR_IRQ_MASK_NOOFST << MML_SPI_SR_IRQ_OFST ) //!< IRQ mask

#define MML_SPI_SR_RFU_OFST						8 //!< RFU offset

/* Enumeration of the different value for a bits field . We recommend to use
 * human readable value. */

#define MML_SPI_CR_SLAS_0						0 //!< Slave selected (SPI=Slave)
#define MML_SPI_CR_SLAS_1						1 //!< Slave not selected (SPI=Slave)

#define MML_SPI_CR_TXST_0						0 //!< No data transmission
#define MML_SPI_CR_TXST_1						1 //!< data transmission

#define MML_SPI_CR_TUND_0						0 //!< error has not occurred
#define MML_SPI_CR_TUND_1						1 //!< error has occurred

#define MML_SPI_CR_ROVR_0						0 //!< error has not occurred
#define MML_SPI_CR_ROVR_1						1 //!< error has occurred

#define MML_SPI_CR_ABT_0						0 //!< Slave mode not detected
#define MML_SPI_CR_ABT_1						1 //!< Slave mode detected

#define MML_SPI_CR_COL_0						0 //!< fault mode not detected
#define MML_SPI_CR_COL_1						1 //!< fault mode detected

#define MML_SPI_CR_TOVR_0						0 //!< error has not occurred
#define MML_SPI_CR_TOVR_1						1 //!< error has occurred

#define MML_SPI_CR_IRQ_0						0 //!< SPI interrupt not pending
#define MML_SPI_CR_IRQ_1						1 //!< SPI interrupt pending
/** @} */ /* @defgroup MML_SPI_REGS_SR */

/** @defgroup MML_SPI_REGS_MR MODE Register (MR)
 *
 * @li SSV [0]		- R/W: slave select value
 * @li SSIO [1]		- R/W: slave select I/O
 * @li NUMBITS [5:2]- R/W: number of Data Bits per Character to Transfer
 * @li DIAG [6]		- R/W: Diagnostic Mode Control
 * @li TX_LJ [7]	- R/W: Used when NUMBITS !=0
 * @li RFU [31:8]	- R: reserved
 *
 * @{
 */
#define MML_SPI_MR_OFST							0x0000000c //!< Register offset
#define MML_SPI_MR_DFLT							0x00000000 //!< Register default value

/* Bits Fields */
#define MML_SPI_MR_SSV_OFST						0 //!< SSV offset
#define MML_SPI_MR_SSV_MASK_NOOFST				0x1 //!< SSV mask no offset
#define MML_SPI_MR_SSV_MASK						( MML_SPI_MR_SSV_MASK_NOOFST << MML_SPI_MR_SSV_OFST ) //!< SSV mask

#define MML_SPI_MR_SSIO_OFST					1 //!< SSIO offset
#define MML_SPI_MR_SSIO_MASK_NOOFST				0x1 //!< SSIO mask no offset
#define MML_SPI_MR_SSIO_MASK					( MML_SPI_MR_SSIO_MASK_NOOFST << MML_SPI_MR_SSIO_OFST ) //!< SSIO mask

#define MML_SPI_MR_NUMBITS_OFST					2 //!< NUMBITS offset
#define MML_SPI_MR_NUMBITS_MASK_NOOFST			0xf //!< NUMBITS mask no offset
#define MML_SPI_MR_NUMBITS_MASK					( MML_SPI_MR_NUMBITS_MASK_NOOFST << MML_SPI_MR_NUMBITS_OFST ) //!< NUMBITS mask

#define MML_SPI_MR_DIAG_OFST					6 //!< DIAG offset
#define MML_SPI_MR_DIAG_MASK_NOOFST				0x1 //!< DIAG mask no offset
#define MML_SPI_MR_DIAG_MASK					( MML_SPI_MR_DIAG_MASK_NOOFST << MML_SPI_MR_DIAG_OFST ) //!< DIAG mask

#define MML_SPI_MR_TX_LJ_OFST					7 //!< TX_LJ offset
#define MML_SPI_MR_TX_LJ_MASK_NOOFST			0x1 //!< TX_LJ mask no offset
#define MML_SPI_MR_TX_LJ_MASK					( MML_SPI_MR_TX_LJ_MASK_NOOFST << MML_SPI_MR_TX_LJ_OFST ) //!< TX_LJ mask

#define MML_SPI_MR_SSL1_OFST					8 //!< SSL1 offset
#define MML_SPI_MR_SSL1_MASK_NOOFST				0x1 //!< SSL1 mask no offset
#define MML_SPI_MR_SSL1_MASK					( MML_SPI_MR_SSL1_MASK_NOOFST << MML_SPI_MR_SSL1_OFST ) //!< SSL1 mask

#define MML_SPI_MR_SSL2_OFST					9 //!< SSL2 offset
#define MML_SPI_MR_SSL2_MASK_NOOFST				0x1 //!< SSL2 mask no offset
#define MML_SPI_MR_SSL2_MASK					( MML_SPI_MR_SSL2_MASK_NOOFST << MML_SPI_MR_SSL2_OFST ) //!< SSL2 mask

#define MML_SPI_MR_SSL3_OFST					10 //!< SSL3 offset
#define MML_SPI_MR_SSL3_MASK_NOOFST				0x1 //!< SSL3 mask no offset
#define MML_SPI_MR_SSL3_MASK					( MML_SPI_MR_SSL3_MASK_NOOFST << MML_SPI_MR_SSL3_OFST ) //!< SSL3 mask

#define MML_SPI_MR_SSLx_MASK					( MML_SPI_MR_SSL1_MASK |\
													MML_SPI_MR_SSL2_MASK |\
													MML_SPI_MR_SSL3_MASK )

#define MML_SPI_MR_RFU_OFST						11 //!< RFU offset

/* Enumeration of the different value for a bits field . We recommend to use
 * human readable value. */
#define MML_SPI_MR_NUMBITS_2					0b000 //!< the field contains the number of bits to shift for each character transfer
#define MML_SPI_MR_NUMBITS_3					0b001
#define MML_SPI_MR_NUMBITS_4					0b010
#define MML_SPI_MR_NUMBITS_5					0b011

/* Enumeration of the different value for a bits field . We recommend to use
 * human readable value. */

#define MML_SPI_MR_SSV_0						0 //!< SSV='0'(asserts nSS low)
#define MML_SPI_MR_SSV_1						1 //!< SSV='1'(deasserts nSS low)

#define MML_SPI_MR_SSIO_0						0 //!< nSS = input
#define MML_SPI_MR_SSIO_1						1 //!< nSS = output

#define MML_SPI_MR_DIAG_0						0 //!< return SPI_BRG
#define MML_SPI_MR_DIAG_1						1 //!< return SPI Baud Rate Counter

#define MML_SPI_MR_TX_LJ_0						0 //!< transmit data loaded from fifo
#define MML_SPI_MR_TX_LJ_1						1 //!< transmit data is left justified as it is loaded
/** @} */ /* @defgroup MML_SPI_REGS_MR */

/** @defgroup MML_SPI_REGS_DSR Diagnostic State Register (DSR)
 *
 * @li SPISTATE [5:0]	- R: SPI state machine
 * @li TCKEN [6]		- R: transmit clock enable
 * @li SCKEN [7]		- R: shift clock enable
 * @li RFU [31:8]		- R: reserved
 * @{
 */
#define MML_SPI_DSR_OFST						0x00000010 //!< Register offset
#define MML_SPI_DSR_DFLT						0x00000000 //!< Register default value

/* Bits Fields */
#define MML_SPI_DSR_SPISTATE_OFST				0 //!< SPISTATE offset
#define MML_SPI_DSR_SPISTATE_MASK_NO_OFST		0x0000003f //!< SPISTATE mask no offset
#define MML_SPI_DSR_SPISTATE_MASK				( MML_SPI_DSR_SPISTATE_MASK_NO_OFST << MML_SPI_DSR_SPISTATE_OFST ) //!< SPISTATE mask

#define MML_SPI_DSR_TCKEN_OFST					6 //!< TCKEN offset
#define MML_SPI_DSR_TCKEN_MASK_NOOFST			0x1 //!< TCKEN mask no offset
#define MML_SPI_DSR_TCKEN_MASK					( MML_SPI_DSR_TCKEN_MASK_NOOFST << MML_SPI_DSR_TCKEN_OFST ) //!< TCKEN mask

#define MML_SPI_DSR_SCKEN_OFST					7 //!< SCKEN offset
#define MML_SPI_DSR_SCKEN_MASK_NOOFST			0x1 //!< SCKEN mask no offset
#define MML_SPI_DSR_SCKEN_MASK					( MML_SPI_DSR_SCKEN_MASK_NOOFST << MML_SPI_DSR_SCKEN_OFST ) //!< SCKEN mask

#define MML_SPI_DSR_RFU_OFST					8 //!< RFU offset

/* Enumeration of the different value for a bits field . We recommend to use
 * human readable value. */

#define MML_SPI_DSR_SPISTATE_0					0 //!< define the current state

#define MML_SPI_DSR_TCKEN_0						0 //!< transmit clock enable de-asserted
#define MML_SPI_DSR_TCKEN_1						1 //!< transmit clock enable asserted

#define MML_SPI_DSR_SCKEN_0						0 //!< shift clock enable de-asserted
#define MML_SPI_DSR_SCKEN_1						1 //!< shift clock enable-asserted
/** @} */ /* @defgroup MML_SPI_REGS_DSR */

/** @defgroup MML_SPI_REGS_BRR Baud Rate Register (BRR)
 *
 * @li BRG [15:0]	- R/W: baud rate reload value
 * @li RFU [31:16]	- R: reserved
 * @{
 */
#define MML_SPI_BRR_OFST						0x00000014 //!< Register offset
#define MML_SPI_BRR_DFLT						0x0000ffff //!< Register default value

/* Bits Fields */
#define MML_SPI_BRR_BRG_OFST					0 //!< BRG offset
#define MML_SPI_BRR_BRG_MASK_NOOFST				0xffff //!< BRG mask no offset
#define MML_SPI_BRR_BRG_MASK					( MML_SPI_BRR_BRG_MASK_NOOFST << MML_SPI_BRR_BRG_OFST ) //!< BRG mask

#define MML_SPI_BRR_RFU_OFST					16 //!< RFU offset
/** @} */ /* @defgroup MML_SPI_REGS_BRR */

/** @defgroup MML_SPI_REGS_DMAR DMA register (DMAR)
 *
 * @li TX_FIFO_LEVEL [1:0]	- R/W: put the bit request threhold
 * @li RFU1 [3:2]			- R: reserved
 * @li TX_FIFO_CLR [4]		- W/O: reset the bit
 * @li RFU2 [7:5]			- R: reserved
 * @li TX_FIFO_CNT [10:8]	- R: number of entries
 * @li RFU3 [14:11]			- R: reserved
 * @li TX_DMA_EN [15]		- R/W: used to enable/disable the bit
 * @li RX_FIFO_LEVEL [17:16]- R/W: put the bit request threshold
 * @li RFU4 [19:18]			- R: reserved
 * @li RX_FIFO_CLR [20]		- W/O: reset the bit
 * @li RFU5 [23:21]			- R: reserved
 * @li RX_FIFO_CNT [26:24]	- R: number of entries
 * @li RFU6 [30:27]			- R: reserved
 * @li RX_DMA_EN [31]		- R/W: Enables DMA for receive
 * @{
 */
#define MML_SPI_DMAR_OFST						0x00000018 //!< Register offset
#define MML_SPI_DMAR_DFLT						0x00030003 //!< Register default value

/* Bits Fields */
#define MML_SPI_DMAR_TX_FIFO_LVL_OFST			0 //!< TX_FIFO_LEVEL offset
#define MML_SPI_DMAR_TX_FIFO_LVL_MASK_NOOFST	0x3 //!< TX_FIFO_LEVEL mask no offset
#define MML_SPI_DMAR_TX_FIFO_LVL_MASK			( MML_SPI_DMAR_TX_FIFO_LVL_MASK_NOOFST << MML_SPI_DMAR_TX_FIFO_LVL_OFST ) //!< TX_FIFO_LEVEL mask

#define MML_SPI_DMAR_RFU1_OFST					2 //!< RFU1 offset

#define MML_SPI_DMAR_TX_FIFO_CLR_OFST			4 //!< TX_FIFO_CLR offset
#define MML_SPI_DMAR_TX_FIFO_CLR_MASK_NOOFST	0x1 //!< TX_FIFO_CLR mask no offset
#define MML_SPI_DMAR_TX_FIFO_CLR_MASK			( MML_SPI_DMAR_TX_FIFO_CLR_MASK_NOOFST << MML_SPI_DMAR_TX_FIFO_CLR_OFST ) //!< TX_FIFO_CLR mask

#define MML_SPI_DMAR_RFU2_OFST					5 //!< RFU2 offset

#define MML_SPI_DMAR_TX_FIFO_CNT_OFST			8 //!< TX_FIFO_CNT offset
#define MML_SPI_DMAR_TX_FIFO_CNT_MASK_NOOFST	0x7 //!< TX_FIFO_CNT mask no offset
#define MML_SPI_DMAR_TX_FIFO_CNT_MASK			( MML_SPI_DMAR_TX_FIFO_CNT_MASK_NOOFST << MML_SPI_DMAR_TX_FIFO_CNT_OFST ) //!< TX_FIFO_CNT mask

#define MML_SPI_DMAR_RFU3_OFST					11 //!< RFU3 offset

#define MML_SPI_DMAR_TX_DMA_EN_OFST				15 //!< TX_DMA_EN offset
#define MML_SPI_DMAR_TX_DMA_EN_MASK_NOOFST		0x1 //!< TX_DMA_EN mask no offset
#define MML_SPI_DMAR_TX_DMA_EN_MASK				( MML_SPI_DMAR_TX_DMA_EN_MASK_NOOFST << MML_SPI_DMAR_TX_DMA_EN_OFST ) //!< TX_DMA_EN mask

#define MML_SPI_DMAR_RX_FIFO_LVL_OFST			16 //!< RX_FIFO_LEVEL offset
#define MML_SPI_DMAR_RX_FIFO_LVL_MASK_NOOFST	0x3 //!< RX_FIFO_LEVEL mask no offset
#define MML_SPI_DMAR_RX_FIFO_LVL_MASK			( MML_SPI_DMAR_RX_FIFO_LVL_MASK_NOOFST << MML_SPI_DMAR_RX_FIFO_LVL_OFST ) //!< RX_FIFO_LEVEL mask

#define MML_SPI_DMAR_RFU4_OFST					18 //!< RFU4 offset

#define MML_SPI_DMAR_RX_FIFO_CLR_OFST			20 //!< RX_FIFO_CLR offset
#define MML_SPI_DMAR_RX_FIFO_CLR_MASK_NOOFST	0x1 //!< RX_FIFO_CLR mask no offset
#define MML_SPI_DMAR_RX_FIFO_CLR_MASK			( MML_SPI_DMAR_RX_FIFO_CLR_MASK_NOOFST << MML_SPI_DMAR_RX_FIFO_CLR_OFST ) //!< RX_FIFO_CLR mask

#define MML_SPI_DMAR_RFU5_OFST					21 //!< RFU5 offset

#define MML_SPI_DMAR_RX_FIFO_CNT_OFST			24 //!< RX_FIFO_CNT offset
#define MML_SPI_DMAR_RX_FIFO_CNT_MASK_NOOFST	0x7 //!< RX_FIFO_CNT mask no offset
#define MML_SPI_DMAR_RX_FIFO_CNT_MASK			( MML_SPI_DMAR_RX_FIFO_CNT_MASK_NOOFST << MML_SPI_DMAR_RX_FIFO_CNT_OFST ) //!< RX_FIFO_CNT mask

#define MML_SPI_DMAR_RFU6_OFST					27 //!< RFU6 offset

#define MML_SPI_DMAR_RX_DMA_EN_OFST				31 //!< RX_DMA_EN offset
#define MML_SPI_DMAR_RX_DMA_EN_MASK_NOOFST		0x1u //!< RX_DMA_EN mask no offset
#define MML_SPI_DMAR_RX_DMA_EN_MASK				( MML_SPI_DMAR_RX_DMA_EN_MASK_NOOFST << MML_SPI_DMAR_RX_DMA_EN_OFST ) //!< RX_DMA_EN mask

/* Enumeration of the different value for a bits field . We recommend to use
 * human readable value. */

#define MML_SPI_DMAR_TX_FIFO_LVL_0				0b00 //!< 1 free entry
#define MML_SPI_DMAR_TX_FIFO_LVL_1				0b01 //!< 2 free entries
#define MML_SPI_DMAR_TX_FIFO_LVL_2				0b10 //!< 3 free entries
#define MML_SPI_DMAR_TX_FIFO_LVL_3				0b11 //!< 4 free entries

#define MML_SPI_DMAR_TX_FIFO_CNT_0				0b000 //!< TxFIFO has 0 entries
#define MML_SPI_DMAR_TX_FIFO_CNT_1				0b001 //!< TxFIFO has 1 entry
#define MML_SPI_DMAR_TX_FIFO_CNT_2				0b010 //!< TxFIFO has 2 entries
#define MML_SPI_DMAR_TX_FIFO_CNT_3				0b011 //!< TxFIFO has 3 entries
#define MML_SPI_DMAR_TX_FIFO_CNT_4				0b100 //!< TxFIFO has 4 entries

#define MML_SPI_DMAR_RX_FIFO_LVL_0				0b00 //!< request RxDMA when RxFIFO has 1 entry
#define MML_SPI_DMAR_RX_FIFO_LVL_1				0b01 //!< request RxDMA when RxFIFO has 2 entries
#define MML_SPI_DMAR_RX_FIFO_LVL_2				0b10 //!< request RxDMA when RxFIFO has 3 entries
#define MML_SPI_DMAR_RX_FIFO_LVL_3				0b11 //!< request RxDMA when RxFIFO has 4 entries

#define MML_SPI_DMAR_RX_FIFO_CNT_0				0b000 //!< RxFIFO has 0 entries
#define MML_SPI_DMAR_RX_FIFO_CNT_1				0b001 //!< RxFIFO has 1 entry
#define MML_SPI_DMAR_RX_FIFO_CNT_2				0b010 //!< RxFIFO has 2 entries
#define MML_SPI_DMAR_RX_FIFO_CNT_3				0b011 //!< RxFIFO has 3 entries
#define MML_SPI_DMAR_RX_FIFO_CNT_4				0b100 //!< RxFIFO has 4 entries

#define MML_SPI_DMAR_TX_FIFO_CLR_0				0 //!< Nothing
#define MML_SPI_DMAR_TX_FIFO_CLR_1				1 //!< Reset
#define MML_SPI_DMAR_TX_DMA_EN_DISABLE			0 //!< TX DMA disable
#define MML_SPI_DMAR_TX_DMA_EN_ENABLE			1 //!< TX DMA enable
#define MML_SPI_DMAR_RX_FIFO_CLR_0				0 //!< Nothing
#define MML_SPI_DMAR_RX_FIFO_CLR_1				1 //!< Reset
#define MML_SPI_DMAR_RX_DMA_EN_DISABLE			0 //!< RX DMA disable
#define MML_SPI_DMAR_RX_DMA_EN_ENABLE			1 //!< RX DMA enable
/** @} */ /* @defgroup MML_SPI_REGS_DMAR */

/** @defgroup MML_SPI_REGS_MCR Mode Control Register (MCR)
 *
 * @li I2S_EN [0]	- R/W: I2S mode enable
 * @li I2S_MUTE [1]	- R/W: I2S mute transmit
 * @li I2S_PAUSE [2]- R/W: I2S pause transmit/receive
 * @li I2S_MONO [3]	- R/W: I2S mono audio mode
 * @li I2S_LJ [4]	- R/W: I2S left justify
 * @li RFU [31:5]	- R: reserved
 * @{
 */
#define MML_SPI_MCR_OFST						0x0000001c //!< Register offset
#define MML_SPI_MCR_DFLT						0x00000000 //!< Register default value

/* Bits Fields */
#define MML_SPI_MCR_I2S_EN_OFST					0 //!< I2S_EN offset
#define MML_SPI_MCR_I2S_EN_MASK_NOOFST			0x1 //!< I2S_EN mask no offset
#define MML_SPI_MCR_I2S_EN_MASK					( MML_SPI_MCR_I2S_EN_MASK_NOOFST << MML_SPI_MCR_I2S_EN_OFST ) //!< I2S_EN mask

#define MML_SPI_MCR_I2S_MUTE_OFST				1 //!< I2S_MUTE offset
#define MML_SPI_MCR_I2S_MUTE_MASK_NOOFST		0x1 //!< I2S_MUTE mask no offset
#define MML_SPI_MCR_I2S_MUTE_MASK				( MML_SPI_MCR_I2S_MUTE_MASK_NOOFST << MML_SPI_MCR_I2S_MUTE_OFST ) //!< I2S_MUTE mask

#define MML_SPI_MCR_I2S_PAUSE_OFST				2 //!< I2S_PAUSE offset
#define MML_SPI_MCR_I2S_PAUSE_MASK_NOOFST		0x1 //!< I2S_PAUSE mask no offset
#define MML_SPI_MCR_I2S_PAUSE_MASK				( MML_SPI_MCR_I2S_PAUSE_MASK_NOOFST << MML_SPI_MCR_I2S_PAUSE_OFST ) //!< I2S_PAUSE mask

#define MML_SPI_MCR_I2S_MONO_OFST				3 //!< I2S_MONO offset
#define MML_SPI_MCR_I2S_MONO_MASK_NOOFST		0x1 //!< I2S_MONO mask no offset
#define MML_SPI_MCR_I2S_MONO_MASK				( MML_SPI_MCR_I2S_MONO_MASK_NOOFST << MML_SPI_MCR_I2S_MONO_OFST ) //!< I2S_MONO mask

#define MML_SPI_MCR_I2S_LJ_OFST					4 //!< I2S_LJ offset
#define MML_SPI_MCR_I2S_LJ_MASK_NOOFST			0x1 //!< I2S_LJ mask no offset
#define MML_SPI_MCR_I2S_LJ_MASK					( MML_SPI_MCR_I2S_LJ_MASK_NOOFST << MML_SPI_MCR_I2S_LJ_OFST ) //!< I2S_LJ mask

#define MML_SPI_MCR_RFU_OFST					5 //!< RFU offset


/* Enumeration of the different value for a bits field . We recommend to use
 * human readable value. */

#define MML_SPI_MCR_I2S_EN_DISABLE				0 //!<disable mode
#define MML_SPI_MCR_I2S_EN_ENABLE				1 //!<enable mode

#define MML_SPI_MCR_I2S_MUTE_0					0 //!<normal transmit/receive
#define MML_SPI_MCR_I2S_MUTE_1					1 //!<transmit '0'

#define MML_SPI_MCR_I2S_PAUSEL_0				0 //!<normal transmit/receive
#define MML_SPI_MCR_I2S_PAUSEL_1				1 //!<halt transmit and receive FIFO&DMA

#define MML_SPI_MCR_I2S_MONO_0					0 //!<stereo mode
#define MML_SPI_MCR_I2S_MONO_1					1 //!<mono mode

#define MML_SPI_MCR_I2S_LJ_0					0 //!<normal I2S audio protocol
#define MML_SPI_MCR_I2S_LJ_1					1 //!<audio data is in sync with nSS

/** @} */ /* @defgroup MML_SPI_REGS_MCR */


#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */
/** SPI Registers.
 *
 */
typedef volatile struct
{
	/** Data register */
	unsigned int								dr;
	/** Control register */
	unsigned int								cr;
	/** Status register */
	unsigned int								sr;
	/** Mode register */
	unsigned int								mr;
	/** Diagnostic state register */
	unsigned int								dsr;
	/** Baud rate register */
	unsigned int								brr;
	/** Dma register */
	unsigned int								dmar;
	/** I2S control register */
	unsigned int								i2s_cr;

} mml_spi_regs_t;
#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_SPI_REGS */

#endif /* _MML_SPI_REGS_H_ */

/******************************************************************************/
/* EOF */
