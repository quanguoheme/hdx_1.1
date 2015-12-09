/*
 * mml_sc_regs.h --
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
 * Created on: Jun 29, 2012
 * Author: Jeremy B. <jeremy.brodt@maxim-ic.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_SC_REGS_H_
#define _MML_SC_REGS_H_

/** @file mml_sc_regs.h SC Registers Header */

/** @defgroup MML_SC SC Driver */

/** @defgroup MML_SC_REGS SC Registers
 *
 * @note SC IP or Specification version number
 *
 * @ingroup MML_SC
 * @{
 */

/** @defgroup MML_SC_REGS_CR SC Control Register Register
 *
 * @li SCxCR [0] - R/W: CONV
 * @li SCxCR [1] - R/W: CREP
 * @li SCxCR [2] - R/W: WTEN
 * @li SCxCR [3] - R/W: UART
 * @li SCxCR [4] - R/W: CCEN
 * @li SCxCR [5] - R/W: RXFLUSH
 * @li SCxCR [6] - R/W: TXFLUSH
 * @li SCxCR [7] - R: RFU
 * @li SCxCR [11:8] - R/W: RXTHD
 * @li SCxCR [15:12] - R/W: TXTHD
 * @li SCxCR [16] - R/W: START
 * @li SCxCR [17] - R/W: BYPASS_PHY
 * @li SCxCR [18] - R/W: PRPOL
 * @li SCxCR [19] - R/W: DEBNCE
 * @li SCxCR [20] - R/W: BYPASS_SEQ
 * @li SCxCR [31:21] - R: RFU
 *
 * @{
 */
#define MML_SC_CR_OFST						0x0	//!< Control Register offset
#define MML_SC_CR_DFLT						0x00000000	//!< Control Register default value
/* Bits Fields */
#define MML_SC_CR_CONV_OFST				0	//!< CONV offset
#define MML_SC_CR_CONV_MASK_NOOFST			0x1	//!< CONV mask
#define MML_SC_CR_CONV_MASK				( MML_SC_CR_CONV_MASK_NOOFST << MML_SC_CR_CONV_OFST )

#define MML_SC_CR_CREP_OFST				1 //!< CREP offset
#define MML_SC_CR_CREP_MASK_NOOFST			0x1 //!< CREP mask
#define MML_SC_CR_CREP_MASK				( MML_SC_CR_CREP_MASK_NOOFST << MML_SC_CR_CREP_OFST )

#define MML_SC_CR_WTEN_OFST				2 //!< WTEN offset
#define MML_SC_CR_WTEN_MASK_NOOFST			0x1 //!< WTEN mask
#define MML_SC_CR_WTEN_MASK				( MML_SC_CR_WTEN_MASK_NOOFST << MML_SC_CR_WTEN_OFST )

#define MML_SC_CR_UART_OFST				3 //!< UART offset
#define MML_SC_CR_UART_MASK_NOOFST			0x1 //!< UART mask
#define MML_SC_CR_UART_MASK				( MML_SC_CR_UART_MASK_NOOFST << MML_SC_CR_UART_OFST )

#define MML_SC_CR_CCEN_OFST				4 //!< CCEN offset
#define MML_SC_CR_CCEN_MASK_NOOFST			0x1 //!< CCEN mask
#define MML_SC_CR_CCEN_MASK				( MML_SC_CR_CCEN_MASK_NOOFST << MML_SC_CR_CCEN_OFST )

#define MML_SC_CR_RXFLUSH_OFST				5 //!< RXFLUSH offset
#define MML_SC_CR_RXFLUSH_MASK_NOOFST		0x1 //!< RXFLUSH mask
#define MML_SC_CR_RXFLUSH_MASK				( MML_SC_CR_RXFLUSH_MASK_NOOFST << MML_SC_CR_RXFLUSH_OFST )

#define MML_SC_CR_TXFLUSH_OFST				6 //!< TXFLUSH offset
#define MML_SC_CR_TXFLUSH_MASK_NOOFST		0x1 //!< TXFLUSH mask
#define MML_SC_CR_TXFLUSH_MASK				( MML_SC_CR_TXFLUSH_MASK_NOOFST << MML_SC_CR_TXFLUSH_OFST )

#define MML_SC_CR_RXTHD_OFST				8 //!< RXTHD offset
#define MML_SC_CR_RXTHD_MASK_NOOFST		0x4 //!< RXTHD mask
#define MML_SC_CR_RXTHD_MASK				( MML_SC_CR_RXTHD_MASK_NOOFST << MML_SC_CR_RXTHD_OFST )

#define MML_SC_CR_TXTHD_OFST				12 //!< TXTHD offset
#define MML_SC_CR_TXTHD_MASK_NOOFST		0x4 //!< TXTHD mask
#define MML_SC_CR_TXTHD_MASK				( MML_SC_CR_TXTHD_MASK_NOOFST << MML_SC_CR_TXTHD_OFST )
#define MML_SC_CR_START_OFST				16 //!< START offset
#define MML_SC_CR_START_MASK_NOOFST		0x1 //!< START mask
#define MML_SC_CR_START_MASK				(MML_SC_CR_START_MASK_NOOFST << MML_SC_CR_START_OFST)

#define MML_SC_CR_BYPASS_PHY_OFST		       	17 //!< BYPASS_PHY offset
#define MML_SC_CR_BYPASS_PHY_MASK_NOOFST	0x1 //!< BYPASS_PHY mask
#define MML_SC_CR_BYPASS_PHY_MASK				(MML_SC_CR_BYPASS_PHY_MASK_NOOFST << MML_SC_CR_BYPASS_PHY_OFST)

#define MML_SC_CR_PRPOL_OFST				18 //!< PRPOL offset
#define MML_SC_CR_PRPOL_MASK_NOOFST		0x1 //!< PRPOL mask
#define MML_SC_CR_PRPOL_MASK				(MML_SC_CR_PRPOL_MASK_NOOFST << MML_SC_CR_PRPOL_OFST)

#define MML_SC_CR_DEBNCE_OFST				19 //!< DEBNCE offset
#define MML_SC_CR_DEBNCE_MASK_NOOFST		0x1 //!< DEBNCE mask
#define MML_SC_CR_DEBNCE_MASK				(MML_SC_CR_DEBNCE_MASK_NOOFST << MML_SC_CR_DEBNCE_OFST)

#define MML_SC_CR_BYPASS_SEQ_OFST			20 //!< BYPASS_SEQ offset
#define MML_SC_CR_BYPASS_SEQ_MASK_NOOFST	0x1 //!< BYPASS_SEQ mask
#define MML_SC_CR_BYPASS_SEQ_MASK				(MML_SC_CR_BYPASS_SEQ_MASK_NOOFST << MML_SC_CR_BYPASS_SEQ_OFST)
/** @} */ /* @defgroup MML_SC_REGS_CR */

/** @defgroup MML_SC_REGS_SR SC Status Register
 *
 * @li SCxSR [0] - R/C: PAR
 * @li SCxSR [1] - R: WTOV
 * @li SCxSR [2] - R: CCOV
 * @li SCxSR [3] - R/C: TXCF
 * @li SCxSR [4] - R: RXEMPTY
 * @li SCxSR [5] - R: RXFULL
 * @li SCxSR [6] - R: TXEMPTY
 * @li SCxSR [7] - R: TXFULL
 * @li SCxSR [11:8] - R: RXELT
 * @li SCxSR [15:12] - R: TXELT
 * @li SCxSR [16] - R: PRES
 * @li SCxSR [17] - R: RFU
 * @li SCxSR [18] - R: PRC
 * @li SCxSR [19] - R: PDL
 * @li SCxSR [20] - R: ACTIV
 * @li SCxSR [31:21] - R: RFU
 *
 * @{
 */
#define MML_SC_SR_OFST						0x4	//!< Status Register offset
#define MML_SC_SR_DFLT						0x00000050	//!< Status Register default value
/* Bits Fields */
#define MML_SC_SR_PAR_OFST					0 //!< PAR offset
#define MML_SC_SR_PAR_MASK_NOOFST			0x1	//!< PAR mask
#define MML_SC_SR_PAR_MASK					( MML_SC_SR_PAR_MASK_NOOFST << MML_SC_SR_PAR_OFST )

#define MML_SC_SR_WTOV_OFST				1 //!< WTOV offset
#define MML_SC_SR_WTOV_MASK_NOOFST			0x1 //!< WTOV mask
#define MML_SC_SR_WTOV_MASK				( MML_SC_SR_WTOV_MASK_NOOFST << MML_SC_SR_WTOV_OFST )

#define MML_SC_SR_CCOV_OFST				2 //!< CCOV offset
#define MML_SC_SR_CCOV_MASK_NOOFST			0x1 //!< CCOV mask
#define MML_SC_SR_CCOV_MASK				( MML_SC_SR_CCOV_MASK_NOOFST << MML_SC_SR_CCOV_OFST )

#define MML_SC_SR_TXCF_OFST				3 //!< TXCF offset
#define MML_SC_SR_TXCF_MASK_NOOFST			0x1 //!< TXCF mask
#define MML_SC_SR_TXCF_MASK				( MML_SC_SR_TXCF_MASK_NOOFST << MML_SC_SR_TXCF_OFST )

#define MML_SC_SR_RXEMPTY_OFST				4 //!< RXEMPTY offset
#define MML_SC_SR_RXEMPTY_MASK_NOOFST		0x1 //!< RXEMPTY mask
#define MML_SC_SR_RXEMPTY_MASK				( MML_SC_SR_RXEMPTY_MASK_NOOFST << MML_SC_SR_RXEMPTY_OFST )

#define MML_SC_SR_RXFULL_OFST				5 //!< RXFULL offset
#define MML_SC_SR_RXFULL_MASK_NOOFST		0x1 //!< RXFULL mask
#define MML_SC_SR_RXFULL_MASK				( MML_SC_SR_RXFULL_MASK_NOOFST << MML_SC_SR_RXFULL_OFST )

#define MML_SC_SR_TXEMPTY_OFST				6 //!< TXEMPTY offset
#define MML_SC_SR_TXEMPTY_MASK_NOOFST		0x1 //!< TXEMPTY mask
#define MML_SC_SR_TXEMPTY_MASK				( MML_SC_SR_TXEMPTY_MASK_NOOFST << MML_SC_SR_TXEMPTY_OFST )

#define MML_SC_SR_TXFULL_OFST				7 //!< TXFULL offset
#define MML_SC_SR_TXFULL_MASK_NOOFST		0x1 //!< TXFULL mask
#define MML_SC_SR_TXFULL_MASK				( MML_SC_SR_TXFULL_MASK_NOOFST << MML_SC_SR_TXFULL_OFST )

#define MML_SC_SR_RXELT_OFST				8 //!< RXELT offset
#define MML_SC_SR_RXELT_MASK_NOOFST		0x4 //!< RXELT mask
#define MML_SC_SR_RXELT_MASK				( MML_SC_SR_RXELT_MASK_NOOFST << MML_SC_SR_RXELT_OFST )

#define MML_SC_SR_TXELT_OFST				12 //!< TXELT offset
#define MML_SC_SR_TXELT_MASK_NOOFST		0x4 //!< TXELT mask
#define MML_SC_SR_TXELT_MASK				( MML_SC_SR_TXELT_MASK_NOOFST << MML_SC_SR_TXELT_OFST )

#define MML_SC_SR_PRES_OFST				16 //!< PRES offset
#define MML_SC_SR_PRES_MASK_NOOFST		0x1 //!< PRES mask
#define MML_SC_SR_PRES_MASK				(MML_SC_SR_PRES_MASK_NOOFST << MML_SC_SR_PRES_OFST)

#define MML_SC_SR_PRC_OFST				18 //!< PRC offset
#define MML_SC_SR_PRC_MASK_NOOFST		0x1 //!< PRC mask
#define MML_SC_SR_PRC_MASK				(MML_SC_SR_PRC_MASK_NOOFST << MML_SC_SR_PRC_OFST)

#define MML_SC_SR_PDL_OFST				19 //!< PDL offset
#define MML_SC_SR_PDL_MASK_NOOFST		0x1 //!< PDL mask
#define MML_SC_SR_PDL_MASK				(MML_SC_SR_PDL_MASK_NOOFST << MML_SC_SR_PDL_OFST)

#define MML_SC_SR_ACTIV_OFST				19 //!< ACTIV offset
#define MML_SC_SR_ACTIV_MASK_NOOFST		0x1 //!< ACTIV mask
#define MML_SC_SR_ACTIV_MASK				(MML_SC_SR_ACTIV_MASK_NOOFST << MML_SC_SR_ACTIV_OFST)
/** @} */ /* @defgroup MML_SC_REGS_SR */

/** @defgroup MML_SC_REGS_PNR SC Pin Register
 *
 * @li SCxPNR [0] - R/W: CRDRST
 * @li SCxPNR [1] - R/W: CRDCLK
 * @li SCxPNR [2] - R/W: CRDIO
 * @li SCxPNR [3] - R/W: CRDC4
 * @li SCxPNR [4] - R/W: CRDC8
 * @li SCxPNR [5] - R/W: CLKSEL
 * @li SCxPNR [7:6] - R: RFU
 * @li SCxPNR [9:8] - R/W: VCCSEL
 * @li SCxPNR [15:10] - R: RFU
 * @li SCxPNR [16] - R/W: IO_EN
 * @li SCxPNR [17] - R/W: CLK_EN
 * @li SCxPNR [18] - R/W: RST_EN
 * @li SCxPNR [19] - R/W: VCC_EN
 * @li SCxPNR [31:20] - R: RFU
 *
 * @{
 */
#define MML_SC_PNR_OFST					0x8	//!< Pin Register offset
#define MML_SC_PNR_DFLT					0x00000000	//!< Pin Register default value
/* Bits Fields */
#define MML_SC_PNR_CRDRST_OFST				0 //!< CRDRST offset
#define MML_SC_PNR_CRDRST_MASK_NOOFST		0x1	//!< CRDRST mask
#define MML_SC_PNR_CRDRST_MASK				( MML_SC_PNR_CRDRST_MASK_NOOFST << MML_SC_PNR_CRDRST_OFST )

#define MML_SC_PNR_CRDCLK_OFST				1 //!< CRDCLK offset
#define MML_SC_PNR_CRDCLK_MASK_NOOFST		0x1 //!< CRDCLK mask
#define MML_SC_PNR_CRDCLK_MASK				( MML_SC_PNR_CRDCLK_MASK_NOOFST << MML_SC_PNR_CRDCLK_OFST )

#define MML_SC_PNR_CRDIO_OFST				2 //!< CRDIO offset
#define MML_SC_PNR_CRDIO_MASK_NOOFST		0x1 //!< CRDIO mask
#define MML_SC_PNR_CRDIO_MASK				( MML_SC_PNR_CRDIO_MASK_NOOFST << MML_SC_PNR_CRDIO_OFST )

#define MML_SC_PNR_CRDC4_OFST				3 //!< CRDC4 offset
#define MML_SC_PNR_CRDC4_MASK_NOOFST		0x1 //!< CRDC4 mask
#define MML_SC_PNR_CRDC4_MASK				( MML_SC_PNR_CRDC4_MASK_NOOFST << MML_SC_PNR_CRDC4_OFST )

#define MML_SC_PNR_CRDC8_OFST				4 //!< CRDC8 offset
#define MML_SC_PNR_CRDC8_MASK_NOOFST		0x1 //!< CRDC8 mask
#define MML_SC_PNR_CRDC8_MASK				( MML_SC_PNR_CRDC8_MASK_NOOFST << MML_SC_PNR_CRDC8_OFST )

#define MML_SC_PNR_CLKSEL_OFST				5 //!< CLKSEL offset
#define MML_SC_PNR_CLKSEL_MASK_NOOFST		0x1 //!< CLKSEL mask
#define MML_SC_PNR_CLKSEL_MASK				( MML_SC_PNR_CLKSEL_MASK_NOOFST << MML_SC_PNR_CLKSEL_OFST )

#define MML_SC_PNR_VCCSEL_OFST			8 //!< VCCSEL offset
#define MML_SC_PNR_VCCSEL_MASK_NOOFST	0x2 //!< VCCSEL mask
#define MML_SC_PNR_VCCSEL_MASK			(MML_SC_PNR_VCCSEL_MASK_NOOFST << MML_SC_PNR_VCCSEL_OFST)

#define MML_SC_PNR_IO_C48_EN_OFST		16 //!< IO_C48_EN offset
#define MML_SC_PNR_IO_C48_EN_MASK_NOOFST	0x1 //!< IO_C48_EN mask
#define MML_SC_PNR_IO_C48_EN_MASK			(MML_SC_PNR_IO_C48_EN_MASK_NOOFST << MML_SC_PNR_IO_C48_EN_OFST)

#define MML_SC_PNR_CLK_EN_OFST			17 //!< CLK_EN offset
#define MML_SC_PNR_CLK_EN_MASK_NOOFST	0x1 //!< CLK_EN mask
#define MML_SC_PNR_CLK_EN_MASK			(MML_SC_PNR_CLK_EN_MASK_NOOFST << MML_SC_PNR_CLK_EN_OFST)

#define MML_SC_PNR_RST_EN_OFST			18 //!< RST_EN offset
#define MML_SC_PNR_RST_EN_MASK_NOOFST	0x1 //!< RST_EN mask
#define MML_SC_PNR_RST_EN_MASK			(MML_SC_PNR_RST_EN_MASK_NOOFST << MML_SC_PNR_RST_EN_OFST)

#define MML_SC_PNR_VCC_EN_OFST			19 //!< VCC_EN offset
#define MML_SC_PNR_VCC_EN_MASK_NOOFST	0x1 //!< VCC_EN mask
#define MML_SC_PNR_VCC_EN_MASK			(MML_SC_PNR_VCC_EN_MASK_NOOFST << MML_SC_PNR_VCC_EN_OFST)
/** @} */ /* @defgroup MML_SC_REGS_PNR */

/** @defgroup MML_SC_REGS_ETUR SC ETU Register
 *
 * @li SCxETUR [14:0] - R/W: ETU
 * @li SCxETUR [15] - R/W: COMP
 * @li SCxETUR [16] - R/W: HALF
 * @li SCxETUR [31:17] - R: RFU
 *
 * @{
 */
#define MML_SC_ETUR_OFST					0xc	//!< ETU Register offset
#define MML_SC_ETUR_DFLT					0x00000000	//!< ETU Register default value
/* Bits Fields */
#define MML_SC_ETUR_ETU_OFST				0 //!< ETU offset
#define MML_SC_ETUR_ETU_MASK_NOOFST		0x7fff //!< ETU mask
#define MML_SC_ETUR_ETU_MASK				( MML_SC_ETUR_ETU_MASK_NOOFST << MML_SC_ETUR_ETU_OFST )

#define MML_SC_ETUR_COMP_OFST				15 //!< COMP offset
#define MML_SC_ETUR_COMP_MASK_NOOFST		0x1 //!< COMP mask
#define MML_SC_ETUR_COMP_MASK				( MML_SC_ETUR_COMP_MASK_NOOFST << MML_SC_ETUR_COMP_OFST )

#define MML_SC_ETUR_HALF_OFST				16 //!< HALF offset
#define MML_SC_ETUR_HALF_MASK_NOOFST		0x1 //!< HALF mask
#define MML_SC_ETUR_HALF_MASK				( MML_SC_ETUR_HALF_MASK_NOOFST << MML_SC_ETUR_HALF_OFST )
/** @} */ /* @defgroup MML_SC_REGS_ETUR */

/** @defgroup MML_SC_REGS_GTR SC Guard Time Register
 *
 * @li SCxGTR [15:0] - R/W: GT
 * @li SCxGTR [31:16] - R: RFU
 *
 * @{
 */
#define MML_SC_GTR_OFST					0x10 //!< GT Register offset
#define MML_SC_GTR_DFLT					0x00000000 //!< GT Register default value
/* Bits Fields */
#define MML_SC_GTR_GT_OFST					0 //!< GT offset
#define MML_SC_GTR_GT_MASK_NOOFST			0xffff //!< GT mask
#define MML_SC_GTR_GT_MASK					( MML_SC_GTR_GT_MASK_NOOFST << MML_SC_GTR_GT_OFST )
/** @} */ /* @defgroup MML_SC_REGS_ETUR */

/** @defgroup MML_SC_REGS_WT0R SC Waiting Time 0 Register
 *
 * @li SCxWT0R [31:0] - R/W: WT
 *
 * @{
 */
#define MML_SC_WT0R_OFST					0x14 //!< WT0 Register offset
#define MML_SC_WT0R_DFLT					0x00000000 //!< WT0 Register default value
/* Bits Fields */
#define MML_SC_WT0R_WT_OFST				0 //!< WT offset
#define MML_SC_WT0R_WT_MASK_NOOFST			0xffffffff //!< WT mask
#define MML_SC_WT0R_WT_MASK				( MML_SC_WT0R_WT_MASK_NOOFST << MML_SC_WT0R_WT_OFST )
/** @} */ /* @defgroup MML_SC_REGS_WT0R */

/** @defgroup MML_SC_REGS_WT0R SC Waiting Time 1 Register
 *
 * @li SCxWT1R [7:0] - R/W: WT
 * @li SCxWT1R [31:8] - R: RFU
 *
 * @{
 */
#define MML_SC_WT1R_OFST					0x18 //!< WT1 Register offset
#define MML_SC_WT1R_DFLT					0x00000000 //!< WT1 Register default value
/* Bits Fields */
#define MML_SC_WT1R_WT_OFST				0 //!< WT offset
#define MML_SC_WT1R_WT_MASK_NOOFST			0xff //!< WT mask
#define MML_SC_WT1R_WT_MASK				( MML_SC_WT1R_WT_MASK_NOOFST << MML_SC_WT1R_WT_OFST )
/** @} */ /* @defgroup MML_SC_REGS_WT1R */

/** @defgroup MML_SC_REGS_IER SC Interrupt Enable Register
 *
 * @li SCxIER [0] - R/W: PARIE
 * @li SCxIER [1] - R/W: WTIE
 * @li SCxIER [2] - R/W: CTIE
 * @li SCxIER [3] - R/W: TCIE
 * @li SCxIER [4] - R/W: RXEIE
 * @li SCxIER [5] - R/W: RXTIE
 * @li SCxIER [6] - R/W: RXFIE
 * @li SCxIER [7] - R/W: TXEIE
 * @li SCxIER [8] - R/W: TXTIE
 * @li SCxIER [9] - R/W: PRCIE
 * @li SCxIER [10] - R/W: PDLIE
 * @li SCxIER [31:11] - R: RFU
 *
 * @{
 */
#define MML_SC_IER_OFST					0x1c //!< Interrupt Enable Register offset
#define MML_SC_IER_DFLT					0x00000000 //!< Interrupt Enable Register default value
/* Bits Fields */
#define MML_SC_IER_PARIE_OFST				0 //!< PARIE offset
#define MML_SC_IER_PARIE_MASK_NOOFST		0x1 //!< PARIE mask
#define MML_SC_IER_PARIE_MASK				( MML_SC_IER_PARIE_MASK_NOOFST << MML_SC_IER_PARIE_OFST )

#define MML_SC_IER_WTIE_OFST				1 //!< WTIE offset
#define MML_SC_IER_WTIE_MASK_NOOFST		0x1 //!< WTIE mask
#define MML_SC_IER_WTIE_MASK				( MML_SC_IER_WTIE_MASK_NOOFST << MML_SC_IER_WTIE_OFST )

#define MML_SC_IER_CTIE_OFST				2 //!< CTIE offset
#define MML_SC_IER_CTIE_MASK_NOOFST		0x1 //!< CTIE mask
#define MML_SC_IER_CTIE_MASK				( MML_SC_IER_CTIE_MASK_NOOFST << MML_SC_IER_CTIE_OFST )

#define MML_SC_IER_TCIE_OFST				3 //!< TCIE offset
#define MML_SC_IER_TCIE_MASK_NOOFST		0x1 //!< TCIE mask
#define MML_SC_IER_TCIE_MASK				( MML_SC_IER_TCIE_MASK_NOOFST << MML_SC_IER_TCIE_OFST )

#define MML_SC_IER_RXEIE_OFST				4 //!< RXEIE offset
#define MML_SC_IER_RXEIE_MASK_NOOFST		0x1 //!< RXEIE mask
#define MML_SC_IER_RXEIE_MASK				( MML_SC_IER_RXEIE_MASK_NOOFST << MML_SC_IER_RXEIE_OFST )

#define MML_SC_IER_RXTIE_OFST				5 //!< RXTIE offset
#define MML_SC_IER_RXTIE_MASK_NOOFST		0x1 //!< RXTIE mask
#define MML_SC_IER_RXTIE_MASK				( MML_SC_IER_RXTIE_MASK_NOOFST << MML_SC_IER_RXTIE_OFST )

#define MML_SC_IER_RXFIE_OFST				6 //!< RXFIE offset
#define MML_SC_IER_RXFIE_MASK_NOOFST		0x1 //!< RXFIE mask
#define MML_SC_IER_RXFIE_MASK				( MML_SC_IER_RXFIE_MASK_NOOFST << MML_SC_IER_RXFIE_OFST )

#define MML_SC_IER_TXEIE_OFST				7 //!< TXEIE offset
#define MML_SC_IER_TXEIE_MASK_NOOFST		0x1 //!< TXEIE mask
#define MML_SC_IER_TXEIE_MASK				( MML_SC_IER_TXEIE_MASK_NOOFST << MML_SC_IER_TXEIE_OFST )

#define MML_SC_IER_TXTIE_OFST				8 //!< TXTIE offset
#define MML_SC_IER_TXTIE_MASK_NOOFST		0x1 //!< TXTIE mask
#define MML_SC_IER_TXTIE_MASK				( MML_SC_IER_TXTIE_MASK_NOOFST << MML_SC_IER_TXTIE_OFST )

#define MML_SC_IER_PRCIE_OFST				9 //!< PRCIE offset
#define MML_SC_IER_PRCIE_MASK_NOOFST		0x1 //!< PRCIE mask
#define MML_SC_IER_PRCIE_MASK				(MML_SC_IER_PRCIE_MASK_NOOFST << MML_SC_IER_PRCIE_OFST)

#define MML_SC_IER_PDLIE_OFST				10 //!< PDLIE offset
#define MML_SC_IER_PDLIE_MASK_NOOFST		0x1 //!< PDLIE mask
#define MML_SC_IER_PDLIE_MASK				(MML_SC_IER_PDLIE_MASK_NOOFST << MML_SC_IER_PDLIE_OFST)
/** @} */ /* @defgroup MML_SC_REGS_IER */

/** @defgroup MML_SC_REGS_ISR SC Interrupt Status Register
 *
 * @li SCxISR [0] - R/C: PARIE
 * @li SCxISR [1] - R/C: WTIS
 * @li SCxISR [2] - R/C: CTIS
 * @li SCxISR [3] - R/C: TCIS
 * @li SCxISR [4] - R/C: RXEIS
 * @li SCxISR [5] - R/C: RXTIS
 * @li SCxISR [6] - R/C: RXFIS
 * @li SCxISR [7] - R/C: TXEIS
 * @li SCxISR [8] - R/C: TXTIS
 * @li SCxISR [9] - R/C: PRCIS
 * @li SCxISR [10] - R/C: PDLIS
 * @li SCxISR [31:11] - R: RFU
 *
 * @{
 */
#define MML_SC_ISR_OFST					0x20 //!< Interrupt Status Register offset
#define MML_SC_ISR_DFLT					0x00000000 //!< Interrupt Status Register default value
/* Bits Fields */
#define MML_SC_ISR_PARIS_OFST				0 //!< PARIS offset
#define MML_SC_ISR_PARIS_MASK_NOOFST		0x1 //!< PARIS mask
#define MML_SC_ISR_PARIS_MASK				( MML_SC_ISR_PARIS_MASK_NOOFST << MML_SC_ISR_PARIS_OFST )

#define MML_SC_ISR_WTIS_OFST				1 //!< WTIS offset
#define MML_SC_ISR_WTIS_MASK_NOOFST		0x1 //!< WTIS mask
#define MML_SC_ISR_WTIS_MASK				( MML_SC_ISR_WTIS_MASK_NOOFST << MML_SC_ISR_WTIS_OFST )

#define MML_SC_ISR_CTIS_OFST				2 //!< CTIS offset
#define MML_SC_ISR_CTIS_MASK_NOOFST		0x1 //!< CTIS mask
#define MML_SC_ISR_CTIS_MASK				( MML_SC_ISR_CTIS_MASK_NOOFST << MML_SC_ISR_CTIS_OFST )

#define MML_SC_ISR_TCIS_OFST				3 //!< TCIS offset
#define MML_SC_ISR_TCIS_MASK_NOOFST		0x1 //!< TCIS mask
#define MML_SC_ISR_TCIS_MASK				( MML_SC_ISR_TCIS_MASK_NOOFST << MML_SC_ISR_TCIS_OFST )

#define MML_SC_ISR_RXEIS_OFST				4 //!< RXEIS offset
#define MML_SC_ISR_RXEIS_MASK_NOOFST		0x1 //!< RXEIS mask
#define MML_SC_ISR_RXEIS_MASK				( MML_SC_ISR_RXEIS_MASK_NOOFST << MML_SC_ISR_RXEIS_OFST )

#define MML_SC_ISR_RXTIS_OFST				5 //!< RXTIS offset
#define MML_SC_ISR_RXTIS_MASK_NOOFST		0x1 //!< RXTIS mask
#define MML_SC_ISR_RXTIS_MASK				( MML_SC_ISR_RXTIS_MASK_NOOFST << MML_SC_ISR_RXTIS_OFST )

#define MML_SC_ISR_RXFIS_OFST				6 //!< RXFIS offset
#define MML_SC_ISR_RXFIS_MASK_NOOFST		0x1 //!< RXFIS mask
#define MML_SC_ISR_RXFIS_MASK				( MML_SC_ISR_RXFIS_MASK_NOOFST << MML_SC_ISR_RXFIS_OFST )

#define MML_SC_ISR_TXEIS_OFST				7 //!< TXEIS offset
#define MML_SC_ISR_TXEIS_MASK_NOOFST		0x1 //!< TXEIS mask
#define MML_SC_ISR_TXEIS_MASK				( MML_SC_ISR_TXEIS_MASK_NOOFST << MML_SC_ISR_TXEIS_OFST )

#define MML_SC_ISR_TXTIS_OFST				8 //!< TXTIS offset
#define MML_SC_ISR_TXTIS_MASK_NOOFST		0x1 //!< TXTIS mask
#define MML_SC_ISR_TXTIS_MASK				( MML_SC_ISR_TXTIS_MASK_NOOFST << MML_SC_ISR_TXTIS_OFST )

#define MML_SC_ISR_PRCIS_OFST				9 //!< PRCIS offset
#define MML_SC_ISR_PRCIS_MASK_NOOFST		0x1 //!< PRCIS mask
#define MML_SC_ISR_PRCIS_MASK				(MML_SC_ISR_PRCIS_MASK_NOOFST << MML_SC_ISR_PRCIS_OFST)

#define MML_SC_ISR_PDLIS_OFST				10 //!< PDLIS offset
#define MML_SC_ISR_PDLIS_MASK_NOOFST		0x1 //!< PDLIS mask
#define MML_SC_ISR_PDLIS_MASK				(MML_SC_ISR_PDLIS_MASK_NOOFST << MML_SC_ISR_PDLIS_OFST)
/** @} */ /* @defgroup MML_SC_REGS_ISR */

/** @defgroup MML_SC_REGS_TXR SC TX Register
 *
 * @li SCxTXR [7:0] - W: DATA
 * @li SCxTXR [31:8] - R: RFU
 *
 * @{
 */
#define MML_SC_TXR_OFST					0x24 //!< TX Register offset
#define MML_SC_TXR_DFLT					0x00000000 //!< TX Register default value
/* Bits Fields */
#define MML_SC_TXR_DATA_OFST				0 //!< DATA offset
#define MML_SC_TXR_DATA_MASK_NOOFST		0xff //!< DATA mask
#define MML_SC_TXR_DATA_MASK				( MML_SC_TXR_DATA_MASK_NOOFST << MML_SC_TXR_DATA_OFST )
/** @} */ /* @defgroup MML_SC_REGS_TXR */

/** @defgroup MML_SC_REGS_RXR SC RX Register
 *
 * @li SCxTXR [7:0] - R: DATA
 * @li SCxTXR [8] - R: PARER
 * @li SCxTXR [31:9] - R: RFU
 *
 * @{
 */
#define MML_SC_RXR_OFST					0x28 //!< RX Register offset
#define MML_SC_RXR_DFLT					0x00000000 //!< RX Register default value
/* Bits Fields */
#define MML_SC_RXR_DATA_OFST				0 //!< DATA offset
#define MML_SC_RXR_DATA_MASK_NOOFST		0xff //!< DATA mask
#define MML_SC_RXR_DATA_MASK				( MML_SC_RXR_DATA_MASK_NOOFST << MML_SC_RXR_DATA_OFST )

#define MML_SC_TXR_PARER_OFST				8 //!< PARER offset
#define MML_SC_TXR_PARER_MASK_NOOFST		0x1 //!< PARER mask
#define MML_SC_TXR_PARER_MASK				( MML_SC_RXR_PARER_MASK_NOOFST << MML_SC_RXR_PARER_OFST )
/** @} */ /* @defgroup MML_SC_REGS_RXR */

/** @defgroup MML_SC_REGS_CCR SC Clock Counter Register
 *
 * @li SCxCCR [23:0] - R/W: CCYC
 * @li SCxCCR [30:24] - R: RFU
 * @li SCxCCR [31] - R/W: MAN
 *
 * @{
 */
#define MML_SC_CCR_OFST					0x2c //!< Clock Counter Register offset
#define MML_SC_CCR_DFLT					0x00000000 //!< Clock Counter Register default value
/* Bits Fields */
#define MML_SC_CCR_CCYC_OFST				0 //!< CCYC offset
#define MML_SC_CCR_CCYC_MASK_NOOFST		0xffffff //!< CCYC mask
#define MML_SC_CCR_CCYC_MASK				( MML_SC_CCR_CCYC_MASK_NOOFST << MML_SC_CCR_CCYC_OFST )

#define MML_SC_CCR_MAN_OFST				31 //!< MAN offset
#define MML_SC_CCR_MAN_MASK_NOOFST			0x1u //!< MAN mask
#define MML_SC_CCR_MAN_MASK				( MML_SC_CCR_MAN_MASK_NOOFST << MML_SC_CCR_MAN_OFST )
/** @} */ /* @defgroup MML_SC_REGS_CCR */

/* -------------------------------------------------------------------------- */
#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */
/** SC Registers.
 *
 */
typedef volatile struct
{
	/** SC control register */
	volatile unsigned int					sc_cr;
	/** SC status register */
	volatile unsigned int					sc_sr;
	/** SC pin register */
	volatile unsigned int					sc_pnr;
	/** SC etu register */
	volatile unsigned int					sc_etur;
	/** SC guard time register */
	volatile unsigned int					sc_gtr;
	/** SC waiting time 0 register */
	volatile unsigned int					sc_wt0r;
	/** SC waiting time 1 register */
	volatile unsigned int					sc_wt1r;
	/** SC interrupt enable register */
	volatile unsigned int					sc_ier;
	/** SC interrupt status register */
	volatile unsigned int					sc_isr;
	/** SC tx register */
	volatile unsigned int					sc_txr;
	/** SC rx register */
	volatile unsigned int					sc_rxr;
	/** SC clock counter register */
	volatile unsigned int					sc_ccr;

} mml_sc_regs_t;

#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_SC_REGS */

#endif /* _MML_SC_REGS_H_ */

/******************************************************************************/
/* EOF */
