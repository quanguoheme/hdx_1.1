/*
 * mml_dma_regs.h --
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
 * Created on: Oct 23, 2013
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_DMA_REGS_H_
#define _MML_DMA_REGS_H_

/** @file mml_dma_regs.h DMA Registers Header */

/** @defgroup MML_DMA DMA Driver
 * @ingroup MML_DRIVER
 */

/** @defgroup MML_DMA_REGS DMA Registers
 *
 * @note DMA IP or Specification version number
 *
 * @ingroup MML_DMA
 * @{
 */

/** General macros for channel offset */
#define MML_DMA_CH0_OFST						0 //!< Channel-0 offset
#define MML_DMA_CH1_OFST						1 //!< Channel-1 offset
#define MML_DMA_CH2_OFST						2 //!< Channel-2 offset
#define MML_DMA_CH3_OFST						3 //!< Channel-3 offset
#define MML_DMA_CH4_OFST						4 //!< Channel-4 offset
#define MML_DMA_CH5_OFST						5 //!< Channel-5 offset
#define MML_DMA_CH6_OFST						6 //!< Channel-6 offset
#define MML_DMA_CH7_OFST						7 //!< Channel-7 offset

/** @defgroup MML_DMA_REGS_CN DMA Control Register
 *
 * @li CHIEN[7:0] 	- R/W: Channel Interrupt Enable
 * @li RFU [15:8] 	- R: Reserved
 * @li EXTPOL [16] 	- R/W: External Polarity.
 * @li RFU [31:17] 	- R: Reserved
 * @{
 */
#define MML_DMA_CN_OFST							0x0 //!< DMA_CN Register offset
#define MML_DMA_CN_DFLT							0x0 //!< DMA_CN Register default value

/** Bits Fields */
#define MML_DMA_CN_CH0_IEN_MASK_NOOFST			0x1 //!< Channel-0 Interrupt Enable mask no offset
#define MML_DMA_CN_CH0_IEN_MASK					( MML_DMA_CN_CH0_IEN_MASK_NOOFST << MML_DMA_CH0_OFST ) //!< Channel-0 Interrupt Enable mask

#define MML_DMA_CN_CH1_IEN_MASK_NOOFST			0x1 //!< Channel-1 Interrupt Enable mask no offset
#define MML_DMA_CN_CH1_IEN_MASK					( MML_DMA_CN_CH1_IEN_MASK_NOOFST << MML_DMA_CH1_OFST ) //!< Channel-1 Interrupt Enable mask

#define MML_DMA_CN_CH2_IEN_MASK_NOOFST			0x1 //!< Channel-2 Interrupt Enable mask no offset
#define MML_DMA_CN_CH2_IEN_MASK					( MML_DMA_CN_CH2_IEN_MASK_NOOFST << MML_DMA_CH2_OFST ) //!< Channel-2 Interrupt Enable mask

#define MML_DMA_CN_CH3_IEN_MASK_NOOFST			0x1 //!< Channel-3 Interrupt Enable mask no offset
#define MML_DMA_CN_CH3_IEN_MASK					( MML_DMA_CN_CH3_IEN_MASK_NOOFST << MML_DMA_CH3_OFST ) //!< Channel-3 Interrupt Enable mask

#define MML_DMA_CN_CH4_IEN_MASK_NOOFST			0x1 //!< Channel-4 Interrupt Enable mask no offset
#define MML_DMA_CN_CH4_IEN_MASK					( MML_DMA_CN_CH4_IEN_MASK_NOOFST << MML_DMA_CH4_OFST ) //!< Channel-4 Interrupt Enable mask

#define MML_DMA_CN_CH5_IEN_MASK_NOOFST			0x1 //!< Channel-5 Interrupt Enable mask no offset
#define MML_DMA_CN_CH5_IEN_MASK					( MML_DMA_CN_CH5_IEN_MASK_NOOFST << MML_DMA_CH5_OFST ) //!< Channel-5 Interrupt Enable mask

#define MML_DMA_CN_CH6_IEN_MASK_NOOFST			0x1 //!< Channel-6 Interrupt Enable mask no offset
#define MML_DMA_CN_CH7_IEN_MASK					( MML_DMA_CN_CH6_IEN_MASK_NOOFST << MML_DMA_CH7_OFST ) //!< Channel-7 Interrupt Enable mask

#define MML_DMA_CN_EXTPOL_OFST					16 //!< EXTPOL External Polarity offset.
#define MML_DMA_CN_EXTPOL_MASK_NOOFST			0x1 //!< EXTPOL External Polarity mask no offset
#define MML_DMA_CN_EXTPOL_MASK					( MML_DMA_CN_EXTPOL_MASK_NOOFST << MML_DMA_CN_EXTPOL_OFST ) //!< EXTPOL External Polarity mask

#define MML_DMA_CN_EXTPOL_AL					~MML_DMA_CN_EXTPOL_MASK //!< EXTPOL 0: Active low
#define MML_DMA_CN_EXTPOL_AH					MML_DMA_CN_EXTPOL_MASK //!< EXTPOL 1: Active high

/** We use the term enabled/disabled for bits field of size 1 */
#define MML_DMA_CN_CHIEN_ENABLE					0x1 //!< CHIEN enable
#define MML_DMA_CN_CHIEN_DISABLE				0x0 //!< CHIEN disable

/** @} */ /** @defgroup MML_DMA_REGS_CN */

/** @defgroup MML_DMA_REGS_INT DMA Interrupt Register
 * @{
 * @li IPEND[7:0] 	- R: Channel Interrupt
 * @li RFU [31:8]	- R: Reserved
 */
#define MML_DMA_INT_OFST						0x4 //!< DMA Interrupt Register offset
#define MML_DMA_INT_DFLT						0x0 //!< DMA Interrupt Register default value

/** Bits Fields */
#define MML_DMA_INT_CH0_IPEND_MASK_NOOFST		0x1 //!< Channel-0 Interrupt pending mask no offset
#define MML_DMA_INT_CH0_IPEND_MASK				( MML_DMA_INT_CH0_IPEND_MASK_NOOFST << MML_DMA_CH0_OFST ) //!< Channel-0 Interrupt pending mask

#define MML_DMA_INT_CH1_IPEND_MASK_NOOFST		0x1 //!< Channel-1 Interrupt pending mask no offset
#define MML_DMA_INT_CH1_IPEND_MASK				( MML_DMA_INT_CH1_IPEND_MASK_NOOFST << MML_DMA_CH1_OFST ) //!< Channel-1 Interrupt pending mask

#define MML_DMA_INT_CH2_IPEND_MASK_NOOFST		0x1 //!< Channel-2 Interrupt pending mask no offset
#define MML_DMA_INT_CH2_IPEND_MASK				( MML_DMA_INT_CH2_IPEND_MASK_NOOFST << MML_DMA_CH2_OFST ) //!< Channel-2 Interrupt pending mask

#define MML_DMA_INT_CH3_IPEND_MASK_NOOFST		0x1 //!< Channel-3 Interrupt pending mask no offset
#define MML_DMA_INT_CH3_IPEND_MASK				( MML_DMA_INT_CH3_IPEND_MASK_NOOFST << MML_DMA_CH3_OFST ) //!< Channel-3 Interrupt pending mask

#define MML_DMA_INT_CH4_IPEND_MASK_NOOFST		0x1 //!< Channel-4 Interrupt pending mask no offset
#define MML_DMA_INT_CH4_IPEND_MASK				( MML_DMA_INT_CH4_IPEND_MASK_NOOFST << MML_DMA_CH4_OFST ) //!< Channel-4 Interrupt pending mask

#define MML_DMA_INT_CH5_IPEND_MASK_NOOFST		0x1 //!< Channel-5 Interrupt pending mask no offset
#define MML_DMA_INT_CH5_IPEND_MASK				( MML_DMA_INT_CH5_IPEND_MASK_NOOFST << MML_DMA_CH5_OFST ) //!< Channel-5 Interrupt pending mask

#define MML_DMA_INT_CH6_IPEND_MASK_NOOFST		0x1 //!< Channel-6 Interrupt pending mask no offset
#define MML_DMA_INT_CH6_IPEND_MASK				( MML_DMA_INT_CH6_IPEND_MASK_NOOFST << MML_DMA_CH6_OFST ) //!< Channel-6 Interrupt pending mask

#define MML_DMA_INT_CH7_IPEND_MASK_NOOFST		0x1 //!< Channel-7 Interrupt pending mask no offset
#define MML_DMA_INT_CH7_IPEND_MASK				( MML_DMA_INT_CH7_IPEND_MASK_NOOFST << MML_DMA_CH7_OFST ) //!< Channel-7 Interrupt pending mask

#define MML_DMA_INT_IPEND_PENDING				0x1 //!< IPEND 1: Interrupt Pending
#define MML_DMA_INT_IPEND_NOT_PENDING 			0x0 //!< IPEND 0: No Interrupt

/** @} */ /** @defgroup MML_DMA_REGS_INT */

/** @defgroup MML_DMA_REGS_CFG DMA Configuration Register
 * @{
 * @li	CHEN[0]	- R/W: Channel Enable
 * @li	RLDEN[1] - R/W: Reload Enable
 * @li	PRI[3:2] - R/W: DMA Priority
 * @li	REQSEL[9:4] - R/W: Request Select
 * @li 	REQWAIT[10] - R/W: Request Wait Enable
 * @li	TOSEL[13:11] - R/W: Time-Out Select
 * @li 	PSSEL[15:14] - R/W: Pre-Scale Select
 * @li	SRCWD[17:16] - R/W: Source Width
 * @li	SRCINC[18] - R/W: Source Increment Enable
 * @li	RFU[19] - R: Reserved
 * @li 	DSTWD[21:20] - R/W: Destination Width
 * @li	DSTINC[22] - R/W: Destination Increment Enable
 * @li	RFU[23] - R: Reserved
 * @li	BRST[28:24]- R/W: Burst Size
 * @li	RFU[29] - R: Reserved
 * @li	CHDIEN[30] - R/W: Channel Disable Interrupt Enable
 * @li	CTZIEN[31] - R/W: Count-to-zero Interrupts Enable
 * @li	RFU[29] - R: Reserved
 */
#define MML_DMA_CFG_OFST						0x100 //!< DMA Channel-0 Configuration Register offset
#define MML_DMA_CFG_DFLT						0x0 //!< DMA Channel-n default value

/** Bits Fields which are identical to all DMA channels */
#define MML_DMA_CFG_CHEN_OFST					0 //!< DMA Channel Enable
#define MML_DMA_CFG_CHEN_MASK_NOOFST			0x1 //!< DMA Channel Enable mask no offset
#define MML_DMA_CFG_CHEN_MASK					( MML_DMA_CFG_CHEN_MASK_NOOFST << MML_DMA_CFG_CHEN_OFST ) //!< DMA Channel Enable mask

#define MML_DMA_CFG_RLDEN_OFST					1 //!< Reload Enable
#define MML_DMA_CFG_RLDEN_MASK_NOOFST			0x1 //!< Reload Enable mask no offset
#define MML_DMA_CFG_RLDEN_MASK					( MML_DMA_CFG_RLDEN_MASK_NOOFST << MML_DMA_CFG_RLDEN_OFST ) //!< Reload Enable mask

#define MML_DMA_CFG_PRI_OFST					2 //!< DMA Priority
#define MML_DMA_CFG_PRI_MASK_NOOFST				0x3 //!< DMA Priority mask no offset
#define MML_DMA_CFG_PRI_MASK					( MML_DMA_CFG_PRI_MASK_NOOFST << MML_DMA_CFG_PRI_OFST ) //!< DMA Priority mask

#define MML_DMA_CFG_REQSEL_OFST					4 //!< Request Select
#define MML_DMA_CFG_REQSEL_MASK_NOOFST			0x3f //!< Request Select mask no offset
#define MML_DMA_CFG_REQSEL_MASK					( MML_DMA_CFG_REQSEL_MASK_NOOFST << MML_DMA_CFG_REQSEL_OFST ) //!< Request Select mask

#define MML_DMA_CFG_REQWAIT_OFST				10 //!< Request Wait Enable
#define MML_DMA_CFG_REQWAIT_MASK_NOOFST			0x1 //!< Request Wait Enable mask no offset
#define MML_DMA_CFG_REQWAIT_MASK				( MML_DMA_CFG_REQWAIT_MASK_NOOFST << MML_DMA_CFG_REQWAIT_OFST ) //!< Request Wait Enable mask

#define MML_DMA_CFG_TOSEL_OFST					11 //!< Time-Out Select
#define MML_DMA_CFG_TOSEL_MASK_NOOFST			0x7 //!< Time-Out Select mask no offset
#define MML_DMA_CFG_TOSEL_MASK					( MML_DMA_CFG_TOSEL_MASK_NOOFST << MML_DMA_CFG_TOSEL_OFST ) //!< Time-Out Select mask

#define MML_DMA_CFG_PSSEL_OFST					14 //!< Pre-Scale Select
#define MML_DMA_CFG_PSSEL_MASK_NOOFST			0x3 //!< Pre-Scale Select mask no offset
#define MML_DMA_CFG_PSSEL_MASK					( MML_DMA_CFG_PSSEL_MASK_NOOFST << MML_DMA_CFG_PSSEL_OFST ) //!< Pre-Scale Select mask

#define MML_DMA_CFG_SRCWD_OFST					16 //!< Source Width
#define MML_DMA_CFG_SRCWD_MASK_NOOFST			0x3 //!< Source Width mask no offset
#define MML_DMA_CFG_SRCWD_MASK					( MML_DMA_CFG_SRCWD_MASK_NOOFST << MML_DMA_CFG_SRCWD_OFST ) //!< Source Width mask

#define MML_DMA_CFG_SRCINC_OFST					18 //!< Source Increment Enable
#define MML_DMA_CFG_SRCINC_MASK_NOOFST			0x1 //!< Source Increment Enable mask no offset
#define MML_DMA_CFG_SRCINC_MASK					( MML_DMA_CFG_SRCINC_MASK_NOOFST << MML_DMA_CFG_SRCINC_OFST ) //!< Source Increment Enable mask

#define MML_DMA_CFG_DSTWD_OFST					20 //!< Destination Width
#define MML_DMA_CFG_DSTWD_MASK_NOOFST			0x3 //!< Destination Width mask no offset
#define MML_DMA_CFG_DSTWD_MASK					( MML_DMA_CFG_DSTWD_MASK_NOOFST << MML_DMA_CFG_DSTWD_OFST ) //!< Destination Width mask

#define MML_DMA_CFG_DSTINC_OFST					22 //!< Destination Increment Enable
#define MML_DMA_CFG_DSTINC_MASK_NOOFST			0x1 //!< Destination Increment Enable mask no offset
#define MML_DMA_CFG_DSTINC_MASK					( MML_DMA_CFG_DSTINC_MASK_NOOFST << MML_DMA_CFG_DSTINC_OFST ) //!< Destination Increment Enable mask

#define MML_DMA_CFG_BRST_OFST					24 //!< Burst Size
#define MML_DMA_CFG_BRST_MASK_NOOFST			0x1f //!< Burst Size mask no offset
#define MML_DMA_CFG_BRST_MASK					( MML_DMA_CFG_BRST_MASK_NOOFST << MML_DMA_CFG_BRST_OFST ) //!< Burst Size mask

#define MML_DMA_CFG_CHDIEN_OFST					30 //!< Channel Disable Interrupt Enable
#define MML_DMA_CFG_CHDIEN_MASK_NOOFST			0x1 //!< Channel Disable Interrupt Enable mask no offset
#define MML_DMA_CFG_CHDIEN_MASK					( MML_DMA_CFG_CHDIEN_MASK_NOOFST << MML_DMA_CFG_CHDIEN_OFST ) //!< Channel Disable Interrupt Enable mask

#define MML_DMA_CFG_CTZIEN_OFST					31 //!< Count-to-zero Interrupts Enable
#define MML_DMA_CFG_CTZIEN_MASK_NOOFST			0x1u //!< Count-to-zero Interrupts Enable mask no offset
#define MML_DMA_CFG_CTZIEN_MASK					( MML_DMA_CFG_CTZIEN_MASK_NOOFST << MML_DMA_CFG_CTZIEN_OFST ) //!< Count-to-zero Interrupts Enable mask

/** @} */ /** @defgroup MML_DMA_REGS_CFG */

/** @defgroup MML_DMA_REGS_ST DMA Status Register
 * @{
 * @li	CH_ST[0]	- R: Channel Status
 * @li	IPEND[1]	- R: Channel Interrupt
 * @li  CTZ_ST[2] - RW1C: Count-to-Zero (CTZ) Status.
 * @li	RLD_ST[3] - RW1C: Reload Status
 * @li 	BUS_ERR[4] - RW1C: Bus Error
 * @li	RFU[5] - R: Reserved
 * @li 	TO_ST[6] - RW1C: Time-Out Status.
 * @li	RFU[31:7] - R: Reserved
 */
#define MML_DMA_ST_OFST							0x104 //!< DMA Channel-0 Configuration Register offset
#define MML_DMA_ST_DFLT							0x0 //!< DMA Channel-n default value

/** Bits Fields which are identical to all DMA channels */
#define MML_DMA_ST_CHST_OFST					0 //!< DMA Channel Status
#define MML_DMA_ST_CHST_MASK_NOOFST				0x1 //!< DMA Channel Status mask no offset
#define MML_DMA_ST_CHST_MASK					( MML_DMA_ST_CHST_MASK_NOOFST << MML_DMA_ST_CHST_OFST ) //!< DMA Channel Status mask

#define MML_DMA_ST_IPEND_OFST					1 //!< Channel Interrupt
#define MML_DMA_ST_IPEND_MASK_NOOFST			0x1 //!< Channel Interrupt mask no offset
#define MML_DMA_ST_IPEND_MASK					( MML_DMA_ST_IPEND_MASK_NOOFST << MML_DMA_ST_IPEND_OFST ) //!< Channel Interrupt mask

#define MML_DMA_ST_CTZST_OFST					2 //!< Count-to-Zero (CTZ) Status
#define MML_DMA_ST_CTZST_MASK_NOOFST			0x1 //!< Count-to-Zero (CTZ) Status mask no offset
#define MML_DMA_ST_CTZST_MASK					( MML_DMA_ST_CTZST_MASK_NOOFST << MML_DMA_ST_CTZST_OFST ) //!< Count-to-Zero (CTZ) Status mask

#define MML_DMA_ST_RLDST_OFST					3 //!< Reload Status
#define MML_DMA_ST_RLDST_MASK_NOOFST			0x1 //!< Reload Status mask no offset
#define MML_DMA_ST_RLDST_MASK					( MML_DMA_ST_RLDST_MASK_NOOFST) //!< Reload Status mask

#define MML_DMA_ST_BUSERR_OFST					4 //!< Bus Error
#define MML_DMA_ST_BUSERR_MASK_NOOFST			0x1 //!< Bus Error mask no offset
#define MML_DMA_ST_BUSERR_MASK					( MML_DMA_ST_BUSERR_MASK_NOOFST << MML_DMA_ST_BUSERR_OFST ) //!< Bus Error mask

#define MML_DMA_ST_TOST_OFST					6 //!<< Time-Out Status
#define MML_DMA_ST_TOST_MASK_NOOFST				0x1 //!<< Time-Out Status mask no offset
#define MML_DMA_ST_TOST_MASK					( MML_DMA_ST_TOST_MASK_NOOFST << MML_DMA_ST_TOST_OFST ) //!<< Time-Out Status mask

#define	MML_DMA_ST_MASK							( MML_DMA_ST_CTZST_MASK |\
													MML_DMA_ST_RLDST_MASK |\
													MML_DMA_ST_BUSERR_MASK |\
													MML_DMA_ST_TOST_MASK )

/** @} */ /** @defgroup MML_DMA_REGS_ST */

/** Generic macro definitions */
#define MML_DMA_CNT_RLD_RLDEN_OFST				31 //!<< DMA CNT_RLD Reload Enable offset
#define	MML_DMA_CNT_RLD_RLDEN_MASK_NOOFST		0x1 //!<< DMA CNT_RLD Reload Enable mask no offset
#define	MML_DMA_CNT_RLD_RLDEN_MASK				( MML_DMA_CNT_RLD_RLDEN_MASK_NOOFST << MML_DMA_CNT_RLD_RLDEN_OFST ) //!<< DMA CNT_RLD Reload Enable mask

#define	MML_DMA_CNT_MASK						0xffffff //!<< DMA_CNT Counter CNT[23:0] mask

#define	MML_DMA_RLD_ADDR_OFST					31 //!<< Address mask for reload source and destination address offset
#define	MML_DMA_RLD_ADDR_MASK_NOOFST			0x01u //!<< Address mask for reload source and destination address no offset
#define	MML_DMA_RLD_ADDR_MASK					( MML_DMA_RLD_ADDR_MASK_NOOFST << MML_DMA_RLD_ADDR_OFST ) //!<< Address mask for reload source and destination address

#ifndef __ASSEMBLER__
/** -------------------------------------------------------------------------- */
/** DMA Registers.
 *
 */
/** DMA Channels Registers.
 *
 */
typedef volatile struct
{
	unsigned int								cfg;	//!< DMA Channel Configuration Register
	unsigned int								stat;	//!< DMA Channel Status Register
	unsigned int								src;	//!< DMA Channel Source Register
	unsigned int								dest;	//!< DMA Channel Destination Register
	unsigned int								cnt;	//!< DMA Channel Count Register
	unsigned int								srld;	//!< DMA Channel Source Reload Register
	unsigned int								drld;	//!< DMA Channel Destination Reload Register
	unsigned int								crld;	//!< DMA Channel Count Reload Register

} mml_dma_ch_regs_t;

typedef volatile struct
{
	/** DMA Control Register */
	unsigned int								ctrl;
	/** DMA Interrupt Status Register */
	unsigned int								it_st;
    /** Reserved */
	unsigned int    							rsvd0[62];
    /** 8 identical DMA channels */
	mml_dma_ch_regs_t 							ch[8];

} mml_dma_regs_t;

#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_DMA_REGS */

#endif /* _MML_DMA_REGS_H_ */

/*******************************************************************************/
/* EOF */
