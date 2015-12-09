/*
 * mml_mcr_regs.h --
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
 * $Revision: 4487 $:  Revision of last commit
 * $Author: jeremy.kongs $:  Author of last commit
 * $Date: 2015-01-14 15:43:29 -0600 (Wed, 14 Jan 2015) $:  Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_MCR_REGS_H_
#define _MML_MCR_REGS_H_

/** @file mml_mcr_regs.h MCR Registers Header */

/** @defgroup MML_MCR MCR Driver */

/** @defgroup MML_MCR_REGS MCR Registers
 *
 * @note MCR IP or Specification version number
 *
 * @ingroup MML_MCR
 * @{
 */

/* Indirect Addressed Register indicies */
#define MML_MCR_IDX_T13_SCALE       (0x00)
#define MML_MCR_IDX_T2_SCALE        (0x01)
#define MML_MCR_IDX_T13_ZCT_FAST    (0x02)
#define MML_MCR_IDX_T13_ZCT_MID     (0x03)
#define MML_MCR_IDX_T13_ZCT_SLOW    (0x04)
#define MML_MCR_IDX_T2_ZCT_FAST     (0x05)
#define MML_MCR_IDX_T2_ZCT_MID      (0x06)
#define MML_MCR_IDX_T2_ZCT_SLOW     (0x07)
#define MML_MCR_IDX_START_PCNT      (0x08)
#define MML_MCR_IDX_T1_DATA_CNT     (0x09)
#define MML_MCR_IDX_T2_DATA_CNT     (0x0A)
#define MML_MCR_IDX_T3_DATA_CNT     (0x0B)
#define MML_MCR_IDX_ADCCFG1         (0x12)

/*  CTRL  ******************************************************************  */
#define MML_MCR_CTRL_EN_POS           (0)
#define MML_MCR_CTRL_EN               (0x00000001UL << MML_MCR_CTRL_EN_POS)
#define MML_MCR_CTRL_PKDETECT_POS     (1)
#define MML_MCR_CTRL_PKDETECT         (0x00000001UL << MML_MCR_CTRL_PKDETECT_POS)
#define MML_MCR_CTRL_FIFO_INTE_POS    (2)
#define MML_MCR_CTRL_FIFO_INTE        (0x00000001UL << MML_MCR_CTRL_FIFO_INTE_POS)
#define MML_MCR_CTRL_DSP_INTF_POS     (4)
#define MML_MCR_CTRL_DSP_INTF         (0x00000001UL << MML_MCR_CTRL_DSP_INTF_POS)
#define MML_MCR_CTRL_T1_INTF_POS      (5)
#define MML_MCR_CTRL_T1_INTF          (0x00000001UL << MML_MCR_CTRL_T1_INTF_POS)
#define MML_MCR_CTRL_T2_INTF_POS      (6)
#define MML_MCR_CTRL_T2_INTF          (0x00000001UL << MML_MCR_CTRL_T2_INTF_POS)
#define MML_MCR_CTRL_T3_INTF_POS      (7)
#define MML_MCR_CTRL_T3_INTF          (0x00000001UL << MML_MCR_CTRL_T3_INTF_POS)
#define MML_MCR_CTRL_DVI_RATIO_POS    (8)
#define MML_MCR_CTRL_DVI_RATIO        (0x0000007FUL << MML_MCR_CTRL_DVI_RATIO_POS)

/*  ADCCFG1  ***************************************************************  */
#define MML_MCR_ADCCFG1_PUADC_POS     (0)
#define MML_MCR_ADCCFG1_PUADC         (0x00000001UL << MML_MCR_ADCCFG1_PUADC_POS)
#define MML_MCR_ADCCFG1_VREF_SEL_POS  (2)
#define MML_MCR_ADCCFG1_VREF_SEL      (0x00000003UL << MML_MCR_ADCCFG1_VREF_SEL_POS)
#define MML_MCR_ADCCFG1_VREF_SEL_1P0V   (0x00000000UL << MML_MCR_ADCCFG1_VREF_SEL_POS)
#define MML_MCR_ADCCFG1_VREF_SEL_0P8V   (0x00000001UL << MML_MCR_ADCCFG1_VREF_SEL_POS)
#define MML_MCR_ADCCFG1_VREF_SEL_0P6V   (0x00000002UL << MML_MCR_ADCCFG1_VREF_SEL_POS)
#define MML_MCR_ADCCFG1_VREF_SEL_0P5V   (0x00000003UL << MML_MCR_ADCCFG1_VREF_SEL_POS)
#define MML_MCR_ADCCFG1_OFSAZ_POS     (4)
#define MML_MCR_ADCCFG1_OFSAZ         (0x00000007UL << MML_MCR_ADCCFG1_OFSAZ_POS)
#define MML_MCR_ADCCFG1_OFSAZ_GND0    (0x00000001UL << MML_MCR_ADCCFG1_OFSAZ_POS)
#define MML_MCR_ADCCFG1_OFSAZ_GND1    (0x00000002UL << MML_MCR_ADCCFG1_OFSAZ_POS)
#define MML_MCR_ADCCFG1_OFSAZ_GND2    (0x00000004UL << MML_MCR_ADCCFG1_OFSAZ_POS)
#define MML_MCR_ADCCFG1_RCR1_POS      (8)
#define MML_MCR_ADCCFG1_RCR1          (0x00000003UL << MML_MCR_ADCCFG1_RCR1_POS)
#define MML_MCR_ADCCFG1_RCR1_SHORT    (0x00000000UL << MML_MCR_ADCCFG1_RCR1_POS)
#define MML_MCR_ADCCFG1_RCR1_0P5KOHM  (0x00000001UL << MML_MCR_ADCCFG1_RCR1_POS)
#define MML_MCR_ADCCFG1_RCR1_1P0KOHM  (0x00000002UL << MML_MCR_ADCCFG1_RCR1_POS)
#define MML_MCR_ADCCFG1_RCR1_2P0KOHM  (0x00000003UL << MML_MCR_ADCCFG1_RCR1_POS)
#define MML_MCR_ADCCFG1_RCR2_POS      (10)
#define MML_MCR_ADCCFG1_RCR2          (0x00000007UL << MML_MCR_ADCCFG1_RCR2_POS)
#define MML_MCR_ADCCFG1_RCR2_OPEN     (0x00000000UL << MML_MCR_ADCCFG1_RCR2_POS)
#define MML_MCR_ADCCFG1_RCR2_1P5KOHM  (0x00000004UL << MML_MCR_ADCCFG1_RCR2_POS)
#define MML_MCR_ADCCFG1_RCR2_3P0KOHM  (0x00000005UL << MML_MCR_ADCCFG1_RCR2_POS)
#define MML_MCR_ADCCFG1_RCR2_6P0KOHM  (0x00000006UL << MML_MCR_ADCCFG1_RCR2_POS)
#define MML_MCR_ADCCFG1_RCR2_12KOHM   (0x00000007UL << MML_MCR_ADCCFG1_RCR2_POS)
#define MML_MCR_ADCCFG1_EXT_REF_POS   (16)
#define MML_MCR_ADCCFG1_EXT_REF       (0x00000001UL << MML_MCR_ADCCFG1_EXT_REF_POS)
#define MML_MCR_ADCCFG1_INT_REF_POS   (17)
#define MML_MCR_ADCCFG1_INT_REF       (0x00000001UL << MML_MCR_ADCCFG1_INT_REF_POS)

/* -------------------------------------------------------------------------- */
#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */
/** MCR Registers.
 *
 */
typedef volatile struct
{
	/** MCRx */
	volatile unsigned int addr; //< Addr register
	volatile unsigned int ctrl; //< Control register
	volatile unsigned int data; //< Data register
	volatile unsigned int tfifo[3]; //< Track FIFO registers

} mml_mcr_regs_t;

#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_MCR_REGS */

#endif /* _MML_SC_REGS_H_ */

/******************************************************************************/
/* EOF */
