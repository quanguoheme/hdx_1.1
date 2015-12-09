/*
 * mml_scbr_regs.h --
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
 * Created on: Jun 11, 2014
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_SCBR_REGS_H_
#define _MML_SCBR_REGS_H_

/** @file mml_sir_regs.h System Control Block Registers (SCBR) Header */

/** @defgroup MML_SCBR_REGS SCBR Driver
 * @ingroup MML_DRIVER
 */

/** @defgroup MML_SCBR_REGS_CPUID
 *
 * @{
 */
#define MML_SCBR_CPUID_OFST					0x00000000 //!< Register offset
#define MML_SCBR_CPUID_DFLT					0x412fc230 //!< Register default value
/* Bits Fields */
#define MML_SCBR_CPUID_REV_OFST				0 //!< offset
#define MML_SCBR_CPUID_REV_MASK_NOOFST			0xf //!< mask no offset
#define MML_SCBR_CPUID_REV_MASK				( MML_SCBR_CPUID_REV_MASK_NOOFST << MML_SCBR_CPUID_REV_OFST ) //!< mask

#define MML_SCBR_CPUID_PN_OFST					4 //!< offset
#define MML_SCBR_CPUID_PN_MASK_NOOFST			0xfff //!< mask no offset
#define MML_SCBR_CPUID_PN_MASK					( MML_SCBR_CPUID_PN_MASK_NOOFST << MML_SCBR_CPUID_PN_OFST ) //!< mask

#define MML_SCBR_CPUID_CST_OFST				16 //!< offset
#define MML_SCBR_CPUID_CST_MASK_NOOFST			0xf //!< mask no offset
#define MML_SCBR_CPUID_CST_MASK				( MML_SCBR_CPUID_CST_MASK_NOOFST << MML_SCBR_CPUID_CST_OFST ) //!< mask

#define MML_SCBR_CPUID_VAR_OFST				20 //!< offset
#define MML_SCBR_CPUID_VAR_MASK_NOOFST			0xf //!< mask no offset
#define MML_SCBR_CPUID_VAR_MASK				( MML_SCBR_CPUID_VAR_MASK_NOOFST << MML_SCBR_CPUID_VAR_OFST ) //!< mask

#define MML_SCBR_CPUID_IMP_OFST				24 //!< offset
#define MML_SCBR_CPUID_IMP_MASK_NOOFST			0xff //!< mask no offset
#define MML_SCBR_CPUID_IMP_MASK				( MML_SCBR_CPUID_IMP_MASK_NOOFST << MML_SCBR_CPUID_IMP_OFST ) //!< mask
/** @} */ /* @defgroup MML_SCBR_REGS_MCR */

/** @defgroup MML_SCBR_REGS_ICSR
 *
 * @{
 */
#define MML_SCBR_ICSR_OFST						0x00000004 //!< Register offset
#define MML_SCBR_ICSR_DFLT						0x00000000 //!< Register default value
/* Bits Fields */
#define MML_SCBR_ICSR_VECTACTIVE_OFST			0 //!< offset
#define MML_SCBR_ICSR_VECTACTIVE_MASK_NOOFST	0x1ff //!< mask no offset
#define MML_SCBR_ICSR_VECTACTIVE_MASK			( MML_SCBR_ICSR_VECTACTIVE_NOOFST << MML_SCBR_ICSR_VECTACTIVE_OFST ) //!< mask

#define MML_SCBR_ICSR_RETTOBASE_OFST			11 //!< offset
#define MML_SCBR_ICSR_RETTOBASE_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_ICSR_RETTOBASE_MASK			( MML_SCBR_ICSR_RETTOBASE_NOOFST << MML_SCBR_ICSR_RETTOBASE_OFST ) //!< mask

#define MML_SCBR_ICSR_VECTPENDING_OFST			12 //!< offset
#define MML_SCBR_ICSR_VECTPENDING_MASK_NOOFST	0x3f //!< mask no offset
#define MML_SCBR_ICSR_VECTPENDING_MASK			( MML_SCBR_ICSR_VECTPENDING_NOOFST << MML_SCBR_ICSR_VECTPENDING_OFST ) //!< mask

#define MML_SCBR_ICSR_ISRPENDING_OFST			22 //!< offset
#define MML_SCBR_ICSR_ISRPENDING_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_ICSR_ISRPENDING_MASK			( MML_SCBR_ICSR_ISRPENDING_NOOFST << MML_SCBR_ICSR_ISRPENDING_OFST ) //!< mask

#define MML_SCBR_ICSR_PENDSTCLR_OFST			25 //!< offset
#define MML_SCBR_ICSR_PENDSTCLR_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_ICSR_PENDSTCLR_MASK			( MML_SCBR_ICSR_PENDSTCLR_NOOFST << MML_SCBR_ICSR_PENDSTCLR_OFST ) //!< mask

#define MML_SCBR_ICSR_PENDSTSET_OFST			26 //!< offset
#define MML_SCBR_ICSR_PENDSTSET_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_ICSR_PENDSTSET_MASK			( MML_SCBR_ICSR_PENDSTSET_NOOFST << MML_SCBR_ICSR_PENDSTSET_OFST ) //!< mask

#define MML_SCBR_ICSR_PENDSVCLR_OFST			27 //!< offset
#define MML_SCBR_ICSR_PENDSVCLR_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_ICSR_PENDSVCLR_MASK			( MML_SCBR_ICSR_PENDSVCLR_NOOFST << MML_SCBR_ICSR_PENDSVCLR_OFST ) //!< mask

#define MML_SCBR_ICSR_PENDSVSET_OFST			28 //!< offset
#define MML_SCBR_ICSR_PENDSVSET_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_ICSR_PENDSVSET_MASK			( MML_SCBR_ICSR_PENDSVSET_NOOFST << MML_SCBR_ICSR_PENDSVSET_OFST ) //!< mask

#define MML_SCBR_ICSR_NMIPENDSET_OFST			31 //!< offset
#define MML_SCBR_ICSR_NMIPENDSET_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_ICSR_NMIPENDSET_MASK			( MML_SCBR_ICSR_NMIPENDSET_NOOFST << MML_SCBR_ICSR_NMIPENDSET_OFST ) //!< mask


/** @} */ /* @defgroup MML_SCBR_REGS_ICSR */

/** @defgroup MML_SCBR_REGS_AIRCR
 *
 * @{
 */
#define MML_SCBR_AIRCR_OFST					0x0000000c //!< Register offset
#define MML_SCBR_AIRCR_DFLT					0xfa050000 //!< Register default value
/* Bits Fields */
#define MML_SCBR_AIRCR_VECTRESET_OFST			0 //!< offset
#define MML_SCBR_AIRCR_VECTRESET_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_AIRCR_VECTRESET_MASK			( MML_SCBR_AIRCR_VECTRESET_NOOFST << MML_SCBR_AIRCR_VECTRESET_OFST ) //!< mask

#define MML_SCBR_AIRCR_VECTCLRACTIVE_OFST		1 //!< offset
#define MML_SCBR_AIRCR_VECTCLRACTIVE_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_AIRCR_VECTCLRACTIVE_MASK		( MML_SCBR_AIRCR_VECTCLRACTIVE_NOOFST << MML_SCBR_AIRCR_VECTCLRACTIVE_OFST ) //!< mask

#define MML_SCBR_AIRCR_SYSRESETREQ_OFST		2 //!< offset
#define MML_SCBR_AIRCR_SYSRESETREQ_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_AIRCR_SYSRESETREQ_MASK		( MML_SCBR_AIRCR_SYSRESETREQ_NOOFST << MML_SCBR_AIRCR_SYSRESETREQ_OFST ) //!< mask

#define MML_SCBR_AIRCR_PRIGROUP_OFST			8 //!< offset
#define MML_SCBR_AIRCR_PRIGROUP_MASK_NOOFST	0x7 //!< mask no offset
#define MML_SCBR_AIRCR_PRIGROUP_MASK			( MML_SCBR_AIRCR_PRIGROUP_NOOFST << MML_SCBR_AIRCR_PRIGROUP_OFST ) //!< mask

#define MML_SCBR_AIRCR_ENDIANNESS_OFST			15 //!< offset
#define MML_SCBR_AIRCR_ENDIANNESS_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_AIRCR_ENDIANNESS_MASK			( MML_SCBR_AIRCR_ENDIANNESS_NOOFST << MML_SCBR_AIRCR_ENDIANNESS_OFST ) //!< mask

#define MML_SCBR_AIRCR_VECTKEY_OFST			16 //!< offset
#define MML_SCBR_AIRCR_VECTKEY_MASK_NOOFST		0xffff //!< mask no offset
#define MML_SCBR_AIRCR_VECTKEY_MASK			( MML_SCBR_AIRCR_VECTKEY_NOOFST << MML_SCBR_AIRCR_VECTKEY_OFST ) //!< mask
/** @} */ /* @defgroup MML_SCBR_REGS_AIRCR */

/** @defgroup MML_SCBR_REGS_SCR
 *
 * @{
 */
#define MML_SCBR_SCR_OFST						0x00000010 //!< Register offset
#define MML_SCBR_SCR_DFLT						0x00000000 //!< Register default value
/* Bits Fields */
#define MML_SCBR_SCR_SLEEPONEXIT_OFST			1 //!< offset
#define MML_SCBR_SCR_SLEEPONEXIT_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_SCR_SLEEPONEXIT_MASK			( MML_SCBR_SCR_SLEEPONEXIT_NOOFST << MML_SCBR_SCR_SLEEPONEXIT_OFST ) //!< mask

#define MML_SCBR_SCR_SLEEPDEEP_OFST			2 //!< offset
#define MML_SCBR_SCR_SLEEPDEEP_MASK_NOOFST		0x1 //!< mask no offset
#define MML_SCBR_SCR_SLEEPDEEP_MASK			( MML_SCBR_SCR_SLEEPDEEP_NOOFST << MML_SCBR_SCR_SLEEPDEEP_OFST ) //!< mask

#define MML_SCBR_SCR_SEVONPEND_OFST			4 //!< offset
#define MML_SCBR_SCR_SEVONPEND_MASK_NOOFST		0x1 //!< mask no offset
#define MML_SCBR_SCR_SEVONPEND_MASK			( MML_SCBR_SCR_SEVONPEND_NOOFST << MML_SCBR_SCR_SEVONPEND_OFST ) //!< mask
/** @} */ /* @defgroup MML_SCBR_REGS_SCR */

/** @defgroup MML_SCBR_REGS_CCR
 *
 * @{
 */
#define MML_SCBR_CCR_OFST						0x00000014 //!< Register offset
#define MML_SCBR_CCR_DFLT						0x00000000 //!< Register default value
/* Bits Fields */
#define MML_SCBR_CCR_NONBASETHRDENA_OFST		0 //!< offset
#define MML_SCBR_CCR_NONBASETHRDENA_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_CCR_NONBASETHRDENA_MASK		( MML_SCBR_CCR_NONBASETHRDENA_NOOFST << MML_SCBR_CCR_NONBASETHRDENA_OFST ) //!< mask

#define MML_SCBR_CCR_USERSETMPEND_OFST			1 //!< offset
#define MML_SCBR_CCR_USERSETMPEND_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_CCR_USERSETMPEND_MASK			( MML_SCBR_CCR_USERSETMPEND_NOOFST << MML_SCBR_CCR_USERSETMPEND_OFST ) //!< mask

#define MML_SCBR_CCR_UNALIGN_TRP_OFST			3 //!< offset
#define MML_SCBR_CCR_UNALIGN_TRP_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_CCR_UNALIGN_TRP_MASK			( MML_SCBR_CCR_UNALIGN_TRP_NOOFST << MML_SCBR_CCR_UNALIGN_TRP_OFST ) //!< mask

#define MML_SCBR_CCR_DIV_0_TRP_OFST			4 //!< offset
#define MML_SCBR_CCR_DIV_0_TRP_MASK_NOOFST		0x1 //!< mask no offset
#define MML_SCBR_CCR_DIV_0_TRP_MASK			( MML_SCBR_CCR_DIV_0_TRP_NOOFST << MML_SCBR_CCR_DIV_0_TRP_OFST ) //!< mask

#define MML_SCBR_CCR_BFHFNMIGN_OFST			8 //!< offset
#define MML_SCBR_CCR_BFHFNMIGN_MASK_NOOFST		0x1 //!< mask no offset
#define MML_SCBR_CCR_BFHFNMIGN_MASK			( MML_SCBR_CCR_BFHFNMIGN_NOOFST << MML_SCBR_CCR_BFHFNMIGN_OFST ) //!< mask

#define MML_SCBR_CCR_STKALIGN_OFST				0 //!< offset
#define MML_SCBR_CCR_STKALIGN_MASK_NOOFST		0x1 //!< mask no offset
#define MML_SCBR_CCR_STKALIGN_MASK				( MML_SCBR_CCR_STKALIGN_NOOFST << MML_SCBR_CCR_STKALIGN_OFST ) //!< mask
/** @} */ /* @defgroup MML_SCBR_REGS_CCR */

/** @defgroup MML_SCBR_REGS_SHPRx
 *
 * @{
 */
#define MML_SCBR_SHPR1_OFST					0x00000018 //!< Register offset
#define MML_SCBR_SHPR2_OFST					0x0000001c //!< Register offset
#define MML_SCBR_SHPR3_OFST					0x00000020 //!< Register offset
#define MML_SCBR_SHPRx_DFLT					0x00000000 //!< Register default value
/* Bits Fields */
#define MML_SCBR_SHPR1_PRI_4_OFST				0 //!< offset
#define MML_SCBR_SHPR1_PRI_4_MASK_NOOFST		0xff //!< mask no offset
#define MML_SCBR_SHPR1_PRI_4_MASK				( MML_SCBR_SHPR1_PRI_4_NOOFST << MML_SCBR_SHPR1_PRI_4_OFST ) //!< mask

#define MML_SCBR_SHPR1_PRI_5_OFST				8 //!< offset
#define MML_SCBR_SHPR1_PRI_5_MASK_NOOFST		0xff //!< mask no offset
#define MML_SCBR_SHPR1_PRI_5_MASK				( MML_SCBR_SHPR1_PRI_5_NOOFST << MML_SCBR_SHPR1_PRI_5_OFST ) //!< mask

#define MML_SCBR_SHPR1_PRI_6_OFST				16 //!< offset
#define MML_SCBR_SHPR1_PRI_6_MASK_NOOFST		0xff //!< mask no offset
#define MML_SCBR_SHPR1_PRI_6_MASK				( MML_SCBR_SHPR1_PRI_6_NOOFST << MML_SCBR_SHPR1_PRI_6_OFST ) //!< mask

#define MML_SCBR_SHPR2_PRI_11_OFST				24 //!< offset
#define MML_SCBR_SHPR2_PRI_11_MASK_NOOFST		0xff //!< mask no offset
#define MML_SCBR_SHPR2_PRI_11_MASK				( MML_SCBR_SHPR2_PRI_11_NOOFST << MML_SCBR_SHPR2_PRI_11_OFST ) //!< mask

#define MML_SCBR_SHPR3_PRI_14_OFST				16 //!< offset
#define MML_SCBR_SHPR3_PRI_14_MASK_NOOFST		0xff //!< mask no offset
#define MML_SCBR_SHPR3_PRI_14_MASK				( MML_SCBR_SHPR3_PRI_14_NOOFST << MML_SCBR_SHPR3_PRI_14_OFST ) //!< mask

#define MML_SCBR_SHPR3_PRI_15_OFST				24 //!< offset
#define MML_SCBR_SHPR3_PRI_15_MASK_NOOFST		0xff //!< mask no offset
#define MML_SCBR_SHPR3_PRI_15_MASK				( MML_SCBR_SHPR3_PRI_15_NOOFST << MML_SCBR_SHPR3_PRI_15_OFST ) //!< mask
/** @} */ /* @defgroup MML_SCBR_REGS_SHPRx */

/** @defgroup MML_SCBR_REGS_SHCRS
 *
 * @{
 */
#define MML_SCBR_SHCRS_OFST					0x00000024 //!< Register offset
#define MML_SCBR_SHCRS_DFLT					0x00000000 //!< Register default value
/* Bits Fields */
#define MML_SCBR_SHCRS_MEMFAULTACT_OFST		0 //!< offset
#define MML_SCBR_SHCRS_MEMFAULTACT_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_SHCRS_MEMFAULTACT_MASK		( MML_SCBR_SHCRS_MEMFAULTACT_NOOFST << MML_SCBR_SHCRS_MEMFAULTACT_OFST ) //!< mask

#define MML_SCBR_SHCRS_BUSFAULTACT_OFST		1 //!< offset
#define MML_SCBR_SHCRS_BUSFAULTACT_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_SHCRS_BUSFAULTACT_MASK		( MML_SCBR_SHCRS_BUSFAULTACT_NOOFST << MML_SCBR_SHCRS_BUSFAULTACT_OFST ) //!< mask

#define MML_SCBR_SHCRS_USGFAULTACT_OFST		3 //!< offset
#define MML_SCBR_SHCRS_USGFAULTACT_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_SHCRS_USGFAULTACT_MASK		( MML_SCBR_SHCRS_USGFAULTACT_NOOFST << MML_SCBR_SHCRS_USGFAULTACT_OFST ) //!< mask

#define MML_SCBR_SHCRS_SVCALLACT_OFST			7 //!< offset
#define MML_SCBR_SHCRS_SVCALLACT_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_SHCRS_SVCALLACT_MASK			( MML_SCBR_SHCRS_SVCALLACT_NOOFST << MML_SCBR_SHCRS_SVCALLACT_OFST ) //!< mask

#define MML_SCBR_SHCRS_MONITORACT_OFST			8 //!< offset
#define MML_SCBR_SHCRS_MONITORACT_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_SHCRS_MONITORACT_MASK			( MML_SCBR_SHCRS_MONITORACT_NOOFST << MML_SCBR_SHCRS_MONITORACT_OFST ) //!< mask

#define MML_SCBR_SHCRS_PENDSVACT_OFST			10 //!< offset
#define MML_SCBR_SHCRS_PENDSVACT_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_SHCRS_PENDSVACT_MASK			( MML_SCBR_SHCRS_PENDSVACT_NOOFST << MML_SCBR_SHCRS_PENDSVACT_OFST ) //!< mask

#define MML_SCBR_SHCRS_SYSTICKACT_OFST			11 //!< offset
#define MML_SCBR_SHCRS_SYSTICKACT_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_SHCRS_SYSTICKACT_MASK			( MML_SCBR_SHCRS_SYSTICKACT_NOOFST << MML_SCBR_SHCRS_SYSTICKACT_OFST ) //!< mask

#define MML_SCBR_SHCRS_USGFAULTPENDED_OFST		12 //!< offset
#define MML_SCBR_SHCRS_USGFAULTPENDED_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_SHCRS_USGFAULTPENDED_MASK		( MML_SCBR_SHCRS_USGFAULTPENDED_NOOFST << MML_SCBR_SHCRS_USGFAULTPENDED_OFST ) //!< mask

#define MML_SCBR_SHCRS_MEMFAULTPENDED_OFST		13 //!< offset
#define MML_SCBR_SHCRS_MEMFAULTPENDED_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_SHCRS_MEMFAULTPENDED_MASK		( MML_SCBR_SHCRS_MEMFAULTPENDED_NOOFST << MML_SCBR_SHCRS_MEMFAULTPENDED_OFST ) //!< mask

#define MML_SCBR_SHCRS_BUSFAULTPENDED_OFST		14 //!< offset
#define MML_SCBR_SHCRS_BUSFAULTPENDED_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_SHCRS_BUSFAULTPENDED_MASK		( MML_SCBR_SHCRS_BUSFAULTPENDED_NOOFST << MML_SCBR_SHCRS_BUSFAULTPENDED_OFST ) //!< mask

#define MML_SCBR_SHCRS_SVCALLPENDED_OFST		15 //!< offset
#define MML_SCBR_SHCRS_SVCALLPENDED_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_SHCRS_SVCALLPENDED_MASK		( MML_SCBR_SHCRS_SVCALLPENDED_NOOFST << MML_SCBR_SHCRS_SVCALLPENDED_OFST ) //!< mask

#define MML_SCBR_SHCRS_MEMFAULTENA_OFST		16 //!< offset
#define MML_SCBR_SHCRS_MEMFAULTENA_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_SHCRS_MEMFAULTENA_MASK		( MML_SCBR_SHCRS_MEMFAULTENA_NOOFST << MML_SCBR_SHCRS_MEMFAULTENA_OFST ) //!< mask

#define MML_SCBR_SHCRS_BUSFAULTENA_OFST		17 //!< offset
#define MML_SCBR_SHCRS_BUSFAULTENA_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_SHCRS_BUSFAULTENA_MASK		( MML_SCBR_SHCRS_BUSFAULTENA_NOOFST << MML_SCBR_SHCRS_BUSFAULTENA_OFST ) //!< mask

#define MML_SCBR_SHCRS_USGFAULTENA_OFST		18 //!< offset
#define MML_SCBR_SHCRS_USGFAULTENA_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_SHCRS_USGFAULTENA_MASK		( MML_SCBR_SHCRS_USGFAULTENA_NOOFST << MML_SCBR_SHCRS_USGFAULTENA_OFST ) //!< mask
/** @} */ /* @defgroup MML_SCBR_REGS_SHCRS */

/** @defgroup MML_SCBR_REGS_CFSR
 *
 * @{
 */
#define MML_SCBR_CFSR_OFST						0x00000028 //!< Register offset
#define MML_SCBR_CFSR_DFLT						0x00000000 //!< Register default value
/* Bits Fields */
#define MML_SCBR_MMFSR_IACCVIOL_OFST			0 //!< offset
#define MML_SCBR_MMFSR_IACCVIOL_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_MMFSR_IACCVIOL_MASK			( MML_SCBR_MMFSR_IACCVIOL_NOOFST << MML_SCBR_MMFSR_IACCVIOL_OFST ) //!< mask

#define MML_SCBR_MMFSR_DACCVIOL_OFST			1 //!< offset
#define MML_SCBR_MMFSR_DACCVIOL_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_MMFSR_DACCVIOL_MASK			( MML_SCBR_MMFSR_DACCVIOL_NOOFST << MML_SCBR_MMFSR_DACCVIOL_OFST ) //!< mask

#define MML_SCBR_MMFSR_MUNSTKERR_OFST			3 //!< offset
#define MML_SCBR_MMFSR_MUNSTKERR_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_MMFSR_MUNSTKERR_MASK			( MML_SCBR_MMFSR_MUNSTKERR_NOOFST << MML_SCBR_MMFSR_MUNSTKERR_OFST ) //!< mask

#define MML_SCBR_MMFSR_MSTKERR_OFST			4 //!< offset
#define MML_SCBR_MMFSR_MSTKERR_MASK_NOOFST		0x1 //!< mask no offset
#define MML_SCBR_MMFSR_MSTKERR_MASK			( MML_SCBR_MMFSR_MSTKERR_NOOFST << MML_SCBR_MMFSR_MSTKERR_OFST ) //!< mask

#define MML_SCBR_MMFSR_MMARVALID_OFST			7 //!< offset
#define MML_SCBR_MMFSR_MMARVALID_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_MMFSR_MMARVALID_MASK			( MML_SCBR_MMFSR_MMARVALID_NOOFST << MML_SCBR_MMFSR_MMARVALID_OFST ) //!< mask


#define MML_SCBR_BFSR_IBUSERR_OFST				0 //!< offset
#define MML_SCBR_BFSR_IBUSERR_MASK_NOOFST		0x1 //!< mask no offset
#define MML_SCBR_BFSR_IBUSERR_MASK				( MML_SCBR_BFSR_IBUSERR_NOOFST << MML_SCBR_BFSR_IBUSERR_OFST ) //!< mask

#define MML_SCBR_BFSR_PRECISERR_OFST			1 //!< offset
#define MML_SCBR_BFSR_PRECISERR_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_BFSR_PRECISERR_MASK			( MML_SCBR_BFSR_PRECISERR_NOOFST << MML_SCBR_BFSR_PRECISERR_OFST ) //!< mask

#define MML_SCBR_BFSR_IMPRECISERR_OFST			2 //!< offset
#define MML_SCBR_BFSR_IMPRECISERR_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_BFSR_IMPRECISERR_MASK			( MML_SCBR_BFSR_IMPRECISERR_NOOFST << MML_SCBR_BFSR_IMPRECISERR_OFST ) //!< mask

#define MML_SCBR_BFSR_UNSTKERR_OFST			3 //!< offset
#define MML_SCBR_BFSR_UNSTKERR_MASK_NOOFST		0x1 //!< mask no offset
#define MML_SCBR_BFSR_UNSTKERR_MASK			( MML_SCBR_BFSR_UNSTKERR_NOOFST << MML_SCBR_BFSR_UNSTKERR_OFST ) //!< mask

#define MML_SCBR_BFSR_STKERR_OFST				4 //!< offset
#define MML_SCBR_BFSR_STKERR_MASK_NOOFST		0x1 //!< mask no offset
#define MML_SCBR_BFSR_STKERR_MASK				( MML_SCBR_BFSR_STKERR_NOOFST << MML_SCBR_BFSR_STKERR_OFST ) //!< mask

#define MML_SCBR_BFSR_BFARVALID_OFST			7 //!< offset
#define MML_SCBR_BFSR_BFARVALID_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_BFSR_BFARVALID_MASK			( MML_SCBR_BFSR_BFARVALID_NOOFST << MML_SCBR_BFSR_BFARVALID_OFST ) //!< mask


#define MML_SCBR_UFSR_UNDEFINSTR_OFST			0 //!< offset
#define MML_SCBR_UFSR_UNDEFINSTR_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_UFSR_UNDEFINSTR_MASK			( MML_SCBR_UFSR_UNDEFINSTR_NOOFST << MML_SCBR_UFSR_UNDEFINSTR_OFST ) //!< mask

#define MML_SCBR_UFSR_INVSTATE_OFST			1 //!< offset
#define MML_SCBR_UFSR_INVSTATE_MASK_NOOFST		0x1 //!< mask no offset
#define MML_SCBR_UFSR_INVSTATE_MASK			( MML_SCBR_UFSR_INVSTATE_NOOFST << MML_SCBR_UFSR_INVSTATE_OFST ) //!< mask

#define MML_SCBR_UFSR_INVPC_OFST				2 //!< offset
#define MML_SCBR_UFSR_INVPC_MASK_NOOFST		0x1 //!< mask no offset
#define MML_SCBR_UFSR_INVPC_MASK				( MML_SCBR_UFSR_INVPC_NOOFST << MML_SCBR_UFSR_INVPC_OFST ) //!< mask

#define MML_SCBR_UFSR_NOCP_OFST				3 //!< offset
#define MML_SCBR_UFSR_NOCP_MASK_NOOFST			0x1 //!< mask no offset
#define MML_SCBR_UFSR_NOCP_MASK				( MML_SCBR_UFSR_NOCP_NOOFST << MML_SCBR_UFSR_NOCP_OFST ) //!< mask

#define MML_SCBR_UFSR_UNALIGNED_OFST			8 //!< offset
#define MML_SCBR_UFSR_UNALIGNED_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_UFSR_UNALIGNED_MASK			( MML_SCBR_UFSR_UNALIGNED_NOOFST << MML_SCBR_UFSR_UNALIGNED_OFST ) //!< mask

#define MML_SCBR_UFSR_DIVBYZERO_OFST			9 //!< offset
#define MML_SCBR_UFSR_DIVBYZERO_MASK_NOOFST	0x1 //!< mask no offset
#define MML_SCBR_UFSR_DIVBYZERO_MASK			( MML_SCBR_UFSR_DIVBYZERO_NOOFST << MML_SCBR_UFSR_DIVBYZERO_OFST ) //!< mask
/** @} */ /* @defgroup MML_SCBR_REGS_CFSR */

/** @defgroup MML_SCBR_REGS_HFSR
 *
 * @{
 */
#define MML_SCBR_HFSR_OFST						0x0000002c //!< Register offset
#define MML_SCBR_HFSR_DFLT						0x00000000 //!< Register default value
/* Bits Fields */
#define MML_SCBR_HFSR_VECTTBL_OFST				1 //!< offset
#define MML_SCBR_HFSR_VECTTBL_MASK_NOOFST		0x1 //!< mask no offset
#define MML_SCBR_HFSR_VECTTBL_MASK				( MML_SCBR_HFSR_VECTTBL_NOOFST << MML_SCBR_HFSR_VECTTBL_OFST ) //!< mask

#define MML_SCBR_HFSR_FORCED_OFST				30 //!< offset
#define MML_SCBR_HFSR_FORCED_MASK_NOOFST		0x1 //!< mask no offset
#define MML_SCBR_HFSR_FORCED_MASK				( MML_SCBR_HFSR_FORCED_NOOFST << MML_SCBR_HFSR_FORCED_OFST ) //!< mask

#define MML_SCBR_HFSR_DEBUGEVT_OFST			31 //!< offset
#define MML_SCBR_HFSR_DEBUGEVT_MASK_NOOFST		0x1 //!< mask no offset
#define MML_SCBR_HFSR_DEBUGEVT_MASK			( MML_SCBR_HFSR_DEBUGEVT_NOOFST << MML_SCBR_HFSR_DEBUGEVT_OFST ) //!< mask
/** @} */ /* @defgroup MML_SCBR_REGS_HFSR */

/** @defgroup MML_SCBR_REGS_MMAR
 *
 * @{
 */
#define MML_SCBR_MMAR_OFST						0x00000030 //!< Register offset
#define MML_SCBR_MMAR_DFLT						0x00000000 //!< Register default value
/* Bits Fields */
#define MML_SCBR_MMAR_ADDR_OFST				0 //!< offset
#define MML_SCBR_MMAR_ADDR_MASK_NOOFST			0xffffffff //!< mask no offset
#define MML_SCBR_MMAR_ADDR_MASK				( MML_SCBR_MMAR_ADDR_NOOFST << MML_SCBR_MMAR_ADDR_OFST ) //!< mask
/** @} */ /* @defgroup MML_SCBR_REGS_MMAR */

/** @defgroup MML_SCBR_REGS_BFAR
 *
 * @{
 */
#define MML_SCBR_BFAR_OFST						0x00000034 //!< Register offset
#define MML_SCBR_BFAR_DFLT						0x00000000 //!< Register default value
/* Bits Fields */
#define MML_SCBR_BFAR_ADDR_OFST				0 //!< offset
#define MML_SCBR_BFAR_ADDR_MASK_NOOFST			0xffffffff //!< mask no offset
#define MML_SCBR_BFAR_ADDR_MASK				( MML_SCBR_BFAR_ADDR_NOOFST << MML_SCBR_BFAR_ADDR_OFST ) //!< mask
/** @} */ /* @defgroup MML_SCBR_REGS_BFAR */

/** @defgroup MML_SCBR_REGS_AFSR
 *
 * @{
 */
#define MML_SCBR_AFSR_OFST						0x00000038 //!< Register offset
#define MML_SCBR_AFSR_DFLT						0x00000000 //!< Register default value
/* Bits Fields */
#define MML_SCBR_AFSR_ADDR_OFST				0 //!< offset
#define MML_SCBR_AFSR_ADDR_MASK_NOOFST			0xffffffff //!< mask no offset
#define MML_SCBR_AFSR_ADDR_MASK				( MML_SCBR_AFSR_ADDR_NOOFST << MML_SCBR_AFSR_ADDR_OFST ) //!< mask
/** @} */ /* @defgroup MML_SCBR_REGS_AFSR */


#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */

/** SCBR Registers.
 *
 */
typedef volatile struct
{
    unsigned int								cpuid; //!< CPUID Base Register
    unsigned int								icsr; //!< Interrupt Control and State Register
    unsigned int								vtor; //!< vector Table Offset Register
    unsigned int								aircr; //!< Application Interrupt and Reset Control Register
    unsigned int								scr; //!< System Control Register
    unsigned int								ccr; //!< Configuration and Control Register
    unsigned int								shpr1; //!< System Handler Priority Register 1
    unsigned int								shpr2; //!< System Handler Priority Register 2
    unsigned int								shpr3; //!< System Handler Priority Register 3
    unsigned int								shcrs; //!< System Handler Control and State Register
    unsigned int								cfsr; //!< Configurable Fault Status Register
    unsigned int								hfsr; //!< HardFault Status Register
    unsigned int								mmar; //!< MemManage Fault Address Register
    unsigned int								bfar; //!< Bus Fault Address Register
    unsigned int								afsr; //!< Auxiliary Fault Status Register

} mml_scbr_regs_t;


#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_SCBR_REGS */

#endif /* _MML_SCBR_REGS_H_ */

/******************************************************************************/
/* EOF */
