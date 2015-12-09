/*
 * mml_rtc_regs.h --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2013, Maxim Integrated Products
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
 * Created on: Oct 25, 2013
 * Author: Yann G. <yann.gaude@maxim-ic.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_RTC_REGS_H_
#define _MML_RTC_REGS_H_

/** @file mml_rtc_regs.h RTC Registers Header */

/** @defgroup MML_RTC RTC Driver
 * @ingroup MML_DRIVER
 */

/** @defgroup MML_RTC_REGS RTC Registers
 *
 * @note MML Design Specification 1.0.0
 *
 * @ingroup MML_RTC
 * @{
 */

/* -------------------------------------------------------------------------- */
/** @defgroup MML_RTC_REGS_SEC RTC Second Counter Register (SEC)
 *
 * This register contains the 32-bit second counter.
 *
 * Read accessible when RDY=1; write accessible when RTCE=0 and BUSY=0.
 *
 * @note This register is battery backed. It is indeterminate following BOR but
 * is unaffected by other resets.
 *
 * @{
 */

#define MML_RTC_SEC_OFST	0x0 //!< SEC register offset
#define MML_RTC_SEC_DFLT	0x0	//!< SEC Register default value
/** @} */ /* @defgroup MML_RTC_REGS_SEC */

/* -------------------------------------------------------------------------- */
/** @defgroup MML_RTC_REGS_SSEC Sub-second Counter Register (SSEC)
 *
 * Read/Write Access: Read accessible when RDY=1, bits [3:0] are read only and
 * [3:1] always return 0 when read. Other bits are write accessible when
 * RTCE=0 and BUSY=0.
 *
 * @note This register is battery backed. It is indeterminate following BOR
 * but is unaffected by other resets.
 *
 * @li UIP
 *
 * . A ‘0’ in the UIP bit location indicates that the RTSS
 * register will not change within the next 15 usec. UIP is not altered by
 * software write.
 *
 * @li UIP[0]: Update In Progress
 * @li COUNT[1:15]: Sub-second Counter bits
 *
 * @{
 */

#define MML_RTC_SSEC_OFST	0x4	//!< SSEC Register offset
#define MML_RTC_SSEC_DFLT	0x0	//!< SSEC Register default value

#define MML_RTC_SSEC_RESOLUTION 244 //!< Resolution in us

/* Bits Fields */

#define MML_RTC_SSEC_UIP_OFST		0	//!< UIP offset
#define MML_RTC_SSEC_COUNT_OFST	1	//!< COUNT offset

#define MML_RTC_SSEC_UIP_MASK		0x1		//!< UIP mask
#define MML_RTC_SSEC_COUNT_MASK	0x4FFF	//!< COUNT mask

/** @} */ /* @defgroup MML_RTC_REGS_SSEC */


/* -------------------------------------------------------------------------- */
/** @defgroup MML_RTC_REGS_RAS Alarm Time-of-Day Register (RAS)
 * @{
 */

#define MML_RTC_RAS_OFST	0x8	//!< RAS Register offset
#define MML_RTC_RAS_DFLT	0x0	//!< RAS Register default value

/* Bits Fields */



/** @} */ /* @defgroup MML_RTC_REGS_RAS */

/* -------------------------------------------------------------------------- */
/** @defgroup MML_RTC_REGS_RSSA Sub-second Alarm Register (RSSA)
 * @{
 */

#define MML_RTC_RSSA_OFST	0xC	//!< RSSA Register offset
#define MML_RTC_RSSA_DFLT	0x0		//!< RSSA Register default value

/* Bits Fields */



/** @} */ /* @defgroup MML_RTC_REGS_RSSA */

/* -------------------------------------------------------------------------- */
/** @defgroup MML_RTC_REGS_CR Control Register (CR)
 *
 * @li RDY[4]: RTC Ready
 *
 * @{
 */

#define MML_RTC_CR_OFST	0x10	//!< CR Register offset
#define MML_RTC_CR_DFLT	0x0		//!< CR Register default value

/* Bits Fields */

/** RTC Enable (RTCE)
 *The RTCE is the Real Time Clock Enable bit. Setting this bit to logic 1
 *activates the clocking by allowing the divided clock to the ripple counters. Clearing this bit to logic
 *0 disables the clock. This bit is battery back and reset to 0 by BOR only
 */
#define MML_RTC_CR_RTCE_OFST 0
#define MML_RTC_CR_RTCE_MASK 0x1	//!< RTCE bits field mask

/** RTC Day of Alarm Enable (ADE)
 *
 */
#define MML_RTC_CR_ADE_OFST 1
#define MML_RTC_CR_ADE_MASK 0x1	//!< ADE bits field mask

/** RTC seconds of Alarm Enable (ASE)
 *
 */
#define MML_RTC_CR_ASE_OFST 2
#define MML_RTC_CR_ASE_MASK 0x1	//!< ASE bits field mask

/** RTC BUSY (BUSY)
 *
 */
#define MML_RTC_CR_BUSY_OFST 3
#define MML_RTC_CR_BUSY_MASK 0x1	//!< BUSY bits field mask

/** RTC Ready (RDY)
 * This bit is set to 1 by hardware when the RTC count registers update.
 * It can be cleared to 0 by software at any time. It will also be cleared to
 * 0 by hardware just prior to an update of the RTC count register. This bit
 * can generate an interrupt if the RDYE bit is set to 1.
 */
#define MML_RTC_CR_RDY_OFST 4
#define MML_RTC_CR_RDY_MASK 0x1	//!< RDY bits field mask

/** RTC Ready Enable (RDYE).
 * Setting this bit to 1 allows a system interrupt to be generated
 * when RDY becomes active (if interrupts are enabled globally and
 * modularly). Clearing this bit to 0 disables the RDY interrupt.
 */
#define MML_RTC_CR_RDYE_OFST 5
#define MML_RTC_CR_RDYE_MASK 0x1	//!< RDY bits field mask

/** Alarm Time-of-Day Flag (ALDF).
 * This bit is set when the contents of RTSH and RTSL counter
 * registers match the 20-bit value in the RASH and RASL
 * alarm registers. Setting the ALDF will cause an interrupt
 * request to the CPU if the ADE is set and interrupt is
 * allowed at the system level.
 */
#define MML_RTC_CR_ALDF_OFST 6
#define MML_RTC_CR_ALDF_MASK 0x1	//!< RDY bits field mask

/** Alarm Sub-second Flag (ALSF).
 * This bit is set when the sub-second timer has been reloaded by
 * the RSSA register. Setting the ALSF will cause an interrupt
 * request to the CPU if the ASE is set and interrupt is allowed at
 * the system level.
 */

#define MML_RTC_CR_ALSF_OFST 7
#define MML_RTC_CR_ALSF_MASK 0x1	//!< RDY bits field mask

/** RTC 32KHz Oscillator Mode (X32KMD).
 * 00: Always operate in Noise Immune Mode. Oscillator warm-up required.
 * 01: Always operate in Quiet Mode. No oscillator warm-up required.
 * 10: Operate in Noise Immune Mode normally, switch to Quiet Mode on Stop Mode entry. Will wait for 32K oscillator warm-up before code execution on Stop Mode exit.
 * 11: Operate in Noise Immune Mode normally, switch to Quiet Mode on Stop Mode entry. Will not wait for 32K oscillator warm-up before code execution on Stop Mode exit.
 */
#define MML_RTC_CR_X32KMD_OFST 11
#define MML_RTC_CR_X32KMD_MASK 0x3	//!< X32KMD bits field mask
#define MML_RTC_CR_X32KMD_QUIET 0x1 //!< X32KMD quiet mode

/** RTC Write Enable (WE).
 *This register bit serves as a protection mechanism against undesirable writes
 *to the critical register bits. This bit must be set to a �1� in order to give write access to the RTRM
 *register and the RTCE and X32D bits; otherwise (when WE bit=0) these protected bits are read
 *only.
 */
#define MML_RTC_CR_WE_OFST 15
#define MML_RTC_CR_WE_MASK 0x1	//!< WE bits field mask

/** @} */ /* @defgroup MML_RTC_REGS_CR */

/* -------------------------------------------------------------------------- */
/** @defgroup MML_RTC_REGS_TMR Trim Register (TMR)
 * @{
 */

#define MML_RTC_TMR_OFST	0x14	//!< TMR Register offset
#define MML_RTC_TMR_DFLT	0x0		//!< TMR Register default value

/* Bits Fields */

/** @} */ /* @defgroup MML_RTC_REGS_TRIM */


#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */
/** RTC Registers.
 *
 */
typedef volatile struct
{
	unsigned sec;	//!< Second Counter Register
	unsigned ssec;	//!< Sub-second Counter Register
	unsigned ras;	//!< Alarm Time-of-Day Register
	unsigned rssa;	//!< Sub-second Alarm Register
	unsigned cr;	//!< Control Register
	unsigned trm;	//!< Trim Register

} mml_rtc_regs_t;

#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_RTC_REGS */

#endif /* _MML_RTC_REGS_H_ */

/******************************************************************************/
/* EOF */
