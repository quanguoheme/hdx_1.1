/*
 * mml_smon_regs.h --
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
 * Created on: Jun 16, 2014
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_SMON_REGS_H_
#define _MML_SMON_REGS_H_

/** @file mml_smon_regs.h SMON Registers Header */

/** @defgroup MML_SMON SMON Driver */

/** @defgroup MML_SMON_REGS SMON Registers
 *
 * @note SMON IP or Specification version number
 *
 * @ingroup MML_SMON
 * @{
 */
/** @defgroup MML_SMON_REGS_EXTSCR External Sensors Control Register
 *
 * @li EXT_EN [7:0] - R/W: These bits are used to enable external sensors.
 * Each bit corresponds to a dynamic sensor input/output pair. If an external
 * sensor is not enabled; the corresponding EXTSAT and DYNF bits are not updated.
 * @li ERRCNT [20:16] - R/W: These bits are used to define error counter
 * value. An alarm will be generated if the error number is upper than the
 * error counter value. One error is generated if ext_sens_in_x ≠ ext_sens_out_x
 * @li EXTFRQ [23:21] - R/W: These bits are used to define the external sensors
 * output frequency on ext_sens_out_x PADs. Customer will use these bits in
 * order to find the best tradeoff between security and consumption.
 * @li DIVCLK [26:24] - R/W: These bits are used to divide the module frequency.
 * The reference clock is the internal 8kHz clock.
 * @li LOCK [31] - R/S: This bit is used to lock the EXTSCR control register.
 * The lock bit acts for all the register bits. After locking, it is no more
 * possible to change the external sensor configuration. After locking the
 * lock bit cannot be unlocked. Only a battery disconnect will erase these bits.
 * VBAT will be used to supply this IP.
 *
 * @ingroup MML_SMON_REGS
 * @{
 */

#define MML_SMON_EXTSCN_OFST					0x00000000 //!< EXTSCN Register offset
#define MML_SMON_EXTSCN_DFLT					0x00000000 //!< EXTSCN Register default value

/* Bits Fields */
#define MML_SMON_EXTSCN_EXTS_EN_OFST			0
#define MML_SMON_EXTSCN_EXTS_EN_MASK_NOOFST		0x3f //!< Mask no offset
#define	MML_SMON_EXTSCN_EXTS_EN_MASK			( MML_SMON_EXTSCN_EXTS_EN_MASK_NOOFST << MML_SMON_EXTSCN_EXTS_EN_OFST )

#define MML_SMON_EXTSCN_EXTCNT_OFST				16
#define MML_SMON_EXTSCN_EXTCNT_MASK_NOOFST		0x1f //!< Mask no offset
#define	MML_SMON_EXTSCN_EXTCNT_MASK				( MML_SMON_EXTSCN_EXTCNT_MASK_NOOFST << MML_SMON_EXTSCN_EXTCNT_OFST )

#define MML_SMON_EXTSCN_EXTFRQ_OFST				21
#define MML_SMON_EXTSCN_EXTFRQ_MASK_NOOFST		0x7 //!< Mask no offset
#define	MML_SMON_EXTSCN_EXTFRQ_MASK				( MML_SMON_EXTSCN_EXTFRQ_MASK_NOOFST << MML_SMON_EXTSCN_EXTFRQ_OFST )

#define MML_SMON_EXTSCN_DIVCLK_OFST				24
#define MML_SMON_EXTSCN_DIVCLK_MASK_NOOFST		0x3 //!< Mask no offset
#define	MML_SMON_EXTSCN_DIVCLK_MASK				( MML_SMON_EXTSCN_DIVCLK_MASK_NOOFST << MML_SMON_EXTSCN_DIVCLK_OFST )

#define MML_SMON_EXTSCN_BUSY_OFST				30
#define MML_SMON_EXTSCN_BUSY_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_EXTSCN_BUSY_MASK				( MML_SMON_EXTSCN_BUSY_MASK_NOOFST << MML_SMON_EXTSCN_BUSY_OFST )

#define MML_SMON_EXTSCN_LOCK_OFST				31
#define MML_SMON_EXTSCN_LOCK_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_EXTSCN_LOCK_MASK				( MML_SMON_EXTSCN_LOCK_MASK_NOOFST << MML_SMON_EXTSCN_LOCK_OFST )
/** @} */

/** @defgroup MML_SMON_REGS_INTSCN SMON Internal Sensors Control Register
 * This register is used to configure the internal sensors.
 * This register is battery backed. It is reset by the BOR, but is unaffected
 * by other resets.
 * @li EXT_EN [7:0] - R/W:
 * @{
 */
#define MML_SMON_INTSCN_OFST					0x00000004 //!< INTSCN Register offset
#define MML_SMON_INTSCN_DFLT					0x00000000 //!< INTSCN Register default value

/* Bits Fields */
#define MML_SMON_INTSCN_SHIELD_EN_OFST			0
#define MML_SMON_INTSCN_SHIELD_EN_MASK_NOOFST	0x1 //!< Mask no offset
#define	MML_SMON_INTSCN_SHIELD_EN_MASK			( MML_SMON_INTSCN_SHIELD_EN_MASK_NOOFST << MML_SMON_INTSCN_SHIELD_EN_OFST )

#define MML_SMON_INTSCN_TEMP_EN_OFST			1
#define MML_SMON_INTSCN_TEMP_EN_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_INTSCN_TEMP_EN_MASK			( MML_SMON_INTSCN_TEMP_EN_MASK_NOOFST << MML_SMON_INTSCN_TEMP_EN_OFST )

#define MML_SMON_INTSCN_VBAT_EN_OFST			2
#define MML_SMON_INTSCN_VBAT_EN_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_INTSCN_VBAT_EN_MASK			( MML_SMON_INTSCN_VBAT_EN_MASK_NOOFST << MML_SMON_INTSCN_VBAT_EN_OFST )

#define MML_SMON_INTSCN_LOTEMP_SEL_OFST			16
#define MML_SMON_INTSCN_LOTEMP_SEL_MASK_NOOFST	0x1 //!< Mask no offset
#define	MML_SMON_INTSCN_LOTEMP_SEL_MASK			( MML_SMON_INTSCN_LOTEMP_SEL_MASK_NOOFST << MML_SMON_INTSCN_LOTEMP_SEL_OFST )

#define MML_SMON_INTSCN_VCORELOEN_OFST			18
#define MML_SMON_INTSCN_VCORELOEN_MASK_NOOFST	0x1 //!< Mask no offset
#define	MML_SMON_INTSCN_VCORELOEN_MASK			( MML_SMON_INTSCN_VCORELOEN_MASK_NOOFST << MML_SMON_INTSCN_VCORELOEN_OFST )

#define MML_SMON_INTSCN_VCOREHIEN_OFST			19
#define MML_SMON_INTSCN_VCOREHIEN_MASK_NOOFST	0x1 //!< Mask no offset
#define	MML_SMON_INTSCN_VCOREHIEN_MASK			( MML_SMON_INTSCN_VCOREHIEN_MASK_NOOFST << MML_SMON_INTSCN_VCOREHIEN_OFST )

#define MML_SMON_INTSCN_VIOLOEN_OFST			20
#define MML_SMON_INTSCN_VIOLOEN_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_INTSCN_VIOLOEN_MASK			( MML_SMON_INTSCN_VIOLOEN_MASK_NOOFST << MML_SMON_INTSCN_VIOLOEN_OFST )

#define MML_SMON_INTSCN_VIOHIEN_OFST			21
#define MML_SMON_INTSCN_VIOHIEN_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_INTSCN_VIOHIEN_MASK			( MML_SMON_INTSCN_VIOHIEN_MASK_NOOFST << MML_SMON_INTSCN_VIOHIEN_OFST )

#define MML_SMON_INTSCN_VGLEN_OFST				22
#define MML_SMON_INTSCN_VGLEN_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_INTSCN_VGLEN_MASK				( MML_SMON_INTSCN_VGLEN_MASK_NOOFST << MML_SMON_INTSCN_VGLEN_OFST )

#define MML_SMON_INTSCN_LOCK_OFST				31
#define MML_SMON_INTSCN_LOCK_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_INTSCN_LOCK_MASK				( MML_SMON_INTSCN_LOCK_MASK_NOOFST << MML_SMON_INTSCN_LOCK_OFST )
/** @} */

/** @defgroup MML_SMON_REGS_SECALM SMON Security Block Alarm Register
 * This register is battery backed. It is reset by the BOR, but is unaffected
 * by other resets. These bits are set by hardware on detection of a DRS and
 * must be cleared by software.
 * @li EXT_EN [0] - R/W: DRS This bit is used by software to initiate a DRS.
 * This bit is never set by hardware and must also be cleared by software
 * after the soft DRS has been initiated. This bit can be used to trigger a
 * software DRS. This bit is not impacted by the ESCR[lock] bit
 * @li EXT_EN [1] - R/W: KEYWIPE Key Wipe trigger, this bit is used by
 * software to initiate a wipe of the secure key register and any other key
 * registers that are normally cleared by a DRS event. It does not reset the
 * part, or log a timestamp, it only cleans the key registers. This bit is
 * self clearing.
 * @li EXT_EN [2] - R/W: SHIELDF Die shield flag. This bit indicates the die
 *  grid has detected a fault.
 * @li EXT_EN [3] - R/W: LOWTEMP Low Temperature Detect. This bit indicates
 * the chip temperature is under the configured low temperature threshold.
 * @li EXT_EN [4] - R/W: HITEMP High Temperature Detect. This bit indicates
 * the chip temperature is over the configured high temperature threshold.
 * @li EXT_EN [5] - R/W: BATLO Battery undervoltage Detect. This bit indicates
 *  the battery voltage is under the low voltage threshold.
 * @li EXT_EN [6] - R/W: BATHI Battery overvoltage Detect. This bit indicates
 * the battery voltage is over the high voltage threshold.
 * @li EXT_EN [7] - R/W: DYNF Dynamic sensor flag. This bit indicates a fault
 * has been detected on one of the external dynamic sensors. Which sensor
 * triggered the fault is contained in the EXTSTAT bits.
 * @li EXT_EN [15:8] - R: RFU
 * @li EXT_EN [23:16] - R/W: EXTSTAT[7:0] External sensor detect. These bits
 * indicated an external temper detect has been detected. The DTD input output
 * pair that has triggered the DRS event is indicated by bit that is asserted
 * (one-hot, so ETD.0 indicates for DTD0 …)
 * @li EXT_EN [31:24] - R: RFU
 * @{
 */
#define MML_SMON_SECALM_OFST					0x00000008 //!< SECALM Register offset
#define MML_SMON_SECALM_DFLT					0x00000000 //!< SECALM Register default value

/* Bits Fields */
#define MML_SMON_SECALM_DRS_OFST				0
#define MML_SMON_SECALM_DRS_MASK_NOOFST			0x1 //!< Mask no offset
#define	MML_SMON_SECALM_DRS_MASK				( MML_SMON_SECALM_DRS_MASK_NOOFST << MML_SMON_SECALM_DRS_OFST )

#define MML_SMON_SECALM_KEYWIPE_OFST			1
#define MML_SMON_SECALM_KEYWIPE_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECALM_KEYWIPE_MASK			( MML_SMON_SECALM_KEYWIPE_MASK_NOOFST << MML_SMON_SECALM_KEYWIPE_OFST )

#define MML_SMON_SECALM_SHIELDF_OFST			2
#define MML_SMON_SECALM_SHIELDF_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECALM_SHIELDF_MASK			( MML_SMON_SECALM_SHIELDF_MASK_NOOFST << MML_SMON_SECALM_SHIELDF_OFST )

#define MML_SMON_SECALM_LOTEMP_OFST				3
#define MML_SMON_SECALM_LOTEMP_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECALM_LOTEMP_MASK				( MML_SMON_SECALM_LOTEMP_MASK_NOOFST << MML_SMON_SECALM_LOTEMP_OFST )

#define MML_SMON_SECALM_HITEMP_OFST				4
#define MML_SMON_SECALM_HITEMP_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECALM_HITEMP_MASK				( MML_SMON_SECALM_HITEMP_MASK_NOOFST << MML_SMON_SECALM_HITEMP_OFST )

#define MML_SMON_SECALM_BATLO_OFST				5
#define MML_SMON_SECALM_BATLO_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECALM_BATLO_MASK				( MML_SMON_SECALM_BATLO_MASK_NOOFST << MML_SMON_SECALM_BATLO_OFST )

#define MML_SMON_SECALM_BATHI_OFST				6
#define MML_SMON_SECALM_BATHI_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECALM_BATHI_MASK				( MML_SMON_SECALM_BATHI_MASK_NOOFST << MML_SMON_SECALM_BATHI_OFST )

#define MML_SMON_SECALM_EXTF_OFST				7
#define MML_SMON_SECALM_EXTF_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECALM_EXTF_MASK				( MML_SMON_SECALM_EXTF_MASK_NOOFST << MML_SMON_SECALM_EXTF_OFST )

#define MML_SMON_SECALM_VDDLO_OFST				8
#define MML_SMON_SECALM_VDDLO_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECALM_VDDLO_MASK				( MML_SMON_SECALM_VDDLO_MASK_NOOFST << MML_SMON_SECALM_VDDLO_OFST )

#define MML_SMON_SECALM_VCORELO_OFST			9
#define MML_SMON_SECALM_VCORELO_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECALM_VCORELO_MASK			( MML_SMON_SECALM_VCORELO_MASK_NOOFST << MML_SMON_SECALM_VCORELO_OFST )

#define MML_SMON_SECALM_VCOREHI_OFST			10
#define MML_SMON_SECALM_VCOREHI_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECALM_VCOREHI_MASK			( MML_SMON_SECALM_VCOREHI_MASK_NOOFST << MML_SMON_SECALM_VCOREHI_OFST )

#define MML_SMON_SECALM_VDDHI_OFST				11
#define MML_SMON_SECALM_VDDHI_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECALM_VDDHI_MASK				( MML_SMON_SECALM_VDDHI_MASK_NOOFST << MML_SMON_SECALM_VDDHI_OFST )

#define MML_SMON_SECALM_VGL_OFST				12
#define MML_SMON_SECALM_VGL_MASK_NOOFST			0x1 //!< Mask no offset
#define	MML_SMON_SECALM_VGL_MASK				( MML_SMON_SECALM_VGL_MASK_NOOFST << MML_SMON_SECALM_VGL_OFST )

#define MML_SMON_SECALM_EXSTAT_OFST				16
#define MML_SMON_SECALM_EXSTAT_MASK_NOOFST		0x3f //!< Mask no offset
#define	MML_SMON_SECALM_EXSTAT_MASK				( MML_SMON_SECALM_EXSTAT_MASK_NOOFST << MML_SMON_SECALM_EXSTAT_OFST )

#define MML_SMON_SECALM_EXTSWARN_OFST			24
#define MML_SMON_SECALM_EXTSWARN_MASK_NOOFST	0x3f //!< Mask no offset
#define	MML_SMON_SECALM_EXTSWARN_MASK			( MML_SMON_SECALM_EXTSWARN_MASK_NOOFST << MML_SMON_SECALM_EXTSWARN_OFST )
/** @} */

/** @defgroup MML_SMON_REGS_SECDIAG SMON Security Block Diagnostic Register
 *
 * @li EXT_EN [1:0] - R: RFU
 * @li EXT_EN [2] - R: SHIELDF Die shield flag. This bit indicates the die
 * grid has detected a fault.
 * @li EXT_EN [3] - R: LOWTEMP Low Temperature Detect. This bit indicates the
 * chip temperature is under the configured low temperature threshold.
 * @li EXT_EN [4] - R: HITEMP High Temperature Detect. This bit indicates the
 * chip temperature is over the configured high temperature threshold.
 * @li EXT_EN [5] - R: BATLO Battery undervoltage Detect. This bit indicates
 * the battery voltage is under the low voltage threshold.
 * @li EXT_EN [6] - R: BATHI Battery overvoltage Detect. This bit indicates
 * the battery voltage is over the high voltage threshold.
 * @li EXT_EN [7] - R: DYNF Dynamic sensor flag. This bit indicates a fault has
 * been detected on one of the external dynamic sensors. Which sensor
 * triggered the fault is contained in the EXTSTAT bits.
 * @li EXT_EN [15:8] - R: RFU
 * @li EXT_EN [23:16] - R: EXTSTAT External sensor detect. These bits indicated
 * an external temper detect has been detected. The DTD input output pair that
 * has triggered the DRS event is indicated by bit that is asserted (one-hot,
 * so ETD.0 indicates for DTD0 …)
 * @li EXT_EN [31:24] - R: RFU
 * @{
 */
#define MML_SMON_SECDIAG_OFST					0x0000000c //!< SECDIAG Register offset
#define MML_SMON_SECDIAG_DFLT					0x00000000 //!< SECDIAG Register default value

/* Bits Fields */
#define MML_SMON_SECDIAG_SHIELDF_OFST			2
#define MML_SMON_SECDIAG_SHIELDF_MASK_NOOFST	0x1 //!< Mask no offset
#define	MML_SMON_SECDIAG_SHIELDF_MASK			( MML_SMON_SECDIAG_SHIELDF_MASK_NOOFST << MML_SMON_SECDIAG_SHIELDF_OFST )

#define MML_SMON_SECDIAG_LOTEMP_OFST			3
#define MML_SMON_SECDIAG_LOTEMP_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECDIAG_LOTEMP_MASK			( MML_SMON_SECDIAG_LOTEMP_MASK_NOOFST << MML_SMON_SECDIAG_LOTEMP_OFST )

#define MML_SMON_SECDIAG_HITEMP_OFST			4
#define MML_SMON_SECDIAG_HITEMP_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECDIAG_HITEMP_MASK			( MML_SMON_SECDIAG_HITEMP_MASK_NOOFST << MML_SMON_SECDIAG_HITEMP_OFST )

#define MML_SMON_SECDIAG_BATLO_OFST				5
#define MML_SMON_SECDIAG_BATLO_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECDIAG_BATLO_MASK				( MML_SMON_SECDIAG_BATLO_MASK_NOOFST << MML_SMON_SECDIAG_BATLO_OFST )

#define MML_SMON_SECDIAG_BATHI_OFST				6
#define MML_SMON_SECDIAG_BATHI_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECDIAG_BATHI_MASK				( MML_SMON_SECDIAG_BATHI_MASK_NOOFST << MML_SMON_SECDIAG_BATHI_OFST )

#define MML_SMON_SECDIAG_DYNF_OFST				7
#define MML_SMON_SECDIAG_DYNF_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECDIAG_DYNF_MASK				( MML_SMON_SECDIAG_DYNF_MASK_NOOFST << MML_SMON_SECDIAG_DYNF_OFST )

#define MML_SMON_SECDIAG_AESKT_OFST				8
#define MML_SMON_SECDIAG_AESKT_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECDIAG_AESKT_MASK				( MML_SMON_SECDIAG_AESKT_MASK_NOOFST << MML_SMON_SECDIAG_AESKT_OFST )

#define MML_SMON_SECDIAG_PARK_OFST				9
#define MML_SMON_SECDIAG_PARK_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECDIAG_PARK_MASK				( MML_SMON_SECDIAG_PARK_MASK_NOOFST << MML_SMON_SECDIAG_PARK_OFST )

#define MML_SMON_SECDIAG_EXSTAT_OFST			16
#define MML_SMON_SECDIAG_EXSTAT_MASK_NOOFST		0x3f //!< Mask no offset
#define	MML_SMON_SECDIAG_EXSTAT_MASK			( MML_SMON_SECDIAG_EXSTAT_MASK_NOOFST << MML_SMON_SECDIAG_EXSTAT_OFST )
/** @} */

/** @defgroup MML_SMON_REGS_DLRTC SMON DRS Log RTC Register
 * @li EXT_EN [31:0] - R: These bits contain the 32 Bit value in the RTC second
 * register when the last DRS event occurred.
 * @{
 */
#define MML_SMON_DLRTC_OFST						0x00000010 //!< DLRTC Register offset
#define MML_SMON_DLRTC_DFLT						0x00000000 //!< DLRTC Register default value

/* Bits Fields */
#define MML_SMON_DLRTC_DLRTC_OFST				0
#define MML_SMON_DLRTC_DLRTC_MASK_NOOFST		0xffffffff //!< Mask no offset
#define	MML_SMON_DLRTC_DLRTC_MASK				( MML_SMON_DLRTC_DLRTC_MASK_NOOFST << MML_SMON_DLRTC_DLRTC_OFST )
/** @} */


/** @defgroup MML_SMON_REGS_SECST SMON
 *
 * @{
 */
#define MML_SMON_SECST_OFST						0x00000014 //!< DLRTC Register offset
#define MML_SMON_SECST_DFLT						0x00000000 //!< DLRTC Register default value

/* Bits Fields */
#define MML_SMON_SECST_EXTSRS_OFST				0
#define MML_SMON_SECST_EXTSRS_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECST_EXTSRS_MASK				( MML_SMON_SECST_EXTSRS_MASK_NOOFST << MML_SMON_SECST_EXTSRS_OFST )

#define MML_SMON_SECST_INTSRS_OFST				1
#define MML_SMON_SECST_INTSRS_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECST_INTSRS_MASK				( MML_SMON_SECST_INTSRS_MASK_NOOFST << MML_SMON_SECST_INTSRS_OFST )

#define MML_SMON_SECST_SECALRS_OFST				2
#define MML_SMON_SECST_SECALRS_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECST_SECALRS_MASK				( MML_SMON_SECST_SECALRS_MASK_NOOFST << MML_SMON_SECST_SECALRS_OFST )

#define MML_SMON_SECST_KET0RS_OFST				3
#define MML_SMON_SECST_KET0RS_MASK_NOOFST		0x1 //!< Mask no offset
#define	MML_SMON_SECST_KET0RS_MASK				( MML_SMON_SECST_KET0RS_MASK_NOOFST << MML_SMON_SECST_KET0RS_OFST )
/** @} */



/** @} */ /* @defgroup MML_SMON_REGS_BR */


#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */
/** SMON Registers.
 *
 */
typedef volatile struct
{
	/** EXTSCN Register - 0x0000 */
	volatile unsigned int						extscn;
	/** INTSCN Register - 0x0004 */
	volatile unsigned int						intscn;
	/** SECALM Register - 0x0008 */
	volatile unsigned int						secalm;
	/** SECDIAG Register - 0x000c */
	volatile unsigned int						secdiag;
	/** DLRTC Register - 0x0010 */
	volatile unsigned int						dlrtc;
	/** RFU - 0x0014 - 30 */
	volatile unsigned int						rfu0[8];
	/** SECST Register - 0x0034 */
	volatile unsigned int						secst;

} mml_smon_regs_t;

#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_SMON_REGS */

#endif /* _MML_SMON_REGS_H_ */

/******************************************************************************/
/* EOF */
