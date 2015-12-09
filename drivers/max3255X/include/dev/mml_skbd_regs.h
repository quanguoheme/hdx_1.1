/*
 * mml_skbd_regs.h --
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
 * Author: Dayananda HB. <dayananda.HB@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_SKBD_REGS_H_
#define _MML_SKBD_REGS_H_

/** @file mml_skbd_regs.h SKBD Registers Header */

/** @defgroup MML_SKBD SKBD Driver
 * @ingroup MML_DRIVER
 */

/** @defgroup MML_SKBD_REGS SKBD Registers
 *
 * @note SKBD IP or Specification version number
 *
 * @ingroup MML_SKBD
 * @{
 */

/** @defgroup MML_SKBD_REGS_CR0 Control Register (CR0)
 *
 * @li IOSEL [15:0] - R/W: KBD IOs Selection mode
 *
 * @{
 */
#define MML_SKBD_CR0_OFST						0x0 //!< Register offset
#define MML_SKBD_CR0_DFLT						0x0 //!< Register default value

/** Bits Fields */
#define MML_SKBD_CR0_IOSEL_OFST					0 //!< IOSEL offset
#define MML_SKBD_CR0_IOSEL_MASK_NOOFST			0xffff //!< IOSEL mask no offset
#define MML_SKBD_CR0_IOSEL_MASK					( MML_SKBD_CR0_IOSEL_MASK_NOOFST << MML_SKBD_CR0_IOSEL_OFST ) //!< IOSEL mask

/** We use the term enabled/disabled for bits field of size 1 */
#define MML_SKBD_CR0_IN_CFG(_n_)				( 0x0 << _n_ ) //!< IOn as Input
#define MML_SKBD_AR_OUT_CFG(_n_)				( 0x1 << _n_ ) //!< IOn as Output
#define MML_SKBD_PIN_CFG(_n_)					( 0x1 << _n_ )

/** @} */ /* @defgroup MML_SKBD_REGS_CR0 */

/** @defgroup MML_SKBD_REGS_CR1 Control Register (CR1)
 *
 * @li AUTOEN 	[0] 	- R/W :	Automatic Mode Selection
 * @li CLEAR  	[1]		- R/W :	Automatic Erase Mode Enable
 * @li CLKDIS   [2]     - R/W : Clock Disable
 * @li RFU1		[7:3]	-  R  :	RFU
 * @li OUTNB	[11:8]	- R/W :	Number of Last Output IO
 * @li RFU2		[12]	-  R  :	RFU
 * @li DBTM		[15:13]	- R/W :	Debouncing Time
 *
 * @{
 */
#define MML_SKBD_CR1_OFST						0x4 //!< Register offset
#define MML_SKBD_CR1_DFLT						0x0 //!< Register default value

/** Bits Fields */
#define MML_SKBD_CR1_AUTOEN_OFST				0 //!< AUTOEN offset
#define MML_SKBD_CR1_AUTOEN_MASK_NOOFST			0x1 //!< AUTOEN mask no offset
#define MML_SKBD_CR1_AUTOEN_MASK				( MML_SKBD_CR1_AUTOEN_MASK_NOOFST << MML_SKBD_CR1_AUTOEN_OFST ) //!< AUTOEN mask

#define MML_SKBD_CR1_CLEAR_OFST					1 //!< CLEAR offset
#define MML_SKBD_CR1_CLEAR_MASK_NOOFST			0x1
#define MML_SKBD_CR1_CLEAR_MASK					( MML_SKBD_CR1_CLEAR_MASK_NOOFST << MML_SKBD_CR1_CLEAR_OFST )

#define MML_SKBD_CR1_CLKDIS_OFST				2 //!< RFU1 offset

#define MML_SKBD_CR1_OUTNB_OFST					8 //!< OUTNB offset
#define MML_SKBD_CR1_OUTNB_MASK_NOOFST			0xf
#define MML_SKBD_CR1_OUTNB_MASK					( MML_SKBD_CR1_OUTNB_MASK_NOOFST << MML_SKBD_CR1_OUTNB_OFST )

#define MML_SKBD_CR1_RFU2_OFST					12 //!< RFU2 offset
#define MML_SKBD_CR1_DBTM_OFST					13 //!< DBTM offset

/** Enumeration of the different value for a bits field . */
#define MML_SKBD_CR1_AUTOEN_AUTO				1 //!< Automatic mode
#define MML_SKBD_CR1_AUTOEN_GPIO				0 //!< GPIO mode

#define MML_SKBD_CR1_CLEAR_AUTO					1 //!< Automatic mode

#define MML_SKBD_CR1_DBTM_10MS					0 //!< Debouncing Time = 10ms
#define MML_SKBD_CR1_DBTM_13MS					1 //!< Debouncing Time = 12.85ms
#define MML_SKBD_CR1_DBTM_15MS					2 //!< Debouncing Time = 15.7ms
#define MML_SKBD_CR1_DBTM_18MS					3 //!< Debouncing Time = 18.55ms
#define MML_SKBD_CR1_DBTM_21MS					4 //!< Debouncing Time = 21.4ms
#define MML_SKBD_CR1_DBTM_24MS					5 //!< Debouncing Time = 24.25ms
#define MML_SKBD_CR1_DBTM_27MS					6 //!< Debouncing Time = 27.1ms
#define MML_SKBD_CR1_DBTM_30MS					7 //!< Debouncing Time = 30ms

/** @} */ /* @defgroup MML_SKBD_REGS_CR1 */

/** @defgroup MML_SKBD_REGS_SR Status Register (SR)
 *
 * @li BUSY [0] - R: KBD Interface status
 *
 * @{
 */
#define MML_SKBD_SR_OFST						0x8 //!< Register offset
#define MML_SKBD_SR_DFLT						0x0 //!< Register default value

/** Bits Fields */
#define MML_SKBD_SR_BUSY_OFST					0 //!< BUSY offset
#define MML_SKBD_SR_BUSY_MASK_NOOFST			0x1 //!< BUSY mask no offset
#define MML_SKBD_SR_BUSY_MASK					( MML_SKBD_SR_BUSY_MASK_NOOFST << MML_SKBD_SR_BUSY_OFST ) //!< BUSY mask

/** Enumeration of the different value for a bits field . We recommend to use
 * human readable value. */
#define MML_SKBD_SR_BUSY						1 //!< Keyboard is running with automatic mode
#define MML_SKBD_SR_IDLE						0 //!< Keyboard is idle

/** @} */ /* @defgroup MML_SKBD_REGS_SR */

/** @defgroup MML_SKBD_REGS_IER Interrupt Enable Register (IER)
 *
 * @li PUSHIE 		[0] 	- R/W :	Enable Interrupt on key push event
 * @li RELEASEIE  	[1]		- R/W :	Enable Interrupt on key release event
 * @li OVERIE		[2]		- R/W :	Enable Interrupt on key overrun event
 * @li GPIOIE		[3]		- R/W :	Enable Interrupt on input change (GPIO mode only)
 * @li RFU1			[31:4]	-  R  :	RFU1
 *
 * @{
 */
#define MML_SKBD_IER_OFST						0xc //!< Register offset
#define MML_SKBD_IER_DFLT						0x0 //!< Register default value

/** Bits Fields */
#define MML_SKBD_IER_PUSHIE_OFST				0 //!< PUSHIE offset
#define MML_SKBD_IER_PUSHIE_MASK_NOOFST			0x1 //!< PUSHIE Interrupt Enable no offset
#define MML_SKBD_IER_PUSHIE_MASK				( MML_SKBD_IER_PUSHIE_MASK_NOOFST << MML_SKBD_IER_PUSHIE_OFST ) //!< PUSHIE Interrupt Enable

#define MML_SKBD_IER_RELEASEIE_OFST				1 //!< RELEASEIE offset
#define MML_SKBD_IER_RELEASEIE_MASK_NOOFST		0x1 //!< RELEASEIE Interrupt Enable no offset
#define MML_SKBD_IER_RELEASEIE_MASK				( MML_SKBD_IER_RELEASEIE_MASK_NOOFST << MML_SKBD_IER_RELEASEIE_OFST ) //!< RELEASEIE Interrupt Enable

#define MML_SKBD_IER_OVERIE_OFST				2 //!< OVERIE offset
#define MML_SKBD_IER_OVERIE_MASK_NOOFST			0x1 //!< OVERIE Interrupt Enable no offset
#define MML_SKBD_IER_OVERIE_MASK				( MML_SKBD_IER_OVERIE_MASK_NOOFST << MML_SKBD_IER_OVERIE_OFST ) //!< OVERIE Interrupt Enable

#define MML_SKBD_IER_GPIOIE_OFST				3 //!< GPIOIE offset
#define MML_SKBD_IER_GPIOIE_MASK_NOOFST			0x1 //!< GPIOIE Interrupt Enable no offset
#define MML_SKBD_IER_GPIOIE_MASK				( MML_SKBD_IER_GPIOIE_MASK_NOOFST << MML_SKBD_IER_GPIOIE_OFST) //!< GPIOIE Interrupt Enable

/** @} */ /* @defgroup MML_SKBD_REGS_IER */

/** @defgroup MML_SKBD_REGS_ISR Interrupt Enable Register (IER)
 *
 * @li PUSHIES 		[0] 	- R/C :	Interrupt on key push event status
 * @li RELEASEIS  	[1]		- R/C :	Interrupt on key release event status
 * @li OVERIS		[2]		- R/C :	Interrupt on key overrun event status
 * @li GPIOIS		[3]		- R/C :	Interrupt on input change status (GPIO mode only)
 * @li RFU1			[31:4]	-  R  :	RFU1
 *
 * @{
 */
#define MML_SKBD_ISR_OFST						0x10 //!< Register offset
#define MML_SKBD_ISR_DFLT						0x0 //!< Register default value

/** Bits Fields */
#define MML_SKBD_ISR_PUSHIS_OFST				0 //!< PUSHIS offset
#define	MML_SKBD_ISR_PUSHIS_MASK_NOOFST			0x01
#define	MML_SKBD_ISR_PUSHIS_MASK				( MML_SKBD_ISR_PUSHIS_MASK_NOOFST << MML_SKBD_ISR_PUSHIS_OFST )

#define MML_SKBD_ISR_RELEASEIS_OFST				1 //!< RELEASEIS offset
#define MML_SKBD_ISR_RELEASEIS_MASK_NOOFST		0x1
#define MML_SKBD_ISR_RELEASEIS_MASK				( MML_SKBD_ISR_RELEASEIS_MASK_NOOFST << MML_SKBD_ISR_RELEASEIS_OFST )

#define MML_SKBD_ISR_OVERIS_OFST				2 //!< OVERIS offset
#define MML_SKBD_ISR_OVERIS_MASK_NOOFST			0x01
#define MML_SKBD_ISR_OVERIS_MASK				( MML_SKBD_ISR_OVERIS_MASK_NOOFST << MML_SKBD_ISR_OVERIS_OFST )

#define MML_SKBD_ISR_GPIOIS_OFST				3 //!< GPIOIS offset
#define MML_SKBD_ISR_GPIOIS_MASK_NOOFST			0x01
#define MML_SKBD_ISR_GPIOIS_MASK				( MML_SKBD_ISR_GPIOIS_MASK_NOOFST << MML_SKBD_ISR_GPIOIS_OFST )

#define MML_SKBD_ISR_RFU1_OFST					4 //!< RFU1 offset

/** @} */ /* @defgroup MML_SKBD_REGS_ISR */

/** @defgroup MML_SKBD_REGS_K0R Key 0 Register (K0R)
 *
 * @li IOIN 	[3:0] 	- R : Number of the input scanned
 * @li RFU1  	[4]		- R : RFU
 * @li IOOUT	[8:5]	- R : Number of the output scanned
 * @li RFU2		[9]		- R : RFU
 * @li PUSH		[10]	- R : Key has been released
 * @li READ		[11]	- R : Key has been read
 * @li NEXT		[12]	- R : Other key has been detected
 * @li RFU3		[31:13]	- R : RFU
 * @{
 */
#define MML_SKBD_K0R_OFST						0x14 //!< Register offset
#define MML_SKBD_K0R_DFLT						0x0 //!< Register default value

/** Bits Fields */
#define MML_SKBD_K0R_IOIN_OFST					0 //!< IOIN offset
#define	MML_SKBD_K0R_IOIN_MASK_NOOFST			0xf
#define	MML_SKBD_K0R_IOIN_MASK					( MML_SKBD_K0R_IOIN_MASK_NOOFST << MML_SKBD_K0R_IOIN_OFST )

#define MML_SKBD_K0R_RFU1_OFST					4 //!< RFU1 offset

#define MML_SKBD_K0R_IOOUT_OFST					5 //!< IOOUT offset
#define MML_SKBD_K0R_IOOUT_MASK_NOOFST			0xf
#define MML_SKBD_K0R_IOOUT_MASK					( MML_SKBD_K0R_IOOUT_MASK_NOOFST << MML_SKBD_K0R_IOOUT_OFST )

#define MML_SKBD_K0R_RFU2_OFST					9 //!< RFU2 offset

#define MML_SKBD_K0R_PUSH_OFST					10 //!< PUSH offset
#define	MML_SKBD_K0R_PUSH_MASK_NOOFST			0x1
#define	MML_SKBD_K0R_PUSH_MASK					( MML_SKBD_K0R_PUSH_MASK_NOOFST << MML_SKBD_K0R_PUSH_OFST )

#define MML_SKBD_K0R_READ_OFST					11 //!< READ offset
#define	MML_SKBD_K0R_READ_MASK_NOOFST			0x1
#define	MML_SKBD_K0R_READ_MASK					( MML_SKBD_K0R_READ_MASK_NOOFST << MML_SKBD_K0R_READ_OFST )

#define MML_SKBD_K0R_NEXT_OFST					12 //!< NEXT offset
#define	MML_SKBD_K0R_NEXT_MASK_NOOFST			0x1
#define	MML_SKBD_K0R_NEXT_MASK					( MML_SKBD_K0R_NEXT_MASK_NOOFST << MML_SKBD_K0R_NEXT_OFST )

#define MML_SKBD_K0R_RFU3_OFST					13 //!< RFU3 offset

/** Enumeration of the different value for a bits field . */
#define MML_SKBD_K0R_KEY_PUSHED					0 //!< Key has been pushed
#define MML_SKBD_K0R_KEY_RELEASED				1 //!< Key has been released

#define MML_SKBD_K0R_KEY_PENDING				0 //!< Key has not been read
#define MML_SKBD_K0R_KEY_READ					1 //!< Key has been read

/** @} */ /* @defgroup MML_SKBD_REGS_K0R */

/** @defgroup MML_SKBD_REGS_K1R Key 1 Register (K1R)
 *
 * @li IOIN 	[3:0] 	- R : Number of the input scanned
 * @li RFU1  	[4]		- R : RFU
 * @li IOOUT	[8:5]	- R : Number of the output scanned
 * @li RFU2		[9]		- R : RFU
 * @li PUSH		[10]	- R : Key has been released
 * @li READ		[11]	- R : Key has been read
 * @li NEXT		[12]	- R : Other key has been detected
 * @li RFU3		[31:13]	- R : RFU
 * @{
 */
#define MML_SKBD_K1R_OFST						0x18 //!< Register offset
#define MML_SKBD_K1R_DFLT						0x0 //!< Register default value

/** Bits Fields */
#define MML_SKBD_K1R_IOIN_OFST					0 //!< IOIN offset
#define MML_SKBD_K1R_RFU1_OFST					4 //!< RFU1 offset
#define MML_SKBD_K1R_IOOUT_OFST					5 //!< IOOUT offset
#define MML_SKBD_K1R_RFU2_OFST					9 //!< RFU2 offset
#define MML_SKBD_K1R_PUSH_OFST					10 //!< PUSH offset
#define MML_SKBD_K1R_READ_OFST					11 //!< READ offset
#define MML_SKBD_K1R_NEXT_OFST					12 //!< NEXT offset
#define MML_SKBD_K1R_RFU3_OFST					13 //!< RFU3 offset

/** Enumeration of the different value for a bits field . */
#define MML_SKBD_K1R_KEY_PUSHED					0 //!< Key has been pushed
#define MML_SKBD_K1R_KEY_RELEASED				1 //!< Key has been released

#define MML_SKBD_K1R_KEY_PENDING				0 //!< Key has not been read
#define MML_SKBD_K1R_KEY_READ					1 //!< Key has been read

/** @} */ /* @defgroup MML_SKBD_REGS_K1R */

/** @defgroup MML_SKBD_REGS_K2R Key 2 Register (K2R)
 *
 * @li IOIN 	[3:0] 	- R : Number of the input scanned
 * @li RFU1  	[4]		- R : RFU
 * @li IOOUT	[8:5]	- R : Number of the output scanned
 * @li RFU2		[9]		- R : RFU
 * @li PUSH		[10]	- R : Key has been released
 * @li READ		[11]	- R : Key has been read
 * @li NEXT		[12]	- R : Other key has been detected
 * @li RFU3		[31:13]	- R : RFU
 * @{
 */
#define MML_SKBD_K2R_OFST						0x1c //!< Register offset
#define MML_SKBD_K2R_DFLT						0x0 //!< Register default value

/** Bits Fields */
#define MML_SKBD_K2R_IOIN_OFST					0 //!< IOIN offset
#define MML_SKBD_K2R_RFU1_OFST					4 //!< RFU1 offset
#define MML_SKBD_K2R_IOOUT_OFST					5 //!< IOOUT offset
#define MML_SKBD_K2R_RFU2_OFST					9 //!< RFU2 offset
#define MML_SKBD_K2R_PUSH_OFST					10 //!< PUSH offset
#define MML_SKBD_K2R_READ_OFST					11 //!< READ offset
#define MML_SKBD_K2R_NEXT_OFST					12 //!< NEXT offset
#define MML_SKBD_K2R_RFU3_OFST					13 //!< RFU3 offset

/** Enumeration of the different value for a bits field . */
#define MML_SKBD_K2R_KEY_PUSHED					0 //!< Key has been pushed
#define MML_SKBD_K2R_KEY_RELEASED				1 //!< Key has been released

#define MML_SKBD_K2R_KEY_PENDING				0 //!< Key has not been read
#define MML_SKBD_K2R_KEY_READ					1 //!< Key has been read

/** @} */ /* @defgroup MML_SKBD_REGS_K2R */

/** @defgroup MML_SKBD_REGS_K3R Key 3 Register (K3R)
 *
 * @li IOIN 	[3:0] 	- R : Number of the input scanned
 * @li RFU1  	[4]		- R : RFU
 * @li IOOUT	[8:5]	- R : Number of the output scanned
 * @li RFU2		[9]		- R : RFU
 * @li PUSH		[10]	- R : Key has been released
 * @li READ		[11]	- R : Key has been read
 * @li RFU3		[31:12]	- R : RFU
 * @{
 */
#define MML_SKBD_K3R_OFST						0x20 //!< Register offset
#define MML_SKBD_K3R_DFLT						0x0 //!< Register default value

/** Bits Fields */
#define MML_SKBD_K3R_IOIN_OFST					0 //!< IOIN offset
#define MML_SKBD_K3R_RFU1_OFST					4 //!< RFU1 offset
#define MML_SKBD_K3R_IOOUT_OFST					5 //!< IOOUT offset
#define MML_SKBD_K3R_RFU2_OFST					9 //!< RFU2 offset
#define MML_SKBD_K3R_PUSH_OFST					10 //!< PUSH offset
#define MML_SKBD_K3R_READ_OFST					11 //!< READ offset
#define MML_SKBD_K3R_RFU3_OFST					12 //!< RFU3 offset

/** Enumeration of the different value for a bits field . */
#define MML_SKBD_K3R_KEY_PUSHED					0 //!< Key has been pushed
#define MML_SKBD_K3R_KEY_RELEASED				1 //!< Key has been released

#define MML_SKBD_K3R_KEY_PENDING				0 //!< Key has not been read
#define MML_SKBD_K3R_KEY_READ					1 //!< Key has been read

/** @} */ /* @defgroup MML_SKBD_REGS_K3R */

/** @defgroup MML_SKBD_REGS_GPIO0R GPIO Register 0 (GPIO0R)
 *
 * @li GPOUT 	[15:0] 	- R/W : Output values of KBD IOs in GPIO mode
 * @li RFU1  	[31:16]	- R   : RFU
 *
 * @{
 */
#define MML_SKBD_GPIO0R_OFST					0x24 //!< Register offset
#define MML_SKBD_GPIO0R_DFLT					0x0 //!< Register default value

/** Bits Fields */
#define MML_SKBD_GPIO0R_OUT_OFST				0 //!< IOIN offset
#define MML_SKBD_GPIO0R_RFU1_OFST				16 //!< RFU1 offset

/** Enumeration of the different value for a bits field . */
#define MML_SKBD_GPIO0R_HIGHZ(_n_)				( 1 << _n_ ) //!< IO is leave in Z state
#define MML_SKBD_GPIO0R_FORCED					0 //!< IO is forced to '0'

/** @} */ /* @defgroup MML_SKBD_REGS_GPIO0R */

/** @defgroup MML_SKBD_REGS_GPIO1R GPIO Register 1 (GPIO1R)
 *
 * @li GPIN 	[15:0] 	- R : Input values of KBD IOs in GPIO mode
 * @li RFU1  	[31:16]	- R : RFU
 *
 * @{
 */
#define MML_SKBD_GPIO1R_OFST					0x28 //!< Register offset
#define MML_SKBD_GPIO1R_DFLT					0x0 //!< Register default value

/** Bits Fields */
#define MML_SKBD_GPIO1R_IN_OFST					0 //!< IOIN offset
#define MML_SKBD_GPIO1R_RFU1_OFST				16 //!< RFU1 offset

/** Enumeration of the different value for a bits field . */
#define MML_SKBD_GPIO1R_HIGHZ(_n_)				( 1 << _n_ ) //!< IO is leave in Z state
#define MML_SKBD_GPIO1R_FORCED					0 //!< IO is forced to '0'

/** @} */ /* @defgroup MML_SKBD_REGS_GPIO1R */

/** Number of key registers present in the keypad interface */
#define	MML_SKBD_TOTAL_KEY_REGS					4
/** Key pressed flag */
#define	MML_SKBD_KEY_PRESS_FLAG					( 0x1 << 8 )

#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */
/** SKBD Registers.
 *
 */
typedef volatile struct
{
	/** Control register 0 */
	volatile unsigned							cr0;
	/** Control register 1 */
	volatile unsigned							cr1;
	/** Status register */
	volatile unsigned							sr;
	/** Interrupt enable register */
	volatile unsigned							ier;
	/** Interrupt status register */
	volatile unsigned							isr;
	/** Key register 0 */
	volatile unsigned							key0;
	/** Key register 1 */
	volatile unsigned							key1;
	/** Key register 2 */
	volatile unsigned							key2;
	/** Key register 3 */
	volatile unsigned							key3;
	/** GPIO register 0 */
	volatile unsigned							gpio0;
	/** GPIO register 1 */
	volatile unsigned							gpio1;

} mml_skbd_regs_t;

#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_SKBD_REGS */

#endif /* _MML_SKBD_REGS_H_ */

/******************************************************************************/
/* EOF */
