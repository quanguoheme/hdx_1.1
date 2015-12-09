/*
 * mml_tmr_regs.h --
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

#ifndef MML_TMR_REGS_H_
#define MML_TMR_REGS_H_

/** @file mml_tmr_regs.h TMR Registers Header */

/** @defgroup MML_TMR TMR Driver */

/** @defgroup MML_TMR_REGS TMR Registers
 *
 * The timer can be used for timing, event counting, or generation of
 * pulse-width modulated (PWM) signals. The timers’ features include:
 * @li 32-bit reload counter
 * @li Timer output pin
 * @li Programmable prescaler with prescale values from 1 to 4096
 * @li PWM output generation
 * @li Capture, compare and capture/compare capability
 * @li External input pin for timer input, clock gating, or capture signal
 * @li Timer interrupt
 *
 *
 * @ingroup MML_TMR
 * @{
 */


/** @defgroup MML_TMR_REGS_CNT Count Register
 *
 * In Counter mode, the Timer register, stores the 32-bit Count value. Writing
 * to the Timer register while the timer is enabled is not recommended. There
 * are no temporary holding registers available for write operations, so
 * simultaneous 32-bit writes are not possible. If the Timer register is
 * written during counting, the 32-bit written value is placed in the counter
 * at the next clock edge. The counter continues counting from the new value.
 *
 * @{
 */
#define MML_TMR_CNT_OFST						0x00000000 //!< Register Offset
#define MML_TMR_CNT_DFLT						0x00000000 //!< Register Default Value

#define MML_TMR_CNT_COUNT_OFST					0
#define MML_TMR_CNT_COUNT_MASK_NOOFST			0xffffffff //!< Mask no offset
#define	MML_TMR_CNT_COUNT_MASK					( MML_TMR_CNT_COUNT_MASK_NOOFST << MML_TMR_CNT_COUNT_OFST )
/** @} */ /* @defgroup MML_TMR_REGS_CNT */

/** @defgroup MML_TMR_REGS_CMP Compare Register
 *
 * In Compare and Capture/Compare modes, the Timer Compare register stores the
 * 32-bit Compare value.
 *
 * @{
 */
#define MML_TMR_CMP_OFST						0x00000004 //!< Register Offset
#define MML_TMR_CMP_DFLT						0xffffffff //!< Register Default Value

#define MML_TMR_CMP_COUNT_OFST					0
#define MML_TMR_CMP_COUNT_MASK_NOOFST			0xffffffff //!< Mask no offset
#define	MML_TMR_CMP_COUNT_MASK					( MML_TMR_CMP_COUNT_MASK_NOOFST << MML_TMR_CMP_COUNT_OFST )
/** @} */ /* @defgroup MML_TMR_REGS_CMP */

/** @defgroup MML_TMR_REGS_PWM PWM Register
 *
 * PWM forms a 32-bit value that is compared to the current
 * 32-bit timer count. When a match occurs, the PWM output changes state. The
 * PWM output value is set by the TPOL bit in the Timer Control Register
 * (MML_TMR_REGS_CTRL). In Capture and Capture/Compare modes, the PWM
 * register stores the 32-bit captured timer value.

 *
 * @li
 *
 * @{
 */
#define MML_TMR_PWM_OFST						0x00000008 //!< Register Offset
#define MML_TMR_PWM_DFLT						0x00000000 //!< Register Default Value

#define MML_TMR_PWM_COUNT_OFST					0
#define MML_TMR_PWM_COUNT_MASK_NOOFST			0xffffffff //!< Mask no offset
#define	MML_TMR_PWM_COUNT_MASK					( MML_TMR_PWM_COUNT_MASK_NOOFST << MML_TMR_PWM_COUNT_OFST )
/** @} */ /* @defgroup MML_TMR_REGS_PWM */

/** @defgroup MML_TMR_REGS_INT Interrupt Register
 *
 * Register for interruption clearing.
 *
 * @{
 */
#define MML_TMR_INT_OFST						0x0000000c //!< Register Offset
#define MML_TMR_INT_DFLT						0x00000000 //!< Register Default Value

#define MML_TMR_INT_CLR_OFST					0
#define MML_TMR_INT_CLR_MASK_NOOFST				0x1
#define MML_TMR_INT_CLR_MASK					( MML_TMR_INT_CLR_MASK_NOOFST << MML_TMR_INT_CLR_OFST )
/** @} */ /* @defgroup MML_TMR_REGS_INT */


/** @defgroup MML_TMR_REGS_CN Controller Register
 *
 * The Timer Control register enables/disables the timers, sets the prescaler
 * value, and determines the timer operating mode.
 *
 * @li TMODE [0:2]: Timer Mode
 * @li PRES [5:3]: Prescale Value
 * @li TPOL [6]: Timer Input/Output Polarity
 * @li TEN [7]: Timer Enable
 *
 * @{
 */
#define MML_TMR_CN_OFST							0x00000010 //!< Register Offset
#define MML_TMR_CN_DFLT							0x00000000 //!< Register Default Value

#define MML_TMR_CN_TMODE_OFST					0 //!< Timer Mode offset
#define MML_TMR_CN_TMODE_MASK_NOOFST			0x7 //!< Timer Mode mask no offset
#define MML_TMR_CN_TMODE_MASK					( MML_TMR_CN_TMODE_MASK_NOOFST << MML_TMR_CN_TMODE_OFST ) //!< Timer Mode mask

#define MML_TMR_CN_TMODE_ONE_SHOT				0 //!< One shot mode
#define MML_TMR_CN_TMODE_CONTINUOUS				1 //!< Continuous mode
#define MML_TMR_CN_TMODE_COUNTER				2 //!< Counter mode
#define MML_TMR_CN_TMODE_PWM					3 //!< PWM mode
#define MML_TMR_CN_TMODE_CAPTURE				4 //!< Capture mode
#define MML_TMR_CN_TMODE_COMPARE				5 //!< Compare mode
#define MML_TMR_CN_TMODE_GATED					6 //!< Gated mode
#define MML_TMR_CN_TMODE_CAPCOMP				7 //!< Capture/Compare mode


#define MML_TMR_CN_PRES_OFST					3 //!< Prescale offset
#define MML_TMR_CN_PRES_MASK_NOOFST				0x27 //!< Prescale make no offset
#define MML_TMR_CN_PRES_MASK					( MML_TMR_CN_PRES_MASK_NOOFST << MML_TMR_CN_PRES_OFST ) //!< Prescale make

#define MML_TMR_CN_PRES_DIV1					0 //!< Divide by 1
#define MML_TMR_CN_PRES_DIV2					1 //!< Divide by 2
#define MML_TMR_CN_PRES_DIV4					2 //!< Divide by 4
#define MML_TMR_CN_PRES_DIV8					3 //!< Divide by 8
#define MML_TMR_CN_PRES_DIV16					4 //!< Divide by 16
#define MML_TMR_CN_PRES_DIV32					5 //!< Divide by 32
#define MML_TMR_CN_PRES_DIV64					6 //!< Divide by 64
#define MML_TMR_CN_PRES_DIV128					7 //!< Divide by 128
#define MML_TMR_CN_PRES_DIV256					0x20 //!< Divide by 256
#define MML_TMR_CN_PRES_DIV512					0x21 //!< Divide by 512
#define MML_TMR_CN_PRES_DIV1024					0x22 //!< Divide by 1024
#define MML_TMR_CN_PRES_DIV2048					0x23 //!< Divide by 2048
#define MML_TMR_CN_PRES_DIV4096					0x24 //!< Divide by 4096

#define MML_TMR_CN_TPOL_OFST					6 //!< Timer Input/Output Polarity offset
#define MML_TMR_CN_TPOL_MASK_NOOFST				0x1 //!< Timer Input/Output Polarity mask no offset
#define MML_TMR_CN_TPOL_MASK					( MML_TMR_CN_TPOL_MASK_NOOFST << MML_TMR_CN_TPOL_OFST ) //!< Timer Input/Output Polarity mask

#define MML_TMR_CN_TPOL_HIGH					1 //!< Polarity high
#define MML_TMR_CN_TPOL_LOW						0 //!< Polarity low

#define MML_TMR_CN_TEN_OFST						7 //!< Timer enable offset
#define MML_TMR_CN_TEN_MASK_NOOFST				0x1 //!< Timer enable mask no offset
#define MML_TMR_CN_TEN_MASK						( MML_TMR_CN_TEN_MASK_NOOFST << MML_TMR_CN_TEN_OFST ) //!< Timer enable mask

#define MML_TMR_CN_TEN_ENABLE					1 //!< Enable timer

#define MML_TMR_CN_PWMSYNC_OFST					9 //!< Timer
#define MML_TMR_CN_PWMSYNC_MASK_NOOFST			0x1 //!< Timer no offset
#define MML_TMR_CN_PWMSYNC_MASK					( MML_TMR_CN_PWMSYNC_MASK_NOOFST << MML_TMR_CN_PWMSYNC_OFST ) //!< Timer mask

#define MML_TMR_CN_NOLHPOL_OFST					10 //!< Timer
#define MML_TMR_CN_NOLHPOL_MASK_NOOFST			0x1 //!< Timer no offset
#define MML_TMR_CN_NOLHPOL_MASK					( MML_TMR_CN_NOLHPOL_MASK_NOOFST << MML_TMR_CN_NOLHPOL_OFST ) //!< Timer mask

#define MML_TMR_CN_NOLLPOL_OFST					11 //!< Timer
#define MML_TMR_CN_NOLLPOL_MASK_NOOFST			0x1 //!< Timer no offset
#define MML_TMR_CN_NOLLPOL_MASK					( MML_TMR_CN_NOLLPOL_MASK_NOOFST << MML_TMR_CN_NOLLPOL_OFST ) //!< Timer mask

#define MML_TMR_CN_PWMCKBD_OFST					12 //!< Timer
#define MML_TMR_CN_PWMCKBD_MASK_NOOFST			0x1 //!< Timer no offset
#define MML_TMR_CN_PWMCKBD_MASK					( MML_TMR_CN_PWMCKBD_MASK_NOOFST << MML_TMR_CN_PWMCKBD_OFST ) //!< Timer mask

#define	MML_TMR_RSTR_OFST						16 //!< Offset for Timer reset [18:16] PWMx reset
#define	MML_TMR_ACTIVE_OFST						16 //!< offset for clock enable [18:16] PWMx: PWM/timers
/** @} */ /* @defgroup MML_TMR_REGS_CN */


/** @defgroup MML_TMR_REGS_NOLCMP Controller Register
 *
 * @li NOLLCMP [0:7]: Non Overlapping Low Compare
 * @li NOLHCMP [8:15]: Non Overlapping High Compare
 *
 * @{
 */
#define MML_TMR_NOLCMP_OFST						0x00000014 //!< Register Offset
#define MML_TMR_NOLCMP_DFLT						0x00000000 //!< Register Default Value

#define MML_TMR_NOLCMP_NOLLCMP_OFST				0 //!< Non Overlapping Low Compare offset
#define MML_TMR_NOLCMP_NOLLCMP_MASK_NOOFST		0xff //!< Non Overlapping Low Compare mask no offset
#define MML_TMR_NOLCMP_NOLLCMP_MASK				( MML_TMR_NOLCMP_NOLLCMP_MASK_NOOFST << MML_TMR_NOLCMP_NOLLCMP_OFST ) //!< Non Overlapping Low Compare mask

#define MML_TMR_NOLCMP_NOLHCMP_OFST				8 //!< Non Overlapping High Compare offset
#define MML_TMR_NOLCMP_NOLHCMP_MASK_NOOFST		0xff //!< Non Overlapping High Compare mask no offset
#define MML_TMR_NOLCMP_NOLHCMP_MASK				( MML_TMR_NOLCMP_NOLHCMP_MASK_NOOFST << MML_TMR_NOLCMP_NOLHCMP_OFST ) //!< Non Overlapping High Compare mask
/** @} */ /* @defgroup MML_TMR_REGS_NOLCMP */

#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */
/** TMR Registers.
 *
 */
typedef volatile struct
{
	/** Count register */
	unsigned int								count;
	 /** Compare register */
	unsigned int								compare;
	 /** PWM register */
	unsigned int								pwm;
	 /** Interrupt register */
	unsigned int								interrupt;
	/** Control register */
	unsigned int								control;
	/** Non-Overlapping Compare */
	unsigned int								nolcmp;

} mml_tmr_regs_t;

/** Timer Prescale Values
 *
 */
typedef enum
{
	MML_TMR_PRES_DIV_1 = 0, //!< Prescale: divide by 1
	MML_TMR_PRES_DIV_2, //!< Prescale: divide by 2
	MML_TMR_PRES_DIV_4, //!< Prescale: divide by 4
	MML_TMR_PRES_DIV_8, //!< Prescale: divide by 8
	MML_TMR_PRES_DIV_16, //!< Prescale: divide by 16
	MML_TMR_PRES_DIV_32, //!< Prescale: divide by 32
	MML_TMR_PRES_DIV_64, //!< Prescale: divide by 64
	MML_TMR_PRES_DIV_128, //!< Prescale: divide by 128
	MML_TMR_PRES_DIV_256 = MML_TMR_CN_PRES_DIV256, //!< Prescale: divide by 256
	MML_TMR_PRES_DIV_512, //!< Prescale: divide by 512
	MML_TMR_PRES_DIV_1024, //!< Prescale: divide by 1024
	MML_TMR_PRES_DIV_2048, //!< Prescale: divide by 2048
	MML_TMR_PRES_DIV_4096 //!< Prescale: divide by 4096

} mml_tmr_prescale_t;

/** Timer Modes
 *
 */
typedef enum
{
	MML_TMR_MODE_ONE_SHOT = 0,//!< One shot mode
	MML_TMR_MODE_CONTINUOUS, //!< Continuous mode
	MML_TMR_MODE_COUNTER, //!< Counter mode
	MML_TMR_MODE_PWM, //!< PWM mode
	MML_TMR_MODE_CAPTURE, //!< Capture mode
	MML_TMR_MODE_COMPARE, //!< Compare mode
	MML_TMR_MODE_GATED, //!< Gated mode
	MML_TMR_MODE_CAPCOMP //!< Capture/Compare mode

} mml_tmr_mode_t;

#endif /* __ASSEMBLER__ */

/** @} *//* @defgroup MML_TMR_REGS */

#endif /* MML_TMR_REGS_H_ */

/******************************************************************************/
/* EOF */
