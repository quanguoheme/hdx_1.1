/*
 * errors.h --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2010, Maxim Integrated
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
 * Created on: Nov 03, 2010
 * Author: Dayananda H.B. <dayananda.hb@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: : Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */
#ifndef _ERRORS_H_
#define _ERRORS_H_

/** @file errors.h Cobra common error codes */

/** @defgroup COBRA Cobra Tools */

/** @defgroup COBRA_ERRORS Cobra common error codes
 *
 * @ingroup COBRA
 *
 * @{
 */

#ifndef __ASSEMBLER__

/* -------------------------------------------------------------------------- */
/* Common error codes */
#define COBRA_COMMON         		0x7000
/**  */
#define COBRA_UART            		0x7071
#define COBRA_I2C             		0x7072
#define COBRA_DMA             		0x7073
#define COBRA_GPIO             		0x7074
#define COBRA_USB	            	0x7075
#define COBRA_KEYPAD             	0x7076
#define COBRA_SPI	            	0x7077
#define COBRA_SMART_CARD            0x7078
#define COBRA_TIMER	             	0x7079
#define COBRA_TFT	            	0x707a
#define	COBRA_SMON					0x707b
#define	COBRA_TRNG					0x707c
#define	COBRA_ADC					0x707d
#define	COBRA_ETH					0x707e
#define	COBRA_GCR					0x707f
#define	COBRA_INIT					0x7081
#define	COBRA_INTC					0x7082
#define	COBRA_MLCD					0x7083
#define	COBRA_RTC					0x7084
#define	COBRA_UCI					0x7085
#define	COBRA_WATCHDOG				0x7086
#define	COBRA_OTP					0x7087


/** Error code prefix offset */
#define COBRA_ERR_PREFIX_OFFSET		16

/** Common errors list */
typedef enum
{
	/** Specific No Error code */
	NO_ERROR = 0,
	/** Generic errors */
	COMMON_ERR_MIN = (COBRA_COMMON << COBRA_ERR_PREFIX_OFFSET),
	/** Error Code: No such device */
	COMMON_ERR_NO_DEV,
	/** Error Code: Value is not appropriate */
	COMMON_ERR_INVAL,
	/** Error Code: Pointer is null */
	COMMON_ERR_NULL_PTR,
	/** Error Code: Value is out of expected range */
	COMMON_ERR_OUT_OF_RANGE,
	/** Error Code: Module not initialized */
	COMMON_ERR_NOT_INITIALIZED,
	/** Error Code: Critical error */
	COMMON_ERR_FATAL_ERROR,
	/** Error Code: Still processing */
	COMMON_ERR_RUNNING,
	/** Error Code: Action not allowed in this state */
	COMMON_ERR_BAD_STATE,
	/** Error Code: Data does not match */
	COMMON_ERR_NO_MATCH,
	/** Error Code: Action already done */
	COMMON_ERR_ALREADY,
	/** Error Code: Action not finished yet, still in progress */
	COMMON_ERR_IN_PROGRESS,
	/** Error Code: Operation is not permitted */
	COMMON_ERR_NOT_PERMITTED,
	/** Error Code: Generic error for unknown behavior */
	COMMON_ERR_UNKNOWN,
	COMMON_ERR_MAX = COMMON_ERR_UNKNOWN,

} cobra_common_errors_t;

/** peripheral modules base errors  */
#define COBRA_COMMON_BASE_ERR       		(COBRA_COMMON << COBRA_ERR_PREFIX_OFFSET)
#define	COBRA_UART_BASE_ERR					(COBRA_UART << COBRA_ERR_PREFIX_OFFSET)
#define COBRA_I2C_BASE_ERR       			(COBRA_I2C << COBRA_ERR_PREFIX_OFFSET)
#define COBRA_DMA_BASE_ERR      			(COBRA_DMA << COBRA_ERR_PREFIX_OFFSET)
#define COBRA_GPIO_BASE_ERR     			(COBRA_GPIO << COBRA_ERR_PREFIX_OFFSET)
#define COBRA_USB_BASE_ERR       			(COBRA_USB << COBRA_ERR_PREFIX_OFFSET)
#define COBRA_KEYPAD_BASE_ERR    			(COBRA_KEYPAD << COBRA_ERR_PREFIX_OFFSET)
#define COBRA_SPI_BASE_ERR	    			(COBRA_SPI << COBRA_ERR_PREFIX_OFFSET)
#define COBRA_SMART_CARD_BASE_ERR    		(COBRA_SMART_CARD << COBRA_ERR_PREFIX_OFFSET)
#define COBRA_TIMER_BASE_ERR	        	(COBRA_TIMER << COBRA_ERR_PREFIX_OFFSET)
#define COBRA_TFT_BASE_ERR	        		(COBRA_TFT << COBRA_ERR_PREFIX_OFFSET)
#define COBRA_SMON_BASE_ERR       			(COBRA_SMON << COBRA_ERR_PREFIX_OFFSET)
#define	COBRA_TRNG_BASE_ERR					(COBRA_TRNG << COBRA_ERR_PREFIX_OFFSET)
#define	COBRA_ADC_BASE_ERR					(COBRA_ADC << COBRA_ERR_PREFIX_OFFSET)
#define	COBRA_ETH_BASE_ERR					(COBRA_ETH << COBRA_ERR_PREFIX_OFFSET)
#define	COBRA_GCR_BASE_ERR					(COBRA_GCR << COBRA_ERR_PREFIX_OFFSET)
#define	COBRA_INIT_BASE_ERR					(COBRA_INIT << COBRA_ERR_PREFIX_OFFSET)
#define	COBRA_INTC_BASE_ERR					(COBRA_INTC << COBRA_ERR_PREFIX_OFFSET)
#define	COBRA_MLCD_BASE_ERR					(COBRA_MLCD << COBRA_ERR_PREFIX_OFFSET)
#define	COBRA_RTC_BASE_ERR					(COBRA_RTC << COBRA_ERR_PREFIX_OFFSET)
#define	COBRA_UCI_BASE_ERR					(COBRA_UCI << COBRA_ERR_PREFIX_OFFSET)
#define	COBRA_WATCHDOG_BASE_ERR				(COBRA_WATCHDOG << COBRA_ERR_PREFIX_OFFSET)
#define	COBRA_OTP_BASE_ERR					(COBRA_OTP << COBRA_ERR_PREFIX_OFFSET)



#endif /* #ifndef __ASSEMBLER__ */

/** @} */ /* @defgroup COBRA_ERRORS */

#endif /* _ERRORS_H_ */

/******************************************************************************/
/* EOF */
