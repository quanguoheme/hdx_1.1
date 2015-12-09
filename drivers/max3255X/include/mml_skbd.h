/*
 * mml_skbd.h --
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

#ifndef _MML_SKBD_H_
#define _MML_SKBD_H_

/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
/** Local includes */
#include <mml_skbd_regs.h>

/** Macros Defines ********************************************************************/
/** COBRA adaptation */
#define	MML_KEYPAD_BASE_ERR				COBRA_KEYPAD_BASE_ERR

/** Flag to indicate the key press/release event */
#define	MML_SKBD_FLAG_KEY_PRESS			MML_SKBD_KEY_PRESS_FLAG

/** Enumerations ***************************************************************/
/** keypad errors list */
typedef enum
{
	MML_SKBD_ERR_MIN = MML_KEYPAD_BASE_ERR,
	MML_SKBD_ERR_NOT_INITIALIZED, /** Error Code: keypad not initialized */
	MML_SKBD_ERR_ALREAD_INITIALIZED, /** Keypad already initialized */
	MML_SKBD_ERR_INVALID_OPERATION, /** Invalid keypad operation */
	MML_SKBD_ERR_OUT_OF_RANGE, /** Invalid parameter or value */
	MML_SKBD_ERR_OVERRUN, 	/** keypad Over run error */
	MML_SKBD_ERR_IRQ, /** IRQ setup error */
	MML_SKBD_ERR_IRQ_NULL, /** NULL IRQ handler */
	/** One or more keypad I/O pins are overlapped or  input/output pin configurations are invalid */
	MML_SKBD_ERR_INVALID_PIN_CONFIGURATION,
	MML_SKBD_ERR_BUSY, /** keypad is busy */
	MML_SKBD_ERR_UNKNOWN, 	/** Error Code: Generic error for unknown behaviour */
	MML_SKBD_ERR_MAX = MML_SKBD_ERR_UNKNOWN

} mml_skbd_errors_t;

/**  Keypad initialization state FSM */
typedef enum
{
	MML_SKBD_STATE_MIN = 0,
	MML_SKBD_STATE_NOT_INITIALIZED = MML_SKBD_STATE_MIN,
	MML_SKBD_STATE_INITIALIZED,
	MML_SKBD_STATE_CLOSED,
	MML_SKBD_STATE_MAX = MML_SKBD_STATE_CLOSED,
	MML_SKBD_STATE_COUNT

} mml_skbd_state_t;

/** Keypad Debouncing Time */
typedef enum
{
	MML_SKBD_DBTM_10MS = 0,
	MML_SKBD_DBTM_13MS,
	MML_SKBD_DBTM_15MS,
	MML_SKBD_DBTM_18MS,
	MML_SKBD_DBTM_21MS,
	MML_SKBD_DBTM_24MS,
	MML_SKBD_DBTM_27MS,
	MML_SKBD_DBTM_30MS

} mml_skbd_debounce_time_t;

/** Keypad events */
typedef enum
{
	MML_SKBD_EVENT_PUSH = MML_SKBD_IER_PUSHIE_MASK,
	MML_SKBD_EVENT_RELEASE = MML_SKBD_IER_RELEASEIE_MASK,
	MML_SKBD_EVENT_OVERRUN = MML_SKBD_IER_OVERIE_MASK,
	MML_SKBD_EVENT_GPIOIS = MML_SKBD_IER_GPIOIE_MASK

} mml_skbd_events_t;

/** Keypad Interrupt Status */
typedef enum
{
	MML_SKBD_INTERRUPT_STATUS_PUSHIS = MML_SKBD_ISR_PUSHIS_MASK,
	MML_SKBD_INTERRUPT_STATUS_RELEASEIS = MML_SKBD_ISR_RELEASEIS_MASK,
	MML_SKBD_INTERRUPT_STATUS_OVERIS = MML_SKBD_ISR_OVERIS_MASK,
	MML_SKBD_INTERRUPT_STATUS_GPIOIS = MML_SKBD_ISR_GPIOIS_MASK

} mml_interrupt_status_t;

/** Keypad I/O's IOSEL */
typedef enum
{
	MML_SKBD_KBDIO0  = (0x01 << 0),
	MML_SKBD_KBDIO1  = (0x01 << 1),
	MML_SKBD_KBDIO2  = (0x01 << 2),
	MML_SKBD_KBDIO3  = (0x01 << 3),
	MML_SKBD_KBDIO4  = (0x01 << 4),
	MML_SKBD_KBDIO5  = (0x01 << 5),
	MML_SKBD_KBDIO6  = (0x01 << 6),
	MML_SKBD_KBDIO7  = (0x01 << 7),
	MML_SKBD_KBDIO8  = (0x01 << 8),
	MML_SKBD_KBDIO9  = (0x01 << 9),
	MML_SKBD_KBDIO10 = (0x01 << 10),
	MML_SKBD_KBDIO11 = (0x01 << 11),
	MML_SKBD_KBDIO12 = (0x01 << 12),
	MML_SKBD_KBDIO13 = (0x01 << 13),
	MML_SKBD_KBDIO14 = (0x01 << 14),
	MML_SKBD_KBDIO15 = (0x01 << 15)

} mml_skbd_io_pins_t;

/** Structures *****************************************************************/
/** keypad IRQ handler function */
typedef void (*irq_handler_t)(void);

/**
 * [TBD] keypad configuration structure
 */
typedef struct
{
	/** I/O pin direction selection for the corresponding keypad pins */
	unsigned short							ioselect;
	/** key register erase flag on key is released */
	unsigned int							reg_erase;
	/** Specifies the keypad pins to be configured as output */
	unsigned short							outputs;
	/** Specifies the keypad pins to be configured as input */
	unsigned short							inputs;
	/** Keypad Debouncing Time */
	mml_skbd_debounce_time_t				debounce;
	/** IRQ handler */
	irq_handler_t							irq_handler;

} mml_skbd_config_t;

/** keypad channel context information */
typedef struct
{
	/**  */
	unsigned int							first_init;
	/** Interrupt request(IRQ) number */
	unsigned int 							irq;
	/** IRQ handler */
	irq_handler_t							irq_handler;
	/** keypad initialization state */
	mml_skbd_state_t						state;
	/** keypad reload callback function */

} mml_skbd_context_t;

/** keyboard Key's scan codes */
typedef struct
{
	/**
	 * key scan code format as follows
	 *  	key(x) bits[3-0] : Input scan code
	 *  	key(x) bits[7-4] : Output scan code
	 * 		key(x) bit[8]	 : Next Key Flag
	 */
	/** Key0 scan code */
	unsigned short							key0;
	/** Key1 scan code */
	unsigned short							key1;
	/** Key2 scan code */
	unsigned short 							key2;
	/** Key3 scan code */
	unsigned short 							key3;

} mml_skbd_keys_t;

/** Functions ******************************************************************/
/** The function is used to initialize the keypad controller */
int mml_keypad_init(mml_skbd_config_t config);
/** Function is used to enable the interrupt events */
int mml_keypad_enable_interrupt_events(unsigned int events);
/** Function to disable the interrupt events */
int mml_keypad_disable_interrupt_events(unsigned int events);
/** Function is used to clear the interrupt events */
int mml_keypad_clear_interrupt_status(unsigned int status);
/** Function is used to read the interrupt status */
int mml_keypad_interrupt_status(unsigned int *status);
/** Function to read the key scan codes */
int mml_keypad_read_keys(mml_skbd_keys_t *keys);
/** Function to close the keypad */
int mml_keypad_close(void);

#endif /* _MML_SKBD_H_ */

/******************************************************************************/
/* EOF */
