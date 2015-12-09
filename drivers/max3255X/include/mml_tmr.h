/*
 * mml_tmr.h --
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

#ifndef _MML_TMR_H_
#define _MML_TMR_H_

/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
/** Local includes */
#include <mml_tmr_regs.h>

/* Defines ********************************************************************/
/* Macros *********************************************************************/
#define	MML_TIMER_BASE_ERR						COBRA_TIMER_BASE_ERR
/* Enumerations ***************************************************************/
/** Timer errors list */
typedef enum
{
	MML_TMR_ERR_MIN = MML_TIMER_BASE_ERR,
	/** Error Code: Timer not initialized */
	MML_TMR_ERR_NOT_INITIALIZED,
	/** Error Code: Invalid operation */
	MML_TMR_ERR_INVALID,
	/** Error Code: Interrupt can not be initialized */
	MML_TMR_ERR_IRQ_SET,
	/** Error Code: Operation not allowed */
	MML_TMR_ERR_NOT_ALLOWED,
	/** Error Code: Timer is not disable */
	MML_TMR_ERR_ENABLE,
	/** Error Code: Generic error for unknown behavior */
	MML_TMR_ERR_UNKNOWN,
	MML_TMR_ERR_MAX = MML_TMR_ERR_UNKNOWN

} mml_tmr_errors_t;

/** Timer device identification numbers (id's) */
typedef enum
{
	MML_TMR_DEV_MIN = 0,
	MML_TMR_DEV0 = MML_TMR_DEV_MIN,
	MML_TMR_DEV1,
	MML_TMR_DEV2,
	MML_TMR_DEV3,
	MML_TMR_DEV4,
	MML_TMR_DEV5,
	MML_TMR_DEV6,
	MML_TMR_DEV7,
	MML_TMR_DEV_MAX = MML_TMR_DEV7,
	MML_TMR_DEV_COUNT

} mml_tmr_id_t;

/** Timer IOCTL commands */
typedef enum
{
	MML_TMR_CMD_SET_PRESCALE,
	MML_TMR_CMD_SET_MODE,
	MML_TMR_CMD_SET_IO_POLARITY
	/*[TBD] If any other IOCTL's goes here **/

} mml_tmr_ioctl_t;

/** Timer initialization state */
typedef enum
{
	MML_TMR_STATE_NOT_INITIALIZED = 0,
	MML_TMR_STATE_INITIALIZED

} mml_tmr_state_t;

/** Timer status values */
typedef enum
{
	MML_TMR_STATE_DISABLED = 0,
	MML_TMR_STATE_ENABLED

} mml_tmr_status_t;

/** Timer input/output polarity */
typedef enum
{
	MML_TMR_POLARITY_LOW,
	MML_TMR_POLARITY_HIGH

} mml_tmr_polarity_t;

/* Structures *****************************************************************/
typedef void(*mml_tmr_handler_t)();

/** This structure contains the Timer device configuration parameters */
typedef struct
{
	unsigned int								timeout;
	unsigned int								count;
	unsigned int 								pwm_value;
	mml_tmr_prescale_t 							clock;
	mml_tmr_mode_t 								mode;
	mml_tmr_polarity_t							polarity;
	/** IRQ handler */
	volatile mml_tmr_handler_t					handler;

} mml_tmr_config_t;

typedef struct
{
	/** Timer physical base address*/
	mml_tmr_regs_t								*mml_tmr;
	/** Timer state */
	mml_tmr_state_t								state;
	/** Timer status */
	mml_tmr_status_t 							status;

} mml_tmr_info_t;


/** Structure holds the Timer device context information */
typedef struct
{
	/** Very first initialization done ? */
	unsigned char								first_init;
	/** Bunch of timers :) */
	mml_tmr_info_t								tmr[MML_TMR_DEV_COUNT];


} mml_tmr_context_t;

/** @defgroup COBRA_TMR Cobra Timer Driver
 *
 * @ingroup COBRA
 *
 * @{
 */

/* -------------------------------------------------------------------------- */
/** Setup the timer to Reload mode and enable it.
 *
 * @param timer		Timer
 * @param prescale	Prescale value
 * @param compare	Comparison value
 */
void mml_tmr_setup_reloadmode(mml_tmr_regs_t *mml_timer, mml_tmr_prescale_t prescale, unsigned int compare);

/** @} */ /* @defgroup COBRA_TMR */

/* Variables ******************************************************************/

/* Functions ******************************************************************/
/** This function initializes specified timer ID with the configuration parameters. */
int mml_tmr_init(mml_tmr_id_t id, mml_tmr_config_t *config);
/** This function resets and closes the specified timer ID */
int mml_tmr_close(mml_tmr_id_t id);
/** This function resets all timer interfaces */
int mml_tmr_reset_interface(void);
/** This function configures the Timer ID with the new configuration values. */
int mml_tmr_config(mml_tmr_id_t id, mml_tmr_config_t *config);
/** This function enables (starts) the timer ID */
int mml_tmr_enable(mml_tmr_id_t id);
/** This function disables (stops) the timer ID */
int mml_tmr_disable(mml_tmr_id_t id);
/** This function reads the timers current value */
int mml_tmr_read(mml_tmr_id_t id, unsigned int *time);
/** This function provides the IOCLT commands for the timer configuration. */
int mml_tmr_ioctl(mml_tmr_id_t id, mml_tmr_ioctl_t cmd, void *data);
/** This function registers the interrupt handler function for the Timer ID */
int mml_tmr_callback_handler_register(mml_tmr_id_t id, mml_tmr_handler_t handler);
/** This function unregisters the interrupt handler function for the Timer ID */
int mml_tmr_callback_handler_unregister(mml_tmr_id_t id);

int mml_tmr_interrupt_enable(mml_tmr_id_t id);

int mml_tmr_interrupt_disable(mml_tmr_id_t id);

void mml_tmr_interrupt_clear(mml_tmr_id_t id);

void mml_tmr_clr_irq(mml_tmr_regs_t *timer);

#endif /* _MML_TMR_H_ */

/******************************************************************************/
/* EOF */
