/*
 * mml_tmr.c --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2012, Maxim Integrated
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
 * Created on: Jun 29, 2012
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

/** @file mml_tmr.c TMR core driver */

/** Global includes */
#include <config.h>
#include <errors.h>
#include <io.h>
/** Other includes */
#include <mml.h>
#include <mml_io.h>
#include <mml_gcr_regs.h>
#include <mml_intc.h>
#include <mml_intc_regs.h>
#include <cobra_defines.h>
/** Local includes */
#include <mml_tmr.h>
#include <mml_tmr_regs.h>


/** This structure is located in bss section, therefore it shall be zero-ized
 * at the very beginning of the appliaction */
__attribute__((section(".bss"))) mml_tmr_context_t tmr_context;

/** Local declarations */
int mml_tmr_mode_ctrl(mml_tmr_id_t id, mml_tmr_config_t config);
/** Macros ********************************************************************/
#define M_MML_CHECK_PARAMS(__tmr_conf__)	( ( ( MML_TMR_PRES_DIV_4096 < ((mml_tmr_config_t *)__tmr_conf__)->clock ) ||\
						    ( MML_TMR_MODE_CAPCOMP < ((mml_tmr_config_t *)__tmr_conf__)->mode ) ||\
						    ( MML_TMR_POLARITY_HIGH < ((mml_tmr_config_t *)__tmr_conf__)->polarity ) ) ? COMMON_ERR_OUT_OF_RANGE : NO_ERROR )

#ifdef _STAND_ALONE_DRIVER_TIMER_
/* This compilation flag is activated only in driver development context
* without any application using it.
* DO NOT define it in application/test context then. */
/*****************************************************************************/
int main(void)
{
	/** We're done */
	return NO_ERROR;
}
#endif /* _STAND_ALONE_DRIVER_TIMER_ */

/******************************************************************************/
/**
 * This function initializes specified timer ID with the configuration parameters.
 * @param[in] id					Timer device ID
 * @param[in] config				Points to configuration parameters
 * @return NO_ERROR					No error
 * @return COMMON_ERR_INVAL			Invalid timer ID
 * @return COMMON_ERR_NULL_PTR		configuration parameter is NULL
 * @return COMMON_ERR_OUT_OF_RANGE	One of the timer configuration parameter is invalid
 * @return COMMON_ERR_NO_MATCH		Timeout value is less than PWM value
 * @return COMMON_ERR_BAD_STATE		Timer already initialized
 * @return MML_TMR_ERR_UNKNOWN	Unknown error occurred
 */
int mml_tmr_init(mml_tmr_id_t id, mml_tmr_config_t *config)
{
	int										vector;
	int 									result = COMMON_ERR_UNKNOWN;
	unsigned int							tmr_clk_id = 0;
	mml_gcr_regs_t							*mml_gcr = (mml_gcr_regs_t*)MML_GCR_IOBASE;

	/** For the first time initialization */
	if ( !tmr_context.first_init )
	{
		/** Initialize the Timers context information to zero's */
		tmr_context.tmr[MML_TMR_DEV0].mml_tmr = (mml_tmr_regs_t*)MML_TMR0_IOBASE;
		tmr_context.tmr[MML_TMR_DEV1].mml_tmr = (mml_tmr_regs_t*)MML_TMR1_IOBASE;
		tmr_context.tmr[MML_TMR_DEV2].mml_tmr = (mml_tmr_regs_t*)MML_TMR2_IOBASE;
		tmr_context.tmr[MML_TMR_DEV3].mml_tmr = (mml_tmr_regs_t*)MML_TMR3_IOBASE;
		tmr_context.tmr[MML_TMR_DEV4].mml_tmr = (mml_tmr_regs_t*)MML_TMR4_IOBASE;
		tmr_context.tmr[MML_TMR_DEV5].mml_tmr = (mml_tmr_regs_t*)MML_TMR5_IOBASE;
		tmr_context.tmr[MML_TMR_DEV6].mml_tmr = (mml_tmr_regs_t*)MML_TMR6_IOBASE;
		tmr_context.tmr[MML_TMR_DEV7].mml_tmr = (mml_tmr_regs_t*)MML_TMR7_IOBASE;
#if _TIMER_RESET_AT_INIT_
		/** Reset all Timer's interface */
		mml_gcr->rstr |= MML_GCR_RSTR_TIMERS_MASK;
		/** Wait for UARTs reset to be done */
		while( MML_GCR_RSTR_TIMERS_MASK & mml_gcr->rstr );
		/** Stop clock of each UART. It'll be set independently */
		mml_gcr->perckcn |= ( 1 << MML_PERCKCN_DEV_T0 );
		mml_gcr->perckcn |= ( 1 << MML_PERCKCN_DEV_T1 );
		mml_gcr->perckcn |= ( 1 << MML_PERCKCN_DEV_T2 );
		mml_gcr->perckcn |= ( 1 << MML_PERCKCN_DEV_T3 );
		mml_gcr->perckcn |= ( 1 << MML_PERCKCN_DEV_T4 );
		mml_gcr->perckcn |= ( 1 << MML_PERCKCN_DEV_T5 );
		mml_gcr->perckcn |= ( 1 << MML_PERCKCN_DEV_T6 );
		mml_gcr->perckcn |= ( 1 << MML_PERCKCN_DEV_T7 );
#endif /* _TIMER_RESET_AT_INIT_ */
		tmr_context.first_init = 1;
	}
	/** Validate timer identifier */
	switch ( id )
	{
		case MML_TMR_DEV0:
			vector = MML_INTNUM_TMR0;
			tmr_clk_id = MML_PERCKCN_DEV_T0;
			break;
		case MML_TMR_DEV1:
			vector = MML_INTNUM_TMR1;
			tmr_clk_id = MML_PERCKCN_DEV_T1;
			break;
		case MML_TMR_DEV2:
			vector = MML_INTNUM_TMR2;
			tmr_clk_id = MML_PERCKCN_DEV_T2;
			break;
		case MML_TMR_DEV3:
			vector = MML_INTNUM_TMR3;
			tmr_clk_id = MML_PERCKCN_DEV_T3;
			break;
		case MML_TMR_DEV4:
			vector = MML_INTNUM_TMR4;
			tmr_clk_id = MML_PERCKCN_DEV_T4;
			break;
		case MML_TMR_DEV5:
			vector = MML_INTNUM_TMR5;
			tmr_clk_id = MML_PERCKCN_DEV_T5;
			break;
		case MML_TMR_DEV6:
			vector = MML_INTNUM_TMR6;
			tmr_clk_id = MML_PERCKCN_DEV_T6;
			break;
		case MML_TMR_DEV7:
			vector = MML_INTNUM_TMR7;
			tmr_clk_id = MML_PERCKCN_DEV_T7;
			break;

		default:
			result = COMMON_ERR_INVAL;
			goto mml_tmr_init_out;
	}
	/** Check state */
	if ( MML_TMR_STATE_INITIALIZED == tmr_context.tmr[id].state )
	{
		result = COMMON_ERR_BAD_STATE;
	}
	else if ( !config || !config->handler )
	{
		result = COMMON_ERR_NULL_PTR;
	}
	/** Validate the configuration parameters */
	else if ( NO_ERROR == M_MML_CHECK_PARAMS(config) )
	{
		volatile mml_tmr_regs_t					*mml_tmr = (volatile mml_tmr_regs_t*)tmr_context.tmr[id].mml_tmr;

		/** Enable the timer clock */
		/** Clear the bit position to enable clock to timer device */
		mml_gcr->perckcn &= ~( 1 << tmr_clk_id );
		/** Default value (Timer is disabled) */
		mml_tmr->control = 0;
		mml_tmr->interrupt = MML_TMR_INT_CLR_MASK;
		mml_tmr->count = config->count;
		mml_tmr->compare = config->timeout;
		result = mml_tmr_mode_ctrl(id, *config);
		if ( NO_ERROR == result )
		{
			result = mml_intc_setup_irq(vector, MML_INTC_PRIO_2, config->handler);
			if ( result )
			{
				result = MML_TMR_ERR_IRQ_SET;
			}
			else
			{
				tmr_context.tmr[id].state = MML_TMR_STATE_INITIALIZED;
				tmr_context.tmr[id].status = MML_TMR_STATE_DISABLED;
			}
		}
	}
	/** We're done */
mml_tmr_init_out:
	return result;
}

/******************************************************************************/
/**
 * This function resets and closes the specified timer ID
 * @param[in] id					Timer ID
 * @return NO_ERROR					No error
 * @return COMMON_ERR_INVAL			Invalid timer ID
 * @return COMMON_ERR_BAD_STATE		Timer not initialized or timer is enabled
 * @return MML_TMR_ERR_UNKNOWN	Unknown error occurred
 */
int mml_tmr_close(mml_tmr_id_t id)
{
	int 									result = COMMON_ERR_UNKNOWN;

	/* Validate the timer id */
	if ( MML_TMR_DEV_COUNT <= id )
	{
		result = COMMON_ERR_INVAL;
	}
	if ( MML_TMR_STATE_INITIALIZED != tmr_context.tmr[id].state )
	{
		result = COMMON_ERR_BAD_STATE;
	}
	else if ( MML_TMR_STATE_ENABLED == tmr_context.tmr[id].status )
	{
		result = MML_TMR_ERR_NOT_ALLOWED;
	}
	else
	{
		unsigned int						tmp;
		volatile mml_gcr_regs_t			*mml_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;

		/** Disable the timer clock */
		tmp = mml_gcr->perckcn;
		/** Set the bit position */
		tmp |= ( ( 0x1 << id ) << MML_TMR_ACTIVE_OFST );
		mml_gcr->perckcn = tmp;
		/** Timer state is not initialized */
		tmr_context.tmr[id].state = MML_TMR_STATE_NOT_INITIALIZED;
		/** Timer has already been disabled */
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
int mml_tmr_reset_interface(void)
{
	unsigned int							loop = K_COBRA_RESET_WAIT_LOOP_MAX;
	volatile mml_gcr_regs_t				*mml_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;

	/** Launch all UARTs reset */
	mml_gcr->rstr |= MML_GCR_RSTR_TIMERS_MASK;
	/** Wait 'til it's done */
	while( ( mml_gcr->rstr & MML_GCR_RSTR_TIMERS_MASK ) && loop-- );
	/** We're done */
	return NO_ERROR;
}

/******************************************************************************/
/**
 * This function configures the Timer ID with the new configuration values.
 * Timer must not be running(enabled) before calling this function.
 * @param id								Timer device id
 * @param config							Points to configuration parameters
 * @return NO_ERROR							No error
 * @return COMMON_ERR_INVAL					Invalid timer ID
 * @return COMMON_ERR_NULL_PTR				configuration parameter is NULL
 * @return COMMON_ERR_OUT_OF_RANGE			One of the timer configuration parameter is invalid
 * @return COMMON_ERR_NO_MATCH				Timeout value is less than PWM value
 * @return MML_TMR_ERR_NOT_INITIALIZED	Timer ID is not initialized or Timer is enabled
 * @return MML_TMR_ERR_UNKNOWN			Unknown error occurred
 */
int mml_tmr_config(mml_tmr_id_t id, mml_tmr_config_t *config)
{
	int										vector;
	int				 						result = COMMON_ERR_UNKNOWN;

	/** Validate timer identifier */
	switch ( id )
	{
		case MML_TMR_DEV0:
			vector = MML_INTNUM_TMR0;
			break;
		case MML_TMR_DEV1:
			vector = MML_INTNUM_TMR1;
			break;
		case MML_TMR_DEV2:
			vector = MML_INTNUM_TMR2;
			break;
		case MML_TMR_DEV3:
			vector = MML_INTNUM_TMR3;
			break;
		case MML_TMR_DEV4:
			vector = MML_INTNUM_TMR4;
			break;
		case MML_TMR_DEV5:
			vector = MML_INTNUM_TMR5;
			break;
		default:
			result = COMMON_ERR_INVAL;
			goto mml_tmr_config_out;
	}
	/** Check state */
	if ( MML_TMR_STATE_INITIALIZED != tmr_context.tmr[id].state )
	{
		result = MML_TMR_ERR_NOT_INITIALIZED;
	}
	else if ( MML_TMR_STATE_ENABLED == tmr_context.tmr[id].status )
	{
		result = MML_TMR_ERR_NOT_ALLOWED;
	}
	else if ( !config || !config->handler )
	{
		result = COMMON_ERR_NULL_PTR;
	}
	/** Validate the configuration parameters */
	else if ( NO_ERROR == M_MML_CHECK_PARAMS(config) )
	{
		result = mml_tmr_mode_ctrl(id, *config);
		if ( NO_ERROR == result )
		{
			mml_tmr_interrupt_clear(id);
			result = mml_intc_setup_irq(vector, MML_INTC_PRIO_2, config->handler);
			if ( result )
			{
				result = MML_TMR_ERR_IRQ_SET;
			}
		}
	}
	/** We're done */
mml_tmr_config_out:
	return result;
}

/******************************************************************************/
/**
 * This function enables (starts) the timer ID
 * @param id								Timer device id
 * @return NO_ERROR							No error
 * @return COMMON_ERR_INVAL					Invalid timer ID
 * @return MML_TMR_ERR_NOT_INITIALIZED	Timer ID is not initialized
 * 											or Timer is already enabled
 * @return MML_TMR_ERR_UNKNOWN			Unknown error occurred
 */
int mml_tmr_enable(mml_tmr_id_t id)
{
	int 									result = COMMON_ERR_UNKNOWN;

	/** Validate the timer id */
	if ( MML_TMR_DEV_COUNT <= id )
	{
		result = COMMON_ERR_INVAL;
	}
	else if ( MML_TMR_STATE_INITIALIZED != tmr_context.tmr[id].state )
	{
		result = MML_TMR_ERR_NOT_INITIALIZED;
	}
	else if ( MML_TMR_STATE_ENABLED == tmr_context.tmr[id].status )
	{
		result = MML_TMR_ERR_NOT_ALLOWED;
	}
	else
	{
		volatile mml_tmr_regs_t			*mml_tmr = (volatile mml_tmr_regs_t*)tmr_context.tmr[id].mml_tmr;

		/** Then enable it */
		mml_tmr->control |= ( MML_TMR_CN_TEN_ENABLE << MML_TMR_CN_TEN_OFST );
		tmr_context.tmr[id].status = MML_TMR_STATE_ENABLED;
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * This function disables (stops) the timer channel
 * @param id								Timer device id
 * @return NO_ERROR							No error
 * @return COMMON_ERR_INVAL					Invalid timer ID
 * @return MML_TMR_ERR_NOT_INITIALIZED	Timer ID is not initialized
 * 											or Timer is already in disabled status
 * @return MML_TMR_ERR_UNKNOWN			Unknown error occurred
 * @return
 */
int mml_tmr_disable(mml_tmr_id_t id)
{
	int 									result = COMMON_ERR_UNKNOWN;

	/* Validate the timer id */
	if ( MML_TMR_DEV_COUNT <= id )
	{
		result = COMMON_ERR_INVAL;
	}
	else if ( MML_TMR_STATE_INITIALIZED != tmr_context.tmr[id].state )
	{
		result = MML_TMR_ERR_NOT_INITIALIZED;
	}
	else
	{
		volatile mml_tmr_regs_t			*mml_tmr = (volatile mml_tmr_regs_t*)tmr_context.tmr[id].mml_tmr;

		mml_tmr->control &= ~MML_TMR_CN_TEN_MASK;
		tmr_context.tmr[id].status = MML_TMR_STATE_DISABLED;
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * This function reads the timer ID current value
 * @param id								Timer device ID
 * @param time								Timer current value
 * @return NO_ERROR							No error
 * @return COMMON_ERR_INVAL					Invalid timer ID
 * @return MML_TMR_ERR_NOT_INITIALIZED	Timer ID is not initialized
 * @return MML_TMR_ERR_UNKNOWN			Unknown error occurred
 */
int mml_tmr_read(mml_tmr_id_t id, unsigned int *time)
{
	int 									result = COMMON_ERR_UNKNOWN;

	/* Validate the timer id */
	if ( MML_TMR_DEV_COUNT <= id )
	{
		result = COMMON_ERR_INVAL;
	}
	else if ( MML_TMR_STATE_NOT_INITIALIZED == tmr_context.tmr[id].state )
	{
		result = MML_TMR_ERR_NOT_INITIALIZED;
	}
	else if ( !time )
	{
		result = COMMON_ERR_NULL_PTR;
	}
	else
	{
		*time = tmr_context.tmr[id].mml_tmr->count;
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * This function provides the IOCLT commands for the timer configuration.
 * @param[In] id							Timer device ID
 * @param[In] cmd							Timer ioctl commands to be performed
 * @param[In/Out] data						Input/Output parameters
 * @return NO_ERROR							No error
 * @return COMMON_ERR_INVAL					Invalid timer ID
 * @return MML_TMR_ERR_NOT_INITIALIZED	Timer ID is not initialized
 * 											or Timer is in enabled status.
 * @return COMMON_ERR_NO_MATCH				Invalid IOCTL command
 * @return COMMON_ERR_OUT_OF_RANGE			Invalid timer configuration parameter
 * @return MML_TMR_ERR_UNKNOWN			Unknown error occurred
 */
int mml_tmr_ioctl(mml_tmr_id_t id, mml_tmr_ioctl_t cmd, void *data)
{
	int 									result = COMMON_ERR_UNKNOWN;


	/* Validate the timer id */
	if ( MML_TMR_DEV_COUNT <= id )
	{
		result = COMMON_ERR_INVAL;
	}
	else if ( MML_TMR_STATE_INITIALIZED != tmr_context.tmr[id].state )
	{
		result = MML_TMR_ERR_NOT_INITIALIZED;
	}
	else if ( MML_TMR_STATE_ENABLED == tmr_context.tmr[id].status )
	{
		result = MML_TMR_ERR_NOT_ALLOWED;
	}
	else if ( !data )
	{
		result = COMMON_ERR_NULL_PTR;
	}
	else
	{
		unsigned int 						temp;
		unsigned int						tmp = 0;
		mml_tmr_config_t 					*config;
		volatile mml_tmr_regs_t			*mml_tmr = (volatile mml_tmr_regs_t*)tmr_context.tmr[id].mml_tmr;

		switch( cmd )
		{
			case MML_TMR_CMD_SET_PRESCALE:
				temp = *(mml_tmr_prescale_t*)data;
				if ( MML_TMR_PRES_DIV_4096 < temp )
				{
					result = COMMON_ERR_OUT_OF_RANGE;
				}
				else
				{
					/** Retrieve "old" control register value */
					tmp = mml_tmr->control;
					/** Remove presacler */
					tmp &= ~MML_TMR_CN_PRES_MASK;
					/** Set new value */
					/** TMR0CTRL[5:3] = PRES[2:0] */
					tmp |= ( ( temp & 0x07 ) << MML_TMR_CN_PRES_OFST );
					/** TMR0CTRL[8] = PRES[3] */
					tmp |= ( ( temp & 0x08 ) << 8 );
					mml_tmr->control = tmp;
					/** No error */
					result = NO_ERROR;
				}
				break;
			case MML_TMR_CMD_SET_MODE:
				config = (mml_tmr_config_t*)data;
				if ( MML_TMR_MODE_CAPCOMP < config->mode )
				{
					result = COMMON_ERR_OUT_OF_RANGE;
				}
				else if ( ( MML_TMR_MODE_PWM == config->mode ) &&
					( tmr_context.tmr[id].mml_tmr->compare < config->pwm_value ) )
				{
					result = COMMON_ERR_OUT_OF_RANGE;
				}
				else
				{
					if ( MML_TMR_MODE_PWM == config->mode )
					{
						mml_tmr->pwm = config->pwm_value;
					}
					/** Retrieve "old" control register value */
					tmp = mml_tmr->control;
					/** Remove mode */
					tmp &= ~MML_TMR_CN_TMODE_MASK;
					/** Set new value */
					/** TMR0CTRL[2:0] = TMODE[2:0] */
					tmp |= config->mode;
					mml_tmr->control = tmp;
					/** No error */
					result = NO_ERROR;
				}
				break;
			case MML_TMR_CMD_SET_IO_POLARITY:
				temp = *(mml_tmr_polarity_t*)data;
				if ( MML_TMR_POLARITY_HIGH < temp )
				{
					result = COMMON_ERR_OUT_OF_RANGE;
				}
				else
				{
					/** Retrieve "old" control register value */
					tmp = mml_tmr->control;
					/** Remove polarity */
					tmp &= ~MML_TMR_CN_TPOL_MASK;
					/** Set new value */
					tmp |= ( temp << MML_TMR_CN_TPOL_OFST );
					mml_tmr->control = tmp;
					/** No error */
					result = NO_ERROR;
				}
				break;
			default:
				result = COMMON_ERR_NO_MATCH;
				break;
		}
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * This function registers the interrupt handler function for the
 * specified Timer ID device.
 * @param[In] id							Timer device ID
 * @param[In] handler						Interrupt handler to be registered
 * @return NO_ERROR							No error
 * @return COMMON_ERR_INVAL					Invalid timer ID
 * @return MML_TMR_ERR_NOT_INITIALIZED	Timer ID is not initialized
 * @return COMMON_ERR_NULL_PTR				Null handler pointer passed
 * @return MML_TMR_ERR_UNKNOWN			Unknown error occurred
 */
int mml_tmr_callback_handler_register(mml_tmr_id_t id, mml_tmr_handler_t handler)
{
	int										vector;
	int 									result = COMMON_ERR_UNKNOWN;

	/** Check input parameter */
	switch( id )
	{
		case MML_TMR_DEV0:
			vector = MML_INTNUM_TMR0;
			break;
		case MML_TMR_DEV1:
			vector = MML_INTNUM_TMR1;
			break;
		case MML_TMR_DEV2:
			vector = MML_INTNUM_TMR2;
			break;
		case MML_TMR_DEV3:
			vector = MML_INTNUM_TMR3;
			break;
		case MML_TMR_DEV4:
			vector = MML_INTNUM_TMR4;
			break;
		case MML_TMR_DEV5:
			vector = MML_INTNUM_TMR5;
			break;
		default:
			result = COMMON_ERR_INVAL;
			goto mml_tmr_callback_handler_register_out;
	}
	/** Check state */
	if ( MML_TMR_STATE_INITIALIZED != tmr_context.tmr[id].state )
	{
		result = MML_TMR_ERR_NOT_INITIALIZED;
	}
	else if ( MML_TMR_STATE_ENABLED == tmr_context.tmr[id].status )
	{
		result = MML_TMR_ERR_NOT_ALLOWED;
	}
	else if ( !handler )
	{
		result = COMMON_ERR_NULL_PTR;
	}
	else
	{
		mml_intc_attach_irq(vector, MML_INTC_PRIO_2, handler);
		result = NO_ERROR;
	}
	/** We're done */
mml_tmr_callback_handler_register_out:
	return result;
}

/******************************************************************************/
/**
 * This function unregisters the interrupt handler function for the
 * specified Timer ID device if any.
 * @param[In] id							Timer device ID
 * @param[In] handler						Interrupt handler to be registered
 * @return NO_ERROR							No error
 * @return COMMON_ERR_INVAL					Invalid timer ID
 * @return MML_TMR_ERR_NOT_INITIALIZED	Timer ID is not initialized
 * @return MML_TMR_ERR_UNKNOWN			Unknown error occurred
 */
int mml_tmr_callback_handler_unregister(mml_tmr_id_t id)
{
	int										vector;
	int 									result = COMMON_ERR_UNKNOWN;

	/** Check input parameter */
	switch( id )
	{
		case MML_TMR_DEV0:
			vector = MML_INTNUM_TMR0;
			break;
		case MML_TMR_DEV1:
			vector = MML_INTNUM_TMR1;
			break;
		case MML_TMR_DEV2:
			vector = MML_INTNUM_TMR2;
			break;
		case MML_TMR_DEV3:
			vector = MML_INTNUM_TMR3;
			break;
		case MML_TMR_DEV4:
			vector = MML_INTNUM_TMR4;
			break;
		case MML_TMR_DEV5:
			vector = MML_INTNUM_TMR5;
			break;
		default:
			result = COMMON_ERR_INVAL;
			goto mml_tmr_callback_handler_unregister_out;
	}
	/** Check state */
	if ( MML_TMR_STATE_INITIALIZED != tmr_context.tmr[id].state )
	{
		result = MML_TMR_ERR_NOT_INITIALIZED;
	}
	else if ( MML_TMR_STATE_ENABLED == tmr_context.tmr[id].status )
	{
		result = MML_TMR_ERR_NOT_ALLOWED;
	}
	else
	{
		mml_intc_detach_irq(vector);
		result = NO_ERROR;
	}
	/** We're done */
mml_tmr_callback_handler_unregister_out:
	return result;
}

/******************************************************************************/
/**
 * [TBD]
 * @param id	Timer device id
 * @return
 */
int mml_tmr_interrupt_enable(mml_tmr_id_t id)
{
	int										vector;
	int										result = COMMON_ERR_UNKNOWN;

	/** Check input parameter */
	switch( id )
	{
		case MML_TMR_DEV0:
			vector = MML_INTNUM_TMR0;
			break;
		case MML_TMR_DEV1:
			vector = MML_INTNUM_TMR1;
			break;
		case MML_TMR_DEV2:
			vector = MML_INTNUM_TMR2;
			break;
		case MML_TMR_DEV3:
			vector = MML_INTNUM_TMR3;
			break;
		case MML_TMR_DEV4:
			vector = MML_INTNUM_TMR4;
			break;
		case MML_TMR_DEV5:
			vector = MML_INTNUM_TMR5;
			break;
		case MML_TMR_DEV6:
			vector = MML_INTNUM_TMR6;
			break;
		case MML_TMR_DEV7:
			vector = MML_INTNUM_TMR7;
			break;
		default:
			result = MML_TMR_ERR_IRQ_SET;
			goto mml_tmr_interrupt_enable_out;
	}
	/** Enable timer's irq */
	mml_intc_enable_irq(vector);
	result = NO_ERROR;
	/** We're done */
mml_tmr_interrupt_enable_out:
	return result;
}

/******************************************************************************/
/**
 * [TBD]
 * @param id	Timer device id
 * @return
 */
int mml_tmr_interrupt_disable(mml_tmr_id_t id)
{
	int										vector;
	int										result = COMMON_ERR_UNKNOWN;

	/** Check input parameter */
	switch( id )
	{
		case MML_TMR_DEV0:
			vector = MML_INTNUM_TMR0;
			break;
		case MML_TMR_DEV1:
			vector = MML_INTNUM_TMR1;
			break;
		case MML_TMR_DEV2:
			vector = MML_INTNUM_TMR2;
			break;
		case MML_TMR_DEV3:
			vector = MML_INTNUM_TMR3;
			break;
		case MML_TMR_DEV4:
			vector = MML_INTNUM_TMR4;
			break;
		case MML_TMR_DEV5:
			vector = MML_INTNUM_TMR5;
			break;
		case MML_TMR_DEV6:
			vector = MML_INTNUM_TMR6;
			break;
		case MML_TMR_DEV7:
			vector = MML_INTNUM_TMR7;
			break;
		default:
			result = MML_TMR_ERR_IRQ_SET;
			goto mml_tmr_interrupt_disable_out;
	}
	/** Disable timer's irq */
	mml_intc_disable_irq(vector);
	result = NO_ERROR;
	/** We're done */
mml_tmr_interrupt_disable_out:
	return result;
}

/******************************************************************************/
/**
 * [TBD]
 * @param id	Timer device id
 * @return
 */
void mml_tmr_interrupt_clear(mml_tmr_id_t id)
{
	int					vector;
	volatile mml_tmr_regs_t		*mml_tmr = (volatile mml_tmr_regs_t*)tmr_context.tmr[id].mml_tmr;

	/** Check input parameter */
	switch( id )
	{
		case MML_TMR_DEV0:
			vector = MML_INTNUM_TMR0;
			break;
		case MML_TMR_DEV1:
			vector = MML_INTNUM_TMR1;
			break;
		case MML_TMR_DEV2:
			vector = MML_INTNUM_TMR2;
			break;
		case MML_TMR_DEV3:
			vector = MML_INTNUM_TMR3;
			break;
		case MML_TMR_DEV4:
			vector = MML_INTNUM_TMR4;
			break;
		case MML_TMR_DEV5:
			vector = MML_INTNUM_TMR5;
			break;
		case MML_TMR_DEV6:
			vector = MML_INTNUM_TMR6;
			break;
		case MML_TMR_DEV7:
			vector = MML_INTNUM_TMR7;
			break;
		default:
			goto mml_tmr_interrupt_clear_out;
	}
	/** Clear the source level interrupt */
	mml_tmr->interrupt = MML_TMR_INT_CLR_MASK;
	mml_intc_ack_irq(vector);
	/** We're done */
mml_tmr_interrupt_clear_out:
	return;
}

/******************************************************************************/
int mml_tmr_mode_ctrl(mml_tmr_id_t id, mml_tmr_config_t config)
{
	int											result = COMMON_ERR_UNKNOWN;
	unsigned int								tmp;
	volatile mml_tmr_regs_t						*mml_tmr = (volatile mml_tmr_regs_t*)tmr_context.tmr[id].mml_tmr;

	/** First, check that timer is NOT enable */
	/** Retrieve old value ... */
	tmp = mml_tmr->control;
	if ( tmp & MML_TMR_CN_TEN_MASK )
	{
		result = MML_TMR_ERR_ENABLE;
	}
	/**  Check timer mode */
	else if ( ( MML_TMR_MODE_PWM == config.mode ) &&
			( config.timeout < config.pwm_value ) )
	{
		result = COMMON_ERR_NO_MATCH;
	}
	else
	{
		mml_tmr->compare = config.timeout;
		mml_tmr->count = config.count;
		/**  */
		if ( MML_TMR_MODE_PWM == config.mode )
		{
			mml_tmr->pwm = config.pwm_value;
		}
		/** ... to remove mode ... */
		tmp &= ~MML_TMR_CN_TMODE_MASK;
		/** ... polarity */
		tmp &= ~MML_TMR_CN_TPOL_MASK;
		/** ... and prescaler */
		tmp &= ~MML_TMR_CN_PRES_MASK;
		/** Now, set new value */
		/** TMR0CTRL[2:0] = TMODE[2:0] */
		tmp |= config.mode;
		/** TMR0CTRL[5:3] = PRES[2:0] */
		tmp |= ( ( config.clock & 0x07 ) << MML_TMR_CN_PRES_OFST );
		/** TMR0CTRL[8] = PRES[3] */
		tmp |= ( ( config.clock & 0x08 ) << 8 );
		tmp |= ( config.polarity << MML_TMR_CN_TPOL_OFST );
		mml_tmr->control = tmp;
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/** COBRA adaptation layer */
/**
 *
 * @param timer
 * @param prescale
 * @param compare
 * @return
 */
void mml_tmr_setup_reloadmode(mml_tmr_regs_t *mml_tmr, mml_tmr_prescale_t prescale, unsigned compare)
{
	/** Disable the timer (TEN = 0). */
	IO_REG_BF_DISABLE(&mml_tmr->control, MML_TMR_CN_TEN);
	/** Configure the timer for Continuous mode. */
	IO_REG_BF_SET_VALNAME(&mml_tmr->control, MML_TMR_CN_TMODE, CONTINUOUS);
	/** Set the prescale value (PRES). */
	IO_REG_BF_SET_VALUE(&mml_tmr->control, MML_TMR_CN_PRES, prescale);
	/** Write to the Timer register to set the starting count value (reset value). */
	mml_tmr->count = 0x0001;
	/** Write to the Timer Compare register to set the Compare value. */
	mml_tmr->compare = compare;
	/** Enable the timer (TEN = 1) */
	IO_REG_BF_ENABLE(&mml_tmr->control, MML_TMR_CN_TEN);
	/** We're done */
	return;
}

/******************************************************************************/
/** Clear the interrupt
 *
 * @param timer Timer
 */
void mml_tmr_clr_irq(mml_tmr_regs_t *mml_tmr)
{
	mml_tmr->interrupt = MML_TMR_INT_CLR_MASK;
	/** We're done */
	return;
}

/******************************************************************************/
/* EOF */

