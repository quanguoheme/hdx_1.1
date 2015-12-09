/*
 * mml_skbd.c --
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

/** @file mml_skbd.c SKBD core driver */

/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
#include <mml.h>
#include <mml_gpio_regs.h>
#include <mml_intc_regs.h>
#include <mml_intc.h>
#include <mml_gcr.h>
#include <cobra_defines.h>
#include <cobra_macros.h>
#include <cobra_functions.h>
/** Local includes */
#include <mml_skbd.h>
#include <mml_skbd_regs.h>


/** Variables ******************************************************************/
/** Keypad context info */
__attribute__((section(".bss"))) mml_skbd_context_t mml_skbd_context;

#ifdef _STAND_ALONE_DRIVER_SKBD_
/* This compilation flag is activated only in driver development context
* without any application using it.
* DO NOT define it in application/test context then. */
/******************************************************************************/
int main(void)
{
	/** We're done */
	return NO_ERROR;
}
#endif /* _STAND_ALONE_DRIVER_SKBD_ */


/***********************************************************************************************/
/**
 * The function is used to initialize the keypad controller.
 */
int mml_keypad_init(mml_skbd_config_t config)
{
	int											result = COMMON_ERR_UNKNOWN;

	if ( !mml_skbd_context.first_init )
	{
		unsigned int 							temp;
		/** Number of output pins */
		unsigned int							outputs = 0;
		volatile mml_gcr_regs_t					*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;
		volatile mml_skbd_regs_t				*reg_skbd = (volatile mml_skbd_regs_t*)MML_SKBD_IOBASE;
		volatile mml_gpio_regs_t				*reg_gpio = (volatile mml_gpio_regs_t*)MML_GPIO0_IOBASE;


		/** Check IRQ handler is NULL */
		if ( !config.irq_handler )
		{
			result = MML_SKBD_ERR_IRQ_NULL;
			goto mml_keypad_init_out;
		}
		/** Check for the I/O pin overlaps */
		if ( ( config.outputs & config.inputs ) ||
			!config.inputs ||
			!config.outputs )
		{
			result = MML_SKBD_ERR_INVALID_PIN_CONFIGURATION;
			goto mml_keypad_init_out;
		}
		/** Memset like procedure */
		memset((unsigned char*)&mml_skbd_context, 0x00, sizeof(mml_skbd_context));
#ifdef _SKBD_RESET_AT_INIT_
		/** Reset the keypad controller */
		reg_gcr->rstr |= MML_GCR_RSTR_KBD_MASK;
		/** Wait until keypad reset completes */
		while( MML_GCR_RSTR_KBD_MASK & reg_gcr->rstr );
#endif /* _SKBD_RESET_AT_INIT_ */
		/** Enable the keypad clock i.e. just in case clock is disabled */
		reg_gcr->perckcn &= ~(1 << MML_PERCKCN_DEV_KBD);
		/** I/O pin direction selection for the keypad pins */
		/** Configure keypad output pins */
		reg_skbd->cr0 |= config.outputs;
		/** Configure keypad input pins */
		reg_skbd->cr0 &= ~(config.inputs);
		/** Configure the keypad input/output pins(GPIO's) to Primary alternate functionality */
		reg_gpio->en &= ~( config.inputs | config.outputs );
		reg_gpio->en1 &= ~( config.inputs | config.outputs );
		reg_gpio->pad_cfg1 |= ( config.inputs | config.outputs );
		reg_gpio->pad_cfg2 &= ~( config.inputs | config.outputs );
		/** Count the number of output keypad lines */
		temp = config.outputs;
		while( temp )
		{
			temp &= ( temp - 1 );
			outputs++;
		}
		/** Configure the keypad  */
		temp = ( ( ( MML_SKBD_CR1_CLEAR_MASK_NOOFST & config.reg_erase )  << MML_SKBD_CR1_CLEAR_OFST ) | MML_SKBD_CR1_AUTOEN_MASK );
		temp |= ( config.debounce  << MML_SKBD_CR1_DBTM_OFST );
		temp |= ( outputs << MML_SKBD_CR1_OUTNB_OFST ) & MML_SKBD_CR1_OUTNB_MASK;
		reg_skbd->cr1 |= temp;
		while ( !( reg_skbd->sr & MML_SKBD_SR_BUSY_MASK ) );
		/** Setup IRQ */
		mml_intc_setup_irq(MML_INTNUM_SKBD, MML_INTC_PRIO_2,  config.irq_handler);
		/** To be done once only */
		mml_skbd_context.first_init = 1;
		mml_skbd_context.state = MML_SKBD_STATE_INITIALIZED;
		/** No error */
		result = NO_ERROR;
	}
	else
	{
		result = MML_SKBD_ERR_ALREAD_INITIALIZED;
	}
	/** We're done */
mml_keypad_init_out:
	return result;
}

/***********************************************************************************************/
/** Function is used to enable the interrupt events */
int mml_keypad_enable_interrupt_events(unsigned int events)
{
	int											result = COMMON_ERR_UNKNOWN;

	if ( MML_SKBD_STATE_INITIALIZED != mml_skbd_context.state )
	{
		result = MML_SKBD_ERR_NOT_INITIALIZED;
	}
	else
	{
		volatile mml_skbd_regs_t				*reg_skbd = (volatile mml_skbd_regs_t*)MML_SKBD_IOBASE;

		reg_skbd->ier |= events;
		/** No error */
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/***********************************************************************************************/
/** Function to disable the interrupt events */
int mml_keypad_disable_interrupt_events(unsigned int events)
{
	volatile mml_skbd_regs_t					*reg_skbd = (volatile mml_skbd_regs_t*)MML_SKBD_IOBASE;

	reg_skbd->ier &= ~events;
	/** We're done */
	return NO_ERROR;
}

/***********************************************************************************************/
/** Function is used to clear the interrupt events */
int mml_keypad_clear_interrupt_status(unsigned int status)
{
	volatile mml_skbd_regs_t					*reg_skbd = (volatile mml_skbd_regs_t*)MML_SKBD_IOBASE;

	reg_skbd->isr &= ~status;
	/** We're done */
	return NO_ERROR;
}

/***********************************************************************************************/
/** Function is used to read the interrupt status */
int mml_keypad_interrupt_status(unsigned int *status)
{
	volatile mml_skbd_regs_t					*reg_skbd = (volatile mml_skbd_regs_t*)MML_SKBD_IOBASE;

	*status = reg_skbd->isr;
	/** We're done */
	return NO_ERROR;
}

/***********************************************************************************************/
/** Function to read the key scan codes */
int mml_keypad_read_keys(mml_skbd_keys_t *keys)
{
	volatile unsigned short						*key = (unsigned short*)&keys->key0;
	volatile unsigned int						i = 0;
	volatile unsigned int						temp;
	volatile unsigned int						*key_reg;
	volatile mml_skbd_regs_t					*reg_skbd = (volatile mml_skbd_regs_t*)MML_SKBD_IOBASE;


	/**  */
	key_reg = (unsigned int*)&reg_skbd->key0;
	for( i = 0;i < MML_SKBD_TOTAL_KEY_REGS;i++ )
	{
		if ( !( reg_skbd->cr1 & MML_SKBD_CR1_CLEAR_MASK ) && ( reg_skbd->ier & MML_SKBD_IER_PUSHIE_MASK ) )
		{
			/** (!(*key_reg & 0xC00) || ((*key_reg & 0x400) && (!(MML_SKBD_K0R_READ_MASK & *key_reg)))) */
			if ( !( *key_reg & ( MML_SKBD_K0R_PUSH_MASK | MML_SKBD_K0R_READ_MASK ) ) )
			{
				*key++ = ( ( *key_reg & MML_SKBD_K0R_IOIN_MASK) |
							( ( *key_reg & MML_SKBD_K0R_IOOUT_MASK ) >> 1 )
							| MML_SKBD_KEY_PRESS_FLAG );
			}
		}
		else if ( !( reg_skbd->cr1 & MML_SKBD_CR1_CLEAR_MASK ) && ( reg_skbd->ier & MML_SKBD_IER_RELEASEIE_MASK ) )
		{
			temp = *key_reg;
			/**  */
			if ( ( temp & MML_SKBD_K0R_PUSH_MASK ) && !( temp & MML_SKBD_K0R_READ_MASK ) )
			{
				*key++ = ( ( *key_reg & MML_SKBD_K0R_IOIN_MASK ) |
							( ( *key_reg & MML_SKBD_K0R_IOOUT_MASK ) >> 1 ) |
							MML_SKBD_KEY_PRESS_FLAG );
			}
		}
		else
		{
			temp = *key_reg;
			/**  */
			if ( !( temp & MML_SKBD_K0R_READ_MASK ) )
			{
				*key++ = ( ( temp & MML_SKBD_K0R_IOIN_MASK ) |
						 ( ( temp & MML_SKBD_K0R_IOOUT_MASK ) >> 1 ) |
						 MML_SKBD_KEY_PRESS_FLAG );
			}
		}
		/**  */
		key_reg++;
	}
	/** We're done */
	return NO_ERROR;
}

/***********************************************************************************************/
/** Function to close the keypad */
int mml_keypad_close(void)
{
	volatile mml_gcr_regs_t						*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;

	/** Reset the keypad controller */
	reg_gcr->rstr |= MML_GCR_RSTR_KBD_MASK;
	/** Wait until keypad reset completes */
	while( MML_GCR_RSTR_KBD_MASK & reg_gcr->rstr );
	/**  */
	mml_intc_disable_irq(MML_INTNUM_SKBD);
	mml_skbd_context.state = MML_SKBD_STATE_CLOSED;
	mml_skbd_context.first_init = 0;
	/** We're done */
	return NO_ERROR;
}

/******************************************************************************/
/* EOF */
