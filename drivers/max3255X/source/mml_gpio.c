/*
 * mml_gpio.c --
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

/** @file mml_gpio.c GPIO core driver */

/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
#include <mml.h>
#include <mml_gcr_regs.h>
#include <cobra_defines.h>
#include <cobra_macros.h>
#include <cobra_functions.h>
/** Local includes */
#include <mml_gpio.h>
#include <mml_gpio_regs.h>

/** GPIO device ID context information */
__attribute__((section(".bss"))) mml_gpio_context_t gpio_context;


#ifdef _STAND_ALONE_DRIVER_GPIO_
/* This compilation flag is activated only in driver development context
* without any application using it.
* DO NOT define it in application/test context then. */
/*****************************************************************************/
int main(void)
{
	/** We're done */
	return NO_ERROR;
}
#endif /* _STAND_ALONE_DRIVER_GPIO_ */

/*****************************************************************************/
/**
 * The function is used to initialize and configure the consecutive 'n' GPIO
 * pins starting from the 'offset' position.
 * @param[in] dev_id						The GPIO device identifier
 * @param[in] offset						Pin offset in the range of 0 to 31
 * @param[in] bits_count					Specifies number of consecutive pins
 * 											starting from 'offset' position.
 * @param[in] config						GPIO configuration
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device identifier
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		Invalid parameter
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_init(mml_gpio_id_t dev_id,
					int offset,
					int bits_count,
					mml_gpio_config_t config)
{
	int											result = COMMON_ERR_UNKNOWN;
#ifdef _GPIO_RESET_AT_INIT_
	volatile mml_gcr_regs_t						*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;
#endif /* _GPIO_RESET_AT_INIT_ */

	/** First initialization ? */
	if ( !gpio_context.first_init )
	{
		gpio_context.port[MML_GPIO_DEV0].reg = (mml_gpio_regs_t*)MML_GPIO0_IOBASE;
		gpio_context.port[MML_GPIO_DEV1].reg = (mml_gpio_regs_t*)MML_GPIO1_IOBASE;
		gpio_context.port[MML_GPIO_DEV2].reg = (mml_gpio_regs_t*)MML_GPIO2_IOBASE;
#ifdef _GPIO_RESET_AT_INIT_
		/** Reset all GPIOs' interface */
		reg_gcr->rstr |= MML_GCR_RSTR_GPIO_MASK;
		/** Wait for UARTs reset to be done */
		while( MML_GCR_RSTR_GPIO_MASK & reg_gcr->rstr );
#endif /* _GPIO_RESET_AT_INIT_ */
		/** To be done once only */
		gpio_context.first_init = 1;
	}
	/** Check input parameters */
	result = mml_gpio_check(dev_id, offset, bits_count);
	if ( result )
	{
		goto mml_gpio_init_out;
	}
	/** Clear OUT, IRQ and WAKE_EN for all bits
	 * it will be done once again for 'offset' and
	 * 'bits_count' into set_config */
	mml_gpio_clear_out_int_wake(dev_id, offset, bits_count);
	/**  */
	result = mml_gpio_set_config(dev_id, offset, bits_count, config);
	if ( NO_ERROR == result )
	{
		gpio_context.port[dev_id].state = MML_GPIO_STATE_INITIALIZED;
	}
	/** We're done */
mml_gpio_init_out:
	return result;
}

/******************************************************************************/
int mml_gpio_reset_interface(void)
{
	unsigned int								loop = K_COBRA_RESET_WAIT_LOOP_MAX;
	volatile mml_gcr_regs_t						*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;

	/** Launch all GPIOs reset */
	reg_gcr->rstr |= ( MML_RSTR_DEV_GPIO0 | MML_RSTR_DEV_GPIO1 | MML_RSTR_DEV_GPIO2 );
	/** Wait until it's done */
	while( ( reg_gcr->rstr & ( MML_RSTR_DEV_GPIO0 | MML_RSTR_DEV_GPIO1 | MML_RSTR_DEV_GPIO2 ) ) && loop-- );
	/** We're done */
	return NO_ERROR;
}

/*****************************************************************************/
/**
 * This function closes the GPIO device ID also resets and disables
 * the clock for the GPIO device ID.
 * @param[in] dev_id						The GPIO device identifier
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device identifier
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_close(mml_gpio_id_t dev_id)
{
	int											result = COMMON_ERR_UNKNOWN;
#ifdef _INPUT_STRICT_CHECK_

	/** Validate the input parameter dev_id */
	if ( MML_GPIO_DEV_COUNT <= dev_id )
	{
		result = MML_GPIO_ERR_INVALID_DEVICE;
		goto mml_gpio_close_out;
	}
#endif /* _INPUT_STRICT_CHECK_ */
	/** Clear OUT, IRQ and WAKE_EN */
	mml_gpio_clear_out_int_wake(dev_id, 0, MML_GPIO_BIT_RANGE_NB);
	/* Disable GPIO's output */
	result = mml_gpio_disable_output(dev_id, 0, MML_GPIO_BIT_RANGE_NB);
	if ( NO_ERROR == result )
	{
		/** RESET not supported by MQ55 currently */
		gpio_context.port[dev_id].state = MML_GPIO_STATE_NOT_INITIALIZED;
	}
	/** We're done */
#ifdef _INPUT_STRICT_CHECK_
mml_gpio_close_out:
#endif /* _INPUT_STRICT_CHECK_ */
	return result;
}

/*****************************************************************************************/
/**
 * The function re-configures an already initialized GPIO device ID with new configuration
 * @param[in] dev_id						The GPIO device identifier
 * @param[in] offset						Pin offset in the range of 0 to 31
 * @param[in] bits_count					Specifies number of consecutive pins
 * 											starting from 'offset' position.
 * @param[in] config						GPIO configuration
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO ID
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		Invalid parameter
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_set_config(mml_gpio_id_t dev_id,
							int offset,
							int bits_count,
							mml_gpio_config_t config)
{
	int											result = COMMON_ERR_UNKNOWN;

	/** Check input parameters */
	result = mml_gpio_check(dev_id, offset, bits_count);
	if ( result )
	{
		goto mml_gpio_config_out;
	}
	/** Clear OUT, IRQ and WAKE_EN */
	mml_gpio_clear_out_int_wake(dev_id, offset, bits_count);
	/** Disable GPIO's output */
	result = mml_gpio_disable_output(dev_id, offset, bits_count);
	if ( result )
	{
		goto mml_gpio_config_out;
	}
	/** Configure the PAD */
	result = mml_gpio_pad_configure(dev_id, config.gpio_pad_config, offset, bits_count);
	if ( result )
	{
		goto mml_gpio_config_out;
	}
	/** Configure GPIO function for range of pins */
	result = mml_gpio_configure_function(dev_id, config.gpio_function, offset, bits_count);
	if ( ( NO_ERROR == result ) &&
		( MML_GPIO_DIR_OUT == config.gpio_direction ) )
	{
		/** Enable GPIO's output */
		result = mml_gpio_enable_output(dev_id, offset, bits_count);
	}
	/** We're done */
mml_gpio_config_out:
	return result;
}

/*******************************************************************************/
/**
 * The function configures the GPIO PAD configuration information
 * @param[in] dev_id						The GPIO device identifier
 * @param[in] config						GPIO pad configuration information
 * @param[in] offset						Pin offset in the range of 0 to 31
 * @param[in] bits_count					Specifies number of consecutive pins
 * 											starting from 'offset' position.
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device identifier
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		One or more parameters are invalid
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_pad_configure(mml_gpio_id_t dev_id,
							mml_gpio_pad_config_t config,
							int offset,
							int bits_count)
{
	int											result = COMMON_ERR_UNKNOWN;

	result = mml_gpio_check(dev_id, offset, bits_count);
	if ( NO_ERROR == result )
	{
		unsigned int							val1;
		unsigned int							val2;
		/** Create Mask */
		unsigned int 							mask = ~( ~0 << bits_count );
		volatile mml_gpio_regs_t				*reg_gpio = (volatile mml_gpio_regs_t*)gpio_context.port[dev_id].reg;


		/** Read the current values */
		val1 = reg_gpio->pad_cfg1;
		val2 = reg_gpio->pad_cfg2;
		switch( config )
		{
			case MML_GPIO_PAD_NORMAL:
				/** Clear the corresponding n-bits from position offset */
				val1 &= ~( mask << offset );
				val2 &= ~( mask << offset );
				break;
			case MML_GPIO_PAD_PULLUP:
				/** Set the corresponding n-bits from position offset */
				val1 |= ( mask << offset );
				/** Clear the corresponding n-bits from position offset */
				val2 &= ~( mask << offset );
				break;
			case MML_GPIO_PAD_PULLDOWN:
				/** Clear the corresponding n-bits from position offset */
				val1 &= ~( mask << offset );
				/** Set the corresponding n-bits from position offset */
				val2 |= ( mask << offset );
				break;
			case MML_GPIO_PAD_WEAK_LATCH:
				/** Set the corresponding n-bits from position offset */
				val1 |= ( mask << offset );
				val2 |= ( mask << offset );
				break;
			default:
				result = MML_GPIO_ERR_OUT_OF_RANGE;
				goto mml_gpio_pad_configure_out;
		}
		/**  */
		reg_gpio->pad_cfg1 = val1;
		reg_gpio->pad_cfg2 = val2;
		result = NO_ERROR;
	}
	/** We're done */
mml_gpio_pad_configure_out:
	return result;
}

/*******************************************************************************************/
/**
 * This function configures the GPIO functionality i.e. GPIO in normal function mode
 * or alternate functions for the specified range of pins.
 * NOTE: All the specified GPIO pins are configured for only one GPIO functionality
 * @param[in] dev_id						The GPIO device identifier
 * @param[in] function						GPIO functionality configuration information
 * 											i.e. GPIO alternate function or normal function.
 * @param[in] offset						Pin offset in the range of 0 to 31
 * @param[in] bits_count					Specifies number of consecutive pins
 * 											starting from 'offset' position.
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device identifier
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		One or more parameters are invalid
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_configure_function(mml_gpio_id_t dev_id,
									mml_gpio_function_t function,
									int offset,
									int bits_count)
{
	int											result = COMMON_ERR_UNKNOWN;

	result = mml_gpio_check(dev_id, offset, bits_count);
	if ( NO_ERROR == result )
	{
		unsigned int							en_val;
		unsigned int							en1_val;
		/** Create Mask */
		unsigned int							mask = ~( ~0 << bits_count );
		/** Assign the GPIO device base address */
		volatile mml_gpio_regs_t				*reg_gpio = (volatile mml_gpio_regs_t*)gpio_context.port[dev_id].reg;



		/** Read the current values */
		en_val = reg_gpio->en;
		en1_val = reg_gpio->en1;
		/** Create the register values */
		switch( function )
		{
			case MML_GPIO_NORMAL_FUNCTION:
				/** Set the corresponding n-bits from position offset */
				en_val |= ( mask << offset );
				/** Clear the corresponding n-bits from position offset */
				en1_val &= ~(mask << offset );
				break;
			case MML_GPIO_SECODARY_ALT_FUNCTION:
				/** Clear the corresponding n-bits from position offset */
				en_val &= ~( mask << offset );
				en1_val &= ~( mask << offset );
				break;
			case MML_GPIO_TERTIARY_ALT_FUNCTION:
				/** Clear the corresponding n-bits from position offset */
				en_val &= ~( mask << offset );
				/** Set the corresponding n-bits from position offset */
				en1_val |= ( mask << offset );
				break;
			default:
				result = MML_GPIO_ERR_OUT_OF_RANGE;
				goto mml_gpio_configure_function_out;
		}
		/**  */
		reg_gpio->en = en_val;
		reg_gpio->en1 = en1_val;
		result = NO_ERROR;
	}
	/** We're done */
mml_gpio_configure_function_out:
	return result;
}

/*******************************************************************************/
/**
 * This function enables a range of consecutive GPIO pins for the output mode.
 * The valid GPIO pins 'offset' range is between 0 to 31.
 * @param[in] dev_id						The GPIO device identifier
 * @param[in] offset						Pin offset in the range of 0 to 31
 * @param[in] bits_count					Specifies number of consecutive pins
 * 											starting from 'offset' position.
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device identifier
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		One or more parameters are invalid
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_enable_output(mml_gpio_id_t dev_id, int offset, int bits_count)
{
	int											result = COMMON_ERR_UNKNOWN;

	result = mml_gpio_check(dev_id, offset, bits_count);
	if ( NO_ERROR == result )
	{
		unsigned int							val;
		/** Create bits_count Mask */
		unsigned int							mask = ~( ~0 << bits_count );
		/** Assign the GPIO base address */
		volatile mml_gpio_regs_t 				*reg_gpio = (volatile mml_gpio_regs_t*)gpio_context.port[dev_id].reg;


		val = reg_gpio->out_en;
		/** set n-bits from offset position */
		val |= ( mask << offset );
		/** Configure the GPIO pins in output mode */
		reg_gpio->out_en = val;
	}
	/** We're done */
	return result;
}


/*******************************************************************************/
/**
 * This function enables a range of consecutive GPIO pins interrupt.
 * The valid GPIO pins 'offset' range is between 0 to 31.
 * @param[in] dev_id						The GPIO device identifier
 * @param[in] offset						Pin offset in the range of 0 to 31
 * @param[in] bits_count					Specifies number of consecutive pins
 * 											starting from 'offset' position.
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device identifier
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		One or more parameters are invalid
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_enable_interrupt(mml_gpio_id_t dev_id, int offset, int bits_count)
{
	int											result = COMMON_ERR_UNKNOWN;

	result = mml_gpio_check(dev_id, offset, bits_count);
	if ( NO_ERROR == result )
	{
		/** Create bits_count Mask */
		unsigned int							mask = ~( ~0 << bits_count );
		/** Assign the GPIO base address */
		volatile mml_gpio_regs_t 				*reg_gpio = (volatile mml_gpio_regs_t*)gpio_context.port[dev_id].reg;


		/** Enable the GPIO pins interrupt */
		reg_gpio->int_en_set = ( mask << offset );
	}
	/** We're done */
	return result;
}


/*******************************************************************************/
/**
 * This function disables a range of consecutive GPIO pins interrupt.
 * The valid GPIO pins 'offset' range is between 0 to 31.
 * @param[in] dev_id						The GPIO device identifier
 * @param[in] offset						Pin offset in the range of 0 to 31
 * @param[in] bits_count					Specifies number of consecutive pins
 * 											starting from 'offset' position.
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device identifier
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		One or more parameters are invalid
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_disable_interrupt(mml_gpio_id_t dev_id, int offset, int bits_count)
{
	int											result = COMMON_ERR_UNKNOWN;

	result = mml_gpio_check(dev_id, offset, bits_count);
	if ( NO_ERROR == result )
	{
		/** Create bits_count Mask */
		unsigned int							mask = ~( ~0 << bits_count );
		/** Assign the GPIO base address */
		volatile mml_gpio_regs_t 				*reg_gpio = (volatile mml_gpio_regs_t*)gpio_context.port[dev_id].reg;


		/** disable the GPIO pins interrupt */
		reg_gpio->int_en_clr = ( mask << offset );
	}
	/** We're done */
	return result;
}

/*******************************************************************************/
/**
 * This function clears a range of consecutive GPIO pins interrupt flags.
 * The valid GPIO pins 'offset' range is between 0 to 31.
 * @param[in] dev_id						The GPIO device identifier
 * @param[in] offset						Pin offset in the range of 0 to 31
 * @param[in] bits_count					Specifies number of consecutive pins
 * 											starting from 'offset' position.
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device identifier
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		One or more parameters are invalid
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_clear_interrupt(mml_gpio_id_t dev_id, int offset, int bits_count)
{
	int											result = COMMON_ERR_UNKNOWN;

	result = mml_gpio_check(dev_id, offset, bits_count);
	if ( NO_ERROR == result )
	{
		/** Create bits_count Mask */
		unsigned int							mask = ~( ~0 << bits_count );
		/** Assign the GPIO base address */
		volatile mml_gpio_regs_t 				*reg_gpio = (volatile mml_gpio_regs_t*)gpio_context.port[dev_id].reg;


		/** clear the GPIO pins interrupt flags */
		reg_gpio->int_clr = ( mask << offset );
	}
	/** We're done */
	return result;
}

/*******************************************************************************/
/**
 * This function gets a range of consecutive GPIO pins interrupt flags.
 * The valid GPIO pins 'offset' range is between 0 to 31.
 * @param[in] dev_id						The GPIO device identifier
 * @param[in] offset						Pin offset in the range of 0 to 31
 * @param[in] bits_count					Specifies number of consecutive pins
 * 											starting from 'offset' position.
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device identifier
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		One or more parameters are invalid
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_get_interrupt_status(mml_gpio_id_t dev_id, int offset, int bits_count, int* status)
{
	int											result = COMMON_ERR_UNKNOWN;

	result = mml_gpio_check(dev_id, offset, bits_count);

	if ( NO_ERROR == result )
	{
		/** Create bits_count Mask */
		unsigned int							mask = ~( ~0 << bits_count );
		/** Assign the GPIO base address */
		volatile mml_gpio_regs_t 				*reg_gpio = (volatile mml_gpio_regs_t*)gpio_context.port[dev_id].reg;


		/** clear the GPIO pins interrupt flags */
		*status = reg_gpio->int_stat & ( mask << offset );
	}
	/** We're done */
	return result;
}

/*******************************************************************************/
/**
 * This function disables a range of consecutive GPIO pins from output mode.
 * The valid GPIO pins range is between 0 to 31.
 * @param[in] dev_id						The GPIO device identifier
 * @param[in] offset						Pin offset in the range of 0 to 31
 * @param[in] bits_count					Specifies number of consecutive pins
 * 											starting from 'offset' position.
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device identifier
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		One or more parameters are invalid
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_disable_output(mml_gpio_id_t dev_id, int offset, int bits_count)
{
	int											result = COMMON_ERR_UNKNOWN;

	result = mml_gpio_check(dev_id, offset, bits_count);
	if ( NO_ERROR == result )
	{
		/** Create bits_count Mask */
		unsigned int							mask = ~( ~0 << bits_count );
		unsigned int							val;
		/** Assign the GPIO base address */
		volatile mml_gpio_regs_t 				*reg_gpio = (volatile mml_gpio_regs_t*)gpio_context.port[dev_id].reg;



		val = reg_gpio->out_en;
		/** Clear n-bits from offset position */
		val &= ~( mask << offset );
		/** Disable the GPIO output mode */
		reg_gpio->out_en = val;
	}
	/** We're done */
	return result;
}

/***********************************************************************************/
/**
 * This function configures the consecutive GPIO0 pins in the open drain output mode.
 * NOTE: This function applies only to GPIO0.
 * @param[in] dev_id						The GPIO device identifier and
 * 											it should be MML_GPIO_DEV0 only.
 * @param[in] offset						Pin offset in the range of 0 to 31
 * @param[in] bits_count					Specifies number of consecutive pins
 * 											starting from 'offset' position.
 * @param[in] config						Open drain configuration
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		Invalid device ID or invalid parameter
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_config_open_drain(mml_gpio_id_t dev_id,
								const unsigned int offset,
								unsigned int bits_count,
								mml_gpio_open_drain_t config)
{
	int											result = COMMON_ERR_UNKNOWN;

	/** Check input parameters */
	if ( ( MML_GPIO_DEV0 != dev_id ) && ( MML_GPIO_OPEN_DRAIN_COUNT > config ) )
	{
		result = MML_GPIO_ERR_OUT_OF_RANGE;
	}
	/** Check the offset is valid */
	else if ( MML_GPIO_BIT_RANGE_MAX < offset )
	{
		result = MML_GPIO_ERR_OUT_OF_RANGE;
	}
	/** Check the bits count is valid */
	else if ( bits_count > ( MML_GPIO_BIT_RANGE_NB - offset ) )
	{
		result = MML_GPIO_ERR_OUT_OF_RANGE;
	}
	else
	{
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/**************************************************************************/
/**
 * This function writes 32-bit data to the GPIO DATA_OUT register
 * @param[in] dev_id						The GPIO device identifier.
 * @param[in] data							Data word for output
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		Invalid GPIO device
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_write_dataout(mml_gpio_id_t dev_id, unsigned int data)
{
#ifdef _INPUT_STRICT_CHECK_
	int											result = COMMON_ERR_UNKNOWN;

	/** Validate the input parameter dev_id */
	if ( MML_GPIO_DEV_COUNT <= dev_id )
	{
		result = MML_GPIO_ERR_INVALID_DEVICE;
	}
	else
	{
		gpio_context.port[dev_id].reg->out = data;
		result = NO_ERROR;
	}
	/** We're done */
	return result;
#else
	gpio_context.port[dev_id].reg->out = data;
	/** We're done */
	return NO_ERROR;
#endif /* _INPUT_STRICT_CHECK_ */
}

/******************************************************************************/
/**
 * This function reads 32-bit data from the GPIO DATA_OUT register
 * @param[in] dev_id						The GPIO device identifier.
 * @param[out] data							Received data word is stored
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_read_dataout(mml_gpio_id_t dev_id, unsigned int *data)
{
#ifdef _INPUT_STRICT_CHECK_
	int											result = COMMON_ERR_UNKNOWN;

	/** Validate the input parameter dev_id */
	if ( MML_GPIO_DEV_COUNT <= dev_id )
	{
		result = MML_GPIO_ERR_INVALID_DEVICE;
	}
	else
	{
		*data = gpio_context.port[dev_id].reg->out;
		result = NO_ERROR;
	}
	/** We're done */
	return result;
#else
	*data = gpio_context.port[dev_id].reg->out;
	/** We're done */
	return NO_ERROR;
#endif /* _INPUT_STRICT_CHECK_ */
}

/*******************************************************************************/
/**
 * This function reads 32-bit data word from the GPIO DATA_IN register
 * @param[in] dev_id						The GPIO device identifier.
 * @param[out] data							Received data word is stored
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		Invalid GPIO device
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_read_datain(mml_gpio_id_t dev_id, unsigned int *data)
{
#ifdef _INPUT_STRICT_CHECK_
	int											result = COMMON_ERR_UNKNOWN;

	/** Validate the input parameter dev_id */
	if ( MML_GPIO_DEV_COUNT <= dev_id )
	{
		result = MML_GPIO_ERR_INVALID_DEVICE;
	}
	else
	{
		/** Read DATA_IN register */
		*data = gpio_context.port[dev_id].reg->in;
		result = NO_ERROR;
	}
	/** We're done */
	return result;
#else
	/** Read DATA_IN register */
	*data = gpio_context.port[dev_id].reg->in;
	/** We're done */
	return NO_ERROR;
#endif /* _INPUT_STRICT_CHECK_ */
}

/************************************************************************************************/
/**
 * This function transmits specified number of data words stored in the buffer to the GPIO device
 * @param[in] dev_id						The GPIO device identifier.
 * @param[in] buffer						buffer pointer from where the data to be transmitted
 * @param[in] length						Specifies number of 32-bit data words
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device
 * @retval COMMON_ERR_NULL_PTR				buffer pointer is NULL
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_write_buffer(mml_gpio_id_t dev_id, unsigned int *buffer, int length)
{
	int											result = COMMON_ERR_UNKNOWN;

#ifdef _INPUT_STRICT_CHECK_
	/** Validate the input parameter dev_id */
	if ( MML_GPIO_DEV_COUNT <= dev_id )
	{
		result = MML_GPIO_ERR_INVALID_DEVICE;
	}
	/** Check the buffer pointer is NULL */
	else if ( !buffer )
#else
	/** Check the buffer pointer is NULL */
	if ( !buffer )
#endif /* _INPUT_STRICT_CHECK_ */
	{
		result = COMMON_ERR_NULL_PTR;
	}
	else
	{
		register unsigned int				i;

		/** Write the data words in loop */
		for( i = 0;i < (unsigned int)length;i++ )
		{
			mml_gpio_write_dataout(dev_id, buffer[i]);
		}
		/** No error */
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/************************************************************************************************/
/**
 * This function receives specified number of data words from the GPIO device and stores in buffer
 * @param[in] dev_id						The GPIO device identifier.
 * @param[in] buffer						buffer pointer where to store the received data
 * @param[in] length						Specifies number of 32-bit data words
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device
 * @retval COMMON_ERR_NULL_PTR				buffer pointer is NULL
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_read_buffer(mml_gpio_id_t dev_id, unsigned int *buffer, int length)
{
	int											result = COMMON_ERR_UNKNOWN;

#ifdef _INPUT_STRICT_CHECK_
	/** Validate the input parameter dev_id */
	if ( MML_GPIO_DEV_COUNT <= dev_id )
	{
		result = MML_GPIO_ERR_INVALID_DEVICE;
	}
	/** Check the buffer pointer is NULL */
	else if ( !buffer )
#else
	/** Check the buffer pointer is NULL */
	if ( !buffer )
#endif /* _INPUT_STRICT_CHECK_ */
	{
		result = COMMON_ERR_NULL_PTR;
	}
	else
	{
		unsigned int							i;

		/**  Read the data words in loop */
		for( i = 0;i < (unsigned int)length;i++ )
		{
			mml_gpio_read_datain(dev_id, buffer + i);
		}
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/************************************************************************************/
/**
 * Writes a consecutive bit pattern stored in the 'data' word in a specified
 * group of pins with in a GPIO port.
 * Example: Input parameters(gpio_id, offset = 3, bits_count = 3, data[31:0])
 * Then the 3-bit pattern is stored as gpio_id[5:3] = data[2:0];
 * @param[in] dev_id						The GPIO device identifier.
 * @param[in] offset						Pin offset in the range of 0 to 31
 * @param[in] bits_count					Specifies number of consecutive pins
 * @param[in] data							LSB adjusted n-bit field of x that begins
 * 											at position 'offset'.
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		Invalid parameters
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_write_bit_pattern(mml_gpio_id_t dev_id, int offset, int bits_count, unsigned int data)
{
	int											result = COMMON_ERR_UNKNOWN;

	result = mml_gpio_check(dev_id, offset, bits_count);
	if ( NO_ERROR == result )
	{
		/** Create bits_count Mask */
		unsigned int 							mask = ~( ~0 << bits_count );
		unsigned int							temp;
		volatile mml_gpio_regs_t				*reg_gpio = (volatile mml_gpio_regs_t*)gpio_context.port[dev_id].reg;


		temp = reg_gpio->out;
		temp &= ~( mask << offset );
		temp |= ( data << offset );
		reg_gpio->out = temp;
	}
	/** We're done */
	return result;
}

/****************************************************************************************/
/**
 * Reads bit pattern from consecutive GPIO pins and left adjusts this pattern and
 * stores it in LSB of 'data' word.
 * Example: Input parameters(gpio_id, offset = 5, bits_count = 6, data_p[31:0])
 * Then the 6-bit pattern is read is stored in data_p[5:0] = gpio_id[10:5];
 * @param[in] dev_id						The GPIO device identifier.
 * @param[in] offset						Pin offset in the range of 0 to 31
 * @param[in] bits_count					Specifies number of consecutive pins
 * @param[in] data							Where the LSB adjusted bit pattern is stored
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		Invalid parameters
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_read_bit_pattern(mml_gpio_id_t dev_id, int offset, int bits_count, unsigned int *data)
{
	int											result = COMMON_ERR_UNKNOWN;

	result = mml_gpio_check(dev_id, offset, bits_count);
	if ( NO_ERROR == result )
	{
		/** Create bits_count Mask */
		unsigned int 							mask = ~( ~0 << bits_count );
		/**  Assign the GPIO device base address */
		volatile mml_gpio_regs_t				*reg_gpio = (volatile mml_gpio_regs_t*)gpio_context.port[dev_id].reg;

		*data = reg_gpio->in;
		*data >>= offset;
		*data &= mask;
	}
	/** We're done */
	return result;
}

/************************************************************************************/
/**
 * This is the output function used to set/clear logic '1' or '0' to a GPIO pin
 * without affecting the other pins.
 * @param[in] dev_id						The GPIO device identifier
 * @param[in] offset						Pin offset in the range of 0 to 31
 * @param[in] output						Specifies the output value logic '0' or '1'
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device identifier
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		One or more parameters are invalid
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_pin_output(mml_gpio_id_t dev_id,
							const unsigned int offset,
							mml_gpio_pin_data_t output)
{
	int											result = COMMON_ERR_UNKNOWN;

#ifdef _INPUT_STRICT_CHECK_
	/** Check input parameters */
	if ( MML_GPIO_DEV_COUNT <= dev_id )
	{
		result = MML_GPIO_ERR_INVALID_DEVICE;
	}
	/** Check the offset */
	else if ( MML_GPIO_BIT_RANGE_MAX < offset )
#else
	/** Check the offset */
	if ( MML_GPIO_BIT_RANGE_MAX < offset )
#endif /* _INPUT_STRICT_CHECK_ */
	{
		result = MML_GPIO_ERR_OUT_OF_RANGE;
	}
	else
	{
		unsigned int							temp = 0;
		volatile mml_gpio_regs_t				*reg_gpio = (volatile mml_gpio_regs_t*)gpio_context.port[dev_id].reg;

		/** Pre-set return value */
		result = NO_ERROR;
#ifdef _GPIO_DIRECT_OUTPUT_
		temp = reg_gpio->out;
#endif /* _GPIO_DIRECT_OUTPUT_ */
		/** Choose according to 'ouput' */
		switch( output )
		{
			case MML_GPIO_OUT_LOGIC_ZERO:
#ifndef _GPIO_DIRECT_OUTPUT_
				temp &= ~( 1 << offset );
#else
				temp = reg_gpio->out_clr;
				temp |= ( 1 << offset );
				reg_gpio->out_clr = temp;
				reg_gpio->out = temp;
#endif /* _GPIO_DIRECT_OUTPUT_ */
				break;
			case MML_GPIO_OUT_LOGIC_ONE:
#ifdef _GPIO_DIRECT_OUTPUT_
				temp |= ( 1 << offset );
				reg_gpio->out = temp;
#else
				temp = reg_gpio->out_set;
				temp |= ( 1 << offset );
				reg_gpio->out_set = temp;
#endif /* _GPIO_DIRECT_OUTPUT_ */
				break;
			default:
				result = MML_GPIO_ERR_OUT_OF_RANGE;
				break;
		}
	}
	/** We're done */
	return result;
}

/*****************************************************************************/
/**
 * This is the input function it can be used to read data from a GPIO pin.
 * @param[in] dev_id						The GPIO device identifier
 * @param[in] offset						Pin offset in the range of 0 to 31
 * @param[in] input							input value either logic '0' or '1'
 * @retval NO_ERROR							No error
 * @retval MML_GPIO_ERR_INVALID_DEVICE	Invalid GPIO device identifier
 * @retval MML_GPIO_ERR_OUT_OF_RANGE		One or more parameters are invalid
 * @retval COMMON_ERR_UNKNOWN				Unknown error occurred
 */
int mml_gpio_pin_input(mml_gpio_id_t dev_id,
						const unsigned int offset,
						mml_gpio_pin_data_t *input)
{
	int											result = COMMON_ERR_UNKNOWN;

#ifdef _INPUT_STRICT_CHECK_
	/** Check input parameters */
	if ( MML_GPIO_DEV_COUNT <= dev_id )
	{
		result = MML_GPIO_ERR_INVALID_DEVICE;
	}
	/** Check the offset */
	else if ( MML_GPIO_BIT_RANGE_MAX < offset )
#else
	/** Check the offset */
	if ( MML_GPIO_BIT_RANGE_MAX < offset )
#endif /* _INPUT_STRICT_CHECK_ */
	{
		result = MML_GPIO_ERR_OUT_OF_RANGE;
	}
	else
	{
		/** Assign the GPIO base address  */
		volatile mml_gpio_regs_t				*reg_gpio = (volatile mml_gpio_regs_t*)gpio_context.port[dev_id].reg;


		if ( reg_gpio->in & ( 1 << offset ) )
		{
			*input = MML_GPIO_OUT_LOGIC_ONE;
		}
		else
		{
			*input = MML_GPIO_OUT_LOGIC_ZERO;
		}
		/** No error */
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/*****************************************************************************/
int mml_gpio_check(mml_gpio_id_t dev_id, int offset, int bits_count)
{
	int											result = COMMON_ERR_UNKNOWN;

	/** Check input parameters */
	if ( MML_GPIO_DEV_COUNT <= dev_id )
	{
		result = MML_GPIO_ERR_INVALID_DEVICE;
	}
	/** Check the offset */
	else if ( MML_GPIO_BIT_RANGE_MAX < offset )
	{
		result = MML_GPIO_ERR_OUT_OF_RANGE;
	}
	/** Check the bits count is valid */
	else if ( ( MML_GPIO_BIT_RANGE_MIN > bits_count ) ||
			( bits_count > ( MML_GPIO_BIT_RANGE_NB - offset ) ) )
	{
		result = MML_GPIO_ERR_OUT_OF_RANGE;
	}
	else
	{
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/*****************************************************************************/
void mml_gpio_clear_out_int_wake(mml_gpio_id_t dev_id, int offset, int bits_count)
{
	unsigned int		 						temp;
	unsigned int		 						mask = ~( ~0 << bits_count );
	volatile mml_gpio_regs_t					*reg_gpio = (volatile mml_gpio_regs_t*)gpio_context.port[dev_id].reg;


	/** Clear GPIO output pins to driver logic zero */
	temp = reg_gpio->out;
	/** Clear the corresponding n-bits from position offset */
	temp &= ~( mask << offset );
	reg_gpio->out = temp;
	/** Disable corresponding interrupts */
	temp = reg_gpio->int_en;
	/** Clear the corresponding n-bits from position offset */
	temp &= ~( mask << offset );
	reg_gpio->int_en = temp;
	/** Disable PMU wake for GPIO pins */
	temp = reg_gpio->wake_en;
	/** Clear the corresponding n-bits from position offset */
	temp &= ~( mask << offset );
	reg_gpio->wake_en = temp;
	/** We're done */
	return;
}

/*****************************************************************************/
void mml_gpio_set_padsmode(const unsigned int dev[3], int mode)
{
	unsigned int								data_read;
	unsigned int								data_write;
	unsigned int								bit_offset;
	unsigned int								bit_mask;
	unsigned int								gpio_base = (unsigned int)dev[0];

	/**  */
	IO_READ_U32(gpio_base, data_read);
	bit_offset = dev[1];
	bit_mask = dev[2];
	/** Apply the mask */
	data_write = data_read & ~(bit_mask<<bit_offset);
	if ( COBRA_PADSMODE_GPIO == mode )
	{
		data_write |= (bit_mask<<bit_offset);
	}
	/**  */
	IO_WRITE_U32(gpio_base, data_write);
	/** We're done */
	return;
}

/*****************************************************************************/
/* EOF */
