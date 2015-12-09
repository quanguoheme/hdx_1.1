/*
 * mml_i2c.c --
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
 * Created on: Aug 09, 2012
 * Author:
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

/** @file mml_i2c_config_t.c I2C core driver */

/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
#include <mml.h>
#include <mml_intc.h>
#include <mml_intc_regs.h>
#include <mml_gpio.h>
#include <mml_gcr.h>
#include <cobra_defines.h>
#include <cobra_macros.h>
#include <cobra_functions.h>
/** Local includes */
#include <mml_i2c.h>
#include <mml_i2c_regs.h>


#define	MML_I2C_MIN_BAUD_RATE			0
#define	MML_I2C_MAX_BAUD_RATE 			400000
#define	MML_I2C_MIN_RXFIFO_THRESHOLD	0
#define	MML_I2C_MAX_RXFIFO_THRESHOLD	8

/** Variables */
__attribute__((section(".bss")))  mml_i2c_context_t mml_i2c_context;

#ifdef _STAND_ALONE_DRIVER_I2C_
/* This compilation flag is activated only in driver development context
* without any application using it.
* DO NOT define it in application/test context then. */
/******************************************************************************/
int main(void)
{
	/** We're done */
	return NO_ERROR;
}
#endif /* _STAND_ALONE_DRIVER_I2C_ */

/****************************************************************************************************/
/**
 * Function to initialize and configure the I2C interface
 * @param config					I2C configuration parameter
 * @retval NO_ERROR					No error
 * @retval COMMON_ERR_NULL_PTR		NULL pointer parameter passed
 * @retval MML_I2C_ERR_INIT		Invalid configuration parameter or initialization error
 */
int mml_i2c_init(mml_i2c_config_t *config)
{
	int 										result = COMMON_ERR_UNKNOWN;
	volatile mml_gcr_regs_t 					*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;


    if ( !mml_i2c_context.first_init )
    {
    	if ( !config->irq_handler )
    	{
    		result = COMMON_ERR_NULL_PTR;
    	}
    	else
    	{
			/** Enable I2C clock */
			reg_gcr->perckcn &= ~( 1  << MML_PERCKCN_DEV_I2C );
#if _I2C_RESET_AT_INIT_
			/** Reset the I2C block */
			reg_gcr->rstr |= MML_GCR_RSTR_I2C_MASK;
			/** Wait for I2C reset to be done */
			while( reg_gcr->rstr & MML_GCR_RSTR_I2C_MASK );
#endif /* _I2C_RESET_AT_INIT_ */
			/** Initialization to be finished ;) */
			mml_i2c_context.first_init = 1;
			/** Set configuration */
			result = mml_i2c_set_configuration(config);
			if ( result )
			{
				/** Close port */
				mml_i2c_close();
			}
			else
			{
				/** Enable the interrupt */
				result = mml_intc_setup_irq(MML_INTNUM_I2C, MML_INTC_PRIO_5, config->irq_handler);
			}
    	}
    }
    /** We're done */
	return result;
}

/****************************************************************************************************/
/**
 * Function to configure the I2C interface which is already been initialized
 * @param config						I2C configuration parameters
 * @retval NO_ERROR						No error
 * @retval MML_I2C_ERR_INIT			Device is not initialized
 * @retval COMMON_ERR_NULL_PTR			NULL pointer parameter passed
 * @retval MML_I2C_ERR_INVALID_BAUD	Baud rate is out of range
 * @retval MML_I2C_ERR_INVALID_RXTHRD	Receive FIFO threshold is invalid
 * @retval MML_I2C_ERR_GPIO_CONFIG		GPIO initialization error
 */
int mml_i2c_set_configuration(mml_i2c_config_t *config)
{
	int 										result = COMMON_ERR_UNKNOWN;


	/** Validate the parameters */
	if ( !mml_i2c_context.first_init )
	{
		result = MML_I2C_ERR_INIT;
	}
	else if( !config )
	{
		result = COMMON_ERR_NULL_PTR;
	}
	/** Maximum baud rate = 400 kbps */
	else if ( ( MML_I2C_MIN_BAUD_RATE >= config->baudrate ) || ( MML_I2C_MAX_BAUD_RATE < config->baudrate ) )
	{
		result = MML_I2C_ERR_INVALID_BAUD;
	}
	/** RX FIFO threshold (1 to 8) */
	else if ( ( MML_I2C_MIN_RXFIFO_THRESHOLD >= config->rxfifo_thr ) || ( MML_I2C_MAX_RXFIFO_THRESHOLD < config->rxfifo_thr ) )
	{
		result = MML_I2C_ERR_INVALID_RXTHRD;
	}
	else
	{
		unsigned int 							freq;
		mml_gpio_config_t						gpio_config;
		volatile mml_i2c_regs_t 				*reg_i2c = (volatile mml_i2c_regs_t*)MML_I2C_IOBASE;

		reg_i2c->cr |= ( MML_I2C_CR_TXFLUSH_MASK | MML_I2C_CR_RXFLUSH_MASK );
		reg_i2c->cr |= config->rxfifo_thr;
		/** Set baudrate */
		mml_get_system_frequency(&freq);
		reg_i2c->brr = freq / ( 32 * config->baudrate );
		/** Autostart asked ? */
		if( config->flags & MML_I2C_AUTOSTART_ENABLE )
		{
			reg_i2c->cr |= MML_I2C_CR_AUTOSTT_MASK;
		}
		/** Configure GPIO multiplexed lines of I2C */
		/** Initialise GPIO for I2C interface [P1.0]: SDA I2C Data, [P1.1]:SCL I2C Clock */
		gpio_config.gpio_direction = MML_GPIO_DIR_OUT; /** [TBD] DHB check direction in GPIO driver */
		gpio_config.gpio_function = MML_GPIO_SECODARY_ALT_FUNCTION;
		gpio_config.gpio_intr_mode = MML_GPIO_INT_MODE_LEVEL_TRIGGERED; /** Not relevant */
		gpio_config.gpio_intr_polarity = MML_GPIO_INT_POL_RAISING; /** Not relevant */
		gpio_config.gpio_pad_config = MML_GPIO_PAD_NORMAL;
		result = mml_gpio_init(MML_GPIO_DEV0, 23, 2, gpio_config);
		if ( result )
		{
			result = MML_I2C_ERR_GPIO_CONFIG;
		}
	}
	/** We're done */
	return result;
}

/****************************************************************************************************/
/**
 * Function initializes the I2C write operation and transmits specified number of data bytes to the
 * I2C slave device.
 * @param address						I2C Device address
 * 										10 bit address pattern = 1111 0xx0 xxxx xxxx
 * 										7 bit address pattern  = 0000 0000 xxxx xxx0
 * @param data							memory address from where the data to be transmitted
 * @param plength						memory address pointing to the 'length' number of bytes to transmit.
 * @retval NO_ERROR						No error
 * @retval MML_I2C_ERR_INIT			Device is not initialized
 * @retval COMMON_ERR_NULL_PTR			NULL pointer parameter passed
 * @retval COMMON_ERR_UNKNOWN			Unknown error
 * @retval MML_I2C_ERR_READY			TX FIFO is empty
 */
int mml_i2c_write_start(unsigned short address, const unsigned char *data, unsigned int *plength)
{
	int 										result = COMMON_ERR_UNKNOWN;
	unsigned int 								len = 0;
	register unsigned int						loop = K_COBRA_I2C_WAIT_LOOP;
	volatile mml_i2c_regs_t 					*reg_i2c = (volatile mml_i2c_regs_t*)MML_I2C_IOBASE;


	if ( !mml_i2c_context.first_init )
	{
		result = MML_I2C_ERR_INIT;
		goto mml_i2c_write_start_out;
	}
	/**  */
	if ( !data || !plength )
	{
		result = COMMON_ERR_NULL_PTR;
		goto mml_i2c_write_start_out;
	}
	/** Check if address is one of 10 bit OR 7 bit pattern */
	if ( ( ( address & 0xf900 ) != 0xf000 ) && ( ( address & 0xff01 ) != 0x0000))
	{
		result = MML_I2C_ERR_INVALID_ADDRESS;
		goto mml_i2c_write_start_out;
	}
	/** Wait for TX/RX FIFO to be empty ... just in case */
	while( !( reg_i2c->sr & MML_I2C_SR_TXEMPTY_MASK ) && loop-- );
	/** De-assert RESTART & STOP bits */
	reg_i2c->cr &= ( ~MML_I2C_CR_RESTART_MASK | MML_I2C_CR_STOP_MASK );
	/** Check if TX FIFO is empty */
	if ( !( reg_i2c->sr & MML_I2C_SR_TXEMPTY_MASK ) )
	{
		result = MML_I2C_ERR_READY;
		goto mml_i2c_write_start_out;
	}
	/** Write address AND R/W = 0 */
	if ( ( address & 0xf900 ) == 0xf000 )
	{
		/** 10-bit address? */
		reg_i2c->dr = ( address >> 8 );
	}
	else
	{
		reg_i2c->dr = ( address & 0x00ff );
	}
	/** Send data */
	do
	{
		reg_i2c->dr = *data++;
		(*plength)--;
		len++;
		/**  */
		if ( !(*plength) )
		{
			break;
		}
	} while( !( reg_i2c->sr & MML_I2C_SR_TXFULL_MASK ) );
	/**  */
	reg_i2c->cr |= ( MML_I2C_CR_START_MASK | MML_I2C_CR_RESTART_MASK );
	if ( !(*plength) )
	{
		reg_i2c->cr &= ~MML_I2C_CR_START_MASK;
	}
	/**  */
	*plength = len;
	/** No error */
	result = NO_ERROR;
	/** We're done */
mml_i2c_write_start_out:
	return result;
}

/****************************************************************************************************/
/**
 * Function to transmit the data to the I2C device, this function should be called after the
 * communication has started.
 * Pre-condition : mml_i2c_write_start() to be called before calling this fn.
 * @param data							memory address from where the data to be transmitted (maximum = 8)
 * @param plength						memory address pointing to the 'length' number of bytes to transmit.
 * @retval NO_ERROR						No error
 * @retval MML_I2C_ERR_INIT			Device is not initialized
 * @retval COMMON_ERR_NULL_PTR			NULL pointer parameter passed
 * @retval COMMON_ERR_UNKNOWN			Unknown error
 */
int mml_i2c_write(unsigned char *data,unsigned int *plength)
{
	int											result = COMMON_ERR_UNKNOWN;
	unsigned int								len = 0;
	volatile mml_i2c_regs_t						*reg_i2c = (volatile mml_i2c_regs_t*)MML_I2C_IOBASE;


	if ( !mml_i2c_context.first_init )
	{
		result = MML_I2C_ERR_INIT;
		goto mml_i2c_write_out;
	}
	/**  */
	if( !data ||  !plength )
	{
		result = COMMON_ERR_NULL_PTR;
		goto mml_i2c_write_out;
	}
	/**  */
	while( !( reg_i2c->sr & MML_I2C_SR_TXFULL_MASK ) )
	{
		reg_i2c->dr = *data++;
		(*plength)--;
		len++;
		/**  */
		if ( !(*plength) )
		{
			break;
		}
	}
	/**  */
	if ( !(*plength) )
	{
		reg_i2c->cr &= ~MML_I2C_CR_START_MASK;
	}
	/**  */
	*plength = len;
	/** No error */
	result = NO_ERROR;
	/** We're done */
mml_i2c_write_out:
	return result;
}

/****************************************************************************************************/
/**
 * Function to initiate the I2C Start condition for read operation.
 * No data is read from the I2C slave device but it just initializes the I2C slave device for reading.
 * @param address						I2C Device address
 * 										10-bit address pattern = 1111 0xx0 xxxx xxxx
 * 										7-bit address pattern  = 0000 0000 xxxx xxx0
 * @retval NO_ERROR						No error
 * @retval MML_I2C_ERR_INIT			Device is not initialized
 * @retval COMMON_ERR_NULL_PTR			NULL pointer parameter passed
 * @retval COMMON_ERR_UNKNOWN			Unknown error
 * @retval MML_I2C_ERR_INVALID_ADDRESS	Invalid device address
 * @retval MML_I2C_ERR_READY			TX FIFO empty
 */
int mml_i2c_read_start(unsigned short address)
{
	int 										result = COMMON_ERR_UNKNOWN;
	register unsigned int						loop = K_COBRA_I2C_WAIT_LOOP;
	volatile mml_i2c_regs_t 					*reg_i2c = (volatile mml_i2c_regs_t*)MML_I2C_IOBASE;


	if ( !mml_i2c_context.first_init )
	{
		result = MML_I2C_ERR_INIT;
		goto mml_i2c_read_start_out;
	}
	/** Check if address is one of 10 bit OR 7 bit pattern */
	if ( ( ( address & 0xf900 ) != 0xf000 ) && ( ( address & 0xff01 ) != 0x0000 ) )
	{
		result = MML_I2C_ERR_INVALID_ADDRESS;
		goto mml_i2c_read_start_out;
	}
	/** Wait for TX/RX FIFO to be empty ... just in case */
	while( !( reg_i2c->sr & MML_I2C_SR_TXEMPTY_MASK ) && loop-- );
	/** De assert RESTART & STOP bits */
	reg_i2c->cr &= ( ~MML_I2C_CR_RESTART_MASK | MML_I2C_CR_STOP_MASK );
	/** Check if TX FIFO is empty */
	if ( !( reg_i2c->sr & MML_I2C_SR_TXEMPTY_MASK ) )
	{
		result = MML_I2C_ERR_READY;
		goto mml_i2c_read_start_out;
	}
	/** Write address AND R/W = 1*/
	if ( ( address & 0xf900 ) == 0xf000 )
	{
		/** 10-bit address? */
		reg_i2c->dr = ( address >> 8 ) | 0x01;
	}
	/**  */
	reg_i2c->dr = ( address & 0x00ff ) | 0x01;
	reg_i2c->cr |= ( MML_I2C_CR_START_MASK | MML_I2C_CR_RESTART_MASK );
	reg_i2c->cr &= ~MML_I2C_CR_START_MASK;
	/** No error */
	result = NO_ERROR;
	/** We're done */
mml_i2c_read_start_out:
	return result;
}

/****************************************************************************************************/
/**
 * Function to receive the data from the I2C device, this function should be called after the
 * communication has started i.e. mml_i2c_read_start() should be invoked before calling this function.
 * @param data							memory address where the read data to be stored (maximum = 8).
 * @param plength						memory address pointing to the 'length' number of bytes to receive.
 * @retval NO_ERROR						No error
 * @retval MML_I2C_ERR_INIT			Device is not initialized
 * @retval COMMON_ERR_NULL_PTR			NULL pointer parameter passed
 * @retval COMMON_ERR_UNKNOWN			Unknown error
 */
int mml_i2c_read(unsigned char *data, unsigned int *plength)
{
	int 										result = COMMON_ERR_UNKNOWN;
	unsigned int 								len;
	volatile mml_i2c_regs_t 					*reg_i2c = (volatile mml_i2c_regs_t*)MML_I2C_IOBASE;


	if ( !mml_i2c_context.first_init )
	{
		result = MML_I2C_ERR_INIT;
	}
	else if ( !data || !plength )
	{
		result = COMMON_ERR_NULL_PTR;
	}
	else if ( 8 < *plength )
	{
		result = MML_I2C_ERR_INVALID_PARAM;
	}
	else
	{
		len = 0;
		while( !( reg_i2c->sr & MML_I2C_SR_RXEMPTY_MASK ) )
		{
			*data++ = reg_i2c->dr;
			(*plength)--;
			len++;
			if ( !(*plength) )
			{
				break;
			}
			if ( reg_i2c->sr & MML_I2C_SR_RXEMPTY_MASK )
			{
				break;
			}
		}
		/**  */
		*plength = len;
		/** No error */
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/****************************************************************************************************/
/**
 * I2C Input/output command control function
 * @param command							IOCTL commands
 * @param data								data if present for the IOCTL command else
 * 											this parameter is ignored.
 * @retval NO_ERROR							No error
 * @retval MML_I2C_ERR_INIT				Device is not initialized
 * @retval COMMON_ERR_NULL_PTR				NULL pointer parameter passed
 * @retval COMMON_ERR_UNKNOWN				Unknown error
 */
int mml_i2c_ioctl(int command, void *data)
{
	int 										result = COMMON_ERR_UNKNOWN;
	volatile mml_i2c_regs_t 					*reg_i2c = (volatile mml_i2c_regs_t*)MML_I2C_IOBASE;


	if ( !mml_i2c_context.first_init )
	{
		result = MML_I2C_ERR_INIT;
	}
	else
	{
		switch( command )
		{
			case MML_I2C_SET_FREEZE:
				reg_i2c->cr |= MML_I2C_CR_FREEZE_MASK;
				break;
			case MML_I2C_CLEAR_FREEZE:
				reg_i2c->cr &= ~MML_I2C_CR_FREEZE_MASK;
				break;
			case MML_I2C_RXFLUSH:
				reg_i2c->cr &= ~MML_I2C_CR_RXFLUSH_MASK;
				break;
			case MML_I2C_TXFLUSH:
				reg_i2c->cr &= ~MML_I2C_CR_TXFLUSH_MASK;
				break;
			case MML_I2C_SET_READCOUNT:
				if ( !data )
				{
					result = COMMON_ERR_NULL_PTR;
					goto mml_i2c_ioctl_out;
				}
				reg_i2c->rcr = *((unsigned int *)data);
				break;
			case MML_I2C_CLEAR_READCOUNT:
				reg_i2c->rcr = 0;
				break;
			case MML_I2C_SET_AUTOSTART:
				reg_i2c->cr |= MML_I2C_CR_AUTOSTT_MASK;
				break;
			case MML_I2C_CLEAR_AUTOSTART:
				reg_i2c->cr &= ~MML_I2C_CR_AUTOSTT_MASK;
				break;
		}
		/** No error */
		result = NO_ERROR;
	}
	/** We're done */
mml_i2c_ioctl_out:
	return result;
}

/****************************************************************************************************/
/**
 * Function to stop the current read communication from the I2C device.
 * @retval NO_ERROR							No error
 * @retval MML_I2C_ERR_INIT				Device is not initialized
 * @retval COMMON_ERR_UNKNOWN				Unknown error
 */
int mml_i2c_stop(void)
{
	int 										result = COMMON_ERR_UNKNOWN;
	volatile mml_i2c_regs_t 					*reg_i2c = (volatile mml_i2c_regs_t*)MML_I2C_IOBASE;

	if ( !mml_i2c_context.first_init )
	{
		result = MML_I2C_ERR_INIT;
	}
	else
	{
		reg_i2c->cr |= MML_I2C_CR_STOP_MASK;
		/** No error */
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/****************************************************************************************************/
/**
 * Close the I2C Device and un install the I2C IRQ handler if installed.
 * @retval NO_ERROR							No error
 * @retval MML_I2C_ERR_INIT				Device is not initialized
 * @retval COMMON_ERR_UNKNOWN				Unknown error
 */
int mml_i2c_close(void)
{
	int 										result = COMMON_ERR_UNKNOWN;
	volatile mml_gcr_regs_t 					*reg_gcr = (mml_gcr_regs_t*)MML_GCR_IOBASE;

	if ( !mml_i2c_context.first_init )
	{
		result = MML_I2C_ERR_INIT;
	}
	else
	{
		/** Reset the I2C block */
		reg_gcr->rstr |= MML_GCR_RSTR_I2C_MASK;
		while( reg_gcr->rstr & MML_GCR_RSTR_I2C_MASK );
		/** Disable i2c clock */
		reg_gcr->perckcn |= ( 0x1  << MML_PERCKCN_DEV_I2C );
		/** Make available GPIO pins used by I2C */
		mml_gpio_clear_out_int_wake(MML_GPIO_DEV0, 30, 2);
		mml_gpio_disable_output(MML_GPIO_DEV0, 30, 2);
		/** Detach the I2C IRQ handler */
		mml_intc_detach_irq(MML_INTNUM_I2C);
		mml_i2c_context.first_init = 0;
		/** No error */
		result = NO_ERROR;
	}
	/** We're done */
    return result;
}

/***********************************************************************************************/
/**
 * Function to enable the desired I2C interrupt event/s
 * @param events							I2C Interrupt events to be enabled
 * @return
 * @retval NO_ERROR							No error
 * @retval MML_I2C_ERR_INIT				Device is not initialized
 * @retval COMMON_ERR_UNKNOWN				Unknown error
 */
int mml_i2c_enable_interrupt_events(unsigned int events)
{
	int 										result = COMMON_ERR_UNKNOWN;
	volatile mml_i2c_regs_t 					*reg_i2c = (volatile mml_i2c_regs_t*)MML_I2C_IOBASE;

	if ( !mml_i2c_context.first_init )
	{
		result = MML_I2C_ERR_INIT;
	}
	else
	{
		reg_i2c->ier |= events;
		/** No error */
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/***********************************************************************************************/
/**
 * Function to disable the desired I2C interrupt event/s
 * @param[in] events						I2C Interrupt events to be disabled
 * @retval NO_ERROR							No error
 */
int mml_i2c_disable_interrupt_events(unsigned int events)
{
	volatile mml_i2c_regs_t 					*reg_i2c = (volatile mml_i2c_regs_t*)MML_I2C_IOBASE;

	reg_i2c->ier &= ~events;
	/** We're done */
	return NO_ERROR;
}

/***********************************************************************************************/
/**
 * Function to acknowledge/clear the desired I2C interrupt status
 * @param[in] status					interrupt status flag/s to be cleared
 * @retval NO_ERROR						No error
 */
int mml_i2c_clear_interrupt_status(unsigned int status)
{
	mml_i2c_regs_t								*reg_i2c = (volatile mml_i2c_regs_t*)MML_I2C_IOBASE;

	reg_i2c->isr &= ~status;
	/** We're done */
	return NO_ERROR;
}

/***********************************************************************************************/
/**
 * Function to read the current interrupt status flags
 * @param[out] status			pointer to interrupt status flag
 * @retval NO_ERROR				No error
 */
int mml_i2c_interrupt_status(unsigned int *status)
{
	mml_i2c_regs_t								*reg_i2c = (volatile mml_i2c_regs_t*)MML_I2C_IOBASE;

	*status = reg_i2c->isr;
	/** We're done */
	return NO_ERROR;
}

/***********************************************************************************************/
/**
 * Get the I2C bus status
 * @param[out] status		I2C bus status
 * @retval NO_ERROR			No error
 */
int mml_i2c_bus_status(unsigned int *status)
{
	volatile mml_i2c_regs_t 					*reg_i2c = (volatile mml_i2c_regs_t*)MML_I2C_IOBASE;

	*status = reg_i2c->sr;
	/** We're done */
	return NO_ERROR;
}

/******************************************************************************/
/* EOF */
