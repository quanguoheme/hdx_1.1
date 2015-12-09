/*
 * mml_uart.c --
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
 * Created on: Jun 29, 2012
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
#include <mml_io.h>
#include <mml.h>
#include <mml_gcr_regs.h>
#include <mml_gpio_regs.h>
#include <mml_intc.h>
#include <cobra_defines.h>
/** Local includes */
#include <mml_uart.h>
#include <mml_uart_regs.h>


/** Should be in .bss section */
__attribute__((section(".bss"))) mml_uart_context_t uart_context;

#ifdef _STAND_ALONE_DRIVER_UART_
/* This compilation flag is activated only in driver development context
* without any application using it.
* DO NOT define it in application/test context then. */
/*****************************************************************************/
int main(void)
{
	/** We're done */
	return NO_ERROR;
}
#endif /* _STAND_ALONE_DRIVER_UART_ */

/******************************************************************************/
/**
 * The function is used to initialize the UART port.
 * @param[in] port						The UART interface identifier
 * @param[in] config					Configuration structure which specifies transfer size
 * 										(byte width), baud rate, parity, parity mode,
 * 										stops bits and flow control
 * @retval NO_ERROR					No error
 * @retval COMMON_ERR_OUT_OF_RANGE	Invalid UART port number
 * @retval COMMON_ERR_NULL_PTR		NULL pointer parameter passed
 * @retval COMMON_ERR_BAD_STATE		UART port is already initialized
 * @retval COMMON_ERR_INVAL			Invalid configuration parameter
 */
int mml_uart_init(mml_uart_id_t port, mml_uart_config_t config)
{
	int											result = COMMON_ERR_UNKNOWN;
	unsigned int								clock1;
	unsigned int								clock2;
	unsigned int								mask = ( MML_UARTx_GPIO_RXD | MML_UARTx_GPIO_TXD | MML_UARTx_GPIO_RTS | MML_UARTx_GPIO_CTS );
	volatile mml_gpio_regs_t					*reg_gpio;
	volatile mml_uart_regs_t					*reg_uart;
	volatile mml_gcr_regs_t						*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;

	/** First initialization ? */
	if ( !uart_context.first_init )
	{
		/** Assign interface base address */
		uart_context.port[MML_UART_DEV0].reg = (volatile mml_uart_regs_t*)MML_UART0_IOBASE;
		uart_context.port[MML_UART_DEV1].reg = (volatile mml_uart_regs_t*)MML_UART1_IOBASE;
		uart_context.port[MML_UART_DEV2].reg = (volatile mml_uart_regs_t*)MML_UART2_IOBASE;
#ifdef _UART_RESET_AT_INIT_
		/** Reset all UART's interface */
		reg_gcr->rstr |= MML_GCR_RSTR_UARTS_MASK;
		/** Wait for UARTs reset to be done */
		while( MML_GCR_RSTR_UARTS_MASK & reg_gcr->rstr );
		/** Stop clock of each UART. It'll be set independently */
		reg_gcr->perckcn |= ( 1 << MML_PERCKCN_DEV_UART0 );
		reg_gcr->perckcn |= ( 1 << MML_PERCKCN_DEV_UART1 );
		reg_gcr->perckcn |= ( 1 << MML_PERCKCN_DEV_UART2 );
#endif /* _UART_RESET_AT_INIT_ */
		/** To be done once only */
		uart_context.first_init = 1;
	}
	/** Validate the input parameter port */
	switch( port )
	{
		case MML_UART_DEV0:
			/** UART base assignment */
			reg_uart = (volatile mml_uart_regs_t*)uart_context.port[port].reg;
			/** GPIO base assignment */
			reg_gpio = (volatile mml_gpio_regs_t*)MML_GPIO0_IOBASE;
			/** Enable the GPIO0 clock */
			clock2 = MML_PERCKCN_DEV_GPIO0;
			/** Enable the UART0 clock */
			clock1 = MML_PERCKCN_DEV_UART0;
			/** Enable the alternate function for UART0 GPIO0[8:11] */
			mask <<= MML_UART0_GPIO_CFG_OFFSET;
			/**  */
			uart_context.port[port].irq = MML_INTNUM_UART0;
			break;
		case MML_UART_DEV1:
			/** UART base assignment */
			reg_uart = (volatile mml_uart_regs_t*)uart_context.port[port].reg;
			/** GPIO base assignment */
			reg_gpio = (volatile mml_gpio_regs_t*)MML_GPIO0_IOBASE;
			/** Enable the GPIO0 clock */
			clock2 = MML_PERCKCN_DEV_GPIO0;
			/** Enable the UART1 clock */
			clock1 = MML_PERCKCN_DEV_UART1;
			/** Enable the alternate function for UART1 GPIO1[12:15] */
			mask <<= MML_UART1_GPIO_CFG_OFFSET;
			/**  */
			uart_context.port[port].irq = MML_INTNUM_UART1;
			break;
		case MML_UART_DEV2:
			/** UART base assignment */
			reg_uart = (volatile mml_uart_regs_t*)uart_context.port[port].reg;
			/** GPIO base assignment */
			reg_gpio = (volatile mml_gpio_regs_t*)MML_GPIO1_IOBASE;
			/** Enable the GPIO1 clock */
			clock2 = MML_PERCKCN_DEV_GPIO1;
			/** Enable the UART0 clock */
			clock1 = MML_PERCKCN_DEV_UART2;
			/** Enable the alternate function for UART2 GPIO1[14:17] */
			mask <<= MML_UART2_GPIO_CFG_OFFSET;
			/**  */
			uart_context.port[port].irq = MML_INTNUM_UART2;
			break;
		default:
			result = COMMON_ERR_OUT_OF_RANGE;
			goto mml_uart_init_out;
	}
	/** Enable chosen UART clock */
	reg_gcr->perckcn &= ~( 1 << clock1 );
	/** Clock up GPIOx */
	reg_gcr->perckcn &= ~( 1 << clock2 );
	/** Activate UART functionality in GPIOx */
	reg_gpio->en &= ~mask;
	reg_gpio->en1 &= ~mask;
	/** Disable all the UART port interrupts */
	reg_uart->ier = 0;
	reg_uart->isr = 0;
	/** Configure the UART communication parameters */
	result = mml_uart_set_config(port, &config);
	/** Flush Tx & Rx FIFOs */
	mml_uart_flush_raw(port, ( MML_UART_CR_TXFLUSH_MASK | MML_UART_CR_RXFLUSH_MASK ));
	/** We're done */
mml_uart_init_out:
	return result;
}

/******************************************************************************/
int mml_uart_reset_interface(void)
{
	unsigned int								loop = K_COBRA_RESET_WAIT_LOOP_MAX;
	volatile mml_gcr_regs_t						*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;

	/** Launch all UARTs reset */
	reg_gcr->rstr |= MML_GCR_RSTR_UARTS_MASK;
	/** Wait 'til it's done */
	while( ( reg_gcr->rstr & MML_GCR_RSTR_UARTS_MASK ) && loop-- );
	/** We're done */
	return NO_ERROR;
}

/******************************************************************************/
/**
 * The function is used to configure the communication parameters of UART Interface.
 * @param[in] port						The UART interface identifier
 * @param[in] config					Configuration structure which specifies transfer
 * 										size(data byte width), baud rate, parity, parity mode,
 * 										stops bits and flow control
 * 										NOTE: structure must be allocated and filled by the
 * 										caller
 * @retval NO_ERROR					No error
 * @retval COMMON_ERR_OUT_OF_RANGE	Invalid UART port number
 * @retval COMMON_ERR_NULL_PTR		NULL pointer parameter passed
 * @retval COMMON_ERR_BAD_STATE		UART port is not initialized
 * @retval COMMON_ERR_INVAL			Invalid configuration parameter
 * @retval COMMON_ERR_RUNNING			UART is currently busy with transmitting or receiving the data
 */
int mml_uart_set_config(mml_uart_id_t port, mml_uart_config_t *config)
{
	int											result = COMMON_ERR_UNKNOWN;
	volatile mml_uart_regs_t					*reg_uart;

	/** Validate the input parameter port */
	if ( MML_UART_DEV_COUNT <= port )
	{
		result = COMMON_ERR_OUT_OF_RANGE;
		goto mml_uart_set_config_out;
	}
	if ( !config )
	{
		result = COMMON_ERR_NULL_PTR;
		goto mml_uart_set_config_out;
	}
	/** Check UART port is initialized */
	if ( !uart_context.first_init )
	{
		result = COMMON_ERR_BAD_STATE;
		goto mml_uart_set_config_out;
	}
	/** Check the configuration parameters */
	if ( MML_UART_DATA_TRANSFER_SIZE_8_BITS < config->data_bits )
	{
		result = COMMON_ERR_INVAL;
		goto mml_uart_set_config_out;
	}
	if ( MML_UART_PARITY_NONE < config->parity )
	{
		result = COMMON_ERR_INVAL;
		goto mml_uart_set_config_out;
	}
	/** If no parity is specified then ignore the parameter parity_mode */
	if ( ( MML_UART_PARITY_NONE != config->parity ) &&
		( MML_UART_PARITY_MODE_ZEROS < config->parity_mode ) )
	{
		result = COMMON_ERR_INVAL;
		goto mml_uart_set_config_out;
	}
	if ( MML_UART_STOPBITS_ONE_TO_TWO < config->stop_bits )
	{
		result = COMMON_ERR_INVAL;
		goto mml_uart_set_config_out;
	}
	if ( MML_UART_HW_FLOW_CTL_ENBALE < config->flwctrl )
	{
		result = COMMON_ERR_INVAL;
		goto mml_uart_set_config_out;
	}
	if ( ( MML_UART_HW_FLOW_CTL_DISABLE == config->flwctrl ) &&
		( MML_UART_RTS_IO_LEVEL_HIGH < config->rts_ctl ) )
	{
		result = COMMON_ERR_INVAL;
		goto mml_uart_set_config_out;
	}
	/** Check the baud rate range */
	if ( ( MML_UART_BAUD_RATE_MIN > config->baudrate ) || ( MML_UART_BAUD_RATE_MAX < config->baudrate ) )
	{
		result = COMMON_ERR_INVAL;
		goto mml_uart_set_config_out;
	}
	/** Assign  the base address to access the registers */
	reg_uart = (volatile mml_uart_regs_t*)uart_context.port[port].reg;
mml_uart_set_config_busy:
	/** Check if the UART is busy (not receiving nor transmitting data) */
	if ( !( reg_uart->sr & ( MML_UART_STATUS_TX_BUSY | MML_UART_STATUS_RX_BUSY ) ) )
	{
		register unsigned int					temp = 0;
		unsigned int		  					idiv;
		unsigned int							ddiv;

		ddiv = (Clock_Speed << 1) / MML_UART_BRR_DIV_FACTOR;

		/** Formula for calculating the baud rate
		 * DIV = (Nominal Frequency) /(128 * Fdesired)
		 * IDIV = (integer) DIV
		 * DDIV = (DIV - IDIV) * 128
		 */
		/** Compute the integer part(IDIV) of DIV */
		idiv = config->baudrate;
		/** Compute the Decimal part(DDIV) of DIV */
		reg_uart->brr = MML_UART_BAUD_RATE_MASK_IDIV & ( ddiv / idiv );
		/** Compute the Decimal part(DDIV) of DIV */
		reg_uart->rcr = MML_UART_BAUD_RATE_MASK_DDIV & ( ( ( ddiv % idiv ) * MML_UART_BRR_DIV_FACTOR ) / config->baudrate );
		/** Set the communication control parameters */
		/** Set data bit transfer size */
		temp = reg_uart->cr;
		temp &= ~MML_UART_CTL_TRANSFER_SIZE_MASK;
		temp |= ( config->data_bits << MML_UART_CR_SIZE_OFST );
		/** Set UART Parity */
		if ( MML_UART_PARITY_NONE == config->parity )
		{
			/** Disable the parity */
			temp &= ~MML_UART_CR_PAREN_MASK;
		}
		else
		{
			/** Set the parity */
			temp |= MML_UART_CR_PAREN_MASK;
			temp &= ~MML_UART_CR_PAREO_MASK;
			temp |= ( config->parity << MML_UART_CR_PAREO_OFST );
			/** Set the parity mode based on 0's or 1's */
			if ( MML_UART_PARITY_NONE != config->parity )
			{
				temp &= ~MML_UART_CR_PARMD_MASK;
				temp |= ( config->parity_mode << MML_UART_CR_PARMD_OFST );
			}
		}
		/** Set stop bits */
		temp &= ~MML_UART_CR_STOP_MASK;
		temp |= ( config->stop_bits << MML_UART_CR_STOP_OFST );
		/** Set hardware flow control */
		temp &= ~MML_UART_CR_RTSCTS_MASK;
		temp |= ( config->flwctrl << MML_UART_CR_RTSCTS_OFST );
		/** Set character threshold to 1 */
		temp &= ~MML_UART_CR_RXTHD_MASK;
		temp |= ( 0x1 << MML_UART_CR_RXTHD_OFST );
		/** Finally write the configurations in the UART control register */
		reg_uart->cr = temp;
		/** UART RTS IO level [Pin register] */
		if ( MML_UART_HW_FLOW_CTL_DISABLE != config->flwctrl )
		{
			reg_uart->pnr |= ( config->rts_ctl << MML_UART_PR_RTS_OFST );
		}
		/**  */
		if ( MML_UART_DEV0 == port )
		{
			/** Check if handler is not null */
			if ( config->handler )
			{
				/** Enable the interrupt */
				result = mml_intc_setup_irq(uart_context.port[port].irq, MML_INTC_PRIO_3, config->handler);
				if ( result )
				{
					result = MML_UART_ERR_IRQ_SET;
				}
			}
			else
			{
				result = COMMON_ERR_NULL_PTR;
			}
		}
		else
		{
			result = NO_ERROR;
		}
	}
	else
	{
		goto mml_uart_set_config_busy;
	}
	/** We're done */
mml_uart_set_config_out:
	return result;
}

/******************************************************************************/
/**
 * The function is used to receive a character data from the UART Interface.
 * @param[in] port							The UART interface identifier
 * @param[out] data							Pointer to the character data where
 * 											the received data to be stored
 * @retval NO_ERROR						No error
 * @retval COMMON_ERR_OUT_OF_RANGE		Invalid UART port number
 * @retval COMMON_ERR_NULL_PTR			Parameter passed is NULL pointer
 * @retval COMMON_ERR_BAD_STATE			UART port is not initialized
 * @retval COMMON_ERR_RUNNING				UART RX FIFO is empty (try later)
 */
int mml_uart_read_char(mml_uart_id_t port, unsigned char *data)
{
	int											result = COMMON_ERR_UNKNOWN;

	/** Validate the input parameter port */
	if ( MML_UART_DEV_COUNT <= port )
	{
		result = COMMON_ERR_OUT_OF_RANGE;
	}
	else if ( !data )
	{
		result = COMMON_ERR_NULL_PTR;
	}
	/** Check UART port is initialized */
	else if ( !uart_context.port[port].reg )
	{
		result = COMMON_ERR_NOT_INITIALIZED;
	}
	else
	{
		/** Assign the UART base address to access the registers */
		volatile mml_uart_regs_t				*reg_uart = (volatile mml_uart_regs_t*)uart_context.port[port].reg;

		/* Check if the RX FIFO is empty */
		if ( !( reg_uart->sr & MML_UART_STATUS_RX_EMPTY ) )
		{
			unsigned int fifodata = reg_uart->dr;
			/** Check if the parity error occurred during the byte reception */
			if ( !( fifodata & MML_UART_DATA_RX_PARITY ) )
			{
				/* No parity error receive the data */
				*data = (unsigned char)fifodata;
				result = NO_ERROR;
			}
			else
			{
				/** Parity error */
				result = COMMON_ERR_NO_MATCH;
			}
		}
		else
		{
			result = COMMON_ERR_RUNNING;
		}
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * The function is used to transmit a character data from the UART Interface.
 * @param[in] port							The UART interface identifier
 * @param[in] data							The character data to transmit
 * @retval NO_ERROR						No error
 * @retval COMMON_ERR_OUT_OF_RANGE		Invalid UART port number
 * @retval COMMON_ERR_BAD_STATE			UART port is not initialized
 * @retval COMMON_ERR_RUNNING				UART TX FIFO is full and try later
 */
int mml_uart_write_char(mml_uart_id_t port, unsigned char data)
{
	int											result = COMMON_ERR_UNKNOWN;

	/** Validate the input parameter port */
	if ( MML_UART_DEV_COUNT <= port )
	{
		result = COMMON_ERR_OUT_OF_RANGE;
	}
	/** Check UART port is initialized */
	else if (  !uart_context.port[port].reg )
	{
		result = COMMON_ERR_NOT_INITIALIZED;
	}
	else
	{
		/** Assign the UART base address to access the registers */
		volatile mml_uart_regs_t				*reg_uart = (volatile mml_uart_regs_t*)uart_context.port[port].reg;

		/** Check if the TX FIFO is full */
		if ( !( reg_uart->sr & MML_UART_STATUS_TX_FULL ) )
		{
			reg_uart->dr = data;
			/** Wait EOT */
			while( !( reg_uart->sr & MML_UART_STATUS_TX_EMPTY ) || ( reg_uart->sr & MML_UART_STATUS_TX_BUSY ) );
			result = NO_ERROR;
		}
		else
		{
			result = COMMON_ERR_RUNNING;
		}
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * The function is used to flush the bytes left out in the transmit FIFO of the UART Interface.
 * @param[in] port							The UART interface identifier
 * @retval NO_ERROR						No error
 * @retval COMMON_ERR_OUT_OF_RANGE		Invalid UART port number
 * @retval COMMON_ERR_BAD_STATE			UART port is not initialized
 */
int mml_uart_flush(mml_uart_id_t port, unsigned int mode)
{
	int											result = COMMON_ERR_UNKNOWN;

	/** Validate the input parameter port */
	if ( MML_UART_DEV_COUNT <= port )
	{
		result = COMMON_ERR_OUT_OF_RANGE;
	}
	/** Check UART port is initialized */
	else if (  !uart_context.port[port].reg )
	{
		result = COMMON_ERR_NOT_INITIALIZED;
	}
	else
	{
		mml_uart_flush_raw(port, mode);
		/**  */
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * The function is used to retrieve the state of an UART Interface.
 * @param[in] port							The UART interface identifier
 * @param[out] status						Current status of the UART Interface.
 *                          				\n It is one or a combination of the following values:
 *                          				\li MML_UART_STATUS_TX_BUSY
 *                          				\li MML_UART_STATUS_RX_BUSY
 *                          				\li MML_UART_STATUS_RX_EMPTY
 *                          				\li MML_UART_STATUS_RX_FULL
 *                          				\li MML_UART_STATUS_TX_EMPTY
 *                          				\li MML_UART_STATUS_TX_FULL
 * @retval NO_ERROR						No error
 * @retval COMMON_ERR_OUT_OF_RANGE		Invalid UART port number
 * @retval COMMON_ERR_NULL_PTR			NULL pointer passed
 * @retval COMMON_ERR_BAD_STATE			UART port is not initialized
 */
int mml_uart_port_status(mml_uart_id_t port, int *status)
{
	int											result = COMMON_ERR_UNKNOWN;

	/** Validate the input parameter port */
	if ( MML_UART_DEV_COUNT <= port )
	{
		result = COMMON_ERR_OUT_OF_RANGE;
	}
	/**  */
	else if ( !status )
	{
		result = COMMON_ERR_NULL_PTR;
	}
	/** Check UART port is initialized */
	else if (  !uart_context.port[port].reg )
	{
		result = COMMON_ERR_NOT_INITIALIZED;
	}
	else
	{
		*status = uart_context.port[port].reg->sr;
		/* [TBD] interrupt status */
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * The function is used to shutdown the UART Interface.
 * @param[in] port							The UART interface identifier
 * @retval NO_ERROR						No error
 * @retval COMMON_ERR_OUT_OF_RANGE		Invalid UART port number
 * @retval COMMON_ERR_BAD_STATE			UART port is not initialized
 */
int mml_uart_deinit(mml_uart_id_t port)
{
	int											result = COMMON_ERR_UNKNOWN;

	/** Validate the input parameter port */
	if ( MML_UART_DEV_COUNT <= port )
	{
		result = COMMON_ERR_OUT_OF_RANGE;
	}
	/** Check UART port is initialized */
	else if (  !uart_context.port[port].reg )
	{
		result = COMMON_ERR_NOT_INITIALIZED;
	}
	else
	{
		unsigned int							clock1;
		unsigned int							mask;
		volatile mml_gcr_regs_t					*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;
		volatile mml_gpio_regs_t				*reg_gpio = (volatile mml_gpio_regs_t*)MML_GPIO0_IOBASE;

		mml_uart_flush_raw(port, ( MML_UART_CR_TXFLUSH_MASK | MML_UART_CR_RXFLUSH_MASK ));
		/** Disable the UART interrupts */
		uart_context.port[port].reg->ier = 0;
		switch( port )
		{
			case MML_UART_DEV0:
				/** Disable the UART0 clock */
				clock1 = MML_PERCKCN_DEV_UART0;
				/** Disable the alternate function for UART0 GPIO0[31:28] */
				mask = MML_UART0_GPIO_CFG_MASK;
				break;
			case MML_UART_DEV1:
				/** Disable the UART1 clock */
				clock1 = MML_PERCKCN_DEV_UART1;
				/** Disable the alternate function for UART1 GPIO0[27:24] */
				mask = MML_UART0_GPIO_CFG_MASK;
				break;
			default:
				result = COMMON_ERR_INVAL;
				goto mml_uart_deinit_out;
		}
		/**  */
		reg_gcr->perckcn |= ( 1 << clock1 );
		reg_gpio->en |= mask;
		reg_gpio->en1 |= mask;
		uart_context.first_init = 0;
		result = NO_ERROR;
	}
	/** We're done */
mml_uart_deinit_out:
	return result;
}

/******************************************************************************/
void mml_uart_flush_raw(mml_uart_id_t port, unsigned int mode)
{
	/** Assign the UART base address to access the registers */
	volatile mml_uart_regs_t					*reg_uart = (volatile mml_uart_regs_t*)uart_context.port[port].reg;

	reg_uart->cr |= mode;
	while( reg_uart->cr & mode );
	/** We're done */
	return;
}

/******************************************************************************/
/* IRQ Management *************************************************************/
/******************************************************************************/
int mml_uart_callback_handler_register(mml_uart_id_t port, mml_uart_handler_t handler)
{
	int 										result = COMMON_ERR_UNKNOWN;

	/** Check input parameter */
	if ( ( MML_UART_DEV0 != port ) &&
		( MML_UART_DEV1 != port ) &&
		( MML_UART_DEV2 != port ) )
	{
		result = COMMON_ERR_INVAL;
	}
	/** Check state */
	else if (  !uart_context.port[port].reg )
	{
		result = MML_UART_ERR_NOT_INITIALIZED;
	}
	else if ( !handler )
	{
		result = COMMON_ERR_NULL_PTR;
	}
	else
	{
		/** Attach to platform handler */
		mml_intc_attach_irq(uart_context.port[port].irq, MML_INTC_PRIO_3, handler);
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
int mml_uart_callback_handler_unregister(mml_uart_id_t port)
{
	int 										result = COMMON_ERR_UNKNOWN;

	/** Check input parameter */
	if ( ( MML_UART_DEV0 != port ) &&
		( MML_UART_DEV1 != port ) &&
		( MML_UART_DEV2 != port ) )
	{
		result = COMMON_ERR_INVAL;
	}
	/** Check state */
	else if (  !uart_context.port[port].reg )
	{
		result = MML_UART_ERR_NOT_INITIALIZED;
	}
	else
	{
		/** Detach from platform handler */
		mml_intc_detach_irq(uart_context.port[port].irq);
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * [TBD]
 * @param id	UART device id
 * @return
 */
int mml_uart_interrupt_activate(mml_uart_id_t port, unsigned char activation)
{
	int											result = COMMON_ERR_UNKNOWN;

	/** Check input port */
	if ( ( MML_UART_DEV0 != port ) &&
		( MML_UART_DEV1 != port ) &&
		( MML_UART_DEV2 != port ) )
	{
		result = COMMON_ERR_INVAL;
	}
	/**  */
	else if ( TRUE == activation )
	{
		/** Enable UART's irq */
		mml_intc_enable_irq(uart_context.port[port].irq);
		result = NO_ERROR;
	}
	else
	{
		/** Disable UART's irq */
		mml_intc_disable_irq(uart_context.port[port].irq);
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * [TBD]
 * @param id	Timer device id
 * @return
 */
void mml_uart_interrupt_clear(mml_uart_id_t port, unsigned int uart_irq)
{
	volatile mml_uart_regs_t					*reg_uart = (volatile mml_uart_regs_t*)uart_context.port[port].reg;

	/** Clear the source level interrupt */
	reg_uart->isr &= ~(uart_irq);
	return;
}

/******************************************************************************/
/**
 * [TBD]
 * @param id	Timer device id
 * @return
 */
void mml_uart_interrupt_set(mml_uart_id_t port, unsigned int uart_irq)
{
	volatile mml_uart_regs_t					*reg_uart = (volatile mml_uart_regs_t*)uart_context.port[port].reg;

	/** Clear status ... just in case */
	reg_uart->isr &= ~(uart_irq);
	/** Set the source level interrupt */
	reg_uart->ier |= uart_irq;
	/** We're done */
	return;
}

/******************************************************************************/
void mml_uart_interrupt_ack(mml_uart_id_t port)
{
	/** Acknowledge interrupt at platform level */
	mml_intc_ack_irq(uart_context.port[port].irq);
	/** We're done */
	return;
}

/******************************************************************************/
/** COBRA adaptation */
int mml_uart_initialise(mml_uart_regs_t *reg_uart, unsigned config, mml_uart_brr_t baud)
{

	reg_uart->cr = config;
	reg_uart->brr = baud.brr;
	reg_uart->rcr = baud.rcr;
	/** We're done */
	return NO_ERROR;
}

/******************************************************************************/
char mml_uart_read_byte(mml_uart_regs_t *reg_uart)
{
	while( MML_UART_SR_RXFULL_MASK & reg_uart->sr );
	/** We're done */
	return (char)reg_uart->dr;
}

/******************************************************************************/
void mml_uart_write_byte(mml_uart_regs_t *reg_uart, char c)
{
	while( MML_UART_SR_TXEMPTY_MASK & reg_uart->sr );
	/**  */
	reg_uart->dr = (unsigned)c;
	/** We're done */
	return;
}

/******************************************************************************/
/* EOF */
