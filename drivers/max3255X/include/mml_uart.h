/*
 * mml_uart.h --
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

#ifndef _MML_UART_H_
#define _MML_UART_H_

/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
/** Local includes */
#include <mml_uart_regs.h>

/* Defines ********************************************************************/
#define	MML_UART_BASE_ERROR						COBRA_UART_BASE_ERR
#define MML_UART_BAUD_RATE_MIN					1200
#define MML_UART_BAUD_RATE_MAX					921600

/** COBRA adaptation */
#define UARTB_CR_SIZE_8BIT       				0x00000c00

/** Nominal Frequency @60MHz */
#define MML_BR_9600 							{156, 32}
#define MML_BR_57600       						{26, 5}
#define MML_BR_115200      						{13, 5}

/* Macros *********************************************************************/

/* Enumerations ***************************************************************/

/** UART errors list */
typedef enum
{
	MML_UART_ERR_MIN = MML_UART_BASE_ERROR,
	/** Error Code: UART port not initialized */
	MML_UART_ERR_NOT_INITIALIZED,
	/** Error Code: Invalid port or UART port not available */
	MML_UART_ERR_NOT_AVAILABLE,
	/** Error Code: UART TX error */
	MML_UART_ERR_TX,
	/** Error Code: UART RX error */
	MML_UART_ERR_RX,
	/** Error Code: Interrupt can not be initialized */
	MML_UART_ERR_IRQ_SET,
	/** Error Code: Generic error for unknown behavior */
	MML_UART_ERR_UNKNOWN,
	MML_UART_ERR_MAX = MML_UART_ERR_UNKNOWN

} mml_uart_errors_t;

/* UART EVEN or ODD parity */
/* NOTE: Don't interchange the values */
typedef enum
{
	MML_UART_PARITY_EVEN = 0,
	MML_UART_PARITY_ODD = 1,
	MML_UART_PARITY_NONE = 2

} mml_uart_parity_t;

/* UART parity mode i.e. the parity is based on 0's or 1's in the frame */
typedef enum
{
	MML_UART_PARITY_MODE_ONES = 0x00,
	MML_UART_PARITY_MODE_ZEROS = 0x01,

} mml_uart_parity_mode_t;

/* UART data byte transfer size */
typedef enum
{
	MML_UART_DATA_TRANSFER_SIZE_5_BITS = 0,
	MML_UART_DATA_TRANSFER_SIZE_6_BITS = 1,
	MML_UART_DATA_TRANSFER_SIZE_7_BITS = 2,
	MML_UART_DATA_TRANSFER_SIZE_8_BITS = 3

} mml_uart_tarsfer_size_t;

/* UART hardware flow control enable/disable */
typedef enum
{
	MML_UART_HW_FLOW_CTL_DISABLE = 0x00,
	MML_UART_HW_FLOW_CTL_ENBALE = 0x01

} mml_uart_flow_ctl_t;

/* UART RTS IO level [Pin register] */
typedef enum
{
	MML_UART_RTS_IO_LEVEL_LOW = 0,
	MML_UART_RTS_IO_LEVEL_HIGH = 1

} mml_uart_rts_ctl_t;

/* UART Stop bits */
typedef enum
{
	/* 1 stop bit generated */
	MML_UART_STOPBITS_ONE = 0x00,
	/* 1.5 or 2 stop bits are generated */
	MML_UART_STOPBITS_ONE_TO_TWO = 0x01

} mml_uart_stop_bits_t;

/**  UART FSM */
typedef enum
{
	MML_UART_STATE_MIN = 0,
	MML_UART_STATE_NOT_INITIALIZED = MML_UART_STATE_MIN,
	MML_UART_STATE_INITIALIZED,
	MML_UART_STATE_CLOSED,
	MML_UART_STATE_MAX = MML_UART_STATE_CLOSED,
	MML_UART_STATE_COUNT

} mml_uart_state_t;

/** UART device id's */
typedef volatile enum
{
	/** Minimum value */
	MML_UART_DEV_MIN = 0,
	MML_UART_DEV0 = MML_UART_DEV_MIN,
	MML_UART_DEV1,
	MML_UART_DEV2,
	/** Maximum value */
	MML_UART_DEV_MAX = MML_UART_DEV2,
	MML_UART_DEV_COUNT

} mml_uart_id_t;

/* Structures *****************************************************************/
typedef void(*mml_uart_handler_t)(void);

typedef struct
{
	unsigned int								brr;
	unsigned int								rcr;

} mml_uart_brr_t;

/* UART configuration information */
typedef struct
{
	/** Data bits */
	mml_uart_tarsfer_size_t						data_bits;
	/** UART Parity */
	mml_uart_parity_t							parity;
	/* Parity Mode based on 0's or 1's in the frame */
	mml_uart_parity_mode_t						parity_mode;
	/** Stop bits */
	mml_uart_stop_bits_t						stop_bits;
	/** Hardware Flow Control bit */
	mml_uart_flow_ctl_t							flwctrl;
	/* UART RTS IO level */
	mml_uart_rts_ctl_t							rts_ctl;
	/** Baud rate */
	unsigned int								baudrate;
	/** IRQ handler */
	volatile mml_uart_handler_t					handler;

} mml_uart_config_t;


typedef struct
{
	/** Interrupt number */
	unsigned int 								irq;
	/** Base address of the interface */
	volatile mml_uart_regs_t					*reg;
	/** UART port parameters */
	mml_uart_config_t							config;

} mml_uart_port_conf_t;

/** UART port context information */
typedef struct
{
	/**  */
	unsigned int								first_init;
	/**  */
	mml_uart_port_conf_t						port[MML_UART_DEV_COUNT];

} mml_uart_context_t;


/* Variables ******************************************************************/

/* Functions ******************************************************************/
/** Initialize an UART. */
int mml_uart_initialise(mml_uart_regs_t *uart, unsigned config, mml_uart_brr_t baud);
char mml_uart_read_byte(mml_uart_regs_t *uart);
void mml_uart_write_byte(mml_uart_regs_t *uart, char c);
int mml_uart_init(mml_uart_id_t port, mml_uart_config_t config);
int mml_uart_reset_interface(void);
int mml_uart_set_config(mml_uart_id_t port, mml_uart_config_t *config);
int mml_uart_read_char(mml_uart_id_t port, unsigned char *data);
int mml_uart_write_char(mml_uart_id_t port, unsigned char data);
int mml_uart_flush(mml_uart_id_t port, unsigned int mode);
#define M_MML_UART_FLUSH_TX(_uart_port_)			mml_uart_flush(_uart_port_, MML_UART_CR_TXFLUSH_MASK)
#define M_MML_UART_FLUSH_RX(_uart_port_)			mml_uart_flush(_uart_port_, MML_UART_CR_RXFLUSH_MASK)
#define M_MML_UART_FLUSH_ALL(_uart_port_)			mml_uart_flush(_uart_port_, ( MML_UART_CR_TXFLUSH_MASK | MML_UART_CR_RXFLUSH_MASK ))
int mml_uart_port_status(mml_uart_id_t port, int *status);
int mml_uart_deinit(mml_uart_id_t port);
void mml_uart_flush_raw(mml_uart_id_t port, unsigned int mode);
int mml_uart_callback_handler_register(mml_uart_id_t port, mml_uart_handler_t handler);
int mml_uart_callback_handler_unregister(mml_uart_id_t port);
int mml_uart_interrupt_activate(mml_uart_id_t port, unsigned char activation);
#define M_MML_UART_INTERRUPT_ENABLE(_uart_port_)	mml_uart_interrupt_activate(_uart_port_, TRUE)
#define M_MML_UART_INTERRUPT_DISABLE(_uart_port_)	mml_uart_interrupt_activate(_uart_port_, FALSE)
void mml_uart_interrupt_clear(mml_uart_id_t port, unsigned int uart_irq);
void mml_uart_interrupt_set(mml_uart_id_t port, unsigned int uart_irq);
void mml_uart_interrupt_ack(mml_uart_id_t port);
int mml_uart_initialise(mml_uart_regs_t *uart, unsigned config, mml_uart_brr_t baud);
char mml_uart_read_byte(mml_uart_regs_t *uart);
void mml_uart_write_byte(mml_uart_regs_t *uart, char c);

#endif /* _MML_UART_H_ */

/******************************************************************************/
/* EOF */
