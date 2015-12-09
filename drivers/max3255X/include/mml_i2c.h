/*
 * mml_i2c.h --
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

#ifndef _MML_I2C_H_
#define _MML_I2C_H_

/** Global includes */
#include <errors.h>
/** Other includes */
/** Local includes */
#include <mml_i2c_regs.h>


/** Defines *******************************************************************/
#define	K_COBRA_I2C_WAIT_LOOP				500000

/** Enumerations **************************************************************/
/** I2C errors list */
typedef enum
{
	MML_I2C_ERR_MIN = COBRA_I2C_BASE_ERR,
	MML_I2C_ERR_INIT,
	MML_I2C_ERR_READY,
	MML_I2C_ERR_BUSY,
	MML_I2C_ERR_AVAIL,
	MML_I2C_ERR_INVALID_CMD,
	MML_I2C_ERR_PADCONFIG,
	MML_I2C_ERR_NOCONFIG,
	MML_I2C_ERR_INVALID_BAUD,
	MML_I2C_ERR_INVALID_RXTHRD,
	MML_I2C_ERR_INVALID_ADDRESS,
	MML_I2C_ERR_INVALID_PARAM,
	MML_I2C_ERR_EVENT,
	MML_I2C_ERR_GPIO_CONFIG
	/** Error Code: No such device */
} mml_i2c_errors_t;

typedef enum mml_i2c_cmd_id
{
	MML_I2C_SET_FREEZE = 0,
	MML_I2C_CLEAR_FREEZE,
	MML_I2C_TXFLUSH,
	MML_I2C_RXFLUSH,
	MML_I2C_SET_READCOUNT,
	MML_I2C_CLEAR_READCOUNT,
	MML_I2C_SET_AUTOSTART,
	MML_I2C_CLEAR_AUTOSTART
} mml_i2c_cmd_t;

typedef enum mml_i2c_autost
{
	MML_I2C_AUTOSTART_ENABLE = MML_I2C_SET_AUTOSTART,
	MML_I2C_AUTOSTART_DISABLE
} mml_i2c_autost_t;

/** I2C events */
typedef enum mml_i2c_events
{
	MML_I2C_EVENT_LOST	= MML_I2C_IER_LOST_MASK,
	MML_I2C_EVENT_NOANS = MML_I2C_IER_NOANS_MASK,
	MML_I2C_EVENT_COMEN = MML_I2C_IER_COMEN_MASK,
	MML_I2C_EVENT_RDYRD = MML_I2C_IER_RDYRD_MASK,
	MML_I2C_EVENT_FFRX = MML_I2C_IER_FFRX_MASK,
	MML_I2C_EVENT_FFTX_ONE = MML_I2C_IER_FFTX_MASK,
	MML_I2C_EVENT_FFTX_HALFEMPTY = MML_I2C_IER_FFTXH_MASK,
} mml_i2c_events_t;

/** I2C Interrupt Status */
typedef enum
{
	MML_I2C_INTERRUPT_STATUS_LOST = MML_I2C_ISR_LOST_MASK,
	MML_I2C_INTERRUPT_STATUS_NOANS = MML_I2C_ISR_NOANS_MASK,
	MML_I2C_INTERRUPT_STATUS_COMEN = MML_I2C_ISR_COMEN_MASK,
	MML_I2C_INTERRUPT_STATUS_RDYRD = MML_I2C_ISR_RDYRD_MASK,
	MML_I2C_INTERRUPT_STATUS_FFRX = MML_I2C_ISR_FFRX_MASK,
	MML_I2C_INTERRUPT_STATUS_FFTX_ONE = MML_I2C_ISR_FFTX_MASK,
	MML_I2C_INTERRUPT_STATUS_FFTX_HALFEMPTY = MML_I2C_ISR_FFTXH_MASK
} mml_i2c_interrupt_status_t;

/** I2C IRQ handler function */
typedef void (*irq_handler_t)(void);

/** Structures ****************************************************************/
typedef struct
{
	/** Baud rate in bps */
	unsigned int 							baudrate;
	/** RX FIFO threshold ( 1 to 8)*/
	unsigned int 							rxfifo_thr;
	/** autostart, restart */
	unsigned int 							flags;
	/** IRQ handler function */
	irq_handler_t							irq_handler;

} mml_i2c_config_t;

typedef struct
{
	unsigned int							first_init;
	/** Interrupt request(IRQ) number */
	unsigned int 							irq;
	/** IRQ handler */
	irq_handler_t							irq_handler;

} mml_i2c_context_t;

/** Functions *****************************************************************/
/** Function to initialize and configure the I2C interface */
int mml_i2c_init(mml_i2c_config_t *config);
/** Function to configure the I2C interface which is already been initialized */
int mml_i2c_set_configuration(mml_i2c_config_t *config);
/**
 * Function initializes the I2C write operation and transmits specified number of data bytes to the
 * I2C slave device.
 */
int mml_i2c_write_start(unsigned short address, const unsigned char *pchar, unsigned int *plength);
/**
 * Function to transmit the data to the I2C device, this function should be called after the
 * communication has started.
 */
int mml_i2c_write(unsigned char *pchar,unsigned int *plength);
/** Function to initiate the I2C Start condition for read operation */
int mml_i2c_read_start(unsigned short address);
/** Function to receive the data from the I2C device */
int mml_i2c_read(unsigned char *pchar,unsigned int *plength);
/** I2C Input/output command control function */
int mml_i2c_ioctl(int command, void *data);
/** Function to stop the current read communication from the I2C device. */
int mml_i2c_stop(void);
/** Close the I2C Device and uninstall the I2C IRQ handler if installed. */
int mml_i2c_close(void);
/** Function to enable the desired I2C interrupt event/s */
int mml_i2c_enable_interrupt_events(unsigned int events);
/** Function to disable the desired I2C interrupt event/s */
int mml_i2c_disable_interrupt_events(unsigned int events);
/** Function to acknowledge/clear the desired I2C interrupt status */
int mml_i2c_clear_interrupt_status(unsigned int status);
/** Function to read the current interrupt status flags */
int mml_i2c_interrupt_status(unsigned int *status);
/** Get the I2C bus status */
int mml_i2c_bus_status(unsigned int *status);

#endif /* _MML_I2C_H_ */

/******************************************************************************/
/* EOF */
