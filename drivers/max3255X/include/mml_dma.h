/*
 * mml_dma.h --
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
 * Created on: Oct 23, 2013
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_DMA_H_
#define _MML_DMA_H_

/** Global includes */
#include <errors.h>
/** Other includes */
/** Local includes */
#include <mml_dma_regs.h>


/** Macros Defines ********************************************************************/
/** COBRA adaptation */
#define	MML_DMA_BASE_ERR						COBRA_DMA_BASE_ERR

/** Enumerations ***************************************************************/
/** DMA errors list */
typedef enum
{
	MML_DMA_ERR_MIN = MML_DMA_BASE_ERR,
	MML_DMA_ERR_NOT_INITIALIZED, /** Error Code: DMA not initialized */
	MML_DMA_ERR_INVALID_DEVICE, /** Invalid DMA channel ID */
	MML_DMA_ERR_OUT_OF_RANGE, /** Invalid parameter or value */
	MML_DMA_ERR_INVALID, 	/** Error Code: Invalid operation */
	MML_DMA_ERR_RELOAD_NOT_CONFIGURED, /** DMA channel is not configured for Re-Load/Chain transfer */
	MML_DMA_ERR_IRQ_SET, /** IRQ setup error */
	MML_DMA_ERR_BUSY, /** DMA is busy */
	MML_DMA_ERR_INTERRUPT_PENDING, /** Interrupt is pending */
	MML_DMA_ERR_UNKNOWN, 	/** Error Code: Generic error for unknown behavior */
	MML_DMA_ERR_MAX = MML_DMA_ERR_UNKNOWN,

} mml_dma_errors_t;

/** DMA device id's */
typedef volatile enum
{
	/** Minimum value */
	MML_DMA_CH_MIN = 0,
	MML_DMA_CH0 = MML_DMA_CH_MIN,
	MML_DMA_CH1,
	MML_DMA_CH2,
	MML_DMA_CH3,
	MML_DMA_CH4,
	MML_DMA_CH5,
	MML_DMA_CH6,
	MML_DMA_CH7,
	/** Maximum value */
	MML_DMA_CH_MAX = MML_DMA_CH7,
	MML_DMA_CH_COUNT

} mml_dma_channel_t;

typedef enum
{
	MML_DMA_EXT_POL_ATIVE_LOW = 0,
	MML_DMA_EXT_POL_ATIVE_HIGH = 1

} mml_dma_ext_polarity_t;

/** DMA Priority */
typedef enum
{
	MML_DMA_PRIORITY_HIGHEST = 0x00,
	MML_DMA_PRIORITY_HIGHER,
	MML_DMA_PRIORITY_LOWER,
	MML_DMA_PRIORITY_LOWEST = 0x03,

} mml_dma_priority_t;

/** Data type for the DMA Request type selection */
typedef enum
{
	MML_DMA_REQ_MEM_TO_MEM = 0x00,
	MML_DMA_REQ_SPI0_TO_MEM = 0x01,
	MML_DMA_REQ_SPI1_TO_MEM = 0x02,
	MML_DMA_REQ_SPI2_TO_MEM = 0x03,
	MML_DMA_REQ_SPI3_TO_MEM = 0x04,
	MML_DMA_REQ_SPI4_TO_MEM = 0x05,
	MML_DMA_REQ_UART0_TO_MEM = 0x06,
	MML_DMA_REQ_UART1_TO_MEM = 0x07,
	MML_DMA_REQ_UART2_TO_MEM = 0x08,
	MML_DMA_REQ_SC0_TO_MEM = 0x09,
	MML_DMA_REQ_SC1_TO_MEM = 0x0A,
	MML_DMA_REQ_I2C_TO_MEM = 0x0B,
	MML_DMA_REQ_ADC_TO_MEM = 0x0C,
	MML_DMA_REQ_SHA_TO_MEM = 0x0D, /** NOTE: Source Address [TBD], See MML Design Specification(V.1.1.1) [Table 3-24] */
	/** 001110 � 100000 Reserved */
	MML_DMA_REQ_MEM_TO_SPI0 = 0x21,
	MML_DMA_REQ_MEM_TO_SPI1 = 0x22,
	MML_DMA_REQ_MEM2_TO_SPI2 = 0x23,
	MML_DMA_REQ_MEM_TO_SPI3_2TX = 0x24,
	MML_DMA_REQ_MEM_TO_SPI4 = 0x25,
	MML_DMA_REQ_MEM_TO_UART0 = 0x26,
	MML_DMA_REQ_MEM_TO_UART1 = 0x27,
	MML_DMA_REQ_MEM_TO_UART2= 0x28,
	MML_DMA_REQ_MEM_TO_SC0 = 0x29,
	MML_DMA_REQ_MEM_TO_SC1 = 0x2A,
	MML_DMA_REQ_MEM_TO_I2C = 0x2B,
	MML_DMA_REQ_MEM_TO_PRINTER = 0x2C,
	MML_DMA_REQ_MEM_TO_MONOLCD = 0x2D,
	MML_DMA_REQ_MEM_TO_SHA = 0x2E /** NOTE: Destination Address [TBD], See MML Design Specification(V.1.1.1) [Table 3-24] */
	/** 101101 - 111111 Reserved */

} mml_dma_request_type_t;

/** Time-Out selection: Selects the number of pre-scale clocks */
typedef enum
{
	MML_DMA_PRESCALE_CLOCKS_3TO4 = 0x00,
	MML_DMA_PRESCALE_CLOCKS_7TO8 = 0x01,
	MML_DMA_PRESCALE_CLOCKS_15TO16 = 0x02,
	MML_DMA_PRESCALE_CLOCKS_31TO32 = 0x03,
	MML_DMA_PRESCALE_CLOCKS_63TO64 = 0x04,
	MML_DMA_PRESCALE_CLOCKS_127TO128 = 0x05,
	MML_DMA_PRESCALE_CLOCKS_255TO256 = 0x06,
	MML_DMA_PRESCALE_CLOCKS_511TO512 = 0x07

} mml_dma_timeout_select_t;

/** Pre-Scale Select: Pre-Scale divider for timer clock input */
typedef enum
{
	MML_DMA_PRESCALE_DIVIDER_DISABLE = 0x00, /** Disable timer */
	MML_DMA_PRESCALE_DIVIDER_256 = 0x01, /** hclk / 256 */
	MML_DMA_PRESCALE_DIVIDER_64K = 0x02, /** hclk / 64k */
	MML_DMA_PRESCALE_DIVIDER_16M = 0x03 /** hclk / 16M */
	/** hclk / 64k */

} mml_dma_pre_scale_select_t;

/** Data structure for DMA data transfer width */
typedef enum
{
	MML_DMA_DATA_WIDTH_BYTE = 0x00,
	MML_DMA_DATA_WIDTH_HALF_WORD = 0x01,
	MML_DMA_DATA_WIDTH_WORD = 0x02

} mml_dma_transfer_width_t;

/** DMA Request Wait Enable */
typedef enum
{
	MML_DMA_START_TIMER_NORMAL = 0,
	MML_DMA_DELAY_TIMER_START = 1

} mml_dma_req_t;

/**  DMA initialization state FSM */
typedef enum
{
	MML_DMA_STATE_MIN = 0,
	MML_DMA_STATE_NOT_INITIALIZED = MML_DMA_STATE_MIN,
	MML_DMA_STATE_INITIALIZED,
	MML_DMA_STATE_CLOSED,
	MML_DMA_STATE_MAX = MML_DMA_STATE_CLOSED,
	MML_DMA_STATE_COUNT

} mml_dma_state_t;

/** DMA channel current status */
typedef enum
{
	MML_DMA_STATUS_MIN = 0,
	MML_DMA_STATUS_NOT_INITIALIZED = MML_DMA_STATUS_MIN,
	MML_DMA_STATUS_IDLE,
	MML_DMA_STATUS_BUSY,
	MML_DMA_STATUS_NORMAL_CONFIGURATION,
	MML_DMA_STATUS_RELOAD_CONFIGURATION,

} mml_dma_channel_status_t;

typedef enum
{
	MML_DMA_STATUS_DISABLED = 0,
	MML_DMA_STATUS_ENABLED

} mml_dma_status_t;

/** Structures *****************************************************************/
/**
 * DMA Configuration structure
 */
typedef struct
{
	mml_dma_priority_t 							priority;
	mml_dma_request_type_t						request;
	mml_dma_req_t								req_wait;
	mml_dma_timeout_select_t					timeout;
	mml_dma_pre_scale_select_t					prescale;
	mml_dma_transfer_width_t					src_width;
	int 										src_increment;
	mml_dma_transfer_width_t					dst_width;
	int											dst_increment;
	int											burst_size; /** 1 to 32 bytes */

} mml_dma_config_t;

/** DMA address configuration */
typedef struct
{
	unsigned int								src;
	unsigned int								dst;
	unsigned int								count;

} mml_dma_addr_t;

/** DMA callback function for reload configuration, invoked from the DMA interrupt context */
typedef void (*mml_dma_realod_callback_t)(mml_dma_addr_t *rld_addr, int *next_rld);

/** DMA channel structure */
typedef struct
{
	/** Interrupt number */
	unsigned int 								irq;
	/** DMA channel initialization state */
	mml_dma_state_t								state;
	/** DMA channel status */
	mml_dma_channel_status_t					status;
	/** DMA reload callback function */
	mml_dma_realod_callback_t					rld_callback;

} mml_dma_channel_struct;

/** DMA channel context information */
typedef struct
{
	unsigned int								first_init;
	/** DMA Channel */
	mml_dma_channel_struct						channel[MML_DMA_CH_COUNT];

} mml_dma_context_t;

/** Functions ******************************************************************/
/** The function is used to initialize the DMA channel */
int mml_dma_init(mml_dma_channel_t ch, int irq, mml_dma_config_t config, mml_dma_addr_t addr);
/** The function configures the DMA channel which is already been initialized/configured */
int mml_dma_set_configuration(mml_dma_channel_t ch, mml_dma_config_t config);
/** This function reads the DMA channel current configuration */
int mml_dma_get_configuration(mml_dma_channel_t ch, mml_dma_config_t *config);
/** The function sets the DMA channel address for new buffer transfers */
int mml_dma_set_address(mml_dma_channel_t ch, mml_dma_addr_t addr);
/** This function sets the DMA channel reload addresses for the chained buffer transfers */
int mml_dma_set_chaintransfer(mml_dma_channel_t ch, mml_dma_addr_t reload_addr, mml_dma_realod_callback_t rld_callback);
/** This function enables the DMA channel for data transfer */
int mml_dma_enable(mml_dma_channel_t ch);
/** This function disables the DMA channel */
int mml_dma_disable(mml_dma_channel_t ch);
/** Function returns the DMA channel current status */
int mml_dma_get_status(mml_dma_channel_t ch, mml_dma_status_t *status);
/** This function sets the DMA External Polarity for all the DMA channels */
int mml_dma_set_polarity(mml_dma_ext_polarity_t ext_pol);

#endif /* _MML_DMA_H_ */

/******************************************************************************/
/* EOF */
