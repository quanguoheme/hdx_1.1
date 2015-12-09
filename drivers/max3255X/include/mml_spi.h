/*
 * mml_spi.h --
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
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_SPI_H_
#define _MML_SPI_H_

/** Global includes */
/** Other includes */
/** Local includes */
#include <mml_spi_regs.h>


/* Defines ********************************************************************/
#define SPI_MAX_BAUD_RATE					12000000
#define SPI_MIN_BAUD_RATE					732
#define	SPI_SYSTEM_FREQ_MAX					108000000
#define SPI_BLOC_FREQUENCY					54000000

#define SPI_WORDSIZE_MAX					16
#define SPI_WORDSIZE_MIN					1
/* Macros *********************************************************************/

/* Enumerations ***************************************************************/
typedef enum
{
	MML_SPI_DEV_MIN = 0,
	MML_SPI_DEV0 = MML_SPI_DEV_MIN,
	MML_SPI_DEV1,
	MML_SPI_DEV2,
	MML_SPI_DEV_MAX = MML_SPI_DEV2,
	MML_SPI_DEV_COUNT

} mml_spi_dev_e;

/** The control enumeration defines the type of control on the SPI interface
 */
typedef enum
{
	/** Enable the SPI module */
	MML_SPI_IOCTL_ENABLE,
	/** Disable the SPI module */
	MML_SPI_IOCTL_DISABLE,
	/** Master mode selected */
	MML_SPI_IOCTL_MODE_MASTER,
	/** Slave mode selected */
	MML_SPI_IOCTL_MODE_SLAVE,
	/** SPI pins not configured for open drain */
	MML_SPI_IOCTL_OPEN_DRAIN_ENABLE,
	/** SPI pins configured for open drain */
	MML_SPI_IOCTL_OPEN_DRAIN_DISABLE,
	/** Reset the transmit FIFO */
	MML_SPI_IOCTL_RESET_TX_FIFO,
	/** Reset the receive FIFO */
	MML_SPI_IOCTL_RESET_RX_FIFO,
	/** Clock idles low */
	MML_SPI_IOCTL_CLKPOL_HIGH,
	/** CLock idles high */
	MML_SPI_IOCTL_CLKPOL_LOW,
	/** Received/Transmit data on the rising/falling edge of the clock */
	MML_SPI_IOTCL_PHASE_HIGH,
	/** Received/transmitted data on the rising/falling edge of the clock */
	MML_SPI_IOCTL_PHASE_LOW,
	/** Reset the error */
	MML_SPI_IOCTL_RESET_ERROR,
	/** Slave Select Value */
	MML_SPI_IOCTL_SSV,
	/** Slave Select IO */
	MML_SPI_IOCTL_SSIO,
	/** Slave Select 1, 2 & 3 - Master mode only */
	MML_SPI_IOCTL_SSLx

} mml_spi_config_e;

/** SPI master/slave mode */
typedef enum
{
	/** Slave */
	MML_SPI_MODE_SLAVE = 0,
	/** Master */
	MML_SPI_MODE_MASTER

} mml_spi_master_mode_e;

/** SPI Wired OR (open drain) Enable */
typedef enum
{
	/** Not open drain */
	MML_SPI_WOR_NOT_OPEN_DRAIN = 0,
	/** Open drain */
	MML_SPI_WOR_OPEN_DRAIN

} mml_spi_wor_e;

/** SPI Clock polarity */
typedef enum
{
	/** Clock idle low */
	MML_SPI_SCLK_LOW = 0,
	/** Clock idle high */
	MML_SPI_SCLK_HIGH

} mml_spi_clk_polarity_e;

/** SPI Phase select */
typedef enum
{
	/** Phase low */
	MML_SPI_PHASE_LOW = 0,
	/** Phase high */
	MML_SPI_PHASE_HIGH

} mml_spi_phase_sel_e;

/** SPI BRG Timer Interrupt Request */
typedef enum
{
	/** Disable */
	MML_SPI_BRG_IRQ_DISABLE = 0,
	/** Enable */
	MML_SPI_BRG_IRQ_ENABLE

} mml_spi_brg_irq_e;

/** SPI Slave select value */
typedef enum
{
	/** Low */
	MML_SPI_SSV_LOW = 0,
	/** High */
	MML_SPI_SSV_HIGH

} mml_spi_ssv_e;

typedef enum
{
	/** Set as input */
	MML_SPI_SSIO_INPUT = 0,
	/** Set as output, Master only */
	MML_SPI_SSIO_OUTPUT

} mml_spi_ssio_e;

/** SPI Transmit left justify */
typedef enum
{
	/** Direct load */
	MML_SPI_TLJ_DIRECT = 0,
	/** Left justified */
	MML_SPI_TLJ_SHIFT

} mml_spi_tlj_e;

/** SPI enable DMA ? */
typedef enum
{
	/** Disable DMA */
	MML_SPI_DMA_DISABLE = 0,
	/** Enable DMA */
	MML_SPI_DMA_ENABLE

} mml_spi_dma_e;

/** SPI DMA TX fifo level */
typedef enum
{
	/** 1 free entry */
	MML_SPI_FIFO_LVL_1 = 0,
	/** 2 free entries */
	MML_SPI_FIFO_LVL_2,
	/** 3 free entries */
	MML_SPI_FIFO_LVL_3,
	/** 4 free entries */
	MML_SPI_FIFO_LVL_4

} mml_spi_dma_fifo_lvl_e;

/** SPI DMA TX fifo count */
typedef enum
{
	/** Empty */
	MML_SPI_FIFO_CNT_0 = 0,
	/** 1 free entry */
	MML_SPI_FIFO_CNT_1,
	/** 2 free entries */
	MML_SPI_FIFO_CNT_2,
	/** 3 free entries */
	MML_SPI_FIFO_CNT_3,
	/** 4 free entries */
	MML_SPI_FIFO_CNT_4

} mml_spi_dma_fifo_cnt_e;

/* Structures *****************************************************************/
/** SPI configuration information */
typedef struct
{
	/**  */
	mml_spi_dma_e								active;
	/**  */
	mml_spi_dma_fifo_lvl_e						lvl;
	/**  */
	mml_spi_dma_fifo_cnt_e						cnt;

} mml_spi_dma_conf_t;

/** SPI configuration information */
typedef struct
{
	/** Baud rate */
	unsigned int								baudrate;
	/** IO configuration:
	 * bit0 - SSELx0
	 * bit1 - SSELx1
	 * bit2 - SSELx2
	 * bit3 - SSELx3
	 * bit4 - bit31 should be null
	 * Set bit to '1' to set it to SSELx
	 * Set bit to '0' to leave it as it is */
	unsigned int								ssel;
	/** Word size */
	unsigned char								word_size;
	/** Select the mode of communication */
	mml_spi_master_mode_e						mode;
	/** SPI Wired OR (open drain) Enable */
	mml_spi_wor_e								wor;
	/** SPI Clock polarity */
	mml_spi_clk_polarity_e						clk_pol;
	/** SPI Phase select */
	mml_spi_phase_sel_e							phase;
	/** SPI BRG Timer Interrupt Request */
	mml_spi_brg_irq_e							brg_irq;
	/** SPI Slave select value */
	mml_spi_ssv_e								ssv;
	mml_spi_ssio_e								ssio;
	/** TLJ */
	mml_spi_tlj_e								tlj;
	/** DMA */
	mml_spi_dma_conf_t							dma_tx;
	mml_spi_dma_conf_t							dma_rx;

} mml_spi_params_t;

/* Private structures *********************************************************/
typedef struct
{
	/** Device initialized */
	unsigned char								initialized;
	/** Word size */
	unsigned char								ws;
	/** Word size mask */
	unsigned short								ws_mask;
	/** Base address */
	volatile mml_spi_regs_t						*reg;

} mml_spi_conf_t;


typedef struct
{
	/** First initialization */
	unsigned char								first_init;
	/** Word size mask */
	mml_spi_conf_t								port[MML_SPI_DEV_COUNT];

} mml_spi_context_t;

/* Variables ******************************************************************/

/* Functions ******************************************************************/
/**
 * The function resets, activates and configures the SPI interface
 *
 * @param[in] devnum			Select the SPI device
 * @param[in] pparams			A pointer on SPI parameters structure allocated and
 * 								filled by the caller.
 *
 * @retval NO_ERROR				No error
 * @retval COMMON_ERR_INVAL     No such device or value is not appropriate
 * @retval COMMON_ERR_NULL_PTR	Parameter passed is NULL pointer
 */
int mml_spi_init(mml_spi_dev_e devnum, mml_spi_params_t *pparams);
/**
 * This function is used to configure the communication parameters of SPI Interface
 *
 * @param[in] devnum					Select the SPI device
 * @param[in] pparams					A pointer on SPI configuration structure allocated
 * 										and filled by the caller.
 *
 * @retval NO_ERROR						No error
 * @retval COMMON_ERR_INVAL       		No such device or value is not appropriate
 * @retval COMMON_ERR_NULL_PTR			Parameter passed is NULL pointer
 * @retval COMMON_ERR_NOT_INITIALIZED	Module not initialized
 *
 */
int mml_spi_set_config(mml_spi_dev_e devnum, mml_spi_params_t *pparams);
/**
 * The function transmits/read data on the SPI interface
 *
 * @param[in] devnum					Select the SPI device
 * @param[in] *data						Pointer on the data buffer
 * @param[in] length					Number of bytes in the buffer
 *
 * @retval NO_ERROR						No error
 * @retval COMMON_ERR_INVAL       		No such device or value is not appropriate
 * @retval COMMON_ERR_NOT_INITIALIZED	Module not initialized
 */
int mml_spi_transmit(mml_spi_dev_e devnum, unsigned char *data, unsigned int length);
/**
 * The function is used to retrieve the status of SPI module
 *
 * @param[in]  devnum					 Select the SPI device.
 * @param[out] pstatus       			 A pointer on a variable allocated by the caller
 *                          			 and filled by the function with the state of SPI
 *                           			 module. It is a combination of the following
 *                           			 values :
 *                          			 \li #MML_SPI_STATE_TX_BUSY
 *                           			 \li #MML_SPI_STATE_TX_EMPTY
 *                           			 \li #MML_SPI_STATE_TX_FULL
 *                          			 \li #MML_SPI_STATE_RX_EMPTY
 *                           			 \li #MML_SPI_STATE_RX_FULL
 *
 * @retval NO_ERROR						No error
 * @retval COMMON_ERR_INVAL       		No such device or value is not appropriate
 * @retval COMMON_ERR_NULL_PTR			Parameter passed is NULL pointer
 * @retval COMMON_ERR_NOT_INITIALIZED	Module not initialized
 *
 */
int mml_spi_get_status(mml_spi_dev_e devnum, unsigned int *pstatus);
/**
 * The function performs a specific control on the SPI Interface.
 *
 * @param[in] devnum		Select the SPI device
 * @param[in] control       The control to be performed.
 *
 *
 * @retval NO_ERROR						No error
 * @retval COMMON_ERR_INVAL       		No such device or value is not appropriate
 * @retval COMMON_ERR_NOT_INITIALIZED	Module not initialized
 */
int mml_spi_ioctl(mml_spi_dev_e devnum, mml_spi_config_e cmd, void* p_data);
#define	M_MML_SPI_ENABLE(_devnum_)			mml_spi_ioctl(_devnum_, MML_SPI_IOCTL_ENABLE, NULL)
#define	M_MML_SPI_DISABLE(_devnum_)		mml_spi_ioctl(_devnum_, MML_SPI_IOCTL_DISABLE, NULL)

int mml_spi_shutdown(mml_spi_dev_e devnum);

#endif /* _MML_SPI_H_ */

/******************************************************************************/
/* EOF */
