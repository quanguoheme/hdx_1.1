/*
 * mml_spi.c --
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
/** @file mml_spi.c SPI core driver */

/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
#include <mml.h>
#include <mml_gcr.h>
#include <mml_gcr_regs.h>
#include <mml_gpio_regs.h>
#include <cobra_defines.h>
#include <cobra_macros.h>
#include <cobra_functions.h>
/** Local includes */
#include <mml_spi_regs.h>
#include <mml_spi.h>


__attribute__((section(".bss"))) mml_spi_context_t spi_context;

#ifdef _STAND_ALONE_DRIVER_SPI_
/* This compilation flag is activated only in driver development context
* without any application using it.
* DO NOT define it in application/test context then. */
/******************************************************************************/
int main(void)
{
	/** We're done */
	return NO_ERROR;
}
#endif /* _STAND_ALONE_DRIVER_SPI_ */

/******************************************************************************/
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
int mml_spi_init(mml_spi_dev_e devnum, mml_spi_params_t *pparams)
{
	int											result = COMMON_ERR_UNKNOWN;
	volatile mml_gpio_regs_t					*reg_gpio;

	/** Check input parameters */
	if( !pparams )
	{
		result = COMMON_ERR_NULL_PTR;
		goto mml_spi_init_out;
	}
	else if ( MML_SPI_DEV_MAX < devnum )
	{
		result = COMMON_ERR_INVAL;
		goto mml_spi_init_out;
	}
	else if ( ( ( pparams->baudrate > SPI_MAX_BAUD_RATE ) ||
			( pparams->baudrate < SPI_MIN_BAUD_RATE ) )
				&& ( pparams->mode == MML_SPI_MODE_MASTER ) )
	{
		result = COMMON_ERR_INVAL;
		goto mml_spi_init_out;
	}
	else if ( ( pparams->word_size > SPI_WORDSIZE_MAX ) ||
			( pparams->word_size < SPI_WORDSIZE_MIN ) )
	{
		result = COMMON_ERR_INVAL;
		goto mml_spi_init_out;
	}
	/** Set related GPIO */
	switch( devnum )
	{
		case MML_SPI_DEV0:
			reg_gpio = (volatile mml_gpio_regs_t*)MML_GPIO0_IOBASE;
			reg_gpio->en &= ~( MML_GPIOx_EN_BIT_MASK(16) |
								MML_GPIOx_EN_BIT_MASK(17) |
								MML_GPIOx_EN_BIT_MASK(18) |
								( ( pparams->ssel << 19 ) & 0xf ) );
			/** Primary function */
			reg_gpio->en1 &= ~( MML_GPIOx_EN1_BIT_MASK(16) |
								MML_GPIOx_EN1_BIT_MASK(17) |
								MML_GPIOx_EN1_BIT_MASK(18) |
								( ( pparams->ssel << 19 ) & 0xf )  );
			break;
		case MML_SPI_DEV1:
			reg_gpio = (volatile mml_gpio_regs_t*)MML_GPIO0_IOBASE;
			reg_gpio->en &= ~( MML_GPIOx_EN_BIT_MASK(25) |
								MML_GPIOx_EN_BIT_MASK(26) |
								MML_GPIOx_EN_BIT_MASK(27) |
								( ( pparams->ssel << 28 ) & 0xf )  );
			/** Primary function */
			reg_gpio->en1 &= ~( MML_GPIOx_EN1_BIT_MASK(25) |
								MML_GPIOx_EN1_BIT_MASK(26) |
								MML_GPIOx_EN1_BIT_MASK(27) |
								( ( pparams->ssel << 28 ) & 0xf )  );
			break;
		case MML_SPI_DEV2:
			reg_gpio = (volatile mml_gpio_regs_t*)MML_GPIO1_IOBASE;
			reg_gpio->en &= ~( MML_GPIOx_EN_BIT_MASK(13) |
								MML_GPIOx_EN_BIT_MASK(21) |
								MML_GPIOx_EN_BIT_MASK(29) |
								( ( pparams->ssel << 19 ) & 0x1 ) |
								( ( pparams->ssel << 20 ) & 0x1 ) |
								( ( pparams->ssel << 27 ) & 0x1 ) |
								( ( pparams->ssel << 28 ) & 0x1 ) );
			/** Primary function */
			reg_gpio->en1 &= ~( MML_GPIOx_EN1_BIT_MASK(13) |
								MML_GPIOx_EN1_BIT_MASK(21) |
								MML_GPIOx_EN1_BIT_MASK(29) |
								( ( pparams->ssel << 19 ) & 0x1 ) |
								( ( pparams->ssel << 20 ) & 0x1 ) |
								( ( pparams->ssel << 27 ) & 0x1 ) |
								( ( pparams->ssel << 28 ) & 0x1 ) );
			break;
		default:
			result = COMMON_ERR_OUT_OF_RANGE;
			goto mml_spi_init_out;
	}
	/** Put pad configuration in normal mode */
	reg_gpio->pad_cfg1 = 0;
	reg_gpio->pad_cfg2 = 0;
	/** Call of set configuration function */
	result = mml_spi_set_config(devnum, pparams);
	if ( NO_ERROR == result )
	{
		spi_context.port[devnum].initialized = TRUE;
	}
	/** We're done */
mml_spi_init_out:
	return result;
}

/******************************************************************************/
int mml_spi_shutdown(mml_spi_dev_e devnum)
{
	int											result = COMMON_ERR_UNKNOWN;
	volatile mml_gcr_regs_t						*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;


	/**  */
	if ( MML_SPI_DEV_COUNT > devnum )
	{
		spi_context.port[devnum].reg->cr &= ~MML_SPI_CR_SPIEN_MASK;
		result = NO_ERROR;
		/** Check and select the device */
		switch( devnum )
		{
			case MML_SPI_DEV0:
			    /** Disable SPI0 port */
				reg_gcr->perckcn |= ( 1 << MML_PERCKCN_DEV_SPI0 );
			    break;
			case MML_SPI_DEV1:
			    /** Disable SPI1 port */
				reg_gcr->perckcn |= ( 1 << MML_PERCKCN_DEV_SPI1 );
			    break;
			case MML_SPI_DEV2:
			    /** Disable SPI2 port */
				reg_gcr->perckcn |= ( 1 << MML_PERCKCN_DEV_SPI2 );
			    break;
			default:
			    result = COMMON_ERR_INVAL;
			    break;
		}
	}
	else
	{
		result = COMMON_ERR_OUT_OF_RANGE;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
int mml_spi_reset_interface(void)
{
	unsigned int								loop = K_COBRA_RESET_WAIT_LOOP_MAX;
	volatile mml_gcr_regs_t						*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;

	/** Launch all UARTs reset */
	reg_gcr->rstr |= MML_GCR_RSTR_SPI_MASK;
	/** Wait until it's done */
	while( ( reg_gcr->rstr & MML_GCR_RSTR_SPI_MASK ) && loop-- );
	/** We're done */
	return NO_ERROR;
}

/******************************************************************************/
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
int mml_spi_set_config(mml_spi_dev_e devnum, mml_spi_params_t *pparams)
{
	int											result = COMMON_ERR_UNKNOWN;
	unsigned int								tmp;
	unsigned int								i;
#ifdef _FPGA_WORKAROUND_
#else
	unsigned int								freq_apb = 0;
#endif /* _FPGA_WORKAROUND_ */
	volatile mml_gcr_regs_t						*reg_gcr = (mml_gcr_regs_t*)MML_GCR_IOBASE;

#ifdef _SPI_RESET_AT_INIT_
	/**  */
	if ( !spi_context.first_init )
	{
		/** Clock up all SPI ports ... */
		reg_gcr->perckcn &= ~( ( 1 << MML_PERCKCN_DEV_SPI0 ) |
								( 1 << MML_PERCKCN_DEV_SPI1 ) |
								( 1 << MML_PERCKCN_DEV_SPI2 ) );
		/** ... then reset ALL SPI interfaces in a row ... */
		reg_gcr->rstr |= MML_GCR_RSTR_SPI_MASK;
		while( reg_gcr->rstr & MML_GCR_RSTR_SPI_MASK );
		/** ... finish with clocking down all interfaces */
		reg_gcr->perckcn |= ( ( 1 << MML_PERCKCN_DEV_SPI0 ) |
								( 1 << MML_PERCKCN_DEV_SPI1 ) |
								( 1 << MML_PERCKCN_DEV_SPI2 ) );
		/** Only once ;) */
		spi_context.first_init = 1;
	}
#endif /* _SPI_RESET_AT_INIT_ */
	/** Select the device */
	switch( devnum )
	{
		case MML_SPI_DEV0:
			spi_context.port[devnum].reg = (volatile mml_spi_regs_t*)MML_SPI0_IOBASE;
			/** Enable SPI0 port */
			reg_gcr->perckcn &= ~( 1 << MML_PERCKCN_DEV_SPI0 );
			break;
		case MML_SPI_DEV1:
			spi_context.port[devnum].reg = (volatile mml_spi_regs_t*)MML_SPI1_IOBASE;
			/** Enable SPI1 port */
			reg_gcr->perckcn &= ~( 1 << MML_PERCKCN_DEV_SPI1 );
			break;
		case MML_SPI_DEV2:
			spi_context.port[devnum].reg = (volatile mml_spi_regs_t*)MML_SPI2_IOBASE;
			/** Enable SPI2 port */
			reg_gcr->perckcn &= ~( 1 << MML_PERCKCN_DEV_SPI2 );
			break;
		default:
			result = COMMON_ERR_OUT_OF_RANGE;
			goto mml_spi_set_config_out;
	}
	/** Retrieve APB frequency */
#ifdef _FPGA_WORKAROUND_
	spi_context.port[devnum].reg->brr = 0x2;
#else
	result = mml_get_apb_frequency(&freq_apb);
	if ( result )
	{
		goto mml_spi_set_config_out;
	}
	/** Baud Rate register ****************************************************/
	if ( MML_SPI_MODE_MASTER == pparams->mode )
	{
		spi_context.port[devnum].reg->brr = (unsigned int)( freq_apb / ( 2 * ( pparams->baudrate ) ) );
	}
	else
	{
		spi_context.port[devnum].reg->brr = MML_SPI_BRR_DFLT;
	}
#endif /* _FPGA_WORKAROUND_ */
	/** Mode register *********************************************************/
	tmp = spi_context.port[devnum].reg->mr;
	tmp &= ~MML_SPI_MR_NUMBITS_MASK;
	if ( SPI_WORDSIZE_MAX == pparams->word_size )
	{
		spi_context.port[devnum].ws = 0;
	}
	else
	{
		spi_context.port[devnum].ws = ( MML_SPI_MR_NUMBITS_MASK_NOOFST & pparams->word_size );
	}
	/** Compute word size mask for further process */
	spi_context.port[devnum].ws_mask = 0;
	for( i = 0;i < spi_context.port[devnum].ws;i++ )
	{
		spi_context.port[devnum].ws_mask |= (unsigned short)( 1 << i );
	}
	/** Re-introduce word size */
	tmp |= ( spi_context.port[devnum].ws << MML_SPI_MR_NUMBITS_OFST );
	/** Remove previous SSV related values */
	tmp &= ~( MML_SPI_MR_SSV_MASK | MML_SPI_MR_SSIO_MASK );
	/** Set SSV depending on SPI master mode */
	if ( MML_SPI_MODE_MASTER == pparams->mode )
	{
		/** Slave Select Value */
		if ( MML_SPI_SSV_HIGH == pparams->ssv )
		{
			tmp |= MML_SPI_MR_SSV_MASK;
		}
		else
		{
			/** No need to do that for now, but just in case */
			tmp &= ~MML_SPI_MR_SSV_MASK;
		}
		/** Slave Select I/O */
		if ( MML_SPI_SSIO_OUTPUT == pparams->ssio )
		{
			tmp |= MML_SPI_MR_SSIO_MASK;
		}
		else
		{
			/** No need to do that for now, but just in case */
			tmp &= ~MML_SPI_MR_SSIO_MASK;
		}
	}
	else if ( MML_SPI_SSIO_OUTPUT == pparams->ssio )
	{
		result = COMMON_ERR_INVAL;
		goto mml_spi_set_config_out;
	}
	else
	{
		tmp &= ~MML_SPI_MR_SSIO_MASK;
	}
	/** Remove previous Transmit Left Justify value */
	tmp &= ~MML_SPI_MR_TX_LJ_MASK;
	if ( MML_SPI_TLJ_SHIFT == pparams->tlj )
	{
		tmp |= MML_SPI_MR_TX_LJ_MASK;
	}
	/** Set new value */
	spi_context.port[devnum].reg->mr = tmp;
	/** FIFO RX Threshold setting *********************************************/
	tmp = spi_context.port[devnum].reg->dmar;
	if ( MML_SPI_DMA_ENABLE == pparams->dma_tx.active )
	{
		/** First erase related bits ... */
		tmp &= ~( MML_SPI_DMAR_TX_DMA_EN_MASK |
					MML_SPI_DMAR_TX_FIFO_CNT_MASK |
					MML_SPI_DMAR_TX_FIFO_LVL_MASK );
		/** ... add clear FIFO bit */
		tmp |= MML_SPI_DMAR_TX_FIFO_CLR_MASK;
		/** ... then set new ones */
		tmp |= ( ( pparams->dma_tx.lvl & MML_SPI_DMAR_TX_FIFO_LVL_MASK_NOOFST ) << MML_SPI_DMAR_TX_FIFO_LVL_OFST );
		tmp |= ( ( pparams->dma_tx.cnt & MML_SPI_DMAR_TX_FIFO_CNT_MASK_NOOFST ) << MML_SPI_DMAR_TX_FIFO_CNT_OFST );
		tmp |= MML_SPI_DMAR_TX_DMA_EN_MASK;
	}
	else
	{
		tmp &= ~MML_SPI_DMAR_TX_DMA_EN_MASK;
	}
	/**  */
	if ( MML_SPI_DMA_ENABLE == pparams->dma_rx.active )
	{
		/** First erase related bits ... */
		tmp &= ~( MML_SPI_DMAR_RX_DMA_EN_MASK |
					MML_SPI_DMAR_RX_FIFO_CNT_MASK |
					MML_SPI_DMAR_RX_FIFO_LVL_MASK );
		/** ... add clear FIFO bit */
		tmp |= MML_SPI_DMAR_RX_FIFO_CLR_MASK;
		/** ... then set new ones */
		tmp |= ( ( pparams->dma_rx.lvl & MML_SPI_DMAR_RX_FIFO_LVL_MASK_NOOFST ) << MML_SPI_DMAR_RX_FIFO_LVL_OFST );
		tmp |= ( ( pparams->dma_rx.cnt & MML_SPI_DMAR_RX_FIFO_CNT_MASK_NOOFST ) << MML_SPI_DMAR_RX_FIFO_CNT_OFST );
		tmp |= MML_SPI_DMAR_RX_DMA_EN_MASK;
	}
	else
	{
		tmp &= ~MML_SPI_DMAR_RX_DMA_EN_MASK;
	}
	/**  */
	spi_context.port[devnum].reg->dmar = tmp;
	/** Control register setting **********************************************/
	tmp = spi_context.port[devnum].reg->cr;
	if ( MML_SPI_MODE_MASTER == pparams->mode )
	{
		tmp |= MML_SPI_CR_MMEN_MASK;
	}
	else
	{
		tmp &= ~MML_SPI_CR_MMEN_MASK;
	}
	/** WOR */
	if ( MML_SPI_WOR_OPEN_DRAIN == pparams->wor )
	{
		tmp |= MML_SPI_CR_WOR_MASK;
	}
	else
	{
		tmp &= ~MML_SPI_CR_WOR_MASK;
	}
	/** SPI clock polarity */
	if ( MML_SPI_SCLK_HIGH == pparams->clk_pol )
	{
		tmp |= MML_SPI_CR_CLKPOL_MASK;
	}
	else
	{
		tmp &= ~MML_SPI_CR_CLKPOL_MASK;
	}
	/** SPI clock phase */
	if ( MML_SPI_PHASE_HIGH == pparams->phase )
	{
		tmp |= MML_SPI_CR_PHASE_MASK;
	}
	else
	{
		tmp &= ~MML_SPI_CR_PHASE_MASK;
	}
	/** BRG Timer interrupt */
	/** Remove value first ... */
	tmp &= ~MML_SPI_CR_BIRQ_MASK;
	if ( MML_SPI_BRG_IRQ_ENABLE == pparams->brg_irq )
	{
		tmp |= MML_SPI_CR_BIRQ_MASK;
	}
	/** Set value */
	spi_context.port[devnum].reg->cr = tmp;
	/** No error */
	result = NO_ERROR;
	/** We're done */
mml_spi_set_config_out:
	return result;
}

/******************************************************************************/
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
int mml_spi_transmit(mml_spi_dev_e devnum, unsigned char *data, unsigned int length)
{
	int											result = COMMON_ERR_UNKNOWN;
	volatile unsigned int						tx;
	unsigned char								wordsize;
	unsigned char								offset;
	unsigned int								size;
	volatile unsigned char						*ptr = (volatile unsigned char*)data;

	/** Check and select the device */
	if ( MML_SPI_DEV_COUNT <= devnum )
	{
	    result = COMMON_ERR_OUT_OF_RANGE;
	    goto mml_spi_transmit_out;
	}
	/**  */
	if ( FALSE == spi_context.port[devnum].initialized )
	{
		result = COMMON_ERR_NOT_INITIALIZED;
		goto mml_spi_transmit_out;
	}
	/** Retrieve word length */
	if ( !spi_context.port[devnum].ws )
	{
		wordsize = 16;
	}
	else
	{
		wordsize = spi_context.port[devnum].ws;
	}
	/** Pointer is aligned on beginning of data
	 * therefore 'offset' is null at first */
	offset = 0;
	size = length;
	/** Asserts nSS I/O */
	spi_context.port[devnum].reg->mr &= ~MML_SPI_MR_SSV_MASK;
	/** Loop until all data is transmitted */
	while( size )
	{
		/** Process depending on word size */
		if ( 8 >= wordsize )
		{
			/** Prepare data to be sent */
			tx = *ptr;
			tx |= ( *( ptr + 1 ) << 8 );
			tx = ( tx >> offset );
			tx &= spi_context.port[devnum].ws_mask;
			tx = ( tx << ( 16 - wordsize ) );
			/** Then removing data to be sent */
			*ptr &= ~( (unsigned char)spi_context.port[devnum].ws_mask << offset );
			if ( 8 < ( wordsize + offset ) )
			{
				*( ptr + 1 ) &= ~( (unsigned char)spi_context.port[devnum].ws_mask << ( 8 - offset ) );
			}
			/** Send data */
			spi_context.port[devnum].reg->dr = tx;
			/** Wait 'til it's done */
			while( spi_context.port[devnum].reg->sr & MML_SPI_SR_TXST_MASK );
			/** Retrieve data */
			tx = spi_context.port[devnum].reg->dr & spi_context.port[devnum].ws_mask;
			/** Fill buffer */
			*ptr |= ( (unsigned char)tx << offset );
			if ( 8 < ( wordsize + offset ) )
			{
				*( ptr + 1 ) |= ( (unsigned char)tx << ( 8 - offset ) );
			}
		}
		else
		{
			/** Prepare data to be sent */
			tx = *ptr;
			tx |= ( *( ptr + 1 ) << 8 );
			tx |= ( *( ptr + 2 ) << 16 );
			tx |= ( *( ptr + 3 ) << 24 );
			tx = ( tx >> offset );
			tx &= spi_context.port[devnum].ws_mask;
			tx = ( tx << ( 16 - wordsize ) );
			/** Then removing data to be sent */
			*ptr &= ~( (unsigned short)spi_context.port[devnum].ws_mask << offset );
			if ( 8 <= offset )
			{
				*( ptr + 1 ) &= ~( (unsigned short)spi_context.port[devnum].ws_mask << ( offset - 8 ) );
			}
			/**  */
			if ( 16 < ( wordsize + offset ) )
			{
				*( ptr + 2 ) &= ~( (unsigned short)spi_context.port[devnum].ws_mask >> ( 16 - offset ) );
				*( ptr + 3 ) &= ~( (unsigned short)spi_context.port[devnum].ws_mask >> ( 24 - offset ) );
			}
			/** Send data */
			spi_context.port[devnum].reg->dr = tx;
			/** Wait 'til it's done */
			while( spi_context.port[devnum].reg->sr & MML_SPI_SR_TXST_MASK );
			/** Retrieve data */
			tx = spi_context.port[devnum].reg->dr & spi_context.port[devnum].ws_mask;
			/** Fill buffer */
			*ptr |= ( (unsigned char)tx << offset );
			if ( 8 <= offset )
			{
				*( ptr + 1 ) |= ~( (unsigned short)tx << ( offset - 8 ) );
			}
			if ( 16 < ( wordsize + offset ) )
			{
				*( ptr + 2 ) &= ~( (unsigned int)tx >> ( 16 - offset ) );
				*( ptr + 3 ) &= ~( (unsigned int)tx >> ( 24 - offset ) );
			}
		}
		/** Increment parameters */
		offset += wordsize;
		while( 8 <= offset )
		{
			/** Increment pointer */
			ptr++;
			/** Decrement offset */
			offset -= 8;
			/** Decrement size */
			size--;
		}
	}
	/** Wait for data transmitting complete and then Deasserts nSS I/O */
	spi_context.port[devnum].reg->mr |= MML_SPI_MR_SSV_MASK;
	result = NO_ERROR;
	/** We're done */
mml_spi_transmit_out:
	return result;
}

/******************************************************************************/
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
int mml_spi_get_status(mml_spi_dev_e devnum, unsigned int *pstatus)
{
	int											result = COMMON_ERR_UNKNOWN;

	if ( !pstatus )
	{
		result = COMMON_ERR_NULL_PTR;
	}
	/** Check and select the device */
	else if ( MML_SPI_DEV_COUNT <= devnum )
	{
			result = COMMON_ERR_OUT_OF_RANGE;
	}
	/**  */
	else if ( FALSE == spi_context.port[devnum].initialized )
	{
		result = COMMON_ERR_NOT_INITIALIZED;
	}
	/**  */
	else
	{
		spi_context.port[devnum].reg->sr &= MML_SPI_SR_TXST_MASK;
		*pstatus = spi_context.port[devnum].reg->sr;
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}


/******************************************************************************/
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
int mml_spi_ioctl(mml_spi_dev_e devnum, mml_spi_config_e cmd, void *p_data)
{
	int											result = COMMON_ERR_UNKNOWN;

	/** Check and select the device */
	if ( MML_SPI_DEV_COUNT <= devnum )
	{
			result = COMMON_ERR_OUT_OF_RANGE;
	}
	/**  */
	else if ( FALSE == spi_context.port[devnum].initialized )
	{
		result = COMMON_ERR_NOT_INITIALIZED;
	}
	/** Treat command */
	else
	{
		/** No error ? */
		result = NO_ERROR;
		switch( cmd )
		{
			case MML_SPI_IOCTL_ENABLE:
				spi_context.port[devnum].reg->cr |= MML_SPI_CR_SPIEN_MASK;
				break;
			case MML_SPI_IOCTL_DISABLE:
				spi_context.port[devnum].reg->cr &= ~MML_SPI_CR_SPIEN_MASK;
				break;
			case MML_SPI_IOCTL_MODE_MASTER:
				spi_context.port[devnum].reg->cr |= MML_SPI_CR_MMEN_MASK;
				break;
			case MML_SPI_IOCTL_MODE_SLAVE:
				spi_context.port[devnum].reg->cr &= ~MML_SPI_CR_MMEN_MASK;
				break;
			case MML_SPI_IOCTL_OPEN_DRAIN_ENABLE:
				spi_context.port[devnum].reg->cr |= MML_SPI_CR_WOR_MASK;
				break;
			case MML_SPI_IOCTL_OPEN_DRAIN_DISABLE:
				spi_context.port[devnum].reg->cr &= ~MML_SPI_CR_WOR_MASK;
				break;
			case MML_SPI_IOCTL_RESET_TX_FIFO:
				while( spi_context.port[devnum].reg->sr & MML_SPI_SR_TXST_MASK );
				/** Set RESET TX FIFO bit */
				spi_context.port[devnum].reg->dmar |= MML_SPI_DMAR_TX_FIFO_CLR_MASK;
				break;
			case MML_SPI_IOCTL_RESET_RX_FIFO:
				while( spi_context.port[devnum].reg->dmar & MML_SPI_DMAR_RX_FIFO_CNT_MASK );
				/* Set RESET RX FIFO bit */
				spi_context.port[devnum].reg->dmar |= MML_SPI_DMAR_RX_FIFO_CLR_MASK;
				break;
			case MML_SPI_IOCTL_CLKPOL_HIGH:
				spi_context.port[devnum].reg->cr |= MML_SPI_CR_CLKPOL_MASK;
				break;
			case MML_SPI_IOCTL_CLKPOL_LOW:
				spi_context.port[devnum].reg->cr &= ~MML_SPI_CR_CLKPOL_MASK;
				break;
			case MML_SPI_IOTCL_PHASE_HIGH:
				spi_context.port[devnum].reg->cr |= MML_SPI_CR_PHASE_MASK;
				break;
			case MML_SPI_IOCTL_PHASE_LOW:
				spi_context.port[devnum].reg->cr &= ~MML_SPI_CR_PHASE_MASK;
				break;
			case MML_SPI_IOCTL_RESET_ERROR:
			{
				unsigned int					tmp;

				tmp = spi_context.port[devnum].reg->sr;
				tmp &= ~( MML_SPI_SR_IRQ_MASK |
								MML_SPI_SR_TOVR_MASK |
								MML_SPI_SR_COL_MASK |
								MML_SPI_SR_ABT_MASK |
								MML_SPI_SR_ROVR_MASK |
								MML_SPI_SR_TUND_MASK );
				spi_context.port[devnum].reg->sr = tmp;
				break;
			}
			case MML_SPI_IOCTL_SSV:
			{
				unsigned int					tmp;
				unsigned int					reg;

				if ( !p_data )
				{
					result = COMMON_ERR_NULL_PTR;
					break;
				}
				else
				{
					/**  */
					reg = ( spi_context.port[devnum].reg->mr & ~MML_SPI_MR_SSV_MASK );
					tmp = *((unsigned int*)p_data);
					reg |= ( ( MML_SPI_MR_SSV_MASK_NOOFST & tmp ) << MML_SPI_MR_SSV_OFST );
					spi_context.port[devnum].reg->mr = reg;
				}
				break;
			}
			case MML_SPI_IOCTL_SSIO:
			{
				unsigned int					tmp;
				unsigned int					reg;

				if ( !p_data )
				{
					result = COMMON_ERR_NULL_PTR;
					break;
				}
				else
				{
					/**  */
					reg = ( spi_context.port[devnum].reg->mr & ~MML_SPI_MR_SSIO_MASK );
					tmp = *((unsigned int*)p_data);
					reg |= ( ( MML_SPI_MR_SSIO_MASK_NOOFST & tmp ) << MML_SPI_MR_SSIO_OFST );
					spi_context.port[devnum].reg->mr = reg;
				}
				break;
			}
			case MML_SPI_IOCTL_SSLx:
			{
				unsigned int					tmp;
				unsigned int					reg;

				if ( !p_data )
				{
					result = COMMON_ERR_NULL_PTR;
					break;
				}
				else
				{
					/**  */
					reg = ( spi_context.port[devnum].reg->mr & ~MML_SPI_MR_SSLx_MASK );
					tmp = *((unsigned int*)p_data);
					reg |= ( ( MML_SPI_MR_SSV_MASK_NOOFST & tmp ) << MML_SPI_MR_SSV_OFST );
					spi_context.port[devnum].reg->mr = reg;
				}
				break;
			}
			default :
				result = COMMON_ERR_OUT_OF_RANGE;
				break;
		}
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/* EOF */
