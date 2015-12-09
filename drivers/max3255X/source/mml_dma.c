/*
 * mml_dma.c --
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

/** @file mml_dma.c DMA core driver */
/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
#include <mml.h>
#include <mml_gcr_regs.h>
#include <mml_intc.h>
#include <mml_intc_regs.h>
#include <cobra_defines.h>
#include <cobra_macros.h>
#include <cobra_functions.h>
/** Local includes */
#include <mml_dma_regs.h>
#include <mml_dma.h>

/** Variables *****************************************************************/
/** DMA channel context info */
__attribute__((section(".bss"))) mml_dma_context_t mml_dma_context;

/** DMA IRQ handler */
static void mml_dma_irq_handler(void);

#ifdef _STAND_ALONE_DRIVER_DMA_
/* This compilation flag is activated only in driver development context
* without any application using it.
* DO NOT define it in application/test context then. */
/******************************************************************************/
int main(void)
{
	/** We're done */
	return NO_ERROR;
}
#endif /* _STAND_ALONE_DRIVER_DMA_ */

/******************************************************************************/
/**
 * The function is used to initialize the DMA channel. It will initialize the DMA driver internal
 * data structures and setups the interrupt handler and enables the channel IRQ.
 * @param[in] ch							DMA channel number
 * @param[in] irq							DMA channel IRQ number.
 * @param[in] config						DMA configuration information.
 * @param[in] addr							DMA source and destination address for data transfer and byte count specifies
 * 											the number of bytes to transfer.
 * @retval NO_ERROR							No error
 * @retval COMMON_ERR_UNKNOWN				unknown error
 * @retval MML_DMA_ERR_INVALID_DEVICE		Invalid DMA channel number passed.
 * @retval COMMON_ERR_BAD_STATE				DMA channel already been initialized.
 * @retval MML_DMA_ERR_OUT_OF_RANGE		IRQ number is out of range
 * 											NOTE: User must pass the appropriate IRQ number to the respective DMA channel.
 * 											Meaning that the IRQ number 48 must be passed for the DMA Channel-0 initialization.
 */
int mml_dma_init(mml_dma_channel_t ch, int irq, mml_dma_config_t config, mml_dma_addr_t addr)
{
	int											result = COMMON_ERR_UNKNOWN;


	if ( !mml_dma_context.first_init )
	{
#if _DMA_RESET_AT_INIT_
		volatile mml_gcr_regs_t					*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;
#endif /* _DMA_RESET_AT_INIT_ */

		/** Memset like procedure */
		memset((unsigned char*)&mml_dma_context, 0x00, MML_DMA_CH_COUNT * sizeof(mml_dma_context_t));
#if _DMA_RESET_AT_INIT_
		/** Reset DMA interface */
		reg_gcr->rstr |= MML_GCR_RSTR_DMA_MASK;
		/** Wait until DMA reset completes */
		while( MML_GCR_RSTR_DMA_MASK & reg_gcr->rstr );
#endif /* _DMA_RESET_AT_INIT_ */
		/** To be done once only */
		mml_dma_context.first_init = 1;
	}
	/** Validate the input parameters */
	if ( MML_DMA_CH_COUNT <= ch )
	{
		result = MML_DMA_ERR_INVALID_DEVICE;
	} /** Check if the DMA channel is already been initialized */
	else if ( MML_DMA_STATE_INITIALIZED == mml_dma_context.channel[ch].state )
	{
		result = COMMON_ERR_BAD_STATE;
	}
	else if ( ( MML_INTNUM_DMA0 > irq ) || ( MML_INTNUM_DMA3 < irq ) )
	{
		result = MML_DMA_ERR_OUT_OF_RANGE;
	}
	else
	{
		volatile unsigned int 					dma_cfg = 0;
		volatile mml_dma_regs_t					*reg_dma = (volatile mml_dma_regs_t*)MML_DMA_IOBASE;


		/** Read-modify-write */
		dma_cfg = reg_dma->ch[ch].cfg;
		dma_cfg |= ( ( config.priority  << MML_DMA_CFG_PRI_OFST ) & MML_DMA_CFG_PRI_MASK );
		dma_cfg |= ( ( config.request << MML_DMA_CFG_REQSEL_OFST ) & MML_DMA_CFG_REQSEL_MASK );
		dma_cfg |= ( ( config.req_wait << MML_DMA_CFG_REQWAIT_OFST ) & MML_DMA_CFG_REQWAIT_MASK );
		dma_cfg |= ( ( config.timeout << MML_DMA_CFG_TOSEL_OFST ) & MML_DMA_CFG_TOSEL_MASK );
		dma_cfg |= ( ( config.prescale << MML_DMA_CFG_PSSEL_OFST ) & MML_DMA_CFG_PSSEL_MASK );
		dma_cfg |= ( ( config.src_width << MML_DMA_CFG_SRCWD_OFST ) & MML_DMA_CFG_SRCWD_MASK );
		dma_cfg |= ( ( config.src_increment << MML_DMA_CFG_SRCINC_OFST ) & MML_DMA_CFG_SRCINC_MASK );
		dma_cfg |= ( ( config.dst_width << MML_DMA_CFG_DSTWD_OFST ) & MML_DMA_CFG_DSTWD_MASK );
		dma_cfg |= ( ( config.dst_increment << MML_DMA_CFG_DSTINC_OFST ) & MML_DMA_CFG_DSTINC_MASK );
		dma_cfg |= ( ( config.burst_size << MML_DMA_CFG_BRST_OFST ) & MML_DMA_CFG_BRST_MASK );
		/** Write to CFG register */
		reg_dma->ch[ch].cfg |= dma_cfg;
		/** Configure the DMA source and destination registers */
		reg_dma->ch[ch].src = addr.src;
		reg_dma->ch[ch].dest = addr.dst;
		reg_dma->ch[ch].cnt = addr.count;
		mml_dma_set_polarity(MML_DMA_EXT_POL_ATIVE_LOW);
		/** Enable DMA channel interrupt */
		reg_dma->ctrl |= ( MML_DMA_CN_CHIEN_ENABLE << ch );
		/** Normal mode without buffer chaining (RLDEN = 0) */
		reg_dma->ch[ch].cfg |= MML_DMA_CFG_CHDIEN_MASK;
		/** Setup DMA channel IRQ */
		result = mml_intc_setup_irq(irq, MML_INTC_PRIO_3, mml_dma_irq_handler);
		if ( result )
		{
			result = MML_DMA_ERR_IRQ_SET;
		}
		else
		{
			mml_dma_context.channel[ch].irq = irq;
			mml_dma_context.channel[ch].status = MML_DMA_STATUS_NORMAL_CONFIGURATION;
			mml_dma_context.channel[ch].rld_callback = NULL;
			mml_dma_context.channel[ch].state = MML_DMA_STATE_INITIALIZED;
			result = NO_ERROR;
		}
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * The function configures the DMA channel which is already been initialized/configured.
 * This function can be used to re-configure an already initialized and/or configured DMA channel
 * with the new DMA channel configurations. If the DMA channel is busy then the function
 * returns error code.
 * @param[in] ch							DMA channel number
 * @param[in] config						Configuration structure which specifies
 * 											DMA configuration info.
 * @retval NO_ERROR							No error
 * @retval COMMON_ERR_UNKNOWN				unknown error
 * @retval MML_DMA_ERR_INVALID_DEVICE		Invalid DMA channel number passed.
 * @retval COMMON_ERR_BAD_STATE				DMA channel is not initialized.
 * @retval MML_DMA_ERR_BUSY				DMA is running.
 */
int mml_dma_set_configuration(mml_dma_channel_t ch, mml_dma_config_t config)
{
	int											result = COMMON_ERR_UNKNOWN;
	volatile mml_dma_regs_t						*reg_dma = (volatile mml_dma_regs_t*)MML_DMA_IOBASE;

	/** Validate the input parameters */
	if ( MML_DMA_CH_COUNT <= ch )
	{
		result = MML_DMA_ERR_INVALID_DEVICE;
	} /** Check if the DMA channel is initialized or not */
	else if ( MML_DMA_STATE_INITIALIZED != mml_dma_context.channel[ch].state )
	{
		result = COMMON_ERR_BAD_STATE;
	}
	/** Is DMA channel disabled ? */
	else if ( !( reg_dma->ch[ch].stat & MML_DMA_ST_CHST_MASK ) )
	{
		unsigned int 							dma_cfg = 0;

		reg_dma->ch[ch].cfg = 0;
		dma_cfg |= ( ( config.priority  << MML_DMA_CFG_PRI_OFST ) & MML_DMA_CFG_PRI_MASK );
		dma_cfg |= ( ( config.request << MML_DMA_CFG_REQSEL_OFST ) & MML_DMA_CFG_REQSEL_MASK );
		dma_cfg |= ( ( config.req_wait << MML_DMA_CFG_REQWAIT_OFST ) & MML_DMA_CFG_REQWAIT_MASK );
		dma_cfg |= ( ( config.timeout << MML_DMA_CFG_TOSEL_OFST ) & MML_DMA_CFG_TOSEL_MASK );
		dma_cfg |= ( ( config.prescale << MML_DMA_CFG_PSSEL_OFST ) & MML_DMA_CFG_PSSEL_MASK );
		dma_cfg |= ( ( config.src_width << MML_DMA_CFG_SRCWD_OFST ) & MML_DMA_CFG_SRCWD_MASK );
		dma_cfg |= ( ( config.src_increment << MML_DMA_CFG_SRCINC_OFST ) & MML_DMA_CFG_SRCINC_MASK );
		dma_cfg |= ( ( config.dst_width << MML_DMA_CFG_DSTWD_OFST ) & MML_DMA_CFG_DSTWD_MASK );
		dma_cfg |= ( ( config.dst_increment << MML_DMA_CFG_DSTINC_OFST ) & MML_DMA_CFG_DSTINC_MASK );
		dma_cfg |= ( ( config.burst_size << MML_DMA_CFG_BRST_OFST ) & MML_DMA_CFG_BRST_MASK );
		/**
		 * Write to CFG register
		 * Clear the Count-to-zero interrupt(CTZIEN)
		 * Enable only Channel Disable interrupt (CHDIEN) for normal operation recommended
		 */
		reg_dma->ch[ch].cfg |= ( dma_cfg & ~MML_DMA_CFG_CTZIEN_MASK ) | MML_DMA_CFG_CHDIEN_MASK;
		/** Set the DMA channel status */
		mml_dma_context.channel[ch].status = MML_DMA_STATUS_NORMAL_CONFIGURATION;
		result = NO_ERROR;
	}
	else
	{
		result = MML_DMA_ERR_BUSY;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * This function reads the DMA channel current configuration.
 * @param[in] ch							DMA channel number
 * @param[in] mml_dma_config_t *config		Pointer to the DMA channel configuration structure.
 * @retval NO_ERROR							No error
 * @retval COMMON_ERR_UNKNOWN				unknown error
 * @retval MML_DMA_ERR_INVALID_DEVICE		Invalid DMA channel number passed.
 */
int mml_dma_get_configuration(mml_dma_channel_t ch, mml_dma_config_t *config)
{
	int											result = COMMON_ERR_UNKNOWN;

	/** Validate the input parameters */
	if ( MML_DMA_CH_COUNT <= ch )
	{
		result = MML_DMA_ERR_INVALID_DEVICE;
	}
	else
	{
		unsigned int 							dma_cfg = 0;
		volatile mml_dma_regs_t					*reg_dma = (volatile mml_dma_regs_t*)MML_DMA_IOBASE;


		dma_cfg = reg_dma->ch[ch].cfg;
		config->priority =  (mml_dma_priority_t)(( dma_cfg & MML_DMA_CFG_PRI_MASK ) >> MML_DMA_CFG_PRI_OFST);
		config->request = (mml_dma_request_type_t)(( dma_cfg & MML_DMA_CFG_REQSEL_MASK ) >> MML_DMA_CFG_REQSEL_OFST);
		config->req_wait = (mml_dma_req_t)(( dma_cfg & MML_DMA_CFG_REQWAIT_MASK ) >> MML_DMA_CFG_REQWAIT_OFST);
		config->timeout = (mml_dma_timeout_select_t)(( dma_cfg & MML_DMA_CFG_TOSEL_MASK ) >> MML_DMA_CFG_TOSEL_OFST);
		config->prescale = (mml_dma_pre_scale_select_t)(( dma_cfg & MML_DMA_CFG_PSSEL_MASK ) >> MML_DMA_CFG_PSSEL_OFST);
		config->src_width = (mml_dma_transfer_width_t)(( dma_cfg & MML_DMA_CFG_SRCWD_MASK ) >> MML_DMA_CFG_SRCWD_OFST);
		config->src_increment = ( dma_cfg & MML_DMA_CFG_SRCINC_MASK ) >> MML_DMA_CFG_SRCINC_OFST;
		config->dst_width = (mml_dma_transfer_width_t)(( dma_cfg & MML_DMA_CFG_DSTWD_MASK ) >> MML_DMA_CFG_DSTWD_OFST);
		config->dst_increment = ( dma_cfg & MML_DMA_CFG_DSTINC_MASK ) >> MML_DMA_CFG_DSTINC_OFST;
		config->burst_size = ( dma_cfg & MML_DMA_CFG_BRST_MASK ) >> MML_DMA_CFG_BRST_OFST;
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * The function sets the DMA channel address for new buffer transfers.
 * If the DMA channel is busy then the function returns error.
 * @param[in] ch							DMA channel number
 * @param[in] config						Configuration structure which specifies DMA configuration info.
 * @retval NO_ERROR							No error
 * @retval COMMON_ERR_UNKNOWN				unknown error
 * @retval MML_DMA_ERR_INVALID_DEVICE		Invalid DMA channel number passed.
 * @retval COMMON_ERR_BAD_STATE				DMA channel is not initialized.
 * @retval MML_DMA_ERR_BUSY				DMA is running.
 */
int mml_dma_set_address(mml_dma_channel_t ch, mml_dma_addr_t addr)
{
	int											result = COMMON_ERR_UNKNOWN;
	volatile mml_dma_regs_t						*reg_dma = (volatile mml_dma_regs_t *)MML_DMA_IOBASE;

	/** Validate the input parameters */
	if ( MML_DMA_CH_COUNT <= ch )
	{
		result = MML_DMA_ERR_INVALID_DEVICE;
	} /** Check if the DMA channel is initialized or not */
	else if ( MML_DMA_STATE_INITIALIZED != mml_dma_context.channel[ch].state )
	{
		result = COMMON_ERR_BAD_STATE;
	}
	/** Is DMA channel disabled? */
	else if ( !( reg_dma->ch[ch].stat & MML_DMA_ST_CHST_MASK ) )
	{
		/** Configure the DMA source and destination registers */
		reg_dma->ch[ch].src = addr.src;
		reg_dma->ch[ch].dest = addr.dst;
		reg_dma->ch[ch].cnt = addr.count & MML_DMA_CNT_MASK;
		result = NO_ERROR;
	}
	else
	{
		result = MML_DMA_ERR_BUSY;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * This function sets the DMA channel reload addresses for the chained buffer transfers. Chained buffer
 * transfers will avoid the interrupt delays such as setting up of source and destination address information.
 * The DMA channel must have been initialized before calling this function for chained DMA transfers.
 *
 * @param[in] ch							DMA channel number
 * @param reload_addr						Reload DMA source and destination address and byte count
 * 											which specifies number of bytes to transfer.
 * @param rld_callback						callback function upon data transfer completion.
 * 											- NULL can be specified for rld_callback to transfer only one chained buffer.
 * 											- Multiple buffers can be specified in the return parameter 'next_rld' of
 * 											  the rld_callback() function itself. Parameters can be controlled by the
 * 											  application layer.
 * 											  next_rld : TRUE - buffer is chained
 * 											  next_rld: FALSE - No more chained buffers to transfer.
 * @retval NO_ERROR							No error
 * @retval COMMON_ERR_UNKNOWN				Unknown error
 * @retval MML_DMA_ERR_INVALID_DEVICE		Invalid DMA channel number passed.
 * @retval COMMON_ERR_BAD_STATE				DMA channel is not initialized.
 * @retval MML_DMA_ERR_BUSY				DMA is running.
 */
int mml_dma_set_chaintransfer(mml_dma_channel_t ch, mml_dma_addr_t reload_addr, mml_dma_realod_callback_t rld_callback)
{
	int											result = COMMON_ERR_UNKNOWN;
	volatile mml_dma_regs_t						*reg_dma = (volatile mml_dma_regs_t*)MML_DMA_IOBASE;

	/** Validate the input parameters */
	if ( MML_DMA_CH_COUNT <= ch )
	{
		result = MML_DMA_ERR_INVALID_DEVICE;
	} /** Check if the DMA channel is initialized or not */
	else if ( MML_DMA_STATE_INITIALIZED != mml_dma_context.channel[ch].state )
	{
		result = COMMON_ERR_BAD_STATE;
	}
	/** Is DMA channel disabled? */
	else if ( !( reg_dma->ch[ch].stat & MML_DMA_ST_CHST_MASK ) )
	{
		/** Configure the DMA source and destination registers */
		reg_dma->ch[ch].srld = reload_addr.src & ~MML_DMA_RLD_ADDR_MASK;
		reg_dma->ch[ch].drld = reload_addr.dst & ~MML_DMA_RLD_ADDR_MASK;
		reg_dma->ch[ch].crld = reload_addr.count & MML_DMA_CNT_MASK;
		mml_dma_context.channel[ch].rld_callback = rld_callback;
		/** Set the DMA channel status */
		mml_dma_context.channel[ch].status = MML_DMA_STATUS_RELOAD_CONFIGURATION;
		result = NO_ERROR;
	}
	else
	{
		result = MML_DMA_ERR_BUSY;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * This function enables the DMA channel for data transfer.
 * @param ch								DMA channel number
 * @retval NO_ERROR							No error
 * @retval COMMON_ERR_UNKNOWN				Unknown error
 * @retval MML_DMA_ERR_INVALID_DEVICE		Invalid DMA channel number passed.
 * @retval COMMON_ERR_BAD_STATE				DMA channel is not initialized.
 */
int mml_dma_enable(mml_dma_channel_t ch)
{
	int											result = COMMON_ERR_UNKNOWN;
	volatile mml_dma_regs_t						*reg_dma = (volatile mml_dma_regs_t*)MML_DMA_IOBASE;

	/** Validate the input parameters */
	if ( MML_DMA_CH_COUNT <= ch )
	{
		result = MML_DMA_ERR_INVALID_DEVICE;
	}
	else if ( MML_DMA_STATE_INITIALIZED != mml_dma_context.channel[ch].state )
	{
		result = COMMON_ERR_BAD_STATE;
	}
	else
	{
		/** Chained buffer transfer */
		if ( MML_DMA_STATUS_RELOAD_CONFIGURATION == mml_dma_context.channel[ch].status )
		{
			/** Enable CHDIEN and CTZIEN interrupts */
			reg_dma->ch[ch].cfg |= ( MML_DMA_CFG_CHDIEN_MASK | MML_DMA_CFG_CTZIEN_MASK );
			/** Must be the last step */
			reg_dma->ch[ch].cfg |= ( MML_DMA_CFG_CHEN_MASK | MML_DMA_CFG_RLDEN_MASK );
		}
		else
		{
			/** Normal DMA transfer */
			volatile unsigned int 				dma_cfg = 0;

			dma_cfg = reg_dma->ch[ch].cfg;
			/** Enable CHDIEN interrupt */
			reg_dma->ch[ch].cfg = ( dma_cfg & ~MML_DMA_CFG_CTZIEN_MASK ) | MML_DMA_CFG_CHDIEN_MASK;
			/** Must be the last step */
			reg_dma->ch[ch].cfg |= MML_DMA_CFG_CHEN_MASK;
		}
		/**  */
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * This function disables the DMA channel. If the interrupt is active/pending for the DMA channel,
 * the function returns appropriate error code and the application must wait till the pending
 * interrupts are processed/cleared.
 * @param ch								DMA channel number
 * @retval NO_ERROR							No error
 * @retval COMMON_ERR_UNKNOWN				Unknown error
 * @retval MML_DMA_ERR_INVALID_DEVICE		Invalid DMA channel number passed.
 * @retval COMMON_ERR_BAD_STATE				DMA channel is not initialized.
 * @retval MML_DMA_ERR_INTERRUPT_PENDING	DMA channel interrupt is pending
 */
int mml_dma_disable(mml_dma_channel_t ch)
{
	int											result = COMMON_ERR_UNKNOWN;
	volatile mml_dma_regs_t						*reg_dma = (volatile mml_dma_regs_t*)MML_DMA_IOBASE;

	/** Validate the input parameters */
	if ( MML_DMA_CH_COUNT <= ch )
	{
		result = MML_DMA_ERR_INVALID_DEVICE;
	}
	else if ( MML_DMA_STATE_INITIALIZED != mml_dma_context.channel[ch].state )
	{
		result = COMMON_ERR_BAD_STATE;
	}
	else if ( reg_dma->it_st & ( 0x01 << ch ) )
	{
		/** Interrupt is pending for the channel */
		result = MML_DMA_ERR_INTERRUPT_PENDING;
	}
	else
	{
		/** Chained buffer transfer */
		if ( MML_DMA_STATUS_RELOAD_CONFIGURATION == mml_dma_context.channel[ch].status )
		{
			/** Disable the CHDIEN and CTZIEN interrupts */
			reg_dma->ch[ch].cfg &= ~(MML_DMA_CFG_CHDIEN_MASK | MML_DMA_CFG_CTZIEN_MASK);
			/** Must be the last step */
			reg_dma->ch[ch].cfg &= ~(MML_DMA_CFG_CHEN_MASK | MML_DMA_CFG_RLDEN_MASK);
		}
		else
		{
			/** Normal DMA transfer */
			volatile unsigned int 				dma_cfg = 0;

			dma_cfg = reg_dma->ch[ch].cfg;
			/** Enable CHDIEN interrupt */
			reg_dma->ch[ch].cfg &= ~(MML_DMA_CFG_CTZIEN_MASK | MML_DMA_CFG_CHDIEN_MASK);
			/** Must be the last step */
			reg_dma->ch[ch].cfg &= ~MML_DMA_CFG_CHEN_MASK;
		}
		/**  */
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * Function returns the DMA channel current status
 * @param[in] ch							DMA channel number
 * @param[out] status						DMA status flag indicates whether the DMA channel
 * 											is enabled/disabled.
 * @retval NO_ERROR							No error
 * @retval COMMON_ERR_UNKNOWN				Unknown error
 * @retval MML_DMA_ERR_INVALID_DEVICE		Invalid DMA channel number passed.
 */
int mml_dma_get_status(mml_dma_channel_t ch, mml_dma_status_t *status)
{
	int											result = COMMON_ERR_UNKNOWN;
	volatile mml_dma_regs_t						*reg_dma = (volatile mml_dma_regs_t*)MML_DMA_IOBASE;

	if ( MML_DMA_CH_COUNT <= ch )
	{
		result = MML_DMA_ERR_INVALID_DEVICE;
	}
	else
	{
		if ( MML_DMA_ST_CHST_MASK & reg_dma->ch[ch].stat )
		{
			*status = MML_DMA_STATUS_ENABLED;
		}
		else
		{
			*status = MML_DMA_STATUS_DISABLED;
		}
		/**  */
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/**
 * This function sets the DMA External Polarity for all the DMA channels
 * i.e. polarity for ALL external DMA inputs and outputs (TxREQ, TxACK, RxREQ, RxACK).
 * @param ext_pol			external polarity.
 * @retval NO_ERROR			No error
 */
int mml_dma_set_polarity(mml_dma_ext_polarity_t ext_pol)
{
	volatile mml_dma_regs_t						*reg_dma = (volatile mml_dma_regs_t*)MML_DMA_IOBASE;

	if ( MML_DMA_EXT_POL_ATIVE_HIGH == ext_pol )
	{
		reg_dma->ctrl |= MML_DMA_CN_EXTPOL_AH;
	}
	else
	{
		reg_dma->ctrl &= MML_DMA_CN_EXTPOL_AL;
	}
	/** We're done */
	return NO_ERROR;
}

/******************************************************************************/
/**
 * IRQ handler for all the DMA channels
 */
static void mml_dma_irq_handler(void)
{
	volatile unsigned int 						ipend;
	volatile unsigned int						i = 0;
	volatile mml_dma_regs_t						*reg_dma = (volatile mml_dma_regs_t*)MML_DMA_IOBASE;


	ipend = reg_dma->it_st & 0xff;
	while( ipend )
	{
		/** Interrupt is pending for the channel */
		if ( 0x01 & ipend )
		{
			volatile unsigned int 				status;
			volatile unsigned int				irq_src = 0;

			status = reg_dma->ch[i].stat;
			/** Count-to-Zero interrupt has occurred */
			if ( MML_DMA_ST_CTZST_MASK & status )
			{
				/** Save the source level interrupt */
				irq_src |= MML_DMA_ST_CTZST_MASK;
			}
			if ( ( ( MML_DMA_ST_RLDST_MASK | MML_DMA_ST_CTZST_MASK ) & reg_dma->ch[i].stat ) &&
					( MML_DMA_STATUS_RELOAD_CONFIGURATION == mml_dma_context.channel[i].status ) )
			{
				if ( mml_dma_context.channel[i].rld_callback )
				{
					int							reload;
					mml_dma_addr_t 				rld_addr;


					mml_dma_context.channel[i].rld_callback(&rld_addr, &reload);
					if ( reload )
					{
						reg_dma->ch[i].srld = rld_addr.src & ~MML_DMA_RLD_ADDR_MASK;
						reg_dma->ch[i].drld = rld_addr.dst & ~MML_DMA_RLD_ADDR_MASK;
						reg_dma->ch[i].crld = rld_addr.count & MML_DMA_CNT_MASK;
						/** NOTE: write in two steps */
						reg_dma->ch[i].crld |= MML_DMA_RLD_ADDR_MASK;
					}
				}
				/** Save the source level interrupt */
				irq_src |= MML_DMA_ST_RLDST_MASK;
			}
			/**  */
			if ( MML_DMA_ST_BUSERR_MASK & reg_dma->ch[i].stat )
			{
				/** irq_src the Bus Error interrupt */
				irq_src |= MML_DMA_ST_BUSERR_MASK;
			}
			/**  */
			if ( MML_DMA_ST_TOST_MASK & reg_dma->ch[i].stat )
			{
				irq_src |= MML_DMA_ST_TOST_MASK;
			}
			/** Clear the source level interrupt */
			reg_dma->ch[i].stat |= irq_src & MML_DMA_ST_MASK;
		}
		/**  */
		ipend >>= 1;
		i++;
	}
	/** Acknowledge interrupt at platform level */
	mml_intc_ack_irq(mml_dma_context.channel[i].irq);
	/** We're done */
	return;
}

/******************************************************************************/
/* EOF */
