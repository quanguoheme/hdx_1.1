/*
 * mml_gcr.c --
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
 * Created on: Oct 09, 2013
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

/** @file mml_gcr.c GCR core driver */
/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
#include <mml.h>
/** Local includes */
#include <mml_gcr.h>
#include <mml_gcr_regs.h>


/* Local defines **************************************************************/
#define	K_COBRA_RESET_WAIT_LOOP_MAX				100000

#ifdef _STAND_ALONE_DRIVER_GCR_
/* This compilation flag is activated only in driver development context
* without any application using it.
* DO NOT define it in application/test context then. */
/*****************************************************************************/
int main(void)
{
    /** We're done */
    return NO_ERROR;
}
#endif /* _STAND_ALONE_DRIVER_GCR_ */

/*****************************************************************************/
/** Reset devices.
 *
 * @param gcr	Pointer to GCR based address
 * @param devs	OR of devices bits
 */
void mml_gcr_reset(unsigned int devs)
{
    unsigned int								loop = K_COBRA_RESET_WAIT_LOOP_MAX;
    volatile mml_gcr_regs_t						*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;

    /** Reset */
    reg_gcr->rstr = devs;
    if ( !( devs & MML_GCR_RSTR_SYSTEM_MASK ) )
    {
	/** Wait 'til resets end */
	while( ( reg_gcr->rstr & devs ) && loop-- );
    }
    /** We're done */
    return;
}

/*****************************************************************************/
/** Reset one or all devices.
 *
 * @param gcr	Pointer to GCR based address
 * @param dev	Device id
 *
 */
void mml_gcr_dev_reset(mml_rstr_dev_t dev)
{
    volatile mml_gcr_regs_t						*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;

    reg_gcr->rstr = ( 1 << dev );
    /** We're done */
    return;
}

/*****************************************************************************/
/** Set System Frequency.
 *
 * @param gcr	Pointer to GCR based address
 * @param freq	Frequency
 *
 */
void mml_gcr_set_sysfreq(mml_gcr_sysfreq_t freq)
{
    volatile mml_gcr_regs_t						*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;

    reg_gcr->clkcn = freq << MML_GCR_CLKCN_PSC_OFST;
    /** We're done */
    return;
}

/*****************************************************************************/
/** Set System Frequency.
 *
 * @param gcr	Pointer to GCR based address
 * @param freq	Pointer on frequency
 *
 */
void mml_gcr_get_sysfreq(mml_gcr_sysfreq_t *p_freq)
{
    volatile mml_gcr_regs_t						*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;

    *p_freq = (mml_gcr_sysfreq_t)(( reg_gcr->clkcn & MML_GCR_CLKCN_PSC_MASK ) >> MML_GCR_CLKCN_PSC_OFST);
    /** We're done */
    return;
}


/******************************************************************************/
/** Set system frequency is at wanted value */
int mml_set_system_divider(mml_gcr_sysfreq_t freq)
{
    int											result = COMMON_ERR_UNKNOWN;
    volatile mml_gcr_regs_t						*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;

    /** Check parameters */
    if ( MML_GCR_DIV_128 < freq )
    {
	    /**  */
	    result = COMMON_ERR_INVAL;
    }
    else
    {
	    /** Set PLL enable and DLL enable bits ... */
	    reg_gcr->pll0cn |= MML_GCR_PLL0CN_PLL0EN_MASK;
#ifndef _FPGA_WORKAROUND_
	    /** Wait PLLs are locked - lock bits shall be set to '1' when ready */
	    while( ( ( MML_GCR_PLL0CN_PLL0LOCK_MASK | MML_GCR_PLL0CN_PLL0EN_MASK ) & reg_gcr->pll0cn ) != ( MML_GCR_PLL0CN_PLL0LOCK_MASK | MML_GCR_PLL0CN_PLL0EN_MASK ) );
#endif /* _FPGA_WORKAROUND_ */
	    /** ... for PLL1CN too ! */
	    reg_gcr->pll1cn |= MML_GCR_PLL1CN_PLL1EN_MASK;
#ifndef _FPGA_WORKAROUND_
	    /** Wait PLLs are locked - lock bits shall be set to '1' when ready */
		while( ( ( MML_GCR_PLL1CN_PLL1LOCK_MASK | MML_GCR_PLL1CN_PLL1EN_MASK ) & reg_gcr->pll1cn ) != ( MML_GCR_PLL1CN_PLL1LOCK_MASK | MML_GCR_PLL1CN_PLL1EN_MASK ) );
#endif /* _FPGA_WORKAROUND_ */
	    /** ... then set prescaler and clock source selection */
	    reg_gcr->clkcn |= ( ( freq << MML_GCR_CLKCN_PSC_OFST ) | ( 0x1 << MML_GCR_CLKCN_CLKSEL_OFST ) );
#ifndef _FPGA_WORKAROUND_
	    /** Wait that it's done */
	    while( !( MML_GCR_CLKCN_CKRDY_MASK & reg_gcr->clkcn ) );
#endif /* _FPGA_WORKAROUND_ */
	    /** No error */
	    result = NO_ERROR;
    }
    /** We're done */
    return result;
}

/******************************************************************************/
/** Get system frequency */
int mml_get_system_frequency(unsigned int *p_freq)
{
    int											result = COMMON_ERR_UNKNOWN;

    /** Check input pointer */
    if ( !p_freq )
    {
	    result = COMMON_ERR_NULL_PTR;
    }
    else
    {
	    unsigned int							freq = 0;
	    volatile mml_gcr_regs_t					*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;


	    freq = ( reg_gcr->clkcn >> MML_GCR_CLKCN_PSC_OFST ) & MML_GCR_CLKCN_PSC_MASK_NOOFST;
	    if ( MML_GCR_DIV_128 >= freq )
	    {
		    *p_freq = Clock_Speed >> freq;
		    result = NO_ERROR;
	    }
	    else
	    {
		    result = COMMON_ERR_OUT_OF_RANGE;
	    }
    }
    /** We're done */
    return result;
}

/******************************************************************************/
/** Get AHB bus frequency */
int mml_get_ahb_frequency(unsigned int *p_freq)
{
#ifndef _FPGA_WORKAROUND_
    /** Retrieve AHB divider - same frequency as core one */
    return mml_get_system_frequency(p_freq);
#else
    if (Part_Number == 32550)
      *p_freq = Clock_Speed >> 1;
    else
      *p_freq = Clock_Speed;
    /** We're done */
    return NO_ERROR;
#endif /* _FPGA_WORKAROUND_ */
}

/******************************************************************************/
/** Get APB bus frequency */
int mml_get_apb_frequency(unsigned int *p_freq)
{
#ifndef _FPGA_WORKAROUND_
    int											result = COMMON_ERR_UNKNOWN;

    /** Retrieve APB divider - Half of AHB frequency */
    result = mml_get_ahb_frequency(p_freq);
    if ( NO_ERROR == result )
    {
		/** Apply divider */
		*p_freq >>= 1;
    }
    /** We're done */
    return result;
#else
    *p_freq = Clock_Speed >> 1;
    /** We're done */
    return NO_ERROR;
#endif /* _FPGA_WORKAROUND_ */
}

/******************************************************************************/
/** Reset entire system */
void mml_reset_system(void)
{
    /**  */
    mml_gcr_reset(MML_GCR_RSTR_SYSTEM_MASK);
    /** We're done */
    return;
}

/*****************************************************************************/
void mml_gcr_flip_iflash(void)
{
	volatile mml_gcr_regs_t						*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;

	/** Activate flip bit - Bottom half mapped to logical top half and vice versa */
	reg_gcr->scon |= MML_GCR_SCON_FLASH_PAGE_FLIP_MASK;
	/** We're done */
	return;
}

/*****************************************************************************/
void mml_gcr_unflip_iflash(void)
{
	volatile mml_gcr_regs_t						*reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;

	/** De-activate flip bit - Physical layout matches logical layout */
	reg_gcr->scon &= ~MML_GCR_SCON_FLASH_PAGE_FLIP_MASK;
	/** We're done */
	return;
}

/*****************************************************************************/
/* EOF */
