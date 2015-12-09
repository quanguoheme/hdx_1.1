/*
 * mml_intc.c --
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
 * Created on: Jun 11, 2014
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
//#include <arch/cortex-m3/nvic.h>
#include <mml.h>
#include <mml_nvic_regs.h>
#include <cobra_macros.h>
/** Local includes */
#include <mml_intc.h>
#include <mml_intc_regs.h>

#ifdef __CC_ARM
extern void handler_default_undefined_(void);
#endif

#ifdef _STAND_ALONE_DRIVER_INTC_
/* This compilation flag is activated only in driver development context
* without any application using it.
* DO NOT define it in application/test context then. */
/*****************************************************************************/
int main(void)
{
	/** We're done */
	return NO_ERROR;
}
#endif /* _STAND_ALONE_DRIVER_INTC_ */

/******************************************************************************/
void mml_intc_attach_irq(unsigned int vector,
									mml_intc_prio_t priority,
									void(*handler)(void))
{
	volatile unsigned int						*vector_base;
    volatile nvic_regs_t						*reg_nvic = (volatile nvic_regs_t*)SCS_BASE;


    if ( reg_nvic->vtor )
    {
		/** Get VTOR */
		vector_base = (volatile unsigned int*)reg_nvic->vtor;
		/** Assign handler to vector */
		vector_base[vector + 16] = M_COBRA_MAKE_ODD(handler);
    }
    /** Assign priority */
    reg_nvic->priority[vector] = priority;
    /** We're done */
    return;
}

/******************************************************************************/
void mml_intc_detach_irq(unsigned int vector)
{
    volatile unsigned int						*vector_base;
    volatile nvic_regs_t						*reg_nvic = (volatile nvic_regs_t*)SCS_BASE;

    if ( reg_nvic->vtor )
    {
		/** Get VTOR */
		vector_base = (volatile unsigned int*)reg_nvic->vtor;
		/** Assign default 'undefined' function */
		vector_base[vector + 16] = M_COBRA_MAKE_ODD(handler_default_undefined_);
    }
    /** We're done */
    return;
}

/******************************************************************************/
void mml_intc_enable_irq(unsigned int vector)
{
    volatile nvic_regs_t						*reg_nvic = (volatile nvic_regs_t*)SCS_BASE;

    if ( K_MML_INTC_IRQ_MAX_COUNT > vector )
    {
	    reg_nvic->enable_set[vector >> 5] = 1 << ( vector & 0x1f );
    }
    /** We're done */
    return;
}

/******************************************************************************/
void mml_intc_disable_irq(unsigned int vector)
{
    volatile nvic_regs_t						*reg_nvic = (volatile nvic_regs_t*)SCS_BASE;

    if ( K_MML_INTC_IRQ_MAX_COUNT > vector )
    {
	    reg_nvic->enable_clr[vector >> 5] = ( 1 << ( vector & 0x1f ) );
    }
    /** We're done */
    return;
}

/******************************************************************************/
void mml_intc_ack_irq(unsigned int vector)
{
    volatile nvic_regs_t						*reg_nvic = (volatile nvic_regs_t*)SCS_BASE;

    if ( K_MML_INTC_IRQ_MAX_COUNT > vector )
    {
	    reg_nvic->pend_clr[vector >> 5] = ( 1 << ( vector & 0x1f ) );
    }
    /** We're done */
    return;
}

/******************************************************************************/
/**
 *
 * @param intc		Interrupt Controller
 * @param vector	Vector to configure
 * @param priority	Priority of the Interrupt
 * @param handler	Handler to attach
 * @return Error code
 */
int mml_intc_setup_irq(unsigned int vector,
						mml_intc_prio_t priority,
						void(*handler)(void))
{
	int											result = COMMON_ERR_UNKNOWN;


	if ( 64 < vector )
	{
		result = COMMON_ERR_INVAL;
	}
	else
	{
		mml_intc_attach_irq(vector, priority, handler);
		mml_intc_enable_irq(vector);
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
#ifndef __CC_ARM
unsigned int __attribute__((naked)) cpsid(void)
{
    unsigned int								result;

    //
    // Read PRIMASK and disable interrupts.
    //
    __asm__ __volatile__
    (
    	"mrs     r0, PRIMASK\n"
    	"cpsid   i\n"
    	"bx      lr\n"
          : "=r" (result)
    );

    //
    // The return is handled in the inline assembly, but the compiler will
    // still complain if there is not an explicit return here (despite the fact
    // that this does not result in any code being produced because of the
    // naked attribute).
    //
    return result;
}
#endif

/******************************************************************************/
#ifndef __CC_ARM
unsigned int __attribute__((naked)) cpsie(void)
{
    unsigned int								result;

    //
    // Read PRIMASK and enable interrupts.
    //
    __asm__ __volatile__
    (
    	"mrs     r0, PRIMASK\n"
    	"cpsie   i\n"
    	"bx      lr\n"
          : "=r" (result)
    );

    //
    // The return is handled in the inline assembly, but the compiler will
    // still complain if there is not an explicit return here (despite the fact
    // that this does not result in any code being produced because of the
    // naked attribute).
    //
    return result;
}
#endif



/******************************************************************************/
/* EOF */
