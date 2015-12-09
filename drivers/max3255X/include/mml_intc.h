/*
 * mml_intc.h --
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

#ifndef _MML_INTC_H_
#define _MML_INTC_H_

/** Global includes */
/** Other includes */
/** Local includes */
#include <mml_intc_regs.h>






/** @file mml_intc.h Cobra Interrupt Controller Driver */

/** @defgroup COBRA_INTC Cobra Interrupt Controller Driver
 *
 * @ingroup COBRA
 *
 * @{
 */

/* -------------------------------------------------------------------------- */
void mml_intc_attach_irq(unsigned int vector,
									mml_intc_prio_t priority,
									void(*handler)(void));

void mml_intc_detach_irq(unsigned int vector);

void mml_intc_enable_irq(unsigned int vector);

void mml_intc_disable_irq(unsigned int vector);

void mml_intc_ack_irq(unsigned int vector);


/** Configure and Enable an IRQ
 *
 * @param intc		Interrupt Controller
 * @param vector	Vector to configure
 * @param priority	Priority of the Interrupt
 * @param handler	Handler to attach
 * @return Error code
 */
int mml_intc_setup_irq(unsigned int vector,
						mml_intc_prio_t priority,
						void(*handler)(void));

#ifndef __CC_ARM
/** Master interrupt disable */
unsigned int __attribute__((naked)) cpsid(void);

/** Master interrupt enable */
unsigned int __attribute__((naked)) cpsie(void);
#else
/** Master interrupt disable */
#define cpsid() __disable_irq()

/** Master interrupt enable */
#define cpsie() __enable_irq()
#endif

/** @} */ /* @defgroup COBRA_INTC */

#endif /* _MML_INTC_H_ */

/******************************************************************************/
/* EOF */
