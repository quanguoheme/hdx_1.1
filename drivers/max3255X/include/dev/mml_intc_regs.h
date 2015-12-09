/*
 * mml_intc_regs.h --
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
 * Created on: Jun 29, 2012
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_INTC_REGS_H_
#define _MML_INTC_REGS_H_

/** Global includes */
/** Other includes */
#include <mml_nvic_regs.h>
/** Local includes */

/** @file mml_intc_regs.h INTC Registers Header */

/** @defgroup MML_INTC INTC Driver
 * @ingroup MML_DRIVER
 */

/** @defgroup MML_INTC_REGS INTC Registers
 *
 * @note INTC IP or Specification version number
 *
 * @ingroup MML_INTC
 * @{
 */

/*
 * INTC Registers
 */
#define MML_INTC_EN0_SET_OFST					NVIC_ISER0_OFST
#define MML_INTC_EN0_CLR_OFST					NVIC_ICER0_OFST
#define MML_INTC_EN1_SET_OFST					NVIC_ISER1_OFST
#define MML_INTC_EN1_CLR_OFST					NVIC_ICER1_OFST
#define MML_INTC_SW_INT0_SET_OFST				NVIC_ISPR0_OFST
#define MML_INTC_SW_INT0_CLR_OFST				NVIC_ICPR0_OFST
#define MML_INTC_SW_INT1_SET_OFST				NVIC_ISPR1_OFST
#define MML_INTC_SW_INT1_CLR_OFST				NVIC_ICPR1_OFST
#define MML_INTC_INT0_STAT_OFST					NVIC_ISPR0_OFST
#define MML_INTC_INT1_STAT_OFST					NVIC_ISPR0_OFST
#define MML_INTC_RAW0_INT_STAT_OFST				NVIC_ISPR0_OFST
#define MML_INTC_RAW1_INT_STAT_OFST				NVIC_ISPR0_OFST

/** @} */

#ifndef __ASSEMBLER__
/** INTC Registers.
 *
 */
typedef nvic_regs_t mml_intc_regs_t;

/** Interruption Priorities
 */
typedef nvic_prio_t mml_intc_prio_t;

#define MML_INTC_PRIO_OFST						4
#define MML_INTC_PRIO_MASK						0xf0
#define MML_INTC_SET_PRIO(priority) 			( priority << MML_INTC_PRIO_OFST )

#define MML_INTC_PRIO_0							MML_INTC_SET_PRIO(0)	//!< Priority 0 (Highest)
#define MML_INTC_PRIO_1							MML_INTC_SET_PRIO(1)	//!< Priority 1
#define MML_INTC_PRIO_2							MML_INTC_SET_PRIO(2)	//!< Priority 2
#define MML_INTC_PRIO_3							MML_INTC_SET_PRIO(3)	//!< Priority 3
#define MML_INTC_PRIO_4							MML_INTC_SET_PRIO(4)	//!< Priority 4
#define MML_INTC_PRIO_5							MML_INTC_SET_PRIO(5)	//!< Priority 5
#define MML_INTC_PRIO_6							MML_INTC_SET_PRIO(6)	//!< Priority 6
#define MML_INTC_PRIO_7							MML_INTC_SET_PRIO(7)	//!< Priority 7
#define MML_INTC_PRIO_8							MML_INTC_SET_PRIO(8)	//!< Priority 8
#define MML_INTC_PRIO_9							MML_INTC_SET_PRIO(9)	//!< Priority 9
#define MML_INTC_PRIO_10						MML_INTC_SET_PRIO(10)	//!< Priority 10
#define MML_INTC_PRIO_11						MML_INTC_SET_PRIO(11)	//!< Priority 11
#define MML_INTC_PRIO_12						MML_INTC_SET_PRIO(12)	//!< Priority 12
#define MML_INTC_PRIO_13						MML_INTC_SET_PRIO(13)	//!< Priority 13
#define MML_INTC_PRIO_14						MML_INTC_SET_PRIO(14)	//!< Priority 14
#define MML_INTC_PRIO_15						MML_INTC_SET_PRIO(15)	//!< Priority 15 (Lowest)

extern unsigned long __section_start_vectors;
#define mml_vectors								(&__section_start_vectors)

/** Maxim count of interrupts */
#define	K_MML_INTC_IRQ_MAX_COUNT				64


#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_INTC_REGS */

#endif /* _MML_INTC_REGS_H_ */

/******************************************************************************/
/* EOF */
