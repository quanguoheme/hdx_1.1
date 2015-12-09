/*
 * mml_sir_regs.h --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2014, Maxim Integrated Products
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
 * THIS SOFTWARE IS PROVIDED BY MAXIM INTEGRATED PRODUCTS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL MAXIM INTEGRATED PRODUCTS BE LIABLE FOR ANY
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
 * $Revision:: 4487     $:  Revision of last commit
 * $Author:: jeremy.kon#$:  Author of last commit
 * $Date:: 2015-01-14 1#$:  Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_SNVRF_REGS_H_
#define _MML_SNVRF_REGS_H_

/** @file mml_snvrf_regs.h */

/** @defgroup MML_SIR SIR Driver
 * @ingroup MML_DRIVER
 */

#ifndef __ASSEMBLER__
/** SIR Registers.
 *
 */
typedef volatile struct
{
	unsigned int				key0_tm; //!< test Register
	unsigned int				key1_tm; //!< test Register
	unsigned int				key2_tm; //!< test Register
	unsigned int				key3_tm; //!< test Register
	unsigned int				key4_tm; //!< test Register
	unsigned int				key5_tm; //!< test Register
	unsigned int				key6_tm; //!< test Register
	unsigned int				key7_tm; //!< test Register

} mml_snvrf_regs_t;


#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_SIR_REGS */

#endif /* _MML_SIR_REGS_H_ */

/******************************************************************************/
/* EOF */
