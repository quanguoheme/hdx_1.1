/*
 * mml_trng_regs.h --
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
 * Created on: Jun 17, 2014
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_TRNG_REGS_H_
#define _MML_TRNG_REGS_H_

/** @file mml_trng_regs.h TRNG Registers Header */

/** @defgroup MML_TRNG TRNG Driver
 * @ingroup MML_DRIVER
 */

/** @defgroup MML_TRNG_REGS TRNG Registers
 *
 * @note TRNG IP or Specification version number
 *
 * @ingroup MML_TRNG
 * @{
 */

/** @defgroup MML_TRNG_REGS_CN TRNG Control Register
 *
 * @li RNG_EN [0] - R/W: TRNG Enable
 * @li RNG_BS [1] - R/W: Bus Scrambling
 * @li RNG_IE [2] - R/W: Random Number Interrupt Enable
 * @li RNG_ISC [3] - W: Random Number Interrupt Status Clear
 * @li RNG_I4S [4] - R: Random Number 4 Words Status
 * @li RNG_IS [5] - R: Random Number Word Status
 * @li AESKG [6] - R/W: AES Key Generator
 *
 * @{
 */

#define MML_TRNGCN_OFST							0x0 //!< TRNGCN Register offset
#define MML_TRNGCN_DFLT							0x00000000 //!< TRNGCN Register default value

/* Bits Fields */

#define MML_TRNGCN_RNG_EN_OFST					0 //!< RNG Enable offset
#define MML_TRNGCN_RNG_EN_MASK_NOOFST			0x1
#define MML_TRNGCN_RNG_EN_MASK					( MML_TRNGCN_RNG_EN_MASK_NOOFST << MML_TRNGCN_RNG_EN_OFST )

#define MML_TRNGCN_RNG_BS_OFST					1 //!< RNG Bus Scrambling offset
#define MML_TRNGCN_RNG_BS_MASK_NOOFST			0x1
#define MML_TRNGCN_RNG_BS_MASK					( MML_TRNGCN_RNG_BS_MASK_NOOFST << MML_TRNGCN_RNG_BS_OFST )

#define MML_TRNGCN_RNG_IE_OFST					2 //!< RNG Interrupt Enable offset
#define MML_TRNGCN_RNG_IE_MASK_NOOFST			0x1
#define MML_TRNGCN_RNG_IE_MASK					( MML_TRNGCN_RNG_IE_MASK_NOOFST << MML_TRNGCN_RNG_IE_OFST )

#define MML_TRNGCN_RNG_ISC_OFST					3 //!< RNG Interrupt Status Clear offset
#define MML_TRNGCN_RNG_ISC_MASK_NOOFST			0x1
#define MML_TRNGCN_RNG_ISC_MASK					( MML_TRNGCN_RNG_ISC_MASK_NOOFST << MML_TRNGCN_RNG_ISC_OFST )

#define MML_TRNGCN_RNG_I4S_OFST					4 //!< RNG 4 Words Status offset
#define MML_TRNGCN_RNG_I4S_MASK_NOOFST			0x1
#define MML_TRNGCN_RNG_I4S_MASK					( MML_TRNGCN_RNG_I4S_MASK_NOOFST << MML_TRNGCN_RNG_I4S_OFST )

#define MML_TRNGCN_RNG_IS_OFST					5 //!< RNG Word Status offset
#define MML_TRNGCN_RNG_IS_MASK_NOOFST			0x1
#define MML_TRNGCN_RNG_IS_MASK					( MML_TRNGCN_RNG_IS_MASK_NOOFST << MML_TRNGCN_RNG_IS_OFST )

#define MML_TRNGCN_AESKG_OFST					6 //!< AES Key Generator offset
#define MML_TRNGCN_AESKG_MASK_NOOFST			0x1
#define MML_TRNGCN_AESKG_MASK					( MML_TRNGCN_AESKG_MASK_NOOFST << MML_TRNGCN_AESKG_OFST )
/** @} */ /* @defgroup MML_TRNG_REGS_TRNGCN */



#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */
/** TRNG Registers.
 *
 */
typedef volatile struct
{
	/** TRNG Control Register */
	volatile unsigned int						trngcr;
	/** TRNG Data Register */
	volatile unsigned int						trngdr;

} mml_trng_regs_t;

#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_TRNG_REGS */

#endif /* _MML_TRNG_REGS_H_ */

/******************************************************************************/
/* EOF */
