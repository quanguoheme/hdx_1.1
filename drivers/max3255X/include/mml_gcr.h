/*
 * mml_gcr.h --
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
 * Created on: Jun 05, 2014
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */
#ifndef _MML_GCR_H_
#define _MML_GCR_H_

/** @file mml_gcr.h GCR Driver */
/** Global includes */
#include <io.h>
/** Other includes */
/** Local includes */
#include <mml_gcr_regs.h>

/** @defgroup COBRA_GCR GCR Driver
 *
 * @ingroup COBRA
 *
 * @{
 */

/* -------------------------------------------------------------------------- */
/** Reset devices.
 *
 * @param gcr	Pointer to GCR based address
 * @param devs	OR of devices bits
 */
void mml_gcr_reset(unsigned int devs);

/** Reset one or all devices.
 *
 * @param gcr	Pointer to GCR based address
 * @param dev	Device id
 *
 */
void mml_gcr_dev_reset(mml_rstr_dev_t dev);

/** Set POWER2 of one or all devices.
 *
 * @param gcr	Pointer to GCR based address
 * @param freq	Frequency
 *
 */
void mml_gcr_set_sysfreq(mml_gcr_sysfreq_t freq);

/** Get System Frequency.
 *
 * @param gcr	Pointer to GCR based address
 * @param freq	Pointer on frequency
 *
 */
void mml_gcr_get_sysfreq(mml_gcr_sysfreq_t *p_freq);

int mml_set_system_divider(mml_gcr_sysfreq_t freq);
int mml_get_system_frequency(unsigned int *p_freq);
int mml_get_ahb_frequency(unsigned int *p_freq);
int mml_get_apb_frequency(unsigned int *p_freq);
void mml_reset_system(void);
void mml_gcr_flip_iflash(void);
void mml_gcr_unflip_iflash(void);

/** @} *//* @defgroup COBRA_GCR */

#endif /* _MML_GCR_H_ */

/*****************************************************************************/
/* EOF */
