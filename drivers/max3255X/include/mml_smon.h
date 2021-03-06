/*
 * mml_smon.h --
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
 * Created on: Jun 16, 2014
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_SMON_H_
#define _MML_SMON_H_

/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
/** Local includes */

/* Defines ********************************************************************/
/* Macros *********************************************************************/
#define	MML_SMON_BASE_ERR						COBRA_SMON_BASE_ERR
/* Enumerations ***************************************************************/

/* Structures *****************************************************************/

/* Variables ******************************************************************/

/* Functions ******************************************************************/
#ifdef _STAND_ALONE_DRIVER_SMON_
int main(void);
#endif /* _STAND_ALONE_DRIVER_SMON_ */
inline void mml_smon_setsecalm(unsigned int reg);
inline void mml_smon_writesecalm(unsigned int reg);
inline void mml_smon_readsecalm(unsigned int *reg);
inline void mml_smon_clearsecalm(unsigned int reg);
inline void mml_smon_readsecdiag(unsigned int *reg);

#endif /* _MML_SMON_H_ */

/******************************************************************************/
/* EOF */
