/*
 * mml_smon.c --
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

/** @file mml_smon.c SMON core driver */
/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
#include <mml_io.h>
#include <mml.h>
/** Local includes */
#include <mml_smon.h>
#include <mml_smon_regs.h>





#ifdef _STAND_ALONE_DRIVER_SMON_
/* This compilation flag is activated only in driver development context
* without any application using it.
* DO NOT define it in application/test context then. */
/******************************************************************************/
int main(void)
{
	/** We're done */
	return NO_ERROR;
}
#endif /* _STAND_ALONE_DRIVER_SMON_ */
/******************************************************************************/
/** Set value to SECALM register
 *
 * @param reg	Value to set in SECALM register
 *
 */
inline void mml_smon_setsecalm(unsigned int reg)
{
	volatile mml_smon_regs_t					*reg_smon = (volatile mml_smon_regs_t*)MML_SEC_MON_IOBASE;
/** Bug fix #0002033 - START */
	reg_smon->secalm |= reg;
/** Bug fix #0002033 - END */
	/** We're done */
	return;
}

/******************************************************************************/
/** Write value to SECALM register
 *
 * @param reg	Value of SECALM register to be written
 *
 */
inline void mml_smon_writesecalm(unsigned int reg)
{
	volatile mml_smon_regs_t					*reg_smon = (volatile mml_smon_regs_t*)MML_SEC_MON_IOBASE;
/** Bug fix #0002033 - START */
	reg_smon->secalm = reg;
/** Bug fix #0002033 - END */
	/** We're done */
	return;
}

/******************************************************************************/
/** Read value from SECALM register
 *
 * @param *reg	Pointer on read SECALM register value
 *
 */
inline void mml_smon_readsecalm(unsigned int *reg)
{
	volatile mml_smon_regs_t					*reg_smon = (volatile mml_smon_regs_t*)MML_SEC_MON_IOBASE;

	/** Check input pointer */
	if ( reg )
	{
		*reg = reg_smon->secalm;
	}
	/** We're done */
	return;
}

/******************************************************************************/
/** Set value to SECALM register
 *
 * @param reg	Value to remove from SECALM register
 *
 */
inline void mml_smon_clearsecalm(unsigned int reg)
{
	volatile mml_smon_regs_t					*reg_smon = (volatile mml_smon_regs_t*)MML_SEC_MON_IOBASE;
	unsigned int								tmp = reg_smon->secalm;

	tmp &= ~reg;
	reg_smon->secalm &= tmp;
	/** We're done */
	return;
}

/******************************************************************************/
/** Read value from SECDIAG register
 *
 * @param *reg	Pointer on read SECDIAG register value
 *
 */
inline void mml_smon_readsecdiag(unsigned int *reg)
{
	volatile mml_smon_regs_t					*reg_smon = (volatile mml_smon_regs_t*)MML_SEC_MON_IOBASE;

	/** Check input pointer */
	if ( reg )
	{
		*reg = reg_smon->secdiag;
	}
	/** We're done */
	return;
}

/******************************************************************************/
/* EOF */
