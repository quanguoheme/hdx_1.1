/*
 * cobra_defines.h --
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
 * Created on: Jul 30, 2012
 * Author: Yann G. (yann.gaude@maximintegrated.com)
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _COBRA_DEFINES_H_
#define _COBRA_DEFINES_H_

/** Global includes */
#include <config.h>
/** Other includes */
/** Local includes */

/******************************************************************************/
#ifndef TRUE
#define	TRUE								1
#endif /* TRUE */

#ifndef FALSE
#define FALSE								0
#endif /* FALSE */

#ifndef NULL
#define	NULL								0
#endif /* NULL */


#define	K_COBRA_ONE_KB						1024
#define	K_COBRA_ONE_MB						( K_COBRA_ONE_KB * K_COBRA_ONE_KB )

#define	K_COBRA_RESET_WAIT_LOOP_MAX			100000

/** Compilation optimization */
#ifndef __CC_ARM
#define	__COBRA_OPT_DEBUG__					__attribute__((optimize("-O0")))
#define	__COBRA_OPT_O1__					__attribute__((optimize("-O1")))
#define	__COBRA_OPT_O2__					__attribute__((optimize("-O2")))
#define	__COBRA_OPT_O3__					__attribute__((optimize("-O3")))
#define	__COBRA_OPT_SIZE__					__attribute__((optimize("-Os")))
#else
#define	__COBRA_OPT_DEBUG__
#define	__COBRA_OPT_O1__
#define	__COBRA_OPT_O2__
#define	__COBRA_OPT_O3__
#define	__COBRA_OPT_SIZE__
#endif 


#endif /* _COBRA_DEFINES_H_ */

/******************************************************************************/
/* EOF */
