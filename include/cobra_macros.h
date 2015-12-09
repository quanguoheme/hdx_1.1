/*
 * cobra_macros.h --
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

#ifndef _COBRA_MACROS_H_
#define _COBRA_MACROS_H_

/** Global includes */
#include <config.h>
/** Other includes */
/** Local includes */
#include <cobra_defines.h>

/******************************************************************************/
#define	M_COBRA_MAX(_a_,_b_)				( ( _a_ > _b_ ) ? _a_ : _b_ )
#define	M_COBRA_MIN(_a_,_b_)				( ( _a_ > _b_ ) ? _b_ : _a_ )
#define M_COBRA_IS_DIGIT(_c_)				( (_c_ >= '0') && (_c_ <= '9') )
#define	M_COBRA_ABS(_x_)					( ( 0 > _x_ ) ? -_x_ : _x_ )

#define	M_COBRA_MAKE_ODD(_a_)				( (unsigned int)_a_ | 0x00000001 )
#define	M_COBRA_MAKE_EVEN(_a_)				( (unsigned int)_a_ & 0xfffffffe )

/** Write _b_ in register _a_ */
#define	M_COBRA_WRITE_REG(_register_,_b_)	IO_WRITE_U32(_register_,(unsigned int)_b_)
/** Return read value from _a_*/
#define	M_COBRA_READ_REG(_register_,_b_)	IO_READ_U32(_register_,(unsigned int)_b_)
/** Add value _b_ to already existing value of register _a_ */
#define	M_COBRA_ADD_REG(_register_,_b_)	(*((volatile unsigned int*)_register_)|=(unsigned int)_b_)
/** Remove value _b_ to already existing value of register _a_ */
#define	M_COBRA_REM_REG(_register_,_b_)	(*((volatile unsigned int*)_register_)&=(unsigned int)~_b_)
/** The platform is stuck in an endless loop */
#define M_COBRA_HARD_DIE()					while(1);

/* reorder the bytes of a 16-bit value from processor order to network order.
 * The macro name can be read "host to network short." */
#define M_COBRA_HTONS(_x_)					((((_x_) & 0xff00) >> 8) | (((_x_) & 0x00ff) << 8))

/* reorder the bytes of a 32-bit value from processor order to network order.
 * The macro name can be read "host to network long." */
#define M_COBRA_HTONL(_x_)					((((_x_) & 0xff000000) >> 24) |\
												(((_x_) & 0x00ff0000) >> 8) |\
												(((_x_) & 0x0000ff00) << 8) |\
												(((_x_) & 0x000000ff) << 24))

/* reorder the bytes of a 16-bit value from network order to processor order.
 * The macro name can be read "network to host short." */
#define M_COBRA_NTOHS(_x_)					M_COBRA_HTONS(_x_)

/* reorder the bytes of a 32-bit value from network order to processor order.
 * The macro name can be read "network to host long." */
#define M_COBRA_NTOHL(_x_)					M_COBRA_HTONL(_x_)

#define M_COBRA_GET_LONG(ptr) 				((unsigned long)\
											   ((((unsigned char*)(ptr))[0] << 0) | \
											   (((unsigned char*)(ptr))[1] << 8) | \
											   (((unsigned char*)(ptr))[2] << 16)  | \
											   (((unsigned char*)(ptr))[3] << 24)))

#define M_COBRA_GET_SHORT(ptr)	((unsigned short)\
                       ((((unsigned char*)(ptr))[0] << 0)  | \
                        (((unsigned char*)(ptr))[1] << 8)))

#define M_COBRA_GET_BYTE(ptr) (((unsigned char*)(ptr))[0])


#define M_COBRA_SET_LONG(ptr,val) ((((unsigned char*)(ptr))[0] = (unsigned char)(((unsigned long)(val)) >> 0) & 0xff), \
                (((unsigned char*)(ptr))[1] = (((unsigned long)(val)) >> 8) & 0xff), \
               (((unsigned char*)(ptr))[2] = (((unsigned long)(val)) >> 16) & 0xff), \
                (((unsigned char*)(ptr))[3] = (((unsigned long)(val)) >> 24) & 0xff))

#define M_COBRA_SET_SHORT(ptr,val) ((((unsigned char*)(ptr))[0] = (((unsigned short)(val)) >> 0) & 0xff), \
                 (((unsigned char*)(ptr))[1] = (((unsigned short)(val)) >> 8) & 0xff))

#define M_COBRA_SET_BYTE(ptr,val) (((unsigned char*)(ptr))[0]=(val))

#define M_COBRA_MAKE_SHORT(val) (((((unsigned short)(val)) >> 8) & 0xff) \
                         + ((((unsigned short)(val) & 0xff) ) << 8) )

#endif /* _COBRA_MACROS_H_ */

/******************************************************************************/
/* EOF */
