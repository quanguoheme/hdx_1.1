/*
 * common.c --
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

/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
/** Local includes */
#include <cobra_defines.h>
#include <cobra_macros.h>
#include <cobra_functions.h>


/* Local declarations *********************************************************/
void cobra_memcpy_char(unsigned char *pdst, unsigned char *psrc, unsigned size);
void cobra_memcpy_short(unsigned short *pdst, unsigned short *psrc, unsigned size);
void cobra_memcpy_int(unsigned int *pdst, unsigned int *psrc, unsigned size);
#ifdef _WITH_128BITS_DATA_
void cobra_memcpy_128bits(unsigned int *pdst, unsigned int *psrc, unsigned size);
#endif /* _WITH_128BITS_DATA_ */
/******************************************************************************/
/* Public *********************************************************************/
/******************************************************************************/
int cobra_memcpy(void *pdst, void* psrc, unsigned int size)
{
	int										result = COMMON_ERR_UNKNOWN;

	/** Check input pointers */
	if ( !psrc || !pdst )
	{
		result = COMMON_ERR_NULL_PTR;
	}
	else
	{
		/** Check input size */
		switch( size % sizeof(unsigned int) )
		{
			case 0:
#ifdef _WITH_128BITS_DATA_
				if ( size % 16 )
				{
					cobra_memcpy_int(pdst, psrc, size);
				}
				else
				{
					cobra_memcpy_128bits(pdst, psrc, size);
				}
#else
				/** 4 Bytes */
				cobra_memcpy_int(pdst, psrc, size);
#endif /* _WITH_128BITS_DATA_ */
				/** No error */
				result = NO_ERROR;
				break;
			case 1:
			case 3:
				/** 1 Byte */
				cobra_memcpy_char(pdst, psrc, size);
				/** No error */
				result = NO_ERROR;
				break;
			case 2:
				/** 2 Bytes */
				cobra_memcpy_short(pdst, psrc, size);
				/** No error */
				result = NO_ERROR;
				break;
			default:
				break;
		}
	}
	/** We're done */
	return result;
}

/******************************************************************************/
/** Converts to unsigned int 32 */
unsigned int convert2uint32(unsigned char *data)
{
	unsigned int							x = 0;

	/**  */
	x = ( data[0] << 24 );
	x |= ( data[1] << 16 );
	x |= ( data[2] << 8 );
	x |= data[3];
	/** We're done */
	return x;
}

/******************************************************************************/
/** converts to unsigned short 16 */
unsigned short convert2uint16(unsigned char *data)
{
	unsigned short							x = 0;

	/**  */
	x |= ( data[0] << 8 );
	x |= data[1];
	/** We're done */
	return x;
}

/******************************************************************************/
void cobra_strcpy(char *dst, char *src, unsigned int length)
{
	register unsigned int		i;

	for( i = 0;i < length;i++ )
	{
		dst[i] = src[i];
	}
	return;
}

/******************************************************************************/
/* Private ********************************************************************/
/******************************************************************************/
void cobra_memcpy_char(unsigned char *pdst, unsigned char *psrc, unsigned int size)
{
	register unsigned int					i;

	/** Loop with char */
	for( i = 0;i < size;i++ )
	{
		pdst[i] = psrc[i];
	}
	/** We're done */
	return;
}

/******************************************************************************/
void cobra_memcpy_short(unsigned short *pdst, unsigned short *psrc, unsigned int size)
{
	register unsigned int					i;

	/** Loop with short */
	for( i = 0;i < ( size / 2 );i++ )
	{
		pdst[i] = psrc[i];
	}
	/** We're done */
	return;
}

/******************************************************************************/
void cobra_memcpy_int(unsigned int *pdst, unsigned int *psrc, unsigned int size)
{
	register unsigned int					i;

	/** Loop with integer */
	for( i = 0;i < ( size / 4 );i++ )
	{
		pdst[i] = psrc[i];
	}
	/** We're done */
	return;
}

#ifdef _WITH_128BITS_DATA_
/******************************************************************************/
void cobra_memcpy_128bits(unsigned int *pdst, unsigned int *psrc, unsigned size)
{
	register unsigned int					i;
	unsigned int							start;
	unsigned int							end;
	unsigned int							reg0;
	unsigned int							reg1;
	unsigned int							reg2;
	unsigned int							reg3;

	/** Destination boundaries */
	start = (unsigned int)pdst;
	end = start + size;
	/** Loop with 128bits number */
	for( i = 0;i < ( size / 4 );i += 4 )
	{
//FIXME
		/** It may have a smarter way to do that but that's just
		 * dummy code for now ... */
		reg0 = psrc[i];
		reg1 = psrc[i + 1];
		reg2 = psrc[i + 2];
		reg3 = psrc[i + 3];
		/** Copy in assembly code */
		__asm__ __volatile__
		(
			".arm \n"
			/* The following is needed for integrity */
			/* Initialize memory using INCR4 */
			"mov	a1, %0 \n"
			"mov	a2, %1 \n"
			"mov	r3, %2 \n"
			"mov	r4, %3 \n"
			"mov	r5, %4 \n"
			"mov	r6, %5 \n"
		"	1:	stmia a1!, { r3 - r6 } \n"
			"cmp	a1, a2 \n"
			"blt	1b \n"
			:: "r" (start), "r" (end), "r" (reg0), "r" (reg1),"r" (reg2), "r" (reg3)
		);
	}

}
#endif /* _WITH_128BITS_DATA_ */

/******************************************************************************/
/* EOF */
