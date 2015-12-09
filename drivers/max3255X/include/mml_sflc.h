/*
 * mml_wdt.h --
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
 * Created on: Jan 30, 2014
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_SFLC_H_
#define _MML_SFLC_H_

/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
#include <mml.h>
#include <cobra_defines.h>
/** Local includes */


/* Defines ********************************************************************/
#define	K_MML_SFLC_BASE_ERROR					( COBRA_OTP << COBRA_ERR_PREFIX_OFFSET )

/** Internal Flash */
#define	K_MML_SFLC_QUANTUM_SIZE_IN_BYTES		sizeof(unsigned int)

#define	K_MML_SFLC_PGM_32BITS_OFST				2

#define	K_MML_SFLC_ERASE_PAGE_SIZE				0x200

#define	K_MML_SFLC_PGM_32BITS_SPECIAL_MASK		0xfffffffc
#define	K_MML_SFLC_ERASE_PAGE_SPECIAL_MASK		( 0xffffffff ^ ( K_MML_SFLC_ERASE_PAGE_SIZE - 1 ) )

/** OTP */
/** Size of OTP line in bytes */
#define	K_MML_SFLC_OTP_LINE_SIZE				0x08

#define	K_MML_SFLC_OTP_BASE						MML_MEM_OTP_BASE
#define	K_MML_SFLC_OTP_BASE_OFFSET				0
#define	K_MML_SFLC_OTP_MAXIM_BASE				MML_MEM_OTP_MXIM_BASE
#define	K_MML_SFLC_OTP_MAXIM_BASE_OFFSET		( MML_MEM_OTP_MXIM_BASE - MML_MEM_OTP_BASE )
#define	K_MML_SFLC_OTP_MAXIM_SIZE				MML_MEM_OTP_MXIM_SIZE /* 1 KiB */

#define	K_MML_SFLC_OTP_USER_BASE				MML_MEM_OTP_USER_BASE
#define	K_MML_SFLC_OTP_USER_BASE_OFFSET			( MML_MEM_OTP_USER_BASE - MML_MEM_OTP_BASE )
#define	K_MML_SFLC_OTP_USER_SIZE				MML_MEM_OTP_USER_SIZE /* 1 KiB */

/** OTP patterns */
#define	K_MML_SFLC_OTP_VIRGIN_PATTERN_BYTE		0xff
#define	K_MML_SFLC_OTP_VIRGIN_PATTERN_SHORT		0xffff
#define	K_MML_SFLC_OTP_VIRGIN_PATTERN_INT		0xffffffff
#define	K_MML_SFLC_OTP_VIRGIN_PATTERN			0xffffffffffffffff
#define	K_MML_SFLC_OTP_LOCK_PATTERN_BYTE		0x00
#define	K_MML_SFLC_OTP_LOCK_PATTERN_SHORT		0x0000
#define	K_MML_SFLC_OTP_LOCK_PATTERN_INT			0x00000000
#define	K_MML_SFLC_OTP_LOCK_PATTERN				0x0000000000000000


/* Macros *********************************************************************/

/* Enumerations ***************************************************************/
/** MML_SLFC errors list */

typedef enum
{
	N_MML_SFLC_ERR_MIN = K_MML_SFLC_BASE_ERROR,
	/** Error Code: BM module not initialized */
	N_MML_SFLC_ERR_NOT_INITIALIZED,
	/** Error Code: Data is not aligned according to memory specification */
	N_MML_SFLC_ERR_LENGTH_NOT_ALIGNED,
	/** Error Code: Address is not aligned according to memory specification */
	N_MML_SFLC_ERR_ADDRESS_NOT_ALIGNED,
	/** Error Code: Data packet is not aligned according to memory specification */
	N_MML_SFLC_ERR_PACKET_NOT_ALIGNED,
	/** Error Code: Memory area is not allowed */
	N_MML_SFLC_ERR_NOT_ALLOWED,
	/** Error Code: Memory is not accessible */
	N_MML_SFLC_ERR_NOT_ACCESSIBLE,
	/** Error Code: Data length is out of memory capacity */
	N_MML_SFLC_ERR_OVERFLOW,
	/** Error Code: 32bits programming failed */
	N_MML_SFLC_ERR_32BITS_PGM_FAILED,
	/** Error Code: Page erase failed */
	N_MML_SFLC_ERR_PAGE_ERASE_FAILED,
	/** Error Code: Mass erase failed */
	N_MML_SFLC_ERR_MASS_ERASE_FAILED,
	/** Error Code: Internal failure */
	N_MML_SFLC_ERR_FAILURE,
	/** Error Code: Generic error for unknown behavior */
	N_MML_SFLC_ERR_UNKNOWN,
	N_MML_SFLC_ERR_MAX = N_MML_SFLC_ERR_UNKNOWN

} e_mml_sflc_error;

/** Number of errors for this module */
#define	K_MML_SFLC_ERR_COUNT					( N_MML_SFLC_ERR_MAX - N_MML_SFLC_ERR_MIN )

/** OTP area identifier */
typedef enum
{
	N_MML_SFLC_OTP_ID_MIN = 0,
	N_MML_SFLC_OTP_ID_MAXIM = N_MML_SFLC_OTP_ID_MIN,
	N_MML_SFLC_OTP_ID_USER,
	N_MML_SFLC_OTP_ID_MAX = N_MML_SFLC_OTP_ID_USER,
	N_MML_SFLC_OTP_ID_COUNT,

} e_mml_sflc_otp_id;

/* Structures *****************************************************************/
/* Functions ******************************************************************/
#ifdef _STAND_ALONE_DRIVER_SFLC_
int main(void);
#endif /* _STAND_ALONE_DRIVER_SFLC_ */
void mml_sflc_init(void);
void mml_sflc_shutdown(void);
void mml_sflc_otp_unlock(void);
void mml_sflc_otp_lock(void);
int mml_write_32bits_ctrl(unsigned int address,
							unsigned int *p_src);
void mml_read_ctrl(unsigned int address, unsigned int *p_data);
int mml_sflc_page_erase_raw(unsigned int address,
								unsigned int length);
int mml_sflc_write_raw(unsigned int address,
						unsigned char *p_src,
						unsigned int length);
int mml_sflc_write(unsigned int address,
					unsigned char *p_src,
					unsigned int length);
int mml_sflc_read(unsigned int address,
					unsigned char *p_dst,
					unsigned int length);
int mml_sflc_erase(unsigned int address,
					unsigned int length);
int mml_sflc_mass_erase(void);
int mml_sflc_otp_write(e_mml_sflc_otp_id id,
						unsigned int address,
						unsigned char *p_src,
						unsigned int length);
int mml_sflc_otp_read(e_mml_sflc_otp_id id,
						unsigned int address,
						unsigned char *p_dst,
						unsigned int length);

#endif /* _MML_SFLC_H_ */
/******************************************************************************/
/* EOF */
