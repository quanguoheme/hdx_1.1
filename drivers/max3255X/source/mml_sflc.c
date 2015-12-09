/*
 * mml_sflc.c --
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
 * Created on: Jun 13, 2014
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

/** @file mml_sflc.c */

/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
#include <mml.h>
#include <cobra_defines.h>
#include <cobra_functions.h>
/** Local includes */
#include <mml_sflc.h>
#include <mml_sflc_regs.h>





unsigned char mml_sflc_buffer[K_MML_SFLC_ERASE_PAGE_SIZE];

#ifdef _STAND_ALONE_DRIVER_SFLC_
/* This compilation flag is activated only in driver development context
* without any application using it.
* DO NOT define it in application/test context then. */
/******************************************************************************/
int main(void)
{
	/** We're done */
	return NO_ERROR;
}
#endif /* _STAND_ALONE_DRIVER_SFLC_ */

/******************************************************************************/
void mml_sflc_init(void)
{
	/**  */
	memset(mml_sflc_buffer, K_MML_SFLC_OTP_VIRGIN_PATTERN_BYTE, sizeof(mml_sflc_buffer));
	/** We're done */
	return;
}

/******************************************************************************/
void mml_sflc_shutdown(void)
{
	/**  */
	memset(mml_sflc_buffer, K_MML_SFLC_OTP_VIRGIN_PATTERN_BYTE, sizeof(mml_sflc_buffer));
	/** We're done */
	return;
}

/******************************************************************************/
/* Generic ********************************************************************/
/******************************************************************************/
/** This function unlocks OTP for programming process */
void mml_sflc_otp_unlock(void)
{
	volatile mml_sflc_regs_t					*reg_sflc = (volatile mml_sflc_regs_t*)MML_SFLC_IOBASE;

	/** Wait for flash controller to release busy bit - set to '1' */
	while( reg_sflc->fcntl & ( MML_SFLC_CN_ALL_CMDS_MASK | MML_SFLC_CN_PEND_MASK ) );
	/** Write in OLOCK the 1st magic value */
	reg_sflc->acntl = (unsigned int)MML_SFLC_ACNTL_MAGIC_WORD1;
	/** Write in OLOCK the 2nd magic value */
	reg_sflc->acntl = (unsigned int)MML_SFLC_ACNTL_MAGIC_WORD2;
	/** Write in OLOCK the 3rd magic value */
	reg_sflc->acntl = (unsigned int)MML_SFLC_ACNTL_MAGIC_WORD3;
	/** Wait for flash controller to release busy bit - set to '1' */
	while( reg_sflc->fcntl & ( MML_SFLC_CN_ALL_CMDS_MASK | MML_SFLC_CN_PEND_MASK ) );
	/** We're done */
	return;
}

/******************************************************************************/
/** This function lock OTP to go back to read process */
void mml_sflc_otp_lock(void)
{
	volatile mml_sflc_regs_t					*reg_sflc = (volatile mml_sflc_regs_t*)MML_SFLC_IOBASE;

	/** Wait for flash controller to release busy bit - set to '1' */
	while( reg_sflc->fcntl & ( MML_SFLC_CN_ALL_CMDS_MASK | MML_SFLC_CN_PEND_MASK ) );
	/** Write any value but magic word */
	reg_sflc->acntl = 0;
	/** Wait for flash controller to release busy bit - set to '1' */
	while( reg_sflc->fcntl & ( MML_SFLC_CN_ALL_CMDS_MASK | MML_SFLC_CN_PEND_MASK ) );
	/** We're done */
	return;
}

/******************************************************************************/
int mml_write_32bits_ctrl(unsigned int address,
							unsigned int *p_src)
{
	int											result = COMMON_ERR_UNKNOWN;
	volatile mml_sflc_regs_t					*reg_sflc = (volatile mml_sflc_regs_t*)MML_SFLC_IOBASE;

	/** Clear out interrupt - if any */
	reg_sflc->fint = 0;
	/** Wait for end of process - if any */
	while( reg_sflc->fcntl & ( MML_SFLC_CN_ALL_CMDS_MASK | MML_SFLC_CN_PEND_MASK ) );
	/** Unlock flash and set 32bits mode */
	reg_sflc->fcntl = ( ( MML_SFLC_CN_UNLOCK_VALUE << MML_SFLC_CN_UNLOCK_OFST ) |
							MML_SFLC_CN_WIDTH_MASK );
	/** Set data */
	reg_sflc->fdata0 = p_src[0];
	/** Set address - It has to be integer aligned. "Byte" Address has to
	 * be converted into "Integer" address by being divided by 4 */
	reg_sflc->faddr = address;
	/** Set command - Unlock inside */
	reg_sflc->fcntl |= MML_SFLC_CN_WR_MASK;
	/** Wait for end of process */
	while( reg_sflc->fcntl & ( MML_SFLC_CN_PEND_MASK | MML_SFLC_CN_WR_MASK ) );
    /** Clear out interrupt */
    reg_sflc->fint = 0;
    /** Manually check flash programming via read back directly from memory. */
    if ( p_src[0] != *((unsigned int*)address) )
    {
		result = N_MML_SFLC_ERR_32BITS_PGM_FAILED;
    }
	else
	{
		result = NO_ERROR;
	}
    /** Lock flash and remove 32bits mode */
	reg_sflc->fcntl &= ~MML_SFLC_CN_UNLOCK_MASK;
	/** We're done */
	return result;
}


/******************************************************************************/
void mml_read_ctrl(unsigned int address, unsigned int *p_data)
{
    volatile unsigned int						*ptr = (volatile unsigned int*)address;

    /* This flash allows reads directly.  Just set the address and go. */
    *p_data = *((unsigned int*)ptr);
	/** We're done */
	return;
}

/******************************************************************************/
int mml_sflc_page_erase_raw(unsigned int address,
								unsigned int length)
{
	int											result = COMMON_ERR_UNKNOWN;
	unsigned int								i;
	unsigned int								base = address & K_MML_SFLC_ERASE_PAGE_SPECIAL_MASK;
	unsigned int								end =  ( address + length ) & K_MML_SFLC_ERASE_PAGE_SPECIAL_MASK;
	volatile unsigned int						*p_src = (volatile unsigned int*)( address & K_MML_SFLC_ERASE_PAGE_SPECIAL_MASK );
	volatile mml_sflc_regs_t					*reg_sflc = (volatile mml_sflc_regs_t*)MML_SFLC_IOBASE;

	/** Input parameters are supposed correct
	 * - granularity of data is flash quantum then ;)
	 * - address shall be erase page aligned */
	/** Clear out interrupt - if any */
	reg_sflc->fint = 0;
	/** Wait for end of process - if any */
	while( reg_sflc->fcntl & ( MML_SFLC_CN_ALL_CMDS_MASK | MML_SFLC_CN_PEND_MASK ) );
	/** Unlock flash and set erase mode */
	reg_sflc->fcntl = ( ( MML_SFLC_CN_UNLOCK_VALUE << MML_SFLC_CN_UNLOCK_OFST ) |
						( MML_SFLC_CN_ERASE_CODE_PAGE << MML_SFLC_CN_ERASE_CODE_OFST ) );
	while( base < end )
	{
		/** Set address */
		reg_sflc->faddr = base;
		/** Set command - Unlock inside */
		reg_sflc->fcntl |= MML_SFLC_CN_PGE_MASK;
		/** Wait for end of process */
		while( reg_sflc->fcntl & ( MML_SFLC_CN_PEND_MASK | MML_SFLC_CN_PGE_MASK ) );
		/**  */
		base += K_MML_SFLC_ERASE_PAGE_SIZE;
	}
	/** Lock flash and remove erase mode */
	reg_sflc->fcntl &= ~( MML_SFLC_CN_UNLOCK_MASK | MML_SFLC_CN_ERASE_CODE_MASK );
	/** Check if it has been erased */
	for( i = 0;i < ( K_MML_SFLC_ERASE_PAGE_SIZE / K_MML_SFLC_QUANTUM_SIZE_IN_BYTES );i++  )
	{
		if ( K_MML_SFLC_OTP_VIRGIN_PATTERN_INT != p_src[i] )
		{
			result = N_MML_SFLC_ERR_PAGE_ERASE_FAILED;
			goto mml_sflc_page_erase_raw_out;
		}
	}
	/** No error */
	result = NO_ERROR;
	/** We're done */
mml_sflc_page_erase_raw_out:
	return result;
}

/******************************************************************************/
int mml_sflc_mass_erase(void)
{
	int											result = COMMON_ERR_UNKNOWN;
#ifdef _WITH_FORCED_EXIT_
	int											forced_loop = 10000;
#endif /*_WITH_FORCED_EXIT_ */
	volatile mml_sflc_regs_t					*reg_sflc = (volatile mml_sflc_regs_t*)MML_SFLC_IOBASE;

	/** Clear out interrupt - if any */
	reg_sflc->fint = 0;
	/** Wait for end of process - if any */
	while( reg_sflc->fcntl & ( MML_SFLC_CN_ALL_CMDS_MASK | MML_SFLC_CN_PEND_MASK ) );
	/** Unlock flash and set erase mode */
	reg_sflc->fcntl = ( ( MML_SFLC_CN_UNLOCK_VALUE << MML_SFLC_CN_UNLOCK_OFST ) |
							( MML_SFLC_CN_ERASE_CODE_MASS << MML_SFLC_CN_ERASE_CODE_OFST ) );
	/** Set address */
	reg_sflc->faddr = MML_MEM_FLASH_BASE;
	/** Set command - Unlock inside */
	reg_sflc->fcntl |= MML_SFLC_CN_ME_MASK;
	/** Wait for end of process */
#ifdef _WITH_FORCED_EXIT_
	while( ( reg_sflc->fcntl & ( MML_SFLC_CN_PEND_MASK | MML_SFLC_CN_ME_MASK ) ) && forced_loop-- );
#else
	while( reg_sflc->fcntl & ( MML_SFLC_CN_PEND_MASK | MML_SFLC_CN_ME_MASK ) );
#endif /* _WITH_FORCED_EXIT_ */
	/** Lock flash and remove erase mode */
	reg_sflc->fcntl &= ~( MML_SFLC_CN_UNLOCK_MASK | MML_SFLC_CN_ERASE_CODE_MASK );
	/** No error */
	result = NO_ERROR;
	/** We're done */
	return result;
}


/******************************************************************************/
int mml_sflc_write_raw(unsigned int address,
						unsigned char *p_src,
						unsigned int length)
{
	int											result = COMMON_ERR_UNKNOWN;
	unsigned int								relative = address;
	unsigned int								i;
	unsigned int								packet = 0;
	unsigned int								last_size = length;
	unsigned int								*p_isrc = (unsigned int*)p_src;


	/** Input parameters are supposed correct - granularity
	 * of data is flash quantum then ;) */
	/** Length should be quantum aligned */
	if ( last_size % K_MML_SFLC_QUANTUM_SIZE_IN_BYTES )
	{
		/**  */
		result = N_MML_SFLC_ERR_LENGTH_NOT_ALIGNED;
		goto mml_sflc_write_raw_error;
	}
	/** Process start of packet */
	/** Check if data lasts */
	packet = last_size / K_MML_SFLC_QUANTUM_SIZE_IN_BYTES;
	/** Programming lasting data if any */
	for( i = 0;i < packet;i++ )
	{
		result = mml_write_32bits_ctrl(relative, p_isrc);
	    if ( result )
	    {
	    	goto mml_sflc_write_raw_error;
	    }
		else
		{
			relative += K_MML_SFLC_QUANTUM_SIZE_IN_BYTES;
			p_isrc++;
		}
	}
	/** No error 'til here */
	result = NO_ERROR;
	/** We're done */
mml_sflc_write_raw_error:
	return result;
}


/******************************************************************************/
/* Internal Flash *************************************************************/
/******************************************************************************/
int mml_sflc_write(unsigned int address,
					unsigned char *p_src,
					unsigned int length)
{
	int											result = COMMON_ERR_UNKNOWN;
	unsigned int								last_size = length;
	unsigned char								*p_tmp = p_src;
	unsigned int								ofst_end = 0;
	unsigned int								size_end = 0;
	unsigned char								p_work[sizeof(unsigned int)];

	/** Check input pointers */
	if ( !p_src )
	{
		result = COMMON_ERR_NULL_PTR;
		goto mml_sflc_write_out;
	}
	/** Check that address is integer aligned */
	else if ( address % K_MML_SFLC_QUANTUM_SIZE_IN_BYTES )
	{
		result = N_MML_SFLC_ERR_ADDRESS_NOT_ALIGNED;
		goto mml_sflc_write_out;
	}
#ifdef _WITH_STRICT_INTEGER_ALIGNED_
	/** Check data packet alignment */
	else if ( ( address + length ) % K_MML_SFLC_QUANTUM_SIZE_IN_BYTES )
	{
		result = N_MML_SFLC_ERR_PACKET_NOT_ALIGNED;
		goto mml_sflc_write_out;
	}
#endif /* _WITH_STRICT_INTEGER_ALIGNED_ */
	/** Overflow ? */
	else if ( ( MML_MEM_FLASH_BASE + MML_MEM_FLASH_SIZE ) < ( address + length ) )
	{
		result = N_MML_SFLC_ERR_OVERFLOW;
		goto mml_sflc_write_out;
	}
	else
	{
		/** #define	K_MML_SFLC_PGM_32BITS_SPECIAL_MASK		0xfffffffc */
		/**  */
		ofst_end = ( address + length ) & K_MML_SFLC_PGM_32BITS_SPECIAL_MASK;
		size_end = ( address + length ) - ofst_end;
		/** Erase Bytes */
		memset((void*)p_work, 0x00, sizeof(p_work));
	}
	/** Safety */
	if ( sizeof(unsigned int) < size_end )
	{
		/** Oops, I did it again ... */
		result = COMMON_ERR_FATAL_ERROR;
		goto mml_sflc_write_out;
	}
	/** Update data for 'Middle' process */
	last_size = length - size_end;
	/** Middle */
	result = mml_sflc_write_raw(address, p_tmp, last_size);
	if ( result )
	{
		/** Oops, I did it again ... */
		goto mml_sflc_write_out;
	}
	else
	{
		/** Update input data pointer */
		p_tmp += last_size;
	}
	/** End */
	if ( size_end )
	{
		/**  */
		last_size = sizeof(unsigned int) - size_end;
		/** Process unaligned start data */
		result = mml_sflc_read(ofst_end, (unsigned char*)p_work, sizeof(unsigned int));
		if ( NO_ERROR == result )
		{
			/** Put input data */
			memcpy((void*)p_work, (void*)p_tmp, size_end);
			/** Program merged data then */
			result = mml_sflc_write_raw(ofst_end, p_work, sizeof(unsigned int));
		}
	}
	/** We're done */
mml_sflc_write_out:
	return result;
}

/******************************************************************************/
int mml_sflc_read(unsigned int address,
					unsigned char *p_dst,
					unsigned int length)
{
	int											result = COMMON_ERR_UNKNOWN;

	/** Check input pointers */
	if ( !p_dst )
	{
		result = COMMON_ERR_NULL_PTR;
	}
	/** Overflow ? */
	else if ( ( MML_MEM_FLASH_BASE + MML_MEM_FLASH_SIZE ) < ( address + length ) )
	{
		result = N_MML_SFLC_ERR_OVERFLOW;
	}
	else
	{
		memcpy(p_dst, (unsigned char*)address, length);
		/**  */
		result = NO_ERROR;
	}
	/** We're done */
	return result;
}

/******************************************************************************/
int mml_sflc_erase(unsigned int address,
					unsigned int length)
{
	unsigned char								sema = FALSE;
	int											result = COMMON_ERR_UNKNOWN;
	unsigned int								i;
	unsigned int								page_base = 0;
	unsigned int								page_end = 0;
	unsigned int								work_size = 0;
	unsigned int								offset = 0;

	/** Check input parameters */
	if ( ( MML_MEM_FLASH_BASE + MML_MEM_FLASH_SIZE ) < ( address + length ) )
	{
		result = N_MML_SFLC_ERR_OVERFLOW;
		goto mml_sflc_erase_out;
	}
	/** Check address alignment  */
	else if ( address % K_MML_SFLC_QUANTUM_SIZE_IN_BYTES )
	{
		result = N_MML_SFLC_ERR_ADDRESS_NOT_ALIGNED;
		goto mml_sflc_erase_out;
	}
	/** Check if start address is page aligned ********************************/
	offset = address & ~K_MML_SFLC_ERASE_PAGE_SPECIAL_MASK;
	page_base = address & K_MML_SFLC_ERASE_PAGE_SPECIAL_MASK;
	/** Start address is not page aligned */
	if ( offset )
	{
		/** Initialize work buffer */
		memset(mml_sflc_buffer, K_MML_SFLC_OTP_VIRGIN_PATTERN_BYTE, sizeof(mml_sflc_buffer));
		/** Compute size of data in page to keep from erasing */
		work_size = address - page_base;
		/** Read data in page to keep from erasing ... */
		result = mml_sflc_read(page_base, mml_sflc_buffer, work_size);
		if ( result )
		{
			/** Oops, I did it again ... */
			goto mml_sflc_erase_out;
		}
		/** Erase page then ... */
		result = mml_sflc_page_erase_raw(page_base, K_MML_SFLC_ERASE_PAGE_SIZE);
		if ( result )
		{
			/** Oops, I did it again ... */
			goto mml_sflc_erase_out;
		}
		/** Program data back into this page */
		result = mml_sflc_write_raw(page_base, mml_sflc_buffer, K_MML_SFLC_ERASE_PAGE_SIZE);
		if ( result )
		{
			/** Oops, I did it again ... */
			goto mml_sflc_erase_out;
		}
		/**  */
		sema = TRUE;
	}
	else
	{
		sema = FALSE;
	}
	/** Check if end address is page aligned **********************************/
	offset = ( address + length ) & ~K_MML_SFLC_ERASE_PAGE_SPECIAL_MASK;
	page_end = ( address + length ) & K_MML_SFLC_ERASE_PAGE_SPECIAL_MASK;
	/** End address is not page aligned */
	if ( ( page_end - page_base ) && offset )
	{
		/** Initialize work buffer */
		memset(mml_sflc_buffer, K_MML_SFLC_OTP_VIRGIN_PATTERN_BYTE, sizeof(mml_sflc_buffer));
		/** Compute size of data in page to keep from erasing */
		work_size = K_MML_SFLC_ERASE_PAGE_SIZE - offset;
		/** Read data in page to keep from erasing ... */
		result = mml_sflc_read(( address + length ), (unsigned char*)( mml_sflc_buffer + offset ), work_size);
		if ( result )
		{
			/** Oops, I did it again ... */
			goto mml_sflc_erase_out;
		}
		/** Erase page then ... */
		result = mml_sflc_page_erase_raw(page_end, K_MML_SFLC_ERASE_PAGE_SIZE);
		if ( result )
		{
			/** Oops, I did it again ... */
			goto mml_sflc_erase_out;
		}
		/** Program data back into this page */
		result = mml_sflc_write_raw(page_end, mml_sflc_buffer, K_MML_SFLC_ERASE_PAGE_SIZE);
		if ( result )
		{
			/** Oops, I did it again ... */
			goto mml_sflc_erase_out;
		}
	}
	/** Treat pages in between if any *****************************************/
	/** Now 'work_size' is use as lasting number of pages to erase and
	 * 'offset' is used as page base address. */
	work_size = ( page_end - page_base ) / K_MML_SFLC_ERASE_PAGE_SIZE;
	if ( !work_size )
	{
		work_size = 1;
	}
	/** Next page ... */
	if ( TRUE == sema )
	{
		/** ... to erase is the one after 'page_base' */
		offset = page_base + K_MML_SFLC_ERASE_PAGE_SIZE;
	}
	else /* FALSE */
	{
		/** ... is 'page_base' */
		offset = page_base;
	}
	/** Loop to erase in between page(s) if any */
	for( i = 0;i < work_size;i++ )
	{
		result = mml_sflc_page_erase_raw(offset, K_MML_SFLC_ERASE_PAGE_SIZE);
		if ( result )
		{
			/** Oops, I did it again ... */
			goto mml_sflc_erase_out;
		}
		else
		{
			/** Point on next page then */
			offset += K_MML_SFLC_ERASE_PAGE_SIZE;
		}
	}
	/** We're done */
mml_sflc_erase_out:
	return result;
}

/******************************************************************************/
/* OTP ************************************************************************/
/******************************************************************************/
int mml_sflc_otp_write(e_mml_sflc_otp_id id,
						unsigned int address,
						unsigned char *p_src,
						unsigned int length)
{
	int											result = COMMON_ERR_UNKNOWN;
	unsigned int								end = 0;

	/** Check input pointers */
	if ( !p_src )
	{
		result = COMMON_ERR_NULL_PTR;
		goto mml_sflc_otp_write_out;
	}
	/** Check that address is integer aligned */
	else if ( address % sizeof(unsigned int) )
	{
		result = N_MML_SFLC_ERR_ADDRESS_NOT_ALIGNED;
		goto mml_sflc_otp_write_out;
	}
	/** Check length alignment */
	else if ( length % K_MML_SFLC_OTP_LINE_SIZE )
	{
		result = N_MML_SFLC_ERR_LENGTH_NOT_ALIGNED;
		goto mml_sflc_otp_write_out;
	}
	/** Check if data to be programmed is totally in chosen OTP area */
	if ( ( N_MML_SFLC_OTP_ID_USER == id ) &&
		( K_MML_SFLC_OTP_USER_BASE <= address ) )
	{
		end = K_MML_SFLC_OTP_USER_BASE + K_MML_SFLC_OTP_USER_SIZE;
	}
	else if ( ( N_MML_SFLC_OTP_ID_MAXIM == id ) &&
			( K_MML_SFLC_OTP_USER_BASE > address ) )
	{
		end = K_MML_SFLC_OTP_MAXIM_BASE + K_MML_SFLC_OTP_MAXIM_SIZE;
	}
	else
	{
		result = COMMON_ERR_INVAL;
		goto mml_sflc_otp_write_out;
	}
	/** Overflow ? */
	if ( end < ( address + length ) )
	{
		result = N_MML_SFLC_ERR_OVERFLOW;
	}
	else
	{
		/** Unlock - contains BUSY bit treatment */
		mml_sflc_otp_unlock();
		/** Ok then, let's do it */
		result = mml_sflc_write_raw(address, p_src, length);
		/** Lock - contains BUSY bit treatment */
		mml_sflc_otp_lock();
	}
	/** We're done */
mml_sflc_otp_write_out:
	return result;
}

/******************************************************************************/
int mml_sflc_otp_read(e_mml_sflc_otp_id id,
						unsigned int address,
						unsigned char *p_dst,
						unsigned int length)
{
	int											result = COMMON_ERR_UNKNOWN;
	unsigned int								end = 0;

	/** Check input pointers */
	if ( !p_dst )
	{
		result = COMMON_ERR_NULL_PTR;
		goto mml_sflc_otp_read_out;
	}
	else if ( ( ( K_MML_SFLC_OTP_MAXIM_BASE > address ) || ( ( K_MML_SFLC_OTP_MAXIM_BASE + K_MML_SFLC_OTP_MAXIM_SIZE ) < ( address + length ) ) ) && ( N_MML_SFLC_OTP_ID_MAXIM == id ) )
	{
		/**  */
		result = COMMON_ERR_OUT_OF_RANGE;
		goto mml_sflc_otp_read_out;
	}
	else if ( ( ( K_MML_SFLC_OTP_USER_BASE > address ) || ( ( K_MML_SFLC_OTP_USER_BASE + K_MML_SFLC_OTP_USER_SIZE ) < ( address + length ) ) ) && ( N_MML_SFLC_OTP_ID_USER == id ) )
	{
		/**  */
		result = COMMON_ERR_OUT_OF_RANGE;
		goto mml_sflc_otp_read_out;
	}
#ifdef _INPUT_STRICT_CHECK_
	/** Check that address is integer aligned */
	else if ( address % sizeof(unsigned int) )
	{
		result = N_MML_SFLC_ERR_ADDRESS_NOT_ALIGNED;
		goto mml_sflc_otp_read_out;
	}
	/** Check length alignment */
	else if ( length % K_MML_SFLC_OTP_LINE_SIZE )
	{
		result = N_MML_SFLC_ERR_LENGTH_NOT_ALIGNED;
		goto mml_sflc_otp_read_out;
	}
#endif /* _INPUT_STRICT_CHECK_ */
	/** Check if data to be read is totally in chosen OTP area */
	if ( ( N_MML_SFLC_OTP_ID_USER == id ) &&
		( K_MML_SFLC_OTP_USER_BASE <= address ) )
	{
		end = K_MML_SFLC_OTP_USER_BASE + K_MML_SFLC_OTP_USER_SIZE;
	}
	else if ( ( N_MML_SFLC_OTP_ID_MAXIM == id ) &&
			( K_MML_SFLC_OTP_USER_BASE > address ) )
	{
		end = K_MML_SFLC_OTP_MAXIM_BASE + K_MML_SFLC_OTP_MAXIM_SIZE;
	}
	else
	{
		result = COMMON_ERR_INVAL;
		goto mml_sflc_otp_read_out;
	}
	/** Overflow ? */
	if ( end < ( address + length ) )
	{
		result = N_MML_SFLC_ERR_OVERFLOW;
	}
	else
	{
		/** Unlock - contains BUSY bit treatment */
		mml_sflc_otp_unlock();
		/** Read data */
		memcpy(p_dst, (unsigned char*)address, length);
		/** Lock - contains BUSY bit treatment */
		mml_sflc_otp_lock();
		/**  */
		result = NO_ERROR;
	}
	/** We're done */
mml_sflc_otp_read_out:
	return result;
}

/******************************************************************************/
/* EOF */
