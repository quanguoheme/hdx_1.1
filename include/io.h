/*
 * io.h --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2010, Maxim Integrated
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
 * Created on: Jun 23, 2010
 * Author:
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: 4631     $:  Revision of last commit
 * $Author:: robert.muc#$:  Author of last commit
 * $Date:: 2015-01-19 2#$:  Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */
#ifndef IO_H_
#define IO_H_

#if __STDC_VERSION__ < 199901L
#ifndef inline
#define inline __inline
#endif
#endif

/** @file io.h Cobra IOs */

/** @defgroup COBRA Cobra Tools */

/** @defgroup COBRA_IO Cobra IOs
 *
 * @ingroup COBRA
 *
 * @{
 */

#ifndef __ASSEMBLER__

/** IO Register. */
typedef volatile unsigned int io_register_t;

/* -------------------------------------------------------------------------- */

/** Shift left */
#define SHIFTL(x,n) ((x)<<(n))


/** Get register bits field mask
 *
 * Expands REG_BF with _OFST and _MASK.
 *
 * @param[in] REG_BF Bits field macro name to expand
 */
#define IO_REG_BF_GET_MASK(REG_BF) ((REG_BF ## _MASK)<<(REG_BF ## _OFST))


/** Set a bits field value using macro name
 *
 * Expands REG_BF with _OFST and _VALNAME.
 *
 * @param[in] reg		Register address
 * @param[in] REG_BF	Bits field macro name to expand
 * @param[in] VALNAME	Bits field value macro name to expand
 */
#define IO_REG_BF_SET_VALNAME(reg, REG_BF, VALNAME)					\
{																	\
	io_register_t *_reg_ = (io_register_t *)reg;					\
	unsigned int _tmp_;												\
	_tmp_ = *_reg_ & ~IO_REG_BF_GET_MASK(REG_BF);					\
																	\
	*_reg_ = _tmp_ | ((REG_BF ## _ ## VALNAME)<<(REG_BF ## _OFST));	\
}

/** Set a bits field value
 *
 * @param[in] reg		Register address
 * @param[in] REG_BF	Bits field macro name to expand
 * @param[in] val		Value
 *
 */
#define IO_REG_BF_SET_VALUE(reg, REG_BF, val)		\
{													\
	io_register_t *_reg_ = (io_register_t *)reg;	\
	unsigned int _tmp_;								\
	_tmp_ = *_reg_ & ~IO_REG_BF_GET_MASK(REG_BF);	\
													\
	*_reg_ = _tmp_ | ((val)<<(REG_BF ## _OFST));	\
}

/** Get a bits field value
 *
 * @param[in] reg		Register address
 * @param[in] REG_BF	Bits field macro name to expand
 *
 */
#define IO_REG_BF_GET_VALUE(reg, REG_BF)			\
({													\
	io_register_t *_reg_ = (io_register_t *)reg;	\
	unsigned int _tmp_;								\
	_tmp_ = *_reg_ & IO_REG_BF_GET_MASK(REG_BF);	\
	(_tmp_ >> REG_BF ## _OFST);						\
})


/** Set a bits field to ENABLED
 *
 * @param[in] reg		Register address
 * @param[in] REG_BF	Bits field macro name to expand
 *
 */
#define IO_REG_BF_ENABLE(reg, REG_BF)					\
{														\
	io_register_t *_reg_ = (io_register_t *)reg;		\
	unsigned _tmp_ = *_reg_ & ~(REG_BF ## _MASK);		\
	_tmp_ |= ((REG_BF ## _ENABLE)<<(REG_BF ## _OFST));	\
	*_reg_ |= _tmp_;									\
}

/** Set a bits field to DISABLED
 *
 * @param[in] reg		Register address
 * @param[in] REG_BF	Bits field macro name to expand
 *
 */
#define IO_REG_BF_DISABLE(reg, REG_BF)							\
{																\
	io_register_t *_reg_ = (io_register_t *)reg;				\
	unsigned _tmp1_ = *_reg_ & ~(REG_BF ## _MASK);				\
	unsigned _tmp2_ = (~(REG_BF ## _ENABLE))&(REG_BF ## _MASK);	\
	*_reg_ = _tmp1_ | (_tmp2_<<(REG_BF ## _OFST));				\
}


/** Read a IO register.
 */
#define IO_READ_U32( _register_, _value_ ) \
	((_value_) = *((io_register_t *)(_register_)))

/** Write a IO register.
 */
#define IO_WRITE_U32( _register_, _value_ ) \
	(*((io_register_t *)(_register_)) = (_value_))

/** Read some bits of a IO register.
 */
#define IO_READ_BITS( _register_, _offset_ , _mask_, _value_ ) \
	( (_value_) = (( *((io_register_t *)(_register_)) )&(_mask_) )>>_offset_)

/** Write some bits in a IO register.
 */
#define IO_WRITE_BITS( _register_, _offset_ , _mask_, _value_ ) \
	(*((io_register_t *)(_register_)) = ( (_value_)<<(_offset_) )&(_mask_))

/* -------------------------------------------------------------------------- */

/** Read a IO register.
 *
 * @param addr	Address of the register
 *
 * @return The content of the register
 */
static inline unsigned io_read_u32(unsigned addr) {
	unsigned value;
	value = *((io_register_t *)addr);
	return value;
}

/** Read a IO register.
 *
 * @param addr	Address of the register
 * @param value	Value to write
 *
 * @return The content of the register
 */
static inline unsigned io_write_u32(unsigned addr, unsigned value) {
	*((io_register_t *)addr) = value;
	value = *((io_register_t *)addr);
	return value;
}

#else /* #ifndef __ASSEMBLER__ */

/** Shift left */
#define SHIFTL(x,n) x, LSL #n

/** Compute register bits field value.
 *
 * @param _register_	Register macro name
 * @param _value_		Value to compute
 * @return
 */
#define REGBITS_GETBITS(_register_, _value_) _value_, LSL #_register_ ## _OFFSET

/** Compute register bits field mask
 *
 * @param _register_
 * @return
 */
#define REGBITS_GETMASK(_register_) _register_ ## _MASK, LSL #_register_ ## _OFFSET))

#endif /* #ifndef __ASSEMBLER__ */

/** @} */ /* @defgroup COBRA_IO */

#endif /* IO_H_ */
