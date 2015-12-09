/** @file fpa.h
 * @defgroup UCL
 * Fixed-Precision Primary Operations
 *
 * @par Header:
 * @link
 *
 * @ingroup UCL
 */
#ifndef _BIGNUM_FPA_H_
#define _BIGNUM_FPA_H_

#include "ufpa.h"
#include "mfpa.h"

/*============================================================================*/
/** <b>Large integer copy</b>.
 * @f$ w = x @f$
 *
 * @param[out] w Pointer to @a w
 * @param[in] x Pointer to @a x
 * @param[in] s The precision
 *
 * @ingroup FPA
 */
int __API__ fpa_cpy(u32 *w, u32 *x, u32 s);


/*============================================================================*/
/** Large integer incrementation.
 * @f$ w = w + 1 @f$
 *
 * @param[in,out] w Pointer to @a w
 * @param[in]  s The precision
 *
 * @return Carry
 *
 * @ingroup FPA
 */
int fpa_inc(u32 *w, u32 s);


/*============================================================================*/
/** Addition of large integer.
 * @f$ w = x + y @f$
 *
 * @param[out] w Pointer to the result of the addition
 * @param[in] x Pointer to @a x
 * @param[in] y Pointer to @a y
 * @param[in] s The precision
 *
 * @return Carry
 *
 * @ingroup FPA
 */
int fpa_add(u32 *w, const u32 *x, u32 *y, u32 s);


/*============================================================================*/
/** Substraction of large integer.
 * @f$ w = x - y @f$
 *
 * @param[out] w Pointer to the result of the substraction
 * @param[in] x Pointer to @a x
 * @param[in] y Pointer to @a y
 * @param[in] s The precision
 *
 * @return Carry
 *
 * @ingroup FPA
 */
int fpa_sub(u32 *w, u32 *x, u32 *y, u32 s);

int  fpa_dec(u32 *x, u32 s);

int fpa_sub_opt (u32* w, u32* x, u32* y, u32 s);

void mult(u32 *t,u32 *a,u32 *b,u32 s);

/* ========================================================================== */
/** Prime Generation using IEEE 1363 Annex A.16.11.
 *
 * Generate a prime number @p p between @p pMax and @p pMin for which @p p-1
 * is relatively prime to @p f
 *
 * @param[out] prime   The prime
 * @param[in] f   A large number
 * @param[in] pMin  A large number
 * @param[in] pMax  A large number
 * @param[in] t  Number of trials for Miller-Rabin Test
 * @param[in]   s   The precision
 *
 * @return Error code
 *
 * @ingroup UCL_FPA_PRIME
 */
int __API__ fpa_gen_prime(u32 *p, u32 *f, u32 *pMin, u32 *pMax, u32 s);
#endif // _BIGNUM_FPA_H_
