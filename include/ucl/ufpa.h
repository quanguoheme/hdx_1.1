/** @file ufpa.h
 * @defgroup UCL
 * Comparaison Test of Unsigned Fixed-Precision Number
 *
 * @par Header:
 * @link
 *
 * @ingroup UCL
 *
 */
#ifndef _UFPA_H_
#define _UFPA_H_

/* ========================================================================== */
/** <b>Test "Greater than" for Unsigned</b>.
 * Test @f$ x > y @f$
 *
 * @param[in] x Pointer to @a x
 * @param[in] y Pointer to @a y
 * @param[in] s The precision
 *
 * @return Test result
 *
 * @retval UCL_TRUE @f$ x > y @f$
 * @retval UCL_FALSE Not @f$ x > y @f$
 *
 * @ingroup UCL_FPA
 */
int ufpa_gt(u32 *x, u32 *y, u32 s);


/* ========================================================================== */
/** <b>Test "Greater or Equal to" for Unsigned</b>.
 * Test @f$ x \ge y @f$
 *
 * @param[in] x Pointer to @a x
 * @param[in] y Pointer to @a y
 * @param[in] s The precision
 *
 * @return Test result
 *
 * @retval UCL_TRUE @f$ x \ge y @f$
 * @retval UCL_FALSE Not @f$ x \ge y @f$
 *
 * @ingroup UCL_FPA
 */
int ufpa_ge(u32 *x, u32 *y, u32 s);


/* ========================================================================== */
/** <b>Test "Less than" for Unsigned</b>.
 * Test @f$ x < y @f$
 *
 * @param[in] x Pointer to @a x
 * @param[in] y Pointer to @a y
 * @param[in] s The precision
 *
 * @return Test result
 *
 * @retval UCL_TRUE @f$ x < y @f$
 * @retval UCL_FALSE Not @f$ x < y @f$
 *
 * @ingroup UCL_FPA
 */
int __API__ ufpa_lt(u32 *x, u32 *y, u32 s);


/*============================================================================*/
/** Shift Right Logical for Unsigned large number.
 * @f$ w = x >> shift @f$
 *
 * @param[out] w A pointer to @a w
 * @param[in] x A pointer to @a x
 * @param[in] s The precision
 * @param[in] r The shift
 *
 * @return Carry or error code
 *
 * @ingroup FPA
 */
int ufpa_srl(u32 *w, u32 *x, u32 s, u32 r);


/*============================================================================*/
/** Shift Left Logical for unsigned large numbers.
 * @f$ w = x << shift @f$
 *
 * @param[out] w A pointer to @a w
 * @param[in] x A pointer to @a x
 * @param[in] s The precision
 * @param[in] r The shift
 *
 * @return Carry or error code
 *
 * @ingroup FPA
 */
int ufpa_sll(u32 *w, u32 *x, u32 s, u32 r);


/*============================================================================*/
/** <b>Double precision multiplication of unsigned large numbers</b>.
 * @f$ w = x \times y \bmod 2^{2s}-1@f$
 *
 * @param[out] w The pointer to the result of the multiplication
 * @param[in] x The pointer to @a x
 * @param[in] y The pointer to @a y
 * @param[in] s The precision of x and y
 *
 * @return Error code
 *
 * @retval OK No error occurred
 *
 * @ingroup FPA
 */
int ufpa_dpmult(u32 *w, const u32 *x, const u32 *y, u32 s);


#endif // _UFPA_H_
