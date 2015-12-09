/*===========================================================================
 *
 * ucl_rng.h
 *
 *==========================================================================*/
/*===========================================================================
 *
 * Copyright © 2009 Innova Card. All Rights Reserved. Do not disclose.
 *
 * This software is the confidential and proprietary information of
 * Innova Card ("Confidential Information"). You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered
 * into with Innova Card.
 *
 * Innova Card makes no representations or warranties about the suitability of
 * the software, either express or implied, including but not limited to
 * the implied warranties of merchantability, fitness for a particular purpose,
 * or non-infrigement. Innova Card shall not be liable for any damages suffered
 * by licensee as the result of using, modifying or distributing this software
 * or its derivatives.
 *
 *==========================================================================*/
/*===========================================================================
 *
 * Purpose: Random Number Generator Interface
 *
 *==========================================================================*/
#ifndef UCL_RNG_H_
#define UCL_RNG_H_

/** @file ucl_rng.h
 * @defgroup UCL_RNG RNG Interface
 * Interface for Random Number Generator.
 *
 * This interface allows to attach any RNG for all random needs of the UCL.
 * The default RNG is the hardware USIP&reg; TRNG.
 *
 * @par Header:
 * @link ucl_rng.h ucl_rng.h @endlink
 *
 *
 * @ingroup UCL_RAND
 */


#include "ucl/ucl_trng.h"

#ifdef __cplusplus
extern "C"
{
#endif /* _ cplusplus  */


/** <b>Attach</b>.
 * Attach a RNG.
 *
 * @param[in] rng The pointer to the RNG
 *
 * @return Error code
 *
 * @retval #UCL_OK No error occurred
 *
 * @ingroup UCL_RNG
 */
int __API__ ucl_rng_attach(int (*rng)(u8* rand, u32 rand_byteLen, int option));


/** <b>Detach</b>.
 * Detach the previously attached RNG.
 *
 * @return Error code
 *
 * @retval #UCL_OK No error occurred
 *
 * @ingroup UCL_RNG
 */
int __API__ ucl_rng_detach(void);


/** <b>Get pointer</b>.
 * Get the pointer of the current RNG.
 *
 * @return A function pointer
 *
 * @ingroup UCL_RNG
 */
void * __API__ ucl_rng_getpt(void);


/** <b>RNG Read</b>.
 * Read random octet string using the attached RNG.
 *
 * @param[out] rand         Random octet string
 * @param[in]  rand_byteLen Random byte length
 * @param[in]  option       A value between:
 *                              @li #UCL_RAND_DEFAULT
 *                              @li #UCL_RAND_NO_NULL
 *
 * @return Error code or a positive value equal to @p the generated byte number
 *
 * @retval #UCL_INVALID_OUTPUT    If @p rand is the pointer #NULL
 * @retval #UCL_NOP               If @p rand_byteLen = 0
 * @retval #UCL_NO_TRNG_INTERFACE IF there is no interface to the USIP&reg; TRNG
 *
 * @ingroup UCL_RNG
 */
int __API__ ucl_rng_read(u8* rand, u32 rand_byteLen, int option);


#ifdef __cplusplus
}
#endif /* _ cplusplus  */


#endif /*UCL_RNG_H_*/