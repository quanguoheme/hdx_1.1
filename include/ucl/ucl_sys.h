/*============================================================================
 *
 * ucl_sys.h
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2009 Innova Card.
 * All Rights Reserved. Do not disclose.
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
/*============================================================================
 *
 * Purpose :
 *
 *==========================================================================*/
#ifndef _UCL_SYS_H_
#define _UCL_SYS_H_

#include "ucl/ucl_config.h"

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_sys.h
 * @defgroup UCL_SYSTEM UCL System
 * UCL System Functions.
 *
 * @par Header:
 * @link ucl_sys.h ucl_sys.h @endlink
 *
 */

/** <b>UCL Init</b>.
 * USIP&reg; Cryptographic Library Initialisation.
 * Initialisation of stack and hardware interfaces (if available).
 *
 * @param[in] buffer Work buffer for pkc
 * @param[in] size   Size of the buffer (number of unsigned ints)
 *
 * @pre @p size must greater than or equal to 1024.
 *
 * @note The UCL Stack error does not matter if you don't use #UCL_PKC functions.
 * @note The error "USIP(R) TRNG not available" occurres for USIP Linux platform
 * if the file '/dev/hwrandom' does not exist.
 * @return #UCL_OK or Error vector
 *
 * @retval The error vector is a combination of:
 *     @li 0x001: UCL Stack error (Buffer NULL or invalid size)
 *     @li 0x010: USIP(R) AES not available
 *     @li 0x020: USIP(R) AES Corrrupted
 *     @li 0x100: USIP(R) TRNG not available
 *     @li 0x200: USIP(R) TRNG Corrupted
 *
 * @warning After the version 2.1.0 the @p buffer is mandatory
 *
 * @ingroup UCL_SYSTEM
 */
int __API__ ucl_init(u32 *buffer, u32 size);

#if defined(JIBE_LINUX_HW)
int __API__ ucl_exit( void );
#endif /*#if defined(__jibe) && defined(JIBE_USERLAND_CRYPTO) && defined(__linux) */
#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_SYS_H_ */
