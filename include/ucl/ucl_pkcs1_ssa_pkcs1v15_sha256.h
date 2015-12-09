/*============================================================================
 *
 * ucl_pkcs1_ssa_pkcs1v15_sha256.h
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2009 Innova Card. All rights reserved. Do not disclose.
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
 * Purpose : PKCS#1 V2.1 RSASSA-PKCS1V15 with SHA256
 *
 *==========================================================================*/
#ifndef _UCL_RSA_SSA_PKCS1V15_SHA256_H_
#define _UCL_RSA_SSA_PKCS1V15_SHA256_H_

#include "ucl_rsa.h"

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */


/** @file ucl_pkcs1_ssa_pkcs1v15_sha256.h
 * @defgroup UCL_PKCS1V21_SSA_PKCSV15_SHA256 RSASSA-PKCS1V15 SHA256
 * Signature scheme RSA PKCS#1 V1.5 using the hash function SHA256.
 *
 * @par Header:
 * @link ucl_pkcs1_ssa_pkcs1v15_sha256.h ucl_pkcs1_ssa_pkcs1v15_sha256.h @endlink
 *
 * @ingroup UCL_PKCS1V21_SSA_PKCS1V15
 */


/*============================================================================*/
/** <b>RSASSA-PKCS1V15-SHA256 Signature Generation</b>.
 * Signature generation using hash function SHA256.
 *
 * @param[out] signature      The generated signature
 * @param[in]  message        The message to be signed
 * @param[in]  message_length The message byte length
 * @param[in]  keyPr          RSA private key
 * 
 * @note The output buffer length is the modulus length
 *
 * @return Error code
 *
 * @retval #UCL_OK if no error occurred
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_SSA_PKCSV15_SHA256
 */
int __API__ ucl_pkcs1_ssa_pkcs1v15_sha256_sign(u8 *signature, u8 *message,
                                       u32 message_length, 
                                       ucl_rsa_private_key_t *keyPr);


/*============================================================================*/
/** <b>RSASSA-PKCS1V15-SHA256 CRT Signature Generation</b>.
 * CRT Signature generation using hash function SHA256.
 *
 * @param[out] signature      The generated signature
 * @param[in]  message        The messageto be signed
 * @param[in]  message_length The message byte length
 * @param[in]  keyPr          RSA CRT private key
 *
 * @note The output buffer length is the modulus length
 * 
 * @return Error code
 *
 * @retval #UCL_OK if no error occurred
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_SSA_PKCSV15_SHA256
 */
int __API__ ucl_pkcs1_ssa_pkcs1v15_sha256_crt_sign(u8 *signature, u8 *message,
        u32 message_length, ucl_rsa_crt_private_key_t *keyPr);


/*============================================================================*/
/** <b>RSASSA-PKCS1V15-SHA256 Signature Verification</b>.
 * Signature verification using hash function SHA256.
 *
 * @param[in] signature      The signature to verify
 * @param[in] message        The message
 * @param[in] message_length The message byte length
 * @param[in] keyPu          The RSA public key
 * 
 * @note The signature length is the modulus length
 *
 * @return Error code
 *
 * @retval #UCL_OK if the signature is valid
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_SSA_PKCSV15_SHA256
 */
int __API__ ucl_pkcs1_ssa_pkcs1v15_sha256_verify(u8 *signature, u8 *message,
        u32 message_length, ucl_rsa_public_key_t *keyPu);


#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /*_UCL_RSA_SSA_PKCS1V15_SHA256_H_ */
