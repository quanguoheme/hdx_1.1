/*============================================================================
 *
 * ucl_aes_gcm.h [22-sep-12]
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright (c) 2012 Maxim Integrated. All Rights Reserved. Do not disclose.
 *
 * This software is the confidential and proprietary information of
 * Maxim Integrated ("Confidential Information"). You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered
 * into with Maxim Integrated.
 *
 * Maxim Integrated makes no representations or warranties about the suitability of
 * the software, either express or implied, including but not limited to
 * the implied warranties of merchantability, fitness for a particular purpose,
 * or non-infrigement. Maxim Integrated shall not be liable for any damages suffered
 * by licensee as the result of using, modifying or distributing this software
 * or its derivatives.
 *
 *==========================================================================*/
/*============================================================================
 *
 * Purpose : AES-GCM and AES-GMAC API description
 *
 *==========================================================================*/
void __API__ ucl_gcm_init_r(void);
#ifdef LARGE_MEMORY
void __API__ ucl_gcm_init_prod(u8 *h);
#endif
int __API__ ucl_aes_gcm_auth(u8 *auth_tag,int auth_tag_bit_len,u8 *c,u8 *p,int input_bit_len_high,int input_bit_len_low,u8 *aad, int aad_bit_len_high,int aad_bit_len_low, u8 *key,u8 *iv,int iv_bit_len_high, int iv_bit_len_low,int mode);
int __API__ ucl_aes_gmac_auth(u8 *auth_tag,int auth_tag_bit_len,u8 *aad, int aad_bit_len_high,int aad_bit_len_low, u8 *key,u8 *iv,int iv_bit_len_high, int iv_bit_len_low);
