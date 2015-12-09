/*============================================================================
 *
 * ucl_dsa.h
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2013 Maxim Integrated.
 * All Rights Reserved. Do not disclose.
 *
 * This software is the confidential and proprietary information of
 * Innova Card ("Confidential Information"). You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered
 * into with Maxim Integrated.
 *
 * Maxim Integrated makes no representations or warranties about the suitability of
 * the software, either express or implied, including but not limited to
 * the implied warranties of merchantability, fitness for a particular purpose,
 * or non-infrigement. Innova Card shall not be liable for any damages suffered
 * by licensee as the result of using, modifying or distributing this software
 * or its derivatives.
 *
 *
 * Purpose : DSA
 *
 *==========================================================================*/
#ifndef _UCL_DSA_H_
#define _UCL_DSA_H_
int ucl_dsa_keygen(u8 *y,u8 *x, u8 *p,u32 plength, u8 *q, u8 *g);
int __API__ ucl_dsa_sign(u8 *r,u8 *s,u8 *p,u32 plength, u8 *q, u8 *g, u8 *x,u8 *input, u32 inputlength,u32 configuration);
int __API__ ucl_dsa_verify(u8 *r,u8 *s,u8 *p,u32 plength, u8 *q, u8 *g, u8 *y,u8 *input, u32 inputlength,u32 configuration);
#endif//UCL_DSA_H
