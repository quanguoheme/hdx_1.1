/*
 * ATR.h -- Smartcard stack Anwser To Reset Management.
 *
 * --------------------------------------------------------------------------
 *
 * Copyright (c) 2015, Maxim Integrated Products, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _ATR_H_
#define _ATR_H_
#include <stdint.h>
#include "sc_errors.h"
#include "sc_types.h"

/** @file    ATR.h Answer To Reset (ATR) management
 *  @version 2.0.5
 *  @date    2015/01/15
 *  @author  Jean-Marc Simohand <jeanmarc.simohand@maximintegrated.com>
 *
 * @{
 */


/** @defgroup ATR    Answer To Reset (ATR) management
 *
 * @ingroup SMARTCARD_DRIVER
 *
 * This file defines the Answer To Reset functions & variables.
 *
 * @note
 * @{
 */


/** @defgroup ATR_DEFINES    Answer To Reset (ATR) constants definition
 *
 * @ingroup ATR
 * @{
 */
#define TA_BIT          (1<<4)  /**< TAx presence bit (bit 4, 0x10) */
#define TB_BIT          (1<<5)  /**< TBx presence bit (bit 5, 0x20) */
#define TC_BIT          (1<<6)  /**< TCx presence bit (bit 6, 0x40) */
#define TD_BIT          (1<<7)  /**< TDx presence bit (bit 7, 0x80) */

#define BGTT0           16      /**< EMV set the BGT to 16 ETU for T=0 */
#define BGTT1           22      /**< EMV set the BGT to 22 ETU for T=1 */

/**  @var NB_BITS
 * @brief Use to get the number of bytes following a TDi byte\n
 *
 * The number of bytes to get is the number of bits set in the
 * most significant nibble of the TDx byte.
 *  */
extern const uint8_t NB_BITS[];

/** @var wDi
 * @brief Di constants lookup array
 */
extern const uint8_t wDI[];


/** @} */ /* @defgroup ATR_DEFINES */
/** @} */ /* 2nd closing brace, I do not understand why I do need it ! */

/**@defgroup ATR_FUNCTIONS    Answer To Reset API
 * @ingroup ATR
 *
 * @func     GetATR(...)
 * @param [in] slotid                 Slot Id to activate
 * @param [out] pRxBuffer             pointer on the output buffer (where the ATR will be stored)
 * @param [out] pATRLength            address of the ATR length word
 * @param [in]  ATRDeactivateOnError  if bTRUE the card will be deactivated on error.
 * @param [in]  ActivationParams      card activation parameters (cf #ActivationParams_t)
 *
 * @return it returns ICC_OK on success
 * @return an #IccReturn_t return code on fail
 */
IccReturn_t GetATR(uint8_t slotid, uint8_t *pRxBuffer,
                   uint32_t *pATRLength,
                   boolean_t ATRDeactivateOnError,
                   ActivationParams_t   *ActivationParams);

/** @} */ /* @defgroup ATR_FUNCTIONS */
/** @} */ /* @defgroup SMARCARD_ATR */
/** @} */ /* @file    atr.h*/
#endif /*_ATR_H_*/
