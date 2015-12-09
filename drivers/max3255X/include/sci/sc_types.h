/*
 * sc_types.h -- Smartcard data types & structures definitions
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
#ifndef _SC_TYPES_H_
#define _SC_TYPES_H_
#include <stdint.h>
#include "sc_errors.h"
#include "sc_states.h"
#include "OS_types.h"

/** @file    sc_types.h  Smartcard data structure defintions
 *  @version 2.0.3
 *  @date    2015/01/15
 *  @author  Jean-Marc Simohand <jeanmarc.simohand@maximintegrated.com>
 *
 * @{
 */

/** @var Di Di constants lookup array
 */
extern const uint8_t Di[16];

/** @var Fi Frequency divider constants lookup array
 */
extern const uint16_t Fi[16];

/** @defgroup SMARTCARD_TYPES   Smartcard Types & Structures definitions
 *
 * @ingroup SMARTCARD_DRIVER
 *
 * This file defines all the drivers/stacks data types & structures.
 *
 * @note
 * @{
 */

/** @def NULL NULL pointer definition
 *  @brief In case of NULL is not define, set NULL as a pointer on 0
 */
#ifndef NULL
# define NULL     (0)
#endif

/** @typedef    boolean_t defines a boolean type & the bTRUE and bFALSE values.
 */
typedef enum {
    bFALSE = 0,
    bTRUE = 1,
} boolean_t;

/** @enum    IccVoltage_t declares the configurable voltages.
 *
 *   This declares the configurable voltages, some may not be applicable.
 *   It depends on the Analog Front End and on the Smartcard itself
*/
typedef enum
{
    VCC_1V8 = 0,        /**< turn on with VCC=1.8v (Class C) */
    VCC_3V  ,           /**< turn on with VCC=3v (Class B) */
    VCC_5V  ,           /**< turn on with VCC=5v (Class A) */
    LAST_ALLOWED_VOLTAGE = VCC_5V,
} IccVoltage_t;


/** @typedef    ActivationParams_t contains all the Slot activation parameters
 *
 *   This struct declares all the configurable slot activation parameters
*/
typedef struct
{
    IccVoltage_t    IccVCC;                 /**< Card voltage setting */
    uint8_t         IccResetDuration;       /**< Reset duration in ETU */
    uint16_t        IccATR_Timeout;         /**< ATR Timeout value in ETU */
    uint8_t         IccTS_Timeout;          /**< TS Byte timeout in ETU */
    uint8_t         IccWarmReset;           /**< If not 0, we operate a warm reset */
} ActivationParams_t;




/** @typedef    IccRequest_t defines a data exchange between the reader a the card
*/
typedef struct
{
    uint8_t        *IccData;            /**< pointer on the receive buffer*/
    uint16_t       IccLen;              /**< length to receive */
    uint16_t       IccReceived;         /**< number of received bytes */
    boolean_t      IccLastByte;         /**< is it the last byte to receive */
    uint16_t       IccEDC;              /**< current LRC/CRC value */
    IccReturn_t    IccStatus;           /**< card status */
} IccRequest_t;


/** @} */ /* @defgroup SMARTCARD_TYPES */

/** @defgroup SMARTCARD_LOW_LEVEL_OPS Smartcard low level driver operations
 * @ingroup ICC_ABSTRACTION
 *
 * @note
 * @{
 */

 /** @typedef UartId_t Defines  UART interfaces
  */
typedef enum {
    SCI_0 = 0,      /**< Smartcard interface 0 */
    SCI_1,          /**< Smartcard interface 1 */
} UartId_t;




 /** @} */

/** @} */ /* @file    sc_types.h*/

#endif /*_SC_TYPES_H_*/
