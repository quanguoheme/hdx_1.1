/*
 * sc_states.h -- Smartcard states
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

#ifndef _SC_STATES_H_
#define _SC_STATES_H_
#include <stdint.h>
#include "sc_types.h"


/** @file    iccabstract.h Smartcard drivers abstraction layer
 *  @version 2.0.3
 *  @date    2015/02/12
 *  @author  Jean-Marc Simohand <jeanmarc.simohand@maximintegrated.com>
 *
 * @{
 */

/** @defgroup ICC_STATES   Smartcard states
 *
 * @ingroup ICC_ABSTRACTION
 *
 * This is not intended to be directly used by the user.
 *
 * @{
 */


/** @enum    CardState_t Card Event/states enum
 */
typedef enum {
    ICC_INSERTION,      /**< a card has been inserted */
    ICC_REMOVAL,        /**< the card has been removed */
    ICC_FAULT,          /**< card fault (over current, short-circuit...)*/
} CardState_t;


/** @enum    CardPowerState_t Icc power states enum.
 */
typedef enum {
    POWER_DOWN = 0, /**< Power off the card */
    POWER_UP,       /**< turn on with the voltage defined in the context */
    RESET_DO,       /**< Put the card in RESET (RST pin low) */
    RESET_RELEASE,  /**< Release the card reset signal (RST pin high) */
} CardPowerState_t;


/** @} */ /*defgroup*/
/** @} */ /*file*/
#endif /* _SC_DATA_H_ */
