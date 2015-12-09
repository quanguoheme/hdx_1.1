/*
 * sc_erros.h -- Smartcard errors definition
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

#ifndef _SC_ERRORS_H_
#define _SC_ERRORS_H_
#include <stdint.h>

/** @file    sc_errors.h Smartcard error codes
 *  @version 2.0.3
 *  @date    2015/01/15
 *  @author  Jean-Marc Simohand <jeanmarc.simohand@maximintegrated.com>
 *
 * @{
 */


/** @defgroup DRIVER_ERRORS    Smartcard error codes definitions
 *
 * @ingroup SMARTCARD_DRIVER
 *
 * @brief    This file defines all the drivers/stacks error codes.
 *
 * @note
 * @{
 */

/** @typedef    IccReturn_t
 *  @brief      contains all the UART / AFE drivers & Stacks error codes \n
 *              Failure value is negative, \n
 *              Success is 0.
*/
typedef enum {
    ICC_ERR_BAD_PARAMETER       = -127,   /**< Inconsistent param from user */
    ICC_ERR_OVERCURRENT,                  /**< Smartcard overcurrent detected */
    ICC_ERR_UNKOWN,                       /**< all unknow error sources */
    ICC_ERR_NOTPERMITTED,                 /**< ICC already powered when trying to change the voltage */
    ICC_ERR_POWERED,                      /**< ICC already powered when trying to change the voltage */
    ICC_ERR_NULL_PTR,                     /**< NULL pointer received */
    ICC_ERR_BAD_BLOCK,                    /**< Wrong block on T=1 procotocol*/
    ICC_ERR_BAD_EDC,                      /**< Bad LRC or CRC */
    ICC_ERR_TRANSMISSION,                 /**< Error when Tx on T=1 */
    ICC_ERR_ABORT_OCCURED,                /**< ABORT Request during a transaction */
    ICC_ERR_PTS_NEGOTIATION,              /**< PTS negociation failed */
    ICC_ERR_BAD_TCK,                      /**< Bad TCK byte */
    ICC_ERR_BAD_TS,                       /**< TS is neither 3B or 3F */
    ICC_ERR_BAD_ATR_VALUE,                /**< Inconsistent ATR received */
    ICC_ERR_BAD_PROCEDURE_BYTE,           /**< Wrong PCB value */
    ICC_ERR_WRONG_LEN,                    /**< Inconsitent command len */
    ICC_ERR_ATR_TIMEOUT,                  /**< Card Mute or parity issue */
    ICC_ERR_BAD_SLOT,                     /**< Unkown or unitialized Slot */
    ICC_ERR_REMOVED,                      /**< Card absent or removed */
    ICC_ERR_PRESENT_INACTIVE,             /**< Card not powered */
    ICC_ERR_PARITY,                       /**< Parity check error */
    ICC_ERR_TIMEOUT,                      /**< timeout during a Rx */
    ICC_ERR_TX_UNDERRUN,                  /**< Tx FIFO underrun (error !) */
    ICC_ERR_RX_OVERRUN,                   /**< overflow on the Rx FIFO */
    ICC_NEED_WARMRESET,                   /**< the Icc needs a warm reset */

    ICC_RX_PENDING,                       /**< Rx is in progress */
    ICC_OK = 0,                           /**< Operation succeed */
} IccReturn_t;


/** @} */ /* @defgroup SMARCARD_ERRORS */
/** @} */ /* @file    sc_errors.h*/
#endif /*_SC_ERRORS_H_*/
