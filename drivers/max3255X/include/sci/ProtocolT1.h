/*
 * ProtocolT1.h -- Send/Receive APDU using T=1 protocol
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

#ifndef _PROTOCOL_T1_H_
#define _PROTOCOL_T1_H_

#include <stdint.h>
#include "sc_errors.h"
#include "slot.h"

/** @file    ProtocolT1.h Send/Receive APDU using T=1 protocol
 *  @version 2.0.3
 *  @date    2015/02/12
 *  @author  Jean-Marc Simohand <jeanmarc.simohand@maximintegrated.com>
 *
 * @{
 */


/** @defgroup PROTOCOL_T1 T=1 protocol internal API
 *
 * @ingroup SMARTCARD_DRIVER
 *
 * This file defines the T=1 protocol API.\n
 * This is not intended to be directly used by the user.
 *
 * @{
 */

 /** @typedef   T1ProtocolState_t
  *  @brief     This declares all the T1 protocol state machine states
  */
typedef enum {
    T1_SEND_BLOCK_STATE = 0,    /**< Default state to send the first block to the card */
    T1_RECEIVE_BLOCK_STATE,     /**< Once a block has been sent, we enter this state to receive the response block*/
    T1_IS_LAST_BLOCK_STATE,     /**< check if it was the last block exchange */
    T1_SEND_RBLOCK_STATE,       /**< Send a R-Block */
    T1_SEND_BLOCK_AGAIN_STATE,  /**< In case of error, this state sends the last block once again */
    T1_SEND_WTX_RSP_STATE,      /**< Send a Wait Time eXtension response (S-Block) to the card */
    T1_SEND_IFS_RSP_STATE,      /**< Send the IFS response (S-Block) to the card */
    T1_SEND_R_M1_STATE,         /**< Acknowledge chained block (initiated by the card) */
    T1_RESYNCH_STATE,           /**< Generate a resynchronization request */
    T1_SEND_ABORT_RSP_STATE,    /**< Answer to an ABORT request */
    T1_DEACTIVATION_STATE,      /**< Transfer is done, deactivate the card */
    T1_STATE_DONE,              /**< transmit done */
} T1ProtocolState_t;


/** @fn SendT1
 *  @brief  Send a T1 APDU and get the Icc response
 *  @param [in]  slotid          Slot Id to activate
 *  @param [in]  TxBuff          APDU to send buffer
 *  @param [in]  SendDataLen     Length of the APDU to transmit
 *  @param [out] RxBuff          Receive buffer (Must be at least 2 bytes in case 1)
 *  @param [out] ReceiveDataLen  pointer on the received length
 *  @param [in]  pfSendWTE       pointer on the WTX handler (if not in EMV mode)
 *
 *
 *  Send a T=1 command (in Tx buffer) and get the response (in Rx buffer)
 */
IccReturn_t SendT1(uint8_t slotid,
                   uint8_t *TxBuff, uint32_t SendDataLen,
                   uint8_t *RxBuff,  uint32_t *ReceiveDataLen,
                   void (*fSend_WTE) (void) );


/** @} */ /*defgroup*/
/** @} */ /*file*/
#endif /*_PROTOCOL_T1_H_*/
