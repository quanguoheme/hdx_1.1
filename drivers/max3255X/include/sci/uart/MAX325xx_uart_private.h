/*
 * MAX325xx_private.h MAX32590, MAX32550 & MAX32555 Smartcard UART private driver functions
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

#ifndef _UART_MAX325xx_PRIVATE_H_
#define _UART_MAX325xx_PRIVATE_H_

#include <stdint.h>
#include "sc_errors.h"
#include "slot.h"
#include "sc_config.h"

/** @file    MAX325xx_private.h MAX32590, MAX32550 & MAX32555 Smartcard UART private driver functions
 *  @version 2.0.0
 *  @date    2015/02/13
 *  @author  Jean-Marc Simohand <jeanmarc.simohand@maximintegrated.com>
 *
 * @{
 */

/** @defgroup UART_MAX325xx_PRIVATE MAX32590, MAX32550 & MAX32555 Smartcard UART driver - private functions
 *
 * @ingroup ICC_ABSTRACTION
 *
 * This file defines UART access functions for calls from the abstraction layer.\n
 * This is not intended to be directly used by the user.
 *
 * @{
 */

/** @def MAX325xx_UART_INTERRUPT_MASK
 *  @brief Smartcard UART interrupt mask
 *
 *  this mask is used to disable all the UART interrupts except the AFE
 *  (PHY) interrupts.
 */
#define MAX325xx_UART_INTERRUPT_MASK      (0x1FF)

/** @fn UartOnCardStateChanged
 *  @brief  Notify the driver that the card status changed
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] CardState    New card state (cf #CardState_t)
 *
 *  Notify the driver that the card status changed, \n
 *  if the card has been removed or is no longer powered, the UART driver
 *  will deactivate the card and return an error code.
 */
static void  UartOnCardStateChanged(SlotContext_t  *SlotCtx, CardState_t CardState);


/** @fn     UartSetConfig
 *  @brief  set the timing parameters according to the actual protocol
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *
 *  set the ETU, EGT, WT according to the #SlotContext_t and the current protocol.
 */
static IccReturn_t UartSetConfig(SlotContext_t  *SlotCtx);

/** @fn                             UartActiveWait
 *  @brief                          Wait for a number of ETUs
 *  @param [in]  SlotCtx             Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in]  etus                number of ETUs to wait
 *
 *  Waits for a defined number of ETUs.
 */
static void UartActiveWait(SlotContext_t  *SlotCtx, uint32_t etus);

/** @fn     UartReceive
 *  @brief  Set the UART driver in Rx state
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] RxRequest    Rx parameters (pointer on the Rx buffer, expected length... cf #IccRequest_t)
 *  @return                 return an #IccReturn_t error code
 *  @reval     ICC_ERR_NULL_PTR if either the RxRequest pointer or the Rx buffer pointer is null
 *  @retval    ICC_OK           if everything went well.
 *
 *  Change the UART Driver state to Rx mode and set the Rx parameters according to the #IccRequest_t RxRequest
 */
static IccReturn_t UartReceive(SlotContext_t  *SlotCtx, IccRequest_t *RxRequest);

/** @fn                     UartGetRxLen
 *  @brief                  return the number of received bytes
 *  @param [in] SlotCtx      Slot configuration context pointer (cf #SlotContext_t)
 *  @param [out] Status      contains an #IccReturn_t error code
 *
 *  @return                 the number of bytes received.
 */
static uint16_t    UartGetRxLen(SlotContext_t  *SlotCtx, IccReturn_t *Status);

/** @fn     UartSend
 *  @brief  Send a buffer to the card
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] pData        Pointer on the Tx buffer
 *  @param [in] datalen      Length of the data to send (excluding the EDC in T=1)
 *
 *  @return                 return an #IccReturn_t error code
 *  @reval     ICC_ERR_NULL_PTR     if pData pointer is null
 *  @retval    ICC_ERR_TX_UNDERRUN  if we were interrupted during the Tx and we got an underrun.
 *  @retval    ICC_OK               if everything went well.
 *
 * Send a buffer to the card.
 * For T=1 protocol, it also append the EDC (CRC16 or LRC8) to the frame.
 */
static IccReturn_t UartSend(SlotContext_t  *SlotCtx, uint8_t *pData, uint16_t datalen);


/** @fn     UartStop
 *  @brief  Stop the UART Rx process and set the IccStatus value
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] status       The Rx status
 *
 */
static void UartStop(SlotContext_t  *SlotCtx, IccReturn_t status);


/** @fn                             UartActivate
 *  @brief                          Start the card activation
 *  @param [in]  SlotCtx            Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in]  ActivationParams    pointer on the activation timings (cf #ActivationParams_t)
 *  @param [in] RxRequest    Rx parameters (pointer on the Rx buffer, expected length... cf #IccRequest_t)
 *
 *  @return                 return an #IccReturn_t error code
 *  @reval     ICC_ERR_NULL_PTR     if pData pointer is null
 *  @retval    ICC_OK               if everything went well.
 *
 */
static IccReturn_t UartActivate(SlotContext_t  *SlotCtx, ActivationParams_t *ActivationParams, IccRequest_t *RxRequest);

/** @} */ /*defgroup*/
/** @} */ /*file*/


#endif /* _UART_MAX325xx_PRIVATE_H_*/
