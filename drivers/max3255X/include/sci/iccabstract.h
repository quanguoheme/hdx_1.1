/*
 * iccabstract.h -- Smartcard drivers abstraction layer
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

#ifndef _ICC_ABSTRACT_H_
#define _ICC_ABSTRACT_H_

#include <stdint.h>
#include "sc_errors.h"
#include "sc_states.h"
#include "slot.h"


/** @file    iccabstract.h Smartcard drivers abstraction layer
 *  @version 2.0.3
 *  @date    2015/02/12
 *  @author  Jean-Marc Simohand <jeanmarc.simohand@maximintegrated.com>
 *
 * @{
 */

/** @defgroup ICC_ABSTRACTION   UART and AFE drivers abstraction layer
 * @ingroup SMARTCARD_DRIVER
 *
 * This file defines UART and AFE access functions for upper layer calls.\n
 * This is not intended to be directly used by the user.
 *
 * @{
 */

/** @defgroup ICC_API  Smartcard drivers abstraction layer API
 *
 * This file defines UART and AFE access functions for upper layer calls.\n
 * This is not intended to be directly used by the user.
 *
 * @{
 */

/** @fn                     IccDeactivate
 *  @brief                  Stop the UART driver for the current transfer and deactivate the slot
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *
 *  @return                 return an #IccReturn_t error code
 *  @retval ICC_OK          if the AFE is now selected (enabled)
 */
IccReturn_t IccDeactivate(SlotContext_t  *SlotCtx);

/** @fn                     IccStartActivation
 *  @brief                  Power up a card
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] RxRequest    Rx parameters (pointer on the Rx buffer, expected length... cf #IccRequest_t)
 *  @return                 return an #IccReturn_t error code
 *  @retval ICC_OK          if the AFE is now selected (enabled)
 *
 *  This function enables the UART driver Rx Mode and Activate the card according \n
 *  to the voltage declared in the Slot configuration context (#SlotContext_t)
 */
IccReturn_t IccStartActivation(SlotContext_t  *SlotCtx, ActivationParams_t   *,
                                IccRequest_t *RxRequest);


/** @fn                     IccSend
 *  @brief                  Send a buffer to the card
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
IccReturn_t IccSend(SlotContext_t  *SlotCtx, uint8_t *pData, uint16_t datalen);

/** @fn                     IccReceive
 *  @brief                  Set the UART driver in Rx state
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] RxRequest    Rx parameters (pointer on the Rx buffer, expected length... cf #IccRequest_t)
 *
 *  @return                 return an #IccReturn_t error code
 *  @reval     ICC_ERR_NULL_PTR if either the RxRequest pointer or the Rx buffer pointer is null
 *  @retval    ICC_OK           if everything went well.
 *
 *  Change the UART Driver state to Rx mode and set the Rx parameters according to the #IccRequest_t RxRequest
 */
IccReturn_t IccReceive(SlotContext_t  *SlotCtx, IccRequest_t *RxRequest);


/** @fn                     IccGetRxLen
 *  @brief                  return the number of received bytes
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [out] Status      contains an #IccReturn_t error code
 *
 *  @return                 the number of bytes received.
 */
uint16_t    IccGetRxLen(SlotContext_t  *SlotCtx, IccReturn_t *Status);

/** @fn                     IccRxDone
 *  @brief                  Stop the UART Rx process
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *
 */
IccReturn_t IccRxDone(SlotContext_t  *SlotCtx);

/** @fn                     IccCheckCardState
 *  @brief                  Return the Icc state
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *
 *  @return                 return an #IccReturn_t error code
 *  @retval    ICC_OK       if the card is inserted and powered.
 *
 */
IccReturn_t IccCheckCardState(SlotContext_t  *SlotCtx);

/** @} */ /*defgroup ICC_API*/


/** @defgroup ICC_AFE_API AFE abstraction layer API
 *
 * This file defines AFE access functions for lower layer calls.\n
 * This is not intended to be directly used by the user.
 *
 * @{
 */

/** @fn                     IccSetAfeVoltage
 *  @brief                  Set the slot voltage
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] Voltage      the requested voltage (cf #IccVoltage_t)
 *
 *  @return                 return an #IccReturn_t error code
 *  @retval ICC_ERR_POWERED if the card session is already active
 *  @retval ICC_OK          if the AFE is now selected (enabled)
 *
 *  Change a card voltage.
 *  @note   The card must be powered off before changing the voltage.
 */
IccReturn_t IccSetAfeVoltage(SlotContext_t  *SlotCtx, IccVoltage_t Voltage);


/** @fn                     IccPowerAfe
 *  @brief                  Turn On/Off the card
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] PowerUp      the requested state (ON/OFF/WARM_RESET, cf #CardPowerState_t)
 *
 *  @return                 return an #IccReturn_t error code
 *  @retval ICC_ERR_POWERED if the card session is already active
 *  @retval ICC_OK          if the AFE is now selected (enabled)
 *
 */
IccReturn_t IccPowerAfe(SlotContext_t  *SlotCtx, CardPowerState_t PowerUp);


/** @fn                     IccRegisterAfe
 *  @brief                  Register an AFE in the Abstraction layer
 *  @param [in] SlotId           Slot (AFE) number
 *  @param [in] UartId               Uart (interface) number (cf #UartId_t)
 *  @param [in] SlotOps          Slot operations structure(cf #SlotData_t)
 *  @param [in] AfePrivateData   Slot private data pointer
 *
 *  @return     it returns a pointer on the Slot configuration structure (#SlotContext_t)
 *  @retval     #NULL               if an error occured
 *
 */
SlotContext_t  *IccRegisterAfe(uint8_t SlotId, UartId_t UartId,
                               SlotOps_t *SlotOps,
                               void *AfePrivateData);

/** @} */ /*defgroup ICC_AFE_API*/


/** @defgroup ICC_UART_API  Smartcard Uart abstraction layer API
 *
 * This file defines UART access functions for lower layer calls.\n
 * This is not intended to be directly used by the user.
 *
 * @{
 */


/** @fn                     IccOnCardStateChange
 *  @brief                  Card status change notifier
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] CardState    New card state (cf #CardState_t)
 *
 * This function is called by the AFE IRQ handler on a card status change: \n
 * - card removal \n
 * - card powered \n
 * - card insertion ...
 */
IccReturn_t IccOnCardStateChange(SlotContext_t  *SlotCtx, CardState_t CardState);


/** @fn                 IccUartRegister
 *  @brief              Register an UART driver in the abstraction layer
 *  @param [in]  UarId               Uart (interface) number (cf #UartId_t)
 *  @param [in]  UartOps             Uart supported operations
 *  @param [in]  UartPrivateData     Uart private data pointer
 *
 *  @return     it returns an #IccReturn_t error code
 *  @retval     ICC_ERR_NULL_PTR    if an error occured
 *
 * Register an UART driver in the Abstraction layer.\n
 * Once registered, the Abstraction layer allows access to this interface
 */
IccReturn_t  IccUartRegister(UartId_t UarId, UartOps_t *UartOps,
                                void *UartPrivateData);

/** @} */ /*defgroup ICC_UART_API*/


/** @fn                 IccSlotGetConfiguration
 *  @brief              Retrieve the Slot Configuration pointer
 *  @param [in] SlotId           Slot (AFE) number
 *
 *  @return     it returns a pointer on the Slot configuration structure (#SlotContext_t)
 *  @retval     #NULL               if an error occured (or the slot does not exist)
 *
 */
inline SlotContext_t  *IccSlotGetConfiguration(uint8_t SlotId);


/**
 * @fn          IccExchange
 * @brief       Send an APDU to the card and get the response
 * @param [in]  slotid          Slot Id to activate
 * @param [in]  TxBuff          APDU to send buffer
 * @param [in]  TxLen           Length of the APDU to transmit
 * @param [out] RxBuff          Receive buffer (Must be at least 2 bytes in case 1)
 * @param [out] RxLen           pointer on the received length
 * @param [in]  pfSendWTE       pointer on the WTX handler (if not in EMV mode)
 *
 * @return it returns ICC_OK on success or an #IccReturn_t return ocode
 */
IccReturn_t IccExchange(uint8_t slotid,
                        uint8_t *TxBuff, uint32_t TxLen,
                        uint8_t *RxBuff, uint32_t *RxLen,
                        void (*pfSendWTE)(void));


/** @fn     IccActivate
 *  @brief    Do the card activation and get the ATR
 * @param [in] slotid                 Slot Id to activate
 * @param [out] pRxBuffer             pointer on the output buffer (where the ATR will be stored)
 * @param [out] pATRLength            pointer on the ATR length byte
 * @param [in]  ActivationParams      card activation parameters (cf #ActivationParams_t)
 *
 * @return it returns ICC_OK on success
 * @return an #IccReturn_t return code on fail
 */
IccReturn_t IccActivate(uint8_t slotid,
                        uint8_t *pRxBuffer, uint32_t *pATRLength,
                        ActivationParams_t   *ActivationParams);

/** @fn                     IccPowerOff
 *  @brief                  Turn off the card without any delay
 *  @param [in] SlotCtx      Slot configuration context pointer (cf #SlotContext_t)
 *
 *  @return                 return an #IccReturn_t error code
 */
IccReturn_t IccPowerOff(SlotContext_t  *SlotCtx);

/** @fn                     IccWait
 *  @brief                  Wait for a number of ETUs (active wait)
 *  @param [in] SlotCtx      Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] etus         Number of ETUs to wait.
 */
void IccWait(SlotContext_t  *SlotCtx, uint32_t etus);

/** @fn                     SetDefaultConfig
 *  @brief                  Reset the default stack configuration
 *  @param [in] SlotCtx      Slot configuration context pointer (cf #SlotContext_t)
 */
void SetDefaultConfig(SlotContext_t *SlotCtx);
/** @} */ /*defgroup ICC_ABSTRACTION*/
/** @} */ /*file*/
#endif /*_ICC_ABSTRACT_H_*/
