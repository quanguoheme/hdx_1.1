/*
 * iccabstract.c -- Smartcard drivers abstraction layer
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

#include <stdint.h>
#include <string.h>
#include "sc_errors.h"
#include "slot.h"
#include "sc_states.h"
#include "iccabstract.h"
#include "OSWrapper.h"
#include "ProtocolT0.h"
#include "ProtocolT1.h"
#include "ATR.h"
#include "MAX325xx_uart.h" /*for UartActiveWait*/
#include "ProtocolT1_BlockMgt.h"

/** @var        SlotDefaultConfig
 *  @brief      Default slot configuration
 */
const SlotContext_t SlotDefaultConfig = {
    .isEMV                                      = bFALSE,
    .IsPoweringUp                               = bTRUE,

    .IccProtocolParams = {
            .IccProtocol              = 0,   /*T=0*/
            .IccIFSC                  = 32, /* ISO/EMV default value*/
            .IccIFSD                  = 254,
    },

    .IccProtocolConfig = {
            .FiDi                     = 0x11,   /*Fi = 1, Di = 1*/
            .IccExtraGuardTime        = 12,     /* EGT = 12 etus */
            .IccBlockGuardTime        = BGTT0,  /* BGT = 22 etus */
            .IccCharWaitingTime       = 17,     /* CWI = 15 etus (only for T=1)*/
            .IccWaitingTime           = 10080,  /* For EMV WWT =  9600 +5% etus */
            .InverseConvention        = bFALSE, /*default is direct convention */
            .AutoPPS                  = bTRUE,  /*default is to do the PPS */
    },

    .T1 = {
        .MaxRetries                   = 0x33,
        .FirstBlock                   = bTRUE,
    },

    .T0 = {
        .FirstAPDU                    = bFALSE,
    },

};

/** @fn                     SetDefaultConfig
 *  @brief                  Reset the default stack configuration
 *  @param [in] SlotCtx      Slot configuration context pointer (cf #SlotContext_t)
 */
void SetDefaultConfig(SlotContext_t *SlotCtx)
{
        /* initialize the slot with the default configuration */
        SlotCtx->IccProtocolParams =  SlotDefaultConfig.IccProtocolParams;
        SlotCtx->IccProtocolConfig =  SlotDefaultConfig.IccProtocolConfig;
        SlotCtx->T1                =  SlotDefaultConfig.T1;
        SlotCtx->T0                =  SlotDefaultConfig.T0;
        SlotCtx->IsPoweringUp      =  SlotDefaultConfig.IsPoweringUp;
        SlotCtx->isEMV             =  SlotDefaultConfig.isEMV;

        /* then copy the default activation params (EMV values) */
        SlotCtx->ActivationParams  = IccEMVActivationParams;
}

/** @fn                     IccWait
 *  @brief                  Wait for a number of ETUs (active wait)
 *  @param [in] SlotCtx      Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] etus         Number of ETUs to wait.
 */
void IccWait(SlotContext_t  *SlotCtx, uint32_t etus)
{
    UartData_t    *Uart = NULL;

    if (NULL == SlotCtx) {
        return;
    }

    Uart = SlotCtx->UartData;
    if (NULL == Uart) {
         return;
    }

    Uart->Operations->wait(SlotCtx, etus);
}

/** @fn                     IccDeactivate
 *  @brief                  Stop the UART driver for the current transfer and deactivate the slot
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *
 *  @return                 return an #IccReturn_t error code
 *  @retval ICC_OK          if the AFE is now selected (enabled)
 *
 *  @note Uart must be stopped prior to calling this function
 */
IccReturn_t IccDeactivate(SlotContext_t  *SlotCtx)
{
    UartData_t    *Uart = NULL;
    SlotData_t    *Afe  = NULL;
    IccReturn_t  ret   = ICC_OK;

    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    Uart = SlotCtx->UartData;
    Afe = SlotCtx->AfeData;
    if ((NULL == Afe) || (NULL == Uart)) {
         return ICC_ERR_NULL_PTR;
    }

    /*check if the card is present and powered */
    ret = IccCheckCardState(SlotCtx);
    if (ICC_OK != ret) {
        return ret;
    }

    Afe->Operations->select(SlotCtx, bTRUE);

    IccGetRxLen(SlotCtx, &ret);
    /* in case of short cut between IO, RST, CLK and GND or a card mute,
     * we do not wait to deactivate
     */
    if (ICC_ERR_TIMEOUT != ret) {
        /* wait for CWT before deactivation */
        Uart->Operations->wait(SlotCtx,
                               SlotCtx->IccProtocolConfig.IccCharWaitingTime);
    }

    ret = Afe->Operations->power(SlotCtx, POWER_DOWN);

    Afe->Operations->select(SlotCtx, bFALSE);

    return ret;
}

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
IccReturn_t IccStartActivation(SlotContext_t  *SlotCtx,
                               ActivationParams_t *ActivationParams,
                               IccRequest_t *RxRequest)
{
    UartData_t    *Uart = NULL;
    SlotData_t    *Afe  = NULL;

    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    Uart = SlotCtx->UartData;
    Afe = SlotCtx->AfeData;
    if ((NULL == Afe) || (NULL == Uart)) {
         return ICC_ERR_NULL_PTR;
    }

    /*select the AFE */
    Afe->Operations->select(SlotCtx, bTRUE);

    /*start the card activation & get the ATR (DO NOT UNSELECT !)*/
    return Uart->Operations->activate(SlotCtx, ActivationParams, RxRequest);
}


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
IccReturn_t IccSend(SlotContext_t  *SlotCtx, uint8_t *pData, uint16_t datalen)
{
    UartData_t    *Uart = NULL;
    SlotData_t    *Afe  = NULL;
    IccReturn_t  ret   = ICC_OK;

    if (0 == datalen) {
        return ICC_OK;
    }

    if ((NULL == SlotCtx) || (NULL == pData)) {
        return ICC_ERR_NULL_PTR;
    }

    Uart = SlotCtx->UartData;
    Afe = SlotCtx->AfeData;
    if ((NULL == Afe) || (NULL == Uart)) {
         return ICC_ERR_NULL_PTR;
    }

    /*check if the card is present */
    ret = Afe->Operations->getcardstatus(SlotCtx);
    if (ICC_OK != ret) {
        return ret;
    }

    /* select the AFE*/
    Afe->Operations->select(SlotCtx, bTRUE);

    /*send the data to the card but DO NOT UNSELECT */
    return Uart->Operations->send(SlotCtx, pData, datalen);
}


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
IccReturn_t IccReceive(SlotContext_t  *SlotCtx, IccRequest_t *RxRequest)
{
    UartData_t   *Uart = NULL;
    SlotData_t    *Afe  = NULL;
    IccReturn_t  ret   = ICC_OK;

    if ((NULL == SlotCtx) || (NULL == RxRequest) ||
        (NULL == RxRequest->IccData)) {
        return ICC_ERR_NULL_PTR;
    }

    if (0 == RxRequest->IccLen) {
        return ICC_OK;
    }

    /* do not select the card, it's already done in the IccSend() */

    Uart = SlotCtx->UartData;
    Afe = SlotCtx->AfeData;
    if ((NULL == Afe) || (NULL == Uart)) {
         return ICC_ERR_NULL_PTR;
    }

    /*check if the card is present */
    ret = Afe->Operations->getcardstatus(SlotCtx);
    if (ICC_OK != ret) {
        return ret;
    }

    return Uart->Operations->receive(SlotCtx, RxRequest);
}


/** @fn                     IccGetRxLen
 *  @brief                  return the number of received bytes
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [out] Status      contains an #IccReturn_t error code
 *
 *  @return                 the number of bytes received.
 */
uint16_t    IccGetRxLen(SlotContext_t  *SlotCtx, IccReturn_t *Status)
{
    UartData_t   *Uart = NULL;

    if ((NULL == SlotCtx) || (NULL == Status)) {
        return (uint16_t)ICC_ERR_NULL_PTR;
    }

    Uart = SlotCtx->UartData;
    if (NULL == Uart) {
         return (uint16_t)ICC_ERR_NULL_PTR;
    }

    return Uart->Operations->getrxlen(SlotCtx, Status);

}

/** @fn                     IccRxDone
 *  @brief                  Stop the UART Rx process
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *
 */
IccReturn_t IccRxDone(SlotContext_t  *SlotCtx)
{
    UartData_t   *Uart = NULL;
    SlotData_t    *Afe  = NULL;
    UartState_t  *UartState = NULL;

    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    Uart = SlotCtx->UartData;
    Afe = SlotCtx->AfeData;
    UartState = Uart->PrivateData;

    if ((NULL == Afe) || (NULL == Uart)) {
         return ICC_ERR_NULL_PTR;
    }

    /* select the AFE*/
    Afe->Operations->select(SlotCtx, bTRUE);

    Uart->Operations->stop(SlotCtx, ICC_OK);

    /*
     * as we finished, delete the RxRequest pointer from the context
     * this avoid to keep pointer on no more available objects
     */
    UartState->RxRequest = NULL;

    /* unselect the AFE */
    Afe->Operations->select(SlotCtx, bFALSE);
    return ICC_OK;
}



/** @fn                     IccCheckCardState
 *  @brief                  Return the Icc state
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *
 *  @return                 return an #IccReturn_t error code
 *  @retval    ICC_OK       if the card is inserted and powered.
 *
 */
IccReturn_t IccCheckCardState(SlotContext_t  *SlotCtx)
{
    SlotData_t    *Afe  = NULL;

	
    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    Afe = SlotCtx->AfeData;
    if (NULL == Afe) {
         return ICC_ERR_NULL_PTR;
    }

    /* I'm not sure if we need to select or not the card...*/
    return Afe->Operations->getcardstatus(SlotCtx);
}


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
IccReturn_t IccSetAfeVoltage(SlotContext_t  *SlotCtx, IccVoltage_t Voltage)
{
    SlotData_t    *Afe  = NULL;

    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    Afe = SlotCtx->AfeData;
    if (NULL == Afe) {
         return ICC_ERR_NULL_PTR;
    }

    return Afe->Operations->setvoltage(SlotCtx, Voltage);
}

/** @fn                     IccPowerOff
 *  @brief                  Turn off the card without any delay
 *  @param [in] SlotCtx      Slot configuration context pointer (cf #SlotContext_t)
 *
 *  @return                 return an #IccReturn_t error code
 */
IccReturn_t IccPowerOff(SlotContext_t  *SlotCtx)
{
    SlotData_t    *Afe  = NULL;
    IccReturn_t   ret = ICC_OK;


    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    Afe = SlotCtx->AfeData;
    if (NULL == Afe) {
         return ICC_ERR_NULL_PTR;
    }

    /*select the AFE (normally it has already been done in the IccActivate() */
    Afe->Operations->select(SlotCtx, bTRUE);

    /*
     * on Power off
     * we reset the slot with the default configuration
     */
    ret =  Afe->Operations->power(SlotCtx, POWER_DOWN);

    SetDefaultConfig(SlotCtx);

    /*select the AFE (normally it has already been done in the IccActivate() */
    Afe->Operations->select(SlotCtx, bFALSE);

    /* then power up/down */
    return ret;
}

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
IccReturn_t IccPowerAfe(SlotContext_t  *SlotCtx, CardPowerState_t PowerUp)
{
    SlotData_t    *Afe  = NULL;
    IccReturn_t    ret = ICC_OK;

    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    Afe = SlotCtx->AfeData;
    if (NULL == Afe) {
         return ICC_ERR_NULL_PTR;
    }

    /*
     * on Power off
     * we reset the slot with the default configuration
     */
    if (POWER_DOWN == PowerUp) {
        ret =  IccDeactivate(SlotCtx);

        SetDefaultConfig(SlotCtx);

        return ret;
    }

    /*select the AFE (normally it has already been done in the IccActivate() */
    Afe->Operations->select(SlotCtx, bTRUE);

    /* then power up/down */
    return Afe->Operations->power(SlotCtx, PowerUp);
}


/** @fn                     IccOnCardStateChange
 *  @brief                  Card status change notifier
 *  @param [in] SlotCtx      Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] CardState    New card state (cf #CardState_t)
 *
 * This function is called by the AFE IRQ handler on a card status change: \n
 * - card removal \n
 * - card powered \n
 * - card insertion ...
 */
IccReturn_t IccOnCardStateChange(SlotContext_t  *SlotCtx, CardState_t CardState)
{
    UartData_t   *Uart = NULL;

    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    Uart = SlotCtx->UartData;

    if (NULL == Uart) {
         return ICC_ERR_NULL_PTR;
    }

    Uart->Operations->oncardevent(SlotCtx, CardState);

    if (SlotCtx->usr_cardstatusisr != NULL) {
        SlotCtx->usr_cardstatusisr(CardState);
    }

    return ICC_OK;
}


/** @fn                 IccSlotGetConfiguration
 *  @brief              Retrieve the Slot Configuration pointer
 *  @param [in] SlotId           Slot (AFE) number
 *
 *  @return     it returns a pointer on the Slot configuration structure (#SlotContext_t)
 *  @retval     #NULL               if an error occured (or the slot does not exist)
 *
 */
SlotContext_t  *IccSlotGetConfiguration(uint8_t SlotId)
{
    return OSWrapper_getSlotCtxbyId(SlotId);
}


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
                                void *UartPrivateData)
{

    if (ICC_OK != OSWrapper_RegisterUART(UarId, UartOps, UartPrivateData))
        return ICC_ERR_NULL_PTR;

    return ICC_OK;
}



/** @fn                     IccRegisterAfe
 *  @brief                  Register an AFE in the Abstraction layer
 *  @param [in] SlotId           Slot (AFE) number
 *  @param [in] UartId           Uart (interface) number (cf #UartId_t)
 *  @param [in] SlotOps          Slot operations structure(cf #SlotData_t)
 *  @param [in] AfePrivateData   Slot private data pointer
 *
 *  @return     it returns a pointer on the Slot configuration structure (#SlotContext_t)
 *  @retval     #NULL               if an error occured
 *
 */
SlotContext_t  *IccRegisterAfe(uint8_t SlotId, UartId_t UartId,
                               SlotOps_t *SlotOps,
                               void *AfePrivateData)
{
    SlotContext_t  *Ctx;

    if (ICC_OK != OSWrapper_RegisterSlot(SlotId, SlotOps, AfePrivateData))
        return NULL;

    Ctx = OSWrapper_getSlotCtxbyId(SlotId);
    if (NULL == Ctx) {
        return NULL;
    }

    /* initialize the slot with the default configuration */
    SetDefaultConfig(Ctx);

    /* add the UartId and the Slot Id */
    Ctx->SlotId = SlotId;
    Ctx->UartId = UartId;

    /* add the pointer on the Uart and Slot private data */
    Ctx->UartData      = OSWrapper_getUARTbyId(UartId);
    Ctx->AfeData       = OSWrapper_getSlotbyId(SlotId);


    if ((NULL == Ctx->UartData) || (NULL == Ctx->AfeData)) {
        return NULL;
    }

    return Ctx;
}

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
                        void (*pfSendWTE)(void))
{
    SlotContext_t  *Ctx;
	  IccReturn_t status = ICC_OK;

    Ctx = OSWrapper_getSlotCtxbyId(slotid);
    if (NULL == Ctx) {
        return ICC_ERR_NULL_PTR;
    }
		
   if (Ctx->IccProtocolParams.IccProtocol == 1) {
        /*T = 1*/
        status = SendT1(slotid,  TxBuff, TxLen, RxBuff, RxLen, pfSendWTE);
    } else {
        status = SendT0(slotid,  TxBuff, TxLen, RxBuff, RxLen, pfSendWTE);
    }

    // set scratchpadbuff with all zero
		OSWrapper_memdestroy(Ctx->ScratchPadBuff, T1_MAX_BLOCK_SIZE);

    return status; /*just to make the compiler happy*/
}


/** @fn       IccActivate
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
                        ActivationParams_t   *ActivationParams)
{
    SlotContext_t  *Ctx;

    Ctx = OSWrapper_getSlotCtxbyId(slotid);
    if (NULL == Ctx) {
        return ICC_ERR_NULL_PTR;
    }

    return GetATR(slotid, pRxBuffer, pATRLength, (boolean_t) Ctx->isEMV, ActivationParams);

}
