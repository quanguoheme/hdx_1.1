/*
 * MAX325xx.c -- MAX32590, MAX32550 & MAX32555 Smartcard UART driver
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
#include "sc_config.h"
#include "sc_errors.h"
#include "sc_types.h"
#include "sc_regs.h"
#include "OSWrapper.h"
#include "OS_types.h"
#include "slot.h"
#include "iccabstract.h"
#include "MAX325xx_uart.h"           /*uart driver header*/
#include "MAX325xx_uart_private.h"           /*uart driver header*/
#include "MAX3255x_afe.h"

/* forward declarations */
static void UartInterrupt_Handler(SlotContext_t  *SlotCtx);

/** @var    UartPrivateData
 *  @brief  UART private data (one per interface)
 */
static UartState_t UartPrivateData[] = {
    {.UartAddress = (uintptr_t) NULL},
#if defined (MAX32590)
    /* second interface for MAX32590 (aka JIBE) */
    {.UartAddress = (uintptr_t) NULL},
#endif
};

#if defined (__MAX32590)
const uintptr_t UartPhysicalAddresses[MAX325xx_INTERFACE_NUMBER] = {
    MAX32590_SC0_BASE_ADDRESS,
    MAX32590_SC1_BASE_ADDRESS,
};
#elif defined (__MAX32550) || defined(MAX32550_B1) || defined(__MAX32555)
const uintptr_t UartPhysicalAddresses[MAX325xx_INTERFACE_NUMBER] = {
    MAX3255x_SC_BASE_ADDRESS,
};
#endif

/******************************************************************************
 *   Private functions
 */
/** @fn     UartSetWaitingTime
 *  @brief  set the waiting time (in ETUs)
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] wt           requested WT
 *
 * @note this is a internal function (not accessible to the user)
 */
static void UartSetWaitingTime(UartState_t *UartState, uint32_t wt)
{
    OSWrapper_WriteReg(UartState->UartAddress + SC_WT0R, wt);
    OSWrapper_WriteReg(UartState->UartAddress + SC_WT1R, 0);
}

/** @fn                             UartActiveWait
 *  @brief                          Wait for a number of ETUs
 *  @param [in]  SlotCtx             Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in]  etus                number of ETUs to wait
 *
 * @note the active wait cannot be used during a card exchange.
 *
 */
static void UartActiveWait(SlotContext_t  *SlotCtx, uint32_t etus)
{
    SCControl_t  sccr = {.word = 0 };  /* control register value */
    SCStatus_t   scsr  = {.word = 0 };  /* status register value */
    SCIER_t      scier = {.word = 0};
    UartState_t *UartState = NULL;


    if ((NULL == SlotCtx) ||(etus == 0)) {
        return ;
    }

    UartState = (UartState_t *)(SlotCtx->UartData->PrivateData);
    if (NULL == UartState) {
        return;
    }

    /* first be sure that the interrupts sources are disabled */
    /* disable all UART interrupts, but keep the AFE interrupts*/
    OSWrapper_ReadReg(UartState->UartAddress + SC_IER, &scier.word);
    scier.word &= ~MAX325xx_UART_INTERRUPT_MASK;
    OSWrapper_WriteReg(UartState->UartAddress + SC_IER, scier.word);

    /* read the control and interrupt status registers */
    OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);
    OSWrapper_ReadReg(UartState->UartAddress + SC_SR, &scsr.word);

    /* stop the WT/CC counter */
    sccr.bits.WTEN  = 0;
    OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

    /*clear the WTIS Flag */
    scsr.bits.WTOV = 0;
    OSWrapper_WriteReg(UartState->UartAddress + SC_SR, scsr.word);

    /*load the WT register with the number of ETUs */
    UartSetWaitingTime(UartState, etus);

    /*Start the WT counter */
    sccr.bits.WTEN  = 1;
    OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

    OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);

    /*wait until the counter expires */
    do {
         OSWrapper_ReadReg(UartState->UartAddress + SC_SR, &scsr.word);
    } while ((scsr.bits.WTOV != 1) && (ICC_OK == IccCheckCardState(SlotCtx)));

    /* stop the WT counter */
    sccr.bits.WTEN  = 0;
    OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

    /*clear the WTIS Flag */
    scsr.bits.WTOV = 0;
    OSWrapper_WriteReg(UartState->UartAddress + SC_SR, scsr.word);
}

/** @fn     UartSetGuardTime
 *  @brief  set the guard time (in ETUs)
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *
 *  @note this is a internal function (not accessible to the user)
 */
static void UartSetGuardTime(SlotContext_t  *SlotCtx)
{
    UartState_t *UartState = NULL;

     if (NULL == SlotCtx) {
        return;
    }

    if (SlotCtx->UartId >= MAX325xx_INTERFACE_NUMBER) {
        return;
    }

    UartState = (UartState_t *)(SlotCtx->UartData->PrivateData);
    if (NULL == UartState) {
        return;
    }

    OSWrapper_WriteReg(UartState->UartAddress + SC_GTR,
                      (uint32_t) SlotCtx->IccProtocolConfig.IccExtraGuardTime);
}

/** @fn     UartSetETU
 *  @brief  set the ETU duration
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *
 *  @note this is a internal function (not accessible to the user)
 */
static IccReturn_t UartSetETU(SlotContext_t  *SlotCtx)
{
    uint8_t  di = 0;
    uint16_t fi = 0;
    uint8_t  fidi = 0;
    UartState_t *UartState = NULL;

    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    if (SlotCtx->UartId >= MAX325xx_INTERFACE_NUMBER) {
        return ICC_ERR_BAD_PARAMETER;
    }

    UartState = (UartState_t *)(SlotCtx->UartData->PrivateData);
    if (NULL == UartState) {
        return ICC_ERR_NULL_PTR;
    }

    fidi = SlotCtx->IccProtocolConfig.FiDi;

    di = Di[fidi & 0x0F];
    fi = Fi[fidi >> 4];

    if ((fi % di) > (di / 2)) {
        OSWrapper_WriteReg(UartState->UartAddress + SC_ETUR,
                           SCI_ETUR_COMP | (uint32_t) ((fi / di) & 0xFFFF));
    } else {
        OSWrapper_WriteReg(UartState->UartAddress + SC_ETUR,
                           (uint32_t) ((fi / di)&0xFFFF));
    }

    return ICC_OK;
}


/** @fn     UartSetConfig
 *  @brief  set the timing parameters according to the actual protocol
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *
 *  set the ETU, EGT, WT according to the #SlotContext_t and the current protocol.
 */
static IccReturn_t UartSetConfig(SlotContext_t  *SlotCtx)
{
    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    if (SlotCtx->UartId >= MAX325xx_INTERFACE_NUMBER) {
        return ICC_ERR_BAD_PARAMETER;
    }

    UartSetETU(SlotCtx);

    UartSetGuardTime(SlotCtx);

    /*other values are dynamically set*/

    return ICC_OK;
}


/** @fn                     UartComputeEDC
 *  @brief                  Compute the EDC (LRC or CRC) of the frame
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] pData        Pointer on the Tx buffer
 *  @param [in] datalen      Length of the data to send (excluding the EDC in T=1)
 *
 *  @return                 return the LRC/CRC value
 *
 * @note No check on parameter is done !
 * @note This is an internal function  (cannot be called by the user)
 * @note CRC16 polynomial: X^12 + X^5 + 1
 */
static uint16_t UartComputeEDC(SlotContext_t  *SlotCtx,
                               uint8_t *pData, uint16_t datalen)
{
    uint16_t EDC  = 0;
    uint16_t i    = 0;
    uint16_t data = 0;

    if ((NULL == SlotCtx) || (NULL == pData)) {
        return 0;
    }

    if (SlotCtx->IccProtocolParams.IccProtocol == 0) {
        return 0;
    }

    /* Initialize EDC value */
    if (SlotCtx->IccProtocolParams.IccEDCTypeCRC == bTRUE) {
        /* CRC 16 */
        EDC = 0xFFFF;

        while (datalen--) {
            data = (uint16_t) (*pData++)<<8;

            for (i = 0; i < 8; i++) {

                if ((EDC ^ data) & 0x8000) {
                    EDC <<= 1;
                    EDC ^= 0x1021; /*0x1021 is the X^12 + X^5 + 1*/
                } else {
                    EDC <<= 1;
                }

                data <<= 1;
            }
        }
    } else {
        /*LRC */
        EDC = 0;
        for (i=0; i<datalen; i++) {
            EDC ^= *pData++;
        }
    }

    return EDC;
}

/***************************************************************************/

/** @fn UartOnCardStateChanged
 *  @brief  Notify the driver that the card status changed
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] CardState    New card state (cf #CardState_t)
 *
 *  Notify the driver that the card status changed, \n
 *  if the card has been removed or is no longer powered, the UART driver
 *  will deactivate the card and return an error code.
 */
static void  UartOnCardStateChanged(SlotContext_t  *SlotCtx, CardState_t CardState)
{
    uint8_t      AcitveSlotId = 0;

    if (NULL == SlotCtx) {
        return;
    }

    if (SlotCtx->UartId >= MAX325xx_INTERFACE_NUMBER) {
        return;
    }

    /* check if we should restore the UART context */
    AcitveSlotId = ((UartState_t *)SlotCtx->UartData->PrivateData)->ActiveSlot;
    if (AcitveSlotId != SlotCtx->SlotId) {
        /* we do not care as if this is not the current slot,
         * that means that the card status will be checked when we
         * will select this slot
         */
         return;
    }

    switch (CardState) {
        case ICC_REMOVAL:
        case ICC_FAULT:
            UartStop(SlotCtx, ICC_ERR_REMOVED);
            break;

        case ICC_INSERTION:
        default:
            break;
    }

}

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
static IccReturn_t UartReceive(SlotContext_t  *SlotCtx, IccRequest_t *RxRequest)
{
    SCControl_t  sccr  = { .word = 0 };   /* control register value */
    SCIER_t      scier = { .word = 0 };      /* interrupt enable register value*/
    SCISR_t      scisr = { .word = 0 };
    SCStatus_t   scsr  = { .word = 0 };  /* status register value */
    UartData_t   *UartData  = NULL;
    UartState_t  *UartState = NULL;

    if ((NULL == SlotCtx) ||
        (NULL == RxRequest) ||
        (NULL == RxRequest->IccData) ) {
        return ICC_ERR_NULL_PTR;
    }

    if (SlotCtx->UartId >= MAX325xx_INTERFACE_NUMBER) {
        return ICC_ERR_BAD_PARAMETER;
    }

    UartData = ((UartData_t *)SlotCtx->UartData);
    if (NULL == UartData) {
        return ICC_ERR_NULL_PTR;
    }

    /* update the Guard Time */
    UartSetGuardTime(SlotCtx);

    /*store the pointer on the Rx Request*/
    UartState = ((UartState_t *)UartData->PrivateData);
    UartState->RxRequest = RxRequest;
    RxRequest->IccReceived = 0;
    RxRequest->IccStatus = ICC_RX_PENDING;

    if (RxRequest->IccLen == 0) {
        return ICC_ERR_WRONG_LEN;
    }

    UartState->TxData.ParityError = bFALSE;

    OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);

    if (SlotCtx->IccProtocolParams.IccProtocol == 0) {
        /* T=0 Protocol
         * Enable Char repetition on parity error
         */
         sccr.bits.CREP = 1;

    } else {
        /* T=1 protocol,
         * Clear Char repetition on parity error
         * (this is managed at block level)
         */
        sccr.bits.CREP = 0;
    }
    sccr.bits.RXFLUSH = 1;
    OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

    /* clear status flags */
    scsr.word = 0;
    OSWrapper_WriteReg(UartState->UartAddress + SC_SR, scsr.word);

    /*clear interrupt flags */
    OSWrapper_ReadReg(UartState->UartAddress + SC_ISR, &scisr.word);
    scisr.word &= ~MAX325xx_UART_INTERRUPT_MASK;
    OSWrapper_WriteReg(UartState->UartAddress + SC_ISR, scisr.word);

    /* Parity interrupt is not set during the power up sequence
     * in order to check the ATR
     */
    OSWrapper_ReadReg(UartState->UartAddress + SC_IER, &scier.word);
    if (bFALSE == SlotCtx->IsPoweringUp) {
        scier.bits.PARIE = 1;
    }
    scier.bits.WTIE  = 1;
    scier.bits.CTIE  = 1;
    scier.bits.RXTIE = 1;
    scier.bits.RXFIE = 1;
    OSWrapper_WriteReg(UartState->UartAddress + SC_IER, scier.word);

    return ICC_RX_PENDING;
}


/** @fn                     UartGetRxLen
 *  @brief                  return the number of received bytes
 *  @param [in] SlotCtx      Slot configuration context pointer (cf #SlotContext_t)
 *  @param [out] Status      contains an #IccReturn_t error code
 *
 *  @return                 the number of bytes received.
 */
static uint16_t UartGetRxLen(SlotContext_t  *SlotCtx, IccReturn_t *Status)
{
    uint16_t     receivedlen = 0;
    UartData_t   *UartData   = NULL;
    UartState_t  *UartState  = NULL;
    IccRequest_t *RxRequest  = NULL;

    if (NULL == SlotCtx) {
getlen_exit_ptr:
        *Status = ICC_ERR_NULL_PTR;
        return 0;
    }

    if (SlotCtx->UartId >= MAX325xx_INTERFACE_NUMBER) {
        *Status = ICC_ERR_BAD_PARAMETER;
        return 0;
    }

    UartData = ((UartData_t *)SlotCtx->UartData);
    if (NULL == UartData) {
        goto getlen_exit_ptr;
    }

    UartState = UartData->PrivateData;
    if (NULL == UartState) {
        goto getlen_exit_ptr;
    }

    RxRequest = UartState->RxRequest;
    if ((NULL == RxRequest) ||
        (NULL == RxRequest->IccData)) {
        goto getlen_exit_ptr;
    }

    receivedlen = RxRequest->IccReceived;
    *Status     = RxRequest->IccStatus;

    return receivedlen;

}


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
static IccReturn_t UartSend(SlotContext_t  *SlotCtx, uint8_t *pData, uint16_t datalen)
{
    uint16_t    EDC    = 0;
    SCISR_t     scisr  = {.word = 0}; /* interrupt status register value */
    SCControl_t sccr   = {.word = 0};  /* control register value */
    SCStatus_t  scsr   = {.word = 0};  /* status register value */
    SCIER_t     scier  = {.word = 0};
    UartState_t *UartState = NULL;
    IccReturn_t retval     = ICC_OK;

    if ((NULL == SlotCtx) ||
        (NULL == pData)    )
        return ICC_ERR_NULL_PTR;

    if (datalen == 0) {
        return ICC_ERR_WRONG_LEN;
    }

    if (SlotCtx->UartId >= MAX325xx_INTERFACE_NUMBER) {
        return ICC_ERR_BAD_PARAMETER;
    }

    UartState = (UartState_t *)(SlotCtx->UartData->PrivateData);
    if (NULL == UartState) {
        return ICC_ERR_NULL_PTR;
    }

    /* update the ETU duration and Guard Time */
    UartSetConfig(SlotCtx);

    /* PreCompute the LRC or CRC */
    EDC = UartComputeEDC(SlotCtx, pData, datalen);

    /* firstly we must wait the end of the BGT + 10 arbitrary ETUs
     * these extra ETUs are MANDATORY for the Micropross 3150 tool to correctly
     * run the EMV campaign
     */
    UartActiveWait(SlotCtx, SlotCtx->IccProtocolConfig.IccBlockGuardTime + 10);

    /* stop the WT counter */
    OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);
    sccr.bits.WTEN = bFALSE;
    OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

    if (SlotCtx->IccProtocolParams.IccProtocol == 0) {
        /* With T=0 we set the Work Waiting time value
         * This is the maximum time allowed to the card to answer
         * it starts on the last sent byte rising edge
         */
        sccr.bits.CREP = 1;
    } else {
        /* in T=1 we set the Block waiting time, this is the maximum time
         * between the last character we sent and the first start bit of the
         * block from the card
         */
        sccr.bits.CREP = 0;
    }

    /* set the BWT or the WWT (both are stored in the same variable)
     * Note: + 10 etus when a packet is received, the interrupt occurs
     * on the stop bit (which is 10 bits after the start bit)
     */
    UartSetWaitingTime(UartState, SlotCtx->IccProtocolConfig.IccWaitingTime + 10);

    /*just to be sure, reload the Guard Time register with the CGT*/
    UartSetGuardTime(SlotCtx);

    /* flush the Tx Fifo*/
    sccr.bits.TXFLUSH = 1;
    OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

    /* clear all pending status flags */
    scsr.word = 0;
    OSWrapper_WriteReg(UartState->UartAddress + SC_SR, scsr.word);

    /*clear the Interrupt Status Flags */
    scisr.word = 0;
    OSWrapper_WriteReg(UartState->UartAddress + SC_ISR, scisr.word);

    /* enable interrupts on parity error and FIFO empty*/
    scier.bits.PARIE = 1;
    scier.bits.TXEIE = 1;
    OSWrapper_WriteReg(UartState->UartAddress + SC_IER, scier.word);

    UartState->TxData.TransmissionComplete  = bFALSE;
    UartState->TxData.ParityError           = bFALSE;
    UartState->TxData.datalen               = datalen-1;
    UartState->TxData.pdata                 = pData;
    UartState->TxData.EDC                   = EDC;
    UartState->TxData.EDCSent               = bFALSE;

    /* now fill the TX Fifo (only the first byte)*/
    OSWrapper_WriteReg(UartState->UartAddress + SC_TXR,
                       *UartState->TxData.pdata++);


    /*
     * wait until the fifo is empty
     * (and no parity issue occured)
     *
     * here we use the TX Fifo Empty interrupt (TXEIE),
     * when the TXEIE fires, we request the interrupt for the TX complete
     * (which will be fired when the last byte will be sent)
     */
        /*wait until the transmission is completed */
    while ((bFALSE == UartState->TxData.TransmissionComplete) &&
           (bFALSE == UartState->TxData.ParityError)) {
        OSWrapper_ReadReg(UartState->UartAddress + SC_SR, &scsr.word);
    }

    /* disable all smartcard interrupts */
    scier.word = 0;
    OSWrapper_WriteReg(UartState->UartAddress + SC_IER, scier.word);

    /*clear the Status Flags */
    scsr.word = 0;
    OSWrapper_WriteReg(UartState->UartAddress + SC_SR, scsr.word);

    if (bTRUE == UartState->TxData.ParityError) {
        retval =  ICC_ERR_PARITY;
        goto send_exit;
    }

    sccr.bits.WTEN = 1;
    /*Start the WT counter (WWT / BWT) */
    OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

send_exit:
    return retval;
}


/** @fn     UartStop
 *  @brief  Stop the UART Rx process and set the IccStatus value
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] status       The Rx status
 *
 */
static void UartStop(SlotContext_t  *SlotCtx, IccReturn_t status)
{
    SCIER_t       scier = {.word = 0};
    SCControl_t   sccr  = {.word = 0};
    SCStatus_t    scsr  = {.word = 0 };  /* status register value */
    UartData_t   *UartData = NULL;
    IccRequest_t *RxRequest = NULL;
    UartState_t  *UartState = NULL;

    if (NULL == SlotCtx) {
        return;
    }

    if (SlotCtx->UartId >= MAX325xx_INTERFACE_NUMBER) {
        return;
    }

    UartData = ((UartData_t *)SlotCtx->UartData);
    if (NULL == UartData)  {
        return;
    }

    UartState = UartData->PrivateData;
    if (NULL == UartState) {
        return;
    }

    RxRequest = UartState->RxRequest;
    if ((NULL == RxRequest) ||
        (NULL == RxRequest->IccData)) {
        return ;
    }

    if (((UartState_t *)UartData->PrivateData)->ActiveSlot != SlotCtx->SlotId) {
        /* we are requested to stop for an other (non active) slot,
         * just ignore !
         */
         return;
    }

    /*
     * first disable the interrupts
     * disable all UART interrupts, but keep the AFE interrupts
     */
    OSWrapper_ReadReg(UartState->UartAddress + SC_IER, &scier.word);
    scier.word &= ~MAX325xx_UART_INTERRUPT_MASK;
    OSWrapper_WriteReg(UartState->UartAddress + SC_IER, scier.word);

    OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);
    sccr.bits.WTEN    = 0; /* Stop the Waiting time counter */
    sccr.bits.CCEN    = 0; /* Stop the Cycle Counter */
    sccr.bits.TXFLUSH = 1;
    sccr.bits.RXFLUSH = 1;
    OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

    /* read the status register */
    OSWrapper_ReadReg(UartState->UartAddress + SC_SR, &scsr.word);

    if ((bTRUE == scsr.bits.PAR) &&
        (SlotCtx->IccProtocolParams.IccProtocol == 0)) {
        /*
         * We are using T=0, if the parity status flag is set,
         * that means that we received 5 bugged bytes.
         *
         * Here, we disable the UART then we deactivate the card using the
         * ICC abstraction layer in order to respect deactivation timings.
         */
        sccr.bits.UART = 0;
        OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

        /*
         * Deactivate the card without any delay
         * (to avoid a retry from the card)
         */
        IccPowerOff(SlotCtx);

        OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);
        sccr.bits.UART = 1;
        OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);
    }

    /* clear pending parity status flag (applicable to T=1 and T=0) */
    scsr.bits.PAR = 0;
    OSWrapper_WriteReg(UartState->UartAddress + SC_SR, scsr.word);

    RxRequest->IccStatus = status;

    if ((SlotCtx->IccProtocolParams.IccProtocol == 1) && (ICC_OK == status)) {

        /* compute the EDC (LRC or CRC) */
        if (SlotCtx->IccProtocolParams.IccEDCTypeCRC == bTRUE) {
            /*
             * when using the CRC16, we compute the CRC value from the
             * NAD to the last payload byte.
             */
            RxRequest->IccEDC = UartComputeEDC(SlotCtx,
                                               RxRequest->IccData,
                                               RxRequest->IccReceived-2);
        } else {
            /*
             * when using a LRC, we compute the LRC on all the bytes
             * thus, the computed value must be 0x00
             */
            RxRequest->IccEDC = UartComputeEDC(SlotCtx,
                                               RxRequest->IccData,
                                               RxRequest->IccReceived);
        }
    }
}

/** @fn     UartInterrupt_Handler
 *  @brief  Manage Uart Interrupts
 *
 */
static void UartInterrupt_Handler(SlotContext_t  *SlotCtx)
{
    SCISR_t         Status  =   {.word = 0};
    SCControl_t     sccr    =   {.word = 0};
    SCStatus_t      scsr    =   {.word = 0};
    SCCCR_t         sc_ccr  =   {.word = 0};
    SCEtu_t         sc_etu  =   {.word = 0};
    SCIER_t         scier   =   {.word = 0};
    UartData_t      *UartData  = NULL;
    UartState_t     *UartState = NULL;
    IccRequest_t    *RxRequest = NULL;

    if (NULL == SlotCtx) {
        UartStop(SlotCtx, ICC_ERR_NULL_PTR);
        goto handler_err_exit ;
    }

    if (SlotCtx->UartId >= MAX325xx_INTERFACE_NUMBER) {
        UartStop(SlotCtx, ICC_ERR_BAD_PARAMETER);
        goto handler_err_exit ;
    }

    UartData = SlotCtx->UartData;
    if (NULL == UartData)  {
        UartStop(SlotCtx, ICC_ERR_NULL_PTR);
        goto handler_err_exit ;
    }

    UartState = (UartState_t *)(UartData->PrivateData);
    if (NULL == UartState) {
        UartStop(SlotCtx, ICC_ERR_NULL_PTR);
        goto handler_err_exit ;
    }

    /* read the Interrupt flags */
    OSWrapper_ReadReg(UartState->UartAddress + SC_ISR, &Status.word);

    /* read the interrupt enable register */
    OSWrapper_ReadReg(UartState->UartAddress + SC_IER, &scier.word);

    /* read the Control Register */
    OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);

    if (bTRUE == Status.bits.RXTIS) {
        /* we received something */

        /* The interrupt is about the RX */
        RxRequest = UartState->RxRequest;
        if ((NULL == RxRequest) ||
            (NULL == RxRequest->IccData)) {
            UartStop(SlotCtx, ICC_ERR_NULL_PTR);
            /* go to the handler exit (not the handler_err_exit)
             * in order to re-enable the interrupts
             */
            goto handler_exit ;
        }
        /*at least one byte has been received */
        do {
            SCRx_t  scrxr; /* Rx register */

            /* read a byte from the FIFO */
            OSWrapper_ReadReg(UartState->UartAddress + SC_RXR, &scrxr.word);

            if (bTRUE ==  SlotCtx->IsPoweringUp) {
                RxRequest->IccReceived = 0;

                /*
                 * load the clock cycle counter with the ATR inter-byte
                 * timeout
                 */
                sccr.bits.CCEN = 0;
                OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);
                OSWrapper_ReadReg(UartState->UartAddress + SC_ETUR, &sc_etu.word);

                /* as the TS has been received, set the ATR timeout */
                sc_ccr.bits.MAN = 1;
                sc_ccr.bits.CCYC =  SlotCtx->IccProtocolConfig.IccWaitingTime * sc_etu.bits.ETU;
                OSWrapper_WriteReg(UartState->UartAddress + SC_CCR, sc_ccr.word);

                /* store the received byte in the user buffer */
                RxRequest->IccData[RxRequest->IccReceived] = scrxr.bits.DATA;

                /* check the TS byte value */
                if (RxRequest->IccData[RxRequest->IccReceived]  == 0x03) {

                    if (bFALSE == Status.bits.PARIS) {
                        /*
                         * with inverse convention, we expect
                         * a parity error on the first received
                         * byte (TS)
                         */
                        UartStop(SlotCtx, ICC_ERR_PARITY);
                        goto handler_exit;
                    }

                    /*clear the Parity bit status*/
                    Status.bits.PARIS = 0;
                    OSWrapper_WriteReg(UartState->UartAddress + SC_ISR,
                                        Status.word);

                    OSWrapper_ReadReg(UartState->UartAddress + SC_SR, &scsr.word);
                    scsr.bits.PAR = 0;
                    OSWrapper_WriteReg(UartState->UartAddress + SC_SR, scsr.word);

                    /* inverse convention detected */
                    sccr.bits.CONV = 1;
                    OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

                    SlotCtx->IccProtocolConfig.InverseConvention = bTRUE;

                    /* force the TS value to 3F */
                    RxRequest->IccData[RxRequest->IccReceived] = 0x3F;
                } else {
                    if (RxRequest->IccData[RxRequest->IccReceived] != 0x3B) {
                         /* we are in direct convention but
                          * the TS byte is not 3B--> error !
                          */
                        UartStop(SlotCtx, ICC_ERR_BAD_ATR_VALUE);
                        goto handler_exit;
                    }

                    if (bTRUE == Status.bits.PARIS) {
                        /* with direct convention, we expect no
                         * parity error on the TS byte
                         */
                        UartStop(SlotCtx, ICC_ERR_PARITY);
                        goto handler_exit;
                    }
                }

                /*
                 * now the TS byte has been received,
                 * enable the Parity Error Interrupt
                 * and start the ATR Timeout counter
                 */
                OSWrapper_ReadReg(UartState->UartAddress + SC_IER, &scier.word);
                scier.bits.PARIE = 1;
                OSWrapper_WriteReg(UartState->UartAddress + SC_IER, scier.word);

                /*
                 * Start the WT counter (the value has been set in the
                 * UartActivate function, it contains the ATR Timeout
                 * value [from TS to the TCK])
                 */
                sccr.bits.WTEN = 1;
                sccr.bits.CCEN = 1; /* next byte timeout (auto reload) */
                OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

                /*
                 * we got the first byte, we are no more in the
                 * Powering up sate
                 */
                SlotCtx->IsPoweringUp = bFALSE;

            } else {
                /* we are not powering up the card,
                 * ie we are not receiving the ATR.
                 * So, just store in the user buffer
                 */
                RxRequest->IccData[RxRequest->IccReceived] = scrxr.bits.DATA;
            }

            /* update length values */
            RxRequest->IccReceived ++;
            RxRequest->IccLen --;

            /*
             * Read the Status Register value
             * This must be done just before the end of the while loop
             * to get the most recent value of RXELT counter
             */
            OSWrapper_ReadReg(UartState->UartAddress + SC_SR, &scsr.word);

        } while ((scsr.bits.RXELT != 0) && (RxRequest->IccLen));

        if (RxRequest->IccLen == 0) {
            /* we have done  */
            RxRequest->IccLastByte = bTRUE;

            /* update the IccStatus and compute the EDC */
            UartStop(SlotCtx, ICC_OK);

        } else if ((SlotCtx->IccProtocolParams.IccProtocol == 1) &&
                   (RxRequest->IccReceived == 1)) {
            /*no need to stop the counter */

            /* set the Char Waiting Time (CWT) */
            UartSetWaitingTime(UartState,
                           SlotCtx->IccProtocolConfig.IccCharWaitingTime);

            /*Start the CWT counter */
            sccr.bits.WTEN = 1;
            OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);
        }
    } else  if ((bTRUE == Status.bits.WTIS) || (bTRUE == Status.bits.CTIS))  {
        /*if we did not received the byt but we got a timeout condition */
        /*
         * TIMEOUT during
         * - either the ATR.TS byte or
         * - or inter bytes
         * - or WWT/BWT
         * disable Rx interrupts and exit
         */
        UartStop(SlotCtx, ICC_ERR_TIMEOUT);
        goto handler_exit;
    }

    if (bTRUE == Status.bits.PARIS) {

        /*
         * as in case of Tx parity error, we will reset the UART,
         * we cannot use the SC_SR register to read the parity error bit
         */
        UartState->TxData.ParityError = bTRUE;
        OSWrapper_ReadReg(UartState->UartAddress + SC_IER, &scier.word);
        if (bFALSE == scier.bits.WTIE) {
            /* we were sending data when the parity error occured */
            /*
             * For MAX32550 A3 / MAX32555 A1 / MAX32590 B5,
             * we must disable the uart to stop the transmission
             * otherwise, the UART continues to send the byte even after
             * 5 tries.
             *
             * Here, we disable the UART then we deactivate the card using the
             * ICC abstraction layer in order to respect deactivation timings.
             */
            OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);
            sccr.bits.UART = 0;
            /* flush the Tx Fifo*/
            sccr.bits.TXFLUSH = 1;
            OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

            /* disable all smartcard interrupts */
            scier.word = 0;
            OSWrapper_WriteReg(UartState->UartAddress + SC_IER, scier.word);

            /* Deactivate the card */
            UartActiveWait(SlotCtx, SlotCtx->IccProtocolConfig.IccCharWaitingTime);
            IccPowerOff(SlotCtx);

            OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);

            /* go back to the Smartcard UART mode */
            sccr.bits.UART = 1;
            OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);
        } else {

            /*
             * Parity error during data RX
             * depending on the protocol it could be an unrecoverable issue
             *
             * Anyway, we report the issue, the upper layer will act depending
             * what is the protocol.
             */
            UartStop(SlotCtx, ICC_ERR_PARITY);
        }
        goto handler_exit;
    }

    /* check if it's a transmission interrupt */
    if (bTRUE == Status.bits.TXEIS) {
        /*
         * the FIFO is now empty,
         */
        if (UartState->TxData.datalen) {

            do {
                OSWrapper_WriteReg_nodelay(UartState->UartAddress + SC_TXR,
                                           *UartState->TxData.pdata++);
                UartState->TxData.datalen--;
                OSWrapper_ReadReg(UartState->UartAddress + SC_SR, &scsr.word);
            } while (UartState->TxData.datalen &&
                    (scsr.bits.TXELT < MAX_TXFIFO_ELT));
        } else {
            /* no more data to send
             * just add the EDC for T=1 protocol
             */
            if ((SlotCtx->IccProtocolParams.IccProtocol == 1) &&
                (bFALSE == UartState->TxData.EDCSent)) {
                UartState->TxData.EDCSent = bTRUE;

                if (bTRUE == SlotCtx->IccProtocolParams.IccEDCTypeCRC) {
                    OSWrapper_WriteReg_nodelay(UartState->UartAddress + SC_TXR,
                                               UartState->TxData.EDC>>8);
                }

                OSWrapper_WriteReg_nodelay(UartState->UartAddress + SC_TXR,
                                               UartState->TxData.EDC&0xFF);
            } else {
                /* wait for the last byte complete flag */
                scier.bits.TCIE  = 1;
                OSWrapper_WriteReg(UartState->UartAddress + SC_IER, scier.word);
            }
        }
    } else if (bTRUE == Status.bits.TCIS) {
        /* we sent the last byte */
        scier.bits.TCIE = 0;
        OSWrapper_WriteReg(UartState->UartAddress + SC_IER, scier.word);
        UartState->TxData.TransmissionComplete = bTRUE;
        goto handler_exit;
    }

    if (bTRUE == Status.bits.RXFIS) {
        /* Rx fifo full (overrun !)
         * No need to continue to receive data,
         * we know that we already missed at least a byte !
         */
        UartStop(SlotCtx, ICC_ERR_RX_OVERRUN);
        goto handler_exit;
    }

handler_exit:
    /* clear the interupt flags */
    /*keep only flags related to the uart (not the PHY)*/
    Status.word &= ~MAX325xx_UART_INTERRUPT_MASK;
    OSWrapper_WriteReg(UartState->UartAddress + SC_ISR, Status.word);

handler_err_exit:
    return ;
}


/** @fn                             UartActivate
 *  @brief                          Start the card activation
 *  @param [in]  SlotCtx            Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in]  ActivationParams   pointer on the activation timings (cf #ActivationParams_t)
 *
 *  @return    return an #IccReturn_t error code
 *  @reval     ICC_ERR_NULL_PTR     if pData pointer is null
 *  @retval    ICC_OK               if everything went well.
 *
 *  This function set the Icc voltage, powers it up and release the
 *  RST signal after an user defined number of ETUs.
 *
 */
static IccReturn_t  UartActivate(SlotContext_t  *SlotCtx,
                          ActivationParams_t *ActivationParams,
                          IccRequest_t *RxRequest)
{
    SCControl_t     sccr       = {.word = 0};  /* control register value */
    SCEtu_t         sc_etu     = {.word = 0};
    SCCCR_t         ATRTimeout = {.word = 0};
    UartState_t     *UartState = NULL;
    IccReturn_t     retval     = ICC_OK;

    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    if (SlotCtx->SlotId >= MAX3255x_SLOT_NUMBER) {
        return ICC_ERR_BAD_PARAMETER;
    }

    UartState = (UartState_t *)(SlotCtx->UartData->PrivateData);
    if (NULL == UartState) {
        return ICC_ERR_NULL_PTR;
    }


    /* update the ETU duration and Guard Time */
    UartSetConfig(SlotCtx);


    /* wait 12 etus before starting */
    UartActiveWait(SlotCtx, 12);

    retval = IccCheckCardState(SlotCtx);
    if (bTRUE == ActivationParams->IccWarmReset) {
        if (ICC_OK != retval) {
            /* the user requests a warm reset but the card is
             * either not present or not powered
             */
            return retval;
        }
    } else {
        /* set the voltage only if the card is powered off
         * (otherwise it's a manual reset release)
         */
        if (ICC_OK != retval) {
            /*
             * set the card voltage
             * note : this has no effect if the card is already powered
             * (warm reset case)
             */
            IccSetAfeVoltage(SlotCtx, ActivationParams->IccVCC);

            /*
             * Power up the card
             * note : this has no effect if the card is already powered
             * (warm reset case)
             */
            IccPowerAfe(SlotCtx, POWER_UP);
        }
    }

    /*
     * use the IsPoweringUp bit for the TS byte check during
     * the Rx handler
     */
    SlotCtx->IsPoweringUp = bTRUE;

    /*reset the card */
    IccPowerAfe(SlotCtx, RESET_DO);

    /*wait 40000 cycles (approx. 110 etus) */
    UartActiveWait(SlotCtx, ActivationParams->IccResetDuration);

    /* prepare to receive */
    UartReceive(SlotCtx, RxRequest);

    /* load the TS timeout
     * the TS timeout is the user defined value + a byte delay (10 etus)
     * (we get the interrupt on the Stop bit when the TS timeout is given
     * from the activation to the start bit
     */
    OSWrapper_ReadReg(UartState->UartAddress + SC_ETUR, &sc_etu.word);
    ATRTimeout.bits.MAN = 1;
    ATRTimeout.bits.CCYC  =  (ActivationParams->IccTS_Timeout +10);
    ATRTimeout.bits.CCYC *=  sc_etu.bits.ETU;
    OSWrapper_WriteReg(UartState->UartAddress + SC_CCR, ATRTimeout.word);

    /* get the Control register value */
    OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);

    /* disable char repetition and counters */
    sccr.bits.WTEN  = 0;
    sccr.bits.CCEN  = 0;
    sccr.bits.CREP  = 0;

    /*
     * For the ATR receiption, we are normally in direct
     * convetion mode, but the user can tweak this...
     * (at his own risks)
     */
    if (bTRUE == SlotCtx->IccProtocolConfig.InverseConvention) {
        /* set inverse convention */
        sccr.bits.CONV = 1;
    } else {
        sccr.bits.CONV = 0;
    }

    /* set the configuration */
    OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

    /*
     * set the Waiting time for each byte
     * adds 10 bits as we can only get interrupts on the stop bit (which is
     * 10 etus after the start bit
     */
    UartSetWaitingTime(UartState, SlotCtx->IccProtocolConfig.IccWaitingTime + 10);

    /*we use the WT as a temporary variable (this will be set
     * once the ATR will be analyzed)
     */
    SlotCtx->IccProtocolConfig.IccWaitingTime = ActivationParams->IccATR_Timeout;

    /* adds 10 bits as we can only get interrupts on the stop bit (which is
     * 10 etus after the start bit
     */
    SlotCtx->IccProtocolConfig.IccWaitingTime += 10;

    /*
     * Note: CTIS, WTIS, PARIS, RTIS Interrupt Flags have been cleared by
     * the UartReceive function
     */

    /* release the reset */
    IccPowerAfe(SlotCtx, RESET_RELEASE);

    /*Start the Clock Cycle counter */
    sccr.bits.CCEN  = 1;
    OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

    return ICC_OK;
}

/** @fn                     UartIoctl
 *  @brief                  process Uart interface related IOCTL
 *  @param [in]  SlotCtx    Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in]  command    IOCTL request (see #sc_ioctl_t)
 *  @param [in]  value      IOCTL parameter.
 *
 *  Initialize the UART driver.\n
 *
 *  @retval    ICC_OK                   if the driver has been registered
 *  @retval    ICC_ERR_BAD_PARAMETER    if the Interface id (uart id) is out of range
 *  @retval    ICC_ERR_NULL_PTR         if we did not get the virtual address
 *
 * @note This function is not used on bare metal. (it is kept for code
 *       compatibility with the Linux version)
 */
IccReturn_t UartIoctl(SlotContext_t  *SlotCtx, sc_ioctl_t command, uint32_t value)
{
        IccReturn_t ret = ICC_OK;
        SCPin_t     scpin= {.word = 0}; /* pin setting register value */
        UartState_t *UartState = NULL;

        UartState = (UartState_t *)SlotCtx->UartData->PrivateData;

        if ((NULL == UartState) || (NULL == (void*)UartState->UartAddress))
                return ICC_ERR_NULL_PTR;


        switch (command) {
        case IOCTL_CARD_RESET:
                OSWrapper_ReadReg(UartState->UartAddress + SC_PN,
                                 &scpin.word);

                scpin.bits.CRDRST = !!value;

                OSWrapper_WriteReg(UartState->UartAddress + SC_PN, scpin.word);
        break;

        default:
                ret = ICC_ERR_BAD_PARAMETER;
        break;
        }

        return ret;

}


/** @var    MAX325xx_UART_ops
 *  @brief  UART supported operations
 *
 *  This structure contains pointer on the supported operations.\n
 *  this will be registered in the driver in order to access to the
 *  UART operations.
 */
static const UartOps_t   MAX325xx_UART_ops = {
    .activate    = UartActivate,
    .oncardevent = UartOnCardStateChanged,
    .receive     = UartReceive,
    .getrxlen    = UartGetRxLen,
    .send        = UartSend,
    .stop        = UartStop,
    .wait        = UartActiveWait,
    .ioctl       = UartIoctl,
};

/** @fn                     UartInit
 *  @brief                  Initialize the UART driver
 *  @param [in] UartId_t    Interface number (cf #UartId_t)
 *  @param [in]  SlotCtx    Slot configuration context pointer (cf #SlotContext_t)
 *
 *  Initialize the UART driver.\n
 *
 *  @retval    ICC_OK                   if the driver has been registered
 *  @retval    ICC_ERR_BAD_PARAMETER    if the Interface id (uart id) is out of range
 *  @retval    ICC_ERR_NULL_PTR         if we did not get the virtual address
 *
 * @note On non-Linux plateforms, this must be called during the Stack initialization.
 * @note the GPIOs must be configured as Alternate function by the caller (OS-Dependant task)
 * @note the GCR.SCCK must be configured by the caller.
 */
IccReturn_t UartInit(UartId_t id, SlotContext_t  *SlotCtx)
{
    SCControl_t sccr = {.word = 0};  /* control register value */
    SCPin_t     scpin= {.word = 0}; /* pin setting register value */
    SCISR_t     scisr= {.word = 0}; /* interrupt status register value */

    if (id >= MAX325xx_INTERFACE_NUMBER) {
        return ICC_ERR_BAD_PARAMETER;
    }

    UartPrivateData[id].UartAddress = OSWrapper_Map(UartPhysicalAddresses[id]);

    if (NULL == (void *)UartPrivateData[id].UartAddress) {
        return ICC_ERR_NULL_PTR;
    }

    /* flush fifos
     * set the UART mode
     * set the convention to default (direct convention)
     * Set the RX Threshold to 1 byte
     */
     sccr.bits.UART    = 1;
     sccr.bits.WTEN    = 0;
     sccr.bits.CCEN    = 0;
     sccr.bits.CONV    = 0;
     sccr.bits.TXFLUSH = 1;
     sccr.bits.RXFLUSH = 1;
     sccr.bits.RXTHD   = 1; /*receive FIFO threshold */
     sccr.bits.TXTHD   = MAX_TXFIFO_ELT-1; /*transmit FIFO depth (not threshold !)*/
		
     OSWrapper_WriteReg(UartPrivateData[id].UartAddress + SC_CR, sccr.word);

     /* Set the default ETU value (372 Clock Cycles)*/
     OSWrapper_WriteReg(UartPrivateData[id].UartAddress + SC_ETUR, Fi[1]/Di[1]);

    /* configure the UART I/Os (C4/C8 high, Clock enabled, IO enabled) */
    scpin.bits.CLKSEL = 1;
    scpin.bits.CRDC8  = 1;
    scpin.bits.CRDC4  = 1;
    scpin.bits.CRDIO  = 1;
    OSWrapper_WriteReg(UartPrivateData[id].UartAddress + SC_PN, scpin.word);

    /*clear all pending interrupt flags*/
    scisr.word = 0;
    OSWrapper_WriteReg(UartPrivateData[id].UartAddress + SC_ISR, scisr.word);

    if (ICC_OK != IccUartRegister(id, (UartOps_t *)&MAX325xx_UART_ops,
                                  &UartPrivateData[id])) {
        return ICC_ERR_NULL_PTR;
     }

     /* register the Handler for our UART */
     OSWrapper_RegisterIRQ(SlotCtx->UartId, UartInterrupt_Handler,
                          SlotCtx, bFALSE);

     return ICC_OK;
}
