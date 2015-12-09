/*
 * PTS.c -- Smartcard protocol negociation.
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
#include "sc_errors.h"
#include "sc_types.h"
#include "slot.h"
#include "iccabstract.h"
#include "ATR.h"
#include "PTS.h"


/**
 * @var MaxAcceptedFiDi
 * @brief Declares the maximum baudrate supported byt the stack
 *
 */
const uint8_t MaxAcceptedFiDi = 0x94;

IccReturn_t IccPTSNegotiate(uint8_t slotid,
    boolean_t ChangeSpeed)
{
    IccReturn_t         TempRet = ICC_OK;
    IccReturn_t         ReturnValue = ICC_OK;
    uint8_t             DataToSend[4] = {0xFF,0x00,0x00,0x00};
    IccRequest_t        PTSRequest;
    uint8_t             ExpectedBytes = 0;
    uint8_t             ReceivedBytes = 0;
    uint8_t             fidi = 0, i = 0;
    uint8_t             pucIccPTS[6];
    uint8_t             IccProtocol;
    SlotContext_t      *SlotCtx = NULL;

    SlotCtx = IccSlotGetConfiguration(slotid);
    if (!SlotCtx) {
        return ICC_ERR_BAD_SLOT;
    }

    if (NULL == pucIccPTS) {
        return ICC_ERR_NULL_PTR;
    }

    /* Check card presence and activation */
    TempRet = IccCheckCardState(SlotCtx);
    if (TempRet != ICC_OK) {
        return TempRet;
    }

    /*
     * Create the frame to send
     * (max size is 6 bytes but only 4 bytes are used)
     *
     * First byte is "The initial Character", always 0xFF
     */

    /* Second byte is "The Format Character" */
    DataToSend[1] |= SlotCtx->IccProtocolParams.IccProtocol;

    /* The PPS1 byte will be sent, and contains FiDi value */
    DataToSend[1] |= 0x10;
    DataToSend[2] = SlotCtx->IccProtocolConfig.PossibleFiDi;

    for (i=0; i<sizeof(DataToSend)/sizeof(DataToSend[0])-1; i++) {
        DataToSend[3] ^= DataToSend[i];
    }

    IccProtocol = SlotCtx->IccProtocolParams.IccProtocol;
    SlotCtx->IccProtocolParams.IccProtocol = 0; /*back to T=0 mode */

    if (ChangeSpeed) {
        /* Check that the wished FiDi is supported or not */
        fidi = SlotCtx->IccProtocolConfig.PossibleFiDi;

        if ((Fi[fidi >> 4]/Di[fidi & 0x0F]) <
                        (Fi[MaxAcceptedFiDi >> 4]/Di[MaxAcceptedFiDi & 0x0F])) {
            /* This TA1 value is not supported
             * we stay on the default TA1 value (0x11)
             */
            SlotCtx->IccProtocolParams.IccProtocol = IccProtocol;
            return ICC_OK;
        }
    }

    /* Set the Response buffer */
    PTSRequest.IccData = pucIccPTS;
    PTSRequest.IccLen = 6; /*up to 6 bytes*/
    PTSRequest.IccLastByte = bFALSE;

    /* Send the PTS Negotiation command */
    TempRet = IccSend(SlotCtx, DataToSend, sizeof(DataToSend));
    if (TempRet != ICC_OK) {
        ReturnValue = TempRet;
        goto EndPTSFunction;
    }

    /* Change this value to be sure that received data are correct if pucIccPTS[0] = 0xFF*/
    pucIccPTS[0] = 0;

    TempRet = IccReceive(SlotCtx, &PTSRequest);
    if (TempRet != ICC_RX_PENDING) {
        ReturnValue = TempRet;
        goto EndPTSFunction;
    }

    ReceivedBytes = 0;

    /* two first bytes */
    while (ReceivedBytes < 2) {

        if ((TempRet != ICC_RX_PENDING) && (TempRet != ICC_OK)) {
            ReturnValue = TempRet;
            goto EndPTSFunction;
        }

        ReceivedBytes = IccGetRxLen(SlotCtx, &TempRet);

        /* Check if card is present and activated */
        if (IccCheckCardState(SlotCtx) != ICC_OK) {
            return ICC_ERR_REMOVED;
        }
    }

    ExpectedBytes = 2 + NB_BITS[((pucIccPTS[1] & 0xF0) >> 4)];
    ExpectedBytes++; /* For PCK */

    while (ReceivedBytes < ExpectedBytes) {

        if ((TempRet != ICC_RX_PENDING) && (TempRet != ICC_OK)) {
            ReturnValue = TempRet;
            goto EndPTSFunction;
        }

        ReceivedBytes = IccGetRxLen(SlotCtx, &TempRet);

        /* Check if card is present and activated */
        if (IccCheckCardState(SlotCtx) != ICC_OK) {
            return ICC_ERR_REMOVED;
        }
    }

    /*
     * All the data have been received,
     * turn off the Rx mode
     */

    if (TempRet == ICC_RX_PENDING) {
        TempRet = ICC_OK;
    }


EndPTSFunction:
    /* do not accept more data */
    IccRxDone(SlotCtx);

    PTSRequest.IccEDC = 0;
    for (i=0; i<ReceivedBytes; i++) {
        PTSRequest.IccEDC ^= pucIccPTS[i];
    }

    /* Check the LRC */
    if (PTSRequest.IccEDC != 0) {
        return ICC_ERR_BAD_EDC;
    }

    /* the FiDi to set eventually */
    ReceivedBytes = SlotCtx->IccProtocolConfig.PossibleFiDi;


    /*
     * Check response
     * The first byte must be 0xFF
     */
    if (pucIccPTS[0] == 0xFF) {
        /*
         * Check PPS0
         * Protocol is OK if bits b1 to b4 are echoed
         */
        if ((pucIccPTS[1] & 0x0F) != (DataToSend[1] & 0x0F)) {
            /* not the same protocol in the PPS0_answer */
            return ICC_ERR_PTS_NEGOTIATION;
        }
        else if ((pucIccPTS[1] & 0xE0) != (DataToSend[1] & 0xE0)) {
            /* b7, b6 or b5 have different values */
            return ICC_ERR_PTS_NEGOTIATION;
        } else {
            if ((pucIccPTS[1] & 0x10) != (DataToSend[1] & 0x10)) {
                /* bit b5 is different (no PPS0 byte)*/
                /* new Speed value is default */
                ReceivedBytes = 0x11;
            } else {
                /* Fi/Di values present, check if the same as expected */
                if (pucIccPTS[2] != DataToSend[2]) {
                    return ICC_ERR_PTS_NEGOTIATION;
                }
            }
        }
    }

    if (TempRet == ICC_OK) {
        /* Change speed settings */
        if (ChangeSpeed) {
            /* Change the communication speed */
            SlotCtx->IccProtocolConfig.FiDi = ReceivedBytes;
            SlotCtx->InitialParams.Di = Di[ReceivedBytes & 0x0F];
        }
    }

    SlotCtx->IccProtocolParams.IccProtocol = IccProtocol;

    return ReturnValue;
}

