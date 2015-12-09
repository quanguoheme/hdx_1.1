/*
 * ProtocolT0.c -- Send/Receive APDU using T=0 protocol
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
#include "iccabstract.h"
#include "ProtocolT0.h"
#include "ProtocolT0_private.h"
#include "ProtocolT1_BlockMgt.h" /* for the T1_MAX_BLOCK_SIZE define*/
#include "OSWrapper.h"      /*for memdestroy*/

/** @def min
 *  @brief return the minimum value
 *  @param [in]  x       first integer to compare
 *  @param [in]  y       second integer to compare
 */
#define min(x,y)        ((x>y?y:x))


/**
 * @var GetResponseTPDU
 * @brief Get Response TPDU
 *
 * The GetResponseTPDU is used as a template for the Get Response TPDU.
 * the Lc field has to be changed with the expected data length from the Card.
 */
const  uint8_t  GetResponseTPDU[5] = {0x00,0xC0,0x00,0x00,0x00};

/**
 * @var    CmdGetResponse
 * @brief  T=0 Get Response command buffer
 */
static uint8_t  CmdGetResponse[5];

IccReturn_t SendT0(uint8_t slotid,
                   uint8_t *TxBuff, uint32_t TxLen,
                   uint8_t *RxBuff, uint32_t *RxLen,
                   void (*pfSendWTE) (void))
{
    uint8_t IccSW1 = 0x00,
            IccSW2 = 0x00,
            TempSW1 = 0x00,
            TempSW2 = 0x00;
    uint8_t *RxPtr;
    uint16_t       LcLen, LeLen;
    IccRequest_t   RxRequest;
    IccReturn_t    RetCode = ICC_OK;
    SlotContext_t  *SlotCtx;

    SlotCtx = IccSlotGetConfiguration(slotid);
    if (NULL == SlotCtx) {
        return ICC_ERR_BAD_SLOT;
    }


    if ((NULL == TxBuff) || (NULL == RxBuff) || (NULL == RxLen)) {
        return ICC_ERR_NULL_PTR;
    }

    /* stores the class and instruction bytes */
    SlotCtx->T0.IccCLA = TxBuff[0];
    SlotCtx->T0.IccINS = TxBuff[1];

    SlotCtx->T0.IccT0Case = T0DetectCmdCase(TxBuff, TxLen, &LcLen, &LeLen);

    if (SlotCtx->T0.IccT0Case == CASE_UNKNOWN)
        return ICC_ERR_WRONG_LEN;
		
		/* Initialize the reception buffer pointer */
    RxPtr                  = &RxBuff[0];

    /*
     * make the Receive structure to points on the temporary Rx Buffer
     * as we do not know the length to receive, set it to an arbitrary number.
     */
    RxRequest.IccData      = SlotCtx->ScratchPadBuff;
    RxRequest.IccLen       = MAXDATALEN;
    RxRequest.IccLastByte  = bFALSE;
    SlotCtx->T0.FirstAPDU  = bFALSE;

    /* Reset received data length */
    *RxLen = 0;

		
    /* restore default GetResponse TPDU */
    memcpy(CmdGetResponse, GetResponseTPDU, sizeof(CmdGetResponse));

		switch (SlotCtx->T0.IccT0Case)
    {
        case CASE_1:
                RetCode = T0Case1 (SlotCtx, TxBuff, &RxRequest,
				   SlotCtx->ScratchPadBuff,
                                   &RxPtr, RxLen, &IccSW1, &IccSW2 );
                break;

        case CASE_2:
                RetCode = T0Case2 (SlotCtx, TxBuff, &RxRequest, LeLen,
                                    SlotCtx->ScratchPadBuff, T0_MAX_APDU_SIZE,
                                    &RxPtr, RxLen,
                                    &IccSW1, &IccSW2, pfSendWTE);
                break;

        case CASE_3:
                RetCode = T0Case3 (SlotCtx, TxBuff, &RxRequest,
                                   SlotCtx->ScratchPadBuff, &RxPtr, RxLen,
                                   &IccSW1, &IccSW2, pfSendWTE);
                break;

        case CASE_4:
                /* Case 4 is a Case 3 followed by Case 2 */
                SlotCtx->T0.IccT0Case = CASE_3;
                TempSW1 = 0;
                TempSW2 = 0;
                RetCode = T0Case3 (SlotCtx, TxBuff, &RxRequest,
                                   SlotCtx->ScratchPadBuff,
                                   &RxPtr, RxLen,
                                   &IccSW1, &IccSW2,
                                   pfSendWTE);
                if (RetCode != ICC_OK) {
                    goto SendT0_exit;
                }

                if (IccSW1 == 0x61) {
                    CmdGetResponse[0] = SlotCtx->T0.IccCLA;
                    if (bTRUE == SlotCtx->isEMV) {
                        /* EMV spec */
                        CmdGetResponse[4] = IccSW2;
                    }
                    else {
                        /* ISO spec */
                        CmdGetResponse[4] = min ( (uint8_t) LeLen, IccSW2 );
                    }

                } else  if ((bTRUE == SlotCtx->T0.FirstAPDU) &&
                            (SlotCtx->isEMV == bTRUE) &&
                            ( (IccSW1 == 0x62) ||
                              (IccSW1 == 0x63) ||
                              ((IccSW1 & 0x9F) > 0x90) ) ) {
                        TempSW1 = IccSW1;
                        TempSW2 = IccSW2;
                        CmdGetResponse[4] = 0;
                } else
                    break;

                /*received length */
                *RxLen = 0;
                /* INS byte is now 0xC0 (get response) */
                SlotCtx->T0.IccINS = CmdGetResponse[1];
                memcpy(TxBuff, CmdGetResponse, (uint16_t) 5);
                SlotCtx->T0.IccT0Case = CASE_2;

                RetCode = T0Case2 ( SlotCtx, TxBuff, &RxRequest,
                                    CmdGetResponse[4],
                                    SlotCtx->ScratchPadBuff,
                                    T0_MAX_APDU_SIZE,
                                   &RxPtr, RxLen, &IccSW1, &IccSW2, pfSendWTE);

                if (TempSW1 != 0) {
                    IccSW1 = TempSW1;
                    IccSW2 = TempSW2;
                }
                break;

        default:
                IccDeactivate(SlotCtx);
                RetCode = ICC_ERR_BAD_PROCEDURE_BYTE;
                goto SendT0_exit;
    }

    if (RetCode != ICC_OK) {
        goto SendT0_exit;
    }

    /* append the status bytes to the caller buffer */
    *RxPtr++ = IccSW1;
    *RxPtr   = IccSW2;

    /* Increment received buffer length */
    *RxLen   += 2;


SendT0_exit:
    OSWrapper_memdestroy(SlotCtx->ScratchPadBuff, T0_MAX_APDU_SIZE);
    return RetCode;

}



IccReturn_t T0Case1(SlotContext_t *SlotCtx, uint8_t *TxBuff,
                    IccRequest_t  *RxRequest,
                    uint8_t *ScratchPadBuff, uint8_t **RxPtr, uint32_t *RxLen,
                    uint8_t *pSW1, uint8_t *pSW2)
{
    IccReturn_t RetCode = ICC_OK;
    uint16_t CurrentByteIndex = 0;

    if ((NULL == SlotCtx) || (NULL == ScratchPadBuff) || (NULL == RxPtr) ||
        (NULL == RxLen) || (NULL == pSW1) || (NULL == pSW2)) {
        return ICC_ERR_NULL_PTR;
    }

    /* send the header with Lc set to 0*/
    RetCode = T0TransmitHeader(SlotCtx, TxBuff, 0x00);
    if (RetCode != ICC_OK) {
        return RetCode;
    }

    /*
     *  Header transmission done.
     *
     *  Now expect the procedure byte and/or SW1 byte
     */
    do {
        RetCode = T0GetProcByte(SlotCtx, RxRequest);

        if (RetCode != ICC_OK)
            break;

    } while ( (ScratchPadBuff[0] == 0x60) || (ScratchPadBuff[0] == SlotCtx->T0.IccINS) );

    CurrentByteIndex++;

    /* Waiting for SW2 byte */
    RetCode = T0GetMoreRxBytes(SlotCtx, CurrentByteIndex+1, CurrentByteIndex,
                               bFALSE, ScratchPadBuff, RxPtr, RxLen);
    if (RetCode != ICC_OK) {
        return RetCode;
    }

    *RxLen = 0;
    RetCode = IccRxDone(SlotCtx);


    if (*RxLen > 2) {
        return ICC_ERR_WRONG_LEN;
    }

    /* store the status byte before exiting */
    *pSW1 = ScratchPadBuff[0];
    *pSW2 = ScratchPadBuff[1];

    return RetCode;

}

IccReturn_t T0Case2 ( SlotContext_t *SlotCtx, uint8_t *TxBuff,
                      IccRequest_t  *RxRequest,
                       uint16_t LeLEN, uint8_t *ScratchPadBuff,
                       uint16_t MaxBuffSize, uint8_t **RxPtr, uint32_t *RxLen,
                       uint8_t *pSW1, uint8_t *pSW2,
                       void (*pfSendWTE) (void))
{
    IccReturn_t RetCode = ICC_OK;
    uint16_t TotalBytes;
    uint16_t CurrentBytesExpected = 0;  /*total data bytes expected to receive*/
    uint16_t CurrentByteIndex = 0;          /*total bytes received*/
    uint16_t DontCareBytes = 0;             /* ~INS/INS/NULL bytes */
    uint16_t i;

    boolean_t ResetRx = bTRUE;   /* Flag to reset all variables */
    boolean_t AnalyzeProcByte = bFALSE;  /* Flag to analyze proc byte AFTER a byte has been received */

    if ((NULL == SlotCtx) || (NULL == ScratchPadBuff) || (NULL == RxPtr) ||
        (NULL == RxLen) || (NULL == pSW1) || (NULL == pSW2)) {
        return ICC_ERR_NULL_PTR;
    }

    if ( (LeLEN > 0) && (LeLEN <= 256) ) {
        CurrentBytesExpected = LeLEN;
    } else if ( (LeLEN == 0) || (LeLEN > 256) ) {
        /* per ISO 7816-4 */
        LeLEN = 0;
        CurrentBytesExpected = 256;
    }

    /*
     * TotalBytes is used to keep track of total bytes to be received
     * and to make sure it is not exceeding MaxBuffSize
     */
    TotalBytes = 0;
    CurrentBytesExpected += 2;  /*add the two status bytes (SW1SW2 to expected byte count)*/
    RetCode = T0TransmitHeader(SlotCtx, TxBuff, (uint8_t) LeLEN);

    if (RetCode != ICC_OK) {
         return RetCode;
    }

    /*
     * The command header has been sent,
     * now we expect the procedure byte and/or SW1 byte
     */
    do
    {
        RxRequest->IccLen      = CurrentBytesExpected+1;
        RxRequest->IccEDC      = 0;
        /*
         * Reset all index and
         * get the first procedure byte
         */
        if (ResetRx)
        {
            ResetRx = bFALSE;
            DontCareBytes = 0;
            CurrentByteIndex = 0;

            RetCode = T0GetProcByte(SlotCtx, RxRequest);

            if (RetCode != ICC_OK) {
                return RetCode;
            }
            CurrentByteIndex++;
        }

        /*
         * Analyze the procedure byte
         * 0x60 is a time extension request
         */
        if ( ScratchPadBuff[CurrentByteIndex - 1] == 0x60)
        {
            ResetRx = bTRUE;
            AnalyzeProcByte = bFALSE;

            if (NULL != pfSendWTE) {
                (*pfSendWTE) ( );
            }
        }
        else
        if ((ScratchPadBuff[CurrentByteIndex - 1]^SlotCtx->T0.IccINS) == 0x00 ) {
            if (CurrentBytesExpected > (CurrentByteIndex - DontCareBytes) ) {
                /*Get all data bytes and store in caller buffer */
                DontCareBytes++;

                /*
                 * Determine # of data bytes left to receive
                 * (excluding SW1SW2 bytes)
                 */
                i = CurrentBytesExpected - (CurrentByteIndex - DontCareBytes) - 2;
                TotalBytes += i; /*Keep track of input size to make sure it
                                  *will not exceed MaxBuffSize
                                  */
                AnalyzeProcByte = bTRUE;
                ResetRx = bFALSE;

                /*
                 * Expected bytes (without SW1SW2) + ~INS/INS/NULL
                 * (DontCareBytes)
                 */
                RetCode = T0GetMoreRxBytes(SlotCtx, CurrentByteIndex + i,
                                           CurrentByteIndex, bTRUE,
                                           ScratchPadBuff, RxPtr, RxLen);

                if (RetCode != ICC_OK) {
                    return RetCode;
                }

                CurrentByteIndex += i;
            }
        }
        else
        if ((ScratchPadBuff[CurrentByteIndex - 1]^SlotCtx->T0.IccINS) == 0xFF ) {
            if (CurrentBytesExpected > (CurrentByteIndex - DontCareBytes) ) {
                /*Get one data byte and store it*/
                RetCode = T0GetMoreRxBytes(SlotCtx, CurrentByteIndex + 1,
                                           CurrentByteIndex, bTRUE, ScratchPadBuff,
                                           RxPtr, RxLen);
                if (RetCode != ICC_OK) {
                    return RetCode;
                }

                CurrentByteIndex++;
                DontCareBytes++;
                TotalBytes++;
                AnalyzeProcByte = bTRUE;
                ResetRx = bFALSE;
            }
        }
        else
        if ( ScratchPadBuff[CurrentByteIndex - 1] == 0x6C)
        {
            /*
             * Wrong length status returned by tge card
             * Get SW2 byte for correct length
             */
            RetCode = T0GetMoreRxBytes(SlotCtx, CurrentByteIndex + 1,
                                       CurrentByteIndex, bFALSE, ScratchPadBuff,
                                       RxPtr, RxLen);
            if (RetCode != ICC_OK) {
                return RetCode;
            }

            CurrentByteIndex++;

            IccRxDone(SlotCtx);
            /* retransmit header with correct length */
            RetCode = T0TransmitHeader(SlotCtx, TxBuff, ScratchPadBuff[CurrentByteIndex - 1]);

            if (RetCode != ICC_OK) {
                return RetCode;
            }

            /*Must include SW1SW2 bytes into byte counts */
            CurrentBytesExpected = ScratchPadBuff[CurrentByteIndex - 1] + 2;
            if ( (TotalBytes + CurrentBytesExpected) > MaxBuffSize) {
                return ICC_ERR_WRONG_LEN;
            }

            /*Reset index to continue while loop */
            CurrentByteIndex = 0;
            ResetRx = bTRUE;
            AnalyzeProcByte = bFALSE;
        }
        else
        if ( ScratchPadBuff[CurrentByteIndex - 1] == 0x61) /*get response  command */
        {
            /*Get SW2 byte to determine Len for get response command */
            RetCode = T0GetMoreRxBytes (SlotCtx, CurrentByteIndex + 1,
                                        CurrentByteIndex, bFALSE, ScratchPadBuff,
                                        RxPtr, RxLen);
            if (RetCode != ICC_OK) {
                return RetCode;
            }

            RetCode = IccRxDone(SlotCtx);

            if ( ScratchPadBuff[CurrentByteIndex] == 0x00 )
            {
                /* Per ISO */
                CurrentBytesExpected = 256 + 2;
                /* Temporary Length value for the GetResponse command */
                i = 0;
            }
            else
            {
                /* Must include SW1SW2 bytes into byte counts */
                CurrentBytesExpected = ScratchPadBuff[CurrentByteIndex] + 2;
                /* Temporary Length value for the GetResponse command */
                i = ScratchPadBuff[CurrentByteIndex];
            }

            /*
             *  Need to make sure MaxBuffSize <= total byte expected
             * (make sure that the RxBuff size is big enough)
             */
            if ((TotalBytes + CurrentBytesExpected)  > MaxBuffSize) {
                    return ICC_ERR_WRONG_LEN;
            }

            /*Reset CurrentByteIndex to continue looping */
            CurrentByteIndex = 0;

            /* INS byte now is the get response CMD (0xC0) */
            SlotCtx->T0.IccINS = CmdGetResponse[1];

            /* retain the same CLA byte as original CMD */
            //~ CmdGetResponse[0] = SlotCtx->T0.IccCLA;
            CmdGetResponse[0] = 0;
            CmdGetResponse[4] = i;

            /* Transmit header with correct Len */
            RetCode = IccSend(SlotCtx, CmdGetResponse, 5 );

            if (RetCode != ICC_OK) {
                return RetCode;
            }

            ResetRx = bTRUE;
            AnalyzeProcByte = bFALSE;
        }
        else if ( ( ( ScratchPadBuff[CurrentByteIndex - 1] & 0xF0) == 0x60) ||
                  ( ( ScratchPadBuff[CurrentByteIndex - 1] & 0xF0) == 0x90) ) {
                /* Get SW2 byte */
                RetCode = T0GetMoreRxBytes(SlotCtx,  CurrentByteIndex + 1,
                                           CurrentByteIndex, bFALSE,ScratchPadBuff,
                                           RxPtr, RxLen);
                if (RetCode != ICC_OK) {
                    return RetCode;
                }

                /* Stop the Rx */
                RetCode = IccRxDone(SlotCtx);

                /* prepare the status bytes to return */
                *pSW1 = ScratchPadBuff[CurrentByteIndex - 1];
                *pSW2 = ScratchPadBuff[CurrentByteIndex];
                break;
        } else {
                IccDeactivate(SlotCtx);
                return ICC_ERR_BAD_PROCEDURE_BYTE;
        }

        if ( (bFALSE == ResetRx) && (bTRUE == AnalyzeProcByte) ) {
            /* Get SW2 byte */
            RetCode = T0GetMoreRxBytes(SlotCtx, CurrentByteIndex + 1,
                                       CurrentByteIndex, bFALSE, ScratchPadBuff,
                                       RxPtr, RxLen);
            if (RetCode != ICC_OK) {
                return RetCode;
            }

            CurrentByteIndex++;
        }
    } while ( CurrentByteIndex < (CurrentBytesExpected + DontCareBytes) );


    return RetCode;

}

IccReturn_t T0Case3 ( SlotContext_t *SlotCtx,uint8_t *TxBuff,
                      IccRequest_t  *RxRequest,
                      uint8_t *ScratchPadBuff, uint8_t **RxPtr, uint32_t *RxLen,
                      uint8_t *pSW1, uint8_t *pSW2,  void (*pfSendWTE) (void))
{
    IccReturn_t RetCode = ICC_OK;
    uint16_t    CurrentTxIndex = 0, CurrentRxIndex = 0;
    uint16_t    BytesRemain = 0;
    uint8_t     *Tmp_TxPtr = NULL;
    boolean_t   ResetRx = bTRUE;

    if ((NULL == SlotCtx) || (NULL == ScratchPadBuff) || (NULL == RxPtr) ||
        (NULL == RxLen) || (NULL == pSW1) || (NULL == pSW2)) {
        return ICC_ERR_NULL_PTR;
    }

    /*Move index pointer to the first data byte*/
    if ( SlotCtx->T0.IccT0Case == CASE_3 )
        CurrentTxIndex = 5;
    else
        CurrentTxIndex = 7; /*case 4*/

    /* initialize the Tx pointer*/
    Tmp_TxPtr = &TxBuff[CurrentTxIndex];

    /*clear the status word */
    *pSW1 = 0;
    *pSW2 = 0;

    BytesRemain = TxBuff[CurrentTxIndex - 1];

    SlotCtx->T0.FirstAPDU = bFALSE;

    RetCode = T0TransmitHeader (SlotCtx, TxBuff, BytesRemain);
    if (RetCode != ICC_OK) {
        return RetCode;
    }

    do {
        if ( ResetRx ) {
            RetCode = T0GetProcByte(SlotCtx, RxRequest);
        } else {
            /* Get SW2 byte */
            RetCode = T0GetMoreRxBytes(SlotCtx, CurrentRxIndex + 1,
                                       CurrentRxIndex, bFALSE , ScratchPadBuff,
                                       RxPtr, RxLen);
        }

        if (RetCode != ICC_OK) {
            return RetCode;
        }

        ResetRx = bFALSE;
        CurrentRxIndex++;


        /* check if we got a Wait Time eXtension (WTX) (NULL PCB) */
        if (ScratchPadBuff[CurrentRxIndex - 1] == 0x60) {
            ResetRx = bFALSE;

            if (NULL != pfSendWTE) {
                (*pfSendWTE) ( );
            }

            /* we must reset Rx since it is going
             * beyond the maximum buffer size
             */
            if (CurrentRxIndex >= MAXDATALEN) {
                ResetRx = bTRUE;
                CurrentRxIndex = 0;
            }

        } else if ((ScratchPadBuff[CurrentRxIndex - 1]^SlotCtx->T0.IccINS) == 0xFF) {
            /* the card answered with ~INS */

            RetCode = IccRxDone(SlotCtx);
            if (BytesRemain > 0) {
                RetCode = IccSend(SlotCtx, Tmp_TxPtr, 1);
                if (RetCode != ICC_OK) {
                    return RetCode;
                }

                SlotCtx->T0.FirstAPDU = bTRUE;
                Tmp_TxPtr++;
                BytesRemain--;
                ResetRx = bTRUE;
                CurrentRxIndex = 0;
            } else {
                IccDeactivate(SlotCtx);
                return ICC_ERR_BAD_PROCEDURE_BYTE;
            }

        } else if ((ScratchPadBuff[CurrentRxIndex - 1]^SlotCtx->T0.IccINS) == 0x00) {
                /* the card answered with INS */

            if (BytesRemain > 0) {
                SlotCtx->T0.FirstAPDU = bTRUE;
                IccRxDone(SlotCtx);
                RetCode = IccSend(SlotCtx, Tmp_TxPtr, BytesRemain);
                if (RetCode != ICC_OK)
                    return RetCode;

                BytesRemain = 0;  /*All bytes have been sent*/
                ResetRx = bTRUE;
                CurrentRxIndex = 0;
            } else {
                IccDeactivate(SlotCtx);
                return ICC_ERR_BAD_PROCEDURE_BYTE;
            }

        } else if ( ( (ScratchPadBuff[CurrentRxIndex - 1] & 0xF0) == 0x60) ||
                  ( (ScratchPadBuff[CurrentRxIndex - 1] & 0xF0) == 0x90)   ) {

            /* Store SW1 byte */
            *pSW1 = ScratchPadBuff[CurrentRxIndex - 1];

            /* Get SW2 byte */
            RetCode = T0GetMoreRxBytes(SlotCtx, CurrentRxIndex + 1,
                                       CurrentRxIndex, bFALSE , ScratchPadBuff,
                                       RxPtr, RxLen);
            if (RetCode != ICC_OK) {
                return RetCode;
            }

            RetCode =  IccRxDone(SlotCtx);
            *pSW2 = ScratchPadBuff[CurrentRxIndex];
            break;

        } else { /* Anything else is a bad procedure byte */

            IccDeactivate(SlotCtx);
            return ICC_ERR_BAD_PROCEDURE_BYTE;
        }

    } while (1); /*( BytesRemain >= 0 );*/

    return RetCode;

}


IccReturn_t T0GetProcByte(SlotContext_t  *SlotCtx,
                          IccRequest_t  *RxRequest)
{
    IccReturn_t RetCode = ICC_OK;
    uint8_t     NbReceivedBytes = 0;

    if ((NULL == SlotCtx) || (NULL == RxRequest)) {
        return ICC_ERR_NULL_PTR;
    }

    /* prepare to receive data from the card */
    RetCode = IccReceive(SlotCtx, RxRequest);

    if ((RetCode == ICC_RX_PENDING) || (RetCode == ICC_OK)) {
        do {
            NbReceivedBytes = IccGetRxLen(SlotCtx, &RetCode);

            /* if status is not pending an error occurred */
            if ((RetCode != ICC_RX_PENDING) && (RetCode != ICC_OK)) {
                return RetCode;
            }

            /* Check for card status */
            if (IccCheckCardState (SlotCtx) != ICC_OK) {
                return RetCode;
            }

        } while ( NbReceivedBytes <= 0 ); /*Loop until get a byte*/
    } else {
        return RetCode;
    }

    return ICC_OK;
}

IccReturn_t T0GetMoreRxBytes (SlotContext_t *SlotCtx,
                              uint16_t FinalOffset,
                              uint16_t InitialOffset, boolean_t SaveByte,
                              uint8_t *ScratchPadBuff,
                              uint8_t **RxPtr, uint32_t *RxLen)
{

    uint16_t l = 0, i = 0;
    IccReturn_t RetCode = ICC_OK;

    if ((NULL == SlotCtx) || (NULL == RxPtr)   ||
        (NULL == RxLen) || (NULL == ScratchPadBuff)) {
        return ICC_ERR_NULL_PTR;
    }


    i = InitialOffset;
    l = InitialOffset;

    while ( l < FinalOffset ) {

        l = IccGetRxLen(SlotCtx, &RetCode);

        /* if status is not pending an error */
        if ((RetCode != ICC_RX_PENDING) && (RetCode != ICC_OK)) {
                return RetCode;
        }

        /* Check for card status */
        if (IccCheckCardState (SlotCtx) != ICC_OK) {
                return ICC_ERR_REMOVED;
        }

        if (SaveByte) {
            /*store data in permanent buffer while accepting characters*/
            if ( i < l)
            {
                /*some byte(s) has just arrived, save it.*/
                **RxPtr = ScratchPadBuff[i++];
                (*RxPtr)++;
                *RxLen = (*RxLen) +1;
            }
        }
    }

    /*Continue storing bytes until reaching end of receiving buffer*/
    if (SaveByte) {
        while ( i < FinalOffset  )
        {
            **RxPtr = ScratchPadBuff[i++];
            (*RxPtr)++;
            *RxLen = (*RxLen) +1;
        }
    }

    if (RetCode == ICC_RX_PENDING) {
        RetCode = ICC_OK;
    }
    return RetCode;
}


Case_t T0DetectCmdCase(uint8_t *TxBuff, uint16_t TxLen,
                       uint16_t *LcLen, uint16_t *LeLen)
{
    uint16_t bLc = 0;
    uint16_t Len = 0;

    if ((NULL == TxBuff) || (NULL == LcLen) || (NULL == LeLen)) {
        return CASE_UNKNOWN;
    }

    *LcLen = 0; /* Initialize Transmit and receive data length */
    *LeLen = 0;

    /*
     * TxLen contains the length of the command
     * Minimum bytes are: CLA INS P1 P2
     */
    if (TxLen < 4) {
        return CASE_UNKNOWN;
    }

    /* Case 1 command */
    if (TxLen == 4) {
        return CASE_1;
    }

    /* TxBuff contains at least 5 bytes */
    bLc = (uint16_t) TxBuff[4];

    /* Case 2 command */
    if (TxLen == 5) {
        /* Set the receive DataLen */
        *LeLen = (uint16_t) TxBuff[4];
        return CASE_2;
    }

    /* Determine datalen for both short and extended cases
     * B1 B2 B3 - B1 between 1 and 255 indicates short cases
     * (Extended case are not supported)
     */
    if ( (bLc == 0x00) && (TxLen > 6) ) {
        /*CLA INS P1 P2 P3=B1B2B3 data...*/
        /* bLc = 0x00 - per ISO,
         * bLc = 0xFF - per JICSAP
         */
        Len = (uint16_t) TxBuff[5];
        Len = (Len << 8) | (uint16_t) TxBuff[6];
    } else {
        if (bLc > 0x00)
            Len = (uint16_t) TxBuff[4];
        else
            Len = 0;
    }

    if ( (Len < 0x0100) ) {
        /* Case 3 - CLA INS P1 P2 Lc>0 [Lc data bytes] */
        if (TxLen == (Len + 5) )
        {
            *LcLen = Len;
            return CASE_3;
        }

        /* Case 4 - CLA INS P1 P2 Lc>0 [Lc data bytes] Le */
        if ( TxLen == (Len + 6) )
        {
            *LeLen = (uint16_t) TxBuff[TxLen-1];
            return CASE_4;
        }

        return CASE_UNKNOWN;
    }

    return CASE_UNKNOWN;
}

IccReturn_t T0TransmitHeader(SlotContext_t *SlotCtx, uint8_t *TxBuff, uint8_t len)
{
    uint8_t     Header[5];

    if (NULL == TxBuff) {
        return ICC_ERR_NULL_PTR;
    }

    memcpy((uint8_t*)Header,(uint8_t*)TxBuff,(uint16_t)4);
    Header[4] = len;
    return IccSend(SlotCtx, Header, 5);
}
