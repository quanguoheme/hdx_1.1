/*
 * ProtocolT1.c -- Send/Receive APDU using T=1 protocol
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
#include "sc_config.h"
#include "slot.h"
#include "iccabstract.h"
#include "ProtocolT1.h"
#include "ProtocolT1_BlockMgt.h"

#define EXIT_ON_NOT_OK(x)       {if ((x) != ICC_OK) {\
                                    nextstate = T1_STATE_DONE; \
                                    break;}}

/** @fn SendT1
 *  @brief  Send a T1 APDU and get the Icc response
 *  @param [in]  slotid          Slot Id to activate
 *  @param [in]  TxBuff          APDU to send buffer
 *  @param [in]  SendDataLen     Length of the APDU to transmit
 *  @param [out] RxBuff          Receive buffer (Must be at least 2 bytes in case 1)
 *  @param [out] RxLen  pointer on the received length
 *  @param [in]  pfSendWTE       pointer on the WTX handler (if not in EMV mode)
 *
 *
 *  Send a T=1 command (in Tx buffer) and get the response (in Rx buffer)
 */
IccReturn_t SendT1(uint8_t slotid,
                   uint8_t *TxBuff, uint32_t SendDataLen,
                   uint8_t *RxBuff,  uint32_t *RxLen,
                   void (*pfSendWTE) (void) )
{
    uint32_t            InitialBWT;     /* used to store the inital BWT (from
                                         * ATR or PTS)
                                         */
    uint32_t            NewBWT;         /* in case of WTX request, we need to
                                         * recalculate the BWT value
                                         * this variable contains a partial
                                         * computation result
                                         */
    uint16_t            InitialLength;  /* length of the block sent, stored
                                         * in case of we need to send the same
                                         * block once again
                                         */
    uint16_t            InitialAPDULen; /* store the ADPU length in case of
                                         * we need to resend the whole
                                         * command
                                         */
    uint8_t             GetResponseLength; /* card answer length */
    uint8_t             *TxPtr;         /*current pointer on the Tx buffer */
    uint8_t             *RxPtr;         /*current pointer on the Rx buffer */
    uint8_t            *PrevTxPtr;      /* previous pointer on the Tx buffer,
                                         * will be used when we need to
                                         * resend a block
                                         */
    uint8_t             ReceivedPCB;    /* Card answer PCB */
    uint8_t             RxErrorCount;   /* number of wrong received packets before
                                         * doing a resynch */
    uint8_t             CurrentRetry;   /* number of allowed Tx retries */
    uint8_t             FailedSyncNb;   /* number of allowed Resynch sent */

    boolean_t           Continue          = bFALSE;  /* this value is set if we detected chained blocks*/
    boolean_t           is3rdBlockValid   = bTRUE;   /* 2 first blocks contain
                                                     * an error, this is set if
                                                     * the last block is the 3rd
                                                     * block contains the error
                                                     * flag
                                                     */
    boolean_t           AbortOccured      = bFALSE;  /* Set when an ABORT is
                                                     *received from the card
                                                     */
    boolean_t           Chaining          = bFALSE;  /* Set when the card does
                                                     * chained blocks
                                                     */
    boolean_t           IFSRequestAtStart = bFALSE;  /*in case of EMV mode,
                                                    * we request the IFSC is requested
                                                    * before the 1st block exchange
                                                    */

    IccReturn_t         RetCode           = ICC_OK;
    IccReturn_t         PrevRetCode       = ICC_OK;
    SlotContext_t       *SlotCtx          = NULL;
    T1ProtocolState_t   nextstate         = T1_SEND_BLOCK_STATE;  /* set the initial state */

    SlotCtx = IccSlotGetConfiguration(slotid);
    if (!SlotCtx) {
        return ICC_ERR_BAD_SLOT;
    }

    /*if EMV mode, there is no need to process the WTX*/
    if ((NULL == TxBuff) || (NULL == RxBuff) || (NULL == RxLen)) {
        return ICC_ERR_NULL_PTR;
    }

    if (((SlotCtx->InitialParams.IFSC != 32) || (bTRUE == SlotCtx->isEMV)) &&
        (bTRUE == SlotCtx->T1.FirstBlock)){
        IFSRequestAtStart = bTRUE;
    }

    *RxLen = 0;

    InitialLength   = SendDataLen;
    InitialAPDULen  = SendDataLen;
    PrevTxPtr       = TxPtr;

    TxPtr           = TxBuff;
    RxPtr           = RxBuff;

    /* Retreive the correct value for the BWT */
    InitialBWT = SlotCtx->IccProtocolConfig.IccWaitingTime;

    /*restore retry default values */
    SlotCtx->T1.MaxRetries   = SlotDefaultConfig.T1.MaxRetries;
    CurrentRetry             = SlotDefaultConfig.T1.MaxRetries;
    FailedSyncNb             = SlotDefaultConfig.T1.MaxRetries&0x0F;
    RxErrorCount             = (SlotDefaultConfig.T1.MaxRetries&0xF0)>>4;

    do {
        switch (nextstate) {
/******************************************************************************
 *                      T1_SEND_BLOCK_STATE
 ******************************************************************************/
        case T1_SEND_BLOCK_STATE:

            /* Initialize R-block parameters */
            SlotCtx->T1.RBlockAlreadySent = bFALSE;
            SlotCtx->T1.LastRBlockErr     = 0;
            is3rdBlockValid               = bTRUE;

            /* IFS request management */
            if ((bTRUE == IFSRequestAtStart) &&
                (bTRUE == SlotCtx->T1.FirstBlock)) {
                /* Send an IFS Request*/
                Chaining = bFALSE;
                SlotCtx->T1.INF = SlotCtx->IccProtocolParams.IccIFSD;

                /* DO NOT CHANGE THE SlotCtx->T1.FirstBlock value !!! */

                RetCode = SendSBlock(SlotCtx, S_IFS_REQ, SlotCtx->T1.INF);
                EXIT_ON_NOT_OK(RetCode);

                SlotCtx->T1.ExpectedPCB = S_IFS_RSP;
                SlotCtx->T1.CurrentPCB = S_IFS_REQ;
                nextstate = T1_RECEIVE_BLOCK_STATE;
                break;
            }

            if (SlotCtx->T1.FirstBlock == bTRUE) {
                /* Length of data to send is SendDataLen */
                TxPtr = TxBuff;
                RxPtr = &RxBuff[0];

                /* Currently no byte has been received */
                *RxLen = 0;

                InitialAPDULen  = SendDataLen;

                SlotCtx->T1.FirstBlock     = bFALSE;
                SlotCtx->T1.IccChaining    = bFALSE;
                PrevRetCode                = ICC_OK;
            }

            /* Update previous Tx/Rx pointer values */
            PrevTxPtr       = TxPtr;
            InitialLength   = SendDataLen;

            /* Check if it is a chained block */
            if (SendDataLen > SlotCtx->IccProtocolParams.IccIFSC) {
                Chaining = bTRUE;
                RetCode = SendIBlock(SlotCtx, SlotCtx->T1.Ns,bTRUE, &TxPtr, SlotCtx->IccProtocolParams.IccIFSC);
                SendDataLen -= SlotCtx->IccProtocolParams.IccIFSC;


                /* If M=1 (chaining), expected block is R(Ns) */
                SlotCtx->T1.ExpectedPCB = PCBValueRBlock((boolean_t)!SlotCtx->T1.Ns);
                SlotCtx->T1.CurrentPCB  = PCBValueIBlock(SlotCtx->T1.Ns, bTRUE);
            } else {
                Chaining = bFALSE;
                /* If M=0, expected block is I(Nr,x) */
                SlotCtx->T1.ExpectedPCB = PCBValueIBlock(SlotCtx->T1.Nr, bFALSE);
                SlotCtx->T1.CurrentPCB  = PCBValueIBlock(SlotCtx->T1.Ns, bFALSE);

                RetCode = SendIBlock(SlotCtx, SlotCtx->T1.Ns,
                                     bFALSE, &TxPtr,
                                     (uint8_t)(SendDataLen & 0x00FF));
                EXIT_ON_NOT_OK(RetCode);
            }
            /* fall-through */

/******************************************************************************
 *                      T1_RECEIVE_BLOCK_STATE
 ******************************************************************************/
        case T1_RECEIVE_BLOCK_STATE:
            GetResponseLength = 0;
            RetCode = ReceiveBlock(SlotCtx, &ReceivedPCB, &RxPtr, &GetResponseLength);

            /* Retrieve the correct BWT */
            if (SlotCtx->T1.WTX == bTRUE) {
                SlotCtx->IccProtocolConfig.IccWaitingTime = InitialBWT;
                SlotCtx->T1.WTX = bFALSE;
            }

            /*  in case of EMV, If a timeout OR
             *  wrong LEN occurs deactivate the card
             */
            if (((RetCode == ICC_ERR_TIMEOUT) ||
                 (RetCode == ICC_ERR_WRONG_LEN)) &&
                 (SlotCtx->isEMV == bTRUE)) {
                    IccDeactivate(SlotCtx);
                    nextstate = T1_STATE_DONE;
                    break;
            }

            /* Analyse block */
            if (RetCode == ICC_OK) {

                /*
                 * if the block is an R(!Ns; no error),
                 * (this is an expected answer to an I-block)
                 * reset the RxErrorCount value
                 */
                if ((ReceivedPCB & 0xFC) == PCBValueRBlock((boolean_t)!SlotCtx->T1.Ns)) {
                    /*the Icc assumes the error */
                    RxErrorCount = (SlotDefaultConfig.T1.MaxRetries&0xF0)>>4;
                } else
                /* Rblock with wrong NS value*/
                if ((ReceivedPCB & 0xFC) == PCBValueRBlock(SlotCtx->T1.Ns)) {
                    RxErrorCount--;
                    if (0 == RxErrorCount) {
                        /* too many errors, try to resynchronize */
                        nextstate = T1_RESYNCH_STATE;
                     } else {
                        nextstate = T1_SEND_BLOCK_AGAIN_STATE;
                    }
                    RetCode = ICC_ERR_BAD_BLOCK;
                    break;
                }

                if (PrevRetCode != ICC_OK) {
                    /* Correct block received after erronous packets
                     * reinit the high part of the retry counter
                     * (if there was an error before)
                     */
                    CurrentRetry &= 0x0F;
                    CurrentRetry |= SlotCtx->T1.MaxRetries & 0xF0;
                    RxErrorCount = (SlotDefaultConfig.T1.MaxRetries&0xF0)>>4;

                }

                PrevRetCode = RetCode;

                /* Switch to the correct state */
                if (ReceivedPCB == S_WTX_REQ) {
                    /* Block is a WTX request */
                    nextstate = T1_SEND_WTX_RSP_STATE;
                    break;
                }

                if (ReceivedPCB == S_IFS_REQ) {
                    /* Block is a IFS request */
                    if (SlotCtx->T1.PCB == S_IFS_REQ) {
                        nextstate = T1_SEND_BLOCK_AGAIN_STATE;
                    } else {
                        nextstate = T1_SEND_IFS_RSP_STATE;
                    }
                    break;
                }

                if (ReceivedPCB == S_ABORT_REQ) {
                    /* Block is an ABORT request */
                    nextstate = T1_SEND_ABORT_RSP_STATE;
                    break;
                }

                if (CheckMBit(ReceivedPCB) &&
                   ((ReceivedPCB & 0x80) == IBLOCK_TYPE)) {
                    /* Received an I-block with M=1 */
                    if ( (ReceivedPCB & 0xDF) == (SlotCtx->T1.ExpectedPCB & 0xDF) ) {
                        nextstate = T1_SEND_R_M1_STATE;
                        break;
                    }
                }

                if ((SlotCtx->T1.PCB == S_RESYNCH_REQ) &&
                    (ReceivedPCB != S_RESYNCH_RSP)) {
                    /*
                     * The block sent was a S(RESYNCH request),
                     * we received something which is neither the Resync response
                     * or a Rblock, we must send this block again
                     */
                    RxErrorCount--;
                    if (0 == RxErrorCount) {
                        /* too many errors, try to resynchronize */
                        nextstate = T1_RESYNCH_STATE;
                     } else {
                        nextstate = T1_SEND_BLOCK_AGAIN_STATE;
                    }
                    break;
                }

                if ((SlotCtx->T1.PCB == S_IFS_REQ) &&
                    (ReceivedPCB != S_IFS_RSP)) {
                    /*
                     * The block sent was a S(IFS request),
                     * we received something which is neither the IFS response
                     * or a Rblock, we must send this block again
                     */
                    RxErrorCount--;
                    if (0 == RxErrorCount) {
                        /* too many errors, try to resynchronize */
                        nextstate = T1_RESYNCH_STATE;
                     } else {
                        nextstate = T1_SEND_BLOCK_AGAIN_STATE;
                    }
                    break;
                }

                /* we got the IFS response, now send the command */
                if ((SlotCtx->T1.PCB == S_IFS_REQ) &&
                    (ReceivedPCB == S_IFS_RSP)) {
                    IFSRequestAtStart = bFALSE;
                    nextstate = T1_SEND_BLOCK_STATE;
                    break;
                }

                if (ReceivedPCB == SlotCtx->T1.ExpectedPCB) {
                    /* The correct block has been received */
                    *RxLen += GetResponseLength;
                    CurrentRetry = SlotCtx->T1.MaxRetries;

                    /* Reset the flag for Rblock error type */
                    SlotCtx->T1.RBlockAlreadySent  = bFALSE;
                    SlotCtx->T1.LastRBlockErr      = 0;
                    is3rdBlockValid                = bTRUE;
                    RxErrorCount                   = (SlotDefaultConfig.T1.MaxRetries&0xF0)>>4;

                    nextstate = T1_IS_LAST_BLOCK_STATE;
                    break;
                }
            }

            RxErrorCount--;

            PrevRetCode = RetCode;

            if ( (SlotCtx->T1.PCB == S_RESYNCH_REQ) ||
                 (SlotCtx->T1.PCB == S_IFS_REQ) ) {
                /* The block is an R(Ns),
                 * we have to send the last block again
                 */
                if (0 == RxErrorCount) {
                    /* too many errors, try to resync
                     */
                    nextstate = T1_RESYNCH_STATE;
                 } else {
                    nextstate = T1_SEND_BLOCK_AGAIN_STATE;
                }
                break;
            }


            if (RetCode == ICC_ERR_PARITY)  {
                nextstate = T1_SEND_RBLOCK_STATE;
                break;
            }

            /*
             * The block received is I-Block
             * but expected block is R(in response to a chained I-block)
             */
            if ( ((ReceivedPCB & 0x80) == IBLOCK_TYPE) &&
                 ((SlotCtx->T1.ExpectedPCB & BLOCK_TYPE_MASK) == RBLOCK_TYPE) ) {

                    RetCode = ICC_ERR_BAD_BLOCK;
                    if ( GetResponseLength > 0)
                        RxPtr -= GetResponseLength;
            }

            /*
             * we received a R-block (no error)
             * but we expected an I-Block
             */
            else if ( ((ReceivedPCB & (BLOCK_TYPE_MASK | BLOCK_ERROR_MASK)) == RBLOCK_TYPE) &&
                      ((SlotCtx->T1.ExpectedPCB & 0x80) == IBLOCK_TYPE)) {
                    RetCode = ICC_ERR_BAD_BLOCK;
            }

            /* we received an I-block with wrong sequence number */
            else if ( ((ReceivedPCB & 0x80) == IBLOCK_TYPE)  &&
                      ((ReceivedPCB & PCB_I_NBIT) != (SlotCtx->T1.ExpectedPCB & PCB_I_NBIT)) ) {
                /*
                 * if there is already data copied,
                 * move the Rx Pointer back
                 */
                if ( RxPtr > RxBuff) {
                    RxPtr -= GetResponseLength;
                }

                RetCode = ICC_ERR_BAD_BLOCK;
            }

            /*
             * we received a R-block (with error) with wrong sequence number,
             * status is bad_block (so an R-block can be sent again),
             * with right sequence number, resend last I-block
            */
            else if ( ( (ReceivedPCB & BLOCK_TYPE_MASK) == RBLOCK_TYPE) &&
                      ( (ReceivedPCB & BLOCK_ERROR_MASK) > 0) )
            {
                /* The Icc assume the error */
                RxErrorCount = (SlotDefaultConfig.T1.MaxRetries&0xF0)>>4;

                if ( (ReceivedPCB & 0xFC) != PCBValueRBlock(SlotCtx->T1.Ns)) {
                    RetCode = ICC_ERR_BAD_BLOCK;
                }
            }

            nextstate = T1_SEND_RBLOCK_STATE;
            break;


/******************************************************************************
 *                      T1_IS_LAST_BLOCK_STATE
 ******************************************************************************/
        case T1_IS_LAST_BLOCK_STATE:

#ifdef CETECOM_FIME_EMVCO_1CF_126_0y_COMPLIANCE
#warning The CETECOM and FIME lab patch for test EMVCO 1CF.126.0y is enabled, read the source code comments to get details.

            /*
             * for CETECOM and FIME Labs,
             * if we received 3 wrong answers to a IFS(request),
             * we do a resynch, then we restart with the select
             * (ie not with a new IFS(Request)
             *
             * Note: this is not specified by the EMV, for the
             * EMVCo ref. 1CF.126.0y test, the pass criteria is to
             * receive a S(Resynch) request.
             */
            if ((IFSRequestAtStart == bTRUE) &&
                (ReceivedPCB != S_RESYNCH_RSP)) {
                nextstate = T1_SEND_BLOCK_STATE;
                break;
            }
#else

            if (IFSRequestAtStart == bTRUE) {
                nextstate = T1_SEND_BLOCK_STATE;
                break;
            }
#endif

            if ((Chaining == 1) ||
                (CheckMBit(ReceivedPCB) &&
                 ((ReceivedPCB & 0x80) == IBLOCK_TYPE)) ) {
                /* M=1,
                 * there is at least one more block to transmit
                 */
                Continue = bTRUE;
            } else {
                /* If M=0 for receiver and sender,
                 * there is no more block to transmit
                 */
                Continue = bFALSE;
            }

            /* if block is different from S(RESYNCH)
             * The flags Nr and Ns are swapped
             */
            if (ReceivedPCB == S_RESYNCH_RSP) {
                SlotCtx->T1.Ns = bFALSE;
                SlotCtx->T1.Nr = bFALSE;

#ifdef CETECOM_FIME_EMVCO_1CF_126_0y_COMPLIANCE
                IFSRequestAtStart = bFALSE;
#endif

                /* Last block or command has to be sent again */
                if (SlotCtx->T1.ResyncSendBlockCurrent == 1) {
                    /* Send the last block */
                    TxPtr = PrevTxPtr;
                } else {
                    /* Send the last entire command */
                    SlotCtx->T1.FirstBlock = 1;
                    SendDataLen = InitialAPDULen;
                }
                Continue = bTRUE;
            } else {
                if (SlotCtx->T1.IccChaining == 0){
                    SlotCtx->T1.Ns = (boolean_t)!SlotCtx->T1.Ns;
                } else {
                    SlotCtx->T1.IccChaining = 0;
                }

                if (Chaining == 0) {
                    SlotCtx->T1.Nr = (boolean_t) !SlotCtx->T1.Nr;
                }
            }

            if (Continue == 1) {
                nextstate = T1_SEND_BLOCK_STATE;
                break;
            } else {
                /* Retreive value of BWT
                 * (it may have been modified by a WTX request)
                 */
                SlotCtx->IccProtocolConfig.IccWaitingTime = InitialBWT;

                /* We are now during the protocol */
                SlotCtx->T1.ProtocolInitStage = 0;

                if (AbortOccured == 1) {
                    RetCode = ICC_ERR_ABORT_OCCURED;
                } else {
                    RetCode = ICC_OK;
                }

                /* we have done ! */
                nextstate = T1_STATE_DONE;
            }
            break;


/******************************************************************************
 *                      T1_SEND_R_STATE
 ******************************************************************************/
        case T1_SEND_RBLOCK_STATE:
            /* Check number of retries */
            if ( (CurrentRetry & 0xF0) == 0 ) {
                /* 3 consecutive R-Blocks are bad */
                if ( RetCode != ICC_OK) {
                    is3rdBlockValid = bFALSE;
                }

                /*
                 * the third retry shall be allowed in the case
                 * of ICC sent 2 bad I-blocks followed by a R-block
                 * which indicates an error.
                 */

                /*  Block received is R-Block with error
                 *  make sure it's not an S-Block.
                 */
                if ( (is3rdBlockValid == bTRUE) &&
                     ( (ReceivedPCB & BLOCK_TYPE_MASK) == RBLOCK_TYPE ) &&
                     ( (ReceivedPCB & BLOCK_ERROR_MASK) != 0x00 ) ) {
                    RetCode = SendRBlock(SlotCtx, SlotCtx->T1.Nr,SlotCtx->T1.LastRBlockErr);
                    EXIT_ON_NOT_OK(RetCode);

                    nextstate = T1_RECEIVE_BLOCK_STATE;
                    break;
                }

                /* otherwise (ISO7816) try to resynchronize */
                CurrentRetry = SlotCtx->T1.MaxRetries;
                nextstate = T1_RESYNCH_STATE;
                break;

            }

            if (RxErrorCount == 0) {
                nextstate = T1_RESYNCH_STATE;
                break;
            }

            /* Check if a previous Rblock was sent */
            if (SlotCtx->T1.RBlockAlreadySent == bTRUE) {
                /* Reset the flag */
                RetCode = SendRBlock(SlotCtx, SlotCtx->T1.Nr,SlotCtx->T1.LastRBlockErr);
                EXIT_ON_NOT_OK(RetCode);
            } else {
                SlotCtx->T1.RBlockAlreadySent = bTRUE;

                /* An incorrect block has been received (or no block)
                 * send a R(Nr) block */
                if ((RetCode == ICC_ERR_BAD_EDC) ||
                    (RetCode == ICC_ERR_PARITY)) {
                    SlotCtx->T1.LastRBlockErr = 1;
                } else if (  RetCode != ICC_OK ) {
                    SlotCtx->T1.LastRBlockErr = 2;
                }

                RetCode = SendRBlock(SlotCtx, SlotCtx->T1.Nr,SlotCtx->T1.LastRBlockErr);
                EXIT_ON_NOT_OK(RetCode);
            }

            nextstate = T1_RECEIVE_BLOCK_STATE;
            break;


/******************************************************************************
 *                      T1_SEND_BLOCK_AGAIN_STATE
 ******************************************************************************/
        case T1_SEND_BLOCK_AGAIN_STATE:
            if ((CurrentRetry & 0x0F) == 0) {
                /*reset the retry counter*/
                CurrentRetry = SlotCtx->T1.MaxRetries;

                if (SlotCtx->T1.PCB == S_IFS_REQ) {
                    if ( SlotCtx->isEMV == bTRUE ) {
                        /* if EMV Compliant and the retry counter is 0,
                         * deactivate the card
                         */
                        IccDeactivate(SlotCtx);

                        /*reset to default */
                        CurrentRetry          = SlotCtx->T1.MaxRetries;
                        SlotCtx->T1.IccChaining    = 0;

                        nextstate = T1_STATE_DONE;
                        RetCode   = ICC_ERR_TRANSMISSION;
                        break;
                    } else {
                        /* The process continues */
                        nextstate = T1_IS_LAST_BLOCK_STATE;
                        break;
                    }
                }

                /* Resynchronisation will be attempted */
                nextstate = T1_RESYNCH_STATE;
                break;
            }

            if ((SlotCtx->T1.CurrentPCB & 0x80) == IBLOCK_TYPE) {
                /* Block to send again is an I-block
                 * source pointer and length are updated
                 */
                TxPtr       = PrevTxPtr;
                SendDataLen = InitialLength;
                nextstate   = T1_SEND_BLOCK_STATE;
                break;
            }

            if (SlotCtx->T1.CurrentPCB  == S_RESYNCH_REQ) {
                nextstate   = T1_RESYNCH_STATE;
                break;
            }

            if ((SlotCtx->T1.CurrentPCB & BLOCK_TYPE_MASK) == SBLOCK_TYPE) {
                /* Block to send again is a S-Block */
                RetCode = SendSBlock(SlotCtx, SlotCtx->T1.CurrentPCB,SlotCtx->T1.INF);
                EXIT_ON_NOT_OK(RetCode);

                /* If Request is 0xCx,
                 * then response is 0xEx
                 */
                SlotCtx->T1.ExpectedPCB = SlotCtx->T1.CurrentPCB | 0x20;
                nextstate = T1_RECEIVE_BLOCK_STATE;
                break;
            }

            /* Block is a R-block
             * Send the Rblock again */
            SlotCtx->T1.RBlockAlreadySent = bTRUE;
            SlotCtx->T1.LastRBlockErr     = 0;
            RetCode = SendRBlock(SlotCtx, SlotCtx->T1.Nr,0);
            EXIT_ON_NOT_OK(RetCode);

            SlotCtx->T1.ExpectedPCB  = PCBValueIBlock (SlotCtx->T1.Nr, bFALSE);
            SlotCtx->T1.CurrentPCB   = PCBValueRBlock (SlotCtx->T1.Nr);
            nextstate = T1_RECEIVE_BLOCK_STATE;
            break;


/******************************************************************************
 *                      T1_SEND_WTX_RSP_STATE
 ******************************************************************************/
        case T1_SEND_WTX_RSP_STATE:
            /* Mastercard :
             * Response time = [ [(2^BWI * 960 * D) + 11] * [INFByte] + [ INFByte * D * (960+480)] ]
             *               = [INFByte] * [ [(2^BWI * 960 * D) + 11]  + [ D * (960+480)] ]
             *
             * according to EMV book: BWT response time = [INFByte] * [ [(2^BWI * 960 * D) + 11]  + [ D * (960+1)] ]
             * according to test case : BWT max is = [INFByte] * [ [(2^BWI * 960 * D) + 11]  + [ D * (960+480)] ]
             * Set BWT between both values.
             *
             * BWT = [INFByte] * [ [(2^BWI * 960 * D) + 11]  + [ D * (960+240)] ]ETUs.
             */
            NewBWT = 960;
            NewBWT *= (1 << SlotCtx->InitialParams.BWI);
            NewBWT *= SlotCtx->InitialParams.Di;
            NewBWT += 11;

            SlotCtx->IccProtocolConfig.IccWaitingTime  = 960 + 240;
            SlotCtx->IccProtocolConfig.IccWaitingTime *= SlotCtx->InitialParams.Di;
            SlotCtx->IccProtocolConfig.IccWaitingTime += NewBWT;
            SlotCtx->IccProtocolConfig.IccWaitingTime *= SlotCtx->T1.INF;

            /* Send S(WTX RESPONSE) */
            RetCode = SendSBlock(SlotCtx, S_WTX_RSP, SlotCtx->T1.INF);
            EXIT_ON_NOT_OK(RetCode);

            /* inform the user that a Waiting time eXtension has been catched*/
            if (NULL != pfSendWTE) {
                (*pfSendWTE) ();
            }

            SlotCtx->T1.WTX = 1;

            nextstate = T1_RECEIVE_BLOCK_STATE;
            break;

/******************************************************************************
 *                      T1_SEND_IFS_RSP_STATE
 ******************************************************************************/
        case T1_SEND_IFS_RSP_STATE:
            RetCode = SendSBlock(SlotCtx, S_IFS_RSP, SlotCtx->T1.INF);
            EXIT_ON_NOT_OK(RetCode);

            SlotCtx->IccProtocolParams.IccIFSC = SlotCtx->T1.INF;
            nextstate = T1_RECEIVE_BLOCK_STATE;
            break;

/******************************************************************************
 *                      T1_SEND_R_M1_STATE
 ******************************************************************************/
        case T1_SEND_R_M1_STATE:
            /*
             * Acknowledge of a chained I-BLock => R(Nr)
             * M=1 on the received I-BLock => chained block
             */
            *RxLen += GetResponseLength;

            /*
             * Previous I-Block has been acknowledged
             * swap Ns bit
             */
            if (SlotCtx->T1.IccChaining == 0) {
                SlotCtx->T1.IccChaining = 1;
                SlotCtx->T1.Ns = (boolean_t)!SlotCtx->T1.Ns;
            }

            /* Swap Nr bit to acknowledge the chained blocks */
            SlotCtx->T1.Nr = (boolean_t)!SlotCtx->T1.Nr;

            SlotCtx->T1.RBlockAlreadySent = bTRUE;
            SlotCtx->T1.LastRBlockErr     = 0;
            RetCode = SendRBlock(SlotCtx, SlotCtx->T1.Nr,0);
            EXIT_ON_NOT_OK(RetCode);

            SlotCtx->T1.ExpectedPCB   = PCBValueIBlock (SlotCtx->T1.Nr, bFALSE);
            SlotCtx->T1.CurrentPCB    = PCBValueRBlock (SlotCtx->T1.Nr);
            nextstate = T1_RECEIVE_BLOCK_STATE;
            break;

/******************************************************************************
 *                      T1_RESYNCH_STATE
 ******************************************************************************/
        case T1_RESYNCH_STATE:
            if ( SlotCtx->T1.ProtocolInitStage == 1 ) {
                IccDeactivate(SlotCtx);
                CurrentRetry = SlotCtx->T1.MaxRetries;
                SlotCtx->T1.IccChaining = 0;
                nextstate = T1_STATE_DONE;
                RetCode = ICC_ERR_TRANSMISSION;
                break;
            }

        /**********************************************************************
         *                        T1_SEND_RESYNCH_STATE
         **********************************************************************/
            if (FailedSyncNb == 0) {
                CurrentRetry = SlotCtx->T1.MaxRetries;
                FailedSyncNb = (SlotCtx->T1.MaxRetries & 0x0F);/* * 3;*/
                SlotCtx->T1.IccChaining = 0;
                nextstate = T1_STATE_DONE;
                RetCode = ICC_ERR_TRANSMISSION;
                break;
            }

            FailedSyncNb--;
            RetCode = SendSBlock(SlotCtx, S_RESYNCH_REQ, 0);
            EXIT_ON_NOT_OK(RetCode);

            SlotCtx->T1.ExpectedPCB = S_RESYNCH_RSP;
            SlotCtx->T1.CurrentPCB = S_RESYNCH_REQ;
            nextstate = T1_RECEIVE_BLOCK_STATE;
            break;


/******************************************************************************
 *                      T1_SEND_ABORT_RSP_STATE
 ******************************************************************************/
        case T1_SEND_ABORT_RSP_STATE:
            /*
             * An abort request has been received
             * If EMV, deactivate
             */
            if (bTRUE == SlotCtx->isEMV) {
                nextstate = T1_DEACTIVATION_STATE;
                break;
            }

            /* If not EMV,
             * we acknowledge with a S(ABORT RSP) block
             */
            RetCode = SendSBlock(SlotCtx, S_ABORT_RSP, 0);
            EXIT_ON_NOT_OK(RetCode);

            if (SlotCtx->T1.IccChaining == 1) {
                /* The chained block has been initiated by the smart card */
                SlotCtx->T1.ExpectedPCB = PCBValueIBlock(SlotCtx->T1.Nr,bFALSE);
                SlotCtx->T1.CurrentPCB = S_ABORT_RSP;

                /* Receive block at offset 0 */
                RxPtr = &RxBuff[0];
                *RxLen = 0;
                nextstate = T1_RECEIVE_BLOCK_STATE;
                /*break;*/
            } else {
                /* The chained block has been initiated by the terminal */
                SlotCtx->T1.ExpectedPCB = PCBValueRBlock ((boolean_t)!SlotCtx->T1.Ns);
                SlotCtx->T1.CurrentPCB  = S_ABORT_RSP;

                AbortOccured    = bTRUE;
                Chaining        = bFALSE;
                SlotCtx->T1.Nr = bFALSE;
                nextstate = T1_RECEIVE_BLOCK_STATE;
                /*break;*/
            }
            break;

/******************************************************************************
 *                      T1_DEACTIVATION_STATE
 ******************************************************************************/
        case T1_DEACTIVATION_STATE:
            /* A fatal error occured
             * deactivate smart card contacts
             */
            IccDeactivate(SlotCtx);
            SlotCtx->T1.IccChaining = 0;
            nextstate = T1_STATE_DONE;
            RetCode = ICC_ERR_ABORT_OCCURED;
            break;

        default:
            break;

        } /*end of switch */
    } while (nextstate != T1_STATE_DONE);

    return RetCode;
}
