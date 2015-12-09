/*
 * ProtocolT1_BlockMgt.c -- T=1 protocol Block Management
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
#include "ProtocolT1.h"
#include "ProtocolT0_private.h" /*for the shared ScratchPadBuffer */
#include "ProtocolT1_BlockMgt.h"
#include "OSWrapper.h"      /*for memdestroy*/

/** @fn PCBValueIBlock
 *  @brief Set the PCB byte value for an I-Block according to the N, M parameters
 *  @param [in] N    Sequence toggle bit
 *  @param [in] M    value of the block to follow bit (chaining)
 *  @return         returns PCB value for the I block type specified
 */
uint8_t PCBValueIBlock(boolean_t N,boolean_t M)
{
    uint8_t PCB = IBLOCK_TYPE;

    if (bTRUE == M) {
        PCB |= PCB_MBIT;
    }

    if (bTRUE == N) {
        PCB |= PCB_I_NBIT;
    }

    return PCB;
}

/** @fn PCBValueRBlock
 *  @brief          Set the PCB byte value for an R-Block
 *  @param [in] N    Sequence toggle bit
 *  @return         returns PCB value for the R block type
 */
uint8_t PCBValueRBlock (boolean_t N)
{
    uint8_t PCB = RBLOCK_TYPE;

    if (bTRUE == N) {
        PCB |= PCB_R_NBIT;
    }

    return PCB;
}

/** @fn             SendIBlock
 *  @brief          Send an I-Block to the card
 *  @param [in] SlotCtx   pointer on the Slot configuration (see #SlotContext_t)
 *  @param [in] N          Sequence toggle bit
 *  @param [in] M          value of the block to follow bit
 *  @param [in] TxPtr      Pointer on the TxBuffer pointer
 *  @param [in] bDataSize  Length of the data to send
 *  @return               returns a #IccReturn_t error code
 *  @retval     ICC_OK    when everything went well
 *
 * @note this function uses the global shared ScratchPadBuff buffer.
 */
IccReturn_t SendIBlock(SlotContext_t  *SlotCtx, boolean_t N, boolean_t M,
                       uint8_t **TxPtr, uint8_t DataSize)
{
    IccReturn_t     retval = ICC_OK;

    if ((NULL == SlotCtx) || (NULL == TxPtr)) {
        return ICC_ERR_NULL_PTR;
    }

    if (!SlotCtx)
        return ICC_ERR_BAD_SLOT;

    /* Get the PCB value */
    SlotCtx->T1.PCB = PCBValueIBlock(N,M);

    /* Update the LEN field*/
    SlotCtx->T1.LEN = DataSize;

    /* Fill buffer  prologue */
    SlotCtx->ScratchPadBuff[NAD_OFFSET] = SlotCtx->T1.NAD;
    SlotCtx->ScratchPadBuff[PCB_OFFSET] = SlotCtx->T1.PCB;
    SlotCtx->ScratchPadBuff[LEN_OFFSET] = SlotCtx->T1.LEN;

    /* Copy to the local buffer */
    memcpy(&SlotCtx->ScratchPadBuff[PAYLOAD_OFFSET],*TxPtr, SlotCtx->T1.LEN);

    /* Send information field (length = DataSize + NAD + PCB + Len,
     * the epilogue (LRC/CRC) is automatically sent)
     */
    retval = IccSend(SlotCtx, SlotCtx->ScratchPadBuff, (uint16_t)(DataSize+3));
    if (retval != ICC_OK) {
        return retval;
    }

    *TxPtr += DataSize;
    return retval;
}

/** @fn                   SendRBlock
 *  @brief                Send a R-Block to the card
 *  @param [in] SlotCtx   pointer on the Slot configuration (see #SlotContext_t)
 *  @param [in] N          Sequence toggle bit
 *  @param [in] bError     Error identifier (0 => ok, 1 => EDC error, 2 => other error)
 *  @return               returns a #IccReturn_t error code
 *  @retval     ICC_OK    when everything went well
 *
 * @note this function uses the global shared ScratchPadBuff buffer.
 */
IccReturn_t SendRBlock(SlotContext_t  *SlotCtx, boolean_t N, uint8_t bError)
{
    IccReturn_t     retval = ICC_OK;

    if (NULL == SlotCtx)  {
        return ICC_ERR_NULL_PTR;
    }

    /* Get the PCB value */
    SlotCtx->T1.PCB = PCBValueRBlock (N) | bError;

    /* add the LEN (which is always 0 for R-block) */
    SlotCtx->T1.LEN = 0;

     /* Fill buffer  prologue */
    SlotCtx->ScratchPadBuff[NAD_OFFSET] = SlotCtx->T1.NAD;
    SlotCtx->ScratchPadBuff[PCB_OFFSET] = SlotCtx->T1.PCB;
    SlotCtx->ScratchPadBuff[LEN_OFFSET] = SlotCtx->T1.LEN;

    /*
     * Send the NAD PCB and LEN of the block
     * the epilogue (LRC/CRC) is automatically sent
     */
    retval = IccSend(SlotCtx, SlotCtx->ScratchPadBuff, (uint16_t)3);
    return retval;
}

/** @fn                   SendSBlock
 *  @brief                Send a S-Block to the card
 *  @param [in] SlotCtx   pointer on the Slot configuration (see #SlotContext_t)
 *  @param [in] BlockType  type of S-Block (ABORT, IFS, RESYNCH, WTX)
 *  @param [in] INFvalue   value for the only one byte INF field if IFS or WTX
 *  @return               returns a #IccReturn_t error code
 *  @retval     ICC_OK    when everything went well
 *
 * @note this function uses the global shared ScratchPadBuff buffer.
 */
IccReturn_t SendSBlock(SlotContext_t  *SlotCtx, uint8_t BlockType,uint8_t INFvalue)
{
    IccReturn_t    retval = ICC_OK;

    if (NULL == SlotCtx)  {
        return ICC_ERR_NULL_PTR;
    }

    if (0 == (BlockType & SBLOCK_TYPE))
        return ICC_ERR_BAD_BLOCK;

    /* Fill the PCB value */
    SlotCtx->T1.PCB = BlockType;

    /* set the LEN according to the received PCB */
    if ( (SlotCtx->T1.PCB == S_IFS_REQ) ||
         (SlotCtx->T1.PCB == S_IFS_RSP) ||
         (SlotCtx->T1.PCB == S_WTX_RSP) ) {
        SlotCtx->T1.LEN = 1;
    } else {
        SlotCtx->T1.LEN = 0;
    }


    SlotCtx->ScratchPadBuff[NAD_OFFSET] = SlotCtx->T1.NAD;
    SlotCtx->ScratchPadBuff[PCB_OFFSET] = SlotCtx->T1.PCB;
    SlotCtx->ScratchPadBuff[LEN_OFFSET] = SlotCtx->T1.LEN;
    SlotCtx->ScratchPadBuff[PAYLOAD_OFFSET] = INFvalue;


    /* Send the NAD PCB and LEN of the block
     * the epilogue (LRC/CRC) is automatically sent
     */
    retval = IccSend(SlotCtx, SlotCtx->ScratchPadBuff, (uint16_t)(3 + SlotCtx->T1.LEN));
    return retval;
}

/** @fn                     ReceiveBlock
 *  @brief                  Receives a block from the smartcard
 *  @param [in] SlotCtx     pointer on the Slot configuration (see #SlotContext_t)
 *  @param [in] ReceivedPCB  PCB of the received block
 *  @param [in] RxPtr        pointer on the RxBuffer pointer
 *  @param [in] ReceivedINF  pointer on the received length
 *  @return                 returns a #IccReturn_t error code
 *  @retval     ICC_OK      when everything went well
 *
 *  Receives a block from the smartcard and gives its PCB and its LEN
 *
 *  @note this function uses the global shared ScratchPadBuff buffer.
 */
IccReturn_t ReceiveBlock (SlotContext_t  *SlotCtx, uint8_t *ReceivedPCB,
                           uint8_t **RxPtr, uint8_t *ReceivedINF)
{
    IccReturn_t     retval;
    uint16_t        ReceivedBytes;
    uint8_t         responseNAD = 0;
    uint16_t        ExpectedLength;
    IccRequest_t    RxRequest;

    if ((NULL == SlotCtx) || (NULL == RxPtr) || (NULL == ReceivedINF)) {
        return ICC_ERR_NULL_PTR;
    }

    /* Initialize parity error bit*/
    /* the UART driver manages retries on parity error,
     * if the uart driver returns a parity error, that means that the
     * 4 Tx tries have been done and failed.
     */

    /* Start reception of the block */
    /* NOTE : ScratchPadBuff is a global shared buffer declared in T=0 protocol*/
    RxRequest.IccData = SlotCtx->ScratchPadBuff;      /*Receive in the temporary buffer */
    RxRequest.IccLen  = T1_MAX_BLOCK_SIZE; /* Maximum value is NAD+PCB+LEN+ 254 + CRC16 bytes*/
    RxRequest.IccLastByte = bFALSE;


    /* Initialize the driver to receive data */
    IccReceive(SlotCtx, &RxRequest);
    ReceivedBytes = 0;

    while (ReceivedBytes < 3) {
        ReceivedBytes = IccGetRxLen(SlotCtx, &retval);

        /* Test if a parity error occured */
        if (retval == ICC_ERR_PARITY) {
            goto receive_exit;
        }

        /* Check if card is present and activated */
        if (IccCheckCardState(SlotCtx) != ICC_OK) {
            retval =  ICC_ERR_REMOVED;
            goto receive_exit;
        }

        if (retval == ICC_ERR_TIMEOUT) {
            break;
        }
    }

    /* Check if card is still present and activated. */
    if (IccCheckCardState(SlotCtx) != ICC_OK) {
            retval =  ICC_ERR_REMOVED;
            goto receive_exit;
    }

    /* if we got less than 3 bytes and a timeout, this is a failure */
    if ((retval == ICC_ERR_TIMEOUT)  && (ReceivedBytes < 3)) {
        goto receive_exit;
    }

    if ( ((retval != ICC_RX_PENDING) && (retval != ICC_OK)) &&
         ((retval != ICC_ERR_TIMEOUT) || (ReceivedBytes < 3))  ) {
            goto receive_exit;
    }

    /* I-Block has been received and LEN = 0xFF */
    if ( ((SlotCtx->ScratchPadBuff[PCB_OFFSET] & 0x80) == IBLOCK_TYPE) &&
         (SlotCtx->ScratchPadBuff[LEN_OFFSET] == 0xFF) ) {

        /*an I-Block with len=255 is not allowed */
        if (SlotCtx->ScratchPadBuff[LEN_OFFSET] == 0xFF) {
                retval = ICC_ERR_WRONG_LEN;
        }

        /* Len is 0xFF, so it is incorrect OR
         * R-block expected instead of the I-Block
         */
        while (ReceivedBytes < (uint16_t)(SlotCtx->ScratchPadBuff[LEN_OFFSET] + 4)) /* 4-bytes for R-Block*/
        {
            ReceivedBytes = IccGetRxLen(SlotCtx, &retval);

            /* Test if a parity error occured */
            if (retval == ICC_ERR_PARITY) {
               goto receive_exit;
            }

            if (retval == ICC_ERR_TIMEOUT) {
               break;
            }

            /* check if we got an error */
            if ((retval != ICC_RX_PENDING) && (retval != ICC_OK)) {
                break;
            }

            /* Check if card is still present and activated. */
            if (IccCheckCardState(SlotCtx) != ICC_OK) {
                retval = ICC_ERR_REMOVED;
                goto receive_exit;
            }
        }

        if (ICC_ERR_TIMEOUT == retval) {
            /* if we got a timeout (not all the data)
             * but the IBlock declares a length of 255 bytes
             * in that case we must deactivate the Icc.
             */

            retval = ICC_ERR_WRONG_LEN;
            goto receive_exit;
        }

        /* Test of the epilogue */
        IccRxDone(SlotCtx);
        if (SlotCtx->IccProtocolParams.IccEDCTypeCRC == 0) {
            /* LRC on a full block must be 0 (XOR'ed values) */
            if (RxRequest.IccEDC != 0) {
                retval = ICC_ERR_BAD_EDC;
            }
        } else {
            /* CRC */
            uint16_t CRC = RxRequest.IccData[RxRequest.IccReceived-2];
            CRC <<= 8;
            CRC |= RxRequest.IccData[RxRequest.IccReceived-1];
            if (RxRequest.IccEDC != CRC) {
                retval = ICC_ERR_BAD_EDC;
            }
        }
        goto receive_exit;
    }

    if (SlotCtx->IccProtocolParams.IccEDCTypeCRC == 1) {
         /* We expect to receive LEN+5 bytes (LEN bytes + NAD PCB LEN CRC16) */
        ExpectedLength = (uint16_t)(SlotCtx->ScratchPadBuff[LEN_OFFSET]+5);
    } else {
        /* We expect to receive LEN+5 bytes (LEN bytes + NAD PCB LEN LRC8) */
        ExpectedLength = (uint16_t)(SlotCtx->ScratchPadBuff[LEN_OFFSET]+4);
    }

    /* receive remaining bytes */
    while ( (ReceivedBytes < ExpectedLength) && (retval == ICC_RX_PENDING) )
    {
        ReceivedBytes = IccGetRxLen(SlotCtx, &retval);

        if (ReceivedBytes > T1_MAX_BLOCK_SIZE) {
            break;
        }

        if ( retval == ICC_ERR_TIMEOUT) {
            goto receive_exit;
        }

        /* Test if a parity error occured */
        if (retval == ICC_ERR_PARITY) {
            goto receive_exit;
        }

        if (IccCheckCardState(SlotCtx) != ICC_OK) {
            retval = ICC_ERR_REMOVED;
            goto receive_exit;
        }
    }

    /* Reception is done, so call IccRxDone to compute the LRC/CRC */
    IccRxDone(SlotCtx);
    retval = RxRequest.IccStatus;

    responseNAD     = SlotCtx->ScratchPadBuff[NAD_OFFSET];
    *ReceivedPCB    = SlotCtx->ScratchPadBuff[PCB_OFFSET];
    *ReceivedINF    = SlotCtx->ScratchPadBuff[LEN_OFFSET];

    if (ReceivedBytes != ExpectedLength) {
        /* Error occurred while waiting for more bytes*/
       retval =  ICC_ERR_BAD_BLOCK;
       goto receive_exit;
    }

    /* Initialize the return code */
    if ((retval == ICC_RX_PENDING) || (retval == ICC_OK) ||
        (retval == ICC_ERR_TIMEOUT)) {
           retval = ICC_OK;
    }


    /* Test of the epilogue */
    if (SlotCtx->IccProtocolParams.IccEDCTypeCRC == 0) {
        /* LRC on a full block must be 0 (XOR'ed values) */
        if (RxRequest.IccEDC != 0) {
            retval = ICC_ERR_BAD_EDC;
            goto receive_exit;
        }
    } else {
        /* CRC */
        uint16_t CRC = RxRequest.IccData[RxRequest.IccReceived-2];
        CRC <<= 8;
        CRC |= RxRequest.IccData[RxRequest.IccReceived-1];
        if (RxRequest.IccEDC != CRC) {
            retval = ICC_ERR_BAD_EDC;
            goto receive_exit;
        }
    }

    /* For EMV, if NAD is different from 0x00, this is an error */
    if (responseNAD != 0x00) {
        if (SlotCtx->isEMV == 1) {
            retval = ICC_ERR_BAD_BLOCK;
            goto receive_exit;
        }
    }

    /* I-block has been received when we sent a S-Block */
    if (((*ReceivedPCB & 0x80) == IBLOCK_TYPE) &&
        ((SlotCtx->T1.PCB & BLOCK_TYPE_MASK) == SBLOCK_TYPE)){

        if ((SlotCtx->T1.PCB == S_RESYNCH_REQ) ||
            (SlotCtx->T1.PCB == S_IFS_REQ)     ||
            (SlotCtx->T1.PCB == S_ABORT_REQ)) {
            retval = ICC_ERR_BAD_BLOCK;
            goto receive_exit;
        }
    }

    /* R-block has been received */
    if ( ((*ReceivedPCB & BLOCK_TYPE_MASK) == RBLOCK_TYPE) ||
         (*ReceivedPCB == S_ABORT_REQ) ) {

        /* R-block, so LEN must be 0 */
        if (*ReceivedINF > 0) {
            retval = ICC_ERR_BAD_BLOCK;
        }

        /* R-block, so bits b6 (More data to come) must be 0 */
        if ((*ReceivedPCB & PCB_MBIT) != 0) {
            retval = ICC_ERR_BAD_BLOCK;
        }

        goto receive_exit;
    }

    /* S-block error management */
    if ((*ReceivedPCB & BLOCK_TYPE_MASK) == SBLOCK_TYPE) {

        /* S-Block should only have Max. 1 byte data length */
        if (*ReceivedINF > 1) {
            retval = ICC_ERR_BAD_BLOCK;
            goto receive_exit;
        }

        if ((*ReceivedPCB == S_RESYNCH_RSP) && (SlotCtx->T1.PCB != S_RESYNCH_REQ)) {

            /* We receive a S(X response) without a S(X request) */
            retval = ICC_ERR_BAD_BLOCK;
            goto receive_exit;

        } else if ((*ReceivedPCB == S_ABORT_RSP) && (SlotCtx->T1.PCB != S_ABORT_REQ)) {

            /* We receive a S(X response) without a S(X request) */
            retval = ICC_ERR_BAD_BLOCK;
            goto receive_exit;

        } else if ((*ReceivedPCB == S_WTX_RSP) && (SlotCtx->T1.PCB != S_WTX_REQ)) {

            /* We receive a S(X response) without a S(X request) */
            retval = ICC_ERR_BAD_BLOCK;
            goto receive_exit;

        } else if ((*ReceivedPCB == S_IFS_RSP) && (SlotCtx->T1.PCB != S_IFS_REQ)) {

            /* We receive a S(IFS response) without a S(IFS request) */
            retval =  ICC_ERR_BAD_BLOCK;
            goto receive_exit;

        } else if ((*ReceivedPCB & 0x1F) > 3) {

            /* Incorrect S block*/
            retval = ICC_ERR_BAD_BLOCK;
            goto receive_exit;

        } else if ((*ReceivedPCB == S_IFS_REQ) && (SlotCtx->T1.ExpectedPCB == S_IFS_RSP)) {

            /* We receive a S(IFS request) instead of a S(IFS response)
             *
             * Note: card reader can only send S(IFS request) at the
             * very first block only (after ATR)
             */
            retval =  ICC_ERR_BAD_BLOCK;
            goto receive_exit;
        }

    }

    /* Reception of Information field */
    if ( (*ReceivedPCB == S_IFS_REQ) ||
         (*ReceivedPCB == S_WTX_REQ) ||
         (*ReceivedPCB == S_IFS_RSP) ) {

        /* First byte of information field is stored at offset 3 */
        ReceivedBytes = SlotCtx->ScratchPadBuff[PAYLOAD_OFFSET];

        /*
         * If block received is IFS request,
         * check LEN value (between 0x10 and 0xFE)
         */
        if (*ReceivedPCB == S_IFS_REQ) {
            if ((ReceivedBytes == 0xFF) || (ReceivedBytes < 0x10)) {
                retval =  ICC_ERR_BAD_BLOCK;
                goto receive_exit;
            }
        }

        /* Check that received value is equal sent value */
        if (*ReceivedPCB == S_IFS_RSP) {

            /* An IFS response has been received */
            if (SlotCtx->T1.INF != ReceivedBytes) {
                retval = ICC_ERR_BAD_BLOCK;
                goto receive_exit;
            } else {

                SlotCtx->IccProtocolParams.IccIFSD = ReceivedBytes;
            }
        } else {
            SlotCtx->T1.INF = ReceivedBytes;
        }

        if (*ReceivedINF > 1) {
            retval = ICC_ERR_BAD_BLOCK;
            goto receive_exit;
        }
    } else {
        /* Copy data to reception buffer */
        if (retval == ICC_OK) {
            memcpy(*RxPtr,(uint8_t*)(&SlotCtx->ScratchPadBuff[PAYLOAD_OFFSET]),
		           (uint16_t)*ReceivedINF);
            *RxPtr += *ReceivedINF;
        }
    }

    retval = RxRequest.IccStatus;

receive_exit:
    /*
     * ICC done is called as this is the error exit path and
     * we may still have the driver in RX mode
     */
    IccRxDone(SlotCtx);

    return retval;
}


/** @fn                     CheckMBit
 *  @brief                  return bTRUE if the PCB indicates More data to come
 *  @param [in] PCBtoCheck   Block PCB value to check

 *  @return                 returns a #boolean_t value
 *  @retval     bTRUE        if the M bit is set in the PCB
 *  @retval     bFALSE       if the M bit is not set
 *
 */
boolean_t CheckMBit(uint8_t PCBtoCheck)
{
    if ((PCBtoCheck & PCB_MBIT) == PCB_MBIT) {
        return bTRUE;
    }

    return bFALSE;
}
