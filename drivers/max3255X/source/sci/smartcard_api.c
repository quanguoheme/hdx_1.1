/*
 * smartcard_api.c -- Smartcard API for users
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
#include <config.h>
#include "sc_errors.h"
#include "sc_config.h"
#include "slot.h"
#include "iccabstract.h"
#include "OSWrapper.h"
#include "MAX3255x_afe.h"
#include "MAX325xx_uart.h"
#include "smartcard_api.h"
#include "ProtocolT1_BlockMgt.h"

/** Other includes */
#include <mml.h>
#include <mml_gcr.h>
#include <mml_gcr_regs.h>

/*** This file is specific to the MAX32550 and MAX32555 running COBRA */
#define CLOCK_REG_SCCK          (MML_GCR_IOBASE + MML_GCR_SCCK_OFST)
#define SCCK_DIVIDER_SHIFT      6
#define SCCK_DIVIDER_MASK       0x3F
#define SC_CLOCK_MAX            4800000
#define SC_CLOCK_MIN            2000000

typedef struct {
    uint8_t            RxBuffer[T1_MAX_BLOCK_SIZE];
    uint8_t            ScratchPadBuffer[T1_MAX_BLOCK_SIZE];
    void               (*pSendWTE)(void);
    ActivationParams_t *ActivationParams;
} IccRx_t;

static IccRx_t   CardResponse[MAX3255x_SLOT_NUMBER];



#ifdef _STAND_ALONE_DRIVER_SCI_
/*
 * This compilation flag is activated only in driver development context
 * without any application using it.
 * DO NOT define it in application/test context then.
 */
/*****************************************************************************/
int main(void)
{
	/** We're done */
	return ICC_OK;
}
#endif /* _STAND_ALONE_DRIVER_SCI_ */

/** @fn         SCAPI_open
 *  @brief      Open a Slot
 *
 *  @param [in]  SlotId      Slot (AFE) Id.
 *
 *  @return     returns an #IccReturn_t error code
 *  @note       This function is available only for COBRA targets
 */
IccReturn_t SCAPI_open(uint8_t SlotId)
{
    IccReturn_t ret;
    SlotContext_t  *Ctx;

    if (MAX3255x_SLOT_NUMBER <= SlotId)
        return ICC_ERR_BAD_SLOT;

    Ctx = OSWrapper_getSlotCtxbyId(SlotId);

    if (NULL == Ctx) {
        return ICC_ERR_NULL_PTR;
    }

    CardResponse[SlotId].pSendWTE = NULL;
    CardResponse[SlotId].ActivationParams = (ActivationParams_t *)&IccEMVActivationParams;

    Ctx->RxBuffer       = CardResponse[SlotId].RxBuffer;
    Ctx->RxLength       = 0;
    Ctx->ScratchPadBuff = CardResponse[SlotId].ScratchPadBuffer;

    ret = UartInit(SCI_0, Ctx); /*MAX3255x have only one UART interface */
    if (ret)
        return ret;

    return AfeInit((MAX3255xSlots_t)SlotId);
}


/** @fn         SCAPI_close
 *  @brief      close a Slot
 *
 *  @param [in]  SlotId      Slot (AFE) Id.
 *
 *  @return     returns an #IccReturn_t error code
 *  @note       This function is available only for COBRA targets
 */
IccReturn_t SCAPI_close(uint8_t SlotId)
{
    if (MAX3255x_SLOT_NUMBER <= SlotId)
        return ICC_ERR_BAD_SLOT;

    /*unregister is not yet implemented */
    CardResponse[SlotId].pSendWTE = NULL;
    return ICC_OK;
}


/** @fn         SCAPI_write
 *  @brief      send an APDU to the card
 *
 *  @param [in]  SlotId      Slot (AFE) Id.
 *  @param [in]  pBuffer     pointer on the command buffer (APDU to send)
 *  @param [in]  length      length of the APDU to send (in bytes)
 *
 *  @return     returns an #IccReturn_t error code
 *  @note       This function is available only for COBRA targets
 */
IccReturn_t SCAPI_write(uint8_t SlotId, uint8_t *pBuffer, uint16_t length)
{
    SlotContext_t  *Ctx;
    IccReturn_t    ret = ICC_OK;

    if (MAX3255x_SLOT_NUMBER <= SlotId)
        return ICC_ERR_BAD_SLOT;

    Ctx = OSWrapper_getSlotCtxbyId(SlotId);

    if (NULL == Ctx) {
        return ICC_ERR_NULL_PTR;
    }

    ret = IccCheckCardState(Ctx);

    if (ICC_OK != ret) {
        return ret;
    }
		
    ret = IccExchange(SlotId,
                    pBuffer, length,
                    Ctx->RxBuffer,
                    &Ctx->RxLength,
                    CardResponse[SlotId].pSendWTE);

    return ret;
}

/** @fn         SCAPI_read
 *  @brief      read the response from the card
 *
 *  @param [in]  SlotId      Slot (AFE) Id.
 *  @param [out] pBuffer     pointer on the answer buffer
 *  @param [in]  plength      pointer on the length to read
 *  @param [out] plength     pointer on the read length
 *
 *  @return     returns an #IccReturn_t error code
 */
IccReturn_t SCAPI_read(uint8_t SlotId, uint8_t *pBuffer, uint32_t *plength)
{
    SlotContext_t  *Ctx;

    if (MAX3255x_SLOT_NUMBER <= SlotId)
        return ICC_ERR_BAD_SLOT;

    Ctx = OSWrapper_getSlotCtxbyId(SlotId);

    if (NULL == Ctx) {
        return ICC_ERR_NULL_PTR;
    }

    if (NULL == pBuffer) {
        return ICC_ERR_NULL_PTR;
    }

    if (*plength > Ctx->RxLength)
        *plength = Ctx->RxLength;

    memcpy(pBuffer, Ctx->RxBuffer, *plength);

    return ICC_OK;
}

/** @fn         SCAPI_ioctl
 *  @brief      Set driver parameters and get driver state
 *
 *  @param [in]  SlotId      Slot (AFE) Id.
 *  @param [in]  control     #sc_ioctl_t request
 *  @param [in]  pparam      pointer on the parameter to set/get
 *
 *  @return     returns an #IccReturn_t error code
 *  @note       This function is available only for COBRA targets
 */
IccReturn_t SCAPI_ioctl(uint8_t SlotId, sc_ioctl_t control, void *pparam)
{
    SlotContext_t  *Ctx;
    IccReturn_t    retval = ICC_OK;

    if (MAX3255x_SLOT_NUMBER <= SlotId)
        return ICC_ERR_BAD_SLOT;

    Ctx = OSWrapper_getSlotCtxbyId(SlotId);
    if (NULL == Ctx) {
        return ICC_ERR_NULL_PTR;
    }

    if (NULL == pparam) {
        return ICC_ERR_NULL_PTR;
    }

    switch (control) {
        case IOCTL_RESET:
            /* set/release the reset signal*/
            if (*(CardPowerState_t *)pparam == RESET_DO) {
                /* put the card in reset mode */
                return IccPowerAfe(Ctx, RESET_DO);
            }

            if (ICC_OK != IccCheckCardState(Ctx)) {
                return ICC_ERR_POWERED;
            }

card_activate:
            /* otherwise, we call the Icc activate to get the ATR */
            retval = IccActivate(SlotId, Ctx->RxBuffer,
                                 &Ctx->RxLength,
                                 CardResponse[SlotId].ActivationParams);
            if (ICC_NEED_WARMRESET == retval) {
                /* the received ATR suggest to do a warm reset */
                CardResponse[SlotId].ActivationParams->IccWarmReset = bTRUE;
                retval = IccActivate(SlotId, Ctx->RxBuffer,
                                     &Ctx->RxLength,
                                     CardResponse[SlotId].ActivationParams);
            }
            return retval;

        case IOCTL_POWER_CARD:

  				/*power UP/DOWN the card Without releasing the RST signal*/
            if ((*(CardPowerState_t *)pparam != POWER_DOWN) &&
                (*(CardPowerState_t *)pparam != POWER_UP))  {
                return ICC_ERR_BAD_PARAMETER;
            }

						
            if (POWER_DOWN == *(CardPowerState_t *)pparam) {
                return IccPowerAfe(Ctx, *(CardPowerState_t *)pparam);
            } else {
                goto card_activate;
            }

        case IOCTL_SET_VOLTAGE:
ioctl_set_voltage:
            /* Set Card Voltage pparam is a pointer on a #IccVoltage_t value*/
            if (*(IccVoltage_t *)pparam > LAST_ALLOWED_VOLTAGE) {
                return ICC_ERR_BAD_PARAMETER;
            }

            if (ICC_OK == IccCheckCardState(Ctx)) {
                return ICC_ERR_POWERED;
            }

            return IccSetAfeVoltage(Ctx, *(IccVoltage_t *)pparam);

        case IOCTL_SET_INITPARAMS:
            /*
             * Set Init params (used during the card activation),
             * pparams points on a #ActivationParams_t struct
             */
             CardResponse[SlotId].ActivationParams = (ActivationParams_t *)pparam;
             pparam = &CardResponse[SlotId].ActivationParams->IccVCC;
             goto ioctl_set_voltage;

        case IOCTL_SET_EMVMODE:
            /* Set the Stack Mode (EMV if *pparam is not 0) */
            if (ICC_OK == IccCheckCardState(Ctx)) {
                return ICC_ERR_POWERED;
            }

            if (0 == *(int32_t *)pparam) {
                Ctx->isEMV = bFALSE;
            } else {
                Ctx->isEMV = bTRUE;
            }
        break;

        case IOCTL_SET_FIDI:
            /* Change (force) the FIDI (TA1) value *USE WITH CAUTION* */
            /* the new parameter will be used with the next card exchange
             * note: there is not check on the value !
             */
            if (ICC_OK != IccCheckCardState(Ctx)) {
                /*
                 * if the card is not powered, you cannot adjust the
                 * baudrate
                 */
                return ICC_ERR_NOTPERMITTED;
            }

            Ctx->IccProtocolConfig.FiDi = *(uint8_t *)pparam;
        break;

        case IOCTL_SET_GUARD_TIME:
            /* Change (force) the GT value */
            /* the new parameter will be used with the next card exchange
             * note: there is not check on the value !
             */
            if (ICC_OK != IccCheckCardState(Ctx)) {
                /*
                 * if the card is powered, you cannot adjust the
                 * guard time
                 */
                return ICC_ERR_NOTPERMITTED;
            }
            Ctx->IccProtocolConfig.IccExtraGuardTime = *(uint16_t *)pparam;
        break;

        case IOCTL_SET_CWT:
            /*
             * Change (force) the CWT value (must be done after the
             * card activation and the PPS to have an effect)
             */
            if (ICC_OK != IccCheckCardState(Ctx)) {
                /*
                 * if the card is powered, you cannot adjust the
                 * char waiting time
                 */
                return ICC_ERR_NOTPERMITTED;
            }
            Ctx->IccProtocolConfig.IccCharWaitingTime = *(uint16_t *)pparam;
        break;

        case IOCTL_SET_BWT:
            /*
             * Change (force) the BWT value (must be done after the
             * card activation and the PPS to have an effect)
             */
            if (ICC_OK != IccCheckCardState(Ctx)) {
                /*
                 * if the card is powered, you cannot adjust the
                 * Block waiting time / Work Waiting time
                 */
                return ICC_ERR_NOTPERMITTED;
            }
            Ctx->IccProtocolConfig.IccWaitingTime = *(uint32_t *)pparam;
        break;

        case IOCTL_SET_WTE_HANDLER:
            /* set the user Wait time extension handler
             * (called only if not in EMV mode)
             */
            CardResponse[SlotId].pSendWTE = (void (*)(void)) pparam;
        break;

        case IOCTL_SET_PRESENCE_IRQ:
            /* set the user handler for the card status changes */
            Ctx->usr_cardstatusisr = (void (*)(CardState_t)) pparam;
        break;

        case IOCTL_REMOVE_PRESENCE_IRQ:
            Ctx->usr_cardstatusisr = NULL;
        break;

        case IOCTL_GET_FIDI:
            /* Returns the FIDI (TA1) current value  */
            *(uint32_t *)pparam = Ctx->IccProtocolConfig.FiDi;
        break;

        case IOCTL_GET_GUARD_TIME:
            /* Returns the GT value */
            if (ICC_OK != IccCheckCardState(Ctx)) {
                return ICC_ERR_NOTPERMITTED;
            }
            *(uint32_t *)pparam = Ctx->IccProtocolConfig.IccExtraGuardTime;
        break;

        case IOCTL_GET_CWT:
            /*
             * Returns the CWT value
             */
            if (ICC_OK != IccCheckCardState(Ctx)) {
                return ICC_ERR_NOTPERMITTED;
            }
            *(uint32_t *)pparam = Ctx->IccProtocolConfig.IccCharWaitingTime;
        break;

        case IOCTL_GET_WT:
            /*
             * Returns the BWT value
             */
            if (ICC_OK != IccCheckCardState(Ctx)) {
                return ICC_ERR_NOTPERMITTED;
            }
             *(uint32_t *)pparam = Ctx->IccProtocolConfig.IccWaitingTime;
        break;

        case IOCTL_GET_CARD_STATE:
             /* Returns the card state in *pparam, cf #IccReturn_t */
             *(uint32_t *)pparam = IccCheckCardState(Ctx);
        break;

        case IOCTL_GET_CARD_CLASS:
             /* Returns the card Class in *pparam, cf #IccReturn_t */
            if (ICC_OK != IccCheckCardState(Ctx)) {
                return ICC_ERR_NOTPERMITTED;
            }
             *(uint32_t *)pparam = Ctx->CardClass;
        break;

        case IOCTL_SET_CLOCK_FREQ:
        {
            uint32_t data, clockdiv, clockfreq;

            if (ICC_OK == IccCheckCardState(Ctx)) {
                /*
                 * if the card is powered, you cannot adjust the
                 * clock frequency
                 */
                return ICC_ERR_NOTPERMITTED;
            }

            clockfreq = *(uint32_t*)pparam;

            if ((clockfreq> SC_CLOCK_MAX) || (clockfreq < SC_CLOCK_MIN)) {
                return ICC_ERR_BAD_PARAMETER;
            }

            /*
             * clock divider formula
             * Fsmartcard = Fpll / SCCKvalue
             *
             * which gives
             * SCCKvalue = Fpll / Fsmartcard
             */
            clockdiv = (uint32_t) (96000000/clockfreq);

            data = *(uint32_t*)CLOCK_REG_SCCK;
            data &= ~SCCK_DIVIDER_MASK;

             /* Update GCR */
            *(uint32_t*)CLOCK_REG_SCCK = data | clockdiv;
        }
        break;

        default:
            return ICC_ERR_BAD_PARAMETER;
    }

    return ICC_OK;
}
