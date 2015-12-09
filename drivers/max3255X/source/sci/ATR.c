/*
 * ATR.c -- Smartcard stack Anwser To Reset Management.
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
#include "slot.h"
#include "iccabstract.h"
#include "ATR.h"
#include "PTS.h"
#include "sc_types.h"


/**  @var NB_BITS
 * @brief Use to get the number of bytes following a TDi byte\n
 *
 * The number of bytes to get is the number of bits set in the
 * most significant nibble of the TDx byte.
 *  */
const uint8_t NB_BITS[] = {0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4};


/** @var Di
 * @brief Di constants lookup array
 */
const uint8_t Di[16] = {
    1, 1, 2, 4, 8, 16, 32, 64, 12, 20, 1, 1, 1, 1, 1, 1
};                                  /**< Di Parameter lookup array */

/** @var Fi
 * @brief Fi constants lookup array
 */
const uint16_t Fi[16] = {
    372, 372, 558, 744, 1116, 1488, 1860, 0,
    0, 512, 768, 1024, 1536, 2048, 0, 0
};

/** @var IccEMVActivationParams
 * @brief declares the default initialisation values for the EMV activation.\n
 *        refer to #ActivationParams_t
 */
const ActivationParams_t   IccEMVActivationParams =
{
    .IccVCC                = VCC_5V,    /**< Start with 5 volts */
    .IccResetDuration      = 112,       /**< Reset delay in etus (39804 clock cycles) */
    .IccATR_Timeout        = 20160,     /**< ATR Total Time value (in etus) */
    .IccTS_Timeout         = 114,       /**< TS waiting time of 42400 clock cycles (114 etus) */
    .IccWarmReset          = bFALSE,    /**< no warm reset */
};

IccReturn_t GetATR(uint8_t slotid, uint8_t *pRxBuffer, uint32_t *pATRLength,
                   boolean_t ATRDeactivateOnError,
                   ActivationParams_t   *ActivationParams)
{
    uint8_t        ATRIndex;
    uint8_t        NbReceivedBytes;
    uint8_t        ExpectedBytesToReceive;
    uint8_t        TD[4] = {0,0,0,0};
    uint8_t        ProtocolTD1 = 0;
    uint8_t        ProtocolTD2 = 0;
    uint8_t        Fi;
    uint8_t        FiDi = 0x11;      /* TA1 byte default value */
    int32_t        i = 0;
    boolean_t      T15Protocol = bFALSE;
    boolean_t      IsTC1Equal255 = bFALSE;
    boolean_t      NegociableMode = bFALSE;
    IccReturn_t    ATR_Status = ICC_OK;
    IccRequest_t   RxRequest;
    SlotContext_t  *SlotCtx;

    SlotCtx = IccSlotGetConfiguration(slotid);
    if (!SlotCtx)
        return ICC_ERR_BAD_SLOT;

    if ((SlotCtx->isEMV) && (!ATRDeactivateOnError))
        return ICC_ERR_BAD_PARAMETER;


    /****************************************************************
     *                ATR is stored in the RxBuffer
     ****************************************************************/

    /* Initialize IccReceive structure */
    RxRequest.IccData     = pRxBuffer;
    RxRequest.IccLen      = 34;         /*ATR length is max 33 bytes, 34 is an error case*/
    RxRequest.IccLastByte = bFALSE;
    RxRequest.IccEDC      = 0;
    RxRequest.IccStatus   = ICC_OK;

    ATRIndex       =  1;
    SlotCtx->IccProtocolConfig.PossibleFiDi = 0x11;


    /* configure initial values */
    SlotCtx->InitialParams.Di   =  1;
    SlotCtx->InitialParams.CWI  =  0;   /*TB3 least significant nible (default value)*/
    SlotCtx->InitialParams.BWI  =  0;    /*TB3 most significant nible (default value)*/
    SlotCtx->InitialParams.IFSC =  32;   /*TA1 ISO/EMV's default value */
    SlotCtx->InitialParams.EGT  =  0;    /*Default Extra Guard time is 0 etu*/
    SlotCtx->InitialParams.WWI  =  10;   /* Default value of Work Wait Initial is 10 etus*/

    SlotCtx->IccProtocolParams.IccProtocol              = 0; /*T=0*/
    SlotCtx->IccProtocolConfig.InverseConvention        = bFALSE;
    SlotCtx->IccProtocolConfig.IccExtraGuardTime        = 12, /* EGT = 12 etus */
    SlotCtx->IccProtocolConfig.IccBlockGuardTime        = BGTT0, /* BGT = 16 etus */
    SlotCtx->IccProtocolConfig.IccCharWaitingTime       = 17, /* CWI = 15 etus*/
    SlotCtx->IccProtocolConfig.IccWaitingTime           = 10080, /* For EMV WWT =  9600 +5% etus */

    /* Prepare for ATR reception */
    NbReceivedBytes        = 0;

    /*we expect at least TS and T0 */
    ExpectedBytesToReceive = 2;

    /* Check if card is present and activated */
    if (IccCheckCardState(SlotCtx) == ICC_ERR_REMOVED) {
        return ICC_ERR_REMOVED;
    }

    /* Initiate Activation sequence */
    ATR_Status = IccStartActivation (SlotCtx, ActivationParams, &RxRequest);

    if ((ATR_Status != ICC_OK)) {
        /*
         * If got an ATR timeout or parity error,
         * deactivate according to EMV spec.
         */
        if ( (ATR_Status == ICC_ERR_TIMEOUT) ||
             (ATR_Status == ICC_ERR_PARITY) ) {
            /* No ATR, so deactive the contacts */
            IccDeactivate(SlotCtx);
            return ICC_ERR_ATR_TIMEOUT;
        }
        return ATR_Status;
    }

    /* Get TS and T0 bytes */
    while (NbReceivedBytes < 2) {
        NbReceivedBytes = IccGetRxLen(SlotCtx, &ATR_Status);

        if ((ATR_Status != ICC_RX_PENDING) && (ATR_Status != ICC_OK)) {
            IccDeactivate(SlotCtx);
            return ATR_Status;
        }

        if (NbReceivedBytes > 0) {
            if ( (pRxBuffer[0] != 0x3F) && (pRxBuffer[0] != 0x3B) ) {
                /* No ATR, so deactive the contacts */
                IccDeactivate(SlotCtx);
                return ICC_ERR_BAD_TS;
            }
        }

        /* Check if card is present and activated */
        ATR_Status = IccCheckCardState(SlotCtx);
        if ( ATR_Status == ICC_ERR_REMOVED)
            return ICC_ERR_REMOVED;
    }

    /* TS= 0x3F -> inverse, 0x3B -> direct*/
    if (pRxBuffer[0] == 0x3F) {
        SlotCtx->IccProtocolConfig.InverseConvention = bTRUE;
    }

    /* T0 byte */
    TD[0] = pRxBuffer[1];
    ExpectedBytesToReceive = 2 + NB_BITS[(TD[0] & 0xF0) >> 4];

    /* Reception of TA1 TB1 TC1 and TD1*/
    while (ExpectedBytesToReceive > NbReceivedBytes) {
        NbReceivedBytes = IccGetRxLen(SlotCtx, &ATR_Status);

        if ((ATR_Status != ICC_RX_PENDING) && (ATR_Status != ICC_OK))
            return ATR_Status;

        /* Check if card is present and activated */
        if (IccCheckCardState(SlotCtx) == ICC_ERR_REMOVED)
            return ICC_ERR_REMOVED;
    }

    /* Is there a TD1, if so determine the next number bytes to expect */
    if ( (TD[0] & TD_BIT) == TD_BIT) {
        TD[1]    = pRxBuffer[ExpectedBytesToReceive - 1];
        ExpectedBytesToReceive += NB_BITS[(TD[1] & 0xF0) >> 4];

        while (ExpectedBytesToReceive > NbReceivedBytes) {
            NbReceivedBytes = IccGetRxLen(SlotCtx, &ATR_Status);

            if ((ATR_Status != ICC_RX_PENDING) && (ATR_Status != ICC_OK))
                return ATR_Status;

            /* Check if card is still present and activated */
            if (IccCheckCardState(SlotCtx) == ICC_ERR_REMOVED)
                return ICC_ERR_REMOVED;

        }

        /* Is there a TD2, if so determine the next number of bytes to expect */
        if ( (TD[1] & TD_BIT) == TD_BIT) {
            TD[2]    = pRxBuffer[ExpectedBytesToReceive - 1];
            ExpectedBytesToReceive += NB_BITS[(TD[2] & 0xF0) >> 4];

            while (ExpectedBytesToReceive > NbReceivedBytes) {
                NbReceivedBytes = IccGetRxLen(SlotCtx, &ATR_Status);

                if ((ATR_Status != ICC_RX_PENDING) && (ATR_Status != ICC_OK))
                    return ATR_Status;

                /* Check if card is present and activated */
                if (IccCheckCardState(SlotCtx) == ICC_ERR_REMOVED)
                    return ICC_ERR_REMOVED;

            }

            /*Is there a TD3, if so determine the next number of bytes to expect */
            if ( (TD[2] & TD_BIT) == TD_BIT) {
                TD[3]    = pRxBuffer[ExpectedBytesToReceive - 1];
                ExpectedBytesToReceive += NB_BITS[(TD[3] & 0xF0) >> 4];

                while (ExpectedBytesToReceive > NbReceivedBytes) {
                    NbReceivedBytes = IccGetRxLen(SlotCtx, &ATR_Status);

                    if ((ATR_Status != ICC_RX_PENDING) && (ATR_Status != ICC_OK))
                        return ATR_Status;

                    /* Check if card is still present and activated */
                    if (IccCheckCardState(SlotCtx) == ICC_ERR_REMOVED)
                        return ICC_ERR_REMOVED;
                }
            }

        }
    }

    /* Reception of the historic bytes */
    ExpectedBytesToReceive += (TD[0] & 0x0F);

    do
    {
        NbReceivedBytes = IccGetRxLen(SlotCtx, &ATR_Status);

        if ((ATR_Status != ICC_RX_PENDING) && (ATR_Status != ICC_OK)) {
            /* we got an error */
            break;
        }

        /* Check if card is still present and activated */
        if (IccCheckCardState(SlotCtx) == ICC_ERR_REMOVED) {
            return ICC_ERR_REMOVED;
        }
    } while ( (ExpectedBytesToReceive > NbReceivedBytes) );

    if (ATR_Status == ICC_ERR_PARITY)
        return ATR_Status;

    /* protocol in byte TD1 - also called 'First Offered Protocol' */
    ProtocolTD1 = TD[1] & 0x0F;

    /* second protocol in byte TD2 */
    ProtocolTD2 = TD[2] & 0x0F;

    if ( (ProtocolTD1 > 0) || (ProtocolTD2 > 0) ) {
        /* Wait for a TCK byte if protocol is different from T=0 */
        ExpectedBytesToReceive++;
        do
        {
            NbReceivedBytes = IccGetRxLen(SlotCtx, &ATR_Status);

            if ((ATR_Status != ICC_RX_PENDING) && (ATR_Status != ICC_OK))
                break;

            /* Check if card is present and activated */
            if (IccCheckCardState(SlotCtx) == ICC_ERR_REMOVED) {
                return ICC_ERR_REMOVED;
            }
        } while ( (ExpectedBytesToReceive > NbReceivedBytes));

        if (ATR_Status == ICC_ERR_PARITY)
            return RxRequest.IccStatus;

        /* The reception is finished (TCK received) */
        IccRxDone(SlotCtx);
        ATR_Status = ICC_OK;

        /****************************************************************
         *               CHECK TCK BYTE
         ****************************************************************/
        /* for the ATR (as for T=0) the EDC is not computed, we must do
         * it there
         */

        RxRequest.IccEDC = 0;
        for (i = 1; i<RxRequest.IccReceived; i++) {
                RxRequest.IccEDC ^= RxRequest.IccData[i];
        }

        /* Check the TCK of the received ATR */
        if (RxRequest.IccEDC != 0) {
            /* Error bad TCK */
            return ICC_ERR_BAD_TCK;
        }

    } else {
        /* The reception is finished (no TCK) */
        IccRxDone(SlotCtx);
        ATR_Status = ICC_OK;
    }

    if (0 == RxRequest.IccLen) {
        /* we received more than 32 bytes
         * which is an error case !
         */

         return ICC_ERR_BAD_ATR_VALUE;
    }

    /****************************************************************
     *          CHECK TS BYTE
     ****************************************************************/
    if ((pRxBuffer[0] != 0x3B) && (pRxBuffer[0] != 0x3F)) {
        /* Error bad TS */
        IccDeactivate(SlotCtx);
        return ICC_ERR_BAD_TS;
    }

    /****************************************************************
     *    CHECK ATR length... card may return more/extra ATR bytes
     ****************************************************************/
    if ( NbReceivedBytes > ExpectedBytesToReceive  ) {
        return  ICC_ERR_BAD_ATR_VALUE;
    }
    /****************************************************************
     *          T0 BYTE
     ****************************************************************/

    ATRIndex = 2;

    /****************************************************************
     *          TA1 BYTE
     ****************************************************************/
    if ((TD[0] & TA_BIT) == TA_BIT) {
        FiDi = pRxBuffer[ATRIndex++];
        Fi = (FiDi & 0xF0) >> 4;

        /* Reject ATR if Fi or Di = 0 */
        if ( ((FiDi & 0x0F) == 0) || (Fi == 0) ) {
            return ICC_ERR_BAD_ATR_VALUE;
        }

        SlotCtx->IccProtocolConfig.PossibleFiDi = FiDi;

        /*
         *  FiDi is not set at this moment
         *  (default value of 0x11 is used for the moment)
         */
        if (bFALSE == ActivationParams->IccWarmReset) {
            NegociableMode = bTRUE;
        }

    }

    /****************************************************************
     *          TB1 BYTE
     ****************************************************************/
    if ((TD[0] & TB_BIT) == TB_BIT) {
        if (pRxBuffer[ATRIndex++] & 0x1F) {

          if ((bFALSE == ActivationParams->IccWarmReset) &&
             (bTRUE == SlotCtx->isEMV)) {
             /* if EMV and cold reset but TB1&0x1F is not 0,
              * reject the ATR
              */
            return ICC_NEED_WARMRESET;
          }
           /*if it's a warm reset, we do not reject the card */
       }
    } else {
        /* no TB1 byte */
         if ((bFALSE == ActivationParams->IccWarmReset) &&
             (bTRUE == SlotCtx->isEMV)) {
             /* if EMV and cold reset but no TB1,
              * reject the ATR
              */
            return ICC_NEED_WARMRESET;
          }
    }

    /****************************************************************
     *          TC1 BYTE
     ****************************************************************/
    if ((TD[0] & TC_BIT) == TC_BIT) {
        /*
         * This case is applicable to EMV (12 <= CGT <= 255).
         * ISO for T=1 CGT is 11 minimally
         */
        NbReceivedBytes = pRxBuffer[ATRIndex++];

        if (NbReceivedBytes != 0xFF) {
            SlotCtx->InitialParams.EGT = NbReceivedBytes + 1;
            SlotCtx->IccProtocolConfig.IccExtraGuardTime = NbReceivedBytes + 12;
        } else {
            SlotCtx->InitialParams.EGT = 0;
            IsTC1Equal255 = bTRUE;
        }
    }

    /****************************************************************
     *          TD1 BYTE
     ****************************************************************/
    if ((TD[0] & TD_BIT) == TD_BIT) {
        ATRIndex++;

        if ((ProtocolTD1 > 1)&&(bTRUE == SlotCtx->isEMV)) {
            /* In EMV, if TD1 different from 0, 1, reject ATR */
            goto check_if_warmreset;
        }


        /****************************************************************
         *          TA2 BYTE
         ****************************************************************/
        if ((TD[1] & TA_BIT) == TA_BIT) {
            /* Specific mode byte */
            NbReceivedBytes = pRxBuffer[ATRIndex++];
            NegociableMode = bFALSE;

            /*
             * The Fi and Di parameters of TA1 are selected
             * if bit b5 is set to 0
             */
            if ((NbReceivedBytes & 0x10) == 0) {
                /*
                 * Bit 5 of TA2 is equal to 0, (explicit mode)
                 * so use the Fi and Di in TA1 immediatly after ATR
                 */
                SlotCtx->IccProtocolConfig.FiDi = FiDi;
                SlotCtx->InitialParams.Di = Di[FiDi & 0x0F];
            } else if (bTRUE == SlotCtx->isEMV) {
                /*TA2.5 is set, in EMV we reject the card*/
                goto check_if_warmreset;
            }

            /*
             * Protocol to use is the one defined in TA2 low byte
             * if in ISO mode
             */
            if (bFALSE == SlotCtx->isEMV) {
                ProtocolTD1 = NbReceivedBytes & 0x0F;
            } else {
                /* EMV mode */
                if (ProtocolTD1 != (NbReceivedBytes & 0x0F)) {
                    /*the TA2 proposed protocol is not valid */
                    goto check_if_warmreset;
                }
            }
        } else {
            /*
             * TA2 is missing -> Negotiable mode.
             * Do not change timing. Let PPS request take care of it.
             */
             NegociableMode = bTRUE;
        }

        /****************************************************************
         *          TB2 BYTE
         ****************************************************************/
        if ((TD[1] & TB_BIT) == TB_BIT) {
            if (bFALSE == SlotCtx->isEMV) {
                /* ISO 7816 */
                ATRIndex++;
            } else {
                /* in EMV if TB2 is present, we reject the ATR */
                goto check_if_warmreset;
            }
        }

        /****************************************************************
         *          TC2 BYTE
         ****************************************************************/
        if ((TD[1] & TC_BIT) == TC_BIT) {
            /* WWI value */
            SlotCtx->InitialParams.WWI = pRxBuffer[ATRIndex++];
            if ((SlotCtx->InitialParams.WWI == 0) &&
                (bTRUE == SlotCtx->isEMV)) {
                /* EMV Only : Reject ATR if WWI = 0  */
                goto check_if_warmreset;
            }
        }

        /****************************************************************
         *          TD2 BYTE
         ****************************************************************/
        if ((TD[1] & TD_BIT) == TD_BIT) {
            /* TD2 value */
            ATRIndex++;
            if (bFALSE == SlotCtx->isEMV) {
                /* ISO 7816 mode */
                /* If TD2 different from 0, 1, 14, 15 reject ATR */
                if ((ProtocolTD2 > 1) && (ProtocolTD2 < 14)) {
                    return ICC_ERR_BAD_ATR_VALUE;
                }

                if (ProtocolTD2 == 0x0F) {
                    T15Protocol = bTRUE;
                }

            } else {

                /* EMV Mode */
                if ((ProtocolTD1 == 0 ) &&
                    (ProtocolTD2 != 1) && (ProtocolTD2 != 14)) {
                    /*
                     * Reject ATR if TD1 gives T0 and
                     * TD2 is neither T=1 or T=14
                     */
                    goto check_if_warmreset;
                }

                if ((ProtocolTD1 == 1 ) &&
                    (ProtocolTD2 != 1)) {
                    /* Reject ATR if TD1 gives T=1 and TD2 is not T=1  */
                    goto check_if_warmreset;
                }
            }

            /****************************************************************
             *          TA3 BYTE
             ****************************************************************/
            if ((TD[2] & TA_BIT) == TA_BIT) {
                if (T15Protocol == bFALSE) {
                    /* TA3 value gives the IFSC */
                    SlotCtx->InitialParams.IFSC = pRxBuffer[ATRIndex++];
                } else {
                    /* int T=15, TA3 value gives the voltage class */
                    SlotCtx->CardClass = pRxBuffer[ATRIndex++];
                }

                if (bFALSE == SlotCtx->isEMV) {
                    /* ISO 7816 mode */
                    if ((SlotCtx->InitialParams.IFSC == 0) ||
                        (SlotCtx->InitialParams.IFSC == 0xFF)) {
                        /* Reject ATR if IFSC is either 0 or 0xFF */
                        ATR_Status |= ICC_ERR_BAD_ATR_VALUE;
                    }
                } else {
                    /* EMV Mode */
                    if ((SlotCtx->InitialParams.IFSC < 0x10) ||
                        (SlotCtx->InitialParams.IFSC == 0xFF)) {
                        /*
                         *  Reject ATR if IFSC is less than 16bytes
                         *  or 255
                         */
                        goto check_if_warmreset;
                    }
                }
            }

            /****************************************************************
             *          TB3 BYTE
             ****************************************************************/
            if ((TD[2] & TB_BIT) == TB_BIT) {
                /*
                 * TB3 value gives intial values for the character
                 * and block waiting times (CWI and BWI)
                */
                NbReceivedBytes = pRxBuffer[ATRIndex++];

                /* set CWI and BWI values */
                SlotCtx->InitialParams.CWI = NbReceivedBytes & 0x0F;
                SlotCtx->InitialParams.BWI = (NbReceivedBytes & 0xF0) >> 4;

                /* Check CWI and BWI values */
                if (bTRUE == SlotCtx->isEMV) {
                             /* EMV only */
                    if ((SlotCtx->InitialParams.BWI > 4) ||
                        (SlotCtx->InitialParams.CWI > 5))  {
                        /* Reject ATR if BWI > 4 or CWI > 5*/
                        goto check_if_warmreset;
                    }

                    if ((1 << SlotCtx->InitialParams.CWI) <=
                        (SlotCtx->InitialParams.EGT)) {
                        /* Reject ATR if (2^CWI) <= IccExtraGuardTime (according to EMV) */
                        goto check_if_warmreset;
                    }
                }
            } else if ((bTRUE == SlotCtx->isEMV) && (ProtocolTD1 == 1)) {
                        /* In EMV Mode, if we are on T=1 but
                         * TB3 is not present, reject the ATR */
                        goto check_if_warmreset;
            }

            /****************************************************************
             *          TC3 BYTE
             ****************************************************************/
            if ((TD[2] & TC_BIT) == TC_BIT) {
                /* TC3 value = error detection mode */
                NbReceivedBytes = pRxBuffer[ATRIndex++];

                if (bFALSE == SlotCtx->isEMV) {
                    /* ISO 7816 : LRC and CRC are managed */
                    if (NbReceivedBytes == 1) {
                        /* Error Detection Code (EDC) is CRC */
                        SlotCtx->IccProtocolParams.IccEDCTypeCRC = bTRUE;
                    } else if (NbReceivedBytes == 0) {
                        /* default value is bFALSE (LRC) */
                        SlotCtx->IccProtocolParams.IccEDCTypeCRC = bFALSE;
                    } else {
                            /* Reject ATR if TC3 > 1 */
                            return ICC_ERR_BAD_ATR_VALUE;
                    }

                    if ((NbReceivedBytes == 1) && (ProtocolTD1 == 0)) {
                        /* Reject ATR if TC3 > 1 and offered protocol is not T=1 */
                        return ICC_ERR_BAD_ATR_VALUE;
                    }
                } else {
                    /* EMV (Only LRC is managed) */
                    if (NbReceivedBytes == 0) {
                         /* Error Detection Code (EDC) is LRC */
                        SlotCtx->IccProtocolParams.IccEDCTypeCRC = bFALSE;
                    } else {
                        /* Reject ATR if TC3 > 0 */
                        goto check_if_warmreset;
                    }

                }
            }

            /****************************************************************
             *          TD3 BYTE
             ****************************************************************/
            if ((TD[2] & TD_BIT) == TD_BIT) {
                ATRIndex++;

                /*************************************************************
                 *          TA4 BYTE
                 **************************************************************/
                if ((TD[3] & TA_BIT) == TA_BIT) {
                    if ((TD[3] & 0x0F) == 0x0F) {
                        /* TA4 value gives the voltage class */
                        SlotCtx->CardClass = pRxBuffer[ATRIndex++];
                    }
                }

                /*************************************************************
                 *          TB4 BYTE
                 **************************************************************/
            }
        } else if ((bTRUE == SlotCtx->isEMV) && (ProtocolTD1 == 1)) {
            /* T= 1 but no TB3, reject the ATR */
            goto check_if_warmreset;
        }
    }

    /****************************************************************
     *          ATR RECEPTION END
     ****************************************************************/

    *pATRLength = ExpectedBytesToReceive;


    if ((ATRDeactivateOnError) && (ATR_Status != ICC_OK)) {
        /* If ATRDeactivateOnError bit is set,
         * deactivate when ATR is invalid */
        IccDeactivate(SlotCtx);

        return ATR_Status;
    }

    /****************************************************************
     *          SET NEW PARAMETERS TO THE INTERFACE
     ****************************************************************/

    SlotCtx->IccProtocolParams.IccProtocol = ProtocolTD1 & 0x01;


    /* set the timing values for T=0 and PPS exchange */
    /* EDC is not enabled when using T=0 */
    SlotCtx->IccProtocolConfig.IccBlockGuardTime = BGTT0; /*according to EMV */

    /*
     * WWT, Default is 10080 etus (WWI=10, Di = 1 until the PPS is accepted)
     *
     * IccWorkWaitingTime Min = (960 * WI * Di)
     * IccWorkWaitingTime Max = (960 * WI * Di) + 9600 * Di
     *
     * Here we set IccWorkWaitingTime to (960 * WI * Di) + 480 * Di + 10
     */
    SlotCtx->IccProtocolConfig.IccWaitingTime  = 960*SlotCtx->InitialParams.WWI;
    SlotCtx->IccProtocolConfig.IccWaitingTime += 480;
    SlotCtx->IccProtocolConfig.IccWaitingTime *= SlotCtx->InitialParams.Di;
    SlotCtx->IccProtocolConfig.IccWaitingTime += 10;


    /*
     * if the user wants an automatic PPS negociation and either :
     * - the card provides TA1 which is not 0x11 (default)
     * - or we are on ISO mode and in negociable mode
     */
    if ((bTRUE == SlotCtx->IccProtocolConfig.AutoPPS) && (
        (SlotCtx->IccProtocolConfig.PossibleFiDi != SlotCtx->IccProtocolConfig.FiDi) ||
        ((bTRUE == NegociableMode) && (bFALSE == SlotCtx->isEMV)) ) ) {
        /*
         * we got a first ATR, we did the warm reset,
         * now process the PPS
         */
        /* if it's possible to use non default TA1*/
        ATR_Status = IccPTSNegotiate(slotid, bTRUE);

        /* set the new Di value */
        SlotCtx->InitialParams.Di = Di[SlotCtx->IccProtocolConfig.FiDi & 0x0f];
    }


    if (SlotCtx->IccProtocolParams.IccProtocol == 0) {
        /* T=0 protocol parameters */

        /* EDC is not enabled when using T=0 */
        SlotCtx->IccProtocolConfig.IccBlockGuardTime = BGTT0; /*according to EMV */

        /*
         * Extra Guard Time
         * If TC1 different from 0xFF, already set
         * Otherwise = 12, also already set (default value)
         */

        /* IFSC, CWI, BWI are not used in T=0 */

        /*
         * WWT values:
         * IccWorkWaitingTime Min = (960 * WI * Di)
         * IccWorkWaitingTime Max = (960 * WI * Di) + 9600 * Di
         *
         * IccWorkWaitingTime = (960 * WWI * Di) + (480 * Di) + 10
         * IccWorkWaitingTime = (960 * WWI + 480) * Di + 10
         */
        SlotCtx->IccProtocolConfig.IccWaitingTime  = 960*SlotCtx->InitialParams.WWI;
        SlotCtx->IccProtocolConfig.IccWaitingTime += 480;
        SlotCtx->IccProtocolConfig.IccWaitingTime *= SlotCtx->InitialParams.Di;
        SlotCtx->IccProtocolConfig.IccWaitingTime += 10;


    } else {

        /* T=1 protocol parameters
         *
         * WWI, Break generation, detection are not used in T=1
         * EDC is enabled by default when using T=1
         */

        /* Extra Guard Time
         * If TC1 different from 0xFF, already set
         * Otherwise EGT = 11 ETUs
         */
        if (IsTC1Equal255 == 1) {
            SlotCtx->IccProtocolConfig.IccExtraGuardTime = 11;
        }

        /* BWT = 2^BWI * 960 * D + 11
         * according to EMV book: BWT response time = BWT + (D*960) + 1
         * according to test case 1789: Max BWT = BWT + D*(960 + 480)
         *
         * Note : the EMV test 1789_11 check that BWT is less than:
         * BWT = (960*2^BWI) + D*(960+240) + 11 ETUs.
         */

        SlotCtx->IccProtocolConfig.IccWaitingTime = 960 * (1 << SlotCtx->InitialParams.BWI);
        SlotCtx->IccProtocolConfig.IccWaitingTime += 960 + 240 ;
        SlotCtx->IccProtocolConfig.IccWaitingTime *= SlotCtx->InitialParams.Di;
        SlotCtx->IccProtocolConfig.IccWaitingTime += 11;

        SlotCtx->IccProtocolConfig.IccBlockGuardTime = BGTT1; /*according to EMV*/
    }

    /*
     * CWI BWI have to be set
     * (Used only when T=1 is selected)
     *
     * CWT = (2^CWI) + 11 + 5
     *
     * Note: in T=0, CWT is also used as a time to wait before deactivation of
     * the card.
     *
     * Note : the max value is theorical value +1
     */
    SlotCtx->IccProtocolConfig.IccCharWaitingTime = (1 << SlotCtx->InitialParams.CWI);
    SlotCtx->IccProtocolConfig.IccCharWaitingTime += 11 + 5;

    SlotCtx->IccProtocolParams.IccIFSC = SlotCtx->InitialParams.IFSC;

    return ATR_Status;



check_if_warmreset:
    /*
     * for EMV, try to do a warm reset,
     * if already done, reject the card with an error
     */
    if (bFALSE == ActivationParams->IccWarmReset) {
        return ICC_NEED_WARMRESET;
    } else {
        return ICC_ERR_BAD_ATR_VALUE;
    }
}

