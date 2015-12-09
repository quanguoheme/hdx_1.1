/*
 * MAX3255x.h -- MAX32550 & MAX32555 on-SoC Analog Front End
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
#include <mml_gpio.h>
#include "sc_errors.h"
#include "sc_states.h"
#include "sc_config.h"
#include "sc_regs.h"
#include "iccabstract.h"
#include "OSWrapper.h"
#include "MAX3255x_afe.h"
#include "MAX3255x_afe_private.h"
#include "slot.h"

#define MAX32550_EVKIT

#ifdef MAX32550_EVKIT

#define MAX32550_EVKIT_SAM_VCC5_PIN 		14
#define MAX32550_EVKIT_SAM_VCC3_PIN 		15
#define MAX32550_EVKIT_SAM_CS_PIN 			20
#define MAX32550_EVKIT_SAM_VCC_PIN_PORT MML_GPIO_DEV0
#define MAX32550_EVKIT_SAM_AFE_RDY 			26
#define MAX32550_EVKIT_SAM_AFE_RDY_PORT MML_GPIO_DEV0

#endif

/**** NOTE: this AFE is specific to the MAX32550 and MAX32555 ****/

/** @fn                     AfeSelect
 *  @brief                  Select (enable) an AFE (Analog Front End)
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] Selected     if #bTRUE, we select the AFE (we enable)
 *
 *  @return                         return an #IccReturn_t error code
 *  @retval ICC_ERR_BAD_PARAMETER   if is the slot number is out of range
 *  @retval ICC_ERR_NULL_PTR        if the SlotCtx is #NULL
 *  @retval ICC_OK                  if the AFE is now selected (enabled)
 */
static IccReturn_t AfeSelect(SlotContext_t  *SlotCtx, boolean_t Selected)
{
    SCControl_t  sccr = {.word = 0};  /* control register value */
    UartState_t *UartState = NULL;

    if ((NULL == SlotCtx) || (NULL == UartState->UartAddress)) {
        return ICC_ERR_NULL_PTR;
    }

    UartState = (UartState_t *)(SlotCtx->UartData->PrivateData);
    if (NULL == UartState) {
        return ICC_ERR_NULL_PTR;
    }

    /* check if the requested slot is available on our target */
    if (MAX3255x_SLOT_NUMBER <= SlotCtx->SlotId) {
        return ICC_ERR_BAD_SLOT;
    }

    OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);

    switch (SlotCtx->SlotId) {
    case SAM_SLOT:
        if (bTRUE == Selected) {
            sccr.bits.BYP_PHY   = 1;
        } else {
            sccr.bits.BYP_PHY   = 0;
        }
    break;

    case SMARTCARD_SLOT:
        /*whatever the Selected value, we stay on the SMARTCARD */
        sccr.bits.BYP_PHY   = 0;
    break;

    default:
        return ICC_ERR_BAD_PARAMETER;
    }

    OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

    UartState->ActiveSlot = SlotCtx->SlotId;

    return ICC_OK;
}

/** @fn                     AfeSetVoltage
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
static IccReturn_t AfeSetVoltage(SlotContext_t  *SlotCtx, IccVoltage_t Voltage)
{
    SCControl_t          sccr  = {.word = 0};  /* control register value */
    SCPin_t              scpin = {.word = 0};
    UartState_t *UartState = NULL;

    if ((NULL == SlotCtx) || (NULL == UartState->UartAddress)) {
        return ICC_ERR_NULL_PTR;
    }

    /* check if the requested slot is available on our target */
    if (MAX3255x_SLOT_NUMBER <= SlotCtx->SlotId)
        return ICC_ERR_BAD_SLOT;

    UartState = (UartState_t *)(SlotCtx->UartData->PrivateData);
    if (NULL == UartState) {
        return ICC_ERR_NULL_PTR;
    }

    if (SAM_SLOT == SlotCtx->SlotId) {
        /* not managed for the SAM slot,
         * return OK to avoid error propagation and cancelation of
         * the power sequence
         */
        return ICC_OK;
    }

    OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);
    OSWrapper_ReadReg(UartState->UartAddress + SC_PN, &scpin.word);

    /* check if the card is already powered */
    if (bTRUE == sccr.bits.START) {
        /* if the card is already powered, we cannot change the voltage ! */
        return ICC_ERR_POWERED;
    }

    switch (Voltage) {
    case VCC_5V:
    case VCC_3V:
    case VCC_1V8:
        scpin.bits.VCCSEL = Voltage;
    break;

    default:
        return ICC_ERR_BAD_PARAMETER;
    }

    OSWrapper_WriteReg(UartState->UartAddress + SC_PN, scpin.word);

    return ICC_OK;
}

/** @fn                     AfePower
 *  @brief                  Turn On/Off the card
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] PowerUp      the requested state (ON/OFF/WARM_RESET, cf #CardPowerState_t)
 *
 *  @return                 return an #IccReturn_t error code
 *  @retval ICC_ERR_POWERED if the card session is already active
 *  @retval ICC_OK          if the AFE is now selected (enabled)
 *
 */
static IccReturn_t AfePower(SlotContext_t  *SlotCtx, CardPowerState_t PowerUp)
{
    SCControl_t          sccr  = {.word = 0};  /* control register value */
    SCPin_t              scpin = {.word = 0};
    SCISR_t              scisr = {.word = 0};
    SCStatus_t           scsr  = {.word = 0};
    UartState_t         *UartState = NULL;
    int32_t             Timeout = 1000;

    if ((NULL == SlotCtx) || (NULL == UartState->UartAddress)) {
        return ICC_ERR_NULL_PTR;
    }

    /* check if the requested slot is available on our target */
    if (MAX3255x_SLOT_NUMBER <= SlotCtx->SlotId)
        return ICC_ERR_BAD_SLOT;

    UartState = (UartState_t *)(SlotCtx->UartData->PrivateData);
    if (NULL == UartState) {
        return ICC_ERR_NULL_PTR;
    }

    OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);
    OSWrapper_ReadReg(UartState->UartAddress + SC_PN, &scpin.word);

    switch (PowerUp) {
    case POWER_DOWN:
        /* reset the Powering up bit
         * (this bit is set if we powered the card and we did not
         * received the first ATR byte)
         */
        if (SAM_SLOT == SlotCtx->SlotId) {
            /* we do not manage the SAM Slot power, but we can put
             * the SAM in reset
             */
            scpin.bits.CRDRST = RESET_ACTIVE;
			OSWrapper_WriteReg(UartState->UartAddress + SC_PN, scpin.word);

#ifdef MAX32550_EVKIT					
						//set VCC5 and VCC3 input to be high to keep sam card power down 
						mml_gpio_write_bit_pattern(MAX32550_EVKIT_SAM_VCC_PIN_PORT,
                                MAX32550_EVKIT_SAM_VCC5_PIN, 2, 3);
#endif					
        } else {
            /*check if the card is already turned off */
            if (sccr.bits.START  == 0) {
                return ICC_OK;
            }

            /*first reset the smarcard */
            scpin.bits.CRDRST = RESET_ACTIVE;
            OSWrapper_WriteReg(UartState->UartAddress + SC_PN, scpin.word);

            /*
             * wait CWT to help the test tool
             * (if we deactivate too fast, the Lab test tool crashes)
             */
            IccWait(SlotCtx, SlotCtx->IccProtocolConfig.IccCharWaitingTime);

            /*then start the deactivation */
            sccr.bits.START = 0;
            OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

            /*wait until the deactivation completed */
            do {
                OSWrapper_ReadReg(UartState->UartAddress + SC_ISR, &scisr.word);
            } while ((0 == scisr.bits.ACTIVIS) && (Timeout--));

            if (0 == Timeout) {
                return ICC_ERR_UNKOWN;
            }

            /* clear the ACTIVIS flag */
            scisr.bits.ACTIVIS = 0;
            OSWrapper_WriteReg(UartState->UartAddress + SC_ISR, scisr.word);
        }
    break;

    case POWER_UP:
        /*
         * here we do not set the isPowering flag,
         * this will be done when we release the RST signal
         */
        if (SAM_SLOT == SlotCtx->SlotId) {
						// modified to power up sam card in evkit. 
					  // customer needs to modify this part of code based on how samcard afe is interfaced with max3255x
					
#ifdef MAX32550_EVKIT					
				
				// release reset
        scpin.bits.CRDRST = !RESET_ACTIVE;
				OSWrapper_WriteReg(UartState->UartAddress + SC_PN, scpin.word);

				// power up sam card with 3v inptut
				mml_gpio_write_bit_pattern(MAX32550_EVKIT_SAM_VCC_PIN_PORT,
                                MAX32550_EVKIT_SAM_VCC5_PIN, 2, 1);
					
				/** Wait for RDY signal of PHY to be set to '1' */
				{
						unsigned int	tmp = 0;
						while( tmp == 0)
						{
							//if ( mml_gpio_read_bit_pattern(MAX32550_EVKIT_SAM_AFE_RDY_PORT,
								//				MAX32550_EVKIT_SAM_AFE_RDY, 1, (unsigned int*)&tmp)) 
							if ( mml_gpio_read_bit_pattern(0, 26, 1, (unsigned int*)&tmp)) 
							{
									return ICC_ERR_UNKOWN;
							}
						}
				}
#endif					
					
        } else {

            /*check if the card is already powered */
            if (sccr.bits.START  == 1) {
                return ICC_OK;
            }

            /*first reset the smarcard */
            scpin.bits.CRDRST = RESET_ACTIVE;
            OSWrapper_WriteReg(UartState->UartAddress + SC_PN, scpin.word);

            /*clear the PRC (Protection (overcurrent)) bit*/
            scsr.bits.PRC = 0;
            OSWrapper_WriteReg(UartState->UartAddress + SC_SR, scsr.word);

            /*then start the deactivation */
            sccr.bits.START = 1;
            OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);

            /*wait until the activation completed */
            do {
                OSWrapper_ReadReg(UartState->UartAddress + SC_ISR, &scisr.word);
                OSWrapper_ReadReg(UartState->UartAddress + SC_SR, &scsr.word);
                OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);
            } while ((1 != scisr.bits.ACTIVIS) && (Timeout--) &&
                     (0 == scsr.bits.PRC));

            if (1 == scsr.bits.PRC) {
                return ICC_ERR_OVERCURRENT;
            }

            if (0 == Timeout) {
                return ICC_ERR_UNKOWN;
            }

            /* clear the ACTIVIS flag */
            scisr.bits.ACTIVIS = 0;
            OSWrapper_WriteReg(UartState->UartAddress + SC_ISR, scisr.word);
        }
    break;

    case RESET_DO:
        if (SAM_SLOT == SlotCtx->SlotId) {
					scpin.bits.CRDRST = RESET_ACTIVE;
					OSWrapper_WriteReg(UartState->UartAddress + SC_PN, scpin.word);
        } else {
            /*check if the card is already turned off */
            if (sccr.bits.START  == 0) {
                return ICC_OK;
            }

            /*first reset the smarcard */
            scpin.bits.CRDRST = RESET_ACTIVE;
            OSWrapper_WriteReg(UartState->UartAddress + SC_PN, scpin.word);
        }
    break;

    case RESET_RELEASE:
        /* Set the Powering bit
         * this will indicates to the UART driver that the next
         * data are the ATR
         */
        SlotCtx->IsPoweringUp = bTRUE;

        if (SAM_SLOT == SlotCtx->SlotId) {
					scpin.bits.CRDRST = !RESET_ACTIVE;
					OSWrapper_WriteReg(UartState->UartAddress + SC_PN, scpin.word);
        } else {
            /*check if the card is already turned off */
            if (sccr.bits.START  == 0) {
                return ICC_OK;
            }

            /*first release the reset (the smarcard becomes active)*/
            scpin.bits.CRDRST = !RESET_ACTIVE;
            OSWrapper_WriteReg(UartState->UartAddress + SC_PN, scpin.word);
        }
    break;

    default:
        return ICC_ERR_BAD_PARAMETER;
    }

    return ICC_OK;
}


/** @fn                     AfeGetCardStatus
 *  @brief                  Return the Icc state
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *
 *  @return                     return an #IccReturn_t error code
 *  @retval    ICC_OK                    if the card is inserted and powered.
 *  @retval    ICC_ERR_REMOVED           if the card is not present.
 *  @retval    ICC_ERR_PRESENT_INACTIVE  if the card is present but not powered.
 *
 */
static IccReturn_t AfeGetCardStatus(SlotContext_t  *SlotCtx)
{
    SCControl_t          sccr;  /* control register value */
    SCStatus_t           scsr;
    UartState_t          *UartState = NULL;
	  int tmp;

    if ((NULL == SlotCtx) || (NULL == UartState->UartAddress))
        return ICC_ERR_NULL_PTR;

    /* check if the requested slot is available on our target */
    if (MAX3255x_SLOT_NUMBER <= SlotCtx->SlotId)
        return ICC_ERR_BAD_SLOT;

    UartState = (UartState_t *)(SlotCtx->UartData->PrivateData);
    if (NULL == UartState) {
        return ICC_ERR_NULL_PTR;
    }

    if (SAM_SLOT == SlotCtx->SlotId) 
		{
        /* we cannot know if a SAM is present or not,
         * we assume there is one... (and powered !)
         */
					if ( mml_gpio_read_bit_pattern(0, 14, 2, (unsigned int*)&tmp)) 
					{
									return ICC_ERR_UNKOWN;
					}			
					
					if ( tmp == 3)
						return ICC_ERR_PRESENT_INACTIVE;
					else
						return ICC_OK;
    }

    /* starting from here we are dealing with the SMARTCARD_SLOT */
    OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);
    OSWrapper_ReadReg(UartState->UartAddress + SC_SR, &scsr.word);

    /* check if a card is present */
    if(bFALSE == scsr.bits.PRES) {
        return ICC_ERR_REMOVED;
    }

    /*check if the card is powered (and not in protection mode)*/
    if ((bTRUE == sccr.bits.START) && (bFALSE == scsr.bits.PRC)) {
        return ICC_OK;
    }

    return ICC_ERR_PRESENT_INACTIVE;
}


/** @fn     AfeInterrupt_Handler
 *  @brief  Manage PHY Interrupts
 *
 */
static void AfeInterrupt_Handler(SlotContext_t  *SlotCtx)
{
    SCISR_t         Status  =   {.word = 0};
    IccReturn_t     retval  = ICC_OK;
    UartData_t      *UartData  = NULL;
    UartState_t     *UartState = NULL;


    if (NULL == SlotCtx) {
        return;
    }

    UartData = SlotCtx->UartData;
    if (NULL == UartData)  {
        return;
    }

    UartState = (UartState_t *)(UartData->PrivateData);
    if (NULL == UartState) {
        return;
    }

    /* read the Interrupt flags */
    OSWrapper_ReadReg(UartState->UartAddress + SC_ISR, &Status.word);
    /* clear the interupt flags */
    Status.word &= MAX325xx_AFE_INTERRUPT_MASK; /*keep only flags related to the PHY (not the UART)*/
    OSWrapper_WriteReg(UartState->UartAddress + SC_ISR, (~Status.word) & 0xFFF);

    if (bTRUE == Status.bits.PDLIS) {
        /* Presence state changed */
        retval = AfeGetCardStatus(SlotCtx);
        if (ICC_ERR_REMOVED == retval) {
            IccOnCardStateChange(SlotCtx, ICC_REMOVAL);
        } else if (ICC_ERR_PRESENT_INACTIVE == retval) {
            IccOnCardStateChange(SlotCtx, ICC_INSERTION);
        }
    }

    if (bTRUE == Status.bits.PRCIS) {
        /* Overcurrent detected !*/
         IccOnCardStateChange(SlotCtx, ICC_FAULT);
    }

    return;
}


/** @var    MAX3255x_AfeOps
 *  @brief  Analog Front End supported operations
 */
static const SlotOps_t MAX3255x_AfeOps = {
    .select         = AfeSelect,
    .setvoltage     = AfeSetVoltage,
    .power          = AfePower,
    .getcardstatus  = AfeGetCardStatus,
};

/** @fn                     AfeInit
 *  @brief                  Initialize the Analog Front End
 *  @param [in] SlotId      slot numbre, cf #MAX3255xSlots_t
 *
 *  @return                 return an #IccReturn_t error code
 *  @retval ICC_OK          if the AFE is now selected (enabled)
 *
 *  @note   The AFE init must be done *AFTER* the UART init.
 *  @note   As this driver is only for bare-metal, we can directly access to
 *          the GPIOs to configure/drive/
 */
IccReturn_t AfeInit(MAX3255xSlots_t SlotId)
{
    SlotContext_t  *SlotCtx = NULL;
    UartState_t    *UartState = NULL;
    SCIER_t        scier = { .word = 0 };
		SCPin_t				 scpn = { .word = 0 };
		
#if defined (CARD_PRESENCE_ON_LOGIC_LOW) || (defined (MAX32550_B1) && defined (USE_ADDITIONAL_SAM))
    SCControl_t     sccr    =   {.word = 0};
#endif
#if defined(MAX32550_B1) && defined (USE_ADDITIONAL_SAM)
    mml_gpio_config_t   config;
    int32_t              result;
#endif

    if (SlotId >= MAX3255x_SLOT_NUMBER) {
        return ICC_ERR_BAD_SLOT;
    }

    /* MAX32550 B1 has 2 slots:
     * one with the on-chip PHY and
     * a second one 'RAW'
     *
     * register a second slot
     */
    SlotCtx =  IccRegisterAfe(SlotId, SCI_0,
                              (SlotOps_t *) &MAX3255x_AfeOps, NULL);
    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    UartState = (UartState_t *)(SlotCtx->UartData->PrivateData);
    if (NULL == UartState) {
        return ICC_ERR_NULL_PTR;
    }

#ifdef CARD_PRESENCE_ON_LOGIC_LOW
    OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);
    sccr.bits.PRPOL = 1;
    OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);
#endif

    /* register the Handler for our PHY */
    OSWrapper_RegisterIRQ(SlotCtx->UartId, AfeInterrupt_Handler,
                          SlotCtx, bTRUE);

    /* enable the Presence Detect & Overcurrent interrupts */
    OSWrapper_ReadReg(UartState->UartAddress + SC_IER, &scier.word);
    scier.bits.PDLIE = 1;
    scier.bits.PRCIE = 1;
    OSWrapper_WriteReg(UartState->UartAddress + SC_IER, scier.word);

#if defined(MAX32550_B1) && defined (USE_ADDITIONAL_SAM)
    /* we use conditionnal code to avoid to request GPIO when the feature is
     * unused.
     *
     * When we are not on a MAX32550B1 or when USE_ADDITIONAL_SAM is not
     * defined, if we request the SAM_SLOT, we get an error from line 420.
     */
    if (SAM_SLOT == SlotId) {

#ifdef MAX32550_EVKIT	

				
				/** Initialize GPIOs */
				/** Out - SC UART GPIOs ***************************************************/
				/** C4 - C8 */
				config.gpio_direction = MML_GPIO_DIR_OUT;
				config.gpio_function = MML_GPIO_TERTIARY_ALT_FUNCTION;
				config.gpio_intr_mode = MML_GPIO_TERTIARY_ALT_FUNCTION;
			  config.gpio_intr_polarity = MML_GPIO_INT_POL_RAISING;
			  config.gpio_pad_config = MML_GPIO_PAD_NORMAL;
		
				result = mml_gpio_init(MML_GPIO_DEV0,	10, 2, config);
				if ( result )
				{
					return result;
				}
		
				/** RST */
				result = mml_gpio_init(MML_GPIO_DEV0, 22, 1, config);
				if ( result )
			  {
					/** Oops, I did it again ... */
					return result;
			  }
		
				/** CLK */
			  result = mml_gpio_init(MML_GPIO_DEV1, 1, 1, config);
				if ( result )
				{
					/** Oops, I did it again ... */
					return result;
				}
		
				/* Out - Direct GPIOs *****************************************************/
				/** NVCC5 - NVCC3 */
			  config.gpio_direction = MML_GPIO_DIR_OUT;
				config.gpio_function = MML_GPIO_NORMAL_FUNCTION;
				config.gpio_intr_mode = MML_GPIO_NORMAL_FUNCTION;
				config.gpio_intr_polarity = MML_GPIO_INT_POL_RAISING;
				config.gpio_pad_config = MML_GPIO_PAD_NORMAL;
				
				result = mml_gpio_init(MML_GPIO_DEV0, 14, 2, config);
				
				if ( result )	{
					return result;
				}

				/** CS */
			  result = mml_gpio_init(MML_GPIO_DEV0, 20, 1,	config);
				if ( result )	{
					return result;
				}

				/** In - SC UART GPIOs ***************************************************/
				/** IO */
				config.gpio_direction = MML_GPIO_DIR_IN;
				config.gpio_function = MML_GPIO_TERTIARY_ALT_FUNCTION;
				config.gpio_intr_mode = MML_GPIO_TERTIARY_ALT_FUNCTION;
				config.gpio_intr_polarity = MML_GPIO_INT_POL_FALLING;
				config.gpio_pad_config = MML_GPIO_PAD_NORMAL;
#if 1				
				result = mml_gpio_init(MML_GPIO_DEV0, 21, 1, config);
				if ( result ) {
					return result;
				}
#endif				
				/** SC DETECT */
				result = mml_gpio_init(MML_GPIO_DEV1, 0, 1, config);
				if ( result ) 
				{
					return result;
				}
		
				/* In - Direct GPIOs *****************************************************/
				/** NOFF - RDY */
				config.gpio_direction = MML_GPIO_DIR_IN;
				config.gpio_function = MML_GPIO_NORMAL_FUNCTION;
				config.gpio_intr_mode = MML_GPIO_NORMAL_FUNCTION;
				config.gpio_intr_polarity = MML_GPIO_INT_POL_RAISING;
				config.gpio_pad_config = MML_GPIO_PAD_NORMAL;
				
				result = mml_gpio_init(MML_GPIO_DEV0, 25,	2, config);
				if ( result )
				{
					return result;
				}

				/** De activate CS */
				result = mml_gpio_write_bit_pattern(MML_GPIO_DEV0, 20, 1, 0);
				if ( result )
				{
					return result;
				}

				/** Activate CS */
				result = mml_gpio_write_bit_pattern(MML_GPIO_DEV0, 20, 1, 1);
				if ( result )
				{
					return result;
				}

				/** Set NVCC5 & NVCC3 to low */
				result = mml_gpio_write_bit_pattern(MML_GPIO_DEV0, 14, 2,	0x3);
				if ( result )
				{
					return result;
				}
			  
				scpn.word =0x38;
				OSWrapper_WriteReg(UartState->UartAddress + SC_PN, scpn.word);
				
				
#endif
        /* set the DUAL MODE bit and select the on-chip PHY*/
        OSWrapper_ReadReg(UartState->UartAddress + SC_CR, &sccr.word);
        sccr.bits.BYP_PHY   = 1;
 	  	  sccr.bits.DUAL_MODE = 1;

        OSWrapper_WriteReg(UartState->UartAddress + SC_CR, sccr.word);
    }
#endif

    return ICC_OK;
}
