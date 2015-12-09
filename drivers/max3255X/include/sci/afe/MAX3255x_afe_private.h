/*
 * MAX3255x.h  MAX32550 & MAX32555 on-SoC Analog Front End - driver private functions
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

#ifndef _AFE_MAX3255x_PRIVATE_H_
#define _AFE_MAX3255x_PRIVATE_H_

#include <stdint.h>
#include "sc_errors.h"
#include "slot.h"
#include "sc_states.h"
#include "sc_config.h"

/** @file    MAX3255x.h  MAX32550 & MAX32555 on-SoC Analog Front End - driver private functions
 *  @version 2.0.0
 *  @date    2015/02/13
 *  @author  Jean-Marc Simohand <jeanmarc.simohand@maximintegrated.com>
 *
 * @{
 */

/** @defgroup AFE_MAX3255x_PRIVATE Analog Front End driver private functions for the MAX32550 and MAX32555
 *
 * @ingroup ICC_ABSTRACTION
 *
 * This file defines AFE access functions for direct calls from the abstraction layer.\n
 * This is not intended to be directly used by the user.
 *
 * @{
 */


/** @def  RESET_ACTIVE    Smartcard RST signal active level
 */
#define RESET_ACTIVE                    0 /**< Smartcard Reset is active LOW*/

/** @def MAX3255x_BYP_CLK_PORT         Clock pin (CLK) port for the bypass bus
 */
# define MAX3255x_BYP_CLK_PORT         MML_GPIO_DEV1   /**< SC_BYP_CLK is on P1.1*/

/** @def MAX3255x_BYP_CLK              Clock pin (CLK) number for the bypass bus
 */
# define MAX3255x_BYP_CLK              1                /**< SC_BYP_CLK is on P1.1*/

/** @def MAX3255x_BYP_IO_PORT          I/O pin (IO) port for the bypass bus
 */
# define MAX3255x_BYP_IO_PORT          MML_GPIO_DEV0   /**< SC_BYP_IO is on P0.21*/

/** @def MAX3255x_BYP_IO               I/O pin (IO) number for the bypass bus
 */
# define MAX3255x_BYP_IO               21               /**< SC_BYP_IO is on P0.21*/

/** @def MAX3255x_BYP_RST_PORT         RESET pin (RST) port for the bypass bus
 */
# define MAX3255x_BYP_RST_PORT         MML_GPIO_DEV0   /**< SC_BYP_RST is on P0.22*/

/** @def MAX3255x_BYP_RST              RESET pin (RST) number for the bypass bus
 */
# define MAX3255x_BYP_RST              22               /**< SC_BYP_RST is on P0.22*/

/** @def MAX325xx_AFE_INTERRUPT_MASK
 *  @brief Smartcard AFE interrupt mask
 */
#define MAX325xx_AFE_INTERRUPT_MASK      (0xE00)

/** @fn                     AfeSelect
 *  @brief                  Select (enable) an AFE (Analog Front End)
 *  @param [in] SlotCtx     Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] Selected     if #bTRUE, we select the AFE (we enable)
 *
 *  @return                 return an #IccReturn_t error code
 *  @retval ICC_ERR_REMOVED when no card is present
 *  @retval ICC_OK          if the AFE is now selected (enabled)
 */
static IccReturn_t AfeSelect(SlotContext_t  *SlotCtx, boolean_t Selected);

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
static IccReturn_t AfeSetVoltage(SlotContext_t  *SlotCtx, IccVoltage_t Voltage);

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
static IccReturn_t AfePower(SlotContext_t  *SlotCtx, CardPowerState_t PowerUp);


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
static IccReturn_t AfeGetCardStatus(SlotContext_t  *SlotCtx);


/** @} */ /*defgroup*/
/** @} */ /*file*/


#endif /* _AFE_MAX3255x_PRIVATE_H_*/
