/*
 * smartcard_api.h -- Smartcard API for users
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


#ifndef _SMARTCARD_API_H_
#define _SMARTCARD_API_H_

#include <stdint.h>
#include "sc_errors.h"
#include "sc_states.h"

/** @file    smartcard_api.h Smartcard API for user application
 *  @version 2.0.3
 *  @date    2015/02/12
 *  @author  Jean-Marc Simohand <jeanmarc.simohand@maximintegrated.com>
 *
 * @{
 */

/** @defgroup SMARTCARD_DRIVER Smartcard and Analog Front End Driver for MAX325xx
 *
 *
 * @{
 */

/** @defgroup SMARTCARD_DRIVER_API Smartcard and Analog Front End Driver Application interface
 *
 * This file defines all the functions which are user accessible.
 *
 * @{
 */

/** @typedef        ioctl_t IO control values
 */
typedef enum {
    IOCTL_RESET = 0,            /**< Card Reset request (Card reset when *pparam is not 0) */
    IOCTL_POWER_CARD,           /**< Card Power Up/Down (Card powered when *pparam is not 0)*/
    IOCTL_SET_VOLTAGE,          /**< Set Card Voltage pparam is a pointer on a #IccVoltage_t value*/
    IOCTL_SET_INITPARAMS,       /**< Set Init params (used during the card activation), pparams points on a #ActivationParams_t struct*/
    IOCTL_SET_EMVMODE,          /**< Set the Stack Mode (EMV if *pparam is not 0) */
    IOCTL_SET_FIDI,             /**< Change (force) the FIDI (TA1) value *USE WITH CAUTION* */
    IOCTL_SET_GUARD_TIME,       /**< Change (force) the GT value */
    IOCTL_SET_CWT,              /**< Change (force) the CWT value (must be done after the card activation and the PPS to have an effect)*/
    IOCTL_SET_BWT,              /**< Change (force) the BWT value (must be done after the card activation and the PPS to have an effect)*/
    IOCTL_SET_WTE_HANDLER,      /**< set the user Wait time extension handler (called only if not in EMV mode)*/
    IOCTL_SET_PRESENCE_IRQ,     /**< Register User handler for the card status changes */
    IOCTL_REMOVE_PRESENCE_IRQ,  /**< Unregister User handler for the card status changes */
    IOCTL_GET_FIDI,             /**< Returns the actual FiDi value in *pparam */
    IOCTL_GET_GUARD_TIME,       /**< Returns the actual GT value in *pparam */
    IOCTL_GET_CWT,              /**< Returns the actual CWT value in *pparam */
    IOCTL_GET_WT,               /**< Returns the actual BWT/WWT value in *pparam */
    IOCTL_GET_CARD_STATE,       /**< Returns the card state in *pparam, cf #IccReturn_t */
    IOCTL_GET_CARD_CLASS,       /**< Returns the card class in *pparam, cf #IccReturn_t */
    IOCTL_SET_CLOCK_FREQ,       /**< change the smartcard clock frequency */
    IOCTL_CARD_RESET,           /**< do a card reset (internal use) */
} sc_ioctl_t;


/** @fn         SCAPI_open Open a smartcard Slot
 *
 *  @param [in]  SlotId      Slot (AFE) Id.
 *
 *  @return     returns an #IccReturn_t error code
 *  @note       This function is available only for COBRA targets
 */
IccReturn_t SCAPI_open(uint8_t SlotId);


/** @fn         SCAPI_close close a smartcard Slot
 *
 *  @param [in]  SlotId      Slot (AFE) Id.
 *
 *  @return     returns an #IccReturn_t error code
 *  @note       This function is available only for COBRA targets
 */
IccReturn_t SCAPI_close(uint8_t SlotId);


/** @fn         SCAPI_write send an APDU to the card
 *
 *  @param [in]  SlotId      Slot (AFE) Id.
 *  @param [in]  pBuffer     pointer on the command buffer (APDU to send)
 *  @param [in]  length      length of the APDU to send (in bytes)
 *
 *  @return     returns an #IccReturn_t error code
 *  @note       This function is available only for COBRA targets
 */
IccReturn_t SCAPI_write(uint8_t SlotId, uint8_t *pBuffer, uint16_t length);


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
IccReturn_t SCAPI_read(uint8_t SlotId, uint8_t *pBuffer, uint32_t *plength);

/** @fn         SCAPI_ioctl Set driver parameters and get driver state
 *
 *  @param [in]  SlotId      Slot (AFE) Id.
 *  @param [in]  control     #sc_ioctl_t request
 *  @param [in]  pparam      pointer on the parameter to set/get
 *
 *  @return     returns an #IccReturn_t error code
 *  @note       This function is available only for COBRA targets
 */
IccReturn_t SCAPI_ioctl(uint8_t SlotId, sc_ioctl_t control, void *pparam);

/** @} */ /*defgroup */
/** @} */ /*defgroup */
/** @} */ /*file*/
#endif /*_SMARTCARD_API_H_*/
