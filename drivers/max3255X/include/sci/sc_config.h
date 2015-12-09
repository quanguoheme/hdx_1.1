/*
 * sc_config.h -- Smartcard driver configuration file
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

/** @file    sc_config.h Smartcard driver configuration file
 *  @version 2.0.3
 *  @date    2015/02/16
 *  @author  Jean-Marc Simohand <jeanmarc.simohand@maximintegrated.com>
 *
 * @{
 */

/** @defgroup DRIVER_CONFIGURATION    Smartcard driver configuration file
 *
 * @ingroup SMARTCARD_DRIVER
 *
 *
 * @{
 */

/** @def    CARD_PRESENCE_ON_LOGIC_LOW
 *  @brief  When set, a Low logic level on PRES is interpreted as a card present
 */
//#define CARD_PRESENCE_ON_LOGIC_LOW


/** @def    BAREMETAL_EMV_LOOPBACK
 *  @brief  When set, We run the EMV loopback application (from the main.c)
 */
#ifdef BAREMETAL_EMV_LOOPBACK
# undef USE_FREERTOS
#endif

/** @def    MAX32550_B1
 *  @brief  MAX32550-B1 MCU type
 */
#define MAX32550_B1           /**< uncomment this definition to use with the MAX32550 B1 */
#ifdef MAX32550_B1
# undef __MAX32550              /* undefine the Compiler defined target (this apply only to the smartcard stack)*/
#endif


/** @def        USE_ADDITIONAL_SAM
 *  @brief      Enable the MAX32550-B1 Dual Mode
 *
 * On MAX32550-B1, there is an on-chip PHY and we can configure an additional
 * ouput for a SAM.\n
 *
 * By defining USE_ADDITIONAL_SAM, the additional outpout becomes available.
 */
#define USE_ADDITIONAL_SAM    /**< uncomment this definition to use the MAX32550 B1 with an additional SAM*/


/** @def    CETECOM_FIME_EMVCO_1CF_126_0y_COMPLIANCE
 *  @brief  avoid to send a IFS(Request) after a resynch
 *
 * for CETECOM and FIME Labs,
 * if we received 3 wrong answers to a IFS(request),
 * we do a resynch, then we restart with the select
 * (ie not with a new IFS(Request)\n
 *
 * Note: this is not specified by the EMV, for the
 * EMVCo ref. 1CF.126.0y test, the pass criteria is to
 * receive a S(Resynch) request.
 */
 #define CETECOM_FIME_EMVCO_1CF_126_0y_COMPLIANCE


 /** @} */ /*defgroup*/
/** @} */ /*file*/
