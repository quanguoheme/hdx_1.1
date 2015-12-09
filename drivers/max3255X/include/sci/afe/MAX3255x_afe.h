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

#ifndef _AFE_MAX3255x_H_
#define _AFE_MAX3255x_H_

#include <stdint.h>
#include "sc_errors.h"
#include "slot.h"
#include "sc_states.h"
#include "sc_config.h"

/** @file    MAX3255x.h  MAX32550 & MAX32555 on-SoC Analog Front End
 *  @version 2.0.2
 *  @date    2015/02/13
 *  @author  Jean-Marc Simohand <jeanmarc.simohand@maximintegrated.com>
 *
 * @{
 */

/** @defgroup AFE_MAX3255x Analog Front End drivers for the MAX32550 and MAX32555
 *
 * @ingroup ICC_ABSTRACTION
 *
 * This file defines AFE access functions for calls from the abstraction layer.\n
 * This is not intended to be directly used by the user.
 *
 * @{
 */

/** @typedef MAX3255xSlots_t
 *  @brief   Smartcard slots definition for MAX3255x
 *
 *  by definition, the smartcard slot is the PHY output and the SAM slot
 *  is the RAW output.
 */
typedef enum {
    SMARTCARD_SLOT = 0,
    SAM_SLOT,
} MAX3255xSlots_t;

#if defined(__MAX32590)
# error The MAX32590 has no on-chip PHY (Analog Front End)
#elif defined(__MAX32550) || defined(__MAX32555)
# define MAX3255x_SLOT_NUMBER       1
#elif defined(MAX32550_B1) && (!defined (USE_ADDITIONAL_SAM))
# define MAX3255x_SLOT_NUMBER       1
#elif defined(MAX32550_B1) && defined (USE_ADDITIONAL_SAM)
# define MAX3255x_SLOT_NUMBER       2
#else
# warning Unkown chip. (you must define either MAX32550, MAX32550_B1, MAX32555 or MAX32590)
#endif

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
IccReturn_t AfeInit(MAX3255xSlots_t SlotId);

/** @} */ /*defgroup*/
/** @} */ /*file*/


#endif /* _AFE_MAX3255x_H_*/
