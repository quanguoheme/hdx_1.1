/*
 * MAX325xx.h -- MAX32590, MAX32550 & MAX32555 Smartcard UART driver
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

#ifndef _UART_MAX325xx_H_
#define _UART_MAX325xx_H_

#include <stdint.h>
#include "sc_errors.h"
#include "slot.h"
#include "sc_config.h"

/** @file    MAX325xx.h MAX32590, MAX32550 & MAX32555 Smartcard UART driver
 *  @version 2.0.2
 *  @date    2015/02/13
 *  @author  Jean-Marc Simohand <jeanmarc.simohand@maximintegrated.com>
 *
 * @{
 */

/** @defgroup UART_MAX325xx MAX32590, MAX32550 & MAX32555 Smartcard UART driver
 *
 * @ingroup ICC_ABSTRACTION
 *
 * This file defines UART access functions for direct calls from the abstraction layer.\n
 * This is not intended to be directly used by the user.
 *
 * @{
 */

#if defined(__MAX32590)
# define MAX325xx_INTERFACE_NUMBER           2
#elif defined(__MAX32550) || defined(__MAX32555) 
# define MAX325xx_INTERFACE_NUMBER           1
#elif defined(MAX32550_B1)
# define MAX325xx_INTERFACE_NUMBER           1
#else
# warning Unkown chip. (you must define either MAX32550, MAX32550_B1, MAX32555 or MAX32590)
#endif


/** @fn                     UartInit
 *  @brief                  Initialize the UART driver
 *  @param [in] UartId_t     Interface number (cf #UartId_t)
 *  @param [in]  SlotCtx            Slot configuration context pointer (cf #SlotContext_t)
 *
 *  Initialize the UART driver.\n
 *
 *  @retval    ICC_OK                   if the driver has been registered
 *  @retval    ICC_ERR_BAD_PARAMETER    if the Interface id (uart id) is out of range
 *  @retval    ICC_ERR_NULL_PTR         if we did not get the virtual address
 *
 * @note On non-Linux plateforms, this must be called during the Stack initialization.
 * @note the GPIOs must be configured as Alternate function by the caller (OS-Dependant task)
 * @note the GCR.SCCK must be configured by the caller.
 */
IccReturn_t  UartInit(UartId_t id, SlotContext_t  *SlotCtx);
IccReturn_t UartExit(UartId_t id, SlotContext_t  *SlotCtx);

/** @} */ /*defgroup*/
/** @} */ /*file*/


#endif /* _UART_MAX325xx_H_*/
