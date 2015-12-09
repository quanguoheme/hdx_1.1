/*
 * OSWrapper.h -- Smartcard OS function wrapper
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

#ifndef _SC_OSWRAPPER_H_
#define _SC_OSWRAPPER_H_

#include <stdint.h>
#include "sc_errors.h"
#include "sc_config.h"
#include "slot.h"

#include "mml.h"  /*for the MML_INTNUM_SC */

/** @file    OSWrapper.h Smartcard OS function wrapper
 *  @version 2.0.4
 *  @date    2015/02/16
 *  @author  Jean-Marc Simohand <jeanmarc.simohand@maximintegrated.com>
 *
 * @{
 */

/** @defgroup OSWRAPPER Smartcard OS function wrapper
 *
 * @ingroup ICC_ABSTRACTION
 *
 * This file defines OS dependant functions/calls as register, memory map....
 *
 * @{
 */


/** @def        MML_INTNUM_SC0
 *  @brief      MAX325xx interrupt vector for UART 0
 */
#define MAX325xx_INTNUM_SC0     MML_INTNUM_SC


/** @def        MAX_INTERFACE_NUMBER
 *  @brief      Number of available UART interface(s)
 */
#if defined(__MAX32550) || defined(__MAX32555)  || defined(MAX32550_B1)
# define MAX_INTERFACE_NUMBER    1
#elif defined(__MAX32590)
# error The EMV Stack v2.0 does not support COBRA version of MAX32590
#else
# warning Unkown chip. (you must define either MAX32550, MAX32550_B1, MAX32555 or MAX32590)
#endif

/** @fn                     OSWrapperMap
 *  @brief                  Map a memory region and returns the virtual address
 *  @param [in] uintptr_t    Physical address of the UART
 *
 *  Map a memory region and returns the virtual address
 *
 *  @retval    NULL             if it failed
 *  @retval    uintptr_t type   the virtual address.
 */
uintptr_t OSWrapper_Map(uintptr_t physical_address);


/** @fn                          OSWrapper_WriteReg
 *  @brief                       Write a 32bit value in a register and
 *                               wait 2µs
 *  @param [in] virtual_address  Virtual address of the *REGISTER*
 *  @param [in] value            a 32bit value to write in the register
 *
 *  @note The input/output parameters are not checked against NULL pointers !
 */
void OSWrapper_WriteReg(uintptr_t virtual_address, volatile uint32_t value);

/** @fn                          OSWrapper_WriteReg
 *  @brief                       Write a 32bit value in a register
 *  @param [in] virtual_address  Virtual address of the *REGISTER*
 *  @param [in] value            a 32bit value to write in the register
 *
 *  @note The input/output parameters are not checked against NULL pointers !
 */
void OSWrapper_WriteReg_nodelay(uintptr_t virtual_address,
                                volatile uint32_t value);

/** @fn                          OSWrapper_ReadReg
 *  @brief                       Read a 32bit value from a register
 *  @param [in] virtual_address  Virtual address of the *REGISTER*
 *  @param [in] value            a pointer on a 32bit value
 *
 *  @note The input/output parameters are not checked against NULL pointers !
 */
void OSWrapper_ReadReg(uintptr_t virtual_address, volatile uint32_t *value);


/** @fn                          OSWrapper_RegisterIRQ
 *  @brief                       register the IRQ handler
 *  @param [in] UartId           Interface number
 *  @param [in] handler          pointer on the interrupt handler
 *  @param [in] SlotCtx          Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] isPHYIRQ         Set to TRUE if the IRQ is related to the PHY
 *
 *  @return     it returns an integer  (32bit signed)
 *  @retval     OS_ERR_BAD_INTERFACE    Wrong interface number (uartid)
 *  @retval     COMMON_ERR_INVAL        if an internal error occured (wrong Vector number)
 *  @retval     NO_ERROR                register succeed
 */
IccReturn_t OSWrapper_RegisterIRQ(uint8_t UartId,
                              void (*handler)(SlotContext_t  *SlotCtx),
                              SlotContext_t  *SlotCtx, boolean_t isPHYIRQ);

/** @fn                          OSWrapper_UnregisterIRQ
 *  @brief                       unregister the IRQ handler
 *  @param [in] UartId           Interface number
 *  @param [in] SlotCtx          Slot configuration context pointer (cf #SlotContext_t)
 *  @param [in] isPHYIRQ         Set to TRUE if the IRQ is related to the PHY
 *
 *  @return     it returns an integer  (32bit signed)
 *  @retval     OS_ERR_BAD_INTERFACE    Wrong interface number (uartid)
 *  @retval     OS_ERR_BAD_SLOT         the Slotconf does not match with the current handler parameter.
 *  @retval     OS_OK                   Unregister succeed
 *
 */
IccReturn_t OSWrapper_UnregisterIRQ(uint8_t UartId,
                                SlotContext_t  *SlotCtx, boolean_t isPHYIRQ);

/** @fn                          OSWrapper_Interrupt_disable
 *  @brief                       disable all the interrupt sources
 */
void OSWrapper_Interrupt_disable( void );


/** @fn                          OSWrapper_Interrupt_enable
 *  @brief                       enable all the interrupt sources
 */
void OSWrapper_Interrupt_enable( void );

/** @fn                          OSWrapper_getUARTbyId
 *  @brief                       return an UART supported operations structure (cf #UartOps_t)
 *  @param [in] UartId           Interface number
 *
 *  @return   a pointer on the #UartData_t for the corresponding UartId
 *  @reval    #NULL     if the UartId is unknown
 */
UartData_t *OSWrapper_getUARTbyId(UartId_t uartid);


/** @fn                          OSWrapper_getSlotbyId
 *  @brief                       return a Slot supported operations structure (cf #SlotOps_t)
 *  @param [in] slotid           Slot Number
 *
 *  @return   a pointer on the #SlotData_t for the corresponding slotid
 *  @reval    #NULL     if the slotid is unknown
 */
SlotData_t *OSWrapper_getSlotbyId(uint8_t slotid);


/** @fn                          OSWrapper_getSlotCtxbyId
 *  @brief                       return a Slot context structure (cf #SlotContext_t)
 *  @param [in] slotid           Slot Number
 *
 *  @return   a pointer on the #SlotContext_t for the corresponding slotid
 *  @reval    #NULL     if the slotid is unknown
 */
SlotContext_t  *OSWrapper_getSlotCtxbyId(uint8_t SlotId);





/** @fn                          OSWrapper_RegisterUART
 *  @brief                       register an UART supported operations pointer (cf #UartOps_t)
 *  @param [in] UartId           Interface number
 *  @param [in] ops              pointer on the operations to register
 *  @param [in] UartPrivateData  interface private (internal) data pointer
 *
 *  @return   an #IccReturn_t  error code
 *  @reval    ICC_OK            on sucess
 */
IccReturn_t OSWrapper_RegisterUART(UartId_t uartid, UartOps_t *ops,
                                   void *UartPrivateData);


/** @fn                          OSWrapper_RegisterSlot
 *  @brief                       register an AFE supported operations pointer (cf #SlotOps_t)
 *  @param [in] slotid           Slot Number
 *  @param [in] ops              pointer on the operations to register
 *  @param [in] SlotPrivateData  Slot private (internal) data pointer
 *
 *  @return   an #IccReturn_t  error code
 *  @reval    ICC_OK            on sucess */
IccReturn_t OSWrapper_RegisterSlot(uint8_t slotid, SlotOps_t *ops,
                                    void *SlotPrivateData);


/** @fn                          OSWrapper_memdestroy
 *  @brief                       destroy a buffer content
 *  @param [in] Buffer           pointer on the buffer to erase
 *  @param [in] length           number of *bytes* to erase
 *
 */
void OSWrapper_memdestroy(uint8_t *Buffer, int32_t length);


/** @fn                          OSWrapper_EnterCritical
 *  @brief                       Disable the scheduler and the interrupt sources
 */
void OSWrapper_EnterCritical( void );


/** @fn                          OSWrapper_ExitCritical
 *  @brief                       Enable the scheduler and the interrupt sources back.
 */
void OSWrapper_ExitCritical( void );

/** @} */ /*defgroup*/
/** @} */ /*file*/


#endif /* _SC_OSWRAPPER_H_ */
