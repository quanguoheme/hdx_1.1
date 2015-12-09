/*
 * OSWrapper.c -- Smartcard OS function wrapper
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
#include "sc_config.h"
#include "slot.h"
#include "iccabstract.h"
#include "ProtocolT1.h"
#include "OSWrapper.h"
#include "mml_intc.h"
#include "MAX3255x_afe.h"

#ifdef USE_FREERTOS
# include "FreeRTOS.h"
# include "task.h"
# include "queue.h"
# include "portmacro.h"
#endif

/**** NOTE: this OSWrapper is specific to the MAX32550 and MAX32555 (COBRA) ****/

/** forward declaration */
void OSWrapper_SC0Handler(void);

/*
 * On MAX32550 and MAX32555, we do not have a dynamic memory allocator,
 * so we use global static variable to own the driver data
 */
static SlotContext_t    SlotContexes[MAX3255x_SLOT_NUMBER];
static UartData_t       UartData[MAX_INTERFACE_NUMBER];
static SlotData_t       SlotData[MAX3255x_SLOT_NUMBER];


/** @typedef               OSWrapperData_t
 *  @brief                 OS Wrapper private data type definition
 */
typedef struct {
    void (*IRQHandler)(void);                           /**< OS Wrapper IRQ handler*/
    void (*UARTHandler)(SlotContext_t  *SlotCtx);       /**< EMV Stack IRQ handler*/
    void (*PHYHandler)(SlotContext_t  *SlotCtx);        /**< PHY (Analog Front End) IRQ handler*/
    SlotContext_t         *SlotCtx;                     /**< Current Slot configuration on this interface*/
    uint32_t              InterruptNumber;              /**< Interrupt Vector number for this interface */
} OSWrapperData_t;


/** @variable               OSWrapperData
 *  @brief                  OS Wrapper private data
 */
OSWrapperData_t OSWrapperData[MAX_INTERFACE_NUMBER] = {
     /* interface 0 */
    {
        .IRQHandler = OSWrapper_SC0Handler,
        .UARTHandler = NULL,
        .PHYHandler = NULL,
        .SlotCtx   = NULL,
        .InterruptNumber = MAX325xx_INTNUM_SC0,
    },
};

/* Basic wait function */
void udelay(int us)
{
    /* @108MHz, each CPU clock cycle is approx 10ns
     * in fact due to the branch, we will flush the
     * pipeline an thus consume more time
     *
     * an empiric estimation gives each loop takes
     * about 10 instructions (branch taken, no optimization)
     *
     * ie, each loop consumes +/- 100ns
     */
    us = us*10;

    for (; us !=0 ; us--)
            __asm("nop;");

}

/** @fn                     OSWrapper_SC0Handler
 *  @brief                  Handler for the UART interrupt
 *
 *  It calls the EMV IRQ handler with the current Slot configuration as parameter.
 *
 */
void OSWrapper_SC0Handler(void)
{
    if (NULL != OSWrapperData[0].SlotCtx) {
        if (NULL != OSWrapperData[0].UARTHandler) {
            OSWrapperData[0].UARTHandler(OSWrapperData[0].SlotCtx);
        }

        if (NULL != OSWrapperData[0].PHYHandler) {
            OSWrapperData[0].PHYHandler(OSWrapperData[0].SlotCtx);
        }
    }

    /* clear Core pending interrupt */
    mml_intc_ack_irq(OSWrapperData[0].InterruptNumber);
}

/** @fn                     OSWrapperMap
 *  @brief                  Map a memory region and returns the virtual address
 *  @param [in] uintptr_t   Physical address of the UART
 *
 *  Map a memory region and returns the virtual address
 *
 *  @retval    NULL             if it failed
 *  @retval    uintptr_t type   the virtual address.
 */
uintptr_t OSWrapper_Map(uintptr_t physical_address)
{
    /* on COBRA, we do not use the MMU */
    return physical_address;
}

/** @fn                          OSWrapper_WriteReg
 *  @brief                       Write a 32bit value in a register
 *  @param [in] virtual_address  Virtual address of the *REGISTER*
 *  @param [in] value            a 32bit value to write in the register
 *
 *  @note The input/output parameters are not checked against NULL pointers !
 */
void OSWrapper_WriteReg(uintptr_t virtual_address, volatile uint32_t value)
{
    *((volatile uint32_t *)virtual_address) = value;
    udelay (2);
}

/** @fn                          OSWrapper_WriteReg
 *  @brief                       Write a 32bit value in a register
 *  @param [in] virtual_address  Virtual address of the *REGISTER*
 *  @param [in] value            a 32bit value to write in the register
 *
 *  @note The input/output parameters are not checked against NULL pointers !
 */
void  OSWrapper_WriteReg_nodelay(
                                uintptr_t virtual_address,
                                volatile uint32_t value)
{
    *((volatile uint32_t *)virtual_address) = value;
}

/** @fn                          OSWrapper_ReadReg
 *  @brief                       Read a 32bit value from a register
 *  @param [in] virtual_address  Virtual address of the *REGISTER*
 *  @param [in] value            a pointer on a 32bit value
 *
 *  @note The input/output parameters are not checked against NULL pointers !
 */
void OSWrapper_ReadReg(uintptr_t virtual_address, volatile uint32_t *value)
{
    *value = *((volatile uint32_t *)virtual_address);
}

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
                              SlotContext_t  *SlotCtx, boolean_t isPHYIRQ)
{
    IccReturn_t ret = ICC_OK;

    if (UartId > MAX_INTERFACE_NUMBER) {
        return ICC_ERR_BAD_PARAMETER;
    }

    if ((NULL == handler) || (NULL == SlotCtx)) {
        return ICC_ERR_NULL_PTR;
    }

    if (isPHYIRQ == bTRUE) {
        OSWrapperData[UartId].PHYHandler = handler;
    } else {
        OSWrapperData[UartId].UARTHandler = handler;
    }
    OSWrapperData[UartId].SlotCtx = SlotCtx;

    ret = mml_intc_setup_irq((unsigned int)OSWrapperData[UartId].InterruptNumber,
                             (mml_intc_prio_t)MML_INTC_PRIO_8,
                             OSWrapperData[UartId].IRQHandler);

    if (ret)
        return ICC_ERR_UNKOWN;

    return ICC_OK;
}


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
                                SlotContext_t  *SlotCtx, boolean_t isPHYIRQ)
{
    IccReturn_t ret = ICC_OK;

    if (UartId > MAX_INTERFACE_NUMBER)
        return ICC_ERR_BAD_PARAMETER;

    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    if (OSWrapperData[UartId].SlotCtx != SlotCtx) {
        ret = ICC_ERR_BAD_PARAMETER;
        goto unregister_exit;
    }

    if (isPHYIRQ == bTRUE) {
        OSWrapperData[UartId].PHYHandler = NULL;
    } else {
        OSWrapperData[UartId].UARTHandler = NULL;
    }

    if ((OSWrapperData[UartId].UARTHandler == NULL) &&
        (OSWrapperData[UartId].PHYHandler == NULL)) {
        mml_intc_detach_irq(OSWrapperData[UartId].InterruptNumber);
    }

unregister_exit:
    return ret;
}

/** @fn                          OSWrapper_getUARTbyId
 *  @brief                       return an UART supported operations structure (cf #UartOps_t)
 *  @param [in] UartId           Interface number
 *
 *  @return   a pointer on the #UartData_t for the corresponding UartId
 *  @reval    #NULL     if the UartId is unknown
 */
UartData_t *OSWrapper_getUARTbyId(UartId_t uartid)
{
    if (uartid >= MAX_INTERFACE_NUMBER) {
        return NULL;
    }

    return &UartData[uartid];
}


/** @fn                          OSWrapper_getSlotbyId
 *  @brief                       return a Slot supported operations structure (cf #SlotOps_t)
 *  @param [in] slotid            Slot Number
 *
 *  @return   a pointer on the #SlotData_t for the corresponding slotid
 *  @reval    #NULL     if the slotid is unknown
 */
SlotData_t *OSWrapper_getSlotbyId(uint8_t slotid)
{
    if (slotid >= MAX3255x_SLOT_NUMBER) {
        return NULL;
    }

    return &SlotData[slotid];
}

/** @fn                          OSWrapper_getSlotCtxbyId
 *  @brief                       return a Slot context structure (cf #SlotContext_t)
 *  @param [in] slotid            Slot Number
 *
 *  @return   a pointer on the #SlotContext_t for the corresponding slotid
 *  @reval    #NULL     if the slotid is unknown
 */
SlotContext_t  *OSWrapper_getSlotCtxbyId(uint8_t SlotId)
{
    if (SlotId >= MAX3255x_SLOT_NUMBER) {
        return NULL;
    }

    return &SlotContexes[SlotId];
}




/** @fn                          OSWrapper_RegisterUART
 *  @brief                       register an UART supported operations pointer (cf #UartOps_t)
 *  @param [in] UartId            Interface number
 *  @param [in] ops               pointer on the operations to register
 *  @param [in] UartPrivateData   interface private (internal) data pointer
 *
 *  @return   an #IccReturn_t  error code
 *  @reval    ICC_OK            on sucess
 */
IccReturn_t OSWrapper_RegisterUART(UartId_t uartid, UartOps_t *ops,
                                   void *UartPrivateData)
{
    if (uartid >= MAX_INTERFACE_NUMBER) {
        return ICC_ERR_BAD_PARAMETER;
    }

    UartData[uartid].Operations  = ops;
    UartData[uartid].PrivateData = UartPrivateData;

    return ICC_OK;
}


/** @fn                          OSWrapper_getSlotbyId
 *  @brief                       register an AFE supported operations pointer (cf #SlotOps_t)
 *  @param [in] slotid            Slot Number
 *  @param [in] ops               pointer on the operations to register
 *  @param [in] SlotPrivateData   Slot private (internal) data pointer
 *
 *  @return   an #IccReturn_t  error code
 *  @reval    ICC_OK            on sucess */
IccReturn_t OSWrapper_RegisterSlot(uint8_t slotid, SlotOps_t *ops,
                                    void *SlotPrivateData)
{
    if (slotid >= MAX3255x_SLOT_NUMBER) {
        return ICC_ERR_BAD_PARAMETER;
    }

    SlotData[slotid].Operations = ops;
    SlotData[slotid].PrivateData = SlotPrivateData;

    return ICC_OK;
}


/** @fn                          OSWrapper_memdestroy
 *  @brief                       destroy a buffer content
 *  @param [in] Buffer            pointer on the buffer to erase
 *  @param [in] length            number of *bytes* to erase
 *
 */
void OSWrapper_memdestroy(uint8_t *Buffer, int32_t length)
{
    #warning memdestroy uses only memset !!! (UNSECURE)
    memset(Buffer, 0, length);
}



/** @fn                          OSWrapper_EnterCritical
 *  @brief                       Disable the scheduler and the interrupt sources
 */
void OSWrapper_EnterCritical( void )
{
    /*
     * disable interrupts sources,
     * this will also disable the scheduler (if one)
     */
#ifdef USE_FREERTOS
    vTaskSuspendAll();
#endif
}


/** @fn                          OSWrapper_ExitCritical
 *  @brief                       Enable the scheduler and the interrupt sources back.
 */
void OSWrapper_ExitCritical( void )
{
    /* enable the interrupts back */
#ifdef USE_FREERTOS
    xTaskResumeAll();
#endif
}

/** @fn                          OSWrapper_Interrupt_disable
 *  @brief                       disable all the interrupt sources
 */
void OSWrapper_Interrupt_disable( void )
{
     cpsid();
}


/** @fn                          OSWrapper_Interrupt_disable
 *  @brief                       enable all the interrupt sources
 */
void OSWrapper_Interrupt_enable( void )
{
     cpsie();
}

