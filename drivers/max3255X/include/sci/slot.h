/*
 * slot.h -- Smartcard slot definition
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

#ifndef _SLOT_H_
#define _SLOT_H_
#include <stdint.h>
#include "sc_types.h"
#include "sc_states.h"
#include "sc_regs.h"
#include "ProtocolT0.h"
#include "OS_types.h"
#include "smartcard_api.h"

/* Ask GCC to NOT pack the structures */
#pragma pack(push)
#pragma pack()

/** @file    slot.h Smartcard Slot definition (INTERNAL data)
 *  @version 2.0.3
 *  @date    2015/01/15
 *  @author  Jean-Marc Simohand <jeanmarc.simohand@maximintegrated.com>
 *
 * @{
 */



/** @defgroup SLOT_LEVEL    Smartcard Slot level definitions
 *
 *  @ingroup SMARTCARD_DRIVER
 *
 *  This file defines the Slot structure and functions.\n
 *  This is not intended to be directly used by the user.
 *
 * @note
 * @{
 */

/** @var    IccEMVActivationParams
 *  @brief  Default EMV activation parameters
 */
extern const ActivationParams_t   IccEMVActivationParams;

/** @typedef    ProtocolConfiguration_t contains all the Smartcard protocol parameters
 *
 *   This struct declares all #slot protocol parameters.
*/
typedef struct {

    uint32_t    IccWaitingTime;         /**< Card BWT (Block Waiting Time)
                                             or WWT (Work Waiting Time)
                                             * value (in etus) */
    uint32_t    IccWorkWaitingTime;     /**< Card WWT (Work Waiting Time)
                                             value (in etus) */
    uint16_t    IccExtraGuardTime;      /**< Card (E)GT ((Extra) Guard Time)
                                             value (in etus) */
    uint16_t    IccCharWaitingTime;     /**< Card CWT (Char Waiting Time )
                                             value (in etus) */
    uint8_t     IccBlockGuardTime;      /**< Card BGT (Block Waiting Time)
                                             value (in etus) */

    uint8_t     FiDi;                   /**< Current Fi/Di parameters*/
    uint8_t     PossibleFiDi;           /**< ATR defined FiDi value */
    boolean_t   InverseConvention;      /**< If true, use the inversion convention */
    boolean_t   AutoPPS;                /**< if TRUE, the stack will process the PPS
                                         *   just after the ATR has been received*/

} ProtocolTimings_t;



/** @typedef    Protocol_t contains all the Protocol parameters
 *
 *   This struct contains all the Protocol related parameters.
*/
typedef struct {
    /*
     * Following variables are NOT user configurable.
     * These values are set by the driver.
     */
    uint8_t     IccIFSC;            /**< Card buffer size */
    uint8_t     IccIFSD;            /**< Reader buffer size */

    uint8_t     IccProtocol:1;      /**< Current Card protocol
                                     *   (0 -> 'T=0', 1 -> 'T=1')*/
    uint8_t     IccEDCTypeCRC:1;    /**< bTRUE if CRC is used for error
                                     *   detection, otherwise it is LRC*/
    uint8_t     :6;

} Protocol_t;



/** @typedef    Case_t
 *  @brief      ADPU exchange cases
 *
 * The APDU exchanges can be splitted in 4 cases:
 *
 * @htmlonly
<table border="3">
    <tr> <!-- Array Title -->
        <th colspan="3">Command APDU</th>
    </tr>
    <tr> <!-- Array 2nd line (column titles) -->
        <th>Field name</th>
        <th>Length (bytes)</th>
        <th>Description</th>
    </tr>
    <tr><!--3rd CLA byte description -->
        <td style="text-align:center;">CLA</td>
        <td style="text-align:center;">1</td>
        <td>Instruction class - indicates the type of command</td>
    </tr>
    <tr>
        <td style="text-align:center;">INS</td>
        <td style="text-align:center;">1</td>
        <td>Instruction code - indicates the specific command</td>
    </tr>
        <tr>
        <td style="text-align:center;">P1-P2</td>
        <td style="text-align:center;">2</td>
        <td>Instruction parameters for the command</td>
    </tr>
    <tr>
        <td style="text-align:center;">L<sub>c</sub></td>
        <td style="text-align:center;">Absent or 1 </td>
        <td>Encodes the number (N<sub>c</sub>) of bytes of command data to follow<br />
            (3 bytes for Extended APDU, not supported)</td>
    </tr>
    <tr>
        <td style="text-align:center;">Data</td>
        <td style="text-align:center;">N<sub>c</sub></td>
        <td>N<sub>c</sub> bytes of data</td>
    </tr>
    <tr>
        <td style="text-align:center;">L<sub>e</sub></td>
        <td style="text-align:center;">Absent or 1</td>
        <td>Encodes the maximum number (N<sub>e</sub>) of response bytes expected<br />
            (3 bytes for Extended APDU, not supported)</td>
    </tr>
    <tr>
        <th colspan="3">Response APDU</th>
    </tr>
    <tr>
        <td style="text-align:center;">Response</td>
        <td style="text-align:center;">N<sub>r</sub> (at most N<sub>e</sub>)</td>
        <td>Response data</td>
    </tr>
    <tr>
        <td style="text-align:center;">SW1-SW2<br />
        (Status Bytes)</td>
        <td style="text-align:center;">2</td>
        <td>Command processing status</td>
    </tr>
</table>
 * @endhtmlonly
 */
typedef enum {
    CASE_1 = 1,     /**< Case 1 : CLA INS P1 P2 */
    CASE_2,         /**< Case 2 : CLA INS P1 P2 Lc (0 <= Lc <= 256) */
    CASE_3,         /**< Case 3 : CLA INS P1 P2 Lc [data] (1 <= Lc <= 255) */
    CASE_4,         /**< Case 4 : CLA INS P1 P2 Lc [data] Le (1 <= Lc <= 255
                                                          and 0 <= Le <= 256)*/
    CASE_UNKNOWN = 0xFF,
} Case_t;
/** @typedef    T0Exchange_t T=0 protocol Apdu exchange state data structure.
 */
typedef struct {
    uint8_t      IccCLA;             /**< Current APDU Class */
    uint8_t      IccINS;             /**< Current APDU Instruction */
    Case_t       IccT0Case;          /**< Exchange Case, refer to #Case_t */
    boolean_t    FirstAPDU;          /**< Set if first APDU exchange */
} T0Exchange_t;

/** @typedef    T1Exchange_t T=1 protocol Apdu exchange state data structure.
 */
typedef struct {
    uint8_t     MaxRetries;             /**< set to 0x22 for 3 synchros & 3 retries*/
    uint8_t     NAD;                 /**< Current Node address*/
    uint8_t     PCB;                 /**< Current PCB */
    uint16_t    LEN;                 /**< Current LEN to Send */
    uint8_t     INF;                 /**< Information byte for S(WTX) and S(IFS) */
    uint8_t     ExpectedPCB;         /**< expected PCB */
    uint8_t     CurrentPCB;          /**< PCB value of the block to send again */
    uint8_t     LastRBlockErr;       /**< R-blocks management (correct error code) */
    uint8_t     RBlockAlreadySent:1; /**< R-blocks management (correct error code) */
    uint8_t     IccChaining:1;       /**< Does Icc initiate chaining ?*/
    uint8_t     WTX:1;               /**< Work Time Extension requested */
    uint8_t     FirstBlock:1;        /**< True is current block is the first block */
    uint8_t     ResyncSendBlockCurrent:1;   /**< Shall Resynch send the block again ? */
    uint8_t     ProtocolInitStage:1; /**< True if it's the first ICC exchange */
    uint8_t     :2;
    boolean_t   Ns;                  /**< Send sequence number */
    boolean_t   Nr;                  /**< Receive sequence number */

} T1Exchange_t;




/** @typedef    ATRParams_t contains the protocol initial values, read from the ATR.
 */
typedef struct {
    uint8_t IFSC;  /**< Card receive buffer depth */
    uint8_t CWI;   /**< Character Wait Initial value */
    uint8_t BWI;   /**< Block Wait Initial value */
    uint8_t WWI;   /**< Work Wait Initial value */
    uint8_t EGT;   /**< Extra Wait Initial value */
    uint8_t Di;    /**< Di value from the TA1 */
} ATRParams_t;


/** @typedef    TxData_t contains the data for the current smartcard transmission
 */
typedef struct {
    uint8_t         *pdata; /**< Pointer on the remaining data to send*/
    uint32_t        datalen;/**< Remaining data length*/
    uint16_t        EDC;    /**< T=1 EDC value */
    boolean_t       TransmissionComplete;/**< Transmission complete flag */
    boolean_t       ParityError;         /**< Parity error on Rx or Tx */
    boolean_t       EDCSent;             /**< Set to true when the ISR sent the EDC */
} TxData_t;

/** @typedef    UartState_t contains the UART driver data for a slot
 */
typedef struct {
    uintptr_t       UartAddress;        /**< uart virtual address (set during the registration */
    IccRequest_t    *RxRequest;         /**< Uart structure for data Rx */
    TxData_t        TxData;             /**< Transmission data struct */
    uint8_t         ActiveSlot;         /**< Current active slot */
} UartState_t;


/* forward declarations */
struct UartData_t;
struct SlotData_t;

/** @typedef    SlotContext_t contains all the Slot parameters, including the protocol
 *
 *   This struct declares a Slot object.\n
 *   A slot is a collection of an AFE, an UART and a Protocol.
*/
typedef struct SlotContext_t {
    uint8_t             UartId;            /**< Smartcard UART number */
    uint8_t             SlotId;            /**< Slot number returned by the register
                                            *   function */
    uint8_t             isEMV:1;           /**< Set to bTRUE if this slot is EMV L1*/
    uint8_t             IsPoweringUp:1;    /**< set to bTRUE by the driver during the Slot power up*/
    uint8_t             :6;
    uint8_t             CardClass;
    struct UartData_t  *UartData;          /**< UART driver private data */
    struct SlotData_t  *AfeData;           /**< Analog Front End driver private data */
    Protocol_t          IccProtocolParams; /**< Protocol related data */
    ProtocolTimings_t   IccProtocolConfig; /**< Timing related data */
    ATRParams_t         InitialParams;     /**< Initial values read from the ATR*/
    ActivationParams_t  ActivationParams;  /**< Card activation parameters */
    T1Exchange_t        T1;                /**< T=1 APDU exchange state */
    T0Exchange_t        T0;                /**< T=0 APDU exchange state */
    void                (*usr_cardstatusisr)(CardState_t CardState); /**< User CardStatus handler*/
    uint8_t             *RxBuffer;         /**< Buffer for card answer */
    uint8_t             *ScratchPadBuff;   /**< temporary Buffer for card exchanges */
    uint32_t            RxLength;          /**< Answer length */} SlotContext_t;


/** @typedef UartOps_t Defines Smartcard UART operations
  */
typedef struct {
    void        (*oncardevent)(SlotContext_t  *SlotCtx, CardState_t CardState);
    IccReturn_t (*receive)(SlotContext_t  *SlotCtx, IccRequest_t *RxRequest);
    IccReturn_t (*send)(SlotContext_t  *SlotCtx, uint8_t *pData, uint16_t datalen);
    IccReturn_t (*activate)(SlotContext_t  *SlotCtx, ActivationParams_t *ActivationParams, IccRequest_t *RxRequest);
    void        (*stop)(SlotContext_t  *SlotCtx, IccReturn_t status);
    uint16_t    (*getrxlen)(SlotContext_t  *SlotCtx, IccReturn_t *Status);
    void        (*wait)(SlotContext_t  *SlotCtx, uint32_t etus);
    IccReturn_t (*ioctl)(SlotContext_t  *SlotCtx, sc_ioctl_t command, uint32_t value);
    void        (*irqhandler)(SlotContext_t  *SlotCtx);
} UartOps_t;


 /** @typedef UartData_t Defines interface (UART) driver internal data
  */
typedef struct UartData_t {
    UartOps_t       *Operations;        /**< Supported operations */
    void            *PrivateData;       /**< pointer on the private data */
} UartData_t;


 /** @typedef SlotOps_t Defines a slot (Analog Front End) operations
  */
typedef struct SlotOps_t {
    IccReturn_t (*select)(SlotContext_t  *SlotCtx, boolean_t Selected);       /**< AFE select pointer */
    IccReturn_t (*setvoltage)(SlotContext_t  *SlotCtx, IccVoltage_t Voltage); /**< AFE set voltage pointer */
    IccReturn_t (*power)(SlotContext_t  *SlotCtx, CardPowerState_t PowerUp);  /**< AFE power pointer */
    IccReturn_t (*getcardstatus)(SlotContext_t  *SlotCtx);                    /**< AFE get card status pointer */
} SlotOps_t;

 /** @typedef SlotData_t Defines  AFE (slot) driver internal data
  */
typedef struct SlotData_t {
    SlotOps_t       *Operations;        /**< Supported operations */
    void            *PrivateData;       /**< pointer on the private data */
} SlotData_t;

/** @var        SlotDefaultConfig Default slot configuration
 *
 *  This is used to initialize the slot.
 */
extern const SlotContext_t SlotDefaultConfig;

/** @} */ /*@defgroup SLOT_LVL    Smartcard Slot level definitions*/
/** @} */ /*@file    slot.h    Smartcard Slot definition (INTERNAL data) */

#pragma pack(pop)
#endif /*_SLOT_H_*/
