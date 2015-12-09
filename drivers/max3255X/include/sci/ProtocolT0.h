/*
 * ProtocolT0.h -- Send/Receive APDU using T=0 protocol
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

#ifndef _PROTOCOL_T0_H_
#define _PROTOCOL_T0_H_
#include <stdint.h>
#include "sc_types.h"
/** @file    ProtocolT0.h Send/Receive APDU using T=0 protocol
 *  @version 2.0.3
 *  @date    2015/01/15
 *  @author  Jean-Marc Simohand <jeanmarc.simohand@maximintegrated.com>
 *
 * @{
 */



/** @defgroup PROTOCOL_T0 T=0 protocol internal API
 *
 * @ingroup SMARTCARD_DRIVER
 *
 * This file defines the T=0 protocol structures and functions.\n
 * This is not intended to be directly used by the user.
 *
 * T=0 APDU formatting:
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
        <td>CLA</td>
        <td>1</td>
        <td>Instruction class - indicates the type of command</td>
    </tr>
    <tr>
        <td>INS</td>
        <td>1</td>
        <td>Instruction code - indicates the specific command</td>
    </tr>
        <tr>
        <td>P1-P2</td>
        <td>2</td>
        <td>Instruction parameters for the command</td>
    </tr>
    <tr>
        <td>L<sub>c</sub></td>
        <td>Absent or 1 </td>
        <td>Encodes the number (N<sub>c</sub>) of bytes of command data to follow<br />
            (3 bytes for Extended APDU, not supported)</td>
    </tr>
    <tr>
        <td>Command data</td>
        <td>N<sub>c</sub></td>
        <td>N<sub>c</sub> bytes of data</td>
    </tr>
    <tr>
        <td>L<sub>e</sub></td>
        <td>Absent or 1</td>
        <td>Encodes the maximum number (N<sub>e</sub>) of response bytes expected<br />
            (3 bytes for Extended APDU, not supported)</td>
    </tr>
    <tr>
        <th colspan="3">Response APDU</th>
    </tr>
    <tr>
        <td>Response data</td>
        <td>N<sub>r</sub> (at most N<sub>e</sub>)</td>
        <td>Response data</td>
    </tr>
    <tr>
        <td>SW1-SW2<br />
        (Command status)</td>
        <td>2</td>
        <td>Command processing status</td>
    </tr>
</table>
 * @endhtmlonly
 * @{
 */



/**
 * @fn          SendT0
 * @brief       Send an APDU to the card and get the response
 * @param [in]  slotid          Slot Id to activate
 * @param [in]  TxBuff          APDU to send buffer
 * @param [in]  TxLen           Length of the APDU to transmit
 * @param [out] RxBuff          Receive buffer (Must be at least 2 bytes in case 1)
 * @param [out] RxLen           pointer on the received length
 * @param [in]  pfSendWTE       pointer on the WTX handler (if not in EMV mode)
 *
 * @return it returns ICC_OK on success or an #IccReturn_t return ocode
 */
IccReturn_t SendT0(uint8_t slotid, uint8_t *TxBuff, uint32_t TxLen,
                   uint8_t *RxBuff, uint32_t *RxLen,
                   void (*pfSendWTE)(void));
#define T0_MAX_APDU_SIZE		258 /*256 + status */

/** @} */ /*@defgroup PROTOCOL_T0    */
/** @} */ /*@file    protocolt0.h    */
#endif /* _PROTOCOL_T0_H_*/
