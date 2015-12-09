/*
 * sc_regs.h -- Smartcard UART registers definition
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

#ifndef _SC_REGISTERS_H_
#define _SC_REGISTERS_H_
#include <stdint.h>


/* Ask GCC to NOT pack the structures */
#pragma pack(push)
#pragma pack()

/** @file    sc_regs.h Smartcard UART register definition
 *  @version 2.0.3
 *  @date    2015/01/15
 *  @author  Jean-Marc Simohand <jeanmarc.simohand@maximintegrated.com>
 *
 * @{
 */

/** @defgroup GCR_REGISTERS Global Control Register -- Smartcard Clock Control Register
 * @ingroup SMARTCARD_DRIVER
 *
 * This file defines the smartcard clock control (GCR.SCCK) registers.
 *
 *
 * @note    The definition are compliant with the MAX32590 User Guide rev.F\n
 *          This applies to MAX32590, MAX32550 and MAX32555
 * @{
 */

/** @def MAX3255x_SC_BASE_ADDRESS
 *  @brief MAX32550 & MAX32555 Smartcard UART physical address
 */
#define MAX3255x_SC_BASE_ADDRESS        0x4002C000   /**< Smartcard UART physical address for MAX3255x */

/** @def MAX32590_SC0_BASE_ADDRESS
 *  @brief MAX32590 Smartcard UART0 physical address
 */
#define MAX32590_SC0_BASE_ADDRESS       0xFFE60000   /**< Smartcard UART0 physical address for MAX32590 */

/** @def MAX32590_SC1_BASE_ADDRESS
 *  @brief MAX32590 Smartcard UART1 physical address
 */
#define MAX32590_SC1_BASE_ADDRESS       0xFFE61000   /**< Smartcard UART1 physical address for MAX32590 */

/** @defgroup GCR_SCCK_REG  Global Control Register - Smarcard Clock Control
 * <table border="3">
    <tr> <th colspan="3">GCR.SCCK Register mapping</th> </tr>
    <tr> <th>Name</th>     <th>bits</th>   <th>descripion</th> </tr>
    <tr> <th>SC0FRQ</th>   <th>0:6</th>    <th>Smarcard 0 Clock Frequency divider
                                               <br>(F<sub>sc0</sub> = F<sub>pll</sub>/(4*SC0FRQ)) </th></tr>
    <tr> <th>SC1FRQ</th>   <th>6:11</th>   <th>Smarcard 1 Clock Frequency divider
                                               <br>(F<sub>sc1</sub> = F<sub>pll</sub>/(4*SC1FRQ))
                                               <br>(Applicable only for MAX32590) </th></tr>
    <tr> <th>RFU</th>      <th>12:31</th>  <th>Reserved for Future Use (RFU) </th></tr>
   </table>
 * @{
 */
 /** @typedef    SCClockControlBits_t Smartcard Clock Control register bit field (GCR.SCCK register).
 */
typedef struct {
    uint32_t       :20;     /**< 20 Most significant bits are Reserved for
                             *   future use (RFU).\n
                             *   These must not be written
                             */
    uint32_t    SC1FRQ:6;   /**< Smarcard 1 Clock Frequency.\n
                             * These bits define the Smarcart 1 clock frequency\n
                             * Fsc1 = Fpll/(4*SC0FRQ).
                             */
    uint32_t    SC0FRQ:6;   /**< Smarcard 0 Clock Frequency.\n
                             * These bits define the Smarcart 0 clock frequency\n
                             * Fsc0 = Fpll/(4*SC0FRQ).
                             */

} SCClockControlBits_t;

/** @typedef    SCClockControl_t SCClockControl_t is an union between a 32bit word and a \n
 *                               Smartcard Control register bit field (SCClockControlBits_t, GCR.SCCK)
*/
typedef union {
    uint32_t                 word;   /**< word (32bits) access to the register*/
    SCClockControlBits_t     bits;   /**< bits access to the register*/
} SCClockControl_t;
 /** @} */ /*GCR_SCCK_REG*/

/** @} */ /* @defgroup GCR_REGISTERS */
/** @} */ /* 3rd closing brace, I do not understand why I do need it ! */







/** @defgroup SMARCARD_REGISTERS Smartcard Block Registers definition
 *
 * @ingroup SMARTCARD_DRIVER
 *
 * This file defines the smartcard block registers.
 *
 * @note    The definition are compliant with the MAX32590 User Guide rev.F\n
 *          This applies to MAX32590, MAX32550 and MAX32555
 * @{
 */


/** @defgroup SC_REGISTERS_GROUP  Smartcard Register Mapping
 * <table border="3">
    <tr> <th colspan="3">Smartcard Register Mapping</th> </tr>
    <tr> <th>Reg Name</th>    <th>Offset</th>  <th>Descripion</th> </tr>
    <tr> <th>SC_CR</th>       <th>0x00</th>   <th>@ref SC_CR_GROUP</th></tr>
    <tr> <th>SC_SR</th>       <th>0x04</th>   <th>@ref SC_SR_GROUP</th></tr>
    <tr> <th>SC_PN</th>       <th>0x08</th>   <th>@ref SC_PIN_GROUP</th></tr>
    <tr> <th>SC_ETUR</th>     <th>0x0C</th>   <th>@ref SC_ETU_GROUP</th></tr>
    <tr> <th>SC_GTR</th>      <th>0x10</th>   <th>@ref SC_GTR_GROUP</th></tr>
    <tr> <th>SC_WT0R</th>     <th>0x14</th>   <th>@ref SC_WT0R_GROUP</th></tr>
    <tr> <th>SC_WT1R</th>     <th>0x18</th>   <th>@ref SC_WT1R_GROUP</th></tr>
    <tr> <th>SC_IER</th>      <th>0x1C</th>   <th>@ref SC_IER_GROUP</th></tr>
    <tr> <th>SC_ISR</th>      <th>0x20</th>   <th>@ref SC_ISR_GROUP</th></tr>
    <tr> <th>SC_TXR</th>      <th>0x24</th>   <th>@ref SC_TXR_GROUP</th></tr>
    <tr> <th>SC_RXR</th>      <th>0x28</th>   <th>@ref SC_RXR_GROUP</th></tr>
    <tr> <th>SC_CCR</th>      <th>0x2C</th>   <th>@ref SC_CCR_GROUP</th></tr>

   </table>
 * @{
 */
/** @enum       SCUartReg_e  SCUartReg_e is the Smarcard UART registers offset
 *  @brief      Smartcard UART register offsets
 *
 * SCUartReg_e is the Smarcard UART registers offset.\n
 * it also provide the name of the union which correponds to each register.
 *
*/
typedef enum {
    SC_CR  = 0x00, /**< @ref SC_CR_GROUP*/
    SC_SR  = 0x04, /**< @ref SC_SR_GROUP*/
    SC_PN  = 0x08, /**< @ref SC_PIN_GROUP*/
    SC_ETUR= 0x0C, /**< @ref SC_ETU_GROUP*/
    SC_GTR = 0x10, /**< @ref SC_GTR_GROUP*/
    SC_WT0R= 0x14, /**< @ref SC_WT0R_GROUP*/
    SC_WT1R= 0x18, /**< @ref SC_WT1R_GROUP*/
    SC_IER = 0x1C, /**< @ref SC_IER_GROUP*/
    SC_ISR = 0x20, /**< @ref SC_ISR_GROUP*/
    SC_TXR = 0x24, /**< @ref SC_TXR_GROUP*/
    SC_RXR = 0x28, /**< @ref SC_RXR_GROUP*/
    SC_CCR = 0x2C, /**< @ref SC_CCR_GROUP*/
    UART_CONTEXT_LEN = SC_CCR+sizeof(uint32_t),
} SCUartReg_e;
/** @} */ /*SC_REGISTERS_GROUP*/


/** @defgroup SC_CR_GROUP  Smarcard Control Register (SC_CR)
 * <table border="3">
    <tr> <th colspan="3">Smarcard Control Register (SC_CR) mapping</th> </tr>
    <tr> <th>Name</th>     <th>Bits</th>   <th>Descripion</th> </tr>
    <tr> <th>CONV</th>     <th>0</th>      <th> Convention select bit.
                                                <br>0 -> Direct convention,
                                                <br>1 -> inverse convention)</th></tr>
    <tr> <th>CREP</th>     <th>1</th>      <th> Character Repeat Enable bit. Enables character
                                                <br>retransmit on parity error</th></tr>
    <tr> <th>WTEN</th>     <th>2</th>      <th> Waiting time counter enable bit</th></tr>
    <tr> <th>UART</th>     <th>3</th>      <th> Smartcard UART mode bit.
                                                <br>Selects manual control or smartcard
                                                <br>UART control of smartcard pins</th></tr>
    <tr> <th>CCEN</th>     <th>4</th>      <th> Clock Counter enable (set to 1 to start)</th></tr>
    <tr> <th>RXFLUSH</th>  <th>5</th>      <th> Receive FIFO flush bit (set to 1 to flush)</th></tr>
    <tr> <th>TXFLUSH</th>  <th>6</th>      <th> Transmit FIFO flush bit (set to 1 to flush)</th></tr>
    <tr> <th>RFU</th>      <th>7</th>      <th> Reserved for Future Use (RFU)</th></tr>
    <tr> <th>RXTHD</th>    <th>8:11</th>   <th> Receive FIFO depth (valid range: [1..7])</th></tr>
    <tr> <th>TXTHD</th>    <th>12:15</th>  <th> Transmit FIFO depth (valid range: [1..7])</th></tr>
    <tr> <th>START</th>    <th>16</th>     <th> Smartcard Activation sequence (only on MAX3255x)</th></tr>
    <tr> <th>BYP_PHY</th>  <th>17</th>     <th> Smartcard PHY Bypass (only on MAX3255x)</th></tr>
    <tr> <th>PRPOL</th>    <th>18</th>     <th> Smartcard Presence detect polarity (only on MAX3255x)</th></tr>
    <tr> <th>DEBNCE</th>   <th>19</th>     <th> Smartcard insertion debounce enable (only on MAX3255x)</th></tr>
    <tr> <th>BYP_SEQ</th>  <th>20</th>     <th> Smartcard Sequencer delay (only on MAX3255x)</th></tr>
    <tr> <th>SEQ_DLY</th>  <th>21:22</th>  <th> Smartcard Sequencer delay (only on MAX3255x)</th></tr>
    <tr> <th>DUAL_MODE</th><th>23</th>     <th> DUAL Smartcard + SAM mode (only on MAX3255x)</th></tr>
    <tr> <th>RFU</th>      <th>14:31</th>  <th>Reserved for Future Use (RFU) </th></tr>
   </table>
 * @{
 */


/** @typedef    SCControlBits_t
 */
/** @brief  Smarcart Control register bit field (SC_CR register).
*/
typedef struct {
    uint32_t    CONV:1;     /**< Convention select bit. (0->Direct convention,
                             *   1->inverse convention) */
    uint32_t    CREP:1;     /**< Character Repeat Enable bit. Enables character
                             *   retransmit on parity error */
    uint32_t    WTEN:1;     /**< Waiting time counter enable bit.
                             *   (set to 1 to start the waiting time counter)*/
    uint32_t    UART:1;     /**< Smartcard UART mode bit. Selects manual control or
                             *   smartcard UART control of smartcard pins*/
    uint32_t    CCEN:1;     /**< Clock Counter enable (set to 1 to start)*/
    uint32_t    RXFLUSH:1;  /**< Receive FIFO flush bit (set to 1 to flush)*/
    uint32_t    TXFLUSH:1;  /**< Transmit FIFO flush bit (set to 1 to flush)*/
    uint32_t       :1;      /**< Reserved bit (RFU). It must not be written */
    uint32_t    RXTHD:4;    /**< Receive FIFO depth (valid range: [1..7])*/
    uint32_t    TXTHD:4;    /**< Transmit FIFO depth (valid range: [1..7])*/
    uint32_t    START:1;    /**< Smartcard Activation sequence (only on MAX3255x)*/
    uint32_t    BYP_PHY:1;  /**< Smartcard PHY Bypass (only on MAX3255x)*/
    uint32_t    PRPOL:1;    /**< Smartcard Presence detect polarity (only on MAX3255x)*/
    uint32_t    DEBNCE:1;   /**< Smartcard insertion debounce enable (only on MAX3255x)*/
    uint32_t    BYP_SEQ:1;  /**< Smartcard Sequencer bypass (only on MAX3255x)*/
    uint32_t    SEQ_DLY:2;  /**< Smartcard Sequencer delay (only on MAX3255x)*/
    uint32_t    DUAL_MODE:1;/**< DUAL Smartcard + SAM mode (only on MAX32550-B1)*/
    uint32_t       :8;      /**< 8 Most significant bits are Reserved for
                             *   future use (RFU).\n
                             *   These must not be written
                             */

} SCControlBits_t;

/** @typedef    SCControl_t
 */

/**
 * @brief SCControl_t is an union between a 32bit word and a \n
 * Smarcart Control register bit field (#SCControlBits_t, SC_CR)
 */
typedef union {
    uint32_t            word;   /**< word (32bits) access to the register*/
    SCControlBits_t     bits;   /**< bits access to the register*/
} SCControl_t;

/** @} */ /*SC_CR_GROUP*/






/** @defgroup SC_SR_GROUP  Smarcard Status Register (SC_SR)
 * <table border="3">
    <tr> <th colspan="3">Smarcard Status Register (SC_SR) mapping</th> </tr>
    <tr> <th>Name</th>    <th>Bits</th>   <th>Descripion</th> </tr>
    <tr> <th>PAR</th>     <th>0</th>      <th> Parity Error Detected Flag.
                                               <br>set by the HW on parity error</th></tr>
    <tr> <th>WTOV</th>    <th>1</th>      <th> Waiting Time Counter Overflow
                                               <br>Set by the HW when Waiting Time counter
                                               <br>overflows </th></tr>
    <tr> <th>CCOV</th>    <th>2</th>      <th> Clock Counter Overflow bit.
                                               <br>Set by the HW when the Clock counter has
                                               <br>reached its maximum value</th></tr>
    <tr> <th>TXCF</th>    <th>3</th>      <th> Transmit Complete Flag. Set by the HW when
                                               <br>a char has been transmitted,
                                               <br>cleared by SW</th></tr>
    <tr> <th>RXEMPTY</th> <th>4</th>      <th> Set when Rx FIFO is empty (RXELT = 0)</th></tr>
    <tr> <th>RXFULL</th>  <th>5</th>      <th> Set when Rx FIFO is full (RXELT = 8)</th></tr>
    <tr> <th>TXEMPTY</th> <th>6</th>      <th> Set when Tx FIFO is empty (TXELT = 0)</th></tr>
    <tr> <th>TXFULL</th>  <th>7</th>      <th> Set when Tx FIFO is full (TXELT = 8)</th></tr>
    <tr> <th>RXELT</th>   <th>8:11</th>   <th> Number of bytes in Receive FIFO</th></tr>
    <tr> <th>TXELT</th>   <th>12:15</th>  <th> Number of bytes in Transmit FIFO</th></tr>
    <tr> <th>PRES</th>    <th>16</th>     <th> Smartcard Presence (only on MAX3255x)</th></tr>
    <tr> <th>RFU</th>     <th>17</th>     <th> Reserved for Future Use</th></tr>
    <tr> <th>PRC</th>     <th>18</th>     <th> Smartcard Over current protection status
                                               <br>(only on MAX3255x)</th></tr>
    <tr> <th>PDL</th>     <th>19</th>     <th> Presence detect level status (only on MAX3255x)</th></tr>
    <tr> <th>ACTIV</th>   <th>20</th>     <th> Activation sequence status (only on MAX3255x)</th></tr>
    <tr> <th>RFU</th>     <th>21:31</th>  <th> Reserved for Future Use</th></tr>
   </table>
 * @{
 */

/** @typedef    SCStatusBits_t
 */
/** @brief Smarcart Status register bit field (SC_SR register). */
typedef struct {
    uint32_t    PAR:1;     /**< Parity Error Detected Flag.
                             *   set by the HW on parity error */
    uint32_t    WTOV:1;     /**< Waiting Time Counter Overflow
                             *   Set by the HW when Waiting Time counter
                             *   overflows */
    uint32_t    CCOV:1;     /**< Clock Counter Overflow bit.
                             *   Set by the HW when the Clock counter has
                             *   reached its maximum value */
    uint32_t    TXCF:1;     /**< Transmit Complete Flag. Set by the HW when
                             *   a char has been transmitted, cleared by SW*/
    uint32_t    RXEMPTY:1;  /**< Set when Rx FIFO is empty (RXELT = 0)*/
    uint32_t    RXFULL:1;   /**< Set when Rx FIFO is full (RXELT = 8)*/
    uint32_t    TXEMPTY:1;  /**< Set when Tx FIFO is empty (TXELT = 0)*/
    uint32_t    TXFULL:1;   /**< Set when Tx FIFO is full (TXELT = 8)*/
    uint32_t    RXELT:4;    /**< Number of bytes in Receive FIFO*/
    uint32_t    TXELT:4;    /**< Number of bytes in Transmit FIFO */
    uint32_t    PRES:1;     /**< Smartcard Presence (only on MAX3255x)*/
    uint32_t    :1;         /**< RFU bit */
    uint32_t    PRC:1;      /**< Smartcard Over current protection status (only on MAX3255x)*/
    uint32_t    PDL:1;      /**< Presence detect level status (only on MAX3255x)*/
    uint32_t    ACTIV:1;    /**< Activation sequence status (only on MAX3255x)*/

    uint32_t       :16;     /**< 16 Most significant bits are Reserved for
                             *   future use (RFU).\n
                             *   These must not be written
                             */

} SCStatusBits_t;

#define MAX_TXFIFO_ELT      7 /**<Tx FIFO length */

/** @typedef    SCStatus_t
 */
/** @brief SCStatus_t is an union between a 32bit word and a \n
 *  Smarcart Status register bit field (#SCStatusBits_t, SC_SR)
 */
typedef union {
    uint32_t            word;   /**< word (32bits) access to the register*/
    SCStatusBits_t      bits;   /**< bits access to the register*/
} SCStatus_t;

/** @} */ /*SC_SR_GROUP*/






/** @defgroup SC_PIN_GROUP  Smarcard PIN Register (SC_PIN)
 * <table border="3">
    <tr> <th colspan="3">Smarcard Pins Register (SC_PIN) mapping</th> </tr>
    <tr> <th>Name</th>     <th>Bits</th>   <th>Descripion</th> </tr>
    <tr> <th>CRDRST</th>   <th>0</th>      <th> Smarcard Reset pin control. \n
                                                <br>Writes to this bit set the associated
                                                <br>pin logic level. Reads from this bit return the
                                                <br>logic value present on the associated pin.</th></tr>
    <tr> <th>CRDCLK</th>   <th>1</th>      <th> Smarcard Clock pin control. \n
                                                <br>Writes to this bit set the associated
                                                <br>pin logic level. Reads from this bit return the
                                                <br>logic value present on the associated pin.</th></tr>
    <tr> <th>CRDIO</th>    <th>2</th>      <th> Smarcard I/O pin control. \n
                                                <br>Writes to this bit set the associated
                                                <br>pin logic level. Reads from this bit return the
                                                <br>logic value present on the associated pin.</th></tr>
    <tr> <th>CRDC4</th>    <th>3</th>      <th> Smarcard C4 pin control. \n
                                                <br>Writes to this bit set the associated
                                                <br>pin logic level. Reads from this bit return the
                                                <br>logic value present on the associated pin.</th></tr>
    <tr> <th>CRDC8</th>    <th>4</th>      <th> Smarcard C8 pin control. \n
                                                <br>Writes to this bit set the associated
                                                <br>pin logic level. Reads from this bit return the
                                                <br>logic value present on the associated pin.</th></tr>
    <tr> <th>CLKSEL</th>   <th>5</th>      <th> Smarcard Clock control select. \n
                                                <br>if set, Clock pin is controlled by the UART
                                                <br>otherwise, Clock pin is controlled by CRDCLK value</th></tr>
    <tr> <th>RFU</th>      <th>6:7</th>    <th> Reserved for Future Use</th></tr>
    <tr> <th>VCCSEL</th>   <th>8:9</th>    <th> Smartcard VCC voltage (only for MAX3255x, cf #IccVoltage_t)</th></tr>
    <tr> <th>RFU</th>      <th>10:15</th>  <th> Reserved for Future Use</th></tr>
    <tr> <th>IO_C48_EN</th><th>16</th>     <th> C4 C8 manual control enable (only for MAX3255x)</th></tr>
    <tr> <th>CLK_EN</th>   <th>17</th>     <th> Clock manual control enable (only for MAX3255x)</th></tr>
    <tr> <th>RST_EN</th>   <th>18</th>     <th> Reset manual control enable (only for MAX3255x)</th></tr>
    <tr> <th>VCC_EN</th>   <th>19</th>     <th> VCC  manual control enable (only for MAX3255x)</th></tr>
    <tr> <th>RAMP_EN</th>  <th>20</th>     <th> RAMP manual control enable (only for MAX3255x)</th></tr>
    <tr> <th>RFU</th>      <th>21:31</th>  <th> Reserved for Future Use</th></tr>
   </table>
 * @{
 */

/** @typedef    SCPinBits_t
 */
/** @brief Smarcart Pins register bit field (SC_PN register).
*/
typedef struct {
    uint32_t    CRDRST:1;   /**< Smarcard Reset pin control. \n
                             *   Writes to this bit set the associated
                             *   pin logic level. Reads from this bit return the
                             *   logic value present on the associated pin.*/
    uint32_t    CRDCLK:1;   /**< Smarcard Clock pin control. \n
                             *   Writes to this bit set the associated
                             *   pin logic level. Reads from this bit return the
                             *   logic value present on the associated pin.*/
    uint32_t    CRDIO:1;    /**< Smarcard I/O pin control. \n
                             *   Writes to this bit set the associated
                             *   pin logic level. Reads from this bit return the
                             *   logic value present on the associated pin.*/
    uint32_t    CRDC4:1;    /**< Smarcard C4 pin control. \n
                             *   Writes to this bit set the associated
                             *   pin logic level. Reads from this bit return the
                             *   logic value present on the associated pin.*/
    uint32_t    CRDC8:1;    /**< Smarcard C8 pin control. \n
                             *   Writes to this bit set the associated
                             *   pin logic level. Reads from this bit return the
                             *   logic value present on the associated pin.*/
    uint32_t    CLKSEL:1;   /**< Smarcard clock control bit, when set
                             *   Clock pin is controlled by the UART, when
                             *   cleared it is controlled by CRDCLK bit */
    uint32_t       :2;      /**< 2 bits are Reserved for future use (RFU).\n
                             *   These must not be written
                             */
    uint32_t    VCCSEL:2;   /**< Smartcard VCC voltage (only for MAX3255x, cf #IccVoltage_t) */
    uint32_t       :6;      /**< 6 bits are Reserved for future use (RFU).\n
                             *   These must not be written
                             */
    uint32_t    IO_C48_EN:1;/**< C4 C8 manual control (only for MAX3255x) */
    uint32_t    CLK_EN:1;   /**< Clock manual control (only for MAX3255x) */
    uint32_t    RST_EN:1;   /**< Reset manual control (only for MAX3255x) */
    uint32_t    VCC_EN:1;   /**< VCC  manual control (only for MAX3255x) */
    uint32_t    RAMP_EN:1;  /**< RAMP manual control (only for MAX3255x) */
    uint32_t        :11;    /**< 11 bits are Reserved for future use (RFU) */


} SCPinBits_t;


/** @typedef SCPin_t
 */

/** @brief  SCPin_t is an union between a 32bit word and a \n
 *          Smarcart Pins register bit field (#SCPinBits_t, SC_PN)
*/
typedef union {
    uint32_t            word;       /**< word (32bits) access to the register*/
    SCPinBits_t         bits;       /**< bits access to the register*/
} SCPin_t;

/** @} */ /*SC_PIN_GROUP*/






/** @defgroup SC_ETU_GROUP  Smarcard ETU Register (SC_ETU)
 * <table border="3">
    <tr> <th colspan="3">Smarcard Pins Register (SC_PIN) mapping</th> </tr>
    <tr> <th>Name</th>   <th>Bits</th>   <th>Descripion</th> </tr>
    <tr> <th>ETU</th>    <th>0:14</th>   <th> Elemental Time Unit. (ETU)
                                              <br> If Half=0, this register must contains the
                                              <br> number of clocks in one asynchronous bit
                                              <br> time
                                              <br> if Half=1, this register must contains the
                                              <br> number of clocks in half of one asynchronous
                                              <br> bit time</th></tr>
    <tr> <th>COMP</th>   <th>15</th>     <th> Compensation Mode Enable bit.\n
                                              <br> Controls 1 clock cycle substraction of ETU
                                              <br> value on odd bits. (Enabled when set)</th></tr>
    <tr> <th>HALF</th>   <th>16</th>     <th> Half ETU Count Selection bit. \n
                                              <br> NOTE: Guard Time (GT) and Wait Time (WT)
                                              <br> must be doubled if HALF=1. \n
                                              <br> When Set, ETU value represents required clocks
                                              <br> for one half ETU, \n
                                              <br> When cleared, ETU value represents required
                                              <br> clocks for one ETU</th></tr>
    <tr> <th>RFU</th>    <th>17:31</th>  <th> Reserved for Future Use</th></tr>
   </table>
 * @{
 */

/** @typedef    SCEtuBits_t
 */
/** @brief Smarcart ETU register bit field (SC_ETU register).
*/
typedef struct {
    uint32_t    ETU:15;     /**< Elemental Time Unit. \n
                             *   If Half=0, this register must contains the
                             *   number of clocks in one asynchronous bit
                             *   time\n
                             *   if Half=1, this register must contains the
                             *   number of clocks in half of one asynchronous
                             *   bit time*/
    uint32_t    COMP:1;     /**< Compensation Mode Enable bit.\n
                             *   Controls 1 clock cycle substraction of ETU
                             *   value on odd bits.\n
                             *   (Enabled when set)*/
    uint32_t    HALF:1;     /**< Half ETU Count Selection bit. \n
                             *   NOTE: Guard Time (GT) and Wait Time (WT)
                             *   must be doubled if HALF=1. \n
                             *   When Set, ETU value represents required clocks
                             *   for one half ETU, \n
                             *   When cleared, ETU value represents required
                             *   clocks for one ETU*/
    uint32_t        :15;     /**< 15 Most significant bits are Reserved for
                             *   future use (RFU).\n
                             *   These must not be written
                             */
} SCEtuBits_t;


/** @def SCI_ETUR_COMP
 *  @brief Compensation Mode Enable bit
 */
#define SCI_ETUR_COMP       (1<<16)/**< Compensation Mode Enable bit.\n
                                     *   Controls 1 clock cycle substraction of ETU
                                     *   value on odd bits.\n
                                     *   (Enabled when set)*/

/** @typedef    SCEtu_t
 */

/** @brief SCEtu_t is an union between a 32bit word and a \n
 *         Smarcart ETU register bit field (#SCEtuBits_t, SC_ETU)
*/
typedef union {
    uint32_t            word;       /**< word (32bits) access to the register*/
    SCEtuBits_t         bits;       /**< bits access to the register*/
} SCEtu_t;

/** @}*/ /*SC_ETU_GROUP*/










/** @defgroup SC_GTR_GROUP  Smartcard GTR Register (SC_GTR)
 * <table border="3">
    <tr> <th colspan="3">Smartcard Guard Time Register (SC_GTR) mapping</th> </tr>
    <tr> <th>Name</th>   <th>Bits</th>   <th>Descripion</th> </tr>
    <tr> <th>GT </th>    <th>0:15</th>   <th> Guard Time.
                                              <br>Minimum time between two consecutive start bits
                                              <br>in transmit mode. (Expressed in ETUs)
                                              <br>Note: GT must be doubled if SC_ETU.HALF = 1
                                              <br>Note: Writes to this register may be delayed
                                              <br>up to two ETU cycles.</th></tr>
    <tr> <th>RFU</th>    <th>16:31</th>  <th> Reserved for Future Use</th></tr>
   </table>
 * @{
 */

/** @typedef    SCGTRBits_t
 */

/** @brief Smartcart Guard Time register bit field (SC_GTR register).
*/
typedef struct {
    uint32_t    GT:16;      /**< Guard Time. \n
                             *   Minimum time between two consecutive start bits
                             *   in transmit mode. (Expressed in ETUs)
                             *   Note: GT must be doubled if SC_ETU.HALF = 1
                             *   Note: Writes to this register may be delayed
                             *   up to two ETU cycles.*/
    uint32_t      :16;     /**< 16 Most significant bits are Reserved for
                             *   future use (RFU).\n
                             *   These must not be written
                             */

} SCGTRBits_t;


/** @typedef    SCGTR_t
 */

/** @brief SCGTR_t is an union between a 32bit word and a \n
 *         Smarcart Guard Time register bit field (#SCGTRBits_t, SC_GTR).
 *
 * The Guard Time Register counts the number of ETU cycles
 * required between start bit edges on transmit
*/
typedef union {
    uint32_t            word;       /**< word (32bits) access to the register*/
    SCGTRBits_t         bits;       /**< bits access to the register*/
} SCGTR_t;

/** @} */ /*SC_GTR_GROUP */




/** @defgroup SC_WT0R_GROUP  Smartcard Waiting Time Register (Least significant bits)(SC_WT0R)
 * <table border="3">
    <tr> <th colspan="3">Smartcard Waiting Time Register (SC_WT0R) mapping</th> </tr>
    <tr> <th>Name</th>   <th>Bits</th>   <th>Descripion</th> </tr>
    <tr> <th>WT</th>     <th>0:31</th>   <th> Wait Time.
                                              <br> Least Significant 32bits of the Wait Time
                                              <br> Counter expressed in ETUs
                                              <br> Note: WT0R must be doubled if SC_ETU.HALF = 1
                                              <br> Note: Writes to this register may be delayed
                                              <br> up to two ETU cycles.</th></tr>
   </table>
 * @{
 */

/** @typedef    SCWT0RBits_t
 */
/** @brief Smarcart Wait Time 0 register bit field (SC_WT0R register).
*/
typedef struct {
    uint32_t    WT;         /**< Wait Time. \n
                             *   Least Significant 32bits of the Wait Time
                             *   Counter expressed in ETUs
                             *   Note: WT0R must be doubled if SC_ETU.HALF = 1
                             *   Note: Writes to this register may be delayed
                             *   up to two ETU cycles.*/

} SCWT0RBits_t;

/** @typedef    SCWT0_t
 */
/** @brief SCWT0_t is an union between a 32bit word and a \n
 *         Smarcart Guard Time register bit field (SCWT0RBits_t, SC_WT0R).
 *
 * Writes to this register set the reload value and stop the
 * current Wait Time counter
 *
 * @note SC_WT register is autoreload.
 */
typedef union {
    uint32_t            word;       /**< word (32bits) access to the register*/
    SCWT0RBits_t        bits;       /**< bits access to the register*/
} SCWT0_t;

/** @} */ /*SC_WT0R_GROUP*/




/** @defgroup SC_WT1R_GROUP  Smartcard Waiting Time Register (Most significant bits)(SC_WT1R)
 * <table border="3">
    <tr> <th colspan="3">Smartcard Waiting Time Register (SC_WT1R) mapping</th> </tr>
    <tr> <th>Name</th>   <th>Bits</th>   <th>Descripion</th> </tr>
    <tr> <th>WT</th>     <th>0:7</th>    <th> Wait Time.
                                              <br> Most Significant 8bits of the Wait Time
                                              <br> Counter expressed in ETUs
                                              <br> Note: WT0R must be doubled if SC_ETU.HALF = 1
                                              <br> Note: Writes to this register may be delayed
                                              <br> up to two ETU cycles.</th></tr>
    <tr> <th>RFU</th>    <th>8:31</th>  <th> Reserved for Future Use</th></tr>
   </table>
 * @{
 */
/** @typedef    SCWT1RBits_t
 */
/** @brief Smarcart Wait Time 1 register bit field (SC_WT1R register).
*/
typedef struct {
    uint32_t    WT:8;       /**< Wait Time. \n
                             *   Most Significant 8bits of the Wait Time
                             *   Counter expressed in ETUs
                             *   Note: WT0R must be doubled if SC_ETU.HALF = 1
                             *   Note: Writes to this register may be delayed
                             *   up to two ETU cycles.*/
    uint32_t      :24;     /**< 24 Most significant bits are Reserved for
                             *   future use (RFU).\n
                             *   These must not be written
                             */

} SCWT1RBits_t;

/** @typedef    SCWT1_t
 */

/** @brief SCWT1_t is an union between a 32bit word and a \n
 *         Smarcart Guard Time register bit field (#SCWT1RBits_t, SC_WT1R).
 *
 * Writes to this register set the reload value and stop the
 * current Wait Time counter
 */
typedef union {
    uint32_t            word;       /**< word (32bits) access to the register*/
    SCWT1RBits_t        bits;       /**< bits access to the register*/
} SCWT1_t;

/** @} */ /*SC_WT1R_GROUP */






/** @defgroup SC_IER_GROUP  Smartcard Interrupt Enable Register (SC_IER)
 * <table border="3">
    <tr> <th colspan="3">Smartcard Interrupt Enable Register (SC_IER) mapping</th> </tr>
    <tr> <th>Name</th>    <th>Bits</th>  <th>Descripion</th> </tr>
    <tr> <th>PARIE</th>   <th>0</th>     <th>Parity Error Interrupt Enable </th></tr>
    <tr> <th>WTIE</th>    <th>1</th>     <th>Waiting Time Overflow Interrupt Enable</th></tr>
    <tr> <th>CTIE</th>    <th>2</th>     <th>Clock Counter Overflow Interrupt Enable</th></tr>
    <tr> <th>TCIE</th>    <th>3</th>     <th>Character Transmission Completion Interrupt Enable</th></tr>
    <tr> <th>RXEIE</th>   <th>4</th>     <th>Receive FIFO Empty Interrupt Enable</th></tr>
    <tr> <th>RXTIE</th>   <th>5</th>     <th>Receive FIFO Threshold Reached Interrupt Enable</th></tr>
    <tr> <th>RXFIE</th>   <th>6</th>     <th>Receive FIFO Full Interrupt Enable</th></tr>
    <tr> <th>TXEIE</th>   <th>7</th>     <th>Transmit FIFO Empty Interrupt Enable</th></tr>
    <tr> <th>TXTIE</th>   <th>8</th>     <th>Transmit FIFO Threshold Reached Interrupt Enable</th></tr>
    <tr> <th>PRCIE</th>   <th>9</th>     <th>Protection Interrupt Enable (overcurrent) (only for MAX3255x)</th></tr>
    <tr> <th>PDLIE</th>   <th>10</th>    <th>Presence Detect Interrupt Enable (only for MAX3255x)</th></tr>
    <tr> <th>ACTIVIE</th> <th>11</th>    <th>Activation Sequencer Interrupt Enable (only for MAX3255x)</th></tr>
    <tr> <th>RFU</th>     <th>12:31</th> <th> Reserved for Future Use </th></tr>
   </table>
 * @{
 */
/** @typedef    SCIERBits_t
 */

/** @brief Smartcard Interrupt Enable register bit field.
 *
 * Smartcard Interrupt Enable register bit field (SC_IER register).\n
 * Interrupt is enabled when the bit is set.
*/
typedef struct {
    uint32_t    PARIE:1;    /**< Parity Error Interrupt Enable */
    uint32_t    WTIE:1;     /**< Waiting Time Overflow Interrupt Enable*/
    uint32_t    CTIE:1;     /**< Clock Counter Overflow Interrupt Enable*/
    uint32_t    TCIE:1;     /**< Character Transmission Completion Interrupt Enable.*/
    uint32_t    RXEIE:1;    /**< Receive FIFO Empty Interrupt Enable */
    uint32_t    RXTIE:1;    /**< Receive FIFO Threshold Reached Interrupt Enable */
    uint32_t    RXFIE:1;    /**< Receive FIFO Full Interrupt Enable */
    uint32_t    TXEIE:1;    /**< Transmit FIFO Empty Interrupt Enable */
    uint32_t    TXTIE:1;    /**< Transmit FIFO Threshold Reached Interrupt Enable */
    uint32_t    PRCIE:1;    /**< Protection Interrupt Enable (overcurrent) */
    uint32_t    PDLIE:1;    /**< Presence Detect Interrupt Enable */
    uint32_t    ACTIVIE:1;  /**< Activation Sequencer Interrupt Enable */
    uint32_t       :20;     /**< 20 Most significant bits are Reserved for
                             *   future use (RFU).\n
                             *   These must not be written
                             */

} SCIERBits_t;

/** @typedef    SCIER_t
 */

/** @brief SCIER_t is an union between a 32bit word and a \n
 *         Smartcard Interrupt Enable register bit field (SCIERBits_t, SC_IER)
*/
typedef union {
    uint32_t            word;       /**< word (32bits) access to the register*/
    SCIERBits_t         bits;       /**< bits access to the register*/
} SCIER_t;
/** @} */ /*SC_IER_GROUP*/






/** @defgroup SC_ISR_GROUP  Smartcard Interrupt Status Register (SC_ISR)
 * <table border="3">
    <tr> <th colspan="3">Smartcard Interrupt Status Register (SC_ISR) mapping</th> </tr>
    <tr> <th>Name</th>    <th>Bits</th>  <th>Descripion</th> </tr>
    <tr> <th>PARIS</th>   <th>0</th>     <th>Parity Error Interrupt Status Flag </th></tr>
    <tr> <th>WTIS</th>    <th>1</th>     <th>Waiting Time Overflow Interrupt Status Flag</th></tr>
    <tr> <th>CTIS</th>    <th>2</th>     <th>Clock Counter Overflow Interrupt Status Flag</th></tr>
    <tr> <th>TCIS</th>    <th>3</th>     <th>Character Transmission Completion Interrupt Status Flag</th></tr>
    <tr> <th>RXEIS</th>   <th>4</th>     <th>Receive FIFO Empty Interrupt Status Flag</th></tr>
    <tr> <th>RXTIS</th>   <th>5</th>     <th>Receive FIFO Threshold Reached Interrupt Status Flag</th></tr>
    <tr> <th>RXFIS</th>   <th>6</th>     <th>Receive FIFO Full Interrupt Status Flag</th></tr>
    <tr> <th>TXEIS</th>   <th>7</th>     <th>Transmit FIFO Empty Interrupt Status Flag</th></tr>
    <tr> <th>TXTIS</th>   <th>8</th>     <th>Transmit FIFO Threshold Reached Interrupt Status Flag</th></tr>
    <tr> <th>PRCIS</th>   <th>9</th>     <th>Protection Interrupt Status Flag (overcurrent) (only for MAX3255x)</th></tr>
    <tr> <th>PDLIS</th>   <th>10</th>    <th>Presence Detect Interrupt Status Flag (only for MAX3255x)</th></tr>
    <tr> <th>ACTIVIS</th> <th>11</th>    <th>Activation Sequencer Interrupt Status Flag (only for MAX3255x)</th></tr>
    <tr> <th>RFU</th>     <th>12:31</th> <th> Reserved for Future Use</th></tr>
   </table>
 * @{
 */

/** @typedef    SCISRBits_t
 */
/** @brief Smartcard Interrupt Status register bit field
 *
 *  These bits are set by hardware. If the corresponding interrupt
 *  is enabled for this flag, a system interrupt will be fired.
 *  If the interrupt enable is not set, the flag will be set,
 *  but no interrupt will fire. \n
 *  This bit cleared by software.*/
typedef struct {
    uint32_t    PARIS:1;    /**< Parity Error Interrupt Status Flag. */
    uint32_t    WTIS:1;     /**< Waiting Time Overflow Interrupt Status Flag.*/
    uint32_t    CTIS:1;     /**< Clock Counter Overflow Interrupt Status Flag.*/
    uint32_t    TCIS:1;     /**< Character Transmission Completion Interrupt
                             *   Status Flag.*/
    uint32_t    RXEIS:1;    /**< Receive FIFO Empty Interrupt Status Flag. */
    uint32_t    RXTIS:1;    /**< Receive FIFO Threshold Reached Interrupt
                             *   Status Flag.*/
    uint32_t    RXFIS:1;    /**< Receive FIFO Full Interrupt Status Flag.*/
    uint32_t    TXEIS:1;    /**< Transmit FIFO Empty Interrupt Status Flag.*/
    uint32_t    TXTIS:1;    /**< Transmit FIFO Threshold Reached Interrupt
                             * Status Flag.*/
    uint32_t    PRCIS:1;    /**< Over current protection flag (only for MAX3255x)*/
    uint32_t    PDLIS:1;    /**< Presence detect flag (only for MAX3255x)*/
    uint32_t    ACTIVIS:1;  /**< Activation Sequence interrupt flag (only for MAX3255x)*/
    uint32_t       :20;     /**< 20 Most significant bits are Reserved for
                             *   future use (RFU).\n
                             *   These must not be written
                             */

} SCISRBits_t;

/** @typedef    SCISR_t
 */
/**  @brief      SCISR_t is an union between a 32bit word and a \n
 *               Smartcard Interrupt Status register bit field (#SCISRBits_t, SC_ISR)
*/
typedef union {
    uint32_t            word;       /**< word (32bits) access to the register*/
    SCISRBits_t         bits;       /**< bits access to the register*/
} SCISR_t;

/** @}*/ /*SC_ISR_GROUP*/





/** @defgroup SC_TXR_GROUP  Smartcard Transmit Register (SC_TXR)
 * <table border="3">
    <tr> <th colspan="3">Smartcard Transmit Register (SC_TXR) mapping</th> </tr>
    <tr> <th>Name</th>    <th>Bits</th>  <th>Descripion</th> </tr>
    <tr> <th>DATA</th>    <th>0:7</th>   <th>Transmit Data.\n
                                             <br>Writes to this register will load the 8 bit
                                             <br>input data into the TX FIFO </th></tr>
    <tr> <th>RFU</th>     <th>8:31</th>  <th> Reserved for Future Use </th></tr>
   </table>
 * @{
 */

/** @typedef    SCTxBits_t
 */
/** @brief Smartcard Transmit register bitfield (SC_TXR).
 */
typedef struct {
    uint32_t    DATA:8;     /**< Transmit Data.\n
                             *   Writes to this register will load the 8 bit
                             *   input data into the TX FIFO*/
    uint32_t       :24;     /**< 24 Most significant bits are Reserved for
                             *   future use (RFU).\n
                             *   These must not be written
                             */
} SCTxBits_t;

/** @typedef    SCTx_t
 */
/** @brief SCTx_t is an union between a 32bit word and a \n
 *         Smarcart Transmit register bit field (#SCTxBits_t, SC_TXR)
*/
typedef union {
    uint32_t            word;       /**< word (32bits) access to the register*/
    SCTxBits_t          bits;       /**< bits access to the register*/
} SCTx_t;
/** @} */ /*SC_TXR_GROUP*/




/** @defgroup SC_RXR_GROUP  Smartcard Receive Register (SC_RXR)
 * <table border="3">
    <tr> <th colspan="3">Smartcard Receive Register (SC_RXR) mapping</th> </tr>
    <tr> <th>Name</th>    <th>Bits</th>  <th>Descripion</th> </tr>
    <tr> <th>DATA</th>    <th>0:7</th>   <th>Receive Data.\n
                                             <br>Reads from this register will unload the 8 bit
                                             <br>receive data into the RX FIFO</th></tr>
    <tr> <th>PARER</th>   <th>8</th>     <th> Parity Error Detect bit
                                              <br>If set, a parity error has been detected</th></tr>
    <tr> <th>RFU</th>     <th>8:31</th>  <th> Reserved for Future Use </th></tr>
   </table>
 * @{
 */

/** @typedef    SCRxBits_t
 */

/** @brief Smartcard Receive register bitfield (SC_RXR register)
 *
 *
 *  This register provides a read interface to the receive FIFO.\n
 *  Each read of this register contains both the received data
 *  and associated parity error detect bit.
 */
typedef struct {
    uint32_t    DATA:8;     /**< Receive Data.\n
                             *   Reads from this register will unload the 8 bit
                             *   receive data into the RX FIFO*/
    uint32_t    PARER:1;    /**< Parity Error Detect bit.\n
                             *   If set, a parity error has been detected
                             */
    uint32_t       :23;     /**< 23 Most significant bits are Reserved for
                             *   future use (RFU).\n
                             *   These must not be written
                             */
} SCRxBits_t;

/** @typedef    SCRx_t
 */

/** @brief SCRx_t is an union between a 32bit word and a \n
 *         Smarcart Receive register bit field (#SCRxBits_t, SC_RXR)
 *
 *  This register provides a read interface to the receive FIFO.\n
 *  Each read of this register contains both the received data
 *  and associated parity error detect bit.
 */
typedef union {
    uint32_t            word;       /**< word (32bits) access to the register*/
    SCRxBits_t          bits;       /**< bits access to the register*/
} SCRx_t;

/** @} */ /*SC_RXR_GROUP */




/** @defgroup SC_CCR_GROUP  Smartcard Clock Counter Register (SC_CCR)
 * <table border="3">
    <tr> <th colspan="3">Smartcard Clock Counter Register (SC_CCR) mapping</th> </tr>
    <tr> <th>Name</th>    <th>Bits</th>  <th>Descripion</th> </tr>
    <tr> <th>CCYC</th>    <th>0:23</th>   <th>Number of Clock Cycles to count.
                                             <br>Note: Writes to this register may be delayed
                                             <br>up to two ETU cycles.</th></tr>
    <tr> <th>RFU</th>     <th>24:30</th>     <th> Reserved for Future Use</th></tr>
    <tr> <th>MAN</th>     <th>31</th>     <th> Manual Mode.
                                               <br>The counter status counting down on if this
                                               <br>bit is set instead of an automatic HW
                                               <br>mechanism.
                                               <br>If cleared, UART controls this counter
                                               <br>(counter is reloaded at each character
                                               <br> transmission).
                                               <br>If set, Counter is loaded when this register
                                               <br>is written and then stars to decrement at
                                               <br>each clock cycle.</th></tr>
   </table>
 * @{
 */

/** @typedef    SCCCRBits_t
 */
/** @brief Smarcart Clock Counter register bit field (SC_CCR register).
 */
typedef struct {
    uint32_t    CCYC:24;    /**< Number of Clock Cycles to count.\n
                             *   Note: Writes to this register may be delayed
                             *   up to two ETU cycles.*/
    uint32_t       :7;      /**< 7 Reserved for future use (RFU) bits.\n
                             *   These must not be written
                             */
    uint32_t    MAN:1;     /**<  Manual Mode.\n
                             *   The counter status counting down on if this
                             *   bit is set instead of an automatic HW
                             *   mechanism. \n
                             *   If cleared, UART controls this counter
                             *   (counter is reloaded at each character
                             *    transmission).\n
                             *   If set, Counter is loaded when this register
                             *   is written and then stars to decrement at
                             *   each clock cycle.
                             */
} SCCCRBits_t;

/** @typedef    SCCCR_t
 */
/** @brief SCCCR_t is an union between a 32bit word and a \n
 *         Smarcart Clock Counter register bit field (#SCCCRBits_t, SC_CCR).
 */
typedef union {
    uint32_t            word;       /**< word (32bits) access to the register*/
    SCCCRBits_t         bits;       /**< bits access to the register*/
} SCCCR_t;

/** @} */ /*SC_CCR_GROUP*/



/** @} */ /* @defgroup SMARCARD_REGISTERS */
/** @} */ /* @file    sc_regs.h*/

#pragma pack(pop)

#endif /*_SC_REGISTERS_H_*/
