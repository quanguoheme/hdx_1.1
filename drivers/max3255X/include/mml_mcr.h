/*
 * mml_mcr.h --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2013, Maxim Integrated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY MAXIM INTEGRATED ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* [INTERNAL] ------------------------------------------------------------------
 * Created on: Mar 18, 2014
 * Author: Jeremy Brodt <jeremy.brodt@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision: 4616 $: Revision of last commit
 * $Author: robert.muchsel $: Author of last commit
 * $Date: 2015-01-19 13:20:10 -0600 (Mon, 19 Jan 2015) $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

/**
* \file
* \brief    Magnetic Stripe Reader driver
* \details  This driver can be used to configure and operate the Magnetic Stripe
*           Reader. It reads and decodes magnetic stripe data that is encoded
*           according to the ISO/IEC standard 7811.
* \details  This file defines the driver API including data types and function
*           prototypes.
*/

#ifndef _MCR_H_
#define _MCR_H_

#include <stdint.h>

/***** Definitions *****/

#ifndef MCR_NUM_TRACKS
/// Number of tracks on magstripe
#define MCR_NUM_TRACKS      3
#endif

/// Maximum number of samples that can be collected per MML spec
#define MCR_MAX_SAMPLES           1536

/// Maximum number of bits on a 3.375 inch, 210 bpi magstripe
#define MCR_MAX_RAW_LEN_BITS      (709)

/// Maximum number of bytes on a 3.375 inch, 210 bpi magstripe
#define MCR_MAX_RAW_LEN_BYTES     ((MCR_MAX_RAW_LEN_BITS + 7) / 8)

/// Maximum size in bytes of decoded track characters (5-bit min to 8-bit max)
#define MCR_MAX_DEC_LEN   (MCR_MAX_RAW_LEN_BITS / 5)

/// Swipe direction: Forward
#define MCR_FORWARD     0
/// Swipe direction: Reverse
#define MCR_REVERSE     1

/// Error codes
#define MCR_ERR_OK          0x00    /// No error
#define MCR_ERR_BAD_LEN     0x01    /// invalid length parameter
#define MCR_ERR_START_SEN   0x02    /// start sentinel was not found
#define MCR_ERR_END_SEN     0x04    /// end sentinel was not found
#define MCR_ERR_OUTLIER     0x08    /// invalid sample value
#define MCR_ERR_PARAM       0x10    /// invalid parameter
#define MCR_ERR_LRC         0x40    /// invalid LRC (LRC != 0)
#define MCR_ERR_PARITY      0x80    /// parity error

/// Structure to contain result of a track decode
typedef struct {
  uint8_t error_code;   /**< Error code value */
  uint8_t parity_errs;  /**< Number of characters with parity errors */
  uint8_t lrc;          /**< LRC check value. A value of '0' indicates a
                             successful LRC check. Any other value should be
                             considered a failure. */
  uint8_t direction;    /**< Swipe direction determined from decode */
  uint8_t len;          /**< Number or decoded characters. This does not include
                             the sentinels or the LRC. */
  uint16_t speed;       /**< Approximate swipe rate in unit of in/sec * 10 */
  uint8_t data[MCR_MAX_DEC_LEN];  /**< The decoded data */
} mcr_decoded_track_t;

/// MCR sample fields
#ifdef __CC_ARM
#pragma anon_unions
#endif
typedef union {
  struct {
    uint16_t time : 9;
    uint16_t amp  : 7;
  };
  uint16_t value;
} mcr_sample_t;

/// Structure to contain raw MCR samples
typedef struct {
  uint16_t len;
  mcr_sample_t data[MCR_MAX_SAMPLES];
} mcr_samples_t;

/***** Function Prototypes *****/

/**
*   \brief    Initializes magnetic stripe reader hardware
*/
int mcr_init(void);

/**
*   \brief    Initializes specified track
*   \param    track   track number (1 to 3)
*/
void mcr_init_track(unsigned int track);

/**
*   \brief    Enables magnetic stripe reader
*   \pre      The reader should be initialized by calling mcr_init() and then
*             waiting at least 100 us before calling this function.
*/
void mcr_enable(void);

/**
*   \brief    Disables magnetic stripe reader
*/
void mcr_disable(void);

/**
*   \brief    Task used to execute driver functionality.
*   \details  This function executes the internal driver functionality that
*             processes MCR events and reads stripe data. This function is used
*             when MCR interrupt servicing is disabled.
*   \returns  1 if all tracking reading is complete, 0 otherwise
*/
int mcr_task(void);

/**
*   \brief    Decodes the specified track of data
*   \param    track           track number (1 to 3)
*   \param    decoded_track   track decode results
*   \returns  number of characters decoded
*   \note     This function has significant stack usage.
*/
unsigned int mcr_track_decode(unsigned int track, mcr_decoded_track_t * decoded_track);

/**
*   \brief    Registers an application callback function
*   \details  The callback function will be called after completion of the read
*             of all enabled tracks
*   \details  Unregistering of the callback can be performed by calling this
*             function function with a NULL parameter.
*   \param    func  application callback function
*/
void mcr_set_complete_callback(void (*func)(void));

/**
*   \brief    Retrieves the raw (undecoded) sample data for the specified track
*             of data
*   \param    track       track number (1 to 3)
*   \param    samples     pointer to where the sample data will be copied
*   \returns  number of samples retrieved
*/
unsigned int mcr_get_track_samples(unsigned int track, mcr_samples_t * samples);

#endif /* _MCR_H_ */

