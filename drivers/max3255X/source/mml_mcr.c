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
 * $Revision: 5258 $: Revision of last commit
 * $Author: lorne.smith $: Author of last commit
 * $Date: 2015-04-21 10:37:18 -0500 (Tue, 21 Apr 2015) $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */
/** Global includes */
#include <config.h>
#include <errors.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
/** Other includes */
#include <mml.h>
#include <cobra_defines.h>
#include <cobra_macros.h>
#include <cobra_functions.h>
#include <mml_dma_regs.h>
#include <mml_intc.h>
#include <mml_intc_regs.h>
/** Local includes */
#include <mml_mcr.h>
#include <mml_mcr_regs.h>

/***** Definitions *****/

#define MCR ((mml_mcr_regs_t*)MML_MCR_IOBASE)
#define DMA ((mml_dma_regs_t*)MML_DMA_IOBASE)

#define DMA_CH_CFG(tidx)  ( MML_DMA_CFG_CTZIEN_MASK |            \
                            (0x7 << MML_DMA_CFG_BRST_OFST) |     \
                            MML_DMA_CFG_DSTINC_MASK |            \
                            (0x02 << MML_DMA_CFG_DSTWD_OFST) |   \
                            (0x01 << MML_DMA_CFG_SRCWD_OFST) |   \
                            ((0x0B + tidx) << MML_DMA_CFG_REQSEL_OFST) )

#define ADC_DSP_FREQ      (4000000 / 3)

// Track parameters
typedef struct {
  uint8_t startsen; // start sentinel
  uint8_t endsen;   // end sentinel
  uint8_t nbits;    // # of bits per char
  uint8_t density;  // bit density used for swipe rate calculation (Note: Track 2 is 2x normal density)
  const uint8_t *charset; // character set
} trackpar_t;

/***** File Scope Data *****/

static uint16_t raw_data[MCR_NUM_TRACKS][MCR_MAX_SAMPLES];
static unsigned int raw_len[MCR_NUM_TRACKS];
static unsigned int raw_idx[MCR_NUM_TRACKS];
static void (*complete_callback)(void);  // user swipe complete callback

// Decode parameters
static uint8_t thr_md = 184;  // equal cost threshold x2^8
static uint8_t thr_up = 215;  // 8:1 cost threshold x2^8
static uint8_t thr_dn = 157;  // 1:8 cost threshold x2^8
static uint8_t narb = 5;      // # of steps in arbitration
static uint8_t pairing = 1;   // enable bit-pairing, force even # of ones (0=pairing disable)

// Track parameters
static const trackpar_t trackpars[] = {
  {
    0x45,   // start sentinel
    0x1F,   // end sentinel
    7,      // # of bits per char
    210,    // bit density used for swipe rate calculation
    //                00 000000000000001111111111111111222222222222222233333333333333334444444444444444555555555555555566666666666666667777777777777 777
    //                01 23456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABC DEF
    (const uint8_t *)"~!\"~$~~`(~~+~-.~0~~3~56~~9:~<~~?@~~C~EF~~IJ~L~~O~QR~T~~WX~~[~]^~ ~~#~%&~~)*~,~~/~12~4~~78~~;~=>~~AB~D~~GH~~K~MN~P~~S~UV~~YZ~\\~~_"
  },
  {
    0x0B,   // start sentinel
    0x1F,   // end sentinel
    5,      // # of bits per char
    150,    // bit density used for swipe rate calculation (Note: Track 2 is 2x normal density)
    //                00000000000000001111111111111111 
    //                0123456789ABCDEF0123456789ABCDEF 
    (const uint8_t *)"~12~4~~78~~;~=>~0~~3~56~~9:~<~~?"
  },
  {
    0x0B,   // start sentinel
    0x1F,   // end sentinel
    5,      // # of bits per char
    210,    // bit density used for swipe rate calculation (Note: Track 2 is 2x normal density)
    //                00000000000000001111111111111111 
    //                0123456789ABCDEF0123456789ABCDEF 
    (const uint8_t *)"~12~4~~78~~;~=>~0~~3~56~~9:~<~~?"
  }
};

/***** Function Prototypes *****/
void MCR_IRQHandler(void);
void MCR_DMA_IRQHandler(void);


#ifdef _STAND_ALONE_DRIVER_MCR_
/* This compilation flag is activated only in driver development context
* without any application using it.
* DO NOT define it in application/test context then. */
/*****************************************************************************/
int main(void)
{
	/** We're done */
	return NO_ERROR;
}
#endif /* _STAND_ALONE_DRIVER_MCR_ */

/*******************************************************************************
* Description: Attempts to remove false peaks from samples
* Expected Arguments:
*   Inputs:
*     samples[]   = sample data from DSP
*     len         = length of sample[]
*     thr         = threshold
*   Outputs:
*     samplesf[]  = sample data with false peaks removed
*     len         = length of samplesf[]
* Returns: void
*******************************************************************************/
static void remove_false_peaks(uint16_t * samples, unsigned int len, unsigned int thr, uint16_t * samplesf, unsigned int * lenf)
{
  // Input:
  //    samples[]   = samples data from DSP
  //    len         = length of samples[]
  //    thr         = filter threshold (thr/16)
  // Output:
  //    samplesf[]  = filtered samples data
  //    lenf        = length of samplesf[]

  unsigned int ii, iout;
  uint16_t smpl, peak, up, dn, ppt, threshold;
  unsigned int iup, idn;

  // init
  up = dn = ppt = 0;
  iup = idn = 0;

  iout = 0;
  for (ii = 0; ii < len; ii++) {
    smpl = (samples[ii] & 0x01FF);
    peak = (samples[ii] & 0xFE00) >> 5;     // scaled <<4

    // establish top and bottom levels
    if (!iup && (peak >= 1064)) {
      iup = 1;
      up = peak;
    }
    if (!idn && iup && (peak < 984)) {
      idn = 1;
      dn = peak;
    }

    if (iup && idn && (ii > 0) && (ii < (len - 1))) {
      // set threshold
      threshold = ((up - dn) * thr) >> 4;

      // check for false peak
      if ( abs( peak - ((samples[ii-1] & 0xFE00) >> 5) ) < threshold || 
           abs( peak - ((samples[ii+1] & 0xFE00) >> 5) ) < threshold
         )
      {
        ppt += smpl;
      }
      else
      {
        if (peak > ((up + dn) >> 1)) {
          // update up
          up = (up + peak) >> 1;
          samplesf[iout++] = (smpl + ppt);
          ppt = 0;
        } else {
          // update dn
          dn = (dn + peak) >> 1;
          samplesf[iout++] = (smpl + ppt);
          ppt = 0;
        }
      }
    }
  }

  *lenf = iout;
}

//******************************************************************************
static uint8_t get_char(uint8_t * bits, unsigned int nbits, unsigned int ii, int direction, uint8_t char_mask)
{
  uint8_t ch;
  uint16_t tmp;
  unsigned int ip;

  if (direction == MCR_FORWARD) {
    ip = (ii >> 3);
    tmp = ((uint16_t)bits[ip+1] << 8) + bits[ip];
    ch = (uint8_t)(tmp >> (ii & 0x7)) & char_mask;
  }
  else {
    ip = ((nbits-ii) >> 3);
    tmp = ((uint16_t)bits[ip] << 8) + bits[ip-1];
    tmp = tmp >> (((nbits-ii) & 0x7)+1);
    ch = (tmp & 0x80) ? 1 : 0;
    ch += (((tmp & 0x40) ? 1 : 0) << 1);
    ch += (((tmp & 0x20) ? 1 : 0) << 2);
    ch += (((tmp & 0x10) ? 1 : 0) << 3);
    ch += (((tmp & 0x08) ? 1 : 0) << 4);
    ch += (((tmp & 0x04) ? 1 : 0) << 5);
    ch += (((tmp & 0x02) ? 1 : 0) << 6);
    ch = ch & char_mask;
  }

  return ch;
}

/*******************************************************************************
* Description: Parses the raw track bits into characters using ISO/IEC standard 
*              7811
* Expected Arguments:
*   Inputs:
*     bits        = raw track bits
*     nbits       = number of bits in bits[]
*     trackpar    = track-specific parameters
*     direction   = parse direction; 0=forward, 1=backwards
*   Outputs:
*     parsed      = parsed characters array
* Returns: number of valid characters parsed, including sentinels and LRC
*******************************************************************************/
unsigned int parse_track(uint8_t * bits, unsigned int nbits, const trackpar_t * trackpar, unsigned int direction, mcr_decoded_track_t * parsed)
{
  unsigned int ii, ii1;
  unsigned int parity_errs, valid_cnt;
  uint8_t lrc, ch;
  uint8_t char_mask;
  int oneflag;

  // Initialize output
  memset(parsed, 0, sizeof(mcr_decoded_track_t));
  parsed->lrc = 0xFF; // invalid
  parsed->error_code = MCR_ERR_START_SEN | MCR_ERR_END_SEN;

  // check length
  if ((nbits == 0) || (nbits > MCR_MAX_RAW_LEN_BITS)) {
    parsed->error_code |= MCR_ERR_BAD_LEN;
    return 0;
  }

  // init
  ii = parity_errs = lrc = 0;
  oneflag = 0;
  char_mask = (1 << trackpar->nbits) - 1;
  valid_cnt = 0;

  // Search bit-wise for start sentinel
  while (1) {
    if ((ii + trackpar->nbits) >= nbits) {
      return valid_cnt;
    }

    ch = get_char(bits, nbits, ii, direction, char_mask);

    if (ch != 0) {
      if (!oneflag) {
        // remember the 1-st one
        oneflag = 1;
        ii1 = ii;
      }

      if (ch == trackpar->startsen) {
        parsed->error_code &= ~MCR_ERR_START_SEN; // clear the sentinel error
        valid_cnt++;
        lrc = (lrc ^ ch);
        ii += trackpar->nbits;  // skip past the sentinel
        break;
      }

      // allow no more than 2 charaters without start sentinel
      if ((ii - ii1) > ((unsigned int)trackpar->nbits << 1)) {
        return valid_cnt;
      }
    }

    ii++;
  }

  // Parse data and search for end sentinel
  while (parsed->len < (MCR_MAX_DEC_LEN - 1)) {

    if ((ii + (trackpar->nbits << 1)) >= nbits) {
      parsed->error_code = MCR_ERR_END_SEN;
      return valid_cnt;
    }

    ch = get_char(bits, nbits, ii, direction, char_mask);
    lrc = (lrc ^ ch);

    // check for end sentinel
    if (ch != trackpar->endsen) {
      // record the character, check parity & lrc
      if ((parsed->data[parsed->len++] = trackpar->charset[ch]) == '~')
        parity_errs++;
      else
        valid_cnt++;
    }
    else {
      parsed->error_code &= ~MCR_ERR_END_SEN; // clear the sentinel error
      valid_cnt++;

      ii += trackpar->nbits;  // skip past the sentinel
      ch = get_char(bits, nbits, ii, direction, char_mask);  // get LRC
      lrc = (lrc ^ ch) & (char_mask >> 1);

      // check parity & lrc
      if (trackpar->charset[ch] == '~')
        parity_errs++;
      else
        valid_cnt++;

      break;
    }

    ii += trackpar->nbits;
  }

  // output and exit
  if (parity_errs > 0)
    parsed->error_code |= MCR_ERR_PARITY;
  if (lrc != 0)
    parsed->error_code |= MCR_ERR_LRC;
  parsed->lrc = lrc;
  parsed->parity_errs = parity_errs;

  return valid_cnt;
}

/*******************************************************************************
* Description: Decodes the samples into the raw track bits
* Expected Arguments:
*   Inputs:
*     samples[]   = sample data from DSP
*     len         = length of sample[]
*     trackpar    = track-specific parameters
*     direction   = decode direction
*   Outputs:
*     bits        = raw track bits
*     nbits       = number of bits in bits[]
*     speed       = approximate swipe speed
* Returns: error code
*******************************************************************************/
uint8_t decode_track(uint16_t * samples, unsigned int len, const trackpar_t * trackpar, unsigned int direction, uint8_t * bits, unsigned int * nbits, unsigned int * speed)
{
  unsigned int ii, jj, kk;
  unsigned int oneflag;
  unsigned int onecnt, onecnt0, zcnt;
  unsigned int lpfc;
  uint16_t avg, avg0;
  uint16_t smpl, smpl0, an, da, corr;
  uint32_t cost, cost0_34, cost1_34, cost0_14;
  unsigned int bit;
  uint32_t avgsum;
  unsigned int avgsum_cnt;

  // check data length
  if ((len <= 5) || (len > MCR_MAX_SAMPLES))
    return MCR_ERR_BAD_LEN;

  // init
  memset(bits, 0, MCR_MAX_RAW_LEN_BYTES);
  *nbits = 0;
  avg = 0;
  avgsum = 0;
  avgsum_cnt = 0;
  oneflag = 0;
  onecnt = 0;
  zcnt = 0;

  // decode loop
  for (ii = 0; (ii < len) && !((zcnt > ((trackpar->nbits-1)*2)) && oneflag) && (*nbits < (MCR_MAX_RAW_LEN_BITS - 8)); ii++) {

    lpfc = 3;

    // new sample
    kk = (direction == MCR_REVERSE) ? len-1-ii : ii;
    smpl = (samples[kk] & 0x1FF) << 3;

    // average first 4 without decoding
    if (ii == 4) {
      // establish avg
      avg = avgsum >> 2;
    }

    if (ii > 3) {
      // normal pass

      // check for outliers
      if (smpl > ((avg * 384 + 128) >> 8))
        return MCR_ERR_OUTLIER;

      // make decision 0 (short) or 1 (long) ?

      if ( ((onecnt & 1) && pairing) || (smpl <= ((avg * thr_dn + 128) >> 8)) ) {
        // certain bit 1 detected
        bit = 1;
      }
      else if ( (smpl >= (avg * thr_up + 128) >> 8) ) {
        // certain bit 0 detected
        bit = 0;
      }
      else {
        // uncertain bit

        // start arbitration procedure

        // save initial state
        avg0 = avg;
        smpl0 = smpl;
        onecnt0 = onecnt;

        // try bit 0
        cost = ((uint32_t)(smpl - avg) * (uint32_t)(smpl - avg)) >> 1; 
        an = (smpl + avg) >> 1; // new avg target
        da = ((an - avg) * lpfc + 2) >> 2;
        avg += da;  // filter avg
        onecnt = 0;
        corr = da;

        // collect cost of forward ppts
        for (jj = 1; (jj <= narb) && (ii+jj < len); jj++)
        {
          kk = (direction == MCR_REVERSE) ? len-1-(ii+jj) : ii+jj;
          smpl = ((samples[kk] & 0x1FF) << 3) + corr;
          if ( ((onecnt & 1) && pairing) || (smpl <= ((avg * thr_md + 128) >> 8)) ) 
          {
            // set 1
            cost += ((uint32_t)(2*smpl - avg) * (uint32_t)(2*smpl - avg)) / 5;
            an = (2 * ((uint32_t)smpl + 2 * (uint32_t)avg)) / 5;  // new avg target
            da = ((an - avg) * lpfc + 4) >> 3;
            avg += da;  // filter avg
            onecnt++;
            corr = 2 * da;
          }
          else
          {
            // set 0
            cost += ((uint32_t)(smpl - avg) * (uint32_t)(smpl - avg)) >> 1; 
            an = (smpl + avg) >> 1; // new avg target
            da = ((an - avg) * lpfc + 2) >> 2;
            avg += da;  // filter avg
            onecnt = 0;
            corr = da;
          }
        }
        cost0_34 = cost;       // total cost of bit 0

        // restore initial state
        avg = avg0;
        smpl = smpl0;
        onecnt = onecnt0;

        // try bit 1
        cost = ((uint32_t)(2*smpl - avg) * (uint32_t)(2*smpl - avg)) / 5; 
        an = (2 * ((uint32_t)smpl + 2 * (uint32_t)avg)) / 5;  // new avg target
        da = ((an - avg) * lpfc + 4) >> 3;
        avg += da;  // filter avg
        onecnt++;
        corr = 2 * da;

        // collect cost of forward ppts
        for (jj = 1; (jj <= narb) && (ii+jj < len); jj++)
        {
          kk = (direction == MCR_REVERSE) ? len-1-(ii+jj) : ii+jj;
          smpl = ((samples[kk] & 0x1FF) << 3) + corr;
          if ( ((onecnt & 1) && pairing) || (smpl <= ((avg * thr_md + 128) >> 8)) ) 
          {
            // set 1
            cost += ((uint32_t)(2*smpl - avg) * (uint32_t)(2*smpl - avg)) / 5;
            an = (2 * ((uint32_t)smpl + 2 * (uint32_t)avg)) / 5;  // new avg target
            da = ((an - avg) * lpfc + 4) >> 3;
            avg += da;  // filter avg
            onecnt++;
            corr = 2 * da;
          }
          else
          {
            // set 0
            cost += ((uint32_t)(smpl - avg) * (uint32_t)(smpl - avg)) >> 1; 
            an = (smpl + avg) >> 1; // new avg target
            da = ((an - avg) * lpfc + 2) >> 2;
            avg += da;  // filter avg
            onecnt = 0;
            corr = da;
          }
        }
        cost1_34 = cost;       // total cost of bit 1

        // set slow lpf
        lpfc = 1;

        // restore initial state
        avg = avg0;
        smpl = smpl0;
        onecnt = onecnt0;

        // try bit 0
        cost = ((uint32_t)(smpl - avg) * (uint32_t)(smpl - avg)) >> 1; 
        an = (smpl + avg) >> 1; // new avg target
        da = ((an - avg) * lpfc + 2) >> 2;
        avg += da;  // filter avg
        onecnt = 0;
        corr = da;

        // collect cost of forward ppts
        for (jj = 1; (jj <= narb) && (ii+jj < len); jj++)
        {
          kk = (direction == MCR_REVERSE) ? len-1-(ii+jj) : ii+jj;
          smpl = ((samples[kk] & 0x1FF) << 3) + corr;
          if ( ((onecnt & 1) && pairing) || (smpl <= ((avg * thr_md + 128) >> 8)) ) 
          {
            // set 1
            cost += ((uint32_t)(2*smpl - avg) * (uint32_t)(2*smpl - avg)) / 5;
            an = 2 * ((uint32_t)smpl + 2 * (uint32_t)avg) / 5;  // new avg target
            da = ((an - avg) * lpfc + 4) >> 3;
            avg += da;  // filter avg
            onecnt++;
            corr = 2 * da;
          }
          else
          {
            // set 0
            cost += ((uint32_t)(smpl - avg) * (uint32_t)(smpl - avg)) >> 1; 
            an = (smpl + avg) >> 1; // new avg target
            da = ((an - avg) * lpfc + 2) >> 2;
            avg += da;  // filter avg
            onecnt = 0;
            corr = da;
          }
        }
        cost0_14 = cost;       // total cost of bit 0

        // restore initial state
        avg = avg0;
        smpl = smpl0;
        onecnt = onecnt0;

        // try bit 1
        cost = ((uint32_t)(2*smpl - avg) * (uint32_t)(2*smpl - avg)) / 5; 
        an = (2 * ((uint32_t)smpl + 2 * (uint32_t)avg) / 5);  // new avg target
        da = ((an - avg) * lpfc + 4) >> 3;
        avg += da;  // filter avg
        onecnt++;
        corr = 2 * da;

        // collect cost of forward ppts
        for (jj = 1; (jj <= narb) && (ii+jj < len); jj++)
        {
          kk = (direction == MCR_REVERSE) ? len-1-(ii+jj) : ii+jj;
          smpl = ((samples[kk] & 0x1FF) << 3) + corr;
          if ( ((onecnt & 1) && pairing) || (smpl <= ((avg * thr_md + 128) >> 8)) ) 
          {
            // set 1
            cost += ((uint32_t)(2*smpl - avg) * (uint32_t)(2*smpl - avg)) / 5;
            an = (2 * ((uint32_t)smpl + 2 * (uint32_t)avg)) / 5;  // new avg target
            da = ((an - avg) * lpfc + 4) >> 3;
            avg += da;  // filter avg
            onecnt++;
            corr = 2 * da;
          }
          else
          {
            // set 0
            cost += ((uint32_t)(smpl - avg) * (uint32_t)(smpl - avg)) >> 1; 
            an = (smpl + avg) >> 1; // new avg target
            da = ((an - avg) * lpfc + 2) >> 2;
            avg += da;  // filter avg
            onecnt = 0;
            corr = da;
          }
        }
        //cost1_14 = cost;       // total cost of bit 1

        // restore initial state
        avg = avg0;
        smpl = smpl0;
        onecnt = onecnt0;

        // select the try with smallest cost
        bit = 1;        // will be bit selection
        if (cost > cost0_14) {
          cost = cost0_14;
          bit = 0;
        }
        if (cost > cost1_34) {
          cost = cost1_34;
          bit = 1;
          lpfc = 3;
        }
        if (cost > cost0_34) {
          cost = cost0_34;
          bit = 0;
          lpfc = 3;
        }
        // end of arbitration
      }

      if (bit) {
        // set 1
        an = (2 * ((uint32_t)smpl + 2 * (uint32_t)avg)) / 5;  // new avg target
        avg = avg + (((an - avg) * lpfc + 4) >> 3);        // filter avg
        if (onecnt & 1) {
          bit = 1 << (*nbits & 0x7);
          bits[*nbits >> 3] |= bit;
          (*nbits)++;
        }
        onecnt++;
        zcnt = 0;
        oneflag = 1;
      }
      else {
        // set 0
        an = (smpl + avg) >> 1;                     // new avg target
        avg = avg + (((an - avg) * lpfc + 2) >> 2); // filter avg
        (*nbits)++;
        onecnt = 0;
        zcnt++;
      }
    }
    else
      avg = smpl;

    // sum avg
    avgsum += (uint32_t)avg;
    avgsum_cnt++;
  }

  *nbits += 8; // add trailing 0s

  // Calculate the approximate swipe speed
  *speed = ((ADC_DSP_FREQ * 10) >> 4) / (((avgsum / avgsum_cnt) * trackpar->density) >> 4);

  return MCR_ERR_OK;
}

//******************************************************************************
int mcr_init(void)
{
  MCR->ctrl = (MML_APB_FREQ / ADC_DSP_FREQ) << MML_MCR_CTRL_DVI_RATIO_POS;
  MCR->ctrl |= MML_MCR_CTRL_PKDETECT;

  // Configure the DSP settings
  MCR->addr = MML_MCR_IDX_T13_ZCT_FAST;
  MCR->data = 0x01F4;
  MCR->addr = MML_MCR_IDX_T13_ZCT_MID;
  MCR->data = 0x00FA;
  MCR->addr = MML_MCR_IDX_T13_ZCT_SLOW;
  MCR->data = 0x007D;
  MCR->addr = MML_MCR_IDX_T2_ZCT_FAST;
  MCR->data = 0x0190;
  MCR->addr = MML_MCR_IDX_T2_ZCT_MID;
  MCR->data = 0x00FA;
  MCR->addr = MML_MCR_IDX_T2_ZCT_SLOW;
  MCR->data = 0x0064;
  MCR->addr = MML_MCR_IDX_START_PCNT;
  MCR->data = 2;

  // Configure and power up the A/D converters
  MCR->addr = MML_MCR_IDX_ADCCFG1;
  MCR->data = MML_MCR_ADCCFG1_INT_REF | MML_MCR_ADCCFG1_PUADC | MML_MCR_ADCCFG1_VREF_SEL_0P5V | MML_MCR_ADCCFG1_RCR2_12KOHM;

  // Setup the interrupt handler
  mml_intc_setup_irq(MML_INTNUM_MCR, MML_INTC_PRIO_8, MCR_IRQHandler);

  return 1;
}

//******************************************************************************
void mcr_init_track(unsigned int track)
{
  unsigned int tidx = track - 1;

  // Check for a valid track number
  if ((track == 0) || (track > MCR_NUM_TRACKS))
    return;

  // Ensure DMA channel is disabled before initializing
  DMA->ch[tidx].cfg &= ~MML_DMA_CFG_CHEN_MASK;
  while (DMA->ch[tidx].stat & MML_DMA_ST_CHST_MASK);

  // Clear track state
  memset(raw_data[tidx], 0, sizeof(raw_data[tidx]));

  raw_len[tidx] = 0;
  raw_idx[tidx] = 0;

  // Configure/Enable the DMA channel
  DMA->ch[tidx].cfg = DMA_CH_CFG(tidx);
  DMA->ch[tidx].dest = (uint32_t)raw_data[tidx];
  DMA->ch[tidx].cnt = 8;  // bytes
  DMA->ch[tidx].cfg |= MML_DMA_CFG_CHEN_MASK;
  DMA->ctrl |= (1 << tidx); // enable DMA channel

  // Setup the interrupt handler
  mml_intc_setup_irq(MML_INTNUM_DMA(tidx), MML_INTC_PRIO_8, MCR_DMA_IRQHandler);
}

//******************************************************************************
void mcr_enable(void)
{
  MCR->ctrl |= MML_MCR_CTRL_EN;
}

//******************************************************************************
void mcr_disable(void)
{
  MCR->ctrl &= ~MML_MCR_CTRL_EN;
}

/****************************************************************************/
void mcr_set_complete_callback(void (*func)(void))
{
  complete_callback = func;
}

//******************************************************************************
void MCR_IRQHandler(void)
{
  unsigned int tidx;

  // Clear the interrupt flag
  MCR->ctrl &= ~MML_MCR_CTRL_DSP_INTF;

  // Disable to prevent corruption of captured data
  mcr_disable();

  for (tidx = 0; tidx < MCR_NUM_TRACKS; tidx++) {
    MCR->addr = MML_MCR_IDX_T1_DATA_CNT + tidx;
    raw_len[tidx] = MCR->data;
    DMA->ch[tidx].cfg &= ~MML_DMA_CFG_CHEN_MASK; // disable DMA
    while (DMA->ch[tidx].stat & MML_DMA_ST_CHST_MASK);
  }

  // Call the callback if it exists
  if (complete_callback != NULL) {
    (*complete_callback)();
  }
}

//******************************************************************************
void MCR_DMA_IRQHandler(void)
{
  unsigned int tidx;

  // Check each track's DMA channel for completion
  for (tidx = 0; tidx < MCR_NUM_TRACKS; tidx++) {
    if (DMA->ch[tidx].stat & MML_DMA_ST_CTZST_MASK) {
      DMA->ch[tidx].stat |= MML_DMA_ST_CTZST_MASK; /* clear interrupt flag */
      raw_idx[tidx] += 4;
      if ( (raw_idx[tidx] < MCR_MAX_SAMPLES) && !(MCR->ctrl & MML_MCR_CTRL_DSP_INTF) ) {
        /* Setup channel for next transfer */
        DMA->ch[tidx].cfg = DMA_CH_CFG(tidx);
        DMA->ch[tidx].dest = (unsigned int)&raw_data[tidx][raw_idx[tidx]];
        DMA->ch[tidx].cnt = 8;  /* bytes */
        DMA->ch[tidx].cfg |= MML_DMA_CFG_CHEN_MASK;
      }
    }
  }
}

//******************************************************************************
int mcr_task(void)
{
  int retval = 0;

  MCR_DMA_IRQHandler();

  if (MCR->ctrl & MML_MCR_CTRL_DSP_INTF) {
    MCR_IRQHandler();
    retval = 1;
  }

  return retval;
}

//******************************************************************************
unsigned int mcr_track_decode(unsigned int track, mcr_decoded_track_t * decoded_track)
{
  unsigned int tidx, i, j, bitdir, chardir;
  uint16_t samplesf[MCR_MAX_SAMPLES], *samples;
  unsigned int len;
  uint8_t bits[MCR_MAX_RAW_LEN_BYTES]; // decoded bit storage
  unsigned int nbits;     // # of decoded bits
  mcr_decoded_track_t best_decode;
  unsigned int best_cnt, cnt;
  unsigned int speed;

  // Check for a valid track number
  if ((track == 0) || (track > MCR_NUM_TRACKS)) {
    memset(decoded_track, 0, sizeof(mcr_decoded_track_t));
    decoded_track->lrc = 0xFF;
    decoded_track->error_code = MCR_ERR_PARAM;
    return 0;
  }

  // Track array index (0 to 2) is 1 less than the track number (1 to 3)
  tidx = track - 1;

  // Get the number of samples
  MCR->addr = MML_MCR_IDX_T1_DATA_CNT + tidx;
  len = MCR->data;
  raw_len[tidx] = len;

  // Check if any samples were acquired
  if (len == 0) {
    memset(decoded_track, 0, sizeof(mcr_decoded_track_t));
    decoded_track->lrc = 0xFF;
    decoded_track->error_code = MCR_ERR_BAD_LEN;
    return 0;
  }

  // Initialize track data
  memset(&best_decode, 0, sizeof(mcr_decoded_track_t));
  best_decode.lrc = 0xFF;  // invalid
  best_decode.error_code = MCR_ERR_START_SEN | MCR_ERR_END_SEN;
  best_cnt = 0;

  // Start with the raw samples
  samples = raw_data[tidx];

  for (i = 0; i < 2; i++) {

    // Increase arbitration steps in second pass
    if (i == 0) {
      narb = 5;
    } else {
      narb = 11;
    }

    for (j = 0; j < 3; j++) {

      // try false peaks removal in 2nd and 3rd pass
      if (j == 1) {
        remove_false_peaks(raw_data[tidx], raw_len[tidx], 4, samplesf, &len);
        samples = samplesf;
      } else if (j == 2) {
        remove_false_peaks(raw_data[tidx], raw_len[tidx], 6, samplesf, &len);
        samples = samplesf;
      }

      // Try decoding in both directions
      for (bitdir = MCR_FORWARD; bitdir <= MCR_REVERSE; bitdir++) {

        // Decode the samples into bits
        if (decode_track(samples, len, &trackpars[tidx], bitdir, bits, &nbits, &speed) == MCR_ERR_OK) {
          // at this point PPTs are decoded into bits in array
          // bits[], containing nbits bits

          // Try parsing in both directions
          for (chardir = MCR_FORWARD; chardir <= MCR_REVERSE; chardir++) {

            // Parse the bits into characters
            cnt = parse_track(bits, nbits, &trackpars[tidx], chardir, decoded_track);
            decoded_track->direction = bitdir ^ chardir;
            decoded_track->speed = speed;

            // If there were no errors and at least the sentinels and LRC are found, the decode was successful
            if ( (decoded_track->error_code == MCR_ERR_OK) && (cnt >= 3) )
              return decoded_track->len;

            // Select the decode with the most valid characters
            if (cnt > best_cnt) {
              memcpy(&best_decode, decoded_track, sizeof(mcr_decoded_track_t));
              best_cnt = cnt;
            }
          }
        }
      }
    }
  }

  // Decode and parsing failed, select the best
  memcpy(decoded_track, &best_decode, sizeof(mcr_decoded_track_t));
  return decoded_track->len;
}

/******************************************************************************/
unsigned int mcr_get_track_samples(unsigned int track, mcr_samples_t * samples)
{
  unsigned int tidx;

  memset(samples, 0, sizeof(mcr_samples_t));

  // Check for a valid track number
  if ((track == 0) || (track > MCR_NUM_TRACKS))
    return 0;

  // Track array index (0 to 2) is 1 less than the track number (1 to 3)
  tidx = track - 1;

  // Copy the raw sample information
  samples->len = raw_len[tidx];
  memcpy(samples->data, raw_data[tidx], raw_len[tidx] * sizeof(raw_data[0][0]));

  return samples->len;
}

/******************************************************************************/
/* EOF */
