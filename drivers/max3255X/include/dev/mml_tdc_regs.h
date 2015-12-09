/*
 * mml_tdc_regs.h --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2013, Maxim Integrated Products
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
 * THIS SOFTWARE IS PROVIDED BY MAXIM INTEGRATED PRODUCTS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL MAXIM INTEGRATED PRODUCTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* [INTERNAL] ------------------------------------------------------------------
 * Created on: Oct 22, 2013
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_TDC_REGS_H_
#define _MML_TDC_REGS_H_

/** @file mml_tdc_regs.h TDC Registers Header */

/** @defgroup MML_TDC TDC Driver
 * @ingroup MML_DRIVER
 */

/** @defgroup MML_TDC_REGS TDC Registers
 *
 * @note TDC IP or Specification version number
 *
 * @ingroup MML_TDC
 * @{
 */

/** @defgroup MML_TDC_REGS_CLKR  Clock Register (CLKR)
 *
 * @li CLKDIV [7:0] - R/W: clock divider -> pixel clock + 1
 * @li ACB [15:8] 	- R/W: AC bias frequency
 * @li DPOL [16] 	- R/W: video data enable polarity
 * @li VPOL [17] 	- R/W: vertical sync polarity
 * @li HPOL [18] 	- R/W: horizontal sync polarity
 * @li EDGE [19] 	- R/W: clock edge select
 * @li PASCLK [20] 	- R/W: clock mode : passive(for TFT)/active(for STN)
 * @li RFU1 [21] 	- R: reserved
 * @li VSCANDIR [22]- R/W: vertical scan direction
 * @li HSCANDIR [23]- R/W: horizontal scan direction
 * @li TESTEN[24]	- R/W: ????????
 * @li RFU2 [31:24] - R: reserved
 *
 * @{
 */

#define MML_TDC_CLKR_OFST						0x0 //!< Register offset
#define MML_TDC_CLKR_DFLT						0x0 //!< Register default value

/** Bits Fields */
#define MML_TDC_CLKR_CLKDIV_OFST				0 //!< CLKDIV offset
#define MML_TDC_CLKR_CLKDIV_MASK_NOOFST			0xff //!< CLKDIV mask
#define MML_TDC_CLKR_CLKDIV_MASK				( MML_TDC_CLKR_CLKDIV_MASK_NOOFST << MML_TDC_CLKR_CLKDIV_OFST ) //!< CLKDIV mask

#define MML_TDC_CLKR_ACB_OFST					8 //!< ACB offset
#define MML_TDC_CLKR_ACB_MASK_NOOFST			0xff //!< ACB mask
#define MML_TDC_CLKR_ACB_MASK					( MML_TDC_CLKR_ACB_MASK_NOOFST << MML_TDC_CLKR_ACB_OFST ) //!< ACB mask

#define MML_TDC_CLKR_DPOL_OFST					16 //!< DPOL offset
#define MML_TDC_CLKR_DPOL_MASK_NOOFST			0x1 //!< DPOL mask
#define MML_TDC_CLKR_DPOL_MASK					( MML_TDC_CLKR_DPOL_MASK_NOOFST << MML_TDC_CLKR_DPOL_OFST ) //!< DPOL mask

#define MML_TDC_CLKR_VPOL_OFST					17 //!< VPOL offset
#define MML_TDC_CLKR_VPOL_MASK_NOOFST			0x1 //!< VPOL mask
#define MML_TDC_CLKR_VPOL_MASK					( MML_TDC_CLKR_VPOL_MASK_NOOFST << MML_TDC_CLKR_VPOL_OFST ) //!< VPOL mask

#define MML_TDC_CLKR_HPOL_OFST					18 //!< HPOL offset
#define MML_TDC_CLKR_HPOL_MASK_NOOFST			0x1 //!< HPOL mask
#define MML_TDC_CLKR_HPOL_MASK					( MML_TDC_CLKR_HPOL_MASK_NOOFST << MML_TDC_CLKR_HPOL_OFST ) //!< HPOL mask

#define MML_TDC_CLKR_EDGE_OFST					19 //!< EDGE offset
#define MML_TDC_CLKR_EDGE_MASK_NOOFST			0x1 //!< EDGE mask
#define MML_TDC_CLKR_EDGE_MASK					( MML_TDC_CLKR_EDGE_MASK_NOOFST << MML_TDC_CLKR_EDGE_OFST ) //!< EDGE mask

#define MML_TDC_CLKR_PASCLK_OFST				20 //!< PASCLK offset
#define MML_TDC_CLKR_PASCLK_MASK_NOOFST			0x1 //!< PASCLK mask
#define MML_TDC_CLKR_PASCLK_MASK				( MML_TDC_CLKR_PASCLK_MASK_NOOFST << MML_TDC_CLKR_PASCLK_OFST ) //!< PASCLK mask

#define MML_TDC_CLKR_RFU1_OFST					21 //!< RFU offset
#define MML_TDC_CLKR_RFU1_MASK					0x200000 //!< RFU mask

#define MML_TDC_CLKR_VSCANDIR_OFST				22 //!< VSCANDIR offset
#define MML_TDC_CLKR_VSCANDIR_MASK_NOOFST		0x1 //!< VSCANDIR mask
#define MML_TDC_CLKR_VSCANDIR_MASK				( MML_TDC_CLKR_VSCANDIR_MASK_NOOFST << MML_TDC_CLKR_VSCANDIR_OFST ) //!< VSCANDIR mask

#define MML_TDC_CLKR_HSCANDIR_OFST				23 //!< HSCANDIR offset
#define MML_TDC_CLKR_HSCANDIR_MASK_NOOFST		0x1 //!< HSCANDIR mask
#define MML_TDC_CLKR_HSCANDIR_MASK				( MML_TDC_CLKR_HSCANDIR_MASK_NOOFST << MML_TDC_CLKR_HSCANDIR_OFST ) //!< HSCANDIR mask

#define MML_TDC_CLKR_TESTEN_OFST				24 //!< TESTEN offset
#define MML_TDC_CLKR_TESTEN_MASK_NOOFST			0x1 //!< TESTEN mask
#define MML_TDC_CLKR_TESTEN_MASK				( MML_TDC_CLKR_TESTEN_MASK_NOOFST << MML_TDC_CLKR_TESTEN_OFST ) //!< TESTEN mask

#define MML_TDC_CLKR_RFU2_OFST					25 //!< RFU offset
#define MML_TDC_CLKR_RFU2_MASK					0xfe000000 //!< RFU mask



/** Enumeration of the different value for a bits field  */
#define MML_TDC_CLKR_DPOL_LOW					0 //!< video data enable active low
#define MML_TDC_CLKR_DPOL_HIGH					1 //!< video data enable active high
#define MML_TDC_CLKR_VPOL_LOW					0 //!< VSYNC active low
#define MML_TDC_CLKR_VPOL_HIGH					1 //!< VSYNC active high
#define MML_TDC_CLKR_HPOL_LOW					0 //!< HSYNC active low
#define MML_TDC_CLKR_HPOL_HIGH					1 //!< HSYNC active high
#define MML_TDC_CLKR_EDGE_LOW					0 //!< data latched on the rising clock edge
#define MML_TDC_CLKR_EDGE_HIGH					1 //!< data latched on the falling clock edge
#define MML_TDC_CLKR_PASCLK_ACTIVE				0 //!< for TFT panel
#define MML_TDC_CLKR_PASCLK_PASSIVE				1 //!< for STN panel
#define MML_TDC_CLKR_VSCANDIR_NORMAL			0 //!< left to right scan
#define MML_TDC_CLKR_VSCANDIR_INVERT			1 //!< right to left scan
#define MML_TDC_CLKR_HSCANDIR_NORMAL			0 //!< up to down scan
#define MML_TDC_CLKR_HSCANDIR_INVERT			1 //!< down to up scan
/** @} */ /* @defgroup MML_TDC_REGS_CLKR */


/** @defgroup MML_TDC_REGS_VTIMR0 Vertical Timing Register 0 (VTIMR0)
 *
 * @li VLINES [11:0]		- R/W: vertical lines number
 * @li RFU1 [15:12]			- R: reserved
 * @li VBACKPORCH [23:16]	- R/W: vertical back porch lines number
 * @li RFU2 [31:24]			- R: reserved
 *
 * @{
 */

#define MML_TDC_VTIMR0_OFST						0x4 //!< Register offset
#define MML_TDC_VTIMR0_DFLT						0x0 //!< Register default value

/** Bits Fields */
#define MML_TDC_VTIMR0_VLINES_OFST				0 //!< lines number offset
#define MML_TDC_VTIMR0_VLINES_MASK_NOOFST		0xfff //!< lines number mask
#define MML_TDC_VTIMR0_VLINES_MASK				( MML_TDC_VTIMR0_VLINES_MASK_NOOFST << MML_TDC_VTIMR0_VLINES_OFST ) //!< lines number mask

#define MML_TDC_VTIMR0_RFU1_OFST				12 //!< reserved 1 offset
#define MML_TDC_VTIMR0_RFU1_MASK				0xf000 //!< reserved 1 mask

#define MML_TDC_VTIMR0_VBACKPORCH_OFST			16 //!< vertical back porch lines number offset
#define MML_TDC_VTIMR0_VBACKPORCH_MASK_NOOFST	0xff //!< vertical back porch lines number mask
#define MML_TDC_VTIMR0_VBACKPORCH_MASK			( MML_TDC_VTIMR0_VBACKPORCH_MASK_NOOFST << MML_TDC_VTIMR0_VBACKPORCH_OFST ) //!< vertical back porch lines number mask

#define MML_TDC_VTIMR0_RFU2_OFST				24 //!< reserved 2 offset
#define MML_TDC_VTIMR0_RFU2_MASK				0xff000000 //!< reserved 2 mask

/** @} */ /* @defgroup MML_TDC_REGS_VTIMR0 */


/** @defgroup MML_TDC_REGS_VTIMR1 Vertical Timing Register 1 (VTIMR1)
 *
 * @li VSYNCWIDTH [7:0]		- R/W: VSYNC pulse lines number
 * @li RFU1 [15:8]			- R: reserved
 * @li VFRONTPORCH [23:16]	- R/W: vertical front porch lines number
 * @li RFU2 [31:24]			- R: reserved
 *
 * @{
 */

#define MML_TDC_VTIMR1_OFST						0x8 //!< Register offset
#define MML_TDC_VTIMR1_DFLT						0x0 //!< Register default value

/* Bits Fields */
#define MML_TDC_VTIMR1_VSYNCWIDTH_OFST			0 //!< VSYNC pulse lines number offset
#define MML_TDC_VTIMR1_VSYNCWIDTH_MASK_NOOFST	0xff //!< VSYNC pulse lines number mask
#define MML_TDC_VTIMR1_VSYNCWIDTH_MASK			(  <<  ) //!< VSYNC pulse lines number mask

#define MML_TDC_VTIMR1_RFU1_OFST				8 //!< reserved 1 offset
#define MML_TDC_VTIMR1_RFU1_MASK				0xff00 //!< reserved 1 mask

#define MML_TDC_VTIMR1_VFRONTPORCH_OFST			16 //!< vertical front porch lines number offset
#define MML_TDC_VTIMR1_VFRONTPORCH_MASK_NOOFST	0xff //!< vertical front porch lines number mask
#define MML_TDC_VTIMR1_VFRONTPORCH_MASK			( MML_TDC_VTIMR1_VFRONTPORCH_MASK_NOOFST << MML_TDC_VTIMR1_VFRONTPORCH_OFST ) //!< vertical front porch lines number mask

#define MML_TDC_VTIMR1_RFU2_OFST				24 //!< reserved 2 offset
#define MML_TDC_VTIMR1_RFU2_MASK				0xff000000 //!< reserved 2 mask

/** @} */ /* @defgroup MML_TDC_REGS_VTIMR1 */


/** @defgroup MML_TDC_REGS_HTIMR Horizontal Timing Register (HTIMR)
 *
 * @li HSYNCWIDTH [7:0]		- R/W: HSYNC width in clock cycles
 * @li HFRONTPORCH [15:8]	- R/W: horizontal front porch clocks number
 * @li HSIZE [23:16]		- R/W: the number of pixels per line
 * @li HBACKPORCH [31:24]	- R/W: the horizontal back porch width in clocks cycles
 *
 * @{
 */

#define MML_TDC_HTIMR_OFST						0xc	//!< Register offset
#define MML_TDC_HTIMR_DFLT						0x0	//!< Register default value

/* Bits Fields */
#define MML_TDC_HTIMR_HSYNCWIDTH_OFST			0 //!< HSYNC pulse clock number offset
#define MML_TDC_HTIMR_HSYNCWIDTH_MASK_NOOFST	0xff //!< HSYNC pulse clock number mask
#define MML_TDC_HTIMR_HSYNCWIDTH_MASK			( MML_TDC_HTIMR_HSYNCWIDTH_MASK_NOOFST << MML_TDC_HTIMR_HSYNCWIDTH_OFST ) //!< HSYNC pulse clock number mask

#define MML_TDC_HTIMR_HFRONTPORCH_OFST			8 //!< horizontal front porch lines number offset
#define MML_TDC_HTIMR_HFRONTPORCH_MASK_NOOFST	0xff //!< horizontal front porch lines number mask
#define MML_TDC_HTIMR_HFRONTPORCH_MASK			( MML_TDC_HTIMR_HFRONTPORCH_MASK_NOOFST << MML_TDC_HTIMR_HFRONTPORCH_OFST ) //!< horizontal front porch lines number mask

#define MML_TDC_HTIMR_HSIZE_OFST				16 //!< number of pixel per line offset
#define MML_TDC_HTIMR_HSIZE_MASK_NOOFST			0xff //!< number of pixel per line mask
#define MML_TDC_HTIMR_HSIZE_MASK				( MML_TDC_HTIMR_HSIZE_MASK_NOOFST << MML_TDC_HTIMR_HSIZE_OFST ) //!< number of pixel per line mask

#define MML_TDC_HTIMR_HBACKPORCH_OFST			24 //!< horizontal back porch lines number offset
#define MML_TDC_HTIMR_HBACKPORCH_MASK_NOOFST	0xff //!< horizontal back porch lines number mask
#define MML_TDC_HTIMR_HBACKPORCH_MASK			( MML_TDC_HTIMR_HBACKPORCH_MASK_NOOFST << MML_TDC_HTIMR_HBACKPORCH_OFST ) //!< horizontal back porch lines number mask

/** @} */ /* @defgroup MML_TDC_REGS_HTIMR */


/** @defgroup MML_TDC_REGS_CR Control Register (CR)
 *
 * @li LCDEN [0]		- R/W: LCD enable
 * @li VISEL [2:1]		- R/W: vertical compare interrupt select
 * @li RFU1 [3]			- R: reserved
 * @li DISTYPE [7:4]	- R/W: display type
 * @li BPP [10:8]		- R/W: bit per pixel
 * @li RFU2 [11]		- R: reserved
 * @li EMODE [13:12]	- R/W: endianess
 * @li RFU3 [14]		- R: reserved
 * @li C24 [15]			- R/W: compact storage mode
 * @li RFU4 [18:16]		- R: reserved
 * @li BURST [20:19]	- R/W: FIFO burst size
 * @li LPOL [21]		- R/W: LEND polarity
 * @li PEN [22]			- R/W: LCD panel power enable
 * @li RFU5 [31:23]		- R: reserved
 *
 * @{
 */

#define MML_TDC_CR_OFST							0x10 //!< Register offset
#define MML_TDC_CR_DFLT							0x0 //!< Register default value

/* Bits Fields */
#define MML_TDC_CR_LCDEN_OFST					0 //!< LCD enable offset
#define MML_TDC_CR_LCDEN_MASK_NOOFST			0x1 //!< LCD enable mask
#define MML_TDC_CR_LCDEN_MASK					( MML_TDC_CR_LCDEN_MASK_NOOFST << MML_TDC_CR_LCDEN_OFST ) //!< LCD enable mask

#define MML_TDC_CR_VISEL_OFST					1 //!< vertical compare interrupt select offset
#define MML_TDC_CR_VISEL_MASK_NOOFST			0x3 //!< vertical compare interrupt select mask
#define MML_TDC_CR_VISEL_MASK					( MML_TDC_CR_VISEL_MASK_NOOFST << MML_TDC_CR_VISEL_OFST ) //!< vertical compare interrupt select mask

#define MML_TDC_CR_RFU1_OFST					3 //!< reserved 1 offset
#define MML_TDC_CR_RFU1_MASK					0x8 //!< reserved 1 mask

#define MML_TDC_CR_DISTYPE_OFST					4 //!< display type offset
#define MML_TDC_CR_DISTYPE_MASK_NOOFST			0xf //!< display type mask
#define MML_TDC_CR_DISTYPE_MASK					( MML_TDC_CR_DISTYPE_MASK_NOOFST << MML_TDC_CR_DISTYPE_OFST ) //!< display type mask

#define MML_TDC_CR_BPP_OFST						8 //!< bit per pixel offset
#define MML_TDC_CR_BPP_MASK_NOOFST				0x7 //!< bit per pixel mask
#define MML_TDC_CR_BPP_MASK						( MML_TDC_CR_BPP_MASK_NOOFST << MML_TDC_CR_BPP_OFST ) //!< bit per pixel mask

#define MML_TDC_CR_RFU2_OFST					11 //!< reserved 2 offset
#define MML_TDC_CR_RFU2_MASK					0x800 //!< reserved 2 mask

#define MML_TDC_CR_EMODE_OFST					12 //!< endianess offset
#define MML_TDC_CR_EMODE_MASK_NOOFST			0x3 //!< endianess mask
#define MML_TDC_CR_EMODE_MASK					( MML_TDC_CR_EMODE_MASK_NOOFST << MML_TDC_CR_EMODE_OFST ) //!< endianess mask

#define MML_TDC_CR_RFU3_OFST					14 //!< reserved 3 offset
#define MML_TDC_CR_RFU3_MASK					0x4000 //!< reserved 3 mask

#define MML_TDC_CR_C24_OFST						15 //!< compact storage mode offset
#define MML_TDC_CR_C24_MASK_NOOFST				0x1 //!< compact storage mode mask
#define MML_TDC_CR_C24_MASK						( MML_TDC_CR_C24_MASK_NOOFST << MML_TDC_CR_C24_OFST ) //!< compact storage mode mask

#define MML_TDC_CR_RFU4_OFST					16 //!< reserved 4 offset
#define MML_TDC_CR_RFU4_MASK					0x70000 //!< reserved 4 mask

#define MML_TDC_CR_BURST_OFST					19 //!< FIFO burst size offset
#define MML_TDC_CR_BURST_MASK_NOOFST			0x3 //!< FIFO burst size mask
#define MML_TDC_CR_BURST_MASK					( MML_TDC_CR_BURST_MASK_NOOFST << MML_TDC_CR_BURST_OFST ) //!< FIFO burst size mask

#define MML_TDC_CR_LPOL_OFST					21 //!< LEND polarity offset
#define MML_TDC_CR_LPOL_MASK_NOOFST				0x1 //!< LEND polarity mask
#define MML_TDC_CR_LPOL_MASK					( MML_TDC_CR_LPOL_MASK_NOOFST << MML_TDC_CR_LPOL_OFST ) //!< LEND polarity mask

#define MML_TDC_CR_PEN_OFST						22 //!< LCD panel power enable offset
#define MML_TDC_CR_PEN_MASK_NOOFST				0x1 //!< LCD panel power enable mask
#define MML_TDC_CR_PEN_MASK						( MML_TDC_CR_PEN_MASK_NOOFST << MML_TDC_CR_PEN_OFST ) //!< LCD panel power enable mask

#define MML_TDC_CR_RFU5_OFST					23 //!< reserved 5 offset
#define MML_TDC_CR_RFU5_MASK					0xff800000 //!< reserved 5 mask





/** Enumeration of the different value for a bits field  */
#define MML_TDC_CR_LCDEN_DISABLE				0 //!< LCD is disable
#define MML_TDC_CR_LCDEN_ENABLE					1 //!< LCD is enable

#define MML_TDC_CR_VISEL_VSYNC					0 //!< VCI start on VSYNC
#define MML_TDC_CR_VISEL_VBACKPORCH				1 //!< VCI start on VSYNC
#define MML_TDC_CR_VISEL_VIDEO_ACTIVE			2 //!< VCI start on VSYNC
#define MML_TDC_CR_VISEL_VFRONTPORCH			3 //!< VCI start on VSYNC

#define MML_TDC_CR_DISTYPE_STN_MONO_4_BIT		0 //!< STN monochrome 4-bit mode
#define MML_TDC_CR_DISTYPE_STN_MONO_4_BIT_DUAL	1 //!< STN monochrome 4-bit dual mode
#define MML_TDC_CR_DISTYPE_STN_MONO_8_BIT		2 //!< STN monochrome 8-bit mode
#define MML_TDC_CR_DISTYPE_STN_MONO_8_BIT_DUAL	3 //!< STN monochrome 8-bit dual mode
#define MML_TDC_CR_DISTYPE_STN_COLOR_8_BIT		4 //!< STN color 8-bit mode
#define MML_TDC_CR_DISTYPE_STN_COLOR_8_BIT_DUAL	5 //!< STN color 8-bit dual mode
#define MML_TDC_CR_DISTYPE_TFT					8 //!< TFT mode
#define MML_TDC_CR_DISTYPE_HRTFT				9 //!< HRTFT mode

#define MML_TDC_CR_BPP_1						0 //!< 1 bit per pixel
#define MML_TDC_CR_BPP_2						1 //!< 2 bit per pixel
#define MML_TDC_CR_BPP_4						2 //!< 4 bit per pixel
#define MML_TDC_CR_BPP_8						3 //!< 8 bit per pixel
#define MML_TDC_CR_BPP_16						4 //!< 16 bit per pixel
#define MML_TDC_CR_BPP_24						5 //!< 24 bit per pixel

#define MML_TDC_CR_EMODE_LBLP					0 //!< Little Endian Byte Little Endian Pixel mode
#define MML_TDC_CR_EMODE_BBBP					1 //!< Big Endian Byte Big Endian Pixel mode
#define MML_TDC_CR_EMODE_LBBP					2 //!< Little Endian Byte, Big Endian Pixel mode (WinCE format)

#define MML_TDC_CR_C24_DISABLE					0 //!< 1 pixel per word
#define MML_TDC_CR_C24_ENABLE					1 //!< 1 1/3 pixel per word

#define MML_TDC_CR_BURST_4						0 //!< FIFO burst size is 4 words
#define MML_TDC_CR_BURST_8						1 //!< FIFO burst size is 8 words
#define MML_TDC_CR_BURST_16_1					2 //!< FIFO burst size is 16 words
#define MML_TDC_CR_BURST_16_2					3 //!< FIFO burst size is 16 words

#define MML_TDC_CR_LPOL_LOW						0 //!< LEND is active low
#define MML_TDC_CR_LPOL_HIGH					1 //!< LEND is active high

#define MML_TDC_CR_PEN_DISABLE					0 //!< PWREN pin set to 0
#define MML_TDC_CR_PEN_ENABLE					1 //!< PWREN pin set to 1

/** @} */ /* @defgroup MML_TDC_REGS_CR */


/** @defgroup MML_TDC_REGS_FRMBR0 Frame Buffer Register 0 (FRMBR0)
 *
 * @li ADDRESS [31:0]		- R/W: starting address of the frame buffer 0
 *
 * @{
 */

#define MML_TDC_FRMBR0_OFST						0x18 //!< Register offset
#define MML_TDC_FRMBR0_DFLT						0x0 //!< Register default value

/** @} */ /* @defgroup MML_TDC_REGS_FRMBR0 */


/** @defgroup MML_TDC_REGS_FRMBR1 Frame Buffer Register 1 (FRMBR1)
 *
 * @li ADDRESS [31:0]		- R/W: starting address of the frame buffer 1
 *
 * @{
 */

#define MML_TDC_FRMBR1_OFST						0x1c //!< Register offset
#define MML_TDC_FRMBR1_DFLT						0x0 //!< Register default value

/** @} */ /* @defgroup MML_TDC_REGS_FRMBR1 */


/** @defgroup MML_TDC_REGS_IER Interrupt Enable Register (IER)
 *
 * @li UFLO [0]		- R/W: FIFO underflow interrupt enable
 * @li ADDRDY [1]	- R/W: address ready interrupt enable
 * @li VCI [2]		- R/W: vertical control interrupt enable
 * @li BERR [3]		- R/W: bus error interrupt enable
 * @li RFU [31:4]	- R: reserved
 *
 * @{
 */

#define MML_TDC_IER_OFST						0x20 //!< Register offset
#define MML_TDC_IER_DFLT						0x0 //!< Register default value

/* Bits Fields */
#define MML_TDC_IER_UFLO_OFST					0 //!< FIFO underflow interrupt enable offset
#define MML_TDC_IER_UFLO_MASK_NOOFST	 		0x1 //!< FIFO underflow interrupt enable mask
#define MML_TDC_IER_UFLO_MASK	 				( MML_TDC_IER_UFLO_MASK_NOOFST << MML_TDC_IER_UFLO_OFST ) //!< FIFO underflow interrupt enable mask

#define MML_TDC_IER_ADDRDY_OFST					1	//!< address ready interrupt enable offset
#define MML_TDC_IER_ADDRDY_MASK_NOOFST			0x1	//!< address ready interrupt enable mask
#define MML_TDC_IER_ADDRDY_MASK					( MML_TDC_IER_ADDRDY_MASK_NOOFST << MML_TDC_IER_ADDRDY_OFST ) //!< address ready interrupt enable mask

#define MML_TDC_IER_VCI_OFST					2 //!< vertical control interrupt enable offset
#define MML_TDC_IER_VCI_MASK_NOOFST				0x1	//!< vertical control interrupt enable mask
#define MML_TDC_IER_VCI_MASK					( MML_TDC_IER_VCI_MASK_NOOFST << MML_TDC_IER_VCI_OFST ) //!< vertical control interrupt enable mask

#define MML_TDC_IER_BERR_OFST					3 //!< bus error interrupt enable offset
#define MML_TDC_IER_BERR_MASK_NOOFST			0x1	//!< bus error interrupt enable mask
#define MML_TDC_IER_BERR_MASK					( MML_TDC_IER_BERR_MASK_NOOFST << MML_TDC_IER_BERR_OFST ) //!< bus error interrupt enable mask

#define MML_TDC_IER_RFU_OFST					4 //!< reserved offset
#define MML_TDC_IER_RFU_MASK					0xfffffff0 //!< reserved mask

#define MML_TDC_IER_UFLO_DISABLE				0 //!< FIFO underflow interrupt disable
#define MML_TDC_IER_UFLO_ENABLE					1 //!< FIFO underflow interrupt enable

#define MML_TDC_IER_ADDRDY_DISABLE				0 //!< address ready interrupt disable
#define MML_TDC_IER_ADDRDY_ENABLE				1 //!< address ready interrupt enable

#define MML_TDC_IER_VCI_DISABLE					0 //!< vertical control interrupt disable
#define MML_TDC_IER_VCI_ENABLE					1 //!< vertical control interrupt enable

#define MML_TDC_IER_BERR_DISABLE				0 //!< bus error interrupt disable
#define MML_TDC_IER_BERR_ENABLE					1 //!< bus error interrupt enable

/** @} */ /* @defgroup MML_TDC_REGS_IER */


/** @defgroup MML_TDC_REGS_SR Status Register (SR)
 *
 * @li UFLO [0]		- R: FIFO underflow interrupt status
 * @li ADDRDY [1]	- R: address ready interrupt status
 * @li VCI [2]		- R: vertical control interrupt status
 * @li BERR [3]		- R: bus error interrupt status
 * @li RFU1 [7:4]	- R: reserved
 * @li LCD_IDLE [8]	- R: LCD status : 0-> running, 1-> idle
 * @li RFU2 [31:9]	- R: reserved
 *
 * @{
 */

#define MML_TDC_SR_OFST							0x24 //!< Register offset
#define MML_TDC_SR_DFLT							0x0 //!< Register default value

/* Bits Fields */
#define MML_TDC_SR_UFLO_OFST					0 //!< FIFO underflow interrupt status offset
#define MML_TDC_SR_UFLO_MASK_NOOFST				0x1 //!< FIFO underflow interrupt enable mask
#define MML_TDC_SR_UFLO_MASK					( MML_TDC_SR_UFLO_MASK_NOOFST << MML_TDC_SR_UFLO_OFST ) //!< FIFO underflow interrupt enable mask

#define MML_TDC_SR_ADDRDY_OFST					1 //!< address ready interrupt status offset
#define MML_TDC_SR_ADDRDY_MASK_NOOFST			0x1 //!< address ready interrupt enable mask
#define MML_TDC_SR_ADDRDY_MASK					( MML_TDC_SR_ADDRDY_MASK_NOOFST << MML_TDC_SR_ADDRDY_OFST ) //!< address ready interrupt enable mask

#define MML_TDC_SR_VCI_OFST						2 //!< vertical control interrupt status offset
#define MML_TDC_SR_VCI_MASK_NOOFST				0x1 //!< vertical control interrupt enable mask
#define MML_TDC_SR_VCI_MASK						( MML_TDC_SR_VCI_MASK_NOOFST << MML_TDC_SR_VCI_OFST ) //!< vertical control interrupt enable mask

#define MML_TDC_SR_BERR_OFST					3 //!< bus error interrupt status offset
#define MML_TDC_SR_BERR_MASK_NOOFST				0x1 //!< bus error interrupt enable mask
#define MML_TDC_SR_BERR_MASK					( MML_TDC_SR_BERR_MASK_NOOFST << MML_TDC_SR_BERR_OFST ) //!< bus error interrupt enable mask

#define MML_TDC_SR_RFU1_OFST					4 //!< reserved 1 offset
#define MML_TDC_SR_RFU1_MASK					0xf0 //!< reserved 1 mask

#define MML_TDC_SR_LCD_IDLE_OFST				8 //!< LCD status offset
#define MML_TDC_SR_LCD_IDLE_MASK_NOOFST			0x1 //!< LCD status mask
#define MML_TDC_SR_LCD_IDLE_MASK				( MML_TDC_SR_LCD_IDLE_MASK_NOOFST << MML_TDC_SR_LCD_IDLE_OFST ) //!< LCD status mask

#define MML_TDC_SR_RFU2_OFST					9 //!< reserved 2 offset
#define MML_TDC_SR_RFU2_MASK					0xfffffe00 //!< reserved 2 mask

/** @} */ /* @defgroup MML_TDC_REGS_SR */


/** @defgroup MML_TDC_REGS_ADVTIMR Advanced Timing Register (ADVTIMR)
 *
 * @li CLS_WIDTH [8:0]		- R/W: CLS width
 * @li RFU1 [15:9]			- R: reserved
 * @li REV_DLY [24:16]		- R/W: CLS falling edge to REV toggling
 * @li RFU2 [25]			- R: reserved
 * @li PS_HOLD_TIME [31:26]	- R/W: CLS falling edge to PS rising edge
 *
 * @{
 */

#define MML_TDC_ADVTIMR_OFST					0x28 //!< Register offset
#define MML_TDC_ADVTIMR_DFLT					0x0//!< Register default value

/* Bits Fields */
#define MML_TDC_ADVTIMR_CLS_WIDTH_OFST			0 //!< CLS_WIDTH offset
#define MML_TDC_ADVTIMR_CLS_WIDTH_MASK_NOOFST	0x1ff //!< CLS_WIDTH mask
#define MML_TDC_ADVTIMR_CLS_WIDTH_MASK			( MML_TDC_ADVTIMR_CLS_WIDTH_MASK_NOOFST << MML_TDC_ADVTIMR_CLS_WIDTH_OFST ) //!< CLS_WIDTH mask

#define MML_TDC_ADVTIMR_RFU1_OFST				9 //!< reserved 1 offset
#define MML_TDC_ADVTIMR_RFU1_MASK				0xfe00 //!< reserved 1 mask

#define MML_TDC_ADVTIMR_REV_DLY_OFST			16 //!< REV_DLY offset
#define MML_TDC_ADVTIMR_REV_DLY_MASK_NOOFST	0x1ff //!< REV_DLY mask
#define MML_TDC_ADVTIMR_REV_DLY_MASK			( MML_TDC_ADVTIMR_REV_DLY_MASK_NOOFST << MML_TDC_ADVTIMR_REV_DLY_OFST ) //!< REV_DLY mask

#define MML_TDC_ADVTIMR_RFU2_OFST				25 //!< reserved 2 offset
#define MML_TDC_ADVTIMR_RFU2_MASK				0x2000000 //!< reserved 2 mask

#define MML_TDC_ADVTIMR_PS_HOLD_TIME_OFST		26 //!< PS_HOLD_TIME offset
#define MML_TDC_ADVTIMR_PS_HOLD_TIME_MASK_NOOFST	0x3f //!< PS_HOLD_TIME mask
#define MML_TDC_ADVTIMR_PS_HOLD_TIME_MASK		( MML_TDC_ADVTIMR_PS_HOLD_TIME_MASK_NOOFST << MML_TDC_ADVTIMR_PS_HOLD_TIME_OFST ) //!< PS_HOLD_TIME mask

/** @} */ /* @defgroup MML_TDC_REGS_ADVTIMR */


/** @defgroup MML_TDC_REGS_PALR Palette Register (PALR)
 *
 * @li INTS [0]			- R/W: intensity
 * @li RED [5:1]		- R/W: red data
 * @li GREEN [10:6]		- R/W: green data
 * @li BLUE [15:11]		- R/W: blue data
 * @li INTS [16]		- R/W: intensity
 * @li RED [21:17]		- R/W: red data
 * @li GREEN [26:22]	- R/W: green data
 * @li BLUE [27]		- R/W: blue data
 *
 * @{
 */

#define MML_TDC_PALR_OFST						0x30 //!< 1st Register offset
#define MML_TDC_PALR_DEPTH						0x100 //!< depth of the palette RAM
#define MML_TDC_PALR_DFLT						0x0 //!< Register default value

#define MML_TDC_PALR_INTS0_OFST					0 //!< INTS0 offset
#define MML_TDC_PALR_INTS0_MASK_NOOFST			0x1 //!< INTS0 mask
#define MML_TDC_PALR_INTS0_MASK					( MML_TDC_PALR_INTS0_MASK_NOOFST << MML_TDC_PALR_INTS0_OFST ) //!< INTS0 mask

#define MML_TDC_PALR_RED0_OFST					1 //!< RED0 offset
#define MML_TDC_PALR_RED0_MASK_NOOFST			0x1f //!< RED0 mask
#define MML_TDC_PALR_RED0_MASK					( MML_TDC_PALR_RED0_MASK_NOOFST << MML_TDC_PALR_RED0_MASK_NOOFST ) //!< RED0 mask

#define MML_TDC_PALR_GREEN0_OFST				6 //!< GREEN0 offset
#define MML_TDC_PALR_GREEN0_MASK_NOOFST			0x1f //!< GREEN0 mask
#define MML_TDC_PALR_GREEN0_MASK				( MML_TDC_PALR_GREEN0_MASK_NOOFST << MML_TDC_PALR_GREEN0_OFST ) //!< GREEN0 mask

#define MML_TDC_PALR_BLUE0_OFST					11 //!< BLUE0 offset
#define MML_TDC_PALR_BLUE0_MASK_NOOFST			0x1f //!< BLUE0 mask
#define MML_TDC_PALR_BLUE0_MASK					( MML_TDC_PALR_BLUE0_MASK_NOOFST << MML_TDC_PALR_BLUE0_OFST ) //!< BLUE0 mask

#define MML_TDC_PALR_INTS1_OFST					16 //!< INTS1 offset
#define MML_TDC_PALR_INTS1_MASK_NOOFST			0x1 //!< INTS1 mask
#define MML_TDC_PALR_INTS1_MASK					( MML_TDC_PALR_INTS1_MASK_NOOFST << MML_TDC_PALR_INTS1_OFST ) //!< INTS1 mask

#define MML_TDC_PALR_RED1_OFST					17 //!< RED1 offset
#define MML_TDC_PALR_RED1_MASK_NOOFST			0x1f //!< RED1 mask
#define MML_TDC_PALR_RED1_MASK					( MML_TDC_PALR_RED1_MASK_NOOFST << MML_TDC_PALR_RED1_OFST ) //!< RED1 mask

#define MML_TDC_PALR_GREEN1_OFST				22 //!< GREEN1 offset
#define MML_TDC_PALR_GREEN1_MASK_NOOFST			0x1f //!< GREEN1 mask
#define MML_TDC_PALR_GREEN1_MASK				( MML_TDC_PALR_GREEN1_MASK_NOOFST << MML_TDC_PALR_GREEN1_OFST ) //!< GREEN1 mask

#define MML_TDC_PALR_BLUE1_OFST					27 //!< BLUE1 offset
#define MML_TDC_PALR_BLUE1_MASK_NOOFST			0x1f //!< BLUE1 mask
#define MML_TDC_PALR_BLUE1_MASK					( MML_TDC_PALR_BLUE1_MASK_NOOFST << MML_TDC_PALR_BLUE1_OFST ) //!< BLUE1 mask
/** @} */ /* @defgroup MML_TDC_REGS_PALR */

#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */
/** TDC Registers.
 *
 */
typedef volatile struct
{
	volatile unsigned int						clock; //< clock register
	volatile unsigned int						vtim0; //< vertical timing 0 register
	volatile unsigned int						vtim1; //< vertical timing 1 register
	volatile unsigned int						htim1; //< horizontal timing register
	volatile unsigned int						control; //< control register
	volatile unsigned int						rsvd0; //< RSVD0
	volatile unsigned int						fbuffer0; //< frame buffer 0 register
	volatile unsigned int						fbuffer1; //< frame buffer 1 register
	volatile unsigned int						interrupt; //< interrupt mask register
	volatile unsigned int						status; //< status register
	volatile unsigned int						advtim; //< HRTFT timing register
	volatile unsigned int						trek; //< Trusted Entery Key Reg
	volatile unsigned int						thv; //< HSYNC-VSYNC phase Register
	volatile unsigned int						rsvd1[243]; //< RSVD1 unmapped
	volatile unsigned int						palette[256]; //< palette RAM register OFFST=400H

} mml_tdc_regs_t;


#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_TDC_REGS */

#endif /* _MML_TDC_REGS_H_ */

/******************************************************************************/
/* EOF */
