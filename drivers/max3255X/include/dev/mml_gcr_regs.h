/*
 * mml_gcr_regs.h --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2014, Maxim Integrated Products
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
 * Created on: June 05, 2014
 * Author: Jeremy B. <jeremy.brodt@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision: 4561 $:  Revision of last commit
 * $Author: robert.muchsel $:  Author of last commit
 * $Date: 2015-01-16 17:42:05 -0600 (Fri, 16 Jan 2015) $:  Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_GCR_REGS_H_
#define _MML_GCR_REGS_H_

/** @file mml_gcr_regs.h GCR Registers Header */

/** @defgroup MML_GCR GCR Driver
 * @ingroup MML_DRIVER
 *
 * */

/** @defgroup MML_GCR_REGS GCR Registers
 *
 *
 *
 * @ingroup MML_GCR
 * @{
 */


/* -------------------------------------------------------------------------- */
/** @defgroup MML_GCR_SCON GCR System Control Register
 *
 * @li 0 BSTAPEN: TAP Disable Bit
 * @li 2-1 RFU
 * @li 3 VBATSW: Battery supply source flag
 * @li 4 FLASH_PAGE_FLIP: Flips~ the Flash bottom and top halves (depending on total Flash size, each half is either 256KB or 512KB)
 * @li 12-5 RFU
 * @li 13 CCHK: Compute ROM Checksum. This bit is self-cleared when calculation is completed. 
 * @li 14 CHKRD: ROM Checksum Ready
 * @li 15 CHKRES: ROM Checksum Result. 
 * @{
 */

#define MML_GCR_SCON_OFST						0x00000000 //!< SCON register offset
#define MML_GCR_SCON_DFLT						0x00000001 //!< SCON register default value

#define MML_GCR_SCON_BSTAPEN_OFST				0 //!< BSTAPEN bits offset
#define MML_GCR_SCON_BSTAPEN_MASK_NOOFST		0x1 //!< BSTAPEN mask
#define MML_GCR_SCON_BSTAPEN_MASK				( MML_GCR_SCON_BSTAPEN_MASK_NOOFST << MML_GCR_SCON_BSTAPEN_OFST ) //!< BSTAPEN mask

#define MML_GCR_SCON_BSTAPEN_ENABLE				1 //! Disable JTAG TAP pins and allows them to be used as GPIO.

#define	MML_GCR_SCON_ONVMAIN_OFST				3
#define	MML_GCR_SCON_ONVMAIN_MASK_NOOFST		0x1
#define	MML_GCR_SCON_ONVMAIN_MASK				( MML_GCR_SCON_ONVMAIN_MASK_NOOFST << MML_GCR_SCON_ONVMAIN_OFST )

#define	MML_GCR_SCON_FLASH_PAGE_FLIP_OFST		4
#define	MML_GCR_SCON_FLASH_PAGE_FLIP_MASK_NOOFST	0x1
#define	MML_GCR_SCON_FLASH_PAGE_FLIP_MASK		( MML_GCR_SCON_FLASH_PAGE_FLIP_MASK_NOOFST << MML_GCR_SCON_FLASH_PAGE_FLIP_OFST )

#define	MML_GCR_SCON_CCHK_OFST					13
#define	MML_GCR_SCON_CCHK_MASK_NOOFST			0x1
#define	MML_GCR_SCON_CCHK_MASK					( MML_GCR_SCON_CCHK_MASK_NOOFST << MML_GCR_SCON_CCHK_OFST )

#define	MML_GCR_SCON_CHKRD_OFST					14
#define	MML_GCR_SCON_CHKRD_MASK_NOOFST			0x1
#define	MML_GCR_SCON_CHKRD_MASK					( MML_GCR_SCON_CHKRD_MASK_NOOFST << MML_GCR_SCON_CHKRD_OFST )

#define	MML_GCR_SCON_CHKRES_OFST				15
#define	MML_GCR_SCON_CHKRES_MASK_NOOFST			0x1
#define	MML_GCR_SCON_CHKRES_MASK				( MML_GCR_SCON_CHKRES_MASK_NOOFST << MML_GCR_SCON_CHKRES_OFST )
/** @} */ /* @defgroup MML_GCR_SCON */

/* -------------------------------------------------------------------------- */
/** @defgroup MML_GCR_RSTR Reset Register (RSTR)
 *
 * @{
 */

#define MML_GCR_RSTR_OFST						0x00000004 //!< Reset register offset
#define MML_GCR_RSTR_DFLT						0x00000000 //!< Reset register default value

#define	MML_GCR_RSTR_DMA_OFST					0
#define	MML_GCR_RSTR_DMA_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_DMA_MASK					( MML_GCR_RSTR_DMA_MASK_NOOFST << MML_GCR_RSTR_DMA_OFST )

#define	MML_GCR_RSTR_WDT_OFST					1
#define	MML_GCR_RSTR_WDT_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_WDT_MASK					( MML_GCR_RSTR_WDT_MASK_NOOFST << MML_GCR_RSTR_WDT_OFST )

#define	MML_GCR_RSTR_GPIO_OFST					2
#define	MML_GCR_RSTR_GPIO_MASK_NOOFST			0x7
#define	MML_GCR_RSTR_GPIO_MASK					( MML_GCR_RSTR_GPIO_MASK_NOOFST << MML_GCR_RSTR_GPIO_OFST )

/** bit5 - bit10, bit24 & bit27  */
#define	MML_GCR_RSTR_TIMER0_OFST				5
#define	MML_GCR_RSTR_TIMER0_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_TIMER0_MASK				( MML_GCR_RSTR_TIMER0_MASK_NOOFST << MML_GCR_RSTR_TIMER0_OFST )

#define	MML_GCR_RSTR_TIMER1_OFST				6
#define	MML_GCR_RSTR_TIMER1_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_TIMER1_MASK				( MML_GCR_RSTR_TIMER1_MASK_NOOFST << MML_GCR_RSTR_TIMER1_OFST )

#define	MML_GCR_RSTR_TIMER2_OFST				7
#define	MML_GCR_RSTR_TIMER2_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_TIMER2_MASK				( MML_GCR_RSTR_TIMER2_MASK_NOOFST << MML_GCR_RSTR_TIMER2_OFST )

#define	MML_GCR_RSTR_TIMER3_OFST				8
#define	MML_GCR_RSTR_TIMER3_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_TIMER3_MASK				( MML_GCR_RSTR_TIMER3_MASK_NOOFST << MML_GCR_RSTR_TIMER3_OFST )

#define	MML_GCR_RSTR_TIMER4_OFST				9
#define	MML_GCR_RSTR_TIMER4_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_TIMER4_MASK				( MML_GCR_RSTR_TIMER4_MASK_NOOFST << MML_GCR_RSTR_TIMER4_OFST )

#define	MML_GCR_RSTR_TIMER5_OFST				10
#define	MML_GCR_RSTR_TIMER5_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_TIMER5_MASK				( MML_GCR_RSTR_TIMER5_MASK_NOOFST << MML_GCR_RSTR_TIMER5_OFST )

#define	MML_GCR_RSTR_UART0_OFST					11
#define	MML_GCR_RSTR_UART0_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_UART0_MASK					( MML_GCR_RSTR_UART0_MASK_NOOFST << MML_GCR_RSTR_UART0_OFST )

#define	MML_GCR_RSTR_UART1_OFST					12
#define	MML_GCR_RSTR_UART1_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_UART1_MASK					( MML_GCR_RSTR_UART1_MASK_NOOFST << MML_GCR_RSTR_UART1_OFST )

#define	MML_GCR_RSTR_SPI_OFST					13
#define	MML_GCR_RSTR_SPI_MASK_NOOFST			0x7
#define	MML_GCR_RSTR_SPI_MASK					( MML_GCR_RSTR_SPI_MASK_NOOFST << MML_GCR_RSTR_SPI_OFST )

#define	MML_GCR_RSTR_I2C_OFST					16
#define	MML_GCR_RSTR_I2C_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_I2C_MASK					( MML_GCR_RSTR_I2C_MASK_NOOFST << MML_GCR_RSTR_I2C_OFST )

#define	MML_GCR_RSTR_RTC_OFST					17
#define	MML_GCR_RSTR_RTC_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_RTC_MASK					( MML_GCR_RSTR_RTC_MASK_NOOFST << MML_GCR_RSTR_RTC_OFST )

#define	MML_GCR_RSTR_CRYPTO_OFST				18
#define	MML_GCR_RSTR_CRYPTO_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_CRYPTO_MASK				( MML_GCR_RSTR_CRYPTO_MASK_NOOFST << MML_GCR_RSTR_CRYPTO_OFST )

#define	MML_GCR_RSTR_MAGDSP_OFST				19
#define	MML_GCR_RSTR_MAGDSP_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_MAGDSP_MASK				( MML_GCR_RSTR_MAGDSP_MASK_NOOFST << MML_GCR_RSTR_MAGDSP_OFST )

#define	MML_GCR_RSTR_SC_OFST					20
#define	MML_GCR_RSTR_SC_MASK_NOOFST				0x1
#define	MML_GCR_RSTR_SC_MASK					( MML_GCR_RSTR_SC_MASK_NOOFST << MML_GCR_RSTR_SC_OFST )

#define	MML_GCR_RSTR_KBD_OFST					21
#define	MML_GCR_RSTR_KBD_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_KBD_MASK					( MML_GCR_RSTR_KBD_MASK_NOOFST << MML_GCR_RSTR_KBD_OFST )

#define	MML_GCR_RSTR_TFTLCD_OFST				22
#define	MML_GCR_RSTR_TFTLCD_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_TFTLCD_MASK				( MML_GCR_RSTR_TFTLCD_MASK_NOOFST << MML_GCR_RSTR_TFTLCD_OFST )

#define	MML_GCR_RSTR_USB_OFST					23
#define	MML_GCR_RSTR_USB_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_USB_MASK					( MML_GCR_RSTR_USB_MASK_NOOFST << MML_GCR_RSTR_USB_OFST )

#define	MML_GCR_RSTR_TIMER6_OFST				24
#define	MML_GCR_RSTR_TIMER6_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_TIMER6_MASK				( MML_GCR_RSTR_TIMER6_MASK_NOOFST << MML_GCR_RSTR_TIMER6_OFST )

#define	MML_GCR_RSTR_ADC_OFST					25
#define	MML_GCR_RSTR_ADC_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_ADC_MASK					( MML_GCR_RSTR_ADC_MASK_NOOFST << MML_GCR_RSTR_ADC_OFST )

#define	MML_GCR_RSTR_DAC_OFST					26
#define	MML_GCR_RSTR_DAC_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_DAC_MASK					( MML_GCR_RSTR_DAC_MASK_NOOFST << MML_GCR_RSTR_DAC_OFST )

#define	MML_GCR_RSTR_TIMER7_OFST				27
#define	MML_GCR_RSTR_TIMER7_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_TIMER7_MASK				( MML_GCR_RSTR_TIMER7_MASK_NOOFST << MML_GCR_RSTR_TIMER7_OFST )

#define	MML_GCR_RSTR_UART2_OFST					28
#define	MML_GCR_RSTR_UART2_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_UART2_MASK					( MML_GCR_RSTR_UART2_MASK_NOOFST << MML_GCR_RSTR_UART2_OFST )

#define	MML_GCR_RSTR_SRST_OFST					29
#define	MML_GCR_RSTR_SRST_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_SRST_MASK					( MML_GCR_RSTR_SRST_MASK_NOOFST << MML_GCR_RSTR_SRST_OFST )

#define	MML_GCR_RSTR_PRST_OFST					30
#define	MML_GCR_RSTR_PRST_MASK_NOOFST			0x1
#define	MML_GCR_RSTR_PRST_MASK					( MML_GCR_RSTR_PRST_MASK_NOOFST << MML_GCR_RSTR_PRST_OFST )

#define	MML_GCR_RSTR_SYSTEM_OFST				31 //!< System Reset bit offset
#define	MML_GCR_RSTR_SYSTEM_MASK_NOOFST			0x1u
#define	MML_GCR_RSTR_SYSTEM_MASK				( MML_GCR_RSTR_SYSTEM_MASK_NOOFST << MML_GCR_RSTR_SYSTEM_OFST ) //!< System Reset bit mask


#define	MML_GCR_RSTR_UARTS_MASK					( MML_GCR_RSTR_UART0_MASK |\
													MML_GCR_RSTR_UART1_MASK |\
													MML_GCR_RSTR_UART2_MASK )

#define	MML_GCR_RSTR_TIMERS_MASK				( MML_GCR_RSTR_TIMER0_MASK |\
													MML_GCR_RSTR_TIMER1_MASK |\
													MML_GCR_RSTR_TIMER2_MASK |\
													MML_GCR_RSTR_TIMER3_MASK |\
													MML_GCR_RSTR_TIMER4_MASK |\
													MML_GCR_RSTR_TIMER5_MASK |\
													MML_GCR_RSTR_TIMER6_MASK |\
													MML_GCR_RSTR_TIMER7_MASK )
/** @} */ /* @defgroup MML_GCR_RSTR */

/* -------------------------------------------------------------------------- */
/** @defgroup MML_GCR_CLKCN Clock Control Register
 *
 * @li 0-2 HDIV [2:0]: AHB Bus Clock Divider.
 * @li 4-5 PDIV [2:0] Peripheral Bus Clock Divider.
 * @li 6-8 PSC [2:0]: Prescaler Select.
 * @li 9-11 CLKSEL [2:0]: Clock Source Select.
 * @li 13 CKRDY [0]: Clock Ready.
 * @li
 *
 *
 *
 * @{
 */
#define MML_GCR_CLKCN_OFST						0x00000008 //!< Clock Control register offset
#define MML_GCR_CLKCN_DFLT						0x00000000 //!< Clock Control register default value

#define MML_GCR_CLKCN_PSC_OFST					6 //!< PSC bits offset
#define MML_GCR_CLKCN_PSC_MASK_NOOFST			0x7 //!< PSC bits offset
#define MML_GCR_CLKCN_PSC_MASK					( MML_GCR_CLKCN_PSC_MASK_NOOFST << MML_GCR_CLKCN_PSC_OFST )

#define MML_GCR_CLKCN_CLKSEL_OFST				9 //!< CLKSEL bits offset
#define MML_GCR_CLKCN_CLKSEL_MASK_NOOFST		0x7 //!< CLKSEL bits offset
#define MML_GCR_CLKCN_CLKSEL					( MML_GCR_CLKCN_CLKSEL_MASK_NOOFST << MML_GCR_CLKCN_CLKSEL_OFST )

#define MML_GCR_CLKCN_CKRDY_OFST				13 //!< CKRDY bits offset
#define MML_GCR_CLKCN_CKRDY_MASK_NOOFST			0x1 //!< CKRDY bits offset
#define MML_GCR_CLKCN_CKRDY_MASK				( MML_GCR_CLKCN_CKRDY_MASK_NOOFST << MML_GCR_CLKCN_CKRDY_OFST )

/** @} */ /* @defgroup MML_GCR_CLKCN */

/* -------------------------------------------------------------------------- */
/** @defgroup MML_GCR_PM Power Management Register (PMR)
 *
 * @{
 */

#define MML_GCR_PM_OFST							0x0000000c //!< Power Mode register offset
#define MML_GCR_PM_DFLT							0x00000000 //!< Power Mode register default value

//PMR reg
#define MML_GCR_PM_MODE_OFST					0 //!< Power Mode Mode bits offset
#define MML_GCR_PM_MODE_MASK_NOOFST				0x3 //!< Power Mode Mode bits mask
#define MML_GCR_PM_MODE_MASK					( MML_GCR_PMR_MODE_MASK_NOOFST << MML_GCR_PMR_MODE_OFST ) //!< Power Mode Mode bits mask

#define	MML_GCR_PM_GPIOWKEN_OFST				4
#define	MML_GCR_PM_GPIOWKEN_MASK_NOOFST			0x1
#define	MML_GCR_PM_GPIOWKEN_MASK				( MML_GCR_PM_GPIOWKEN_MASK_NOOFST << MML_GCR_PM_GPIOWKEN_OFST )

#define	MML_GCR_PM_RTCWKEN_OFST					5
#define	MML_GCR_PM_RTCWKEN_MASK_NOOFST			0x1
#define	MML_GCR_PM_RTCWKEN_MASK					( MML_GCR_PM_RTCWKEN_MASK_NOOFST << MML_GCR_PM_RTCWKEN_OFST )

#define	MML_GCR_PM_USBWKEN_OFST					6
#define	MML_GCR_PM_USBWKEN_MASK_NOOFST			0x1
#define	MML_GCR_PM_USBWKEN_MASK					( MML_GCR_PM_USBWKEN_MASK_NOOFST << MML_GCR_PM_USBWKEN_OFST )

#define	MML_GCR_PM_SCWKEN_OFST					7
#define	MML_GCR_PM_SCWKEN_MASK_NOOFST			0x1
#define	MML_GCR_PM_SCWKEN_MASK					( MML_GCR_PM_SCWKEN_MASK_NOOFST << MML_GCR_PM_SCWKEN_OFST )

#define	MML_GCR_PM_OSCPD_OFST					12
#define	MML_GCR_PM_OSCPD_MASK_NOOFST			0x1
#define	MML_GCR_PM_OSCPD_MASK					( MML_GCR_PM_OSCPD_MASK_NOOFST << MML_GCR_PM_OSCPD_OFST )

#define	MML_GCR_PM_PLL01PD_OFST					13
#define	MML_GCR_PM_PLL01PD_MASK_NOOFST			0x1
#define	MML_GCR_PM_PLL01PD_MASK					( MML_GCR_PM_PLL01PD_MASK_NOOFST << MML_GCR_PM_PLL01PD_OFST )

#define	MML_GCR_PM_COPD_OFST					14
#define	MML_GCR_PM_COPD_MASK_NOOFST				0x1
#define	MML_GCR_PM_COPD_MASK					( MML_GCR_PM_COPD_MASK_NOOFST << MML_GCR_PM_COPD_OFST )

#define	MML_GCR_PM_CM3PMUEN_OFST				16
#define	MML_GCR_PM_CM3PMUEN_MASK_NOOFST			0x1
#define	MML_GCR_PM_CM3PMUEN_MASK				( MML_GCR_PM_CM3PMUEN_MASK_NOOFST << MML_GCR_PM_CM3PMUEN_OFST )

#define	MML_GCR_PM_CM3WICACK_OFST				17
#define	MML_GCR_PM_CM3WICACK_MASK_NOOFST		0x1
#define	MML_GCR_PM_CM3WICACK_MASK				( MML_GCR_PM_CM3WICACK_MASK_NOOFST << MML_GCR_PM_CM3WICACK_OFST )

#define MML_GCR_PM_MODE_ACTIVE					0x0
#define MML_GCR_PM_MODE_IDLE					0x1
#define MML_GCR_PM_MODE_STANDBY					0x2
#define MML_GCR_PM_MODE_SHUTDOWN				0x3
/** @} */ /* @defgroup MML_GCR_PM */

/* -------------------------------------------------------------------------- */
/** @defgroup MML_GCR_PLL0CN PLL0 Control Register (PLL0CN)
 *
 * @{
 */
#define MML_GCR_PLL0CN_OFST						0x00000010 //!< PLL Control Register offset
#define MML_GCR_PLL0CN_DFLT						0x00000000 //!< PLL Control Register default value

#define	MML_GCR_PLL0CN_PLL0EN_OFST				0
#define	MML_GCR_PLL0CN_PLL0EN_MASK_NOOFST		0x1
#define	MML_GCR_PLL0CN_PLL0EN_MASK				( MML_GCR_PLL0CN_PLL0EN_MASK_NOOFST << MML_GCR_PLL0CN_PLL0EN_OFST )

#define	MML_GCR_PLL0CN_PLL0LOCK_OFST			1
#define	MML_GCR_PLL0CN_PLL0LOCK_MASK_NOOFST		0x1
#define	MML_GCR_PLL0CN_PLL0LOCK_MASK			( MML_GCR_PLL0CN_PLL0LOCK_MASK_NOOFST << MML_GCR_PLL0CN_PLL0LOCK_OFST )
/** @} */ /* @defgroup MML_GCR_PLLCN */

/* -------------------------------------------------------------------------- */
/** @defgroup MML_GCR_PLL1CN PLL1 Control Register (PLL1CN)
 *
 * @{
 */
#define MML_GCR_PLL1CN_OFST						0x00000014 //!< PLL Control Register offset
#define MML_GCR_PLL1CN_DFLT						0x00000000 //!< PLL Control Register default value

#define	MML_GCR_PLL1CN_PLL1EN_OFST				0
#define	MML_GCR_PLL1CN_PLL1EN_MASK_NOOFST		0x1
#define	MML_GCR_PLL1CN_PLL1EN_MASK				( MML_GCR_PLL1CN_PLL1EN_MASK_NOOFST << MML_GCR_PLL1CN_PLL1EN_OFST )

#define	MML_GCR_PLL1CN_PLL1LOCK_OFST			1
#define	MML_GCR_PLL1CN_PLL1LOCK_MASK_NOOFST		0x1
#define	MML_GCR_PLL1CN_PLL1LOCK_MASK			( MML_GCR_PLL1CN_PLL1LOCK_MASK_NOOFST << MML_GCR_PLL1CN_PLL1LOCK_OFST )
/** @} */ /* @defgroup MML_GCR_PLLCN */


/* -------------------------------------------------------------------------- */
/** @defgroup MML_GCR_PCKDIV
 *
 * @{
 */
#define MML_GCR_PCKDIV_OFST						0x00000018 //!< PCKDIV register offset
#define MML_GCR_PCKDIV_DFLT						0x00000001 //!< PCKDIV register default value

#define MML_GCR_PCKDIV_PCF_OFST					0
#define MML_GCR_PCKDIV_PCF_MASK_NOOFST			0x7
#define MML_GCR_PCKDIV_PCF_MASK					( MML_GCR_PCKDIV_PCF_MASK_NOOFST << MML_GCR_PCKDIV_PCF_OFST )
/** @} */ /* @defgroup MML_GCR_PCKDIV */

/* -------------------------------------------------------------------------- */
/** @defgroup MML_GCR_PERCKCN GCR Peripheral Clock Disable Register
 *
 * @{
 */

#define MML_GCR_PERCKCN_OFST					0x00000024 //!< PERCKCN register offset
#define MML_GCR_PERCKCN_DFLT					0x00000000 //!< PERCKCN register default value

#define MML_GCR_PERCKCN_GPIOxD_OFST				0
#define MML_GCR_PERCKCN_GPIOxD_MASK_NOOFST		0x7
#define MML_GCR_PERCKCN_GPIOxD_MASK				( MML_GCR_PERCKCN_GPIOxD_MASK_NOOFST << MML_GCR_PERCKCN_GPIOxD_OFST )

#define MML_GCR_PERCKCN_USBD_OFST				3
#define MML_GCR_PERCKCN_USBD_MASK_NOOFST		0x1
#define MML_GCR_PERCKCN_USBD_MASK				( MML_GCR_PERCKCN_USBD_MASK_NOOFST << MML_GCR_PERCKCN_USBD_OFST )

#define MML_GCR_PERCKCN_UART2D_OFST				4
#define MML_GCR_PERCKCN_UART2D_MASK_NOOFST		0x1
#define MML_GCR_PERCKCN_UART2D_MASK				( MML_GCR_PERCKCN_UART2D_MASK_NOOFST << MML_GCR_PERCKCN_UART2D_OFST )

#define MML_GCR_PERCKCN_DMAD_OFST				5
#define MML_GCR_PERCKCN_DMAD_MASK_NOOFST		0x1
#define MML_GCR_PERCKCN_DMAD_MASK				( MML_GCR_PERCKCN_DMAD_MASK_NOOFST << MML_GCR_PERCKCN_DMAD_OFST )

#define MML_GCR_PERCKCN_SPIxD_OFST				6
#define MML_GCR_PERCKCN_SPIxD_MASK_NOOFST		0x7
#define MML_GCR_PERCKCN_SPIxD_MASK				( MML_GCR_PERCKCN_SPIxD_MASK_NOOFST << MML_GCR_PERCKCN_SPIxD_OFST )

#define MML_GCR_PERCKCN_UART0D_OFST				9
#define MML_GCR_PERCKCN_UART0D_MASK_NOOFST		0x1
#define MML_GCR_PERCKCN_UART0D_MASK				( MML_GCR_PERCKCN_UART0D_MASK_NOOFST << MML_GCR_PERCKCN_UART0D_OFST )

#define MML_GCR_PERCKCN_UART1D_OFST				10
#define MML_GCR_PERCKCN_UART1D_MASK_NOOFST		0x1
#define MML_GCR_PERCKCN_UART1D_MASK				( MML_GCR_PERCKCN_UART1D_MASK_NOOFST << MML_GCR_PERCKCN_UART1D_OFST )

#define MML_GCR_PERCKCN_MAGDSPD_OFST			11
#define MML_GCR_PERCKCN_MAGDSPD_MASK_NOOFST		0x1
#define MML_GCR_PERCKCN_MAGDSPD_MASK			( MML_GCR_PERCKCN_MAGDSPD_MASK_NOOFST << MML_GCR_PERCKCN_MAGDSPD_OFST )

#define MML_GCR_PERCKCN_SCD_OFST				12
#define MML_GCR_PERCKCN_SCD_MASK_NOOFST			0x1
#define MML_GCR_PERCKCN_SCD_MASK				( MML_GCR_PERCKCN_SCD_MASK_NOOFST << MML_GCR_PERCKCN_SCD_OFST )

#define MML_GCR_PERCKCN_I2CD_OFST				13
#define MML_GCR_PERCKCN_I2CD_MASK_NOOFST		0x1
#define MML_GCR_PERCKCN_I2CD_MASK				( MML_GCR_PERCKCN_I2CD_MASK_NOOFST << MML_GCR_PERCKCN_I2CD_OFST )

#define MML_GCR_PERCKCN_CRYPTOD_OFST			14
#define MML_GCR_PERCKCN_CRYPTOD_MASK_NOOFST		0x1
#define MML_GCR_PERCKCN_CRYPTOD_MASK			( MML_GCR_PERCKCN_CRYPTOD_MASK_NOOFST << MML_GCR_PERCKCN_CRYPTOD_OFST )

#define MML_GCR_PERCKCN_T0D_OFST				15
#define MML_GCR_PERCKCN_T0D_MASK_NOOFST			0x1
#define MML_GCR_PERCKCN_T0D_MASK				( MML_GCR_PERCKCN_T0D_MASK_NOOFST << MML_GCR_PERCKCN_T0D_OFST )

#define MML_GCR_PERCKCN_T1D_OFST				16
#define MML_GCR_PERCKCN_T1D_MASK_NOOFST			0x1
#define MML_GCR_PERCKCN_T1D_MASK				( MML_GCR_PERCKCN_T1D_MASK_NOOFST << MML_GCR_PERCKCN_T1D_OFST )

#define MML_GCR_PERCKCN_T2D_OFST				17
#define MML_GCR_PERCKCN_T2D_MASK_NOOFST			0x1
#define MML_GCR_PERCKCN_T2D_MASK				( MML_GCR_PERCKCN_T2D_MASK_NOOFST << MML_GCR_PERCKCN_T2D_OFST )

#define MML_GCR_PERCKCN_T3D_OFST				18
#define MML_GCR_PERCKCN_T3D_MASK_NOOFST			0x1
#define MML_GCR_PERCKCN_T3D_MASK				( MML_GCR_PERCKCN_T3D_MASK_NOOFST << MML_GCR_PERCKCN_T3D_OFST )

#define MML_GCR_PERCKCN_T4D_OFST				19
#define MML_GCR_PERCKCN_T4D_MASK_NOOFST			0x1
#define MML_GCR_PERCKCN_T4D_MASK				( MML_GCR_PERCKCN_T4D_MASK_NOOFST << MML_GCR_PERCKCN_T4D_OFST )

#define MML_GCR_PERCKCN_T5D_OFST				20
#define MML_GCR_PERCKCN_T5D_MASK_NOOFST			0x1
#define MML_GCR_PERCKCN_T5D_MASK				( MML_GCR_PERCKCN_T5D_MASK_NOOFST << MML_GCR_PERCKCN_T5D_OFST )

#define MML_GCR_PERCKCN_MLCDD_OFST				21
#define MML_GCR_PERCKCN_MLCDD_MASK_NOOFST		0x1
#define MML_GCR_PERCKCN_MLCDD_MASK				( MML_GCR_PERCKCN_MLCDD_MASK_NOOFST << MML_GCR_PERCKCN_MLCDD_OFST )

#define MML_GCR_PERCKCN_KBDD_OFST				22
#define MML_GCR_PERCKCN_KBDD_MASK_NOOFST		0x1
#define MML_GCR_PERCKCN_KBDD_MASK				( MML_GCR_PERCKCN_KBDD_MASK_NOOFST << MML_GCR_PERCKCN_KBDD_OFST )

#define MML_GCR_PERCKCN_ADCD_OFST				23
#define MML_GCR_PERCKCN_ADCD_MASK_NOOFST		0x1
#define MML_GCR_PERCKCN_ADCD_MASK				( MML_GCR_PERCKCN_ADCD_MASK_NOOFST << MML_GCR_PERCKCN_ADCD_OFST )

#define MML_GCR_PERCKCN_DACD_OFST				24
#define MML_GCR_PERCKCN_DACD_MASK_NOOFST		0x1
#define MML_GCR_PERCKCN_DACD_MASK				( MML_GCR_PERCKCN_DACD_MASK_NOOFST << MML_GCR_PERCKCN_DACD_OFST )

#define MML_GCR_PERCKCN_T6D_OFST				26
#define MML_GCR_PERCKCN_T6D_MASK_NOOFST			0x1
#define MML_GCR_PERCKCN_T6D_MASK				( MML_GCR_PERCKCN_T6D_MASK_NOOFST << MML_GCR_PERCKCN_T6D_OFST )

#define MML_GCR_PERCKCN_T7D_OFST				27
#define MML_GCR_PERCKCN_T7D_MASK_NOOFST			0x1
#define MML_GCR_PERCKCN_T7D_MASK				( MML_GCR_PERCKCN_T7D_MASK_NOOFST << MML_GCR_PERCKCN_T7D_OFST )

#define	MML_GCR_PERCKCN_UARTxD_MASK				( MML_GCR_PERCKCN_UART0D_MASK |\
													MML_GCR_PERCKCN_UART1D_MASK |\
													MML_GCR_PERCKCN_UART2D_MASK )

#define	MML_GCR_PERCKCN_TxD_MASK				( MML_GCR_PERCKCN_T0D_MASK |\
													MML_GCR_PERCKCN_T1D_MASK |\
													MML_GCR_PERCKCN_T2D_MASK |\
													MML_GCR_PERCKCN_T3D_MASK |\
													MML_GCR_PERCKCN_T4D_MASK |\
													MML_GCR_PERCKCN_T5D_MASK |\
													MML_GCR_PERCKCN_T6D_MASK |\
													MML_GCR_PERCKCN_T7D_MASK )

/** @} */ /* @defgroup MML_GCR_PERCKCN */

/* -------------------------------------------------------------------------- */
/** @defgroup MML_GCR_MEMZCN GCR Memory Zero-ize Control Register
 *
 * @{
 */

#define MML_GCR_MEMZCN_OFST						0x0000002c //!< MEMZCN register offset
#define MML_GCR_MEMZCN_DFLT						0x00000000 //!< MEMZCN register default value

#define MML_GCR_MEMZCN_SRAMx_OFST				0
#define MML_GCR_MEMZCN_SRAMx_MASK_NOOFST		0x3
#define MML_GCR_MEMZCN_SRAMx_MASK				( MML_GCR_MEMZCN_SRAMx_MASK_NOOFST << MML_GCR_MEMZCN_SRAMx_OFST )

#define MML_GCR_MEMZCN_NVSRAM_OFST				2
#define MML_GCR_MEMZCN_NVSRAM_MASK_NOOFST		0x1
#define MML_GCR_MEMZCN_NVSRAM_MASK				( MML_GCR_MEMZCN_NVSRAM_MASK_NOOFST << MML_GCR_MEMZCN_NVSRAM_OFST )

#define MML_GCR_MEMZCN_ICACHE_OFST				3
#define MML_GCR_MEMZCN_ICACHE_MASK_NOOFST		0x1
#define MML_GCR_MEMZCN_ICACHE_MASK				( MML_GCR_MEMZCN_ICACHE_MASK_NOOFST << MML_GCR_MEMZCN_ICACHE_OFST )

#define MML_GCR_MEMZCN_CRYPTO_OFST				5
#define MML_GCR_MEMZCN_CRYPTO_MASK_NOOFST		0x1
#define MML_GCR_MEMZCN_CRYPTO_MASK				( MML_GCR_MEMZCN_CRYPTO_MASK_NOOFST << MML_GCR_MEMZCN_CRYPTO_OFST )
/** @} */ /* @defgroup MML_GCR_MEMZCN */


/* -------------------------------------------------------------------------- */
/** @defgroup MML_GCR_SCCK GCR Smart Card Clock Control Register
 *
 * @{
 */

#define MML_GCR_SCCK_OFST						0x00000034 //!< SCCK register offset
#define MML_GCR_SCCK_DFLT						0x0000000a //!< SCCK register default value

#define	MML_GCR_SCCK_SC0FRQ_OFST				0
#define	MML_GCR_SCCK_SC0FRQ_MASK_NOOFST			0x3f
#define	MML_GCR_SCCK_SC0FRQ_MASK				( MML_GCR_SCCK_SC0FRQ_MASK_NOOFST << MML_GCR_SCCK_SC0FRQ_OFST )
/** @} */ /* @defgroup MML_GCR_SCCK */

#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */

/** GCR Registers.
 *
 */
typedef volatile struct
{
	/** System Control Register - 0x00000000 */
	unsigned int								scon;
	/** Reset Register - 0x00000004 */
	unsigned int								rstr;
	/** Clock Control Register - 0x00000008 */
	unsigned int								clkcn;
	/** Power Management Register - 0x0000000c */
	unsigned int								pm;
	/** PLL0 Control Register - 0x00000010 */
	unsigned int								pll0cn;
	/** PLL1 Control Register - 0x00000014 */
	unsigned int								pll1cn;
	/** Peripheral clock divider - 0x00000018 */
	unsigned int								pckdiv;
	/** RFU - 0x0000001c - 20*/
	unsigned int								rfu0[2];
	/** Peripheral Clock Disable Register - 0x00000024 */
	unsigned int								perckcn;
	/** RFU - 0x00000028 */
	unsigned int								rfu1;
	/** Memory Zero-ize Control Register - 0x0000002c */
	unsigned int								memzcn;
	/** RFU - 0x00000030 */
	unsigned int								rfu2;
	/** Smart Card Clock Control Register - 0x00000034 */
	unsigned int								scck;

} mml_gcr_regs_t;

/** RSTR devices
 *
 */
typedef enum
{
    /**  */
    MML_RSTR_DEV_MIN = 0,
    MML_RSTR_DEV_DMA = MML_RSTR_DEV_MIN,
    MML_RSTR_DEV_WDT,
    MML_RSTR_DEV_GPIO0,
    MML_RSTR_DEV_GPIO1,
    MML_RSTR_DEV_GPIO2,
    MML_RSTR_DEV_TIMER0,
    MML_RSTR_DEV_TIMER1,
    MML_RSTR_DEV_TIMER2,
    MML_RSTR_DEV_TIMER3,
    MML_RSTR_DEV_TIMER4,
    MML_RSTR_DEV_TIMER5,
    MML_RSTR_DEV_UART0,
    MML_RSTR_DEV_UART1,
    MML_RSTR_DEV_SPI0,
    MML_RSTR_DEV_SPI1,
    MML_RSTR_DEV_SPI2,
    MML_RSTR_DEV_I2C,
    MML_RSTR_DEV_RTC,
    MML_RSTR_DEV_CRYPTO,
    MML_RSTR_DEV_MAGDSP,
    MML_RSTR_DEV_SC,
    MML_RSTR_DEV_KBD,
    MML_RSTR_DEV_LCD_TFT,
    MML_RSTR_DEV_USB_BD,
    MML_RSTR_DEV_TRNG_TIMER6,
    MML_RSTR_DEV_ADC,
    MML_RSTR_DEV_DAC,
    MML_RSTR_DEV_TIMER7,
    MML_RSTR_DEV_UART2,
    MML_RSTR_DEV_SRST,
    MML_RSTR_DEV_PRST,
    MML_RSTR_DEV_ALL,
    MML_RSTR_DEV_MAX = MML_RSTR_DEV_ALL,
    MML_RSTR_DEV_COUNT

} mml_rstr_dev_t;

/** PERCKCN devices
 *
 */
typedef enum
{
    MML_PERCKCN_DEV_MIN = 0, //!< MML_PERCKCN_DEV_MIN
    MML_PERCKCN_DEV_GPIO0 = MML_PERCKCN_DEV_MIN, //! Disables the GPIO0 clock.
    MML_PERCKCN_DEV_GPIO1, //! Disables the GPIO1 clock.
    MML_PERCKCN_DEV_GPIO2, //! Disables the GPIO2 clock.
    MML_PERCKCN_DEV_USB, //! Disables the USB clock.
    MML_PERCKCN_DEV_UART2, //! Disables the UART2 clock.
    MML_PERCKCN_DEV_DMA, //! Disables the DMA clock.
    MML_PERCKCN_DEV_SPI0, //! Disables the SPI0 clock.
    MML_PERCKCN_DEV_SPI1, //! Disables the SPI1 clock.
    MML_PERCKCN_DEV_SPI2, //! Disables the SPI2 clock.
    MML_PERCKCN_DEV_UART0, //! Disables the UART0 clock.
    MML_PERCKCN_DEV_UART1, //! Disables the UART1 clock.
    MML_PERCKCN_DEV_MAGDSP, //! Disables the MAGDSP clock.
    MML_PERCKCN_DEV_SC, //! Disables the SC clock.
    MML_PERCKCN_DEV_I2C, //! Disables the I²C clock.
    MML_PERCKCN_DEV_CRYPTO, //! Disables the Crypto clock.
    MML_PERCKCN_DEV_T0, //! Disables the Timer 0 clock.
    MML_PERCKCN_DEV_T1, //! Disables the Timer 1 clock.
    MML_PERCKCN_DEV_T2, //! Disables the Timer 2 clock.
    MML_PERCKCN_DEV_T3, //! Disables the Timer 3 clock.
    MML_PERCKCN_DEV_T4, //! Disables the Timer 4 clock.
    MML_PERCKCN_DEV_T5, //! Disables the Timer 5 clock.
    MML_PERCKCN_DEV_MLCD, //! Disables the MLCD clock.
    MML_PERCKCN_DEV_KBD, //! Disables the KBD clock.
    MML_PERCKCN_DEV_ADC, //! Disables the ADC clock.
    MML_PERCKCN_DEV_DAC, //! Disables the DAC clock.
    MML_PERCKCN_DEV_T6, //! Disables the Timer 6 clock.
    MML_PERCKCN_DEV_T7, //! Disables the Timer 7 clock.
    MML_PERCKCN_DEV_MAX = MML_PERCKCN_DEV_DAC, //!< MML_PERCKCN_DEV_MAX
    MML_PERCKCN_DEV_COUNT //!< MML_PERCKCN_DEV_COUNT

} mml_perckcn_dev_t;

/** System Frequencies
 *
 */
typedef enum
{
    MML_GCR_DIV_1 = 0x0,
    MML_GCR_DIV_2,
    MML_GCR_DIV_4,
    MML_GCR_DIV_8,
    MML_GCR_DIV_16,
    MML_GCR_DIV_32,
    MML_GCR_DIV_64,
    MML_GCR_DIV_128

} mml_gcr_sysfreq_t;


#endif /* __ASSEMBLER__ */

/** @} */ /* @defgroup MML_GCR_REGS */

#endif /* _MML_GCR_REGS_H_ */

/******************************************************************************/
/* EOF */
