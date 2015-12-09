/*
 * mml_io.h --
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
 * Created on: June 12, 2014
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision: 4487 $:  Revision of last commit
 * $Author: jeremy.kongs $:  Author of last commit
 * $Date: 2015-01-14 15:43:29 -0600 (Wed, 14 Jan 2015) $:  Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */
#ifndef _MML_IO_H_
#define _MML_IO_H_

/** Global includes */
#include <io.h>
/** Other includes */
//#include <mml_adc10_regs.h>
//#include <mml_bbsir_regs.h>
//#include <mml_crypto_regs.h>
//#include <mml_dac_regs.h>
//#include <mml_dma_regs.h>
//#include <mml_gcr_regs.h>
//#include <mml_gpio_regs.h>
//#include <mml_i2c_regs.h>
//#include <mml_i2cl_regs.h>
//#include <mml_icache_regs.h>
//#include <mml_intc_regs.h>
//#include <mml_mcr_regs.h>
//#include <mml_mlcd_regs.h>
//#include <mml_mpu_regs.h>
//#include <mml_rpu_regs.h>
//#include <mml_rtc_regs.h>
//#include <mml_sc_regs.h>
//#include <mml_sflc_regs.h>
//#include <mml_sir_regs.h>
//#include <mml_skbd_regs.h>
//#include <mml_smon_regs.h>
//#include <mml_snvrf_regs.h>
//#include <mml_spi_regs.h>
//#include <mml_tdc_regs.h>
//#include <mml_test_regs.h>
//#include <mml_tmr_regs.h>
//#include <mml_trng_regs.h>
//#include <mml_uart_regs.h>
//#include <mml_usb_regs.h>
//#include <mml_wdt_regs.h>
/** Local includes */


#ifndef __ASSEMBLER__
//#include <mml_intc.h>

// Nominal Frequency (Nf) Calculation
// Per MQ55 Spec: PCF = PSC + 2
// Nf = PCLK * 2^PCF
// PCLK = PLLO / 2^(PSC+1)
// Nf = 2 * PLLO
#define MML_NOM_FREQ         					( 2 * MML_PLLO_FREQ )

#define MML_UART_FREQ							MML_NOM_FREQ
#define MML_I2C_FREQ							MML_NOM_FREQ
#define MML_FLC_FREQ							MML_APB_FREQ
#define MML_SPI_FREQ							MML_APB_FREQ
#define MML_TIMER_FREQ							MML_APB_FREQ

/** MQ55 IOs Structure. */

//typedef struct
//{
//    mml_gcr_regs_t     *gcr; //! GCR
//    mml_sir_regs_t     *sir; //! SIR
//    mml_test_regs_t    *test; //! TEST interface control
//    mml_crypto_regs_t  *crypto; //! CRYPTO
//    mml_wdt_regs_t     *wdt; //! WDOG
//    mml_smon_regs_t    *smon; //! Security Monitor
//    mml_snvrf_regs_t   *snvrf; //! AES keys
//    mml_bbsir_regs_t   *bbsir; //! BBSIR
//    mml_rtc_regs_t     *rtc; //! RTC
//    mml_gpio_regs_t    *gpio0; //! GPIO 0
//    mml_gpio_regs_t    *gpio1; //! GPIO 1
//    mml_gpio_regs_t    *gpio2; //! GPIO 2
//    mml_i2c_regs_t     *i2c; //! I2C
//    mml_tmr_regs_t     *tmr0; //! Timer 0
//    mml_tmr_regs_t     *tmr1; //! Timer 1
//    mml_tmr_regs_t     *tmr2; //! Timer 2
//    mml_tmr_regs_t     *tmr3; //! Timer 3
//    mml_spi_regs_t     *spi0; //! SPI 0
//    mml_spi_regs_t     *spi1; //! SPI 1
//    mml_spi_regs_t     *spi2; //! SPI 2
//    mml_uart_regs_t    *uart0; //! UART 0
//    mml_uart_regs_t    *uart1; //! UART 1
//    mml_dma_regs_t     *dma; //! DMA
//    mml_sflc_regs_t    *sflc; //! Flash Control
//    mml_icache_regs_t  *icache; //! ICACHE
//    mml_mcr_regs_t     *mcr; //! Magnetic Stripe Reader DSP
//    mml_sc_regs_t      *sc; //! Smart Card
//    mml_mlcd_regs_t    *mlcd; //! mono LCD
//    mml_tdc_regs_t     *tdc; //! TDC/CLCD
//    mml_skbd_regs_t    *skbd; //! Secure KBD
//    mml_adc10_regs_t   *adc; //! Secure KBD
//    mml_dac_regs_t     *dac; //! DAC
//    mml_usb_regs_t     *usb; //! USB/BD
//    mml_trng_regs_t    *trng; //! TRNG
//    mml_intc_regs_t    *intc; //! Interrupt controller
//    mml_mpu_regs_t     *mpu;    //! Memory Protection Unit

//} mml_ios_t;


#define MML_GPIO_MODE_UART0  {MML_GPIO0_IOBASE,  8, 0x3}
#define MML_GPIO_MODE_UART1  {MML_GPIO0_IOBASE, 12, 0x3}

#endif /* __ASSEMBLER__ */

#endif /* _MML_IO_H_ */

/******************************************************************************/
/* EOF */
