/*
 * mml.h --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2014, Maxim Integrated
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
 * Created on: June 10, 2014
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision: 4487 $:  Revision of last commit
 * $Author: jeremy.kongs $:  Author of last commit
 * $Date: 2015-01-14 15:43:29 -0600 (Wed, 14 Jan 2015) $:  Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_H_
#define _MML_H_

/** Global includes */
#include <io.h>
/** Other includes */
//#include <arch/cortex-m3/bitband.h>
/** Local includes */


/* Memories */
#define MML_MEM_ROM_BASE						0x00000000 //!< ROM base address
#define MML_MEM_ROM_SIZE						0x10000 //!< ROM size (64KB)
#define ROM_BASE								MML_MEM_ROM_BASE //!< ROM base address
#define ROM_SIZE								MML_MEM_ROM_SIZE //!< ROM size
#define MML_MEM_FLASH_BASE						0x10000000 //!< Flash base address
#define MML_MEM_FLASH_SIZE						0x80000 //!< Flash size (512Kb)
#define MML_MEM_FLASH_HALF_SIZE					( MML_MEM_FLASH_SIZE / 2 )
#define MML_MEM_FLASH_1ST_HALF_BASE				MML_MEM_FLASH_BASE //!< Flash 1st half base address
#define MML_MEM_FLASH_1ST_HALF_SIZE				MML_MEM_FLASH_HALF_SIZE //!< Flash size (256Kb)
#define MML_MEM_FLASH_2ND_HALF_BASE				( MML_MEM_FLASH_1ST_HALF_BASE + MML_MEM_FLASH_1ST_HALF_SIZE ) //!< Flash 2nd half base address
#define MML_MEM_FLASH_2ND_HALF_SIZE				MML_MEM_FLASH_HALF_SIZE //!< Flash size (256Kb)

#define FLASH_BASE								MML_MEM_FLASH_BASE //!< FLASH base address
#define FLASH_SIZE								MML_MEM_FLASH_SIZE //!< FLASH size

#define MML_MEM_OTP_BASE						0x10100000
#define MML_MEM_OTP_MXIM_BASE					MML_MEM_OTP_BASE
#define MML_MEM_OTP_MXIM_SIZE					0x400
#define MML_MEM_OTP_USER_BASE					0x10101000
#define MML_MEM_OTP_USER_SIZE					0x400
#define MML_MEM_OTP_SIZE						( MML_MEM_OTP_MXIM_SIZE + MML_MEM_OTP_USER_SIZE )
#define MML_MEM_SNVSRAM_BASE					0x20080000
#define MML_MEM_SNVSRAM_SIZE					0x2000

/** Memory mapping of Cortex-M3 Hardware */
#define	SCS_BASE								0xe000e000 /*!< System Control Space Base Address  */
#define	ITM_BASE								0xe0000000 /*!< ITM Base Address                   */
#define	DWT_BASE								0xe0001000 /*!< DWT Base Address                   */
#define	TPI_BASE								0xe0040000 /*!< TPI Base Address                   */
#define	CoreDebug_BASE							0xe000edf0 /*!< Core Debug Base Address            */
#define	SysTick_BASE							( SCS_BASE + 0x0010 ) /*!< SysTick Base Address               */
#define	NVIC_BASE								( SCS_BASE + 0x0100 ) /*!< NVIC Base Address                  */
#define	SCB_BASE								( SCS_BASE + 0x0d00 ) /*!< System Control Block Base Address  */

#define	SCnSCB									( (SCnSCB_Type*)SCS_BASE ) /*!< System control Register not in SCB */
#define	SCB										( (SCB_Type*)SCB_BASE ) /*!< SCB configuration structure           */
#define	SysTick									( (SysTick_Type*)SysTick_BASE ) /*!< SysTick configuration structure       */
#define	NVIC									( (NVIC_Type*)NVIC_BASE ) /*!< NVIC configuration structure          */
#define	ITM										( (ITM_Type*)ITM_BASE ) /*!< ITM configuration structure           */
#define	DWT										( (DWT_Type*)DWT_BASE ) /*!< DWT configuration structure           */
#define	TPI										( (TPI_Type*)TPI_BASE ) /*!< TPI configuration structure           */
#define	CoreDebug								( (CoreDebug_Type*)CoreDebug_BASE ) /*!< Core Debug configuration structure    */

#define	MPU_BASE								( SCS_BASE +  0x0d90) /*!< Memory Protection Unit             */
#define	MPU										( (MPU_Type*)MPU_BASE ) /*!< Memory Protection Unit             */


#ifndef __ASSEMBLY__
/** Convert PA to VA. */
#  define IO_ADDRESS(x)							(volatile unsigned char*)(x)
#else
/** Convert PA to VA. */
#  define IO_ADDRESS(x)							(x)
#endif /* __ASSEMBLY__ */

/* IO Base Addresses */
#define MML_IOBASE								0x40000000 				//!< APB/AHB IO Physical Base Address
#define MML_IOBASE_ADDR							IO_ADDRESS(MML_IOBASE)	//!< APB/AHB IO Virtual Base Address

#define MML_IO_BITBAND_BASE						0x42000000 				        //!< APB/AHB IO Bit-band Physical Base Address
#define MML_IO_BITBAND_BASE_ADDR				IO_ADDRESS(MML_IO_BITBAND_BASE)	//!< APB/AHB IO Bit-band Virtual Base Address

/* IO Offsets */
#define MML_GCR_PORT							0x000000 //! GCR
#define MML_NBB_SIR_PORT						0x000400 //! NBB SIR
#define MML_TEST_PORT							0x000c00 //! TEST
#define MML_CRYPTO_PORT							0x001000 //! Crypto Engine
#define MML_WDT_PORT							0x003000 //! Watchdog
#define MML_SEC_MON_PORT						0x004000 //! Security Monitor
#define MML_AES_KEYS_PORT						0x005000 //! AES Keys
#define MML_BB_SIR_PORT							0x005400 //! BB SIR
#define MML_RTC_PORT							0x006000 //! RTC
#define MML_GPIO0_PORT							0x008000 //! GPIO 0
#define MML_GPIO1_PORT							0x009000 //! GPIO 1
#define MML_GPIO2_PORT							0x00a000 //! GPIO 2
#define MML_I2C_PORT							0x00c000 //! I2C
#define MML_TMR0_PORT							0x010000 //! PWM Timer 0
#define MML_TMR1_PORT							0x011000 //! PWM Timer 1
#define MML_TMR2_PORT							0x012000 //! PWM Timer 2
#define MML_TMR3_PORT							0x013000 //! PWM Timer 3
#define MML_TMR4_PORT							0x014000 //! PWM Timer 4
#define MML_TMR5_PORT							0x015000 //! PWM Timer 5
#define MML_TMR6_PORT							0x016000 //! PWM Timer 6
#define MML_TMR7_PORT							0x017000 //! PWM Timer 7
#define MML_SPI0_PORT							0x018000 //! SPI 0
#define MML_SPI1_PORT							0x019000 //! SPI 1
#define MML_SPI2_PORT							0x01a000 //! SPI 2
#define MML_UART0_PORT							0x020000 //! UART 0
#define MML_UART1_PORT							0x021000 //! UART 1
#define MML_UART2_PORT							0x022000 //! UART 2
#define MML_DMA_PORT							0x028000 //! DMA
#define MML_SFLC_PORT							0x029000 //! Secure Flash Controller
#define MML_ICACHE_PORT							0x02a000 //! Cache Controller
#define MML_MCR_PORT							0x02b000 //! Magnetic Stripe Reader DSP
#define MML_SC_PORT								0x02c000 //! Smart Card
#define MML_MLCD_PORT							0x030000 //! Mono LCD Controller
#define MML_SKBD_PORT							0x032000 //! Secure Keyboard
#define MML_ADC10_PORT							0x034000 //! 10-bit ADC
#define MML_DAC_PORT							0x038000 //! DAC
#define MML_USB_PORT							0x0b0000 //! USB
#define MML_TRNG_PORT							0x0b5000 //! TRNG
#define MML_DAC_MM_PORT							0x0b8000 //! DAC Memory Map
#define MML_TDC_PORT							0x031000 //! TDC Phys


/* IO Addresses */
#define MML_GCR_IOBASE	        				( MML_IOBASE + MML_GCR_PORT )
#define MML_NBB_SIR_IOBASE 						( MML_IOBASE + MML_NBB_SIR_PORT )
#define MML_TEST_IOBASE							( MML_IOBASE + MML_TEST_PORT )
#define MML_CRYPTO_IOBASE 					 	( MML_IOBASE + MML_CRYPTO_PORT )
#define MML_WDT_IOBASE	   					    ( MML_IOBASE + MML_WDT_PORT )
#define MML_SEC_MON_IOBASE						( MML_IOBASE + MML_SEC_MON_PORT )
#define MML_AES_KEYS_IOBASE						( MML_IOBASE + MML_AES_KEYS_PORT )
#define MML_BB_SIR_IOBASE						( MML_IOBASE + MML_BB_SIR_PORT )
#define MML_RTC_IOBASE	  						( MML_IOBASE + MML_RTC_PORT )
#define MML_GPIO0_IOBASE						( MML_IOBASE + MML_GPIO0_PORT )
#define MML_GPIO1_IOBASE						( MML_IOBASE + MML_GPIO1_PORT )
#define MML_GPIO2_IOBASE						( MML_IOBASE + MML_GPIO2_PORT )
#define MML_I2C_IOBASE							( MML_IOBASE + MML_I2C_PORT )
#define MML_TMR0_IOBASE							( MML_IOBASE + MML_TMR0_PORT )
#define MML_TMR1_IOBASE							( MML_IOBASE + MML_TMR1_PORT )
#define MML_TMR2_IOBASE							( MML_IOBASE + MML_TMR2_PORT )
#define MML_TMR3_IOBASE							( MML_IOBASE + MML_TMR3_PORT )
#define MML_TMR4_IOBASE							( MML_IOBASE + MML_TMR4_PORT )
#define MML_TMR5_IOBASE							( MML_IOBASE + MML_TMR5_PORT )
#define MML_TMR6_IOBASE							( MML_IOBASE + MML_TMR6_PORT )
#define MML_TMR7_IOBASE							( MML_IOBASE + MML_TMR7_PORT )
#define MML_SPI0_IOBASE							( MML_IOBASE + MML_SPI0_PORT )
#define MML_SPI1_IOBASE							( MML_IOBASE + MML_SPI1_PORT )
#define MML_SPI2_IOBASE							( MML_IOBASE + MML_SPI2_PORT )
#define MML_UART0_IOBASE						( MML_IOBASE + MML_UART0_PORT )
#define MML_UART1_IOBASE						( MML_IOBASE + MML_UART1_PORT )
#define MML_UART2_IOBASE						( MML_IOBASE + MML_UART2_PORT )
#define MML_DMA_IOBASE	   						( MML_IOBASE + MML_DMA_PORT )
#define MML_SFLC_IOBASE							( MML_IOBASE + MML_SFLC_PORT )
#define MML_ICACHE_IOBASE						( MML_IOBASE + MML_ICACHE_PORT )
#define MML_MCR_IOBASE	  						( MML_IOBASE + MML_MCR_PORT )
#define MML_SC_IOBASE							( MML_IOBASE + MML_SC_PORT )
#define MML_MLCD_IOBASE							( MML_IOBASE + MML_MLCD_PORT )
#define MML_SKBD_IOBASE							( MML_IOBASE + MML_SKBD_PORT )
#define MML_ADC10_IOBASE						( MML_IOBASE + MML_ADC10_PORT )
#define MML_DAC_IOBASE							( MML_IOBASE + MML_DAC_PORT )
#define MML_USB_IOBASE							( MML_IOBASE + MML_USB_PORT )
#define MML_TRNG_IOBASE							( MML_IOBASE + MML_TRNG_PORT )
#define MML_DAC_MM_IOBASE						( MML_IOBASE + MML_DAC_MM_PORT )
#define MML_TDC_IOBASE 						  ( MML_IOBASE + MML_TDC_PORT )

/* System Memory */

#define MML_SYSBASE								0xe0000000 //!< Cortex M3 System Base Address
#define MML_SYSBASE_ADDR						IO_ADDRESS(MML_SYSBASE) //!< Cortex M3 System Base Address

#define MML_NVIC_PORT							0xe000 							//! Interrupt Controller
#define MML_NVIC_IOBASE							( MML_SYSBASE + MML_NVIC_PORT ) //! Interrupt Controller
#define MML_NVIC_IOMEM							IO_ADDRESS(MML_NVIC_IOBASE) //! Interrupt Controller

#define MML_INTC_IOBASE							MML_NVIC_IOBASE //! Interrupt Controller (legacy)
#define MML_INTC_IOMEM							MML_NVIC_IOMEM //! Interrupt Controller (legacy)

#define MML_MPU_PORT							0xed90 //! Memory Protection Unit
#define MML_MPU_IOBASE							( MML_SYSBASE + MML_MPU_PORT )
#define MML_MPU_IOMEM							IO_ADDRESS(MML_MPU_IOBASE)

/** Interrupt Channel Numbers */
#define MML_INTNUM_PFW							0
#define MML_INTNUM_WDT							1
#define MML_INTNUM_USB							2 // Shared between USB and BD SPM function
#define MML_INTNUM_SPM							2
#define MML_INTNUM_RTC							3
#define MML_INTNUM_TRNG							4
#define MML_INTNUM_TMR0							5
#define MML_INTNUM_TMR1							6
#define MML_INTNUM_TMR2							7
#define MML_INTNUM_TMR3							8
#define MML_INTNUM_TMR4							9
#define MML_INTNUM_TMR5							10
#define MML_INTNUM_SC							11 // Shared between SC and BD TX
#define MML_INTNUM_BDTX							11
#define MML_INTNUM_LCD							12 // Shared between mono LCD and TFT
#define MML_INTNUM_I2C							13
#define MML_INTNUM_UART0						14
#define MML_INTNUM_UART1						15
#define MML_INTNUM_SPI0							16
#define MML_INTNUM_SPI1							17
#define MML_INTNUM_SPI2							18
#define MML_INTNUM_SKBD							19 // Shared between KBD and BD RX
#define MML_INTNUM_BDRX							19
#define MML_INTNUM_ADC							20
#define MML_INTNUM_DAC							21
#define MML_INTNUM_MCR							22
#define MML_INTNUM_SFLC							23
#define MML_INTNUM_GPIO0						24
#define MML_INTNUM_GPIO1						25
#define MML_INTNUM_GPIO2						26
#define MML_INTNUM_CRYPTO						27
#define MML_INTNUM_DMA_BASE						28
/** DMA interrupt number with index to select one of the channel */
#define MML_INTNUM_DMA(x)						( MML_INTNUM_DMA_BASE + (x) )
#define MML_INTNUM_DMA0							MML_INTNUM_DMA(0) // 28
#define MML_INTNUM_DMA1							MML_INTNUM_DMA(1) // 29
#define MML_INTNUM_DMA2							MML_INTNUM_DMA(2) // 30
#define MML_INTNUM_DMA3							MML_INTNUM_DMA(3) // 31
/**  */
#define MML_INTNUM_TMR6							32
#define MML_INTNUM_TMR7							33
#define MML_INTNUM_UART2						34


/* System Frequency */
#ifdef _FPGA_WORKAROUND_
#define MML_PLLO_FREQ							40000000 // emulator
#else
#define MML_PLLO_FREQ							60000000 // real silicon
#endif /* _FPGA_WORKAROUND_ */

#define MML_SYS_FREQ							MML_PLLO_FREQ
#define MML_APB_FREQ							( MML_SYS_FREQ / 2 )

#endif /* MML_H_ */

/******************************************************************************/
/* EOF */
