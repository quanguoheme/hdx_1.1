/*
 * mml_sc_dev.h --
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
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* [INTERNAL] ------------------------------------------------------------------
 * Created on: Jun 29, 2012
 * Author: Yann G. <yann.gaude@maxim-ic.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_SC_DEV_H
#define _MML_SC_DEV_H

#if __STDC_VERSION__ < 199901L
#ifndef inline
#define inline __inline
#endif
#endif

/** Global includes */
#include <errors.h>
/** Other includes */
#include <mml_io.h>
/** Local includes */
#include <mml_sc_regs.h>
#include <mml_sc.h>

/*_____ I N C L U D E S ____________________________________________________*/
#define MSK_SCICR_VCARD0						0x10
#define MSK_SCICR_VCARD1						0x20
#define SC_0_VOLT								0x00 
#define SC_1_8_VOLT								(unsigned char)MSK_SCICR_VCARD0 
#define SC_3_VOLT								(unsigned char)MSK_SCICR_VCARD1 
#define SC_5_VOLT								(unsigned char)(MSK_SCICR_VCARD1|MSK_SCICR_VCARD0)

#define	ISR_OFFSET_PARIS						0
#define	ISR_OFFSET_WTIS							1
#define	ISR_OFFSET_CTIS							2
#define	ISR_OFFSET_TCIS							3
#define	ISR_OFFSET_RXEIS						4
#define	ISR_OFFSET_RXTIS						5
#define	ISR_OFFSET_RXFIS						6
#define	ISR_OFFSET_TXEIS						7
#define	ISR_OFFSET_TXTIS						8
#define	ISR_OFFSET_PDLIS						10

#define BIT_ErrParity							(1 << ISR_OFFSET_PARIS)
#define BIT_WaitTimeout							(1 << ISR_OFFSET_WTIS)
#define	BIT_Counter								(1 << ISR_OFFSET_CTIS)
#define	BIT_TCIS								(1 << ISR_OFFSET_TCIS)
#define BIT_RxNotEmpty							(1 << ISR_OFFSET_RXEIS)
#define BIT_RxThreshold							(1 << ISR_OFFSET_RXTIS)
#define BIT_RxFull								(1 << ISR_OFFSET_RXFIS)
#define BIT_TxEmpty								(1 << ISR_OFFSET_TXEIS)
#define BIT_Tx									(1 << ISR_OFFSET_TCIS)
#define BIT_TxThreshold							(1 << ISR_OFFSET_TXTIS)
#define BIT_PresenceCard						(1 << ISR_OFFSET_PDLIS)

#define MML_SC_EVENT_EMPTY						0
#define MML_SC_EVENT_TX						1
#define MML_SC_EVENT_RX						2
#define MML_SC_EVENT_TIMEOUT					3

#define	MML_SC_EVENT_PARITY					(0x10 + ISR_OFFSET_PARIS)
#define	MML_SC_EVENT_WT_TIMEOUT				(0x10 + ISR_OFFSET_WTIS)
#define	MML_SC_EVENT_COUNTER					(0x10 + ISR_OFFSET_CTIS)
#define	MML_SC_EVENT_RX_NOT_EMPTY				(0x10 + ISR_OFFSET_RXEIS)
#define	MML_SC_EVENT_RX_THRESHOLD				(0x10 + ISR_OFFSET_RXTIS)
#define	MML_SC_EVENT_RX_FULL					(0x10 + ISR_OFFSET_RXFIS)
#define	MML_SC_EVENT_TX_EMPTY					(0x10 + ISR_OFFSET_TXEIS)
#define	MML_SC_EVENT_TX_THRESHOLD				(0x10 + ISR_OFFSET_TXTIS)
#define	MML_SC_EVENT_PRESENCE_CARD				(0x10 + ISR_OFFSET_PDLIS)

#define MML_SCVSR_COUNT						6

//typedef int (*__p_ioctl)( int cmd, void *param );
typedef void (*__p_handlers)(  int event );

typedef struct mml_sc_dev_s
{
    /** Speed communication (parameter TA1 of ISO 7816-3) */
    unsigned char FIDI;    
    /** Extra Guard Time (parameter TC1 or N of ISO 7816-3) */
    unsigned char EGT;
    /** 
     * If protocol T=0 is selected, the parameter indicates the Waiting 
     * Integer (parameter TC2 of ISO 7816-3 - the default value is 10) 
     * If the protocol T=1 is selected, the parameter indicates the 
     * Block and Character Waiting Time Integer (parameter TB3 of ISO 7816-3) 
     */
    unsigned char WI;   
     /** If the protocol T=1 is selected, the parameter indicates the 
      * Waiting Time Extention (the default value is 1). */
    unsigned char WTX;    
    /** If the protocol T=1 is selected, the parameter indicates the computing 
     * mode for EDC : #MML_SCEDC_LRC or #MML_SCEDC_CRC (The default value
     * is an LRC) */ 
    unsigned char EDC;           
    /** The parameter indicates the selected protocol : 
     * #MML_SCPROTOCOL_T0 or #MML_SCPROTOCOL_T1 */
    unsigned char protocol;
    /** The power supply value */              
    unsigned char power;                   
	/* Character Waiting Time Integer */
	unsigned char	CWI;
	/* Block Waiting Time Integer */
	unsigned char	BWI;
	/* Boolean signaling not to overwrite BWT */
	/* false : not te be erased */
	/* true : can be erased */
	unsigned char	ucEraseBWT;
	/* Boolean signaling not to overwrite CWT */
	/* false : not te be erased */
	/* true : can be erased */
	unsigned char	ucEraseCWT;
    /* SCIB number to associate with the device number */
    /* from N_MML_SC_DEV0 to MML_SCSCIB_2, or MML_SCNO_SCIB */
	//unsigned char	ucSCIB;
    /* Interrupt Vector number */
   // unsigned short vnum;
    /** State machine */
    unsigned short state;
    /** SmartCard Clock */
  //  unsigned int uiCardClock;
    /* Current status */       
    int status;
    /* Current error */
    int last_err;                   
    /* Buffer for received data. */
    unsigned char* rx_buf /*__attribute__ ((aligned (4)))*/;
    /* Buffer for data to send */   
    unsigned char* tx_buf /*__attribute__ ((aligned (4)))*/;
    /** timeout inter caractere in T=1 WARNING in T=0 Cwt=Bwt*/
    unsigned int cwt;             
    /** timeout inter block in T=1 WARNING in T=0 Cwt=Bwt*/
    unsigned int bwt;
    /* Base registers address */
    mml_sc_regs_t *reg_sc;
    /* Number of allocated bytes for data. */
    unsigned int rx_alloc /*__attribute__ ((aligned (4)))*/;
    /* Number of available bytes for data. */
    unsigned int *rx_size /*__attribute__ ((aligned (4)))*/;
    /* Number of bytes expected for data. */
    unsigned int rx_expected /*__attribute__ ((aligned (4)))*/;
    /* Number of data bytes sent */
    unsigned int tx_size /*__attribute__ ((aligned (4)))*/;
    /* Bug correction #794 */
	unsigned int uiSecurityLoop;
    /* Number of data bytes to send */
    /* due to SCIB's behaviour, the tx_expected field has to be doubled :
     * - one for TxEmptyBuffer's IT
     * - one for TxTransmitted's IT */
    unsigned int tx_expected_txempty;  
    unsigned int tx_expected_tx;
    /* Entry point to the analog interface */   
//    __p_ioctl ioctl;
    /** Interrupt Virtual Service Routine */
    void (*vsr)(int event );
    /** Interrupt user service routines */
    __p_handlers handlers[ MML_SC_EVENT_COUNT ]; //FIXME : MML_SC_EVENT_COUNT
    /** Interrupt External Service Routine */
    void (*esr)(int event );
} volatile mml_sc_dev_t __attribute__ ((aligned(4)));


//const unsigned short mml_sc_ftab[16] =
// { 372, 372, 558, 744, 1116, 1488, 1860, 0,
//   0, 512, 768, 1024, 1536, 2048, 0, 0};

//const unsigned char mml_sc_dtab[16] = {
//    0, 1, 2, 4, 8, 16, 32, 64, 12, 20, 0, 0, 0, 0, 0, 0};


extern mml_sc_dev_t *dev;

/* Replace memset */
/* Note here, because it's just the simplest rewriting of C standard function,
 * and because this function is private to this module,no check on pointers is done */
inline static void mml_sc_memset(unsigned char *ptr, unsigned char value, unsigned int length)
{
	register unsigned int	i;

	for( i = 0;i < length;i++ )
	{
		ptr[i] = value;
	}
	return;
}
 
#define MML_SCSTATUS_INITIALIZED			0x01
#define MML_SCSTATUS_OPERATION_PENDING	0x08
#define MML_SCSTATUS_CONV_INVERSE			0x100
#define MML_SCSTATUS_PARITY_ERROR			0x200


/**  */
#define MML_SC_SIZE_AREA					0x0f
/**  */
#define MSK_SIGNAL_VANA						0x03
/**  */
#define MSK_SIGNAL_CRD_SWITCH				0x04
/**  */
#define MSK_SIGNAL_VCARD_OK					0x08
/**  */
#define MSK_SIGNAL_ICC_OVER					0x10
/**  */
#define MSK_SIGNAL_CLKCH					0x20

/** Macros ********************************************************************/
void mml_sc_dev_set_conv(void);

#define	mml_sc_dev_flush_tx()			(dev->reg_sc->sc_cr |= MML_SC_CR_TXFLUSH_MASK)
#define	mml_sc_dev_flush_rx()			(dev->reg_sc->sc_cr |= MML_SC_CR_RXFLUSH_MASK)
#define	mml_sc_dev_flush_all()			mml_sc_dev_flush_tx(); \
											mml_sc_dev_flush_rx()

#define mml_sc_dev_disable_char_rep()	(dev->reg_sc->sc_cr &= ~MML_SC_CR_CREP_MASK)
#define mml_sc_dev_enable_char_rep()	(dev->reg_sc->sc_cr |= MML_SC_CR_CREP_MASK)
#define mml_sc_dev_disable_parity_error()	mml_sc_dev_disable_char_rep()

#define mml_sc_dev_disable_uart()		(dev->reg_sc->sc_cr &= ~MML_SC_CR_UART_MASK)
#define mml_sc_dev_enable_uart()		(dev->reg_sc->sc_cr |= MML_SC_CR_UART_MASK)
/*  */
#define mml_sc_dev_clear_parity_error()	(dev->reg_sc->sc_sr &= ~MML_SC_SR_PAR_MASK)
#define mml_sc_dev_get_parity_error()	(dev->reg_sc->sc_sr & MML_SC_SR_PAR_MASK)

#define mml_sc_dev_clear_prc_error()	(dev->reg_sc->sc_sr &= ~MML_SC_SR_PRC_MASK)
#define mml_sc_dev_get_prc_error()	(dev->reg_sc->sc_sr & MML_SC_SR_PRC_MASK)

#define mml_sc_dev_set_etu_comp()         (dev->reg_sc->sc_etur |= MML_SC_ETUR_COMP_MASK)
#define mml_sc_dev_clear_etu_comp()       (dev->reg_sc->sc_etur &= ~MML_SC_ETUR_COMP_MASK)


#define mml_sc_dev_set_start()         (dev->reg_sc->sc_cr |= MML_SC_CR_START_MASK)
#define mml_sc_dev_clear_start()       (dev->reg_sc->sc_cr &= ~MML_SC_CR_START_MASK)
#define mml_sc_dev_get_start()       (dev->reg_sc->sc_cr & MML_SC_CR_START_MASK)

#define mml_sc_dev_set_power(val)         (dev->reg_sc->sc_pnr |= (MML_SC_PNR_VCCSEL_MASK & (val)))


#define mml_sc_dev_get_card_pres()     (dev->reg_sc->sc_sr & MML_SC_SR_PRES_MASK)
#define mml_sc_dev_get_card_activ()     (dev->reg_sc->sc_sr & MML_SC_SR_ACTIV_MASK)





#define mml_sc_dev_set_card_clk_sel()     (dev->reg_sc->sc_pnr |= MML_SC_PNR_CLKSEL_MASK)
#define mml_sc_dev_clear_card_clk_sel()   (dev->reg_sc->sc_pnr &= ~MML_SC_PNR_CLKSEL_MASK)
#define mml_sc_icc_set_rst()              (dev->reg_sc->sc_pnr |= MML_SC_PNR_CRDRST_MASK)
#define mml_sc_icc_clear_rst()            (dev->reg_sc->sc_pnr &= ~MML_SC_PNR_CRDRST_MASK)
#define mml_sc_icc_get_rst()              (dev->reg_sc->sc_pnr & MML_SC_PNR_CRDRST_MASK)
#define mml_sc_icc_set_clk()              (dev->reg_sc->sc_pnr |= MML_SC_PNR_CRDCLK_MASK)
#define mml_sc_icc_clear_clk()            (dev->reg_sc->sc_pnr &= ~MML_SC_PNR_CRDCLK_MASK)
#define mml_sc_icc_get_clk()              (dev->reg_sc->sc_pnr & MML_SC_PNR_CRDCLK_MASK)
#define mml_sc_icc_set_io()               (dev->reg_sc->sc_pnr |= MML_SC_PNR_CRDIO_MASK)
#define mml_sc_icc_clear_io()             (dev->reg_sc->sc_pnr &= ~MML_SC_PNR_CRDIO_MASK)
#define mml_sc_icc_get_io()               (dev->reg_sc->sc_pnr & MML_SC_PNR_CRDIO_MASK)
#define mml_sc_icc_set_c4()               (dev->reg_sc->sc_pnr |= MML_SC_PNR_CRDC4_MASK)
#define mml_sc_icc_clear_c4()             (dev->reg_sc->sc_pnr &= ~MML_SC_PNR_CRDC4_MASK)
#define mml_sc_icc_get_c4()               (dev->reg_sc->sc_pnr & MML_SC_PNR_CRDC4_MASK)
#define mml_sc_icc_set_c8()               (dev->reg_sc->sc_pnr |= MML_SC_PNR_CRDC8_MASK)
#define mml_sc_icc_clear_c8()             (dev->reg_sc->sc_pnr &= ~MML_SC_PNR_CRDC8_MASK)
#define mml_sc_icc_get_c8()               (dev->reg_sc->sc_pnr & MML_SC_PNR_CRDC8_MASK)

/** _DBG_YG_ */
#define mml_sc_intr_mask_all()			(dev->reg_sc->sc_ier &= (~(MML_SC_IER_PARIE_MASK |\
																	MML_SC_IER_WTIE_MASK |\
																	MML_SC_IER_CTIE_MASK |\
																	MML_SC_IER_TCIE_MASK |\
																	MML_SC_IER_RXEIE_MASK |\
																	MML_SC_IER_RXTIE_MASK |\
																	MML_SC_IER_RXFIE_MASK |\
																	MML_SC_IER_TXEIE_MASK |\
																	MML_SC_IER_TXTIE_MASK)))

#define scib_clear_all_status()            (dev->reg_sc->sc_sr &= ~MML_SC_SR_PAR_MASK)
/*  */
#define mml_sc_intr_mask_empty_buf()      (dev->reg_sc->sc_ier &= ~MML_SC_IER_TXEIE_MASK)
#define mml_sc_intr_unmask_empty_buf()    (dev->reg_sc->sc_ier |= MML_SC_IER_TXEIE_MASK)
#define mml_sc_intr_mask_timeout()        (dev->reg_sc->sc_ier &= ~MML_SC_IER_WTIE_MASK)
#define mml_sc_intr_unmask_timeout()      (dev->reg_sc->sc_ier |= MML_SC_IER_WTIE_MASK)

/** _DBG_YG_ */
#define mml_sc_intr_mask_clk_counter()	    (dev->reg_sc->sc_ier &= ~MML_SC_IER_CTIE_MASK)
#define mml_sc_intr_unmask_clk_counter()	(dev->reg_sc->sc_ier |= MML_SC_IER_CTIE_MASK)
/*  */

/** _DBG_YG_ */
#define mml_sc_intr_mask_rx()				(dev->reg_sc->sc_ier &= ~MML_SC_IER_RXTIE_MASK)
#define mml_sc_intr_unmask_rx()				(dev->reg_sc->sc_ier |= MML_SC_IER_RXTIE_MASK)
/*  */

#define mml_sc_intr_mask_tx()              (dev->reg_sc->sc_ier &= ~MML_SC_ISR_TCIS_MASK)
#define mml_sc_intr_unmask_tx()            (dev->reg_sc->sc_ier |= MML_SC_ISR_TCIS_MASK)
#define mml_sc_intr_mask_parity_error()    (dev->reg_sc->sc_ier &= ~MML_SC_IER_PARIE_MASK)
#define mml_sc_intr_unmask_parity_error()  (dev->reg_sc->sc_ier |= MML_SC_IER_PARIE_MASK)

#define mml_sc_intr_mask_presence_card()    (dev->reg_sc->sc_ier &= ~MML_SC_IER_PDLIE_MASK)
#define mml_sc_intr_unmask_presence_card()  (dev->reg_sc->sc_ier |= MML_SC_IER_PDLIE_MASK)

/** _DBG_YG_ */
/** Write byte */
#define mml_sc_dev_outb(y)               (dev->reg_sc->sc_txr = y)
#define mml_sc_dev_inb()                  (dev->reg_sc->sc_rxr)
/**  */


#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* WT counter */
#define mml_sc_dev_wt_stop()				(dev->reg_sc->sc_cr &= ~MML_SC_CR_WTEN_MASK)
#define mml_sc_dev_wt_run()				(dev->reg_sc->sc_cr |= MML_SC_CR_WTEN_MASK)
void mml_sc_dev_wt_load(unsigned long long timeout);

/** Clock conuter */
#define mml_sc_dev_counter_clk_stop()			(dev->reg_sc->sc_cr &= ~MML_SC_CR_CCEN_MASK)
#define mml_sc_dev_counter_clk_run()			(dev->reg_sc->sc_cr |= MML_SC_CR_CCEN_MASK)
#define mml_sc_dev_counter_clk_load(count)	(dev->reg_sc->sc_ccr = count)
//#define mml_sc_dev_counter_clk_load_man(dev, count)	(dev->reg_sc->sc_ccr = count/*(count | MML_SC_CCR_MAN)*/)
#define mml_sc_dev_counter_clk_load_man(count)	(dev->reg_sc->sc_ccr = (count | MML_SC_CCR_MAN_MASK))
#define	mml_sc_dev_counter_clk_set_man()		(dev->reg_sc->sc_ccr |= MML_SC_CCR_MAN_MASK)
#define	mml_sc_dev_counter_clk_clr_man()		(dev->reg_sc->sc_ccr &= (~(MML_SC_CCR_MAN_MASK)))



int mml_sc_dev_send_data(unsigned int length);
int mml_sc_dev_send_data_T1(void);
int mml_sc_dev_receive_data(int length);
int mml_sc_dev_receive_data_T1(void);
int mml_sc_dev_cancel_transfer(void);
void mml_sc_dev_etu_load( unsigned int etu);
void mml_sc_dev_wt_load( unsigned long long etu);
void mml_sc_dev_gt_load( unsigned int etu);
void mml_sc_dev_set_conv(void);

void mml_sc_dev_call_service( int event);
void mml_sc_dev_call_handler( int event);
int mml_sc_dev_selected_device(void);
int mml_sc_dev_selection(int cmd);


void delay_micro(int us_to_delay);

#endif /* _MML_SC_DEV_H_ */

/******************************************************************************/
/* EOF */
