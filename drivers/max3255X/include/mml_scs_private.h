/*============================================================================
 *
 *      mml_sc_private.h
 *
 *      
 *
 *==========================================================================*/
/*============================================================================
 * 
 * Copyright (c) 2002-2010 Maxim Integrated Products.
 * All Rights Reserved.
 *
 * This software is the confidential and proprietary information of
 * Maxim Integrated Products ("Confidential Information"). You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered
 * into with Maxim Integrated Products.
 *
 * Maxim Integrated Products makes no representations or warranties about the suitability of 
 * the software, either express or implied, including but not limited to 
 * the implied warranties of merchantability, fitness for a particular purpose, 
 * or non-infrigement. Maxim Integrated Products shall not be liable for any damages suffered 
 * by licensee as the result of using, modifying or distributing this software 
 * or its derivatives.
 * 
 *==========================================================================*/
/*============================================================================
 * 
 * Author(s): Maxim Integrated Products
 * Contributors:  
 * Date:          
 * Purpose:       
 * Description:
 * 
 *==========================================================================*/
#ifndef MML_SCPRIVATE_H_
#define MML_SCPRIVATE_H_

/* defines =================================================================*/
#define OFFSET_INS										1
#define OFFSET_Lc										4

/* enums and structures ====================================================*/
#define	STATE_T0_IN_CHECK_PROC_BYTE						0x10
#define	STATE_T0_IN_CHECK_SW1							0x11
#define	STATE_T0_IN_CHECK_SW2							0x12
#define	STATE_T0_IN_SEND_REMAINING_DATA					0x13
#define	STATE_T0_IN_SEND_ONE_DATA						0x14
#define	STATE_T0_IN_CHECK_BGT2							0x15
#define	STATE_T0_IN_DONE								0x16
	/* enum for T=0 out exchange */
#define	STATE_T0_OUT_CHECK_PROC_BYTE					0x20
#define	STATE_T0_OUT_CHECK_SW1							0x21
#define	STATE_T0_OUT_CHECK_SW2							0x22
#define	STATE_T0_OUT_CHECK_REMAINING_DATA				0x23
#define	STATE_T0_OUT_CHECK_ONE_DATA						0x24
#define	STATE_T0_OUT_CHECK_BGT2							0x25
#define	STATE_T0_OUT_DONE								0x26
	/* enum for T=1 block exchange */
#define	STATE_T1_PROLOGUE								0x40
#define	STATE_T1_REMAINING_DATA							0x41
#define	STATE_T1_CHECK_BGT2								0x42
#define	STATE_T1_DONE									0x43
#define	STATE_T1_PRE_BGT								0x44
	/* enum for PPS exchange */
#define	STATE_PPS_REQUEST								0x80
#define	STATE_PPS_RESPONSE_TS							0x81
#define	STATE_PPS_RESPONSE_T0							0x82
#define	STATE_PPS_RESPONSE								0x83
#define	STATE_PPS_CHECK_BGT								0x84
#define	STATE_PPS_DONE									0x85
	/* enum for Col/Warm Reset */
#define	STATE_WAIT_BGT									0
#define	STATE_WAIT_40000_CYCLES							1
#define	STATE_WAIT_40000_CYCLES_BIS						2
#define	ATR_STATE_CHECK_TS_BYTE							3
#define	ATR_STATE_CHECK_T0_BYTE							4
#define	ATR_STATE_CHECK_HIST_BYTES						5
#define	ATR_STATE_CHECK_INTERFACE_BYTES					6
#define	ATR_STATE_CHECK_TCK_BYTE						7
#define	ATR_STATE_CHECK_EXTRA_BYTES						8
#define	ATR_STATE_DONE									9
#define	ATR_STATE_ABORT									10
	/* Default value for the power up safety counter */
#define	MML_SCDEFAULT_SEC_LOOP_VALUE					500

#endif /*MML_SCPRIVATE_H_*/
