/*============================================================================
 *
 *      mml_sc_counter.h
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
#ifndef _MML_SCCOUTER_H_
#define _MML_SCCOUTER_H_


/*_____ M A C R O S ________________________________________________________*/
//Normaly is possible TA1=19 but TA1=95 is normaly 
#define Fi_MIN                                  512
#define Di_MAX                                  16
#define MAX_SPEED_Fi_Di							Fi_MIN/Di_MAX

#define ISO_40000_CYCLE							108 //((unsigned int)38316/(unsigned int)372)  //107 etu   
  
//#define ISO_40000_CYCLE                 ((unsigned int)40748/(unsigned int)372) //335   
#define ISO_42000_CYCLE							((unsigned int)43500/(unsigned int)372) //335   

#define ISO_9600_ETU							10200//9600+480+120 //9600+480 accept 9600+480+240 refuse
                                                           
#define WAIT_SILENCE_LINE						400 //960max  wait silence line affer error parity
#define MAX_SIZE_ATR							36
#define MIN_BGT_T1								24// minimun bgt 22 etu

/* Bug fix #1139 */
#define BWT_OFFSET_ERROR						1
/*  */

#define K_SCS_WT_INITIAL_WT						0
#define K_SCS_WT_WORK_WT						1
#define K_SCS_WT_CHARACTER_WT					2
#define K_SCS_WT_BLOCK_WT						3
#define K_SCS_WT_START_ATR						4
#define K_SCS_WT_UNKNOWN_WT						0xff



void mml_sc_counter_etu_set(void);
void mml_sc_counter_clk_start_man(unsigned int count);
void mml_sc_counter_clk_start(unsigned int count);
void mml_sc_counter_start (unsigned int etu);
void mml_sc_counter_clk_halt_man(void);
void mml_sc_counter_clk_halt(void);
void mml_sc_counter_halt(void);
#define mml_sc_counter_halt_bgt				mml_sc_counter_halt
void mml_sc_counter_set_gt(void);
void mml_sc_counter_start_bgt(void);
int mml_sc_calculate_wt( unsigned int timer);
unsigned int mml_sc_counter_get_bgt(void);


extern const unsigned short mml_sc_ftab[16];
extern const unsigned char mml_sc_dtab[16];

#endif  /* _MML_SCCOUTER_H_ */

