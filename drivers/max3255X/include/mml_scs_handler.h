/*============================================================================
 *
 *      mml_sc_handler.h
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
#ifndef _mml_SCHANDLER_H_ 
#define _mml_SCHANDLER_H_

int mml_sc_handler_parity(void);
int mml_sc_handler_tx(void);
int mml_sc_handler_empty(void);
int mml_sc_handler_error(void);
int mml_sc_handler_atr_error(void);

void mml_sc_isr(void);
void mml_sc_isr0(void);
void mml_sc_isr1(void);
void mml_sc_isr2(void);





    
#endif /* _mml_SCHANDLER_H_ */
