/*
 * main.c --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2012-2015, Maxim Integrated Products
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Maxim Integrated Products nor the
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
 * Created on: Feb 06, 2012
 * Author: Yann G. (yann.gaude@maxim-ic.com)
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: 2167     $:  Revision of last commit
 * $Author:: yann.gaude $:  Author of last commit
 * $Date:: 2011-09-29 1#$:  Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
#include <cobra_defines.h>
#include <cobra_macros.h>
#include <cobra_functions.h>
#include <mml.h>
#include <mml_gcr.h>
#include <mml_intc.h>
#include <mml_uart.h>
#include <mml_gpio.h>
/** Local includes */
#include <uart_config.h>
#include <printf_lite.h>
#include <private.h>

/* smartcard specific */
#include "smartcard_api.h"
#include "sc_errors.h"
#include "sc_types.h"

#define INTER_APDU_DELAY     500 /*500ms between each APDU request*/

#define CARD_SLOT_0
//#define CARD_SLOT_1

//define REGRESS_APDU_EXCHANGE to perform APDU exchanges with both the slot with out power down
//#define REGRESS_APDU_EXCHANGE

#define EMV_LOOPBACK
static uint8_t  command0[256], command1[256];
static uint8_t  response0[258], response1[256]; /*256 + status bytes*/

ActivationParams_t ActivationParams = {
    .IccVCC = VCC_5V,
    .IccResetDuration = 112,    /* 41664 clock cycles*/
    .IccATR_Timeout   = 20160,  /* 20160 etus*/
    .IccTS_Timeout    = 114,    /* 42408 clock cycles*/
    .IccWarmReset     = 0,
};

const uint8_t selectFile[] = {
    0x00, 0xA4, 0x04, 0x00,         /*select df file */
    14,                             /* payload length (Lc byte) */
    '1', 'P', 'A', 'Y', '.', 'S', 'Y', 'S', '.', 'D', 'D', 'F', '0', '1',
    0x00,                           /*answer length (Le byte) */
};

/******************************************************************************/
int main(void)
{
    /** Return value */
    int   result = COMMON_ERR_UNKNOWN;
	  mml_uart_config_t 	uart_conf;
    /** Local variables list */
    /* for smartcard tests */
    uint32_t    status = 0;
    uint32_t    len1 =0, len2 = 0;
    IccReturn_t retval = ICC_OK;
    volatile mml_gcr_regs_t *reg_gcr = (volatile mml_gcr_regs_t*)MML_GCR_IOBASE;

	  /** Set the system frequency */
		mml_set_system_divider(MML_GCR_DIV_1);
		/** Initialize UATR0 port with default configurations */
		uart_conf.baudrate = K_LITE_UART0_DEFAULT_BAUDRATE;
		uart_conf.data_bits = MML_UART_DATA_TRANSFER_SIZE_8_BITS;
		uart_conf.flwctrl = MML_UART_HW_FLOW_CTL_DISABLE;
		uart_conf.parity = MML_UART_PARITY_NONE;
		uart_conf.parity_mode = MML_UART_PARITY_MODE_ONES;
		uart_conf.rts_ctl = MML_UART_RTS_IO_LEVEL_LOW;
		uart_conf.stop_bits = MML_UART_STOPBITS_ONE;
		/** Dummy IRQ function for UART0 */
		uart_conf.handler = (mml_uart_handler_t)main;
		result = mml_uart_init(MML_UART_DEV0, uart_conf);
		if ( result )
		{
			/** Oops, I did it again ... */
			goto main_out;
		}

    /*enable the interrupts*/
    cpsie ();
			mml_gpio_config_t config;
			  config.gpio_direction = MML_GPIO_DIR_OUT;
				config.gpio_function = MML_GPIO_NORMAL_FUNCTION;
				config.gpio_intr_mode = MML_GPIO_NORMAL_FUNCTION;
				config.gpio_intr_polarity = MML_GPIO_INT_POL_RAISING;
				config.gpio_pad_config = MML_GPIO_PAD_NORMAL;
				
				result = mml_gpio_init(MML_GPIO_DEV0, 24, 1, config);
				result = mml_gpio_write_bit_pattern(MML_GPIO_DEV0, 24, 1, 1);
				
				result = mml_gpio_init(MML_GPIO_DEV0, 26, 1, config);
				result = mml_gpio_write_bit_pattern(MML_GPIO_DEV0, 26, 1, 0);
				
								
				
#ifdef CARD_SLOT_0
    /* open slot 0 */
    SCAPI_open(0);
#endif

#ifdef CARD_SLOT_1
    /* open slot 1 */
    SCAPI_open(1);
#endif

#ifdef CARD_SLOT_0
    /* set the card frequency */
    status = 4000000; /*4MHz*/
    retval = SCAPI_ioctl(0, IOCTL_SET_CLOCK_FREQ, &status);
		lite_printf("\n return value of clock set in card slot 0:%d",retval);
    if (ICC_OK != retval) {
        goto power_off;
    }
#endif

#ifdef CARD_SLOT_1
    /* set the card frequency */
    status = 4000000; /*4MHz*/
    retval = SCAPI_ioctl(1, IOCTL_SET_CLOCK_FREQ, &status);
		lite_printf("\n return value of clock set in card slot 1:%d",retval);
    if (ICC_OK != retval) {
        goto power_off;
    }
#endif
		
    while( 1 )
    {
#ifdef CARD_SLOT_0			
        /* check if a card is present and NOT powered */
        retval = SCAPI_ioctl(0, IOCTL_GET_CARD_STATE, &status);
			  lite_printf("\n return value of get card state card slot 0:%d, status :%d",retval, status);
        if ((ICC_OK != retval) || (ICC_ERR_PRESENT_INACTIVE != (IccReturn_t)status) ) {
            goto power_off;
        }
#endif

#ifdef CARD_SLOT_1			
        /* check if a card is present and NOT powered */
        retval = SCAPI_ioctl(1, IOCTL_GET_CARD_STATE, &status);
			  lite_printf("\n return value of get card state in card slot 1:%d, status :%d",retval, status);
        if ((ICC_OK != retval) || (ICC_ERR_PRESENT_INACTIVE != (IccReturn_t)status) ) {
            goto power_off;
        }
#endif

#ifdef CARD_SLOT_0			
        /*restore card parameters (EMV Mode and initial params) */
        /* set the ATR timings  + card voltage */
        retval = SCAPI_ioctl(0, IOCTL_SET_INITPARAMS, (void *)&ActivationParams);
			  lite_printf("\n return value of set init params  in card slot 0:%d",retval);
        if (ICC_OK != retval) {
            goto power_off;
        }
#endif

#ifdef CARD_SLOT_1			
        /*restore card parameters (EMV Mode and initial params) */
        /* set the ATR timings  + card voltage */
        retval = SCAPI_ioctl(1, IOCTL_SET_INITPARAMS, (void *)&ActivationParams);
			  lite_printf("\n return value of set init params in card slot 1:%d",retval);
        if (ICC_OK != retval) {
            goto power_off;
        }
#endif
				
#ifdef CARD_SLOT_0
        /*
         * Switch the stack to EMV mode
         * This must be done each time you closed a card session
         */
        status = 1;
        retval = SCAPI_ioctl(0, IOCTL_SET_EMVMODE, &status);
			  lite_printf("\n return value of set emv mode in card slot 0:%d",retval);
        if (ICC_OK != retval) {
            goto power_off;
        }
#endif

#ifdef CARD_SLOT_1
        /*
         * Switch the stack to EMV mode
         * This must be done each time you closed a card session
         */
        status = 1;
        retval = SCAPI_ioctl(1, IOCTL_SET_EMVMODE, &status);
			  lite_printf("\n return value of set emv mode in card slot 1:%d",retval);
        if (ICC_OK != retval) {
            goto power_off;
        }
#endif

#ifdef CARD_SLOT_0
				lite_printf("\n Going to power up the card 0");
        status = POWER_UP;
        /*power up the card */
        ActivationParams.IccWarmReset = bFALSE;
        retval = SCAPI_ioctl(0, IOCTL_POWER_CARD, &status);
				lite_printf("\n return value of POWER_UP : %d",retval);

        if (ICC_OK != retval) {
            goto power_off;
        }
#endif

#ifdef CARD_SLOT_1
				lite_printf("\n Going to power up the card 1");
        status = POWER_UP;
        /*power up the card */
        ActivationParams.IccWarmReset = bFALSE;
        retval = SCAPI_ioctl(1, IOCTL_POWER_CARD, &status);
				lite_printf("\n return value of POWER_UP : %d",retval);

        if (ICC_OK != retval) {
            goto power_off;
        }
#endif				

#ifdef REGRESS_APDU_EXCHANGE
				while(1)
				{
#endif				
        memset(response0, 0, sizeof(response0));
        memset(command0, 0, sizeof(command0));
			
        memset(response1, 0, sizeof(response1));
        memset(command1, 0, sizeof(command1));

        /* Default APDU Command = select PSE */
        memcpy(command0, selectFile, sizeof(selectFile));
        memcpy(command1, selectFile, sizeof(selectFile));
        len1 = len2 = sizeof(selectFile);

#ifdef CARD_SLOT_0				
				lite_printf("\n Going to Perform APDU Exchanges in card slot 0...");

#ifdef EMV_LOOPBACK				
        while (1) 
#endif					
				{
                /* Process a card exchange :
                 * send the command and get the result into the
                 * stack working buffer
                 */
                retval = SCAPI_write(0, command0, len1);
                if (ICC_OK != retval) {
									
                    goto end_of_apdu_exchange;
                }

                /* read the answer from the stack working buffer */
                status = sizeof(response0);
                retval = SCAPI_read(0, response0, &status);
                if (retval != ICC_OK) {
                    goto end_of_apdu_exchange;
                }

                /*
                 * Per the lab request, we must wait at least 200ms between each
                 * APDU exchange in order to let the test tool collect the data
                 */
                Delay(INTER_APDU_DELAY);
								
								lite_printf("\n Card 0 APDU exchange. the status = %d", status);

                /* Prepare next APDU */
                if (status < 6) {
                    /* Default APDU Command = select PSE */
                    memcpy(command0, selectFile, sizeof(selectFile));
                    len1 = sizeof(selectFile);
                } else if(response0[1] == 0x70) {
                    /* Abort loopback */
                    goto end_of_apdu_exchange;
                } else {
                    /* Loopback */
                    len1 = status -2;
                    memcpy(command0, response0, len1);
                }
			}
#endif
			
#ifdef CARD_SLOT_1				
				lite_printf("\n Going to Perform APDU Exchanges in card slot 1...");

#ifdef EMV_LOOPBACK				
        while (1) 
#endif					
				{
					
								/* Process a card exchange :
                 * send the command and get the result into the
                 * stack working buffer
                 */
                retval = SCAPI_write(1, command1, len2);
                if (ICC_OK != retval) {
										lite_printf("\n Card 1 APDU exchange. the retval= %d", retval);
                    goto end_of_apdu_exchange;
                }

                /* read the answer from the stack working buffer */
                status = sizeof(response1);
                retval = SCAPI_read(1, response1, &status);
                if (retval != ICC_OK) {
                    goto end_of_apdu_exchange;
                }

                /*
                 * Per the lab request, we must wait at least 200ms between each
                 * APDU exchange in order to let the test tool collect the data
                 */
                Delay(INTER_APDU_DELAY);

								lite_printf("\n Card 1 APDU exchange. the status = %d", status);
                
								/* Prepare next APDU */
                if (status < 6) {
                    /* Default APDU Command = select PSE */
                    memcpy(command1, selectFile, sizeof(selectFile));
                    len2 = sizeof(selectFile);
                } else if(response1[1] == 0x70) {
                    /* Abort loopback */
                    goto end_of_apdu_exchange;
                } else {
                    /* Loopback */
                    len2 = status -2;
                    memcpy(command1, response1, len2);
                }
			}
#endif
end_of_apdu_exchange:
#ifdef REGRESS_APDU_EXCHANGE
		}
#endif
			
power_off:
			
#ifdef CARD_SLOT_0			
			lite_printf("\n Going to Power down the card 0");

			status = POWER_DOWN;
			/*power off the card */
			SCAPI_ioctl(0, IOCTL_POWER_CARD, &status);

#endif			

#ifdef CARD_SLOT_1			
			lite_printf("\n Going to Power down the card 1");

			status = POWER_DOWN;
			/*power off the card */
			SCAPI_ioctl(1, IOCTL_POWER_CARD, &status);
#endif			
      Delay(2000);
    }
    /** We're done */
main_out:
		
#ifdef CARD_SLOT_0
    SCAPI_close(0);
#endif		

#ifdef CARD_SLOT_1		
    SCAPI_close(1);
#endif
    return result;
}



/******************************************************************************/
/* EOF */
