/*
 * mml_gpio.h --
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
 * Created on: Jun 13, 2014
 * Author: Yann G. <yann.gaude@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_GPIO_H_
#define _MML_GPIO_H_

#if __STDC_VERSION__ < 199901L
#ifndef inline
#define inline __inline
#endif
#endif

/** Global includes */
#include <config.h>
#include <errors.h>
/** Other includes */
/** Local includes */
#include <mml_gpio_regs.h>


/* Defines ********************************************************************/
#define MML_GPIO_BIT_RANGE_MIN 					0
#define MML_GPIO_BIT_RANGE_MAX 					31
#define	MML_GPIO_BIT_RANGE_NB					( MML_GPIO_BIT_RANGE_MAX + 1 )

/* Macros *********************************************************************/
/** COBRA adaptation */
#define	MML_GPIO_BASE_ERR						COBRA_GPIO_BASE_ERR
#define COBRA_PADSMODE_GPIO 					1
#define COBRA_PADSMODE_DEV  					0

/* Enumerations ***************************************************************/
/** GPIO errors list */
typedef enum
{
	MML_GPIO_ERR_MIN = MML_GPIO_BASE_ERR,
	/** Error Code: Timer not initialized */
	MML_GPIO_ERR_NOT_INITIALIZED,
	/** Invalid GPIO device ID */
	MML_GPIO_ERR_INVALID_DEVICE,
	/** Invalid parameter or value */
	MML_GPIO_ERR_OUT_OF_RANGE,
	/** Error Code: Invalid operation */
	MML_GPIO_ERR_INVALID,
	/** Error Code: add any other TMR error codes */
	/** Error Code: Generic error for unknown behavior */
	MML_GPIO_ERR_UNKNOWN,
	MML_GPIO_ERR_MAX = MML_GPIO_ERR_UNKNOWN

} mml_gpio_errors_t;

/** GPIO device identification numbers (id's) */
typedef enum
{
	MML_GPIO_DEV_MIN = 0,
	MML_GPIO_DEV0 = MML_GPIO_DEV_MIN,
	MML_GPIO_DEV1,
	MML_GPIO_DEV2,
	MML_GPIO_DEV_MAX = MML_GPIO_DEV2,
	MML_GPIO_DEV_COUNT

} mml_gpio_id_t;

/** GPIO pin input/output data logic '0' or logic '1' */
typedef enum
{
	MML_GPIO_OUT_LOGIC_ZERO, /** Drive logic �0� on GPIO  */
	MML_GPIO_OUT_LOGIC_ONE, /** Drive logic �1� on GPIO */

} mml_gpio_pin_data_t;

/** GPIO direction configuration for input or output */
typedef enum
{
	MML_GPIO_DIR_IN, /** direction input */
	MML_GPIO_DIR_OUT /** direction output */

} mml_gpio_direction_t;

/** Open drain configuration for GPIO0 port only */
typedef enum
{
	MML_GPIO_OPEN_DRAIN_MIN = 0,
	MML_GPIO_OPEN_DRAIN_NORMAL_MODE = MML_GPIO_OPEN_DRAIN_MIN, /** applies to GPIO0 pins only */
	MML_GPIO_OPEN_DRAIN_OUTPUT_MODE, /** Open Drain output mode: only applies to GPIO0 port pins */
	MML_GPIO_OPEN_DRAIN_MAX = MML_GPIO_OPEN_DRAIN_OUTPUT_MODE,
	MML_GPIO_OPEN_DRAIN_COUNT

} mml_gpio_open_drain_t;

/** Defines GPIO functionality for normal or alternate functions */
typedef enum
{
	MML_GPIO_NORMAL_FUNCTION, 		/** GPIO function enabled */
	MML_GPIO_SECODARY_ALT_FUNCTION, 	/** Secondary alternate function enabled*/
	MML_GPIO_TERTIARY_ALT_FUNCTION 	/** Tertiary alternate function enabled*/

} mml_gpio_function_t;

/** Interrupt mode */
typedef enum
{
	MML_GPIO_INT_MODE_LEVEL_TRIGGERED,
	MML_GPIO_INT_MODE_EDGE_TRIGGERED

} mml_gpio_intr_mode_t;

/** GPIO pin interrupt polarity */
typedef enum
{
	MML_GPIO_INT_POL_RAISING, /** raising edge */
	MML_GPIO_INT_POL_FALLING, /** falling edge */
	MML_GPIO_INT_POL_HIGH, /** high level triggering */
	MML_GPIO_INT_POL_LOW, /** low level triggering */
	MML_GPIO_INT_POL_DUAL_EDGE /** Dual edge [raising edge or falling edge] */

} mml_gpio_intr_polarity_t;

/** GPIO pad configuration information */
typedef enum
{
	MML_GPIO_PAD_NORMAL,
	MML_GPIO_PAD_PULLUP,
	MML_GPIO_PAD_PULLDOWN,
	MML_GPIO_PAD_WEAK_LATCH

} mml_gpio_pad_config_t;

/**
 * This is the GPIO configuration data structure
 */
typedef struct
{
	/** GPIO direction  */
	mml_gpio_direction_t						gpio_direction;
	/** GPIO function: primary, alternate function */
	mml_gpio_function_t  						gpio_function;
	/** GPIO interrupt mode : level triggered or edge triggered */
	mml_gpio_intr_mode_t 						gpio_intr_mode;
	/** interrupt polarity */
	mml_gpio_intr_polarity_t 					gpio_intr_polarity;
	mml_gpio_pad_config_t 						gpio_pad_config;
	/* any other configuration parameters goes here */

} mml_gpio_config_t;

/** GPIO initialization state */
typedef enum
{
	MML_GPIO_STATE_NOT_INITIALIZED = 0,
	MML_GPIO_STATE_INITIALIZED

} mml_gpio_state_t;

/* Structures *****************************************************************/


typedef struct
{
	/** Internal module state */
	mml_gpio_state_t							state;
	/** GPIO device Register base addresses */
	mml_gpio_regs_t								*reg;

} mml_gpio_port_conf_t;


/** Context structure related to only one GPIO port */
typedef struct
{
	/** First init ? */
	unsigned int								first_init;
	/** GPIO interface */
	mml_gpio_port_conf_t						port[MML_GPIO_DEV_COUNT];

} mml_gpio_context_t;

/* Variables ******************************************************************/

/* Functions ******************************************************************/
/**
 * The function is used to initialize and configure the consecutive 'n' GPIO
 * pins starting from the 'offset' position.
 */
int mml_gpio_init(mml_gpio_id_t dev_id,
					int offset,
					int bits_count,
					mml_gpio_config_t config);
/**
 * This function closes the GPIO device ID, also resets and disables
 * the GPIO clock.
 */
int mml_gpio_close(mml_gpio_id_t dev_id);
/** The function re-configures an already initialized GPIO device ID with new configuration */
int mml_gpio_set_config(mml_gpio_id_t dev_id,
							int offset,
							int bits_count,
							mml_gpio_config_t config);
/** The function configures the GPIO PAD configuration information */
int mml_gpio_pad_configure(mml_gpio_id_t dev_id,
							mml_gpio_pad_config_t config,
							int offset,
							int bits_count);
/**
 * This function configures the GPIO functionality i.e. GPIO in normal function mode
 * or alternate functions for the specified range of pins.
 */
int mml_gpio_configure_function(mml_gpio_id_t dev_id,
									mml_gpio_function_t function,
									int offset,
									int bits_count);
/**
 * This function enables a range of consecutive GPIO pins for the output mode.
 * The valid GPIO pins 'offset' range is between 0 to 31.
 */
int mml_gpio_enable_output(mml_gpio_id_t dev_id,
							int offset,
							int bits_count);

int mml_gpio_enable_interrupt(mml_gpio_id_t dev_id, int offset, int bits_count);
int mml_gpio_disable_interrupt(mml_gpio_id_t dev_id, int offset, int bits_count);
int mml_gpio_clear_interrupt(mml_gpio_id_t dev_id, int offset, int bits_count);
int mml_gpio_get_interrupt_status(mml_gpio_id_t dev_id, int offset, int bits_count, int* status);

/**
 * This function disables a range of consecutive GPIO pins from output mode.
 * The valid GPIO pins 'offset' range is between 0 to 31.
 */
int mml_gpio_disable_output(mml_gpio_id_t dev_id, int offset, int bits_count);
/**
 * This function configures the consecutive GPIO0 pins in the open drain output mode.
 * NOTE: This function applies only to GPIO0.
 */
int mml_gpio_config_open_drain(mml_gpio_id_t dev_id,
								const unsigned int offset,
								unsigned int bits_count,
								mml_gpio_open_drain_t config);
/** This function writes 32-bit data to the DATA_OUT register */
int mml_gpio_write_dataout(mml_gpio_id_t dev_id, unsigned int data);
/** This function reads 32-bit data from the DATA_OUT register */
int mml_gpio_read_dataout(mml_gpio_id_t dev_id, unsigned int *data);
/** This function reads 32-bit data from the DATA_IN register */
int mml_gpio_read_datain(mml_gpio_id_t dev_id, unsigned int *data);
/** This function transmits specified number of data words stored in the buffer to the GPIO device */
int mml_gpio_write_buffer(mml_gpio_id_t dev_id, unsigned int *buffer, int length);
/** This function receives specified number of data words from the GPIO device and stores in buffer */
int mml_gpio_read_buffer(mml_gpio_id_t dev_id, unsigned int *buffer, int length);
/**
 * Writes a consecutive bit pattern stored in the 'data' word in a specified
 * group of pins with in a GPIO port.
 */
int mml_gpio_write_bit_pattern(mml_gpio_id_t dev_id, int offset, int bits_count, unsigned int data);
/**
 * Reads bit pattern from consecutive GPIO pins and left adjusts this pattern and
 * stores it in LSB of 'data' word.
 */
int mml_gpio_read_bit_pattern(mml_gpio_id_t dev_id, int offset, int bits_count, unsigned int *data);
/**
 * This is the output function used to set/clear logic '1' or '0' to a GPIO pin
 * without affecting the other pins.
 */
int mml_gpio_pin_output(mml_gpio_id_t dev_id, const unsigned int offset,
		mml_gpio_pin_data_t output);
/** This is the input function it can be used to read data from a GPIO pin */
int mml_gpio_pin_input(mml_gpio_id_t dev_id, const unsigned int offset,
		mml_gpio_pin_data_t *input);

/** inline functions for COBRA adaptation */
/** GPIO Flip.
 * Set gpios and unset gpios.
 *
 */
static inline void mml_gpio_flip(mml_gpio_regs_t *gpio, unsigned int value)
{
	unsigned int old;
	old = gpio->out;
	gpio->out = (old & ~value) | value;
	gpio->out = (old & ~value);
}

/** GPIO Set Pads Mode.
 * Set the mode of pads.
 *
 */
void mml_gpio_set_padsmode(const unsigned int dev[3], int mode);

int mml_gpio_check(mml_gpio_id_t dev_id, int offset, int bits_count);
void mml_gpio_clear_out_int_wake(mml_gpio_id_t dev_id, int offset, int bits_count);
int mml_gpio_reset_interface(void);

#endif /* _MML_GPIO_H_ */

/******************************************************************************/
/* EOF */
