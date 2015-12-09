/*
 * mml_sc.h --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2012, Maxim Integrated Products
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
 * Created on: Oct 30, 2012
 * Author: Yann G. <yann.gaude@maxim-ic.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: $: Revision of last commit
 * $Author:: $: Author of last commit
 * $Date:: $: Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */
#ifndef _MML_SC_H_
#define _MML_SC_H_

#define MML_SC_ERR_COUNT 0x70000000
#undef SC_DEBUG

/**
 *  
 * @defgroup MML_SCError_Codes Smart Card Slot Error Codes
 * 
 * @ingroup HAL_SCS 
 * @ingroup Error_Codes 
 * 
 * This chapter contains descriptions of all error codes used by the \link 
 * HAL_SCS Smart Card Slot Interface \endlink of Maxim Integrated Products USIP Professional Hardware 
 * Abstraction Layer. 
 * 
 * @{
 * 
 */
typedef enum
{
	N_SCS_ERR_MIN = MML_SC_ERR_COUNT,
	/** \showinitializer Error code : Command aborted */
	N_SCS_ERR_CMD_ABORTED = N_SCS_ERR_MIN,
	/** \showinitializer Error code : Card mute. The card does not respond to the reset or has interrupted an exchange and a time-out has occured */
	N_SCS_ERR_CARD_MUTE,
	/** \showinitializer Error code : Parity error during an exchange with the card. */
	N_SCS_ERR_PARITY,
	/** \showinitializer Error code : Card consuming too much current or is short-circuiting */
	N_SCS_ERR_OVERRUN,
	/** \showinitializer Error code : Card absent. There is no card in the smart card interface. */
	N_SCS_ERR_CARD_ABSENT,
	/** \showinitializer Error code : Card powered down. A call to the #mml_sc_power_up() function must be done prior to any other operation */
	N_SCS_ERR_POWER_UP,
	/** \showinitializer Error code : Error in the card reset response */
	N_SCS_ERR_INVALID_ATR,
	/** \showinitializer Error code : In the card reset response, the Check Character (TCK) is not correct */
	N_SCS_ERR_BAD_ATR_TCK,
	/** \showinitializer Error code : Protocol error */
	N_SCS_ERR_PROTOCOL,
	/** \showinitializer Error code : The first byte of command (CLA) is invalid */
	N_SCS_ERR_CLASS_NOT_SUPP,
	/** \showinitializer Error code : The card sent an invalid procedure byte */
	N_SCS_ERR_PROC_BYTE,
	/** \showinitializer Error code : Speed (TA1 parameter) not supported */
	N_SCS_ERR_INVALID_SPEED,
	/** \showinitializer Error code : Function not implemented */
	N_SCS_ERR_UNKNOW,
	/** \showinitializer Error code : The card sent an invalid NAD (T=1) */
	N_SCS_ERR_INVALID_NAD,
	/** \showinitializer Error code : No Chipset driver is linked to device */
	N_SCS_ERR_NO_SYS,
	/** \showinitializer Error code : No buffer allocated for data */
	N_SCS_ERR_NO_BUFS,
	/** \showinitializer Error code : Device not configured */
	N_SCS_ERR_NXIO,
	/** \showinitializer Error code : No SC interface matches with device */
	N_SCS_ERR_NO_ENT,
	/** \showinitializer Error code : Operation not supported. Device not initialized */
	N_SCS_ERR_DEV_NO_SUPP,
	/** \showinitializer Error code : Response too long for the buffer */
	N_SCS_ERR_OVERFLOW,
	/**  */
	N_SCS_ERR_MAX = N_SCS_ERR_OVERFLOW,
	N_SCS_ERR_COUNT

} e_scs_errors;
/**  @} */
 
/**
 *  
 * @defgroup HAL_SCS HAL Smart Card Slot Interface
 * 
 * This chapter contains descriptions of the configuration structure, 
 * and functions available to access the Smart Card Interface.
 *
 *
 * @{
 * 
 * @section DESIGN_OVERVIEW Design overview
 * 
 * @subsection TRANSPARENT_MODE Transparent mode
 * The HAL Smart Card Slot Interface manages communication with ISO 7816 1-2-3 
 * asynchronous cards in transparent mode. 
 * Given the variety of customer needs, an upper layer can be necessary to 
 * manage specific cards and protocols by setting dynamically the communication
 * parameters.    
 * 
 * @subsection ANALOG_DEVICE_DRIVER Analog device driver
 * USIP embeds the digital part of a smart card interface.  
 * To complete the interface, customer should provide a physical interface 
 * control (the analog part). Its rule is to drive the card consumption.
 * The HAL Smart Card Slot Interface has been designed to handle a complete 
 * interface. 
 * So, customer has to provide the callback functions (hardware dependant) 
 * which will allow the HAL Smart Card Slot Interface to drive also, the analog 
 * interface.  The callback functions are registred through the implemetation
 * of the #MML_SCDRIVER_IO structure used with the #mml_sc_init() function.
 *
 * @subsection CLOCK_SIGNAL Clock signal
 * To function, the Smart Card Interface needs a clock signal. USIP can provide 
 * it-self this clock signal. Several frequencies are available : 
 * 48MHz, 24MHz, 16MHz, 12MHz, 9.6MHz, 8MHz, 6.86MHz, 6MHz, 5.33MHz, 4.8MHz, 
 * 4.33MHz, 4MHz, 3.69MHz, 3.43MHz, 3.2MHz and 3MHz.
 * For specific needs, this clock can be provide by an external signal.
 * 
 * @subsection TIMER_CONSIDERATION Timer consideration
 * No internal timer of the microcontroller is used by the HAL Smart Card Slot 
 * Interface. 
 * All the timing issues (Work Waiting Time, Extra Guard Time, Character 
 * Waiting Time, Block Waiting Time, Block Guard Time, Block Waiting Time 
 * extension) are fully supported by the specific timers of the ISO UART 
 * whatever the baud rate is on the I/O line. 
 * 
 */

/**
 *  
 * @defgroup MML_SCSTATUS Smart Card's status
 * 
 * @ingroup HAL_SCS 
 * 
 * This chapter lists Smart Card basic status in for a USIP's Smart Card slot.
 * 
 * @{
 * 
 */
/** \showinitializer Status indicator : Card not inserted */
#define MML_SCSTATUS_CARD_NOT_INSERTED        0x80

/** \showinitializer Status indicator : Card inserted */
#define MML_SCSTATUS_CARD_INSERTED            0x00

/** \showinitializer Status indicator : Card not powered */
#define MML_SCSTATUS_CARD_NOT_POWERED         0x04

/** \showinitializer Status indicator : Card powered */
#define MML_SCSTATUS_CARD_POWERED             0x00

/** \showinitializer Status indicator : Power supply = 5V */
#define MML_SCSTATUS_POWER_5V                 0x50

/** \showinitializer Status indicator : Power supply = 3V */
#define MML_SCSTATUS_POWER_3V                 0x30

/** \showinitializer Status indicator : Power supply = 1_8V */
#define MML_SCSTATUS_POWER_1_8V               0x18
/**  @} */

/**
 *  
 * @defgroup N_SCS_IOCTL Smart Card's IOCTL entries.
 * 
 * @ingroup HAL_SCS
 * 
 * This chapter lists IOCTL entries for a USIP slot.
 * 
 * @{
 * 
 */
/**
 *  
 * @defgroup N_SCS_IOCTL_DRIVER IOCTL Entries implemented in Smart Card chipset's driver.
 * 
 * @ingroup N_SCS_IOCTL
 * 
 * This chapter lists entries implemented in Smart Card chipset's driver, but not implemented in mml_sc_ioctl().
 * 
 * @{
 * 
 */
typedef enum
{
	N_SCS_IOCTL_MIN = 0,
	/** \showinitializer Code for the reset command of the smart card interface */
	N_SCS_IOCTL_RESET = N_SCS_IOCTL_MIN,
	/** \showinitializer Code for the power up command of the smart card interface */
	N_SCS_IOCTL_POWER_UP,
	/** \showinitializer Code for the power down command of the smart card interface */
	N_SCS_IOCTL_POWER_DOWN,
	/** \showinitializer Code for the chip select of the analog interface */
	N_SCS_IOCTL_SELECT,
	/** \showinitializer Code for the chip unselect of the analog interface */
	N_SCS_IOCTL_UNSELECT,
	/** \showinitializer Code for the check voltage command of the analog interface */
	N_SCS_IOCTL_CHECK_VOLT_OK,
	/** \showinitializer Code for the power configuration command of the analog interface */
	N_SCS_IOCTL_CONFIG_POWER,
	/** \showinitializer Code for the check card presence command of the analog interface */
	N_SCS_IOCTL_CHECK_CARD_PRESENCE,
	/** \showinitializer Code for the card reset command from the analog interface */
	N_SCS_IOCTL_CARD_RESET,
	/** \showinitializer Code asking for the number of the selected device  */
	N_SCS_IOCTL_WHICHSELECTED,
	/** \showinitializer Code for the direct driving of the Card IO signal */
	N_SCS_IOCTL_SET_CARDIO,
	/** \showinitializer Code for the direct driving of the Card Reset signal */
	N_SCS_IOCTL_SET_CARDRESET,
	/** \showinitializer Code for the direct driving of the Card Clock signal */
	N_SCS_IOCTL_SET_CARDCLK,
	/** \showinitializer Code to set up the speed communication parameter TA1 of ISO 7816-3 */
	N_SCS_IOCTL_SET_SPEED,
	/** \showinitializer Code to set up the guard time : parameter N of ISO 7816-3 */
	N_SCS_IOCTL_SET_GUARD_TIME,
	/** \showinitializer Code to set up the Waiting Time : parameter WI of ISO 7816-3*/
	N_SCS_IOCTL_SET_WAITING_TIME,
	/** \showinitializer Code to set up the Character Waiting Time */
	N_SCS_IOCTL_SET_CWT,
	/** \showinitializer Code to set up the Block Waiting Time */
	N_SCS_IOCTL_SET_BWT,
	/** \showinitializer Code to retrieve the Character Waiting Time value */
	N_SCS_IOCTL_GET_CWT,
	/** \showinitializer Code to retrieve the Block Waiting Time value */
	N_SCS_IOCTL_GET_BWT,
	/** \showinitializer Code to indicate a card withdrawn with consequenses a cancel of the current operation. */
	N_SCS_IOCTL_CARD_WITHDRAWN,
	/** \showinitializer Code to attach handler on card moved event */
	N_SCS_IOCTL_ATTACH_CARD_MOVED,
	/** \showinitializer Code for the enable/disable the UART mode */
	N_SCS_IOCTL_SET_MODE_AUTO,
	/** \showinitializer Code for the direct driving of the Card C4 signal */
	N_SCS_IOCTL_SET_CARDC4,
	/** \showinitializer Code for the direct driving of the Card C8 signal */
	N_SCS_IOCTL_SET_CARDC8,
	/** \showinitializer Code for the direct driving of the Card Vcc signal */
	N_SCS_IOCTL_SET_CARDVCC,
	/** \showinitializer Code for the direct reading of the Card IO signal */
	N_SCS_IOCTL_GET_CARDIO,
	/** \showinitializer Code for the direct reading of the Card Reset signal */
	N_SCS_IOCTL_GET_CARDRESET,
	/** \showinitializer Code for the direct reading of the Card Clock signal */
	N_SCS_IOCTL_GET_CARDCLK,
	/** \showinitializer Code for the direct reading of the Card C4 signal */
	N_SCS_IOCTL_GET_CARDC4,
	/** \showinitializer Code for the direct reading of the Card C8 signal */
	N_SCS_IOCTL_GET_CARDC8,
	/** \showinitializer Code for the direct reading of the Card Vcc signal */
	N_SCS_IOCTL_GET_CARDVCC,
	/** \showinitializer Code setting the safety loop for the power up sequence */
	N_SCS_IOCTL_SECURITY_LOOP,
	/** \showinitializer Code setting clock counter to chosen value and launch it Parameter is in ETUs not smartcard clock value !!! */
	N_SCS_IOCTL_LAUNCH_CLK_COUNTER,
	/** \showinitializer Code for halting smartcard clock counters */
	N_SCS_IOCTL_HALT_CLK_COUNTER,
	/**  */
	N_SCS_IOCTL_MAX = N_SCS_IOCTL_HALT_CLK_COUNTER,
	N_SCS_IOCTL_COUNT

} e_scs_ioctl_cmd;
/**  @} */

typedef enum
{
	/** \showinitializer Power supply = 0V */
	N_MML_SC_POWER_0V = 0x00,
	/** \showinitializer Power supply = 1.8V */
	N_MML_SC_POWER_1_8V = 0x18,
	/** \showinitializer Power supply = 3V */
	N_MML_SC_POWER_3V = 0x30,
	/** \showinitializer Power supply = 5V */
	N_MML_SC_POWER_5V = 0x50

} e_mml_sc_power;

/** \showinitializer Smart card protocol: Asynchronous T=0 */
#define MML_SCPROTOCOL_T0          0x00

/** \showinitializer Smart card protocol: Asynchronous T=1 */
#define MML_SCPROTOCOL_T1          0x01

/** \showinitializer Smart card protocol: Synchronous */
#define HAL_SCS_PROTOCOL_SYNC        0x02

/** \showinitializer Internal Clock Frequency : 5.05MHz */
#define MML_SCCLOCK_5_05MHz	19

/** \showinitializer Internal Clock Frequency : 4.8MHz */
#define MML_SCCLOCK_4_8MHz	20

/** \showinitializer Internal Clock Frequency : 3.2MHz */
#define MML_SCCLOCK_3_2MHz	30
/** \showinitializer Internal Clock Frequency : 3MHz */
#define MML_SCCLOCK_3MHz		32
/** \showinitializer Internal Clock Frequency : 2MHz */
#define MML_SCCLOCK_2MHz		48

/** \showinitializer This indicator is used to specify that the external clock entry is the main 
 * working clock for the interface.  */
#define MML_SCCLOCK_EXTERNAL  0x10

/** \showinitializer This indicator is used to specify That the main working clock for the smart 
 * card interface must be generated internally.
 */
#define MML_SCCLOCK_INTERNAL  0
/**  @} */
/**
 *  
 * @defgroup MML_SCCONV USIP's Smart Card slot convention supported.
 * 
 * @ingroup HAL_SCS
 * 
 * This chapter lists USIP's Smart Card slot convention supported.
 * 
 * @{
 * 
 */
/** \showinitializer Convention used to code data bytes : Direct convention */
#define MML_SCCONV_DIRECT    0

/** \showinitializer Convention used to code data bytes : Direct inverse */
#define MML_SCCONV_INVERSE   1
/**  @} */

/** Macros ********************************************************************/
#define mml_sc_dev_register_service(_service_) \
    (dev->vsr = _service_)

/** Enums *********************************************************************/
typedef enum
{
	MML_SC_EVENT_ATR_RECEIVED = 0,
	MML_SC_EVENT_DATA_RECEIVED,
	MML_SC_EVENT_DATA_SENT,
	MML_SC_EVENT_ERROR,
	MML_SC_EVENT_ATR_TS_RECEIVED,
	MML_SC_EVENT_WAITING_TIME_EXTENSION,
	MML_SC_EVENT_COLLISION_BGT,
	MML_SC_EVENT_COLLISION_PARITY,
	MML_SC_EVENT_COLLISION_OTHER,
	MML_SC_EVENT_CLOCK_COUNTER,
	MML_SC_EVENT_MAX,
	MML_SC_EVENT_COUNT,
} e_mml_sc_event;

typedef enum
{
    /** \showinitializer Identifier of the first Smart Card Slot */
    N_MML_SC_DEV0 = 0,
    /** \showinitializer Identifier of the second Smart Card Slot */
    N_MML_SC_DEV1,
    /** \showinitializer This constant is used to fix the number of available Smart Card Slots */
    N_MML_SC_DEV_COUNT,
    /**  */
    N_MML_SC_DEV_NOSCIB = 0xff

} e_mml_sc_devices;

/** Structures ****************************************************************/
typedef struct 
{
    /** init() function initializes the analog interface. */
    int (*init)( int devnum );
    /** ioctl() function is the main entry point to perform the analog
     * interface control. */
    int (*ioctl)(int cmd, void *param );
    
} t_mml_sc_driver_io;


/** The configuration structure used to set up the Smart Card Interface */
typedef struct 
{
	/** Power supply. */
	e_mml_sc_power							power;
	/* Compatibility with libemv */
	unsigned int protocol;
	/** Smart Card Clock Frequency. */
	unsigned int							clock;
	/** Valid pointer to the #MML_SCDRIVER_IO structure (driver functions
	* which control the analog part of the interface) */
	t_mml_sc_driver_io						*devio;

	/** SCIB number to associate with the device number. */

} t_mml_sc_config;


/** The structure used to retrieve or to set up the communication parameters */
typedef struct 
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
   
    /** The power supply value :
     *      -#MML_SCPOWER_1_8V
     *      -#MML_SCPOWER_3V
     *      -#MML_SCPOWER_5V
     */
    unsigned char power;                   
           
    /** Convention used to transfer byte : 
     * #MML_SCCONV_DIRECT or #MML_SCCONV_INVERSE */
    unsigned char conv;  
    
} mml_sc_slot_info_t;

typedef void (*mml_sc_handler_t)(int, int);


/*----------------------------------------------------------------------------
 * Function declarations
 *---------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif


/**
 * The function initializes a Smart Card Interface.
 * 
 * @param [in] params   Pointer to an initialized configuration structure.
 *  
 * @retval NO_ERROR			No error 
 * @retval COMMON_ERR_INVAL       Invalid parameter.
 * @retval Other            See \link Error_Codes Error Codes \endlink
 *							or mml_sc_set_config()'s and hal_apm_set_device_mode()'s ones.

 * 
 */
int mml_sc_init(t_mml_sc_config *params );


/**
* The function shutdowns a Smart Card Interface.
 * 
 * @retval NO_ERROR			No error
 * @retval N_SCS_ERR_NO_ENT		No SCIB matches with this device.
 * @retval Other            See \link Error_Codes Error Codes \endlink
 *							or mml_sc_power_down()'s and hal_apm_set_device_mode()'s ones.
 * 
 */
int mml_sc_shutdown(void);

/** 
 * The function provides the current state of a Smart Card Interface. 
 * It returns information indicating :
 *  \li The power supply value
 *  \li Card presence
 *  \li The card power status 
 *  \li The communication protocol (T=0 or T=1)
 * 
 * @param[out] status   Receives the current state of the Smart Card Interface. 
 *                      Upon success, it receives a combinaison of the following 
 *                      state indicators: 
 *                          \li #MML_SCSTATUS_POWER_5V
 *                          \li #MML_SCSTATUS_POWER_3V
 *                          \li #MML_SCSTATUS_CARD_NOT_POWERED
 *                          \li #MML_SCSTATUS_CARD_POWERED
 *                          \li #MML_SCSTATUS_CARD_NOT_INSERTED
 *                          \li #MML_SCSTATUS_CARD_INSERTED
 *  
 * @retval NO_ERROR			No error 
 * @retval N_SCS_ERR_NXIO        Device not configured. Use mml_sc_init() first.
 * @retval COMMON_ERR_INVAL       Invalid parameter.
 * @retval Other            See \link Error_Codes Error Codes \endlink
 * 
 */
int mml_sc_get_status( int *status );

/** 
 * The function configures a Smart Card Interface and selects the working 
 * clock for the interface.
 * 
 * @param[in] params    Pointer to an initialized configuration structure.
 * 
 * @retval NO_ERROR           No error
 * @retval N_SCS_ERR_NXIO    	Device not configured. Use mml_sc_init() first.
 * @retval COMMON_ERR_INVAL   	Invalid parameter.
 * @retval Other        	See \link Error_Codes Error Codes \endlink
 * 
 */
int mml_sc_set_config( t_mml_sc_config *params );

/**
 * The function powers up and resets an asynchronous card. During the card 
 * Answer To Reset (ATR), the function performs a check for the T0 and TDi 
 * bytes to compute the number of bytes that must be received.
 * 
 * @param[out] atr      Valid pointer to the card Answer To Reset (ATR).
 * 
 * @param[out] length   Valid pointer to the length of the Answer To Reset.
 * 
 * @retval NO_ERROR			No error
 * @retval N_SCS_ERR_NXIO        Device not configured. Use mml_sc_init() first.
 * @retval COMMON_ERR_INVAL       Invalid parameter.
 * @retval COMMON_ERR_ALREADY     Operation already pending
 * @retval  N_SCS_ERR_CARD_ABSENT Card absent
 * @retval  ERR_SCS_BAD_ATR     Bad Answer To Reset
 * @retval Other            See \link Error_Codes Error Codes \endlink
 * 
 * @note The function is synchronous since a call to #mml_sc_attach() makes
 * the function asynchronous by returning immediatly without waiting the end 
 * of the operation.
 * 
 * 
 */
int mml_sc_power_up( unsigned char *atr, unsigned int *length );


/**
 * The function powers down a card.
 * 
 * 
 * @retval NO_ERROR			No error 
 * @retval N_SCS_ERR_NXIO        Device not configured. Use mml_sc_init() first.
 * @retval Other            See \link Error_Codes Error Codes \endlink.
 * 
 */
int mml_sc_power_down(void);


/**
 * The function is used to retreive the working communication parameters.
 * 
 * @param[out] slot_info    Valid pointer to an initialized #mml_sc_slot_info_t
 *                          structure. 
 * 
 * @retval NO_ERROR			No error
 * @retval N_SCS_ERR_NXIO        Device not configured. Use mml_sc_init() first.
 * @retval COMMON_ERR_INVAL       Invalid parameter.
 * 
 */
int mml_sc_get_slot_info( mml_sc_slot_info_t *slot_info );


/**
 * The function is used to set the working communication parameters.
 * 
 * @param[in] slot_info     Valid pointer to an initialized #mml_sc_slot_info_t
 *                          structure.
 * 
 * @retval NO_ERROR               No error
 * @retval COMMON_ERR_INVAL       Invalid parameter.
 * 
 */
int mml_sc_set_slot_info( mml_sc_slot_info_t *slot_info );

/**
 * The function sends a T=0 ingoing command to the card , and retreives the 
 * response status word SW1 SW2. This command can send up to 255 bytes.
 * 
 * @param[in]   c_apdu   Valid pointer to a command APDU buffer 
 *                      {CLA,INS,P1,P2,Lc, data }.
 * 
 * @param[out]  r_apdu   Valid pointer to the card response SW1 SW2.
 *
 * @param[in,out] r_length    Valid pointer to the size of response buffer, 
 *                          updated with the number of bytes available 
 *                          in the response buffer.
 * 
 * @retval NO_ERROR			No error.
 * @retval N_SCS_ERR_NXIO        Device not configured. Use mml_sc_init() first.
 * @retval COMMON_ERR_INVAL       Invalid parameter.
 * @retval COMMON_ERR_ALREADY		An operation is pending.
 * @retval N_SCS_ERR_CARD_ABSENT	No card present.
 * @retval N_SCS_ERR_POWER_UP	Card not powered up.
 * @retval Other            See \link Error_Codes Error Codes \endlink
 * 							or those from communication.
 * 
 * @note The function is synchronous since a call to #mml_sc_attach() makes a
 * the function asynchronous by returning immediatly without waiting the end 
 * of the operation.
 * 
 */
int mml_sc_t0_in(unsigned char *c_apdu,
                   unsigned char *r_apdu, unsigned int *r_length );


/**
 * The function sends a T=0 outgoing command to a card, 
 * and retreives the response card, plus the status word SW1 SW2.
 * 
 * @param[in]    c_apdu   Valid pointer to a command APDU buffer 
 *                          {CLA,INS,P1,P2,Le}.
 * 
 * @param[out]   r_apdu   Valid pointer to the card response
 * 
 * @param[in,out] r_length    Valid pointer to the size of response buffer, 
 *                          updated with the number of bytes available 
 *                          in the response buffer.
 * 
 * @retval NO_ERROR			No error
 * @retval N_SCS_ERR_NXIO        Device not configured. Use mml_sc_init() first.
 * @retval COMMON_ERR_INVAL       Invalid parameter.
 * @retval N_SCS_ERR_CARD_ABSENT	No card present.
 * @retval N_SCS_ERR_POWER_UP	Card not powered up.
 * @retval Other            See \link Error_Codes Error Codes \endlink
 * 							or those from communication.
 * 
 * @note The function is synchronous since a call to #mml_sc_attach() makes a
 * the function asynchronous by returning immediatly without waiting the end 
 * of the operation.
 * 
 */
int mml_sc_t0_out(unsigned char *c_apdu,
                    unsigned char *r_apdu, unsigned int *r_length );


/**
 * This function may be used when for T=1 protocol exchanges when the 
 * application layer provides the complete T=1 frame including prologue, 
 * information and epilogue fields. The frame is passed to this function from 
 * address frame_in and is assumed to be \c len length. The function will send 
 * this frame to the smart card without any verification and will return 
 * the length of the received frame from the card.
 * 
 * @param[in]   frame_in    Valid pointer to the send buffer
 * 
 * @param[out]  frame_out   Valid pointer to the card response
 * 
 * @param[in] length_in   : Value of the size buffer frame_in,
 * @param[in,out] length_out  : Valid pointer to the length of the frame_out.
 * 
 * @retval NO_ERROR			No error
 * @retval N_SCS_ERR_NXIO        Device not configured. Use mml_sc_init() first.
 * @retval COMMON_ERR_INVAL       Invalid parameter.
 * @retval N_SCS_ERR_CARD_ABSENT	No card present.
 * @retval N_SCS_ERR_POWER_UP	Card not powered up.
 * @retval Other            See \link Error_Codes Error Codes \endlink
 * 							or those from communication.
 * 
 * @note The function is synchronous since a call to #mml_sc_attach() makes a
 * the function asynchronous by returning immediatly without waiting the end 
 * of the operation. * 
 * 
 */                        
int mml_sc_t1_block( unsigned char *frame_in,
					  int length_in,
                      unsigned char *frame_out,
                      unsigned int *length_out );

/**
 * The functions sends a Protocol Parameter Selection (PPS) to a card, 
 * and retreives the response card . 
 * 
 * @param[in]   pps_request    Valid pointer to the send buffer
 * 
 * @param[out]  pps_response       Valid pointer to the card response
 * 
 * @param[in]   size_request size buffer in
 * 
 * @param[in,out] size_response in   : Valid pointer to the size buffer dataOut,
 *                          out : Valid pointer to the length of the response.
 * 
 * @retval NO_ERROR			No error
 * @retval N_SCS_ERR_NXIO        Device not configured. Use mml_sc_init() first.
 * @retval COMMON_ERR_INVAL       Invalid parameter.
 * @retval Other            See \link Error_Codes Error Codes \endlink
 * 							or those from treatment PPS negotiation.
 *
 * @note The function is synchronous since a call to #mml_sc_attach() makes a
 * the function asynchronous by returning immediatly without waiting the end 
 * of the operation.
 *
 */
int mml_sc_pps_exchange( unsigned char *pps_request,
                          int size_request,
                          unsigned char *pps_response,
                          unsigned int *size_response);
                    

/**
 * The function performs Smart Card Interface specific control functions.
 * 
 * @param[in]     cmd   Requested control function. Possible values are:
 *                          \li N_SCS_IOCTL_SET_CARDIO
 *                          \li N_SCS_IOCTL_SET_CARDRESET
 *                          \li N_SCS_IOCTL_SET_CARDCLK
 *                          \li N_SCS_IOCTL_SET_SPEED
 *                          \li N_SCS_IOCTL_SET_GUARD_TIME
 *                          \li N_SCS_IOCTL_SET_WAITING_TIME
 *                          \li N_SCS_IOCTL_SET_CWT
 *                          \li N_SCS_IOCTL_SET_BWT
 *                          \li N_SCS_IOCTL_GET_CWT
 *                          \li N_SCS_IOCTL_GET_BWT
 *                          \li N_SCS_IOCTL_CARD_WITHDRAWN
 *                          \li N_SCS_IOCTL_SET_MODE_AUTO
 *                          \li N_SCS_IOCTL_SET_CARDC4
 *                          \li N_SCS_IOCTL_SET_CARDC8
 *                          \li N_SCS_IOCTL_SET_CARDVCC
 *                          \li N_SCS_IOCTL_GET_CARDIO
 *                          \li N_SCS_IOCTL_GET_CARDRESET
 *                          \li N_SCS_IOCTL_GET_CARDCLK
 *                          \li N_SCS_IOCTL_GET_CARDC4
 *                          \li N_SCS_IOCTL_GET_CARDC8
 *                          \li N_SCS_IOCTL_GET_CARDVCC
 *                          \li N_SCS_IOCTL_SECURITY_LOOP
 * 
 * @param[in,out] param  Points to a buffer that contains any data required for 
 *                      the given control function or receives data from 
 *                      that function.
 *
 * @retval NO_ERROR			No error
 * @retval COMMON_ERR_INVAL       Invalid parameter.
 * @retval Other            See \link Error_Codes Error Codes \endlink
 * 
 */
int mml_sc_ioctl(int cmd, void *param);

/**
 * The function registers an interrupt handler for a type of event.
 *
 * @param[in] event     Event type. Possible values are the following: 
 *                          \li #MML_SC_EVENT_ATR_RECEIVED
 *                          \li #MML_SC_EVENT_DATA_RECEIVED
 *                          \li #MML_SC_EVENT_DATA_SENT
 *                          \li #MML_SC_EVENT_ERROR
 * 
 * @param[in] handler   Pointer to the interrupt handler function.
 * 
 * @retval NO_ERROR			No error
 * @retval N_SCS_ERR_NXIO        Device not configured. Use mml_sc_init() first.
 * @retval COMMON_ERR_INVAL       Invalid parameter.
 * @retval Other            Valid pointer of the previous handler for chosen event.
 * 
 */
int mml_sc_attach(int event, void (*handler)(int));

/**
 * The function unregisters an interrupt handler that is used to process 
 * a particular event.
 *
 * @param[in] event      Event type. Possible values are the following: 
 *                          \li #MML_SC_EVENT_ATR_RECEIVED
 *                          \li #MML_SC_EVENT_DATA_RECEIVED
 *                          \li #MML_SC_EVENT_DATA_SENT
 *                          \li #MML_SC_EVENT_ERROR
 *  
 * @retval NO_ERROR			No error
 * @retval N_SCS_ERR_NXIO        Device not configured. Use mml_sc_init() first.
 * @retval COMMON_ERR_INVAL       Invalid parameter.
 * @retval COMMON_ERR_ALREADY     Operation already done.
 * 
 */
int mml_sc_detach( int event);


/**
 * The function is used to cancel the current smart card transaction.
 * 
 * @retval NO_ERROR			No error
 * @retval N_SCS_ERR_NXIO        Device not configured. Use mml_sc_init() first.
 * @retval Other            See \link Error_Codes Error Codes \endlink
 * 
 */
int mml_sc_cancel(void);

/**
 * The function is used to retrieve the last status error (usefull with 
 * asynchronous operations).
 *
 * @param[out]  err     Valid pointer to the last error. 
 * 
 * @retval NO_ERROR			No error
 * @retval N_SCS_ERR_NXIO        Device not configured. Use mml_sc_init() first.
 * @retval COMMON_ERR_INVAL       Invalid parameter.
 * 
 */
int mml_sc_get_last_error(int *err);

int mml_sc_init_presence_card(void (*func)(int) );


#ifdef __cplusplus
}
#endif

/** @} */

#endif /* _MML_SC_H_ */

/******************************************************************************/
/* EOF */
