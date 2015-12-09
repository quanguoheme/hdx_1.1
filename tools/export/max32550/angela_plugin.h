/*
 * angela_plugin.h --
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
 * Created on: Apr 06, 2012
 * Author:
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: 4490     $:  Revision of last commit
 * $Author:: jeremy.kon#$:  Author of last commit
 * $Date:: 2015-01-14 1#$:  Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */
#ifndef _ANGELA_PLUGIN_H_
#define _ANGELA_PLUGIN_H_

/** @file angela_plugin.h Flora Plugin
 *
 */

/** Header Magic Value */
#define ANGELA_PLUGIN_HEADER_MAGIC 0x0F104A01

/** Plugin Header Section */
#define __plugin_hdr__ __attribute__ ((section (".header")))
/** Plugin Operation Section */
#define __plugin_ops__ __attribute__ ((section (".header.ops")))

/** Flora Plugin Type
 * Currently only one exist!
 */
typedef enum
{
	ANGELA_PLUGIN_TYPE_MIN = 0,
	ANGELA_PLUGIN_TYPE_NAND = ANGELA_PLUGIN_TYPE_MIN, /**< NAND Plugin */
	ANGELA_PLUGIN_TYPE_NOR,
	ANGELA_PLUGIN_TYPE_SDRAM, /**< SDRAM plugin */
	ANGELA_PLUGIN_TYPE_DDR, /**< DDR RAM plugin */
	ANGELA_PLUGIN_TYPE_IRAM,
	ANGELA_PLUGIN_TYPE_SPI,
	ANGELA_PLUGIN_TYPE_I2C,
	ANGELA_PLUGIN_TYPE_GENERIC,
	ANGELA_PLUGIN_TYPE_UNKNOWN,
	ANGELA_PLUGIN_TYPE_MAX = ANGELA_PLUGIN_TYPE_UNKNOWN,
	ANGELA_PLUGIN_TYPE_COUNT

} angela_plugin_type_t;

typedef enum
{
	VERSION_TYPE_NONE,
	VERSION_TYPE_RELEASE,
	VERSION_TYPE_RC,
	VERSION_TYPE_BETA,
	VERSION_TYPE_ALPHA

} version_type_t;

typedef struct
{
	unsigned char 	major;
	unsigned char 	minor;
	unsigned short 	patch;

} version_t;

typedef int(*__scp_applet_write)(unsigned int dest, unsigned int length,unsigned char *p_src);
typedef int(*__scp_applet_erase)(unsigned int dest, unsigned int length);

/** Generic Plugin Operations */
typedef struct
{
	/**< Write to memory */
	__scp_applet_write			write;
	/**< Compare memory data */
	__scp_applet_write			compare;
	/**< Erase memory */
	__scp_applet_erase			erase;

} angela_plugin_generic_ops_t;


/** Flora Plugin Header *******************************************************/
typedef struct
{
	/**< Magic value */
	unsigned int 				magic[2];
	/**< Plugin version */
	version_t 					version;
	/**< Base address of memory targetted by applet */
	unsigned int				mem_base_addr;
	/**< Size of this memory */
	unsigned int				mem_size;
	/**< Operations of the plugins */
	angela_plugin_generic_ops_t	ops;

} angela_plugin_header_t;

/** NAND Plugin Operations ****************************************************/
typedef struct
{
	/**< Init			*/
	int (*init)(unsigned int addr, unsigned int size);
	/**< Get version 	*/
	const char *(*get_version)(void);
	/**< Read NAND 		*/
	int (*read)(char *buffer, unsigned int size, unsigned int offset);

} angela_plugin_nand_ops_t;

#endif /* _ANGELA_PLUGIN_H_ */
