/*
 * mml_i2cl_regs.h --
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
 * Created on: June 5, 2012
 * Author: Jeremy B. <jeremy.brodt@maximintegrated.com>
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision: 4487 $:  Revision of last commit
 * $Author: jeremy.kongs $:  Author of last commit
 * $Date: 2015-01-14 15:43:29 -0600 (Wed, 14 Jan 2015) $:  Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#ifndef _MML_I2CL_REGS_H_
#define _MML_I2CL_REGS_H_

/** @file mml_i2cl_regs.h I2CL Registers Header */

/** @defgroup MML_I2CL I2C Driver
 * @ingroup MML_DRIVER
 */

// I2CCN bit offset definitions
#define MML_I2C_CN_MST_OFST        1
#define MML_I2C_CN_READ_OFST       11

// I2CCN bit mask definitions
#define MML_I2C_CN_I2CEN_MASK      0x0001
#define MML_I2C_CN_MST_MASK        0x0002
#define MML_I2C_CN_GCEN_MASK       0x0004
#define MML_I2C_CN_IRXM_MASK       0x0008
#define MML_I2C_CN_ACK_MASK        0x0010
#define MML_I2C_CN_ACKV_MASK       0x0020
#define MML_I2C_CN_SCLO_MASK       0x0040
#define MML_I2C_CN_SDAO_MASK       0x0080
#define MML_I2C_CN_SCL_MASK        0x0100
#define MML_I2C_CN_SDA_MASK        0x0200
#define MML_I2C_CN_OEN_MASK        0x0400
#define MML_I2C_CN_READ_MASK       0x0800
#define MML_I2C_CN_HSMDEN_MASK     0x8000

// I2CST bit offset definitions
#define MML_I2C_ST_ERR_OFST	      0
#define MML_I2C_ST_ERR_ARBL_OFST   0
#define MML_I2C_ST_ERR_TO_OFST     1
#define MML_I2C_ST_ERR_ANAK_OFST   2
#define MML_I2C_ST_ERR_DNAK_OFST   3
#define MML_I2C_ST_ERR_RXFIFO_OFST 4
#define MML_I2C_ST_ERR_SEQSTA_OFST 5
#define MML_I2C_ST_ERR_SEQSTO_OFST 6
#define MML_I2C_ST_ST_OFST	        13

// I2CST bit mask definitions
#define MML_I2C_ST_ERR_MASK        0x007F
#define MML_I2C_ST_ERR_ARBL_MASK   0x0001
#define MML_I2C_ST_ERR_TO_MASK     0x0002
#define MML_I2C_ST_ERR_ANAK_MASK   0x0004
#define MML_I2C_ST_ERR_DNAK_MASK   0x0008
#define MML_I2C_ST_ERR_RXFIFO_MASK 0x0010
#define MML_I2C_ST_ERR_SEQSTA_MASK 0x0020
#define MML_I2C_ST_ERR_SEQSTO_MASK 0x0040
#define MML_I2C_ST_BUS_MASK        0x0080
#define MML_I2C_ST_RXE_MASK        0x0100
#define MML_I2C_ST_RXF_MASK        0x0200
#define MML_I2C_ST_TXE_MASK        0x0400
#define MML_I2C_ST_TXF_MASK        0x0800
#define MML_I2C_ST_CKMD_MASK       0x1000
#define MML_I2C_ST_ST_IDLE_MASK    0x0000
#define MML_I2C_ST_ST_START_MASK   0x2000
#define MML_I2C_ST_ST_MCODE_MASK   0x4000
#define MML_I2C_ST_ST_ADDR_MASK    0x6000
#define MML_I2C_ST_ST_DATA_MASK    0x8000
#define MML_I2C_ST_ST_NOPART_MASK  0xA000
#define MML_I2C_ST_ST_NACK_MASK    0xC000
#define MML_I2C_ST_ST_MASK         0xE000

// I2CINT bit offset definitions
#define MML_I2C_INT_DONEI_OFST     0
#define MML_I2C_INT_IRXMI_OFST     1
#define MML_I2C_INT_GCI_OFST       2
#define MML_I2C_INT_AMI_OFST       3
#define MML_I2C_INT_RXTHI_OFST     4
#define MML_I2C_INT_TXTHI_OFST     5
#define MML_I2C_INT_STOPI_OFST     6
#define MML_I2C_INT_ERRI_OFST      7
#define MML_I2C_INT_DONEIE_OFST    8
#define MML_I2C_INT_IRXMIE_OFST    9
#define MML_I2C_INT_GCIE_OFST      10
#define MML_I2C_INT_AMIE_OFST      11
#define MML_I2C_INT_RXTHIE_OFST    12
#define MML_I2C_INT_TXTHIE_OFST    13
#define MML_I2C_INT_STOPIE_OFST    14
#define MML_I2C_INT_ERRIE_OFST     15

// I2CINT bit mask definitions
#define MML_I2C_INT_ALL_MASK       0x00FF
#define MML_I2C_INT_DONEI_MASK     0x0001
#define MML_I2C_INT_IRXMI_MASK     0x0002
#define MML_I2C_INT_GCI_MASK       0x0004
#define MML_I2C_INT_AMI_MASK       0x0008
#define MML_I2C_INT_RXTHI_MASK     0x0010
#define MML_I2C_INT_TXTHI_MASK     0x0020
#define MML_I2C_INT_STOPI_MASK     0x0040
#define MML_I2C_INT_ERRI_MASK      0x0080
#define MML_I2C_INT_DONEIE_MASK    0x0100
#define MML_I2C_INT_IRXMIE_MASK    0x0200
#define MML_I2C_INT_GCIE_MASK      0x0400
#define MML_I2C_INT_AMIE_MASK      0x0800
#define MML_I2C_INT_RXTHIE_MASK    0x1000
#define MML_I2C_INT_TXTHIE_MASK    0x2000
#define MML_I2C_INT_STOPIE_MASK    0x4000
#define MML_I2C_INT_ERRIE_MASK     0x8000

// I2CFIFO bit mask definitions
#define MML_I2C_FIFO_RXLEN_MASK    0x00FF
#define MML_I2C_FIFO_RXLEN_OFST	  0
#define MML_I2C_FIFO_TXLEN_MASK    0xFF00
#define MML_I2C_FIFO_TXLEN_OFST	  8

// I2CRXCFG bit offset definitions
#define MML_I2C_RXCFG_RXFSH_OFST   7
#define MML_I2C_RXCFG_RXTH_OFST    8

// I2CRXCFG bit mask definitions
#define MML_I2C_RXCFG_DNR_MASK     0x0001
#define MML_I2C_RXCFG_RXFSH_MASK   0x0080
#define MML_I2C_RXCFG_RXTH_MASK    0x0F00

// I2CRX bit offset definitions
#define MML_I2C_RX_RXCNT_OFST	    0
#define MML_I2C_RX_RXFIFO_OFST	    8

// I2CRX bit mask definitions
#define MML_I2C_RX_RXCNT_MASK      0x000F
#define MML_I2C_RX_RXFIFO_MASK     0x0F00

// I2CTXCFG bit offset definitions
#define MML_I2C_TXCFG_TXLAST_OFST  5
#define MML_I2C_TXCFG_TXLO_OFST    6
#define MML_I2C_TXCFG_TXFSH_OFST   7
#define MML_I2C_TXCFG_TXTH_OFST    8

// I2CTXCFG bit mask definitions
#define MML_I2C_TXCFG_TXOVRIE_MASK 0x0001
#define MML_I2C_TXCFG_TXOVRI_MASK  0x0002
#define MML_I2C_TXCFG_TXLAST_MASK  0x0020
#define MML_I2C_TXCFG_TXLO_MASK    0x0040
#define MML_I2C_TXCFG_TXFSH_MASK   0x0080
#define MML_I2C_TXCFG_TXTH_MASK    0x0F00

// I2CTX bit offset definitions
#define MML_I2C_TX_TXFIFO_OFST	    8

// I2CTX bit mask definitions
#define MML_I2C_TX_TXFIFO_MASK     0x0F00

// I2CMCN bit offset definitions
#define MML_I2C_MCN_START_OFST     0
#define MML_I2C_MCN_RESTART_OFST   1
#define MML_I2C_MCN_STOP_OFST      2
#define MML_I2C_MCN_SEA_OFST       7
#define MML_I2C_MCN_IMCODE_OFST	  8

// I2CMCN bit mask definitions
#define MML_I2C_MCN_START_MASK     0x0001
#define MML_I2C_MCN_RESTART_MASK   0x0002
#define MML_I2C_MCN_STOP_MASK      0x0004
#define MML_I2C_MCN_SEA_MASK       0x0080
#define MML_I2C_MCN_IMCODE_MASK    0x0700

// I2CHSCK bit mask definitions
#define MML_I2C_HSCK_HSCKL_MASK    0x00FF
#define MML_I2C_HSCK_HSCKL_OFST	  0
#define MML_I2C_HSCK_HSCKH_MASK    0xFF00
#define MML_I2C_HSCK_HSCKH_OFST	  8

// I2CSLA bit offset definitions
#define MML_I2C_SLA_SLA_OFST	      0
#define MML_I2C_SLA_EA_OFST        15

// I2CSLA bit mask definitions
#define MML_I2C_SLA_SLA_MASK       0x03FF
#define MML_I2C_SLA_EA_MASK        0x8000


#ifndef __ASSEMBLER__
/* -------------------------------------------------------------------------- */
/** I2CL Registers.
 *
 */
typedef volatile struct
{
	unsigned int								i2ccn; //!< 0x00 Control
	unsigned int								i2cst; //!< 0x04 Status
	unsigned int								i2cint; //!< 0x08 Interrupt
	unsigned int								i2cfifo; //!< 0x0C FIFO Configuration
	unsigned int								i2crxcfg; //!< 0x10 RX Configuration
	unsigned int								i2crx; //!< 0x14 RX
	unsigned int								i2ctxcfg; //!< 0x18 TX Configuration
	unsigned int								i2ctx; //!< 0x1C TX
	unsigned int								i2cdata; //!< 0x20 Data
	unsigned int								i2cmcn; //!< 0x24 Master Control
	unsigned int								i2cckl; //!< 0x28 Clock Low Divide
	unsigned int								i2cckh; //!< 0x2C Clock High Divide
	unsigned int								i2chsck; //!< 0x30 Hs-Clock Divide
	unsigned int								i2cto; //!< 0x34 Timeout
	unsigned int								i2csla; //!< 0x38 Slave Address

} mml_i2cl_regs_t;

#endif /* __ASSEMBLER__ */

/** @} *//* @defgroup MML_I2CL_REGS */

#endif /* _MML_I2CL_REGS_H_ */

/******************************************************************************/
/* EOF */
