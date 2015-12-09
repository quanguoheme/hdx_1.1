/*
 * jibe_maa.h --
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
 * Created on: 08-Jun-2012
 * Author:
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: 4487     $:  Revision of last commit
 * $Author:: jeremy.kon#$:  Author of last commit
 * $Date:: 2015-01-14 1#$:  Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */


#if defined (__jibe) && !defined(__linux)

#ifndef JIBE_CRYPTO_H_
#define JIBE_CRYPTO_H_

// MAA operand types
typedef enum {
        OPERAND_A,
        OPERAND_B,
        OPERAND_E,
        OPERAND_N,
        OPERAND_R
}E_OPERANDS;

// MAA operations
typedef enum {

        A_POW_E_MODN,
        B_SQR_MODN,
        A_MULT_B_MODN,
        B_SQR_A_MODN,
        A_PLUS_B_MODN,
        A_MINUS_B_MODN,
        A_POW_E_MODN_OCALC,                     // with OCALC eset
        FAIL_TEST                               // FAILURE

}E_OPERATIONS;

//typedef unsigned short DIGIT;
#define MAX_INCOMING_DATA       (256+3)

// JIBE CRYPTO CTL bits
#define CRYPTO_MAA_DONE (1<<28)
#define CRYPTO_DONE     (unsigned int)(1<<31)
#define CRYPTO_READY    (1<<30)
#define CRYPTO_BUSERR   (1<<29)

#define MAA_CLR_DONE    (0xefffffff)

#define MAA_BASE        0xFFE50000

// JIBE CRYPTO block address
#define TRNG_BASE			0xFFF61000
#define	TRNG_TRNGC			TRNG_BASE + 0x0
#define AES_KEY_GEN			0x43

//Crypto
#define  CRYPTO_CTL  (MAA_BASE + 0x7000)
#define CRYPTO_BASE 		0xFFE57000
#define CRYPTO_CIPHER_CTL	CRYPTO_BASE + 0x04
#define HASH_CTRL               CRYPTO_BASE + 0x08
#define CIPHER_DATA_IN_0	CRYPTO_BASE + 0x20
#define CIPHER_DATA_IN_1	CRYPTO_BASE + 0x24
#define CIPHER_DATA_IN_2	CRYPTO_BASE + 0x28
#define CIPHER_DATA_IN_3	CRYPTO_BASE + 0x2C
#define CIPHER_DATA_OUT_0	CRYPTO_BASE + 0x30
#define CIPHER_DATA_OUT_1	CRYPTO_BASE + 0x34
#define CIPHER_DATA_OUT_2	CRYPTO_BASE + 0x38
#define CIPHER_DATA_OUT_3	CRYPTO_BASE + 0x3C
#define CIPHER_KEY0			CRYPTO_BASE + 0x60
#define CIPHER_KEY1			CRYPTO_BASE + 0x64
#define CIPHER_KEY2			CRYPTO_BASE + 0x68
#define CIPHER_KEY3			CRYPTO_BASE + 0x6C
#define CIPHER_KEY4			CRYPTO_BASE + 0x70
#define CIPHER_KEY5			CRYPTO_BASE + 0x74
#define CIPHER_KEY6			CRYPTO_BASE + 0x78
#define CIPHER_KEY7			CRYPTO_BASE + 0x7C

// JIBE MAA memory addresses
#define MAA_CTL  (MAA_BASE + 0x701C)   // MAA CNTL offset
#define MAA_MAWS (MAA_BASE + 0x70D0)   // MAWS
#define MAA_A (MAA_BASE + 0x7100)       // MAA_A memory offset
#define MAA_B (MAA_BASE + 0x7200)       // MAA_B memory offset
#define MAA_R (MAA_BASE + 0x7300)       // MAA_R memory offset
#define MAA_T (MAA_BASE + 0x7400)       // MAA_T memory offset
#define MAA_E (MAA_BASE + 0x7500)       // MAA_E memory offset
#define MAA_M (MAA_BASE + 0x7600)       // MAA_M memory offset

#define MAA_SEGA 0
#define MAA_SEGB 2
#define MAA_SEGR 4
#define MAA_SEGT 6
#define MAA_SEGE 8

#define MAA_SHIFT_RMA 24
#define MAA_SHIFT_AMA 16
#define MAA_SHIFT_BMA 20
#define MAA_SHIFT_TMA 28

#endif /* JIBE_CRYPTO_H_ */
#endif//_jibe
