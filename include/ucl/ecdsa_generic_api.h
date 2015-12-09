/** @file ecdsa_generic_api.h
 * @defgroup UCL_ECDSA ECDSA
 * Elliptic Curve Digital signature Algorithm.
 *
 * ECDSA is the implementation of DSA using the ECC.
 *
 * @par Header:
 * @link ecdsa_generic_api.h ecdsa_generic_api.h @endlink
 *
 *
 * @ingroup UCL_ECC
 */

#ifndef _UCL_ECDSA_GENERIC_API_H_
#define _UCL_ECDSA_GENERIC_API_H_
#include "ucl/ucl_config.h"
#include "ucl/ucl_types.h"
#include "ucl/bignum_ecdsa_generic_api.h"

/* ECDSA key lengths */
//#define ECDSA_BLOCK_SIZE 256/8
//#define ECDSA_BLOCK_SIZE 32
//#define ECDSA_BLOCK_SIZE 192/8
//#define ECDSA_BLOCK_SIZE 32
#define ECDSA_BLOCK_SIZE 32
//sufficient for p192
//#define ECDSA_DIGITS 13
//sufficient for p2
#ifdef WORD32
#define ECDSA_DIGITS 17
#endif
#ifdef WORD16
#define ECDSA_DIGITS 40
#endif

#define SECP192R1 0
#define SECP224R1 1
#define SECP256R1 2
#define SECP160R1 3
#define SECP384R1 4
#define SECP521R1 5
#define SM2FP192 6
#define SM2FP256 7
#define BP256R1 8
#define BP384R1 9
#define BP512R1 10
#define UNKNOWN_CURVE 10

#define SECP160R1_BYTESIZE 20
#define SECP192R1_BYTESIZE 24
#define SM2FP192_BYTESIZE 24
#define SECP224R1_BYTESIZE 28
#define SECP256R1_BYTESIZE 32
#define BP256R1_BYTESIZE 32
#define SM2FP256_BYTESIZE 32
#define SECP384R1_BYTESIZE 48
#define SECP521R1_BYTESIZE 66
#define BP384R1_BYTESIZE 48
#define BP512R1_BYTESIZE 64

#ifdef WORD16
#define SECP160R1_WORDSIZE 16
#define SECP192R1_WORDSIZE 16
#define SM2FP192_WORDSIZE 16
#define SECP224R1_WORDSIZE 16
#define SECP256R1_WORDSIZE 16
#define BP256R1_WORDSIZE 16
#define SM2FP256_WORDSIZE 16
#define SECP384R1_WORDSIZE 24
#define SECP521R1_WORDSIZE 34
#define BP384R1_WORDSIZE 24
#define BP512R1_WORDSIZE 32
#endif//WORD16

#ifdef WORD32
#define SECP160R1_WORDSIZE 8
#define SECP192R1_WORDSIZE 8
#define SM2FP192_WORDSIZE 8
#define SECP224R1_WORDSIZE 8
#define SECP256R1_WORDSIZE 8
#define BP256R1_WORDSIZE 8
#define SM2FP256_WORDSIZE 8
#define SECP384R1_WORDSIZE 12
#define SECP521R1_WORDSIZE 17
#define BP384R1_WORDSIZE 12
#define BP512R1_WORDSIZE 16
#endif//WORD32

//internal defines

#if defined(MAXQ1060)
//#define P224
#define P384
#define P521
//#define BP256
//#define BP384
//#define BP512
//#define P192
#define P256

#else//!MAXQ1060

#ifndef PROFILE_2
#ifndef PROFILE_1
#define SM2P192
#define SM2P256
#define P384
#define P521
#define BP256
#define BP384
#define BP512
#endif//PROFILE_1
#define P224
#define P160
#define P192
#endif//PROFILE_2
#define P256
#endif//1060

#ifdef WORD32
static const DIGIT one[ECDSA_DIGITS]={0x00000001,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};
static const DIGIT two[ECDSA_DIGITS]={0x00000002,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};
static const DIGIT three[ECDSA_DIGITS]={0x00000003,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};
static const DIGIT four[ECDSA_DIGITS]={0x00000004,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};
#endif
#ifdef WORD16
#ifndef MAXQ1060
static const DIGIT one[ECDSA_DIGITS]={0x0001,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
static const DIGIT two[ECDSA_DIGITS]={0x0002,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
static const DIGIT three[ECDSA_DIGITS]={0x0003,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
static const DIGIT four[ECDSA_DIGITS]={0x0004,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
#endif//1060
#endif//WORD16

#ifndef MAXQ1060
 extern __API__ void *precompx;
 extern __API__ void *precompy;
#endif//1060

int __API__ ucl_ecdsa_sign(u8 *r_and_others,u8 *s,u8 *d,u8 *input, u32 inputlength, u32 configuration);


#ifdef P160
int __API__ ucl_ecdsa_sign_p160r1_sha1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_p160r1_sha256(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
void __API__ precomputation_p160_Q(DIGIT qx[40][8],DIGIT qy[40][8],DIGIT *x,DIGIT *y,DIGIT *p, unsigned int digits);
int __API__ ucl_ecdsa_verify_p160r1_sha1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_p160r1_sha256(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int ucl_ecdsa_verify_p160r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
#endif

#ifdef P192
int __API__ ucl_ecies_decrypt_p192r1_aes_hmac_sha256(u8 *m,u32 keylength,u8 * xG,u8 *yG,u8 *a,u8 *n,u8 *p, u8 *d,u8 *rx,u8 *ry, u8 *crypto,int crypto_len,u8 *t);
int __API__ ucl_ecies_encrypt_p192r1_aes_hmac_sha256(u8 *rx,u8 *ry, u8 *crypto, u8 *t,u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_p192r1_sha1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_p192r1_sha256(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_p192r1_sia256(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
void __API__ precomputation_p192_Q(DIGIT qx[48][8],DIGIT qy[48][8],DIGIT *x,DIGIT *y,DIGIT *p, unsigned int digits);
int __API__ ucl_ecdsa_verify_p192r1_sha1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_p192r1_sha256(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_p192r1_sia256(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int ucl_ecdsa_verify_p192r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
#endif

#ifdef P224
int __API__ ucl_ecdsa_sign_p224r1_sha1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_p224r1_sha256(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_p224r1_sha224(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
void __API__ precomputation_p224_Q(DIGIT qx[56][8],DIGIT qy[56][8],DIGIT *x,DIGIT *y,DIGIT *p, unsigned int digits);
int __API__ ucl_ecdsa_verify_p224r1_sha1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_p224r1_sha224(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int ucl_ecdsa_verify_p224r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
#endif

#ifdef P256
int __API__ ucl_ecdsa_sign_p256r1_sha1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_p256r1_sha256(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
void __API__ precomputation_p256_Q(DIGIT qx[64][SECP256R1_WORDSIZE],DIGIT qy[64][SECP256R1_WORDSIZE],DIGIT *x,DIGIT *y,DIGIT *p, unsigned int digits);
int __API__ ucl_ecdsa_verify_p256r1_sha1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_p256r1_sha256(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int ucl_ecdsa_verify_p256r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_verify_p256r1_sha256_with_precomp(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
#endif

#ifdef BP256
int __API__ ucl_ecdsa_sign_bp256r1_sha1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_bp256r1_sha256(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
void __API__ precomputation_bp256_Q(DIGIT qx[64][8],DIGIT qy[64][8],DIGIT *x,DIGIT *y,DIGIT *p, DIGIT digits);
int __API__ ucl_ecdsa_verify_bp256r1_sha256(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_bp256r1_sha1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int ucl_ecdsa_verify_bp256r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
#endif

#ifdef P384
int __API__ ucl_ecdsa_sign_p384r1_sha1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_p384r1_sha256(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_p384r1_sha384(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
void __API__ precomputation_p384_Q(DIGIT qx[96][8],DIGIT qy[96][8],DIGIT *x,DIGIT *y,DIGIT *p, unsigned int digits);
int __API__ ucl_ecdsa_verify_p384r1_sha1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_p384r1_sha256(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_p384r1_sha384(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
#endif

#ifdef BP384
int __API__ ucl_ecdsa_sign_bp384r1_sha1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_bp384r1_sha256(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_bp384r1_sha384(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_bp384r1_sha1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_bp384r1_sha256(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_bp384r1_sha384(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int ucl_ecdsa_verify_bp384r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
int ucl_ecdsa_verify_p384r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
#endif

#ifdef P521
int __API__ ucl_ecdsa_sign_p521r1_sha1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_p521r1_sha256(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_p521r1_sha512(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_p521r1_sha1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_p521r1_sha256(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_p521r1_sha512(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int ucl_ecdsa_verify_p521r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
#endif

#ifdef BP512
int __API__ ucl_ecdsa_sign_bp512r1_sha1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_bp512r1_sha256(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_bp512r1_sha384(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_sign_bp512r1_sha512(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_bp512r1_sha1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_bp512r1_sha256(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_bp512r1_sha384(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int __API__ ucl_ecdsa_verify_bp512r1_sha512(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 * yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *m,u32 MsgLng);
int ucl_ecdsa_verify_bp512r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
#endif

int __API__ ucl_ecdsa_verify(u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *input,u32 inputlength,u32 configuration);
int __API__ ucl_ecdsa_verify_with_full_precomp(DIGIT px[64][8],DIGIT py[64][8],DIGIT qx[64][8],DIGIT qy[64][8],u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *input,u32 inputlength,u32 configuration);
int precomputation_Q_generic(u32 rsize,u32 digits,DIGIT qx[96][8],DIGIT qy[96][8],DIGIT *x,DIGIT *y,DIGIT *p,u32 curve);

void __API__ ecc_ellipticmult(DIGIT *xres,DIGIT *yres, DIGIT *m, DIGIT *a,DIGIT *x,DIGIT *y,DIGIT *p, unsigned digits,int curve);
void __API__ ecc_mod192(DIGIT *b,DIGIT *c,DIGIT cDigits,DIGIT *p,DIGIT pDigits);
int __API__ ecc_modsub(DIGIT *a, DIGIT *b, DIGIT *c, DIGIT *p, DIGIT digits, u8 Mod,int curve);
int __API__ ecc_modleftshift(DIGIT *a,DIGIT *b,DIGIT c,DIGIT digits,DIGIT *p,DIGIT pDigits,int curve);
int __API__ ecc_modmult(DIGIT *r,DIGIT *a,DIGIT *b,DIGIT *m,DIGIT k,int curve);
void __API__ ecc_modmultscalar(DIGIT *r,DIGIT a,DIGIT *b,DIGIT *m,DIGIT k,int curve);
int __API__ ecc_modsquare(DIGIT *r,DIGIT *a,DIGIT *m,DIGIT k,int curve);
void __API__ ecc_mod256(DIGIT *b,DIGIT *c,DIGIT cDigits,DIGIT *p,DIGIT pDigits);
void __API__ ecc_mod224(DIGIT *b,DIGIT *c,DIGIT cDigits,DIGIT *p,DIGIT pDigits);
int __API__ bignum_modsub(DIGIT *a, DIGIT *b, DIGIT *c, DIGIT *p, DIGIT digits);
int __API__ ecc_infinite_affine(DIGIT *x,DIGIT *y,int digits,int curve);
int __API__ ecc_mult_affine(DIGIT *xq,DIGIT *yq, DIGIT *m, DIGIT *a,DIGIT *x,DIGIT *y,DIGIT *p, unsigned digits,int curve);
int __API__ ecc_infinite_jacobian(DIGIT *x,DIGIT *y,DIGIT *z,int digits,int curve);
int __API__ ecc_double_jacobian(DIGIT *x3,DIGIT *y3,DIGIT *z3,DIGIT *x1,DIGIT *y1,DIGIT *z1,DIGIT *p, DIGIT digits,int curve);
int __API__ ecc_add_jacobian_affine(DIGIT *x3,DIGIT *y3,DIGIT *z3,DIGIT *x1,DIGIT *y1,DIGIT *z1,DIGIT *x2,DIGIT *y2,DIGIT *p,DIGIT digits,int curve);
int __API__ ecc_convert_affine_to_jacobian(DIGIT *xq,DIGIT *yq,DIGIT *zq,DIGIT *x,DIGIT *y,int digits,int curve);
int __API__ ecc_convert_jacobian_to_affine(DIGIT *x,DIGIT *y,DIGIT *xq,DIGIT *yq,DIGIT *zq,DIGIT *p,int digits,int curve);
int __API__ ecc_mult_jacobian(DIGIT *xq,DIGIT *yq, DIGIT *m, DIGIT *a,DIGIT *x,DIGIT *y,DIGIT *p, DIGIT digits,int curve);

#ifdef P160
void __API__ ecc_mult_jacobian_window4_p160(DIGIT *xres,DIGIT *yres, DIGIT *m, DIGIT *a,DIGIT *x,DIGIT *y,DIGIT *p, DIGIT digits,int curve);
#endif//P160
#ifdef P192
void __API__ ecc_mult_jacobian_window4_p192(DIGIT *xres,DIGIT *yres, DIGIT *m, DIGIT *a,DIGIT *x,DIGIT *y,DIGIT *p, DIGIT digits,int curve);
#endif//P192
#ifdef P256
void __API__ ecc_mult_jacobian_window4_p256(DIGIT *xres,DIGIT *yres, DIGIT *m, DIGIT *a,DIGIT *x,DIGIT *y,DIGIT *p, DIGIT digits,int curve);
#endif//P256
#ifdef P384
void __API__ ecc_mult_jacobian_window4_p384(DIGIT *xres,DIGIT *yres, DIGIT *m, DIGIT *a,DIGIT *x,DIGIT *y,DIGIT *p, DIGIT digits,int curve);
#endif//P384
#ifdef P224
void __API__ ecc_mult_jacobian_window4_p224(DIGIT *xres,DIGIT *yres, DIGIT *m, DIGIT *a,DIGIT *x,DIGIT *y,DIGIT *p, DIGIT digits,int curve);
#endif//P224
#ifdef P521
void __API__ ecc_mult_jacobian_window4_p521(DIGIT *xres,DIGIT *yres, DIGIT *m, DIGIT *a,DIGIT *x,DIGIT *y,DIGIT *p, DIGIT digits,int curve);
#endif//P521

//int __API__ ucl_ecdsa_sign(u8 *r_and_others,u8 *s,u8 *d,u8 *input, u32 inputlength, u32 configuration);
//int __API__ ucl_ecdsa_verify(u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *input,u32 inputlength,int configuration);

#ifdef P256
int __API__ ucl_ecdsa_sign_p256r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_verify_p256r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_r_precomp_sign_p256r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r, u8 *kminus1, u8 *a,u8 *n, u8 *p,u8 *d);
int __API__ ucl_ecdsa_sign_p256r1_r_precomp(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *kminus1, u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
#endif

#ifdef BP256
int __API__ ucl_ecdsa_sign_bp256r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_verify_bp256r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_r_precomp_sign_bp256r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r, u8 *kminus1, u8 *a,u8 *n, u8 *p,u8 *d);
int __API__ ucl_ecdsa_sign_bp256r1_r_precomp(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *kminus1, u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
#endif

#ifdef P224
int __API__ ucl_ecdsa_sign_p224r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_verify_p224r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_r_precomp_sign_p224r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r, u8 *kminus1, u8 *a,u8 *n, u8 *p,u8 *d);
int __API__ ucl_ecdsa_sign_p224r1_r_precomp(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *kminus1, u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
#endif
#ifdef BP384
int __API__ ucl_ecdsa_sign_bp384r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_verify_bp384r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_r_precomp_sign_bp384r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r, u8 *kminus1, u8 *a,u8 *n, u8 *p,u8 *d);
int __API__ ucl_ecdsa_sign_bp384r1_r_precomp(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *kminus1, u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
#endif
#ifdef BP512
int __API__ ucl_ecdsa_sign_bp512r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_verify_bp512r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_r_precomp_sign_bp512r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r, u8 *kminus1, u8 *a,u8 *n, u8 *p,u8 *d);
int __API__ ucl_ecdsa_sign_bp512r1_r_precomp(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *kminus1, u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
#endif

#ifdef P192
int __API__ ucl_ecdsa_sign_p192r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_verify_p192r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_r_precomp_sign_p192r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r, u8 *kminus1, u8 *a,u8 *n, u8 *p,u8 *d);
int __API__ ucl_ecdsa_sign_p192r1_r_precomp(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *kminus1, u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
#endif

#ifdef P384
int __API__ ucl_ecdsa_sign_p384r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_verify_p384r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_r_precomp_sign_p384r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r, u8 *kminus1, u8 *a,u8 *n, u8 *p,u8 *d);
int __API__ ucl_ecdsa_sign_p384r1_r_precomp(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *kminus1, u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
#endif

#ifdef P521
int __API__ ucl_ecdsa_sign_p521r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_verify_p521r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_r_precomp_sign_p521r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r, u8 *kminus1, u8 *a,u8 *n, u8 *p,u8 *d);
int __API__ ucl_ecdsa_sign_p521r1_r_precomp(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *kminus1, u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
#endif

#ifdef P160
int __API__ ucl_ecdsa_sign_p160r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_verify_p160r1(u32 keylength,u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *s,u8 *a,u8 *n,u8 *p, u8 *hash,u32 hashlength);
int __API__ ucl_ecdsa_r_precomp_sign_p160r1(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r, u8 *kminus1, u8 *a,u8 *n, u8 *p,u8 *d);
int __API__ ucl_ecdsa_sign_p160r1_r_precomp(u32 keylength, u8 * xG,u8 *yG,u8 *xQ,u8 *yQ,u8 *r,u8 *kminus1, u8 *s,u8 *a,u8 *n, u8 *p,u8 *d,u8 *m,u32 MsgLng);
#endif

#ifdef SM2P256
int __API__ ucl_sm2dsa_sign(u8 *r,u8 *s,u8 *d,u8 *xA,u8 *yA,u8 *input, u32 inputlength, u8 *ida, u8 *entla,u32 configuration);
int __API__ ucl_sm2dsa_verify(u8 *r,u8 *s,u8 *xA,u8 *yA,u8 *input,u32 inputlength,u8 *ida,u8 *entla,u32 configuration);
int __API__ ucl_sm2dsa_sign_sm2fp256_sm3(u8 *r,u8 *s,u8 *d,u8 *xQ,u8 * yQ,u8 *m,u32 MsgLng,u8 *ida,u8 *entla);
int __API__ ucl_sm2dsa_verify_sm2fp256_sm3(u8 *r,u8 *s,u8 *xQ,u8 * yQ,u8 *m,u32 MsgLng,u8 *ida,u8 *entla);
#endif
#endif//ECDSA_GENERIC_API
