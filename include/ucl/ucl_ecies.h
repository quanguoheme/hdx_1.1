#ifndef UCL_ECIES_H_
#define UCL_ECIES_H_
#include <ucl/ucl_retdefs.h>
#include <ucl/ucl_types.h>
#include <ucl/ucl_config.h>
#include <ucl/ucl_defs.h>
#include <ucl/ucl_sys.h>
int ucl_ecies_encrypt_p192r1_aes_hmac_sha256(unsigned char *rx,unsigned char *ry, unsigned char *crypto, unsigned char *t,unsigned int keylength,unsigned char * xG,unsigned char *yG,unsigned char *xQ,unsigned char * yQ,unsigned char *a,unsigned char *n,unsigned char *p, unsigned char *m,unsigned MsgLng);
int ucl_ecies_decrypt_p192r1_aes_hmac_sha256(unsigned char *m,unsigned int keylength,unsigned char * xG,unsigned char *yG,unsigned char *a,unsigned char *n,unsigned char *p, unsigned char *d,unsigned char *rx,unsigned char *ry, unsigned char *crypto,int crypto_len,unsigned char *t);

#endif//UCL_ECIES_H
