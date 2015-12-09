/**
 * \file sm3.h
 * thanks to Xyssl
 * SM3 standards:http://www.oscca.gov.cn/News/201012/News_1199.htm
 * author:goldboar
 * email:goldboar@163.com
 * 2011-10-26
 */
#include "ucl/ucl_config.h"
#include "ucl/ucl_defs.h"
#include "ucl/ucl_retdefs.h"
#include "ucl/ucl_types.h"

#ifndef UCL_SM3_H
#define UCL_SM3_H
#include <ucl/ucl_hash.h>
#ifdef HASH_SM3

#define UCL_SM3_BLOCKSIZE 64
/** <b>Hash size</b>.
 * Byte size of the output of SM3.
 *
 * @ingroup UCL_SM3
 */
#define UCL_SM3 7
#define UCL_SM3_HASHSIZE 32


/**
 * \brief          SM3 context structure
 */
typedef struct
{
    u32 total[2];     /*!< number of bytes processed  */
    u32 state[8];     /*!< intermediate digest state  */
    u8 buffer[64];   /*!< data block being processed */

    u8 ipad[64];     /*!< HMAC: inner padding        */
    u8 opad[64];     /*!< HMAC: outer padding        */
}
ucl_sm3_ctx_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief          SM3 context setup
 *
 * \param ctx      context to be initialized
 */
void ucl_sm3_init( ucl_sm3_ctx_t *ctx );

/**
 * \brief          SM3 process buffer
 *
 * \param ctx      SM3 context
 * \param input    buffer holding the  data
 * \param ilen     length of the input data
 */
void ucl_sm3_update( ucl_sm3_ctx_t *ctx, u8 *input, int ilen );

/**
 * \brief          SM3 final digest
 *
 * \param ctx      SM3 context
 */
  void ucl_sm3_finish( u8 *output,ucl_sm3_ctx_t *ctx );

/**
 * \brief          Output = SM3( input buffer )
 *
 * \param input    buffer holding the  data
 * \param ilen     length of the input data
 * \param output   SM3 checksum result
 */
  void ucl_sm3(u8 *output, u8 *input, int ilen);

void ucl_sm3_hmac_start( ucl_sm3_ctx_t *ctx, u8 *key, int keylen);

/**
 * \brief          SM3 HMAC process buffer
 *
 * \param ctx      HMAC context
 * \param input    buffer holding the  data
 * \param ilen     length of the input data
 */
void ucl_sm3_hmac_update(ucl_sm3_ctx_t *ctx, u8 *input, int ilen );

/**
 * \brief          SM3 HMAC final digest
 *
 * \param ctx      HMAC context
 * \param output   SM3 HMAC checksum result
 */
void ucl_sm3_hmac_finish(ucl_sm3_ctx_t *ctx, u8 *output);

/**
 * \brief          Output = HMAC-SM3( hmac key, input buffer )
 *
 * \param key      HMAC secret key
 * \param keylen   length of the HMAC key
 * \param input    buffer holding the  data
 * \param ilen     length of the input data
 * \param output   HMAC-SM3 result
 */
void ucl_sm3_hmac( u8 *key, int keylen, u8 *input, int ilen, u8 *output );


#ifdef __cplusplus
}
#endif//HASH_SM3
#endif//UCL_SM3.h

#endif /* ucl_sm3.h */
