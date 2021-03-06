#ifndef UCL_X931_H_
#define UCL_X931_H_
#include <ucl/ucl_retdefs.h>
#include <ucl/ucl_types.h>
#include <ucl/ucl_config.h>
#include <ucl/ucl_defs.h>
#include <ucl/ucl_sys.h>
#include <ucl/ucl_rsa.h>

int ucl_x931_encode(u8 *EM, u8 *message, u32 message_length, u32 k);
int __API__ ucl_x931_ssa_sign(u8 *signature,u8 *message,u32 message_length,ucl_rsa_private_key_t *keyPr);
int __API__ ucl_x931_ssa_crt_sign(u8 *signature,u8 *message,u32 message_length,ucl_rsa_crt_private_key_t *keyPr);
int __API__ ucl_x931_ssa_verify(u8 *signature,u8 *message,u32 message_length,ucl_rsa_public_key_t *keyPu);
int __API__ ucl_x931_verify(u8 *EM, u8 *message, u32 message_length, u32 k);

#endif//X931
