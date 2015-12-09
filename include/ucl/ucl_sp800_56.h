#ifndef UCL_SP80056_H_
#define UCL_SP80056_H_
#include <ucl/bignum_ecdsa_generic_api.h>
#include <ucl/ecdsa_generic_api.h>
#include <ucl/ucl_retdefs.h>
#include <ucl/ucl_types.h>
#include <ucl/ucl_config.h>
#include <ucl/ucl_defs.h>
#include <ucl/ucl_sys.h>
#include <ucl/ucl_sha1.h>
#include <ucl/ucl_sha224.h>
#include <ucl/ucl_sha256.h>
#include <ucl/ucl_sha384.h>
#include <ucl/ucl_sha512.h>
#include <ucl/ucl_rsa.h>
int __API__ ucl_sp800_56_ecc_cdh(u8 *z, u8 *dU, u8 *xV,u8 *yV, u32 configuration);
int __API__ ucl_sp800_56_concat_kdf(u8 *derivedkeyingmaterial, u32 keydatalen, u8 *z, u32 zbytelength, u8 *otherinfo, u32 otherinfobitlength, u32 configuration);
int __API__ ucl_sp800_56_eum_ecc_cdh(u8 *derivedkeyingmaterial, u32 keydatalen, u8 *dU, u8 *xU, u8 *yU, u8 *xV, u8 *yV, u8 *otherinfo, u32 otherinfobitlength, u32 configuration);
#endif
