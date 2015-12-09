#ifndef UCL_ECCKEYGEN_H_
#define UCL_ECCKEYGEN_H_
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
int __API__ ucl_ecc_keygen_generic(u8 *xr,u8 *yr,u8 *d,int curve);
int __API__ ucl_ecc_keygen_p160r1(u8 *xr,u8 *yr,u8 *d);
int __API__ ucl_ecc_keygen_p192r1(u8 *xr,u8 *yr,u8 *d);
int __API__ ucl_ecc_keygen_p224r1(u8 *xr,u8 *yr,u8 *d);
int __API__ ucl_ecc_keygen_p256r1(u8 *xr,u8 *yr,u8 *d);
int __API__ ucl_ecc_keygen_p384r1(u8 *xr,u8 *yr,u8 *d);
int __API__ ucl_ecc_keygen_bp256r1(u8 *xr,u8 *yr,u8 *d);
int __API__ ucl_ecc_keygen_bp384r1(u8 *xr,u8 *yr,u8 *d);
int __API__ ucl_ecc_keygen_bp512r1(u8 *xr,u8 *yr,u8 *d);
int __API__ ucl_ecc_keygen_p521r1(u8 *xr,u8 *yr,u8 *d);

int __API__ ucl_ecc_on_curve_p160r1(u8 *x,u8 *y, u8 *a,u8 *b, u8 *p);
int __API__ ucl_ecc_on_curve_p192r1(u8 *x,u8 *y, u8 *a,u8 *b, u8 *p);
int __API__ ucl_ecc_on_curve_p224r1(u8 *x,u8 *y, u8 *a,u8 *b, u8 *p);
int __API__ ucl_ecc_on_curve_p256r1(u8 *x,u8 *y, u8 *a,u8 *b, u8 *p);
int __API__ ucl_ecc_on_curve_bp256r1(u8 *x,u8 *y, u8 *a,u8 *b, u8 *p);
int __API__ ucl_ecc_on_curve_p384r1(u8 *x,u8 *y, u8 *a,u8 *b, u8 *p);
int __API__ ucl_ecc_on_curve_bp384r1(u8 *x,u8 *y, u8 *a,u8 *b, u8 *p);
int __API__ ucl_ecc_on_curve_bp512r1(u8 *x,u8 *y, u8 *a,u8 *b, u8 *p);
int __API__ ucl_ecc_on_curve_p521r1(u8 *x,u8 *y, u8 *a,u8 *b, u8 *p);

int __API__ ucl_ecc_on_curve_generic(u8 *x,u8 *y, u8 *a,u8 *b, u8 *p,int curve);

#endif
