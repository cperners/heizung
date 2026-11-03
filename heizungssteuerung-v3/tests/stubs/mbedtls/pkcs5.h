#pragma once
#include <stdint.h>
#include <stddef.h>
#define MBEDTLS_MD_SHA256 6
int mbedtls_pkcs5_pbkdf2_hmac_ext(int,const unsigned char*,size_t,const unsigned char*,size_t,unsigned,unsigned,unsigned char*);
