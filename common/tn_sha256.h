/********************************************************************************************************
 * @file	tn_sha256.h
 *
 * @brief	This is the header file for TLSR8232
 *
 * @author	Driver Group
 * @date	May 8, 2018
 *
 * @par     Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
 *
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions are met:
 *
 *              1. Redistributions of source code must retain the above copyright
 *              notice, this list of conditions and the following disclaimer.
 *
 *              2. Unless for usage inside a TELINK integrated circuit, redistributions
 *              in binary form must reproduce the above copyright notice, this list of
 *              conditions and the following disclaimer in the documentation and/or other
 *              materials provided with the distribution.
 *
 *              3. Neither the name of TELINK, nor the names of its contributors may be
 *              used to endorse or promote products derived from this software without
 *              specific prior written permission.
 *
 *              4. This software, with or without modification, must only be used with a
 *              TELINK integrated circuit. All other usages are subject to written permission
 *              from TELINK and different commercial license may apply.
 *
 *              5. Licensee shall be solely responsible for any claim to the extent arising out of or
 *              relating to such deletion(s), modification(s) or alteration(s).
 *
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *          ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *          WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *          DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 *          DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *          LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *          ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *          (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *          SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************************************/
#ifndef _TN_SHA256_H_
#define _TN_SHA256_H_
#include "string.h"
#include "types.h"
//#include "tn_dtls_include.h"
//typedef unsigned int u32;
#define SHA256_BLOCK_LENGTH       64
#define SHA256_DIGEST_LENGTH      32

typedef struct
{
    unsigned int total[2];
    unsigned int state[8];
    unsigned char buffer[64];
} sha256_context;

void sha256_init( sha256_context *ctx );

void sha256_starts( sha256_context *ctx, int is224 );

void sha256_update( sha256_context *ctx, const unsigned char *input, u32 ilen );

void sha256_finish( sha256_context *ctx, unsigned char output[32] );

void sha256( const unsigned char *input, u32 ilen, unsigned char output[32], int is224 );

void sha256_hmac( const unsigned char *key, u32 keylen,
                  const unsigned char *input, u32 ilen,
                  unsigned char output[32] );

unsigned char tl_hmac_sha256(unsigned char *key, int counter,  unsigned char *kmac, unsigned char *kmle);
// stack size: 300-byte

int sha256_self_test( int verbose );

#endif /* _TN_SHA256_H_ */
