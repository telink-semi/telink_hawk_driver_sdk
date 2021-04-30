/********************************************************************************************************
 * @file	tn_sha256.c
 *
 * @brief	This is the source file for TLSR8232
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
#include "tn_sha256.h"
#define     tl_printf       my_printf

typedef unsigned int    uint32_t;

//////////////////////////////////////////////
const unsigned int tl_sha256_hinit[8] = {
    0x6A09E667, 0xBB67AE85, 0x3C6EF372, 0xA54FF53A, 0x510E527F, 0x9B05688C, 0x1F83D9AB, 0x5BE0CD19
};

const unsigned int tl_sha256_K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b,
    0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01,
    0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7,
    0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152,
    0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
    0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
    0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819,
    0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08,
    0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f,
    0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

inline  unsigned int sha256_shr (unsigned int x, int n)
  {
      //return ((x & 0xFFFFFFFF) >> n);
      return x >> n;
  }

__attribute__((noinline)) unsigned int rot(unsigned long x, unsigned long n)
{
   volatile unsigned int r = x >> n;
   volatile unsigned int l = x << (32 - n);
   //volatile unsigned int l =0;
   return r | l;
}


inline  unsigned int sha256_rot (unsigned int x, int n)
  {
	  //return (x >> n) | (x << (32 - n));
	  return rot(x,n);
  }



unsigned int sha256_test(unsigned long x,unsigned long n)
{
	return sha256_rot(x,n);
}

//  #define S0(x) (ROTR(x, 7) ^ ROTR(x,18) ^  SHR(x, 3))
inline unsigned int sha256_s0 (unsigned int x)
 {
    return sha256_rot(x, 7) ^ sha256_rot(x, 18) ^ sha256_shr(x, 3);
 }

 //#define S1(x) (ROTR(x,17) ^ ROTR(x,19) ^  SHR(x,10))
inline unsigned int sha256_s1 (unsigned int x)
 {
    return sha256_rot(x, 17) ^ sha256_rot(x, 19) ^ sha256_shr(x, 10);
 }

 //#define S2(x) (ROTR(x, 2) ^ ROTR(x,13) ^ ROTR(x,22))
inline unsigned int sha256_s2 (unsigned int x)
 {
    return sha256_rot(x, 2) ^ sha256_rot(x, 13) ^ sha256_rot(x, 22);
 }

 //#define S3(x) (ROTR(x, 6) ^ ROTR(x,11) ^ ROTR(x,25))
inline unsigned int sha256_s3 (unsigned int x)
 {
    return sha256_rot(x, 6) ^ sha256_rot(x, 11) ^ sha256_rot(x, 25);
 }

// #define F0(x,y,z) ((x & y) | (z & (x | y)))
inline unsigned int sha256_f0 (unsigned int x, unsigned int y, unsigned int z)
 {
    return ((x & y) | (z & (x | y)));
 }

//#define F1(x,y,z) (z ^ (x & (y ^ z)))
inline unsigned int sha256_f1 (unsigned int x, unsigned int y, unsigned int z)
 {
    return (z ^ (x & (y ^ z)));
 }

  void sha256_process( sha256_context *ctx, const unsigned char data[64] )
{
    uint32_t temp1, temp2, W[32], hash[16], *pd, *pw;
    int i;
    for (i=0; i<16; i++)
    {
        W[i] = (data[i*4]<<24) + (data[i*4+1]<<16) + (data[i*4+2]<<8) + data[i*4+3];
    }

    memcpy (hash + 8, ctx->state, 32);
    pd = hash + 8;
    for (i=0; i<16; i++)
    {
        temp1 = pd[7] + sha256_s3(pd[4]) + sha256_f1(pd[4], pd[5], pd[6]) + tl_sha256_K[i] + W[i];
        temp2 = sha256_s2(pd[0]) + sha256_f0(pd[0], pd[1], pd[2]);
        pd[3] += temp1;
        pd[7] = temp1 + temp2;

        pd --;
        pd[0] = pd[8];

        if ((i & 7) == 7)
        {
            memcpy (hash + 8, hash, 32);
            pd += 8;
        }

    }

    pw = W;
    for (i=16; i<64; i++)
    {
        pw[16] = sha256_s1(pw[14]) + pw[9] +  sha256_s0(pw[1]) + pw[0];

        temp1 = pd[7] + sha256_s3(pd[4]) + sha256_f1(pd[4], pd[5], pd[6]) + tl_sha256_K[i] + pw[16];
        temp2 = sha256_s2(pd[0]) + sha256_f0(pd[0], pd[1], pd[2]);
        pd[3] += temp1;
        pd[7] = temp1 + temp2;

        pd --;
        pd[0] = pd[8];

        if ((i & 7) == 7)
        {
            memcpy (hash + 8, hash, 32);
            pd += 8;
        }

        pw++;
        if ( (i & 15) == 15)
        {
            memcpy (W, W + 16, 64);
            pw -= 16;
        }

    }

    for (i = 0; i<8; i++)
    {
        ctx->state[i] += hash[i];
    }
}


void tl_sha256_zero( void *v, u32 n ) {
    volatile unsigned char *p = (unsigned char *)v; while( n-- ) *p++ = 0;
}

void sha256_init( sha256_context *ctx )
{
    memset( ctx, 0, sizeof( sha256_context ) );
}

void sha256_starts( sha256_context *ctx, int is224)
{
    ctx->total[0] = 0;
    ctx->total[1] = 0;

    memcpy (ctx->state, tl_sha256_hinit, 32);

}

static void sha256_free( sha256_context *ctx )
{

}

void sha256_update( sha256_context *ctx, const unsigned char *input, u32 ilen )
{
    uint32_t left;

    if( ilen == 0 )
        return;

    left = ctx->total[0] & 0x3F;

    ctx->total[0] += (uint32_t) ilen;

    while (ilen + left >= 64)
    {
        if( left )
        {
            ilen += left;
            memcpy( (void *) (ctx->buffer + left), input, 64 - left );
            sha256_process( ctx, ctx->buffer );
            input += 64 - left;
            left = 0;
        }
        else
        {
            sha256_process( ctx, input );
            input += 64;
        }
        ilen  -= 64;
    }

    if( ilen > 0 )
        memcpy( (void *) (ctx->buffer + left), input, ilen );
}

// SHA-256 finish
void sha256_finish( sha256_context *ctx, unsigned char output[32] )
{
    uint32_t i, len;
    unsigned char buff[64];

    len = ctx->total[0] <<  3;

    memset (buff, 0, 64);
    buff[0] = 0x80;
    i = (ctx->total[0] & 0x3F);
    sha256_update( ctx, buff, i < 56 ? ( 56 - i ) : ( 120 - i ));

    memset (buff, 0, 4);
    buff[4] = len >> 24;
    buff[5] = len >> 16;
    buff[6] = len >> 8;
    buff[7] = len >> 0;

    sha256_update( ctx, buff, 8);

    for (i=0; i<32; i+=4)
    {
        unsigned int d = ctx->state[i >> 2];
        output[i + 0] = d >> 24;
        output[i + 1] = d >> 16;
        output[i + 2] = d >> 8;
        output[i + 3] = d >> 0;
    }
}

unsigned long  sha256_all;
#define   DEBUG_TIME_BEGIN   
#define   DEBUG_TIME_END(a) 

void sha256( const unsigned char *input, u32 ilen, unsigned char output[32], int is224 )
{
    sha256_context ctx;
    DEBUG_TIME_BEGIN
    sha256_init( &ctx );
    sha256_starts( &ctx, is224 );
    sha256_update( &ctx, input, ilen );
    sha256_finish( &ctx, output );
    DEBUG_TIME_END(sha256)
}

// output = HMAC-SHA-256( hmac key, input buffer )
void sha256_hmac( const unsigned char *key, u32 keylen, const unsigned char *input, u32 ilen, unsigned char output[32] )
{
    sha256_context ctx;
    u32 i;
    unsigned char sum[32];
    unsigned char pad[64];

    sha256_init( &ctx );

    //sha256_hmac_starts( &ctx, key, keylen, 0 );
    if( keylen > 64 )
    {
        sha256( key, keylen, sum, 0 );
        keylen = 32;
        key = sum;
    }

    for( i = 0; i < 64; i++ )
    {
        pad[i] = i < keylen ? (unsigned char)( 0x36 ^ key[i] ) : 0x36;
    }

    sha256_starts( &ctx, 0 );
    sha256_update( &ctx, pad, 64 );

    //sha256_hmac_update( &ctx, input, ilen );
    sha256_update( &ctx, input, ilen );

    //sha256_hmac_finish( &ctx, output );
    for( i = 0; i < 64; i++ )
    {
        pad[i] = i < keylen ? (unsigned char)( 0x5c ^ key[i] ) : 0x5c;
    }

    sha256_finish( &ctx, sum );
    sha256_starts( &ctx, 0 );
    sha256_update( &ctx, pad, 64 );
    sha256_update( &ctx, sum, 32 );
    sha256_finish( &ctx, output );
    sha256_free( &ctx );
}

unsigned char tl_hmac_sha256(unsigned char *key, int counter,  unsigned char *kmac, unsigned char *kmle)
{
    unsigned char buff[32] = {0, 0, 0, 0, 0x54,0x68,0x72,0x65,0x61,0x64};   // counter_"Thread"
    buff[0] = counter >> 24;
    buff[1] = counter >> 16;
    buff[2] = counter >> 8;
    buff[3] = counter >> 0;

    sha256_hmac (key, 16, buff, 10, buff);

    memcpy (kmle, buff, 16);
    memcpy (kmac, buff+16, 16);
    return (counter & 0x7f) + 1;
}

/*static  void test_print128(unsigned char *bytes)
  {
      int         j;
      for (j=0; j<16;j++) {
          tl_printf("%02x",bytes[j]);
          if ( (j%4) == 3 ) tl_printf(" ");
      }
  }*/

#define TEST_VECTOR_OUT1    "\
MLE key = 54 45 f4 15 8f d7 59 12 17 58 09 f8 b5 7a 66 a4 \n\
MAC key = de 89 c5 3a f3 82 b4 21 e0 fd e5 a9 ba e3 be f0 \n\
Key index = 1"

#define TEST_VECTOR_OUT2    "\
MLE key = 8f 4c d1 a2 7d 95 c0 7d 12 db 89 74 bd 61 5c 13 \n\
MAC key = 9b e0 d1 af 7b d8 73 50 de ab cd d0 7f eb b9 d5 \n\
Key index = 2"

#define TEST_VECTOR_OUT3    "\
MLE key = 01 6e 2a b8 ec 88 87 96 87 a7 2e 0a 35 7e cf 2a \n\
MAC key = 56 41 09 e9 d2 aa d7 f7 23 ec 3b 96 11 0e ef a3 \n\
Key index = 3"


/*void test_hmac_sha256 ()
{
     unsigned char mk[16] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
     unsigned char msg[10] = {0x00,0x00,0x00,0x00,0x54,0x68,0x72,0x65,0x61,0x64};
     unsigned char kmac[16], kmle[16];
     int i;
     for (i=0; i<3; i++)
     {
         unsigned char idx = tl_hmac_sha256 (mk, i, kmac, kmle);
         tl_printf("--------------------------------------------------\n");
         tl_printf("HMAC_SHA256-MAC  "); test_print128(kmac); tl_printf("\n");
         tl_printf("HMAC_SHA256-MLE  "); test_print128(kmle); tl_printf("\n");
         tl_printf("HMAC_SHA256-IDX  %d", idx); tl_printf("\n");

         tl_printf("        vs     Expectation \n%s\n", i==0 ? TEST_VECTOR_OUT1 : i == 1? TEST_VECTOR_OUT2 : TEST_VECTOR_OUT3);

     }
}*/
