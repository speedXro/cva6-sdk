// To run  local tests execute: (gcc -o exe SWDCT2DQ.c >/dev/null 2>&1 || (gcc -fdiagnostics-color=always -o exe SWDCT2DQ.c 2>&1 | less -R && false))  && ./exe
#include "SWDCT2DQ.h"

#ifndef __riscv
//include for off-target test code results display purposes only
#include <stdio.h>
const uint8_t QC = 255;
#endif

#define USE_QUANT_RECIPROCAL_LOOKUP_TABLE 0

#define CONST_BITS   (13-0)	   /* Fixed point fractional bits */ //minimal needed seems to be 9, jpeg lib uses 13

#define QB           (13)       /* FDCT2DQ actual output bits and IDCT2DQ actual input bits*/

#define FINPUT_UNSG  ( 1)
#define FROUND       ( 0)       /* in fdct2d with rounded quantization early rounding does not seem necesary */
#define QROUND       ( 1)       /* in fdct2d with rounded quantization early rounding does not seem necesary */
#define FINPUT_BITS  ( 8)       /* Input data bit width */
#define FPASS1_BITS  ( 2+0)       /* Extra bits after 1st pass */     //confirmed that (8 + 3) + 2 needs to be passed to columns processing
#define FPASS2_BITS  ( 0)       /* Extra bits after 2nd pass */     // ((8 + 3 + 2) + 3) + 0 need to go to quantization
#define FQUANT_BITS  (17)       /* Fixed point quantization fractional bits */ //16-17 bit should be enough, but 
#define FOUTPT_BITS  (QB)       /* Actual bits in output after quantization */ // after quantization it seems that 8 + 3 + 2 is enough for exact idct2d

#define IINPUT_BITS  (QB)       /* Input data bit width */ //before quantization (after dequantization it may exceed this because of quantization rounding)
#define IROUND       ( 1)       
#define IPASS1_BITS  ( 0)       /* Extra bits after 1st pass */     //confirmed that (13 + 3) - 3  should be enough (as data is decorelated less fractional part is needed)
#define IOUTPT_BITS  ( 8)       /* Actual bits in output after rows idct is done */ // only original 8 bit should be kept, but due to noise in quantization clamping is needed before trunctation
#define IDCT2DQ_DO_ICLAMP (1)   /* Enable/disable integer clamping to specified output range - quantization introduces noise that may lead to out of range values on reconstruction */

#if FQUANT_BITS > 30-1 
        #error FQUANT_BITS (reciprocal fractional bits) for optimizing quantization through multiplication with reciprocal must not exceed 30!
#endif

#if USE_QUANT_RECIPROCAL_LOOKUP_TABLE
static int32_t QUANT_RECIPROCAL_LOOKUP_TABLE[256] = FDCT2DQ_QUANT_RECIPROCALS_LOOKUP_TABLE_INIT(FQUANT_BITS); //Quantization reciprocal lookup 1 bit sign 1 bit integer part 30 bit fractionary (although it seems 13 fractionary bits would be enough for exact 13 bit output)
#endif

#define ONE	((int32_t) 1)

#if CONST_BITS == -13
#define FIX_0_298631336  ((int16_t)  2446)	/* FIXED_POINT(0.298631336) */
#define FIX_0_390180644  ((int16_t)  3196)	/* FIXED_POINT(0.390180644) */
#define FIX_0_541196100  ((int16_t)  4433)	/* FIXED_POINT(0.541196100) */
#define FIX_0_765366865  ((int16_t)  6270)	/* FIXED_POINT(0.765366865) */
#define FIX_0_899976223  ((int16_t)  7373)	/* FIXED_POINT(0.899976223) */
#define FIX_1_175875602  ((int16_t)  9633)	/* FIXED_POINT(1.175875602) */ 
#define FIX_1_501321110  ((int16_t)  12299)	/* FIXED_POINT(1.501321110) */
#define FIX_1_847759065  ((int16_t)  15137)	/* FIXED_POINT(1.847759065) */
#define FIX_1_961570560  ((int16_t)  16069)	/* FIXED_POINT(1.961570560) */
#define FIX_2_053119869  ((int16_t)  16819)	/* FIXED_POINT(2.053119869) */
#define FIX_2_562915447  ((int16_t)  20995)	/* FIXED_POINT(2.562915447) */
#define FIX_3_072711026  ((int16_t)  25172)	/* FIXED_POINT(3.072711026) */
#else
#define FIXED_POINT(x)	((int32_t) ((x) * (ONE << CONST_BITS) + 0.5)) /* Convert a positive real constant to an integer scaled by CONST_SCALE */
//#define FIXED_POINT(x)	(((int32_t) ((x) * (ONE << (CONST_BITS+1)) + 1))>>1) /* Convert a positive real constant to an integer scaled by CONST_SCALE */
#define FIX_0_298631336  FIXED_POINT(0.298631336)
#define FIX_0_390180644  FIXED_POINT(0.390180644)
#define FIX_0_541196100  FIXED_POINT(0.541196100)
#define FIX_0_765366865  FIXED_POINT(0.765366865)
#define FIX_0_899976223  FIXED_POINT(0.899976223)
#define FIX_1_175875602  FIXED_POINT(1.175875602)
#define FIX_1_501321110  FIXED_POINT(1.501321110)
#define FIX_1_847759065  FIXED_POINT(1.847759065)
#define FIX_1_961570560  FIXED_POINT(1.961570560)
#define FIX_2_053119869  FIXED_POINT(2.053119869)
#define FIX_2_562915447  FIXED_POINT(2.562915447)
#define FIX_3_072711026  FIXED_POINT(3.072711026)
#endif
//rounding issure in sw could be fixed by additional bit in fpass1 or by better values for fixed point?
//in hw 13 bit fp value seems to work well when rounded, without additional rounding at pass1&2, only at quant (where reciprocal is rounded)
void LLM_FDCT_2D_Q_get_reciprocals_for_qantization_components(int32_t *rm, const uint8_t *qm) {
    for(int i=0;i<64;i++){
        #if 0==USE_QUANT_RECIPROCAL_LOOKUP_TABLE
            //rm[i] = (((int64_t)1)<<FQUANT_BITS)/(*(((uint8_t*)qm)+i)); //this is the actual procedure to obtain the quant lookup table
            rm[i] = ((((int64_t)1)<<(FQUANT_BITS+1))/(*(((uint8_t*)qm)+i))+0)>>1; //this is the actual procedure to obtain the quant lookup table
        #else
                rm[i] = QUANT_RECIPROCAL_LOOKUP_TABLE[*(((uint8_t*)qm)+i)];
        #endif
    }
    
}

void LLM_FDCT_2D_Q_u8_2_s13(uint8_t *ptr_in, int16_t *out, int32_t const *qrm) {
    int16_t temp[64];
    int16_t* ptr_out=temp;
    
    /* Pass 1: process rows.
     * Note results are scaled up by sqrt(8) compared to a true DCT1D
     * also output will need at least 3 additional bits as for LLM_FDCT1D
     * and the results are additionally scaled by 2**FPASS1_BITS (additional fractional bits)
    */
    for (int i = 0; i < 8; i++, ptr_in+=8, ptr_out+=8) {
        /* Stage 1 */
        int16_t stage1[8];
        stage1[0] = ptr_in[0] + ptr_in[7];
        stage1[1] = ptr_in[1] + ptr_in[6];
        stage1[2] = ptr_in[2] + ptr_in[5];
        stage1[3] = ptr_in[3] + ptr_in[4];
        stage1[4] = ptr_in[3] - ptr_in[4];
        stage1[5] = ptr_in[2] - ptr_in[5];
        stage1[6] = ptr_in[1] - ptr_in[6];
        stage1[7] = ptr_in[0] - ptr_in[7];

        /* Stage 2 */
        int16_t stage2[8];
        stage2[0] = stage1[0] + stage1[3];
        stage2[1] = stage1[1] + stage1[2];
        stage2[2] = stage1[1] - stage1[2];
        stage2[3] = stage1[0] - stage1[3];
        stage2[4] = stage1[4] + stage1[7];
        stage2[5] = stage1[4] + stage1[6];
        stage2[6] = stage1[5] + stage1[7];
        stage2[7] = stage1[5] + stage1[6];

        /* Stage 3 */
        int16_t stage3[4];
        stage3[0] = stage2[0] + stage2[1];
        stage3[1] = stage2[0] - stage2[1];
        stage3[2] = stage2[2] + stage2[3];
        stage3[3] = stage2[5] + stage2[6];

        // unsigned -> signed conversion
        //#if FINPUT_UNSG
        stage3[0] -= 1<<(FINPUT_BITS+3-1);
        //#endif

        /* Stage 4: Multiplications */
        int32_t stage4[13];
        stage4[ 0] = stage2[2] * FIX_1_847759065;  /* c2+c6 */
        stage4[ 1] = stage3[2] * FIX_0_541196100;  /* c6 */
        stage4[ 2] = stage2[3] * FIX_0_765366865;  /* c2-c6 */
        stage4[ 3] = stage2[4] * FIX_0_899976223;  /* c3-c7 */
        stage4[ 4] = stage1[4] * FIX_0_298631336;  /* -c1+c3+c5-c7 */
        stage4[ 5] = stage2[5] * FIX_1_961570560;  /* c3+c5 */
        stage4[ 6] = stage1[5] * FIX_2_053119869;  /* c1+c3-c5+c7 */
        stage4[ 7] = stage3[3] * FIX_1_175875602;  /* c3 */
        stage4[ 8] = stage1[6] * FIX_3_072711026;  /* c1+c3+c5-c7 */
        stage4[ 9] = stage2[6] * FIX_0_390180644;  /* c3-c5 */
        stage4[10] = stage1[7] * FIX_1_501321110;  /* c1+c3-c5-c7 */
        stage4[11] = stage2[7] * FIX_2_562915447;  /* c1+c3 */

        /* Rounding if ROUND macro is "true" */
        #if FROUND
        stage4[1] += ONE << (CONST_BITS - FPASS1_BITS - 1); // +0.5
        stage4[7] += ONE << (CONST_BITS - FPASS1_BITS - 1); // +0.5
        #endif

        /* Stage 5 */
        int32_t stage5[8];
        stage5[0] = stage4[ 1] - stage4[ 0];
        stage5[1] = stage4[ 1] + stage4[ 2];
        stage5[2] = stage4[ 4] - stage4[ 3];
        stage5[3] = stage4[ 7] - stage4[ 5];
        stage5[4] = stage4[ 6] - stage4[11];
        stage5[5] = stage4[ 8] - stage4[11];
        stage5[6] = stage4[ 7] - stage4[ 9];
        stage5[7] = stage4[10] - stage4[ 3];

        /* Stage 6 */
        int32_t stage6[4];
        stage6[0] = stage5[2] + stage5[3];
        stage6[1] = stage5[4] + stage5[6];
        stage6[2] = stage5[3] + stage5[5];
        stage6[3] = stage5[6] + stage5[7];

        /* Output stage: Arithmetic shifts */
        ptr_out[0] = (int16_t)(stage3[0] << (             FPASS1_BITS));
        ptr_out[4] = (int16_t)(stage3[1] << (             FPASS1_BITS));
        ptr_out[6] = (int16_t)(stage5[0] >> (CONST_BITS - FPASS1_BITS));
        ptr_out[2] = (int16_t)(stage5[1] >> (CONST_BITS - FPASS1_BITS));
        ptr_out[7] = (int16_t)(stage6[0] >> (CONST_BITS - FPASS1_BITS));
        ptr_out[5] = (int16_t)(stage6[1] >> (CONST_BITS - FPASS1_BITS));
        ptr_out[3] = (int16_t)(stage6[2] >> (CONST_BITS - FPASS1_BITS));
        ptr_out[1] = (int16_t)(stage6[3] >> (CONST_BITS - FPASS1_BITS));
    }
    
    /* Pass 2: process columns.
    *  The PASS1_BITS and the overall scale factor of 8(=(2*srt(2))^2) should be removed for DCT2 compatibility (keep only the most significant 11 bits of the result, but additional fractional bits are needed for exact reconstruction even when quantization matrix is filled with ones).
    *  Octave DCT2 needs at least two fractional bits compared to the 11 integer bits for exact reconstruction of 8 bit samples (even when not doing quantization)
    */
    for (int i = 0; i < 8; i++) {
        int16_t* data_ptr = temp + i;        
        /* Stage 1 */
        int32_t stage1[8];
        stage1[0] = (int32_t)data_ptr[8 * 0] + (int32_t)data_ptr[8 * 7];
        stage1[1] = (int32_t)data_ptr[8 * 1] + (int32_t)data_ptr[8 * 6];
        stage1[2] = (int32_t)data_ptr[8 * 2] + (int32_t)data_ptr[8 * 5];
        stage1[3] = (int32_t)data_ptr[8 * 3] + (int32_t)data_ptr[8 * 4];
        stage1[4] = (int32_t)data_ptr[8 * 3] - (int32_t)data_ptr[8 * 4];
        stage1[5] = (int32_t)data_ptr[8 * 2] - (int32_t)data_ptr[8 * 5];
        stage1[6] = (int32_t)data_ptr[8 * 1] - (int32_t)data_ptr[8 * 6];
        stage1[7] = (int32_t)data_ptr[8 * 0] - (int32_t)data_ptr[8 * 7];

        /* Stage 2 */
        int32_t stage2[8];
        stage2[0] = stage1[0] + stage1[3];
        stage2[1] = stage1[1] + stage1[2];
        stage2[2] = stage1[1] - stage1[2];
        stage2[3] = stage1[0] - stage1[3];
        stage2[4] = stage1[4] + stage1[7];
        stage2[5] = stage1[4] + stage1[6];
        stage2[6] = stage1[5] + stage1[7];
        stage2[7] = stage1[5] + stage1[6];

        #if FROUND // && (FINPUT_BITS + 3 + FPASS1_BITS + 3 + FPASS2_BITS - FOUTPT_BITS - 1 >= 0)
        stage2[0] += ONE << (FINPUT_BITS + 3 + FPASS1_BITS + 3 + FPASS2_BITS - FOUTPT_BITS - 1); // +0.5
        #endif

        /* Stage 3 */
        int32_t stage3[4];
        stage3[0] = stage2[0] + stage2[1];
        stage3[1] = stage2[0] - stage2[1];
        stage3[2] = stage2[2] + stage2[3];
        stage3[3] = stage2[5] + stage2[6];

        /* Stage 4: Multiplications */
        int32_t stage4[13];
        stage4[ 0] = stage2[2] * FIX_1_847759065;  /* c2+c6 */
        stage4[ 1] = stage3[2] * FIX_0_541196100;  /* c6 */
        stage4[ 2] = stage2[3] * FIX_0_765366865;  /* c2-c6 */
        stage4[ 3] = stage2[4] * FIX_0_899976223;  /* c3-c7 */
        stage4[ 4] = stage1[4] * FIX_0_298631336;  /* -c1+c3+c5-c7 */
        stage4[ 5] = stage2[5] * FIX_1_961570560;  /* c3+c5 */
        stage4[ 6] = stage1[5] * FIX_2_053119869;  /* c1+c3-c5+c7 */
        stage4[ 7] = stage3[3] * FIX_1_175875602;  /* c3 */
        stage4[ 8] = stage1[6] * FIX_3_072711026;  /* c1+c3+c5-c7 */
        stage4[ 9] = stage2[6] * FIX_0_390180644;  /* c3-c5 */
        stage4[10] = stage1[7] * FIX_1_501321110;  /* c1+c3-c5-c7 */
        stage4[11] = stage2[7] * FIX_2_562915447;  /* c1+c3 */

        #if FROUND 
        stage4[1] += ONE << (FINPUT_BITS + 3 + FPASS1_BITS + 3 + FPASS2_BITS + CONST_BITS - FOUTPT_BITS - 1); // +0.5
        stage4[7] += ONE << (FINPUT_BITS + 3 + FPASS1_BITS + 3 + FPASS2_BITS + CONST_BITS - FOUTPT_BITS - 1);  // +0.5
        #endif

        /* Stage 5 */
        int32_t stage5[8];
        stage5[0] = stage4[ 1] - stage4[ 0];
        stage5[1] = stage4[ 1] + stage4[ 2];
        stage5[2] = stage4[ 4] - stage4[ 3];
        stage5[3] = stage4[ 7] - stage4[ 5];
        stage5[4] = stage4[ 6] - stage4[11];
        stage5[5] = stage4[ 8] - stage4[11];
        stage5[6] = stage4[ 7] - stage4[ 9];
        stage5[7] = stage4[10] - stage4[ 3];

        /* Stage 6 */
        int32_t stage6[4];
        stage6[0] = stage5[2] + stage5[3];
        stage6[1] = stage5[4] + stage5[6];
        stage6[2] = stage5[3] + stage5[5];
        stage6[3] = stage5[6] + stage5[7];

        /* Output stage: Arithmetic shifts and quantization */
        int16_t *out_ptr = out + i; //column selection
        
        #if PASS2_BITS
        stage3[0] <<= PASS2_BITS;
        stage3[1] <<= PASS2_BITS;
        stage5[0] >>= (CONST_BITS - PASS2_BITS);
        stage5[1] >>= (CONST_BITS - PASS2_BITS);
        stage6[0] >>= (CONST_BITS - PASS2_BITS);
        stage6[1] >>= (CONST_BITS - PASS2_BITS);
        stage6[2] >>= (CONST_BITS - PASS2_BITS);
        stage6[3] >>= (CONST_BITS - PASS2_BITS);
        #endif
        const int32_t *rm_ptr         = qrm + i; //for 8 bit input the integer part of output should be 8 + 3 + 3 = 14 bit to correspond to octave round(dct2(x-128)*8); less bits means downscaling, more bits means upscaling
        const uint8_t  MostSignBitPos = FINPUT_BITS + 3 + FPASS1_BITS + 3 + FPASS2_BITS + FQUANT_BITS; //calculate the position of the most significant bit of the result after quantization
        const  int8_t  DISCARD04      = MostSignBitPos - FOUTPT_BITS;  // calculate the number of least significant bits to discard given a specific number of bits from the result to be kept (for unscaded components)
        const uint8_t  DISCARDxx      = DISCARD04 + CONST_BITS; //discard bits for the scaled components
        #if(__riscv_xlen == 64) //__riscv
            #define QUANT_BY_MULT_WITH_RECIPROCAL_ROUND_AND_TRIM(out, in, rq, discard) \
            __asm__ volatile (              \
                "mul  %0, %1, %2 \n\t"      \
                "srai %0, %0, %c3 \n\t"      \
                "addi %0, %0, 1\n\t"        \
                "srai %0, %0, 1\n\t"        \
                : "=r" (out)                \
                : "r" (in), "r" (rq), "i" (discard-1) \
            )

            QUANT_BY_MULT_WITH_RECIPROCAL_ROUND_AND_TRIM(out_ptr[8 * 0], stage3[0], rm_ptr[8 * 0], DISCARD04);
            QUANT_BY_MULT_WITH_RECIPROCAL_ROUND_AND_TRIM(out_ptr[8 * 4], stage3[1], rm_ptr[8 * 4], DISCARD04);
            QUANT_BY_MULT_WITH_RECIPROCAL_ROUND_AND_TRIM(out_ptr[8 * 6], stage5[0], rm_ptr[8 * 6], DISCARDxx);
            QUANT_BY_MULT_WITH_RECIPROCAL_ROUND_AND_TRIM(out_ptr[8 * 2], stage5[1], rm_ptr[8 * 2], DISCARDxx);
            QUANT_BY_MULT_WITH_RECIPROCAL_ROUND_AND_TRIM(out_ptr[8 * 7], stage6[0], rm_ptr[8 * 7], DISCARDxx);
            QUANT_BY_MULT_WITH_RECIPROCAL_ROUND_AND_TRIM(out_ptr[8 * 5], stage6[1], rm_ptr[8 * 5], DISCARDxx);
            QUANT_BY_MULT_WITH_RECIPROCAL_ROUND_AND_TRIM(out_ptr[8 * 3], stage6[2], rm_ptr[8 * 3], DISCARDxx);
            QUANT_BY_MULT_WITH_RECIPROCAL_ROUND_AND_TRIM(out_ptr[8 * 1], stage6[3], rm_ptr[8 * 1], DISCARDxx);
        #elif 1 //just generic code...            
            //do optimized quantization by multiplying with reciprocal values of the quantization matrix and then round the least significant bit to be kept in the output
            out_ptr[8 * 0] = (int16_t)( ( ((int64_t)(stage3[0])) * rm_ptr[8 * 0] + (((int64_t)QROUND)<<(DISCARD04-1)) ) >> (DISCARD04));
            out_ptr[8 * 4] = (int16_t)( ( ((int64_t)(stage3[1])) * rm_ptr[8 * 4] + (((int64_t)QROUND)<<(DISCARD04-1)) ) >> (DISCARD04));
            out_ptr[8 * 6] = (int16_t)( ( ((int64_t)(stage5[0])) * rm_ptr[8 * 6] + (((int64_t)QROUND)<<(DISCARDxx-1)) ) >> (DISCARDxx));
            out_ptr[8 * 2] = (int16_t)( ( ((int64_t)(stage5[1])) * rm_ptr[8 * 2] + (((int64_t)QROUND)<<(DISCARDxx-1)) ) >> (DISCARDxx));
            out_ptr[8 * 7] = (int16_t)( ( ((int64_t)(stage6[0])) * rm_ptr[8 * 7] + (((int64_t)QROUND)<<(DISCARDxx-1)) ) >> (DISCARDxx));
            out_ptr[8 * 5] = (int16_t)( ( ((int64_t)(stage6[1])) * rm_ptr[8 * 5] + (((int64_t)QROUND)<<(DISCARDxx-1)) ) >> (DISCARDxx));
            out_ptr[8 * 3] = (int16_t)( ( ((int64_t)(stage6[2])) * rm_ptr[8 * 3] + (((int64_t)QROUND)<<(DISCARDxx-1)) ) >> (DISCARDxx));
            out_ptr[8 * 1] = (int16_t)( ( ((int64_t)(stage6[3])) * rm_ptr[8 * 1] + (((int64_t)QROUND)<<(DISCARDxx-1)) ) >> (DISCARDxx));
            __asm__ volatile ("nop");
        #elif 1 //for debug purposes only, return data without quantization
            out_ptr[8 * 0] = (int16_t)(stage3[0]);
            out_ptr[8 * 4] = (int16_t)(stage3[1]);
            out_ptr[8 * 6] = (int16_t)((stage5[0]>>(CONST_BITS)));
            out_ptr[8 * 2] = (int16_t)((stage5[1]>>(CONST_BITS)));
            out_ptr[8 * 7] = (int16_t)((stage6[0]>>(CONST_BITS)));
            out_ptr[8 * 5] = (int16_t)((stage6[1]>>(CONST_BITS)));
            out_ptr[8 * 3] = (int16_t)((stage6[2]>>(CONST_BITS)));
            out_ptr[8 * 1] = (int16_t)((stage6[3]>>(CONST_BITS)));
        #elif 1 //for debug purposes only, return data without quantization
            out_ptr[8 * 0] = (int16_t)(stage3[0]>>3);
            out_ptr[8 * 4] = (int16_t)(stage3[1]>>3);
            out_ptr[8 * 6] = (int16_t)((stage5[0]>>(CONST_BITS+3)));
            out_ptr[8 * 2] = (int16_t)((stage5[1]>>(CONST_BITS+3)));
            out_ptr[8 * 7] = (int16_t)((stage6[0]>>(CONST_BITS+3)));
            out_ptr[8 * 5] = (int16_t)((stage6[1]>>(CONST_BITS+3)));
            out_ptr[8 * 3] = (int16_t)((stage6[2]>>(CONST_BITS+3)));
            out_ptr[8 * 1] = (int16_t)((stage6[3]>>(CONST_BITS+3)));
        #else //for debug purposes only, return data without quantization
            out_ptr[8 * 0] = (int16_t)(stage3[0]>>(DISCARD04));
            out_ptr[8 * 4] = (int16_t)(stage3[1]>>(DISCARD04));
            out_ptr[8 * 6] = (int16_t)((stage5[0]>>(DISCARDxx)));
            out_ptr[8 * 2] = (int16_t)((stage5[1]>>(DISCARDxx)));
            out_ptr[8 * 7] = (int16_t)((stage6[0]>>(DISCARDxx)));
            out_ptr[8 * 5] = (int16_t)((stage6[1]>>(DISCARDxx)));
            out_ptr[8 * 3] = (int16_t)((stage6[2]>>(DISCARDxx)));
            out_ptr[8 * 1] = (int16_t)((stage6[3]>>(DISCARDxx)));
        #endif
    }
}

void LLM_IDCT_2D_Q_s13_2_u8(int16_t *in, uint8_t *out, uint8_t const *qm) {
    int16_t temp[64];
    /* Pass 1: process columns from input.
     * Note results are scaled up by sqrt(8) compared to a true DCT;
     * also, the results are scaled by 2**PASS1_BITS.
    */
    for (int i = 0; i < 8; i++) {
        int16_t* ptr_in = &in[i];
        const uint8_t* qm_ptr = &qm[i];

        /* inputs */
        int32_t stage0[8];
        stage0[0] = ((int32_t)(((int16_t)qm_ptr[8 * 0]) * ((int16_t)ptr_in[8 * 0]))) << CONST_BITS;
        stage0[4] = ((int32_t)(((int16_t)qm_ptr[8 * 4]) * ((int16_t)ptr_in[8 * 4]))) << CONST_BITS;
        stage0[6] = (int32_t)(((int16_t)qm_ptr[8 * 6]) * ((int16_t)ptr_in[8 * 6]));
        stage0[2] = (int32_t)(((int16_t)qm_ptr[8 * 2]) * ((int16_t)ptr_in[8 * 2]));
        stage0[7] = (int32_t)(((int16_t)qm_ptr[8 * 7]) * ((int16_t)ptr_in[8 * 7]));
        stage0[5] = (int32_t)(((int16_t)qm_ptr[8 * 5]) * ((int16_t)ptr_in[8 * 5]));
        stage0[3] = (int32_t)(((int16_t)qm_ptr[8 * 3]) * ((int16_t)ptr_in[8 * 3]));
        stage0[1] = (int32_t)(((int16_t)qm_ptr[8 * 1]) * ((int16_t)ptr_in[8 * 1]));


        /* Rounding */
        #if IROUND
        stage0[0] += ONE << (CONST_BITS - IPASS1_BITS - 1); // +0.5
        #endif

        /* Stage 1 */
        int32_t stage1[7];
        stage1[0] = (stage0[0] + stage0[4]);
        stage1[1] = (stage0[0] - stage0[4]);
        stage1[2] = (stage0[6] + stage0[2]);
        stage1[3] = (stage0[7] + stage0[1]);
        stage1[4] = (stage0[7] + stage0[3]);
        stage1[5] = (stage0[5] + stage0[1]);
        stage1[6] = (stage0[5] + stage0[3]);

        /* Stage 2 */
        int32_t stage2[1];
        stage2[0] = stage1[4] + stage1[5];

        /* Stage 3: Multiplications */
        int32_t stage3[12];
        stage3[ 0] = stage0[6] * FIX_1_847759065;  /* c2+c6 */
        stage3[ 1] = stage1[2] * FIX_0_541196100;  /* c6 */
        stage3[ 2] = stage0[2] * FIX_0_765366865;  /* c2-c6 */
        stage3[ 3] = stage1[3] * FIX_0_899976223;  /* c3-c7 */
        stage3[ 4] = stage0[7] * FIX_0_298631336;  /* -c1+c3+c5-c7 */
        stage3[ 5] = stage1[4] * FIX_1_961570560;  /* c3+c5 */
        stage3[ 6] = stage0[5] * FIX_2_053119869;  /* c1+c3-c5+c7 */
        stage3[ 7] = stage2[0] * FIX_1_175875602;  /* c3 */
        stage3[ 8] = stage0[3] * FIX_3_072711026;  /* c1+c3+c5-c7 */
        stage3[ 9] = stage1[5] * FIX_0_390180644;  /* c3-c5 */
        stage3[10] = stage0[1] * FIX_1_501321110; /* c1+c3-c5-c7 */
        stage3[11] = stage1[6] * FIX_2_562915447; /* c1+c3 */

        /* Stage 4 */
        int32_t stage4[8];
        stage4[0] = stage3[ 1] - stage3[ 0];
        stage4[1] = stage3[ 1] + stage3[ 2];
        stage4[2] = stage3[ 4] - stage3[ 3];
        stage4[3] = stage3[ 7] - stage3[ 5];
        stage4[4] = stage3[ 6] - stage3[11];
        stage4[5] = stage3[ 8] - stage3[11];
        stage4[6] = stage3[ 7] - stage3[ 9];
        stage4[7] = stage3[10] - stage3[ 3];

        /* Stage 5 */
        int32_t stage5[8];
        stage5[0] = stage1[0] + stage4[1];
        stage5[1] = stage1[1] + stage4[0];
        stage5[2] = stage1[1] - stage4[0];
        stage5[3] = stage1[0] - stage4[1];
        stage5[4] = stage4[2] + stage4[3];
        stage5[5] = stage4[4] + stage4[6];
        stage5[6] = stage4[3] + stage4[5];
        stage5[7] = stage4[6] + stage4[7];

        /* Stage 6 */
        int32_t stage6[8];
        stage6[0] = stage5[0] + stage5[7];
        stage6[1] = stage5[1] + stage5[6];
        stage6[2] = stage5[2] + stage5[5];
        stage6[3] = stage5[3] + stage5[4];
        stage6[4] = stage5[3] - stage5[4];
        stage6[5] = stage5[2] - stage5[5];
        stage6[6] = stage5[1] - stage5[6];
        stage6[7] = stage5[0] - stage5[7];

        int16_t* ptr_out = &temp[i];
        /* Output stage: Arithmetic shifts */
        ptr_out[8 * 0] = (int16_t)(stage6[0] >> (CONST_BITS - IPASS1_BITS));
        ptr_out[8 * 1] = (int16_t)(stage6[1] >> (CONST_BITS - IPASS1_BITS));
        ptr_out[8 * 2] = (int16_t)(stage6[2] >> (CONST_BITS - IPASS1_BITS));
        ptr_out[8 * 3] = (int16_t)(stage6[3] >> (CONST_BITS - IPASS1_BITS));
        ptr_out[8 * 4] = (int16_t)(stage6[4] >> (CONST_BITS - IPASS1_BITS));
        ptr_out[8 * 5] = (int16_t)(stage6[5] >> (CONST_BITS - IPASS1_BITS));
        ptr_out[8 * 6] = (int16_t)(stage6[6] >> (CONST_BITS - IPASS1_BITS));
        ptr_out[8 * 7] = (int16_t)(stage6[7] >> (CONST_BITS - IPASS1_BITS));
    }
    /* Pass 2: process rows.
    */
    for (int i = 0; i < 8; i++) {
        int16_t* data_ptr = &temp[i*8];
        uint8_t* ptr_out = &out[i*8];

        /* inputs */
        int32_t stage0[8];
        stage0[0] = (int32_t)data_ptr[0] << CONST_BITS;
        stage0[4] = (int32_t)data_ptr[4] << CONST_BITS;
        stage0[2] = (int32_t)data_ptr[2];
        stage0[6] = (int32_t)data_ptr[6];
        stage0[7] = (int32_t)data_ptr[7];
        stage0[5] = (int32_t)data_ptr[5];
        stage0[3] = (int32_t)data_ptr[3];
        stage0[1] = (int32_t)data_ptr[1];
        
        /* Rounding */
        #if IROUND
        stage0[0] += ONE << (CONST_BITS - IPASS1_BITS + IINPUT_BITS - IOUTPT_BITS - 1); // +0.5
        #endif
        /* Stage 1 */
        int32_t stage1[7];
        stage1[0] = stage0[0] + stage0[4];
        stage1[1] = stage0[0] - stage0[4];
        stage1[2] = stage0[6] + stage0[2];
        stage1[3] = stage0[7] + stage0[1];
        stage1[4] = stage0[7] + stage0[3];
        stage1[5] = stage0[5] + stage0[1];
        stage1[6] = stage0[5] + stage0[3];

        /* Stage 2 */
        int32_t stage2[1];
        stage2[0] = stage1[4] + stage1[5];

        /* Stage 3: Multiplications */
        int32_t stage3[12];
        stage3[ 0] = stage0[6] * FIX_1_847759065;  /* c2+c6 */
        stage3[ 1] = stage1[2] * FIX_0_541196100;  /* c6 */
        stage3[ 2] = stage0[2] * FIX_0_765366865;  /* c2-c6 */
        stage3[ 3] = stage1[3] * FIX_0_899976223;  /* c3-c7 */
        stage3[ 4] = stage0[7] * FIX_0_298631336;  /* -c1+c3+c5-c7 */
        stage3[ 5] = stage1[4] * FIX_1_961570560;  /* c3+c5 */
        stage3[ 6] = stage0[5] * FIX_2_053119869;  /* c1+c3-c5+c7 */
        stage3[ 7] = stage2[0] * FIX_1_175875602;  /* c3 */
        stage3[ 8] = stage0[3] * FIX_3_072711026;  /* c1+c3+c5-c7 */
        stage3[ 9] = stage1[5] * FIX_0_390180644;  /* c3-c5 */
        stage3[10] = stage0[1] * FIX_1_501321110; /* c1+c3-c5-c7 */
        stage3[11] = stage1[6] * FIX_2_562915447; /* c1+c3 */

        /* Stage 4 */
        int32_t stage4[8];
        stage4[0] = stage3[ 1] - stage3[ 0];
        stage4[1] = stage3[ 1] + stage3[ 2];
        stage4[2] = stage3[ 4] - stage3[ 3];
        stage4[3] = stage3[ 7] - stage3[ 5];
        stage4[4] = stage3[ 6] - stage3[11];
        stage4[5] = stage3[ 8] - stage3[11];
        stage4[6] = stage3[ 7] - stage3[ 9];
        stage4[7] = stage3[10] - stage3[ 3];

        /* Stage 5 */
        int32_t stage5[8];
        stage5[0] = stage1[0] + stage4[1];
        stage5[1] = stage1[1] + stage4[0];
        stage5[2] = stage1[1] - stage4[0];
        stage5[3] = stage1[0] - stage4[1];
        stage5[4] = stage4[2] + stage4[3];
        stage5[5] = stage4[4] + stage4[6];
        stage5[6] = stage4[3] + stage4[5];
        stage5[7] = stage4[6] + stage4[7];

        /* Stage 6 */
        int32_t stage6[8];
        stage6[0] = stage5[0] + stage5[7];
        stage6[1] = stage5[1] + stage5[6];
        stage6[2] = stage5[2] + stage5[5];
        stage6[3] = stage5[3] + stage5[4];
        stage6[4] = stage5[3] - stage5[4];
        stage6[5] = stage5[2] - stage5[5];
        stage6[6] = stage5[1] - stage5[6];
        stage6[7] = stage5[0] - stage5[7];

        /* Output stage: Arithmetic shifts and then clacmping*/ //controlling so that the most significant result bit is in output (cutting down less significant bits)
        data_ptr[0] = (int16_t)(stage6[0] >> (IINPUT_BITS + IPASS1_BITS + CONST_BITS - IOUTPT_BITS));
        data_ptr[1] = (int16_t)(stage6[1] >> (IINPUT_BITS + IPASS1_BITS + CONST_BITS - IOUTPT_BITS));
        data_ptr[2] = (int16_t)(stage6[2] >> (IINPUT_BITS + IPASS1_BITS + CONST_BITS - IOUTPT_BITS));
        data_ptr[3] = (int16_t)(stage6[3] >> (IINPUT_BITS + IPASS1_BITS + CONST_BITS - IOUTPT_BITS));
        data_ptr[4] = (int16_t)(stage6[4] >> (IINPUT_BITS + IPASS1_BITS + CONST_BITS - IOUTPT_BITS));
        data_ptr[5] = (int16_t)(stage6[5] >> (IINPUT_BITS + IPASS1_BITS + CONST_BITS - IOUTPT_BITS));
        data_ptr[6] = (int16_t)(stage6[6] >> (IINPUT_BITS + IPASS1_BITS + CONST_BITS - IOUTPT_BITS));
        data_ptr[7] = (int16_t)(stage6[7] >> (IINPUT_BITS + IPASS1_BITS + CONST_BITS - IOUTPT_BITS));
        #define ICLAMP(x) ( IDCT2DQ_DO_ICLAMP \
                           ? ( ( x<((-1)<<(IOUTPT_BITS-1))) \
                                 ? ((-1)<<(IOUTPT_BITS-1)) \
                                 :   (x>((1<<(IOUTPT_BITS-1))-1)) \
                                     ? (((1<<(IOUTPT_BITS-1))-1)) \
                                     : (x) ) \
                           : x )

        data_ptr[0] = ICLAMP(data_ptr[0]);
        data_ptr[1] = ICLAMP(data_ptr[1]);
        data_ptr[2] = ICLAMP(data_ptr[2]);
        data_ptr[3] = ICLAMP(data_ptr[3]);
        data_ptr[4] = ICLAMP(data_ptr[4]);
        data_ptr[5] = ICLAMP(data_ptr[5]);
        data_ptr[6] = ICLAMP(data_ptr[6]);
        data_ptr[7] = ICLAMP(data_ptr[7]);

        ptr_out[0] = data_ptr[0]+(1<<(IOUTPT_BITS-1));
        ptr_out[1] = data_ptr[1]+(1<<(IOUTPT_BITS-1));
        ptr_out[2] = data_ptr[2]+(1<<(IOUTPT_BITS-1));
        ptr_out[3] = data_ptr[3]+(1<<(IOUTPT_BITS-1));
        ptr_out[4] = data_ptr[4]+(1<<(IOUTPT_BITS-1));
        ptr_out[5] = data_ptr[5]+(1<<(IOUTPT_BITS-1));
        ptr_out[6] = data_ptr[6]+(1<<(IOUTPT_BITS-1));
        ptr_out[7] = data_ptr[7]+(1<<(IOUTPT_BITS-1));
    }
}

