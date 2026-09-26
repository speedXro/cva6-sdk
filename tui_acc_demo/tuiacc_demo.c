#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "dma.h"
#include "SWDCT2DQ.h"
#include "stft_ref.h"

//#define N_SAMPLES 2048
#define N_SAMPLES 2112
#define I_SAMPLES  128
#define W_SAMPLES (I_SAMPLES/2)

#define NT_STFT ((((N_SAMPLES - I_SAMPLES) / W_SAMPLES)) + 1)

#define O_SAMPLES 64
#define O_SAMPLE_SIZE 32

#define USE_RDCYCLE 1

#ifdef USE_RDCYCLE
static uint64_t time_now(void)
{
    uint64_t res;
    __asm__ volatile ("csrr %0, cycle" : "=r" (res));
    return res;
}
#define TIME_UNIT "cycles"
#else
#include <time.h>
static uint64_t time_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}
#define TIME_UNIT "ns"
#endif

void Delay_s(int nrs);
void Delay_ms(int nrs);
void Delay_us(int nrs);

void STFT_Test(uint64_t* stft_in, uint64_t *hw_out, uint32_t *sw_out, int type, int print_values, uint8_t pixfloor, uint8_t datatest);
void FIDCT2D_Test(uint64_t* fdct_in, uint64_t* fdct_hw_out, uint64_t* fdct_sw_out, uint64_t* idct_hw_out, uint64_t* idct_sw_out, int print_values);

void u64_to_u16_array(uint64_t input[(N_SAMPLES/4)], uint16_t output[N_SAMPLES]);
void u16_to_i16_12b_16bi_array(uint16_t input[N_SAMPLES], int16_t output[N_SAMPLES]);
void i16_to_u64_array(int16_t input[N_SAMPLES], uint64_t output[(N_SAMPLES/2)]);
int16_t map12_to_i16_correct(uint16_t v);
void u64_12ub_to_u64_16ib(uint64_t input[(N_SAMPLES/4)], uint64_t output[(N_SAMPLES/2)]);
void u64_to_u32_le_be_array(uint64_t input[NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64], uint32_t output[(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2]);
void u32_to_u8_for_fdct2d_array(uint32_t input[(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2], uint8_t output[(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2]);
void u8_to_u64_for_fdct2d_array(uint8_t input[(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2], uint64_t output[((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2) / 8]);

void SetQuality(uint8_t quality, const uint8_t stdTable[8][8], uint8_t QTable[8][8]);
void Matrix2Vector(uint8_t Q_matrix[8][8], uint8_t Q_array[64]);

void ProcessingLoop(uint8_t wsel, uint8_t pixfloor, uint64_t* stft_in, uint64_t* stft_out, uint64_t* fdct2d_in, uint64_t* fdct2d_out, uint64_t* idct2d_in, uint64_t* idct2d_out, int print_debug);


void Delay_s(int nrs)
{
    uint64_t start = time_now();
    while((time_now()- start ) < nrs*100000000);
}
void Delay_ms(int nrs)
{
    uint64_t start = time_now();
    while((time_now()- start ) < nrs*100000);
}
void Delay_us(int nrs)
{
    uint64_t start = time_now();
    while((time_now()- start ) < nrs*100);
}

int16_t stft_data_in_init_i16_sins[128] = {
    0, 17581, 22478, 20048, 23050, 29758, 26266, 9350,
    -7000, -12081, -13332, -21980, -32768, -31689, -17120, -3850,
    0, 3850, 17120, 31689, 32767, 21980, 13332, 12081,
    7000, -9350, -26266, -29758, -23050, -20048, -22478, -17581,
    0, 17581, 22478, 20048, 23050, 29758, 26266, 9350,
    -7000, -12081, -13332, -21980, -32768, -31689, -17120, -3850,
    0, 3850, 17120, 31689, 32767, 21980, 13332, 12081,
    7000, -9350, -26266, -29758, -23050, -20048, -22478, -17581,
    0, 17581, 22478, 20048, 23050, 29758, 26266, 9350,
    -7000, -12081, -13332, -21980, -32768, -31689, -17120, -3850,
    0, 3850, 17120, 31689, 32767, 21980, 13332, 12081,
    7000, -9350, -26266, -29758, -23050, -20048, -22478, -17581,
    0, 17581, 22478, 20048, 23050, 29758, 26266, 9350,
    -7000, -12081, -13332, -21980, -32768, -31689, -17120, -3850,
    0, 3850, 17120, 31689, 32767, 21980, 13332, 12081,
    7000, -9350, -26266, -29758, -23050, -20048, -22478, -17581
};

int16_t stft_data_in_init_i16_testdata[128] = {
    2048,2048,2048,2048,2048,2048,2048,2048,
    2048,2048,2048,2048,2048,2048,2048,2048,
    2048,2048,2048,2048,2048,2048,2048,2048,
    2048,2048,2048,2048,2048,2048,2048,2048,
    2048,2048,2048,2048,2048,2048,2048,2048,
    2048,2048,2048,2048,2048,2048,2048,2048,
    2048,2048,2048,2048,2048,2048,2048,2048,
    2048,2048,2048,2048,2048,2048,2048,2048,
    2048,2048,2048,2048,2048,2048,2048,2048,
    2048,2048,2048,2048,2048,2048,2048,2048,
    2048,2048,2048,2048,2048,2048,2048,2048,
    2048,2048,2048,2048,2048,2048,2048,2048,
    2048,2048,2048,2048,2048,2048,2048,2048,
    2048,2048,2048,2048,2048,2048,2048,2048,
    2048,2048,2048,2048,2048,2048,2048,2048,
    2048,2048,2048,2048,2048,2048,2048,2048
};

uint64_t stft_data_in_init_u64_sins[32] = {
    0x0800080008000800ULL,0x0800080008000800ULL,0x0800080008000800ULL,0x0800080008000800ULL,
    0x0800080008000800ULL,0x0800080008000800ULL,0x0800080008000800ULL,0x0800080008000800ULL,
    0x0800080008000800ULL,0x0800080008000800ULL,0x0800080008000800ULL,0x0800080008000800ULL,
    0x0800080008000800ULL,0x0800080008000800ULL,0x0800080008000800ULL,0x0800080008000800ULL,
    0x0800080008000800ULL,0x0800080008000800ULL,0x0800080008000800ULL,0x0800080008000800ULL,
    0x0800080008000800ULL,0x0800080008000800ULL,0x0800080008000800ULL,0x0800080008000800ULL,
    0x0800080008000800ULL,0x0800080008000800ULL,0x0800080008000800ULL,0x0800080008000800ULL,
    0x0800080008000800ULL,0x0800080008000800ULL,0x0800080008000800ULL,0x0800080008000800ULL,
};

uint64_t stft_data_in_init_u64_testdata[32] = {
    0x77a3ca66c94310b4ULL, 0x0511192180a2c12a3ULL, 0x0ac7f8b4c9bcc970ULL, 0x6762029453f5c485ULL,
    0x708b87523607c311ULL, 0x598b72aa619bb314ULL, 0x72de7d8068f7eb2cULL, 0xc150a040ea30a9fbULL,
    0xfaae8cb50b7757b1ULL, 0x2203354c90e86621ULL, 0x3c241fa9e09353d9ULL, 0x0a9b170cf4bb741eULL,
    0x04da0ef6d3039228ULL, 0xcd3a81dcf835ad9dULL, 0x194f3ace2ed6b658ULL, 0x49bb0bd2d6c150bbULL,
    0xa554d650a9cb61e6ULL, 0x2d5c37b21b6ec3cfULL, 0x95484cb21b68f270ULL, 0xf5338568e673d402ULL,
    0x8458bcf25028c573ULL, 0x7a7c1c97db362112ULL, 0xc5d0d1d345933dd7ULL, 0xcfcedd2dcbec4aeeULL,
    0xa059b7b78dde6898ULL, 0xdf746d14400a2e16ULL, 0xcd696637c75b3a7dULL, 0x738394ed38e355aaULL,
    0x04de61e216c0637dULL, 0xd7e768f1bcc5a656ULL, 0xf865bf25720cd72fULL, 0xb44ba548cd53ebfcULL
};

void STFT_Test(uint64_t* stft_in, uint64_t *hw_out, uint32_t *sw_out, int type, int print_values, uint8_t pixfloor, uint8_t datatest)
{
    uint64_t i,j;

    uint64_t data_in_init_u64[64];
    uint32_t data_out_u32[64];

    STFT_Config stft_config;
    STFT_Config stft_config_old;

    uint64_t tt_start, tt_stop;

    uint64_t dur_hw, dur_sw;

    /* Buffers must live in the uncached DMA pool, not on the stack. */
    
    int16_t* stft_data_in_init_i16 = (datatest == 0) ? stft_data_in_init_i16_sins : stft_data_in_init_i16_testdata; 
    uint64_t* stft_data_in_init_u64 = (datatest == 0) ? stft_data_in_init_u64_sins : stft_data_in_init_u64_testdata;
    
    for (i = 0; i < 128; i+=2) 
    {
        uint64_t p = 0;
        p|= ((uint64_t)((uint32_t)((int32_t)stft_data_in_init_i16[i+0]))) <<  0;   //correct for DWC 32 to 64
        p|= ((uint64_t)((uint32_t)((int32_t)stft_data_in_init_i16[i+1]))) << 32;
        data_in_init_u64[i/2] = p;
    }

    if (!stft_in || !hw_out || !sw_out) {
        fprintf(stderr, "FIDCT2D_Test: buffer allocation failed\n");
        return;
    }

    memcpy(stft_in, data_in_init_u64, sizeof(data_in_init_u64));
    
    for (i = 0; i < 32; ++i) {
        hw_out[i] = 0xFFFFFFFFFFFFFFFFul;
    }
    for (i = 0; i < 64; ++i) {
        sw_out[i] = 0xFFFFFFFFul;
    }

    if(print_values == 1)
    {
        printf("SW Inputs u | HW Inputs init u | HW Inputs:\r\n");
        for(i=0;i<128;i+=2)
        {
            printf("%08X %08X | %016lX | %016lX\r\n", stft_data_in_init_i16[i], stft_data_in_init_i16[i+1], stft_data_in_init_u64[i/2], stft_in[i/2]);
        }
    }
    
    if(type == 0)
    {   
        tt_start = time_now();
        stft_ref_complex(stft_data_in_init_i16, 1, sw_out);
        tt_stop = time_now();
        dur_sw = tt_stop - tt_start;

        stft_config.enable = 1;
        stft_config.out_format = FMT_CPX;
        stft_config.window_sel = WSEL_HANN;
        stft_config.pix_flor = pixfloor;
    }
    else if(type == 1)
    {
        tt_start = time_now();
        stft_ref_power32(stft_data_in_init_i16, 1, sw_out);
        tt_stop = time_now();
        dur_sw = tt_stop - tt_start;

        stft_config.enable = 1;
        stft_config.out_format = FMT_PWR32;
        stft_config.window_sel = WSEL_HANN;
        stft_config.pix_flor = pixfloor;
    }
    else if(type == 2)
    {
        tt_start = time_now();
        stft_ref_pixel(stft_data_in_init_i16, 1, pixfloor, sw_out);
        tt_stop = time_now();
        dur_sw = tt_stop - tt_start;

        stft_config.enable = 1;
        stft_config.out_format = FMT_PIX8;
        stft_config.window_sel = WSEL_HANN;
        stft_config.pix_flor = pixfloor;
    }
    else
    {
        tt_start = time_now();
        stft_ref_pixel(stft_data_in_init_i16, 1, pixfloor, sw_out);
        tt_stop = time_now();
        dur_sw = tt_stop - tt_start;

        stft_config.enable = 1;
        stft_config.out_format = FMT_PIX8;
        stft_config.window_sel = WSEL_RECT;
        stft_config.pix_flor = pixfloor;
    }
    
    STFT_Reset();
    STFT_Configure(stft_config, &stft_config_old);
    STFT_Start();

    RVEXP_DMA_SelectDevice(2);
    tt_start = time_now();
    RVEXP_DMA_Transaction(stft_in, hw_out, 512, 256);
    tt_stop = time_now();
    dur_hw = tt_stop - tt_start;

    double speedup;
    speedup = ((double)(dur_sw))/((double)(dur_hw));
    printf("STFT SW Implementation Execution Latency = %lx cycles @ 100 MHz\n", dur_sw);
    printf("STFT Hardware Accelerator Execution Latency = %lx cycles @ 100 MHz\n", dur_hw);
    printf("STFT Hardware Acceleration Speedup = %lf x @\n", speedup);

    for (i = 0; i < 32; i++) 
    {
        data_out_u32[i * 2 + 0] = (uint32_t)((hw_out[i] >>  0) & 0xFFFFFFFFu);     // high 32 bits //correct for DWC 32 64
        data_out_u32[i * 2 + 1] = (uint32_t)((hw_out[i] >> 32) & 0xFFFFFFFFu);             // low 32 bits
    }

    if(print_values == 1)
    {
        printf("STFT SW result | STFT HW result\r\n");
        for(i=0;i<64;++i)
        {
            printf("%08X | %08X\r\n", sw_out[i], data_out_u32[i]);
        }
    }
    
    //Cleanup
    STFT_Status stft_stat = STFT_GetStatus();
    printf("STFT_Busy = %u , STFT_done = %u , STFT_full = %u , STFT_fcount = %08X\r\n",
        stft_stat.ctrl_busy, stft_stat.frame_done, stft_stat.frame_full, stft_stat.frame_count
    );

    printf("STFT Test Succes!\n");
}

static const uint64_t z_init[256] = {
    0xA3F2C81D7E459B06ULL, 0x2F1D8C3A9B4E2F71ULL, 0xC8D3A5914F2E8B1DULL, 0x7A3C9F25E6B14D83ULL,
    0x1C7A2F958E3D6B14ULL, 0xA59C2F713D8E1B4AULL, 0x6F2C9D53B1E4A87FULL, 0x2D5C9A31E8F4B2C6ULL,
    0x5C1A9E37B4F2D806ULL, 0xA3E7C1928D4F6B25ULL, 0x1E9C3A74F2B58D1EULL, 0x6C4A9F37B1D2E854ULL,
    0x3A7F1C96D5E2B481ULL, 0x9C3F7A2EB4D16C85ULL, 0x2F8A3E971D4C9B5FULL, 0xA6E3F2817C4B9D15ULL,
    0x8E2F4A1CB7D39506ULL, 0xF1C4A82E3D9B5F17ULL, 0xA2E8C3F41B6D9A52ULL, 0x7F3C8E14D5A2B936ULL,
    0x4C1F7E93A8D2B546ULL, 0x3E9C1F72B4A57D18ULL, 0x6F2E9C43D1B58A27ULL, 0x9E4C3F16A7B2D581ULL,
    0x1F7C4B9EA2D83651ULL, 0xC9F2A7145E8B3D26ULL, 0xF4A19C827D3E5B14ULL, 0xA6C2F9813E5D7B42ULL,
    0x9C1F4A83D7E2B516ULL, 0x4A8C3F921E6B9D45ULL, 0xF7A2C3845D1E9B67ULL, 0xA3F8C2E16B4D9571ULL,
    0xD4A28F136C5E9B72ULL, 0x3F1A8C4DB9E25F61ULL, 0x7C3A9F14E8D25B43ULL, 0x1F9C4A72A6B3D815ULL,
    0x5E2F8C319D4A7B16ULL, 0xC3F1E8526A9D4C27ULL, 0xB8F2E3154C7A9D62ULL, 0x1E3F8B549A5D2C87ULL,
    0xF3A1C85E2D9B4F71ULL, 0x6C8A3F14B2E79D45ULL, 0x9D2C5F81A4E3B716ULL, 0x3F8B1C92E7A45D26ULL,
    0xB4F2E8371C9A5D63ULL, 0x7A1F3C94D6E2B851ULL, 0xE8C3A5F12D7B4916ULL, 0x4F9A2C71B8E35D18ULL,
    0x2C8F4A96E1B57D32ULL, 0xA7D3F18C5E9B2461ULL, 0x6B1E4C97F3A28D51ULL, 0x9C5F2A83D4B71E36ULL,
    0x3E8A1F72C6D94B52ULL, 0xF2C7B4193A8E5D61ULL, 0x8D3A9F56C2E1B742ULL, 0x1B6C4F82E9A37D51ULL,
    0xD5A2C8F13B7E4916ULL, 0x4F1C9B73A6E28D52ULL, 0x7E3F5A14C8B92D61ULL, 0xB2D4F891A3C57E26ULL,
    0x6A9C3E75F1D28B41ULL, 0xC4B87F23E5A19D62ULL, 0x3D1A5F96B8C2E741ULL, 0x9F4C2B18E7A35D63ULL,
    0xA1F3C85E2D9B4671ULL, 0x5C8A2F14B7E39D42ULL, 0xE9D3A6F12C8B5714ULL, 0x4A1F7C93D5E2B861ULL,
    0x2B8C4F96E1A57D32ULL, 0xA6D2F18C5E9B3471ULL, 0x7B1E3C97F2A48D51ULL, 0x8C5F1A83D4B62E36ULL,
    0x3F8A2E72C6D94B51ULL, 0xF1C7B4193A8D5E61ULL, 0x9D2A8F56C3E1B742ULL, 0x1A6C3F82E8A47D51ULL,
    0xD4A1C8F13B6E5916ULL, 0x5F1C9B73A7E28D42ULL, 0x6E3F4A14C9B82D61ULL, 0xB1D3F891A4C57E26ULL,
    0x7A8C2E75F1D39B41ULL, 0xC3B87F23E6A18D62ULL, 0x2D1A4F96B9C3E741ULL, 0x8F3C1B18E6A25D63ULL,
    0xF2A4C85E3D8B5671ULL, 0x4C9A1F14B6E38D42ULL, 0xE8D2A6F13C9B4714ULL, 0x5A2F6C93D4E1B861ULL,
    0x1B9C3F96E2A47D32ULL, 0xA5D1F28C6E8B3471ULL, 0x6B2E4C97F3A58D51ULL, 0x9C4F2A83D5B71E36ULL,
    0x2E9A1F72C7D83B51ULL, 0xF3C6B4193B8D4E61ULL, 0x8D1A9F56C4E2B742ULL, 0x2A5C4F82E9B36D51ULL,
    0xD3A2C8F14B5E6916ULL, 0x4E2C8B73A6F19D42ULL, 0x7F2E5A14C8B93D61ULL, 0xB3D2F891A5C46E26ULL,
    0x5A9C1E75F2D38B41ULL, 0xC2B96F23E7A28D62ULL, 0x1D2A3F96B8C4E741ULL, 0x9F2C1B18E5A36D63ULL,
    0xE3A5C84E2D9B6571ULL, 0x3C8A1F24B5E49D42ULL, 0xF9D3A5F12C8B6714ULL, 0x4B1F5C93D3E2B861ULL,
    0x2A8C2F96E3A56D32ULL, 0xB5D2F18C7E9A3471ULL, 0x5B1E3C97F4A69D51ULL, 0x8C3F1A83D6B52E36ULL,
    0x1E8A3F72C5D94B51ULL, 0xF4C5B4193C8D3E61ULL, 0x7D2A8F56C5E3B742ULL, 0x3A4C5F82E8B47D51ULL,
    0xD2A3C8F15B4E7916ULL, 0x3E1C9B73A8F28D42ULL, 0x6F1E4A14C7B94D61ULL, 0xB4D1F891A6C35E26ULL,
    0x4A8C3E75F3D27B41ULL, 0xC1B85F23E8A39D62ULL, 0x3D1A2F96B7C5E741ULL, 0x8E1C2B18F5A47D63ULL,
    0xC4A6F85E1D8B7471ULL, 0x2C9A3F14B4E58D42ULL, 0xE8D4A5F23C9B5714ULL, 0x3B2F4C93D2E1B961ULL,
    0x1A9C1F96E4A65D32ULL, 0xB4D3F28C8E9A2471ULL, 0x4B2E2C97F5A78D51ULL, 0x7C2F1A83D7B43E36ULL,
    0x3F7A4E72C4D85B51ULL, 0xF5C4B4193D8C2E61ULL, 0x6D3A8F56C6E4B742ULL, 0x4B3C6F82E7A58D51ULL,
    0xD1A4C8F16B3E8916ULL, 0x2E3C9B73A9F37D42ULL, 0x5F3E3A14C6B85D61ULL, 0xB5D4F891A7B24E26ULL,
    0x3A7C4E75F4D16B41ULL, 0xC8B74F23E9A48D62ULL, 0x4D2A1F96B6C6E741ULL, 0x7E2C3B18F4A58D63ULL,
    0xD5A7F85E2D7B8471ULL, 0x1C8A4F14B3E67D42ULL, 0xE7D5A4F34C8B4714ULL, 0x2B3F3C93D1E2BA61ULL,
    0x3A8C8F96E5A74D32ULL, 0xC3D4F28C9E8A1471ULL, 0x3B3E1C97F6A87D51ULL, 0x6C1F2A83D8B34E36ULL,
    0x4E6A5E72C3D96B51ULL, 0xF6C3B4193E8C1E61ULL, 0x5D4A8F56C7E5B742ULL, 0x5B2C7F82E6B69D51ULL,
    0xD8A5C8F17B2E9916ULL, 0x1E4C9B73BAF46D42ULL, 0x4F4E2A14C5B96E61ULL, 0xB6D5F891A8B13E26ULL,
    0x2A6C5E75F5D25B41ULL, 0xC7B63F23EAA57D62ULL, 0x5D3A8F96B5C7E741ULL, 0x6E3C4B18F3A69D63ULL,
    0xE6A8F85E3C6B9471ULL, 0x8C7A5F14B2E78D42ULL, 0xE6D6A3F45D8B3714ULL, 0x1B4F2C93D8E3BB61ULL,
    0x4B7C9F96E6A83D32ULL, 0xD2E5F28CAEBA8471ULL, 0x2B4F8C97F7A96D51ULL, 0x5C8E3A83D9B25E36ULL,
    0x5F5A6E72C2DA7B51ULL, 0xF7C2B4193F8B8E61ULL, 0x4E5A8F56C8E6B742ULL, 0x6B1C8F82E5B7AD51ULL,
    0xD7A6C8F18B1EAA16ULL, 0x8E5C9B73CBF55D42ULL, 0x3F5E1A14C4BA7F61ULL, 0xB7D6F891A9A22E26ULL,
    0x1A5C6E75F6D34B41ULL, 0xC6B52F23EBA66C62ULL, 0x6E4A9F96B4C8E741ULL, 0x5E4C5B18F2A7AD63ULL,
    0xF7A9F85E4B5BAA71ULL, 0x9C6A6F14B1E89D42ULL, 0xE5D7A2F56E8B2714ULL, 0x8B5F1C93D7E4BC61ULL,
    0x5B6CAF96E7A92D32ULL, 0xE1F6F28CBFBA7471ULL, 0x1B5F7C97F8AA5D51ULL, 0x4C7F4A83DAB16E36ULL,
    0x6E4A7E72C1DB8B51ULL, 0xF8C1B419408AAE61ULL, 0x3F6A8F56C9E7B742ULL, 0x7B8C9F82E4B8BE51ULL,
    0xD6A7C8F19B8EBB16ULL, 0x9F6CAB73DCF64D42ULL, 0x2F6F8A14C3BB8061ULL, 0xB8D7F891AAA11E26ULL,
    0x8A4C7E75F7D43B41ULL, 0xC5B41F23ECA77B62ULL, 0x7F5AAFF6B3C9E741ULL, 0x4F5C6B18F1A8BE63ULL,
    0x08AAFE5E5A4ABB71ULL, 0xAC5A7F14B8E9AD42ULL, 0xE4D8A1F67F8B1714ULL, 0x9B6E8C93D6E5BD61ULL,
    0x6B5BBF96E8AA1D32ULL, 0xF8F7F28CD0BA6471ULL, 0x8B6F6C97F9BB4D51ULL, 0x3C6F5A83DBB07E36ULL,
    0x7F3A8E72C8DC9B51ULL, 0xF9C8B41941ABBD61ULL, 0x2F7A8F56CAE8B742ULL, 0x8BACACF82E3B9CF5ULL,
    0xD5A8C8F1AABECB16ULL, 0xAF7CBB73EDF74D42ULL, 0x1F7F7914C2BC9161ULL, 0xB9D8F891ABA80E26ULL,
    0x9A3C8E75F8D52B41ULL, 0xC4B38F23EDC88A62ULL, 0x806BBFF6B2CAE741ULL, 0x3F6C7B18F8A9CF63ULL,
    0x19BBFD5E6B39CC71ULL, 0xBC4A8F14B7EABE42ULL, 0xE3D9A8F780AB8714ULL, 0xAB7F5C93D5E6BE61ULL,
    0x7B4CCF96E9BB8D32ULL, 0x0908F28CE1BA5471ULL, 0x9B7F5C97FACC3D51ULL, 0x2C5E6A83DCB18E36ULL,
    0x8E2A9E72C7DDAB51ULL, 0xFACFB41942BCCC61ULL, 0x1F8A8F56CBE9B742ULL, 0x9CBCADF82F2AADD1ULL,
    0xD4A9C8F1BBBFDC16ULL, 0xB08DCB73FEF84D42ULL, 0x8F8F6814C1BDAB61ULL, 0xBAD9F891BCB9FE26ULL,
    0xAA2C9E75F9E61B41ULL, 0xC3B27F23EEC99B62ULL, 0x917CCFF6B1CBF741ULL, 0x2F7C8B18F9AADF63ULL,
    0xD5A7F85E2D7B8471ULL, 0x1C8A4F14B3E67D42ULL, 0xE7D5A4F34C8B4714ULL, 0x2B3F3C93D1E2BA61ULL,
    0x3A8C8F96E5A74D32ULL, 0xC3D4F28C9E8A1471ULL, 0x3B3E1C97F6A87D51ULL, 0x6C1F2A83D8B34E36ULL,
    0x4E6A5E72C3D96B51ULL, 0xF6C3B4193E8C1E61ULL, 0x5D4A8F56C7E5B742ULL, 0x5B2C7F82E6B69D51ULL,
};

void FIDCT2D_Test(uint64_t* fdct_in, uint64_t* fdct_hw_out, uint64_t* fdct_sw_out, uint64_t* idct_hw_out, uint64_t* idct_sw_out, int print_values)
{
    uint64_t i;
    uint64_t tt_start, tt_stop;
    uint64_t dur_hw, dur_sw;
    uint64_t *PSRC, *PDST;
    double mpxps_sw, mpxps_hw;
    double speedup;

    uint64_t dur_hw1, dur_sw1;
    uint64_t *PSRC1, *PDST1;
    double mpxps_sw1, mpxps_hw1;
    double speedup1;

    memcpy(fdct_in, z_init, sizeof(z_init));
    for (i = 0; i < 512; ++i) {
        fdct_hw_out[i] = 0xFFFFFFFFFFFFFFFFul;
    }
    for (i = 0; i < 256; ++i) {
        idct_hw_out[i] = 0xFFFFFFFFFFFFFFFFul;
    }

    PSRC = &fdct_in[0];
    PDST = &fdct_sw_out[0];

    const uint8_t QC = 1;
    uint8_t qm[8][8] = {
        {QC, QC, QC, QC, QC, QC, QC, QC}, 
        {QC, QC, QC, QC, QC, QC, QC, QC}, 
        {QC, QC, QC, QC, QC, QC, QC, QC}, 
        {QC, QC, QC, QC, QC, QC, QC, QC},
        {QC, QC, QC, QC, QC, QC, QC, QC},
        {QC, QC, QC, QC, QC, QC, QC, QC},
        {QC, QC, QC, QC, QC, QC, QC, QC},
        {QC, QC, QC, QC, QC, QC, QC, QC}
    };

    int32_t rm[64]; //reciprocal quantization matrix
    LLM_FDCT_2D_Q_get_reciprocals_for_qantization_components(rm, &qm[0][0]);

    tt_start = time_now();
    for(int i=0;i<32;i++)
        LLM_FDCT_2D_Q_u8_2_s13(PSRC+i*0x8, PDST+i*0x10, rm);
    tt_stop = time_now();
    dur_sw = tt_stop - tt_start;

    RVEXP_DMA_SelectDevice(0);
    tt_start = time_now();
    RVEXP_DMA_Transaction(fdct_in, fdct_hw_out, 2048, 4096);
    tt_stop = time_now();
    dur_hw = tt_stop - tt_start;

    PSRC1 = &fdct_sw_out[0];
    PDST1 = &idct_sw_out[0];
    tt_start = time_now();
    for(int i=0;i<32;i++)
        LLM_IDCT_2D_Q_s13_2_u8(PSRC1+i*0x10, PDST1+i*0x8, qm);
    tt_stop = time_now();
    dur_sw1 = tt_stop - tt_start;

    tt_start = time_now();
    RVEXP_DMA_SelectDevice(1);
    RVEXP_DMA_Transaction(fdct_hw_out, idct_hw_out, 4096, 2048);
    tt_stop = time_now();
    dur_hw1 = tt_stop - tt_start;

    printf("FDCT Test Results SW vs HW:\n");
    for(int k=0;k<512;k+=4)
    {
        printf("%016lx %016lx %016lx %016lx || %016lx %016lx %016lx %016lx\n", 
            fdct_sw_out[k+0], fdct_sw_out[k+1], fdct_sw_out[k+2],fdct_sw_out[k+3],
            fdct_hw_out[k+0], fdct_hw_out[k+1], fdct_hw_out[k+2],fdct_hw_out[k+3]
        );
    }

    uint8_t diff[8];
    uint64_t diff_sum = 0;
    for(int k=0;k<256;++k)
    {
        for(int m=0;m<8;++m)
        {
            diff[m] = (uint8_t)((fdct_in[k] >> ((8-m-1)*8))&0xFFu) - (uint8_t)((idct_hw_out[k] >> ((8-m-1)*8))&0xFFu);
            if(print_values == 1) printf("%02X ", diff[m]);
            diff_sum += diff[m];
        }
        if(print_values == 1) printf("\n");
    }
    printf("FDCT2D-IDCT2D Test Succes!\n");

    printf("-------------------------------------------------------------------------------\n");
    mpxps_sw = (((double)1000000000*(double)2048 / (((double)(dur_sw)*10))))/1000000;
    printf("TUIASI FDCT 2D RISC-V Accelerator statistics:\n");
    printf("-------------------------------------------------------------------------------\n");
    printf("Processing 32 blocks of 64 pixels (8 X 8-bit pixels\n");
    printf("SW FDCT 2D Implementation: %lu cycles @ 100 MHz -> %8.3lf MPixels/s\n", (unsigned long)(dur_sw), mpxps_sw);
    printf("-------------------------------------------------------------------------------\n");
    mpxps_hw = (((double)1000000000*(double)2048 / (((double)(dur_hw)*10))))/1000000;
    printf("HW FDCT 2D Accelerator   : %lu cycles @ 100 MHz -> %8.3lf MPixels/s\n", (unsigned long)(dur_hw), mpxps_hw);
    printf("-------------------------------------------------------------------------------\n");
    speedup = ((double)(dur_sw))/((double)(dur_hw));
    printf("FDCT2D Hardware Acceleration Speedup = %lf x\n", speedup);
    printf("-------------------------------------------------------------------------------\n\n");

    printf("-------------------------------------------------------------------------------\n");
    mpxps_sw1 = (((double)1000000000*(double)2048 / (((double)(dur_sw1)*10))))/1000000;
    printf("TUIASI IDCT 2D RISC-V Accelerator statistics:\n");
    printf("-------------------------------------------------------------------------------\n");
    printf("Processing 32 blocks of 64 pixels (16 X 16-bit pixels\n");
    printf("SW IDCT 2D Implementation: %lu cycles @ 100 MHz -> %8.3lf MPixels/s\n", (unsigned long)(dur_sw1), mpxps_sw1);
    printf("-------------------------------------------------------------------------------\n");
    mpxps_hw1 = (((double)1000000000*(double)2048 / (((double)(dur_hw1)*10))))/1000000;
    printf("HW IDCT 2D Accelerator   : %lu cycles @ 100 MHz -> %8.3lf MPixels/s\n", (unsigned long)(dur_hw1), mpxps_hw1);
    printf("-------------------------------------------------------------------------------\n");
    speedup = ((double)(dur_sw1))/((double)(dur_hw1));
    printf("IDCT2D Hardware Acceleration Speedup = %lf x\n", speedup);
    printf("-------------------------------------------------------------------------------\n\n");
}

//Processing Loop Helper Function
void u64_to_u16_array(uint64_t input[(N_SAMPLES/4)], uint16_t output[N_SAMPLES])
{
    int i;
    for(i=0;i<N_SAMPLES;i+=4) //2048
    {
        output[i+0] = (uint16_t)((input[i/4] >> (3 * 16)) & 0xFFFFu);
        output[i+1] = (uint16_t)((input[i/4] >> (2 * 16)) & 0xFFFFu);
        output[i+2] = (uint16_t)((input[i/4] >> (1 * 16)) & 0xFFFFu);
        output[i+3] = (uint16_t)((input[i/4] >> (0 * 16)) & 0xFFFFu);
    }
}


int16_t map12_to_i16_correct(uint16_t v)
{
    v &= 0x0FFF;                            /* guard against out-of-range input */
    uint16_t u = (uint16_t)((v << 4));   /* 0..4095 -> 0..65535 */
    return (int16_t)((int32_t)u - 32768);
}

void u16_to_i16_12b_16bi_array(uint16_t input[N_SAMPLES], int16_t output[N_SAMPLES])
{
    int i;
    for(i=0;i<N_SAMPLES;++i)
    {
        output[i] = map12_to_i16_correct(input[i]);
    }
}

void i16_to_u64_array(int16_t input[N_SAMPLES], uint64_t output[(N_SAMPLES/2)])
{
    int i;

    for(i=0;i<N_SAMPLES;i+=2)
    {
        output[i/2] = 0;
        output[i/2] |= ((uint64_t)((uint32_t)((int32_t)input[i+0]))) <<  0;
        output[i/2] |= ((uint64_t)((uint32_t)((int32_t)input[i+1]))) << 32;
    }
}

void u64_12ub_to_u64_16ib(uint64_t input[(N_SAMPLES/4)], uint64_t output[(N_SAMPLES/2)])
{
    uint16_t temp1[N_SAMPLES];
    int16_t  temp2[N_SAMPLES];

    u64_to_u16_array(input, temp1);
    u16_to_i16_12b_16bi_array(temp1, temp2);
    i16_to_u64_array(temp2, output);
}

void u64_to_u32_le_be_array(uint64_t input[NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64], uint32_t output[(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2])
{
    int i;
    for(i=0;i<(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64);++i)
    {
        output[i * 2 + 0] = (uint32_t)((input[i] >>  0) & 0xFFFFFFFFu);     // high 32 bits //correct for DWC 32 64
        output[i * 2 + 1] = (uint32_t)((input[i] >> 32) & 0xFFFFFFFFu);
    }
}

void u32_to_u8_for_fdct2d_array(uint32_t input[(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2], uint8_t output[(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2])
{
    int i;
    for(i=0;i<((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2);++i)
    {
        output[i] = (uint8_t)(input[i] & 0xFFu);
    }
}

void u8_to_u64_for_fdct2d_array(uint8_t input[(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2], uint64_t output[((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2) / 8])
{
    int i;
    for(i=0;i<(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2;i+=8)
    {
        output[i/8] = 0;
        output[i/8] |= (((uint64_t)input[i + 0]) << (7 * 8));
        output[i/8] |= (((uint64_t)input[i + 1]) << (6 * 8));
        output[i/8] |= (((uint64_t)input[i + 2]) << (5 * 8));
        output[i/8] |= (((uint64_t)input[i + 3]) << (4 * 8));
        output[i/8] |= (((uint64_t)input[i + 4]) << (3 * 8));
        output[i/8] |= (((uint64_t)input[i + 5]) << (2 * 8));
        output[i/8] |= (((uint64_t)input[i + 6]) << (1 * 8));
        output[i/8] |= (((uint64_t)input[i + 7]) << (0 * 8));
    }
}

uint16_t map_i16_to_u12(int16_t v)
{
    uint16_t u = (uint16_t)((int32_t)v + 32768);   /* -32768..32767 -> 0..65535 */
    return (uint16_t)(u >> 4);                     /* 0..65535 -> 0..4095 */
}

uint16_t map_u8_to_u12(uint8_t v)
{
    return (uint16_t)(((uint32_t)v)*4095/255);
}

const uint8_t g_stdTableY[8][8] = {	// Standard Quantization table for Y (luminance)
	{16,  11,  10,  16,  24,  40,  51,  61},
	{12,  12,  14,  19,  26,  58,  60,  55},
	{14,  13,  16,  24,  40,  57,  69,  56},
	{14,  17,  22,  29,  51,  87,  80,  62},
	{18,  22,  37,  56,  68, 109, 103,  77},
	{24,  35,  55,  64,  81, 104, 113,  92},
	{49,  64,  78,  87, 103, 121, 120, 101},
	{72,  92,  95,  98, 112, 100, 103,  99 } };
const uint8_t g_stdTableUV[8][8] = {	// Standard Quantization table for UV (Cromimances)
	{17,  18,  24,  47,  99,  99,  99,  99},
	{18,  21,  26,  66,  99,  99,  99,  99},
	{24,  26,  56,  99,  99,  99,  99,  99},
	{47,  66,  99,  99,  99,  99,  99,  99},
	{99,  99,  99,  99,  99,  99,  99,  99},
	{99,  99,  99,  99,  99,  99,  99,  99},
	{99,  99,  99,  99,  99,  99,  99,  99},
	{99,  99,  99,  99,  99,  99,  99,  99} };

void SetQuality(uint8_t quality, const uint8_t stdTable[8][8], uint8_t QTable[8][8])
{
	unsigned long temp, scale_factor;
	/* Safety limit the quality factor. Convert 0 to 1 to avoid zero divide. */
	scale_factor = quality == 0 ? 1 : quality > 100 ? 100 : quality;
	/* The standard quantization tables are used as-are (scaling 100) for a quality
	 * of 50. Qualities between 50..100 are converted to scaling percentage 200-2*Q;
	 * Qualities between 1..50 are converted to scaling percentage 5000/Q. */
	scale_factor = scale_factor < 50 ? 5000 / scale_factor : 200 - scale_factor * 2;
	for (size_t i = 0; i < 8; i++) {
		for (size_t j = 0; j < 8; j++) {
			temp = (scale_factor * stdTable[i][j] + 50L) / 100L;
			QTable[i][j] = temp < 1 ? 1 : temp > 255 ? 255 : (uint8_t)temp;
		}
	}
}

void Matrix2Vector(uint8_t Q_matrix[8][8], uint8_t Q_array[64])
{
    int i,j;
    for(i=0;i<8;++i)
    {
        for(j=0;j<8;++j)
        {
            Q_array[(8*i)+j] = Q_matrix[i][j];
        }
    }
}

void transpose(const uint8_t *src, uint8_t *dst, size_t n, size_t m)
{
    for (size_t i = 0; i < n; i++)
        for (size_t j = 0; j < m; j++)
            dst[j * n + i] = src[i * m + j];
}

void ProcessingLoop(uint8_t wsel, uint8_t pixfloor, uint64_t* stft_in, uint64_t* stft_out, uint64_t* fdct2d_in, uint64_t* fdct2d_out, uint64_t* idct2d_in, uint64_t* idct2d_out, int print_debug)
{
    uint8_t transfer_result;

    uint64_t u64_ad_values[(N_SAMPLES/4)]; //checked
    uint64_t u64_stft_input_values[(N_SAMPLES/2)]; //checked
    uint64_t u64_stft_output_values[NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64]; //checked
    uint32_t u32_stft_output_values[(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2]; //checked
    uint8_t  u8_stft_output_values[(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2];

    uint64_t fdct2d_input_values[(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8]; //to be send as stft_output values
    uint64_t fdct2d_output_values[(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8 * 2]; //to be send as fdct2d_output values
    uint64_t idct2d_output_values[(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8]; //to be send as idct2d_output values
    
    uint8_t ad_loop;
    uint8_t ad_full = 0;

    STFT_Config stft_config;
    STFT_Config stft_config_old;
    //int stft_res_cnt;

    int stft_in_start_index;
    int stft_out_start_index;

    uint64_t ua_array[4096];

    uint16_t da_array_u12[O_SAMPLES];
    uint64_t da_array_u12x4[(O_SAMPLES)/4];

    
    int i,j;
    int ua_loop;
    int da_loop;
    uint8_t ua_empty;
    uint8_t da_empty;

    stft_config.enable = 0; //1
    stft_config.out_format = FMT_PIX8;
    stft_config.window_sel = wsel;
    stft_config.pix_flor = pixfloor;

    //ADC Acquisition
    AD_Aq_Clear();
    AD_Aq_Start();
    ad_loop = 1;
    while(ad_loop == 1)
    {
        AD_Get_Statuses(&ad_full);
        if(ad_full == 1)
        {
            break;
        }
    }
    AD_Aq_Clear(); ad_full = 0;
    
    Read_AD_Array(u64_ad_values, (N_SAMPLES/4)); //checked

    u64_12ub_to_u64_16ib(u64_ad_values, u64_stft_input_values); //checked

    //STFT Loop
    stft_in_start_index = 0;
    stft_out_start_index = 0;

    STFT_Reset();
    STFT_Configure(stft_config, &stft_config_old);
    RVEXP_DMA_SelectDevice(2);
    for(i=0;i<NT_STFT;++i)
    {
        STFT_Status stft_stat;

        for(j=0;j<(I_SAMPLES/2);++j)
        {
            stft_in[j] = u64_stft_input_values[stft_in_start_index + j];
        }
        stft_in_start_index += (W_SAMPLES/2);
        STFT_Start();
        transfer_result = RVEXP_DMA_Transaction(stft_in, stft_out, ((I_SAMPLES/2) * 64 / 8), ((O_SAMPLES/2) * 64 / 8));
        if(transfer_result == 0x98) printf("Transfer result timeout at transfer %d\n", i);

        do{
            stft_stat = STFT_GetStatus();
        } while(stft_stat.frame_done == 0);
        
        for(j=0;j<(O_SAMPLES/2);++j)
        {
            u64_stft_output_values[stft_out_start_index+j] = stft_out[j];
        }
        stft_out_start_index += (O_SAMPLES/2);
    }

    u64_to_u32_le_be_array(u64_stft_output_values, u32_stft_output_values);
    u32_to_u8_for_fdct2d_array(u32_stft_output_values, u8_stft_output_values);
    u8_to_u64_for_fdct2d_array(u8_stft_output_values, fdct2d_input_values);
    
    //FDCT2D
    
    if(print_debug == 1) printf("STFT Finished\n");

    memcpy(fdct2d_in, fdct2d_input_values, sizeof(fdct2d_input_values));

    RVEXP_DMA_SelectDevice(0);
    RVEXP_DMA_Transaction(fdct2d_in, fdct2d_out, ((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8 * 64 / 8), ((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8 * 64 * 2 / 8));

    for(i=0;i<((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8 * 2);++i)
    {
        fdct2d_output_values[i] = fdct2d_out[i];
    }
    
    if(print_debug == 1) printf("FDCT2D Finished\n");

    memcpy(idct2d_in, fdct2d_output_values, sizeof(fdct2d_output_values));

    RVEXP_DMA_SelectDevice(1);
    RVEXP_DMA_Transaction(idct2d_in, idct2d_out, ((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8 * 64 * 2 / 8), ((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8 * 64 / 8));

    for(i=0;i<((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8);++i)
    {
        idct2d_output_values[i] = idct2d_out[i];
    }

    if(print_debug == 1) printf("IDCT2D Finished\n");

    for(i=0;i<O_SAMPLES;++i)
    {
        da_array_u12[i] = map_u8_to_u12(u8_stft_output_values[i]);
    }

    for(i=0;i<(O_SAMPLES/4);++i)
    {
        da_array_u12x4[i] = 0;
        da_array_u12x4[i] |= ((uint64_t)da_array_u12[(4*i)+0]) << (3*16);
        da_array_u12x4[i] |= ((uint64_t)da_array_u12[(4*i)+1]) << (2*16);
        da_array_u12x4[i] |= ((uint64_t)da_array_u12[(4*i)+2]) << (1*16);
        da_array_u12x4[i] |= ((uint64_t)da_array_u12[(4*i)+3]) << (0*16);
    }

    int uaidx1 = N_SAMPLES/4;
    int uaidx2 = uaidx1 + ((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8);
    int uaidx3 = uaidx2 + ((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8 * 2);
    int uaidx4 = uaidx3 + ((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8);

    if(print_debug == 1) printf("UIndexex computing Finished\n");

    for(i=0;i<(N_SAMPLES/4);++i)
    {
        ua_array[i+0] = u64_ad_values[i];
    }

    for(i=0;i<((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8);++i)
    {
        ua_array[i+uaidx1] = fdct2d_input_values[i];
    }

    for(i=0;i<(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8 * 2;++i)
    {
        ua_array[i+uaidx2] = fdct2d_output_values[i];
    }

    for(i=0;i<(NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8;++i)
    {
        ua_array[i+uaidx3] = idct2d_output_values[i];
    }

    for(i=0;i<(O_SAMPLES/4);++i)
    {
        ua_array[i+uaidx4] = da_array_u12x4[i];
    }

    Write_UA_Array(ua_array,((N_SAMPLES/4)+((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8)+((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8 * 2)+((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8)+(O_SAMPLES/4)));

    if(print_debug == 1) printf("UA Writing Finished\n");

    ua_loop = 1;
    UA_Clear();
    UA_Start();
    while(ua_loop == 1)
    {
        UA_Get_Statuses(&ua_empty);
        if(ua_empty)
        {
            UA_Clear();
            ua_loop = 0;
            break;
        }
    }
    UA_Clear();
    ua_empty = 0;

    if(print_debug == 1) printf("UA Sending Finished\n");

    Write_DA_Array(da_array_u12x4, (O_SAMPLES/4));

    if(print_debug == 1) printf("DA Writing Finished\n");

    da_loop = 1;
    DA_Clear();
    DA_Start();
    while(da_loop == 1)
    {
        DA_Get_Statuses(&da_empty);
        if(da_empty)
        {
            DA_Clear();
            da_loop = 0;
            break;
        }
    }

    if(print_debug == 1) printf("DA Sending Finished\n");

    if(print_debug == 1) printf("Processing Loop Finished\n");
}

int main(int argc, char **argv)
{
    int opt, processing_loop = 1;
    uint8_t ready = 0, stop = 0;
    uint8_t urqf = 100, urqf_new = 100;
    uint8_t pxf = 0, pxf_new = 0;
    uint8_t wsel = 0, wsel_new = 0;
    //int del1,del2,del3;
    int i;
    uint8_t q_matrix[8][8];
    uint8_t qm[64];
    int stfttd = 0;
     
    RVEXP_DMA_SelectDevice(0);
    RVEXP_DMA_Init();

    RVEXP_DMA_SelectDevice(1);
    RVEXP_DMA_Init_NoAlloc();

    RVEXP_DMA_SelectDevice(2);
    RVEXP_DMA_Init_NoAlloc();

    //STFT Test Mem
    uint64_t *test_stft_in     = (uint64_t *)DMA_Buf_Alloc(64 * sizeof(uint64_t), NULL); /* source */ //checked
    uint64_t *test_stft_hw_out = (uint64_t *)DMA_Buf_Alloc(32 * sizeof(uint64_t), NULL); /* dest   */ //checked
    uint32_t *test_stft_sw_out = (uint32_t *)DMA_Buf_Alloc(64 * sizeof(uint32_t), NULL); /* dest   */ //checked
    
    //FDCT2D-IDCT2D Test Mem
    uint64_t* test_fdct_in     = (uint64_t *)DMA_Buf_Alloc(256 * sizeof(uint64_t), NULL);
    uint64_t* test_fdct_hw_out = (uint64_t *)DMA_Buf_Alloc(512 * sizeof(uint64_t), NULL); 
    uint64_t* test_fdct_sw_out = (uint64_t *)DMA_Buf_Alloc(512 * sizeof(uint64_t), NULL);
    uint64_t* test_idct_hw_out = (uint64_t *)DMA_Buf_Alloc(256 * sizeof(uint64_t), NULL);
    uint64_t* test_idct_sw_out = (uint64_t *)DMA_Buf_Alloc(256 * sizeof(uint64_t), NULL);    

    //Processing loop Mem
    uint64_t* stft_in  = (uint64_t *)DMA_Buf_Alloc((I_SAMPLES/2) * sizeof(uint64_t), NULL); 
    uint64_t* stft_out = (uint64_t *)DMA_Buf_Alloc((O_SAMPLES/2) * sizeof(uint64_t), NULL); 

    uint64_t* fdct2d_in  = (uint64_t *)DMA_Buf_Alloc(((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8) * sizeof(uint64_t), NULL);  
    uint64_t* fdct2d_out = (uint64_t *)DMA_Buf_Alloc(((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8 * 2) * sizeof(uint64_t), NULL);

    uint64_t* idct2d_in  = (uint64_t *)DMA_Buf_Alloc(((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8 * 2) * sizeof(uint64_t), NULL);
    uint64_t* idct2d_out = (uint64_t *)DMA_Buf_Alloc(((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8) * sizeof(uint64_t), NULL); 

    for(i=0;i<64;++i) qm[i] = 1;
    Set_QM(qm);

    AD_Set_Lim((N_SAMPLES/4));
    UA_Set_Lim((N_SAMPLES/4)+((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8)+((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8 * 2)+((NT_STFT * (O_SAMPLES * O_SAMPLE_SIZE) / 64) * 2 / 8)+(O_SAMPLES/4));
    DA_Set_Lim((O_SAMPLES/4));

    while(1)
    {
        printf("OPTION   = "); scanf("%d", &opt);
        printf("STFTTD   = "); scanf("%d", &stfttd);

        if(opt == 1) //STFT Test
        {
            STFT_Test(test_stft_in, test_stft_hw_out, test_stft_sw_out, 2, 1, (uint8_t)pxf, (uint8_t)stfttd);
        }
        else if(opt == 2) //FDCT2D-IDCT2D Test
        {
            FIDCT2D_Test(test_fdct_in, test_fdct_hw_out, test_fdct_sw_out, test_idct_hw_out, test_idct_sw_out, 1);
        }
        else if(opt == 3)
        {
            ProcessingLoop((uint8_t)wsel, (uint8_t)pxf, stft_in, stft_out, fdct2d_in, fdct2d_out, idct2d_in, idct2d_out, 1);
        }
        else if(opt == 4)
        {
            int nn = 1000;
            do{
                ProcessingLoop((uint8_t)wsel, (uint8_t)pxf, stft_in, stft_out, fdct2d_in, fdct2d_out, idct2d_in, idct2d_out, 1);
            }while(nn-- > 0);
        }
        else if(opt == 5)
        {
            while(processing_loop == 1)
            {
                stop     = UR_GetStop(); 
                ready    = UR_GetReady();
                urqf_new = UR_GetQFact();
                pxf_new  = UR_GetPixFloor();
                wsel_new = UR_GetWindowSel();
                if(urqf != urqf_new)
                {
                    urqf = urqf_new;
                    SetQuality(urqf, g_stdTableY, q_matrix);
                    Matrix2Vector(q_matrix, qm);
                    Set_QM(qm);
                }
                if(pxf != pxf_new)
                {
                    pxf = pxf_new;
                }
                if(wsel != wsel_new)
                {
                    wsel = wsel_new;
                }
                
                if(stop == 1)
                {
                    stop = 0;
                    UR_ClearStop();
                    break;
                }
                if(ready == 1)
                { 
                    ready = 0;
                    ProcessingLoop((uint8_t)wsel, (uint8_t)pxf, stft_in, stft_out, fdct2d_in, fdct2d_out, idct2d_in, idct2d_out, 0);
                    UR_ClearReady();
                }
                
            }
        }
        else if(opt == 6)
        {
            while(processing_loop == 1)
            {
                stop = UR_GetStop(); 
                ready = UR_GetReady();
                if(stop == 1)
                {
                    stop = 0;
                    UR_ClearStop();
                    printf("UR STOP is 1\n");
                    break;
                }
                if(ready == 1)
                { 
                    ready = 0;
                    UR_ClearReady();
                    printf("UR READY is 1\n");
                }
                
            }
        }
        else if(opt == 0)
        {
            break;
        }
    }

    RVEXP_DMA_DeInit();

    return 0;
}
