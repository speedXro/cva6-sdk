
#include "stft_ref.h"

static int64_t rnd_shr(int64_t x, int s)
{
    if (s == 0) return x;
    return (x + ((int64_t)1 << (s - 1))) >> s;   /* arithmetic shift assumed */
}

static int16_t sat16(int64_t x)
{
    if (x >  32767) return  32767;
    if (x < -32768) return -32768;
    return (int16_t)x;
}

/* reverse the low STFT_LOG2N bits of i */
static int bitrev(int i)
{
    int r = 0, b;
    for (b = 0; b < STFT_LOG2N; b++) { r = (r << 1) | (i & 1); i >>= 1; }
    return r;
}

/* -------------------------------------------------------------------------- */
void stft_ref_fft(const int16_t *in, int wsel, int16_t *out_re, int16_t *out_im)
{
    int16_t Ar[STFT_N], Ai[STFT_N];
    const int16_t *win = stft_win[wsel & (STFT_NWIN - 1)];
    int i, s, k, j;

    for (i = 0; i < STFT_N; i++) {
        int32_t xw = (int32_t)rnd_shr((int64_t)in[i] * win[i], 15);
        int br = bitrev(i);
        Ar[br] = sat16(xw);
        Ai[br] = 0;
    }


    for (s = 0; s < STFT_LOG2N; s++) {
        int m = 1 << (s + 1);          
        int h = m >> 1;                
        int stride = STFT_N / m;       
        for (k = 0; k < STFT_N; k += m) {
            for (j = 0; j < h; j++) {
                int16_t wr = stft_tw_re[j * stride];
                int16_t wi = stft_tw_im[j * stride];
                int p = k + j, q = p + h;
                int ur = Ar[p], ui = Ai[p];
                int vr = Ar[q], vi = Ai[q];
                /* t = v * w  (Q15), rounded */
                int64_t tr = rnd_shr((int64_t)vr * wr - (int64_t)vi * wi, 15);
                int64_t ti = rnd_shr((int64_t)vr * wi + (int64_t)vi * wr, 15);
                Ar[p] = sat16(rnd_shr(ur + tr, 1));
                Ai[p] = sat16(rnd_shr(ui + ti, 1));
                Ar[q] = sat16(rnd_shr(ur - tr, 1));
                Ai[q] = sat16(rnd_shr(ui - ti, 1));
            }
        }
    }
    for (i = 0; i < STFT_N; i++) { out_re[i] = Ar[i]; out_im[i] = Ai[i]; }
}

uint32_t stft_ref_power(int16_t re, int16_t im)
{
    int32_t r = re, i = im;                       
    return (uint32_t)(r * r) + (uint32_t)(i * i); 
}

uint8_t stft_ref_logpix(uint32_t pwr, int fl)
{
    static const int EW[4] = {4, 3, 5, 2};
    static const int MW[4] = {4, 5, 3, 6};
    static const int EF[4] = {16, 24, 0, 28};
    int ew, mw, efloor, msb, e;
    uint32_t norm, frac6, mant;

    if (pwr == 0) return 0;
    fl &= 3; ew = EW[fl]; mw = MW[fl]; efloor = EF[fl];

    msb = 31; while (!((pwr >> msb) & 1u)) msb--;  
    if (msb < efloor) return 0;

    norm  = pwr << (31 - msb);                     
    frac6 = (norm >> 25) & 0x3Fu;                 
    mant  = (frac6 >> (6 - mw)) & ((1u << mw) - 1u);
    e = msb - efloor;
    if (e > (1 << ew) - 1) e = (1 << ew) - 1;
    return (uint8_t)(((uint32_t)e << mw) | mant);
}

void stft_ref_complex(const int16_t *in, int wsel, uint32_t *out)
{
    int16_t re[STFT_N], im[STFT_N]; int i;
    stft_ref_fft(in, wsel, re, im);
    for (i = 0; i < STFT_N; i++)
        out[i] = ((uint32_t)(uint16_t)re[i] << 16) | (uint16_t)im[i];
}

void stft_ref_power32(const int16_t *in, int wsel, uint32_t *out)
{
    int16_t re[STFT_N], im[STFT_N]; int i;
    stft_ref_fft(in, wsel, re, im);
    for (i = 0; i < STFT_N; i++) out[i] = stft_ref_power(re[i], im[i]);
}

void stft_ref_pixel(const int16_t *in, int wsel, int fl, uint8_t *out)
{
    int16_t re[STFT_N], im[STFT_N]; int i;
    stft_ref_fft(in, wsel, re, im);
    for (i = 0; i < STFT_NPIX; i++)
        out[i] = stft_ref_logpix(stft_ref_power(re[i], im[i]), fl);
}

int stft_ref_check_complex(const int16_t *in, int wsel,
                           const uint32_t *hw, int *first)
{
    uint32_t ref[STFT_N]; int i, n = 0, f = -1;
    stft_ref_complex(in, wsel, ref);
    for (i = 0; i < STFT_N; i++)
        if (hw[i] != ref[i]) { if (f < 0) f = i; n++; }
    if (first) *first = f;
    return n;
}

int stft_ref_check_power32(const int16_t *in, int wsel,
                           const uint32_t *hw, int *first)
{
    uint32_t ref[STFT_N]; int i, n = 0, f = -1;
    stft_ref_power32(in, wsel, ref);
    for (i = 0; i < STFT_N; i++)
        if (hw[i] != ref[i]) { if (f < 0) f = i; n++; }
    if (first) *first = f;
    return n;
}

int stft_ref_check_pixel(const int16_t *in, int wsel, int fl,
                         const uint8_t *hw, int *first)
{
    uint8_t ref[STFT_NPIX]; int i, n = 0, f = -1;
    stft_ref_pixel(in, wsel, fl, ref);
    for (i = 0; i < STFT_NPIX; i++)
        if (hw[i] != ref[i]) { if (f < 0) f = i; n++; }
    if (first) *first = f;
    return n;
}
