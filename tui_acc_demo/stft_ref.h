#ifndef STFT_REF_H
#define STFT_REF_H

#include <stdint.h>
#include <stddef.h>
#include "stft_coeffs.h"   

#ifdef __cplusplus
extern "C" {
#endif

void stft_ref_fft(const int16_t *in, int wsel,
                  int16_t *out_re, int16_t *out_im);

uint32_t stft_ref_power(int16_t re, int16_t im);

uint8_t  stft_ref_logpix(uint32_t pwr, int fl);

void stft_ref_complex(const int16_t *in, int wsel, uint32_t *out /*[STFT_N]*/);
void stft_ref_power32(const int16_t *in, int wsel, uint32_t *out /*[STFT_N]*/);
void stft_ref_pixel(const int16_t *in, int wsel, int fl,
                    uint8_t *out /*[STFT_NPIX]*/);
int stft_ref_check_complex(const int16_t *in, int wsel,
                           const uint32_t *hw /*[STFT_N]*/, int *first);
int stft_ref_check_power32(const int16_t *in, int wsel,
                           const uint32_t *hw /*[STFT_N]*/, int *first);
int stft_ref_check_pixel  (const int16_t *in, int wsel, int fl,
                           const uint8_t  *hw /*[STFT_NPIX]*/, int *first);
#ifdef __cplusplus
}
#endif
#endif /* STFT_REF_H */
