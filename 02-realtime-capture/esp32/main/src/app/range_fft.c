// + ----------------------------------------------------------------- +
// - File Name    : range_fft.c
// - Description  : ESP32-P4 Range FFT implementation
// - Author       : Onodera T.
// - Date Created : 2026/11
// - Organization : Hasunosora IC Design Club.
// - Version      : 1.0
// + ----------------------------------------------------------------- +
// - Change Log   :
//   Date          Author            Version    Description
//   2026/11/21    Onodera T.        1.0        Initial version
// + ----------------------------------------------------------------- +
// Copyright (C) 2026 Hasunosora IC Design Club.
// All rights reserved.
// + ----------------------------------------------------------------- +

#include "app/range_fft.h"

#include <string.h>

#include <dsps_fft2r.h>

#if dsps_fft2r_fc32_arp4_enabled != 1
#error "The ESP32-P4 optimized Range FFT must be enabled"
#endif

static _Alignas(16) float twiddles[RADAR_ADC_MAX];
static _Alignas(16) float fft_work[RADAR_COMPLEX_WORDS * RADAR_ADC_MAX];

static uint16_t fft_length(uint16_t samples)
{
    uint16_t length = 2;
    while (length < samples)
    {
        length *= 2;
    }
    return length;
}

esp_err_t range_fft_init(void)
{
    return dsps_fft2r_init_fc32(twiddles, RADAR_ADC_MAX);
}

uint16_t range_fft_bins(uint16_t samples)
{
    return fft_length(samples) / 2;
}

esp_err_t range_fft_chirp(const int16_t* adc, uint16_t samples, bool descending,
                          float* output)
{
    const uint16_t length = fft_length(samples);
    const uint16_t bins = length / 2;

    for (unsigned rx = 0; rx < RADAR_RX_COUNT; ++rx)
    {
        for (unsigned sample = 0; sample < samples; ++sample)
        {
            // AWR2243 interleaves RX channels with I before Q.
            const size_t index = (sample * RADAR_RX_COUNT + rx) * RADAR_COMPLEX_WORDS;
            fft_work[2 * sample] = adc[index];
            fft_work[2 * sample + 1] = adc[index + 1];
        }
        memset(&fft_work[2 * samples], 0, 2 * (length - samples) * sizeof(float));

        // The P4 kernel uses hardware loops and fused floating-point operations.
        esp_err_t result = dsps_fft2r_fc32_arp4(fft_work, length);
        if (result == ESP_OK)
        {
            result = dsps_bit_rev2r_fc32(fft_work, length);
        }
        if (result != ESP_OK)
        {
            return result;
        }

        // Follow the beat frequency sign for each slope, with DC at bin zero.
        for (unsigned bin = 0; bin < bins; ++bin)
        {
            const unsigned source = descending && bin != 0 ? length - bin : bin;
            *output++ = fft_work[2 * source];
            *output++ = fft_work[2 * source + 1];
        }
    }
    return ESP_OK;
}
