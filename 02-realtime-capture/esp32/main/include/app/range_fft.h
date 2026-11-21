// + ----------------------------------------------------------------- +
// - File Name    : range_fft.h
// - Description  : Range FFT interface
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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <esp_err.h>

#include "app/protocol.h"

esp_err_t range_fft_init(void);
uint16_t range_fft_bins(uint16_t samples);

/// Transform a single interleaved complex chirp into channel-major float I/Q pairs.
esp_err_t range_fft_chirp(const int16_t* adc, uint16_t samples, bool descending,
                          float* output);
