// + ----------------------------------------------------------------- +
// - File Name    : radar_csi.h
// - Description  : PSRAM CSI frame ring interface
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
#include <stddef.h>
#include <stdint.h>

#include <esp_err.h>

typedef struct
{
    const int16_t* data;
    uint32_t index;
} radar_raw_frame_t;

esp_err_t radar_csi_init(void);
esp_err_t radar_csi_start(uint32_t bytes_per_chirp, uint32_t chirps_per_frame);

/// Stop DMA while keeping frames already queued available to the consumer.
esp_err_t radar_csi_stop(void);
void radar_csi_release(void);
bool radar_csi_take(radar_raw_frame_t* frame);
void radar_csi_return(const radar_raw_frame_t* frame);
uint32_t radar_csi_frames(void);
uint32_t radar_csi_dropped(void);
bool radar_csi_failed(void);
