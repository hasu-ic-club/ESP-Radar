// + ----------------------------------------------------------------- +
// - File Name    : radar.h
// - Description  : AWR2243 RF control
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

#include "app/protocol.h"

/// Initialize RF and preserve the failed step and native error code for status replies.
int32_t radar_init(const char** stage);
int32_t radar_configure(const usb_config_t* config, const usb_chirp_t* chirps,
                        const char** stage);
int32_t radar_trigger(bool start);
int32_t radar_power_down(void);
void radar_clear_events(void);
bool radar_frame_ended(void);
bool radar_faulted(void);
void radar_version(char* text, size_t size);
