// + ----------------------------------------------------------------- +
// - File Name    : sdcard.h
// - Description  : Peripheral low-level functions for SD Card / SDIO
// - Author       : Onodera Tsusaki
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

#ifndef SDCARD_H
#define SDCARD_H

#include <sd_protocol_types.h>
#include <esp_err.h>

esp_err_t init_sdcard(sdmmc_card_t** sdcard_handle);

#endif // SDCARD_H
