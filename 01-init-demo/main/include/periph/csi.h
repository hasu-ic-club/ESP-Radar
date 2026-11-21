// + ----------------------------------------------------------------- +
// - File Name    : csi.h
// - Description  : Peripheral low-level functions for CSI camera link
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

#ifndef CSI_H
#define CSI_H

#include <driver/isp_types.h>
#include <esp_cam_ctlr_types.h>
#include <esp_ldo_regulator.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

typedef struct
{
    uint32_t bytes_per_chirp;
    uint32_t chirps_per_frame;
} radar_csi_config_t;

typedef struct
{
    uint8_t *buffer;
    size_t size;
    SemaphoreHandle_t done;
    bool submitted;
} radar_csi_rx_t;

esp_err_t init_csi_periph(esp_ldo_channel_handle_t* csi_ldo);
esp_err_t prepare_csi_capture(
    esp_cam_ctlr_handle_t* radar_cam,
    isp_proc_handle_t* radar_isp,
    const radar_csi_config_t* cfg,
    radar_csi_rx_t* rx);

#endif // CSI_H
