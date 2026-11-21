// + ----------------------------------------------------------------- +
// - File Name    : csi.c
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

#include "periph/csi.h"

#include <driver/isp_core.h>

#include <esp_cam_ctlr.h>
#include <esp_cam_ctlr_csi.h>

#include <esp_cache.h>
#include <esp_check.h>
#include <esp_log.h>

static const char* TAG = "CSI";

static bool radar_csi_get_buffer(esp_cam_ctlr_handle_t cam,
    esp_cam_ctlr_trans_t* trans, void* arg);
static bool radar_csi_done(esp_cam_ctlr_handle_t cam,
    esp_cam_ctlr_trans_t* trans, void* arg);

esp_err_t init_csi_periph(esp_ldo_channel_handle_t* csi_ldo)
{
    const esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = 3,
        .voltage_mv = 2500
    };

    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_cfg, csi_ldo),
        TAG, "Failed to acquire LDO channel for CSI.");

    return ESP_OK;
}

esp_err_t prepare_csi_capture(
    esp_cam_ctlr_handle_t* radar_cam,
    isp_proc_handle_t* radar_isp,
    const radar_csi_config_t* cfg,
    radar_csi_rx_t* rx)
{
    const esp_cam_ctlr_csi_config_t csi_cfg = {
        .clk_src = MIPI_CSI_PHY_CLK_SRC_PLL_F25M,
        .ctlr_id = 0,
        .h_res = cfg->bytes_per_chirp,
        .v_res = cfg->chirps_per_frame,
        .data_lane_num = 2,
        .lane_bit_rate_mbps = 600,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RAW8,
        .queue_items = 1,
        .bk_buffer_dis = false
    };

    ESP_RETURN_ON_ERROR(esp_cam_new_csi_ctlr(&csi_cfg, radar_cam),
        TAG, "Failed to initialize CSI controller.");

    const esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RAW8,
        .has_line_start_packet = true,
        .has_line_end_packet = true,
        //.h_res = cfg->bytes_per_chirp,
        .v_res = cfg->chirps_per_frame,
        .h_res = 1920, // Fake resolution since we do not use ISP
        .flags.bypass_isp = true,
    };

    ESP_RETURN_ON_ERROR(esp_isp_new_processor(&isp_cfg, radar_isp),
        TAG, "Failed to initialize radar image ISP. H = %d, V = %d.", isp_cfg.h_res, isp_cfg.v_res);

    const uint64_t size = cfg->bytes_per_chirp * cfg->chirps_per_frame;
    rx->size = size;
    rx->buffer = heap_caps_aligned_alloc(
            CONFIG_CACHE_L2_CACHE_LINE_SIZE,
            size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);

    const esp_cam_ctlr_evt_cbs_t callbacks = {
        .on_get_new_trans = radar_csi_get_buffer,
        .on_trans_finished = radar_csi_done
    };

    ESP_RETURN_ON_ERROR(esp_cam_ctlr_register_event_callbacks(*radar_cam, &callbacks, rx),
        TAG, "Failed to register event callbacks.");
    ESP_RETURN_ON_ERROR(esp_cam_ctlr_enable(*radar_cam),
        TAG, "Failed to enable CSI controller.");
    ESP_RETURN_ON_ERROR(esp_cam_ctlr_start(*radar_cam),
        TAG, "Failed to start CSI controller.");

    return ESP_OK;
}

// Callbacks for CSI event
static bool IRAM_ATTR radar_csi_get_buffer(esp_cam_ctlr_handle_t cam, esp_cam_ctlr_trans_t* trans, void* arg)
{
    (void)cam;
    radar_csi_rx_t* rx = arg;
    if (!rx->submitted)
    {
        trans->buffer = rx->buffer;
        trans->buflen = rx->size;
        rx->submitted = true;
    }

    return false;
}

static bool IRAM_ATTR radar_csi_done(esp_cam_ctlr_handle_t cam, esp_cam_ctlr_trans_t* trans, void* arg)
{
    (void)cam;
    const radar_csi_rx_t *rx = arg;
    BaseType_t awakened = pdFALSE;
    if (trans->buffer == rx->buffer)
    {
        xSemaphoreGiveFromISR(rx->done, &awakened);
    }

    return awakened == pdTRUE;
}
