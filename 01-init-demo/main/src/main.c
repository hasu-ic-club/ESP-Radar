// + ----------------------------------------------------------------- +
// - File Name    : main.c
// - Description  : Top application entry
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
#include <sd_protocol_types.h>

#include <esp_cache.h>
#include <esp_check.h>
#include <esp_log.h>

#include "soc/isp_struct.h"
#include "soc/mipi_csi_bridge_struct.h"
#include "soc/mipi_csi_host_struct.h"
#include "hal/mipi_csi_host_ll.h"

// FreeRTOS
#include <freertos/FreeRTOS.h>

// Peripherals
#include "periph/sdcard.h"
#include "periph/csi.h"

// Custom applications
#include "port/mmwavelink_port.h"

// mmWaveLink library
#include "esp_cam_ctlr.h"

#include <mmwavelink.h>
#include <mmwavelink_esp.h>
#include <mmwl_esp_freertos.h>

static const char* TAG = "main";

static mmwl_esp_context_t mmwl_handle;
static EventGroupHandle_t radar_async_event;

static sdmmc_card_t* sdcard_handle;

static radar_csi_rx_t radar_rx_handle;
static esp_cam_ctlr_handle_t radar_cam;
static isp_proc_handle_t radar_isp;
static esp_ldo_channel_handle_t csi_ldo;

constexpr static uint8_t device_map = RL_DEVICE_MAP_CASCADED_1;

static esp_err_t wait_for_radar_event(uint32_t event, uint32_t timeout);
static esp_err_t init_radar_rf_system(mmwl_esp_config_t* mmwl_config);
static esp_err_t config_radar_profile(rlFrameCfg_t* frame_config);
static esp_err_t start_radar_capture();
static esp_err_t save_radar_captured();

void app_main()
{
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "TI xWR Radar w/ ESP32 system is now starting up");
    radar_async_event = xEventGroupCreate();

    ESP_GOTO_ON_ERROR(init_sdcard(&sdcard_handle),
        critical_err, TAG, "Failed to initialize sdcard.");

    ESP_GOTO_ON_ERROR(init_radar_peripherals(&mmwl_handle),
        critical_err, TAG, "Failed to initialize radar peripherals.");

    ESP_GOTO_ON_ERROR(init_csi_periph(&csi_ldo),
        critical_err, TAG, "Failed to initialize CSI peripherals.");

    link_mmwl_handle(&mmwl_handle, &radar_async_event);
    ESP_LOGI(TAG, "Radar peripherals and mmWaveLink handle initialized.");

    // Configure the RF subsystem
    ESP_LOGI(TAG, "Run radar RF system initialization");
    mmwl_esp_config_t mmwl_config;
    ESP_GOTO_ON_ERROR(init_radar_rf_system(&mmwl_config),
        critical_err, TAG, "Failed to initialize RF subsystem.");

    // Configure the profiles (chirp, frame, etc.)
    ESP_LOGI(TAG, "Starting to configure the chirp profile and frames.");
    rlFrameCfg_t frame_config = {0};
    ESP_GOTO_ON_ERROR(config_radar_profile(&frame_config),
        critical_err, TAG, "Failed to configure profile.");

    const uint32_t bytes_per_chirp = frame_config.numAdcSamples * sizeof(int16_t) *
        __builtin_popcount(mmwl_config.rx_channel_enabled);
    const uint32_t chirps_per_frame = (1 + frame_config.chirpEndIdx - frame_config.chirpStartIdx) *
            frame_config.numLoops;

    // Configure the CSI receiver
    radar_rx_handle.done = xSemaphoreCreateBinary();
    const radar_csi_config_t csi_cfg = {
        .bytes_per_chirp = bytes_per_chirp,
        .chirps_per_frame = chirps_per_frame,
    };

    ESP_GOTO_ON_ERROR(prepare_csi_capture(&radar_cam, &radar_isp, &csi_cfg, &radar_rx_handle),
        critical_err, TAG, "Failed to prepare CSI capture.");
    ESP_GOTO_ON_ERROR(radar_rx_handle.submitted ? ESP_OK : ESP_ERR_INVALID_STATE,
        critical_err, TAG, "The CSI controller has not submitted the first frame buffer.");

    // Start the radar
    ESP_GOTO_ON_ERROR(start_radar_capture(),
        critical_err, TAG, "Failed to start the radar.");

    // Radar started, wait for capture
    if (xSemaphoreTake(radar_rx_handle.done, pdMS_TO_TICKS(10000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "The CSI has not received anything from radar after 10s.");
        goto critical_err;
    }

    ESP_LOGI(TAG, "Radar capture is done.");

    ESP_GOTO_ON_ERROR(save_radar_captured(),
        critical_err, TAG, "Failed to save the radar data.");

    ESP_LOGI(TAG, "All works done.");

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    critical_err:
    ESP_LOGE(TAG, "There is a critical error occurred and can not be recovery: %d.", ret);
    ESP_ERROR_CHECK(ESP_FAIL);
}

static esp_err_t wait_for_radar_event(uint32_t event, uint32_t timeout)
{
    auto eb = xEventGroupWaitBits(radar_async_event, event,
    pdTRUE,
    pdTRUE, pdMS_TO_TICKS(timeout));
    if ((eb & event) != event)
    {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static esp_err_t init_radar_rf_system(mmwl_esp_config_t* mmwl_config)
{
    // Power on the radar
    auto ret = rlDevicePowerOn(device_map, mmwl_handle.client_context);

    if (ret != 0)
    {
        ESP_LOGE(TAG, "Failed to power on the radar: %d.", ret);
        return ESP_FAIL;
    }

    ESP_RETURN_ON_ERROR(wait_for_radar_event(XWR_ASYNC_EVENT_RADAR_PWUP_DONE, 1000),
        TAG, "Failed to power up the radar.");

    ESP_LOGI(TAG, "Radar powered on. Start initialize radar array.");

    // Get the version of the radar and show
    rlFwVersionParam_t fw_ver;
    ret = rlDeviceGetMssVersion(device_map, &fw_ver);

    if (ret != 0)
    {
        ESP_LOGE(TAG, "Failed to get firmware version of radar.");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "HW Version: %d.%d.%d", fw_ver.hwMajor, fw_ver.hwMinor, fw_ver.hwVarient);
    ESP_LOGI(TAG, "MSS Version: %d.%d.%d.%d", fw_ver.fwMajor, fw_ver.fwMinor, fw_ver.fwBuild, fw_ver.fwDebug);
    ESP_LOGI(TAG, "MSS Build at %d/%d/%d", fw_ver.fwYear, fw_ver.fwMonth, fw_ver.fwDay);

    // Start radar RF subsystem
    ret = rlDeviceRfStart(device_map);

    if (ret != 0)
    {
        ESP_LOGE(TAG, "Failed to start radar RF subsystem.");
        return ESP_FAIL;
    }

    ESP_RETURN_ON_ERROR(wait_for_radar_event(XWR_ASYNC_EVENT_RF_PWUP_DONE, 1000),
        TAG, "Failed to start radar RF subsystem.");

    ESP_LOGI(TAG, "Radar RF subsystem started.");

    // Set standard subsystem configurations
    mmwl_config->cascade_role = MMWL_ESP_CASCADE_ROLE_SINGLE;

    mmwl_config->crc_type = RL_CRC_TYPE_32BIT;
    mmwl_config->rx_channel_enabled = 0b1111; // Enable all RX
    mmwl_config->tx_channel_enabled = 0b101;  // Enable TX0, TX2

    mmwl_config->adc_bits = RL_ADC_DATA_16_BIT;
    mmwl_config->adc_format = RL_ADC_FORMAT_COMPLEX_2X;
    mmwl_config->adc_iq_swap = 0;

    mmwl_config->packet0_type = MMWL_ESP_CSI_PACKET_0_ADC_ONLY;
    mmwl_config->packet0_virtual_channel = MMWL_ESP_CSI_VIRTUAL_CHANNEL(0);
    mmwl_config->packet1_type = MMWL_ESP_CSI_PACKET_1_NONE;
    mmwl_config->packet1_virtual_channel = MMWL_ESP_CSI_VIRTUAL_CHANNEL(1);
    mmwl_config->cq_size[0] = 128;
    mmwl_config->cq_size[1] = 128;
    mmwl_config->cq_size[2] = 128;

    mmwl_config->data_rate = MMWL_ESP_CSI_DATA_RATE_150MBPS;
    mmwl_config->csi_lane_setting = 0x30021; // Use 2-lane CSI interface

    // Write configurations to radar
    ret = mmwl_esp_pre_rf_init(device_map, mmwl_config);

    if (ret != 0)
    {
        ESP_LOGE(TAG, "Failed to configure the radar: %d.", ret);
        return ESP_FAIL;
    }

    // Initialize the RF subsystem with new configurations
    ret = rlRfInit(device_map);
    if (ret != 0)
    {
        ESP_LOGE(TAG, "Failed to initialize the RF subsystem: %d.", ret);
        return ESP_FAIL;
    }


    ESP_RETURN_ON_ERROR(wait_for_radar_event(XWR_ASYNC_EVENT_CALIB_DONE, 1000),
        TAG, "Failed to run initial calibration.");

    ESP_LOGI(TAG, "Initial calibration started.");

    ret = mmwl_esp_post_rf_init(device_map, mmwl_config);

    if (ret != 0)
    {
        ESP_LOGE(TAG, "Failed to run post RF initialize: %d.", ret);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Radar subsystem has been configured.");
    return ESP_OK;
}

static esp_err_t config_radar_profile(rlFrameCfg_t* frame_config)
{
    rlProfileCfg_t profile =
    {
        .profileId = 0,
        .pfVcoSelect = 0b000,                 // Use VCO1 and internal VCO selection
        .pfCalLutUpdate = 0b00,               // Automatically update the calibration LUT
        .startFreqConst = 0x558E38E4,         // Start @ 77 GHz (1 LSB = 53.644 Hz)
        .idleTimeConst = 2000,                // Idle = 20 us
        .adcStartTimeConst = 600,             // ADC start time = 6 us
        .rampEndTime = 8000,                  // Ramp end time = 80 us
        .txOutPowerBackoffCode = 0,           // No TX power back-off
        .txPhaseShifter = 0,                  // No TX phase shift
        .freqSlopeConst = 1035,               // Ramp slope @ 49.970 MHz/us (1 LSB = 48.279 kHz/us)
        .txStartTime = 0,                     // TX start time = 0us
        .numAdcSamples = 384,		          // ADC samples (max 1024 in 16KB complex 4-ch)
        .digOutSampleRate = 6000,             // ADC sample rate = 6 Msps (1 LSB = 1 ksps)
        .hpfCornerFreq1 = RL_RX_HPF1_175_KHz, // HPF1 corner freq = 175 kHz
        .hpfCornerFreq2 = RL_RX_HPF2_350_KHz, // HPF2 corner freq = 350 kHz
        .txCalibEnCfg =  0, //0x816D,         // Enable only TX0, TX2 while calibration
        .rxGain = 0xA4,                       // RX Gain = 36 dB, target RF gain = 36 dB
    };

    auto ret = rlSetProfileConfig(device_map, 1, &profile);
    if (ret != 0)
    {
        ESP_LOGE(TAG, "Failed to set the profile: %d.", ret);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Radar profile is configured.");

    rlChirpCfg_t chirp_config =
    {
        .chirpStartIdx = 0,
        .chirpEndIdx = 0,
        .profileId = 0,
        .startFreqVar = 0,
        .idleTimeVar = 0,
        .adcStartTimeVar = 0,
        .txEnable = 0b001,    // Enable only TX 0
    };

    ret = rlSetChirpConfig(device_map, 1, &chirp_config);
    if (ret != 0)
    {
        ESP_LOGE(TAG, "Failed to set the chirp config: %d.", ret);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Chirp is configured.");

    frame_config->chirpStartIdx = 0;
    frame_config->chirpEndIdx = 0;
    frame_config->numLoops = 1;
    frame_config->numFrames = 1;   // Only 1 frame
    frame_config->numAdcSamples = profile.numAdcSamples * 2;
    frame_config->framePeriodicity = 300000;   // Frame period = 1.5 ms (1 LSB = 5 ns)
    frame_config->triggerSelect = 0x0001;      // Use software trigger
    frame_config->frameTriggerDelay = 0;       // No trigger delay

    ret = rlSetFrameConfig(device_map, frame_config);
    if (ret != 0)
    {
        ESP_LOGE(TAG, "Failed to set the frame config: %d.", ret);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Frame is configured.");
    return ESP_OK;
}

static esp_err_t start_radar_capture()
{
    rlFrameTrigger_t trigger =
    {
        .startStop = 1
    };

    ESP_RETURN_ON_ERROR(rlFrameStartStop(device_map, &trigger),
        TAG, "Failed to send trigger to radar");

    return ESP_OK;
}

static esp_err_t save_radar_captured()
{
    // Synchronous the cache
    esp_cache_msync(radar_rx_handle.buffer, radar_rx_handle.size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

    // Show some data from radar
    ESP_LOGI(TAG, "Radar frame has %d bytes.", radar_rx_handle.size);
    ESP_LOG_BUFFER_HEX(TAG, radar_rx_handle.buffer,
        radar_rx_handle.size < 64 ? radar_rx_handle.size : 64);

    // Write to SD card
    const char* file_path = "/sdcard/radar_acq.bin";
    const auto fid = fopen(file_path, "wb+");

    if (fid == nullptr || setvbuf(fid, nullptr, _IONBF, 0) != 0)
    {
        ESP_LOGE(TAG, "Failed to open file %s for writing.", file_path);
        return ESP_FAIL;
    }

    const size_t written = fwrite(radar_rx_handle.buffer, 1, radar_rx_handle.size, fid);
    const bool failed = written != radar_rx_handle.size || ferror(fid);

    if (failed)
    {
        ESP_LOGE(TAG, "Error during writing the radar data file.");
        return ESP_FAIL;
    }

    fflush(fid);
    fclose(fid);

    ESP_LOGI(TAG, "Radar data has been written to %s.", file_path);

    // Free buffer
    heap_caps_free(radar_rx_handle.buffer);
    ESP_LOGI(TAG, "Radar buffer has been free.");

    return ESP_OK;
}

