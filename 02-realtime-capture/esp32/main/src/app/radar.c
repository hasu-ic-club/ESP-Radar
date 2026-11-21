// + ----------------------------------------------------------------- +
// - File Name    : radar.c
// - Description  : AWR2243 RF initialization and control
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

#include "app/radar.h"

#include <inttypes.h>
#include <stdio.h>

#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <mmwavelink.h>

#include "port/mmwavelink_port.h"

static const char* const tag = "radar";
static const uint8_t device_map = RL_DEVICE_MAP_CASCADED_1;
static mmwl_esp_context_t mmwl;
static EventGroupHandle_t events;
static rlFwVersionParam_t firmware;
static bool port_ready;
static bool firmware_known;

static void frame_end_cb(uint8_t device_id, uint8_t* payload, uint16_t length)
{
    xEventGroupSetBits(events, XWR_ASYNC_EVENT_FRAME_END);
}

static void fault_cb(uint8_t device_id, uint8_t* payload, uint16_t length)
{
    xEventGroupSetBits(events, XWR_ASYNC_EVENT_FAULT);
}

static int32_t wait_event(EventBits_t expected)
{
    const EventBits_t event = xEventGroupWaitBits(
        events, expected | XWR_ASYNC_EVENT_FAULT, pdTRUE, pdFALSE, pdMS_TO_TICKS(1500));
    if (event & XWR_ASYNC_EVENT_FAULT)
    {
        return CAPTURE_ERR_RADAR_FAULT;
    }

    return (event & expected) ? 0 : CAPTURE_ERR_TIMEOUT;
}

/// Use the same RF, ADC and CSI settings as the initialization demo.
int32_t radar_init(const char** stage)
{
    if (!port_ready)
    {
        *stage = "Initialize radar SPI and GPIO";
        if (!events)
        {
            events = xEventGroupCreate();
        }
        if (!events)
        {
            return ESP_ERR_NO_MEM;
        }

        const esp_err_t result = init_radar_peripherals(&mmwl);
        if (result != ESP_OK)
        {
            return result;
        }

        link_mmwl_handle(&mmwl, &events);
        mmwl.rf_ae_event_cb.frame_end = frame_end_cb;
        mmwl.dev_event_cb.cpu_fault = fault_cb;
        port_ready = true;
    }

    firmware_known = false;
    xEventGroupClearBits(events, XWR_ASYNC_EVENT_RADAR_PWUP_DONE |
                                     XWR_ASYNC_EVENT_RF_PWUP_DONE |
                                     XWR_ASYNC_EVENT_CALIB_DONE |
                                     XWR_ASYNC_EVENT_FRAME_END | XWR_ASYNC_EVENT_FAULT);

    *stage = "Power on radar";
    int32_t result = rlDevicePowerOn(device_map, mmwl.client_context);
    if (result == 0)
    {
        result = wait_event(XWR_ASYNC_EVENT_RADAR_PWUP_DONE);
    }
    if (result != 0)
    {
        return result;
    }

    *stage = "Read radar firmware version";
    result = rlDeviceGetMssVersion(device_map, &firmware);
    if (result != 0)
    {
        return result;
    }
    firmware_known = true;

    *stage = "Power on radar RF subsystem";
    result = rlDeviceRfStart(device_map);
    if (result == 0)
    {
        result = wait_event(XWR_ASYNC_EVENT_RF_PWUP_DONE);
    }
    if (result != 0)
    {
        return result;
    }

    mmwl_esp_config_t config = {
        .cascade_role = MMWL_ESP_CASCADE_ROLE_SINGLE,
        .crc_type = RL_CRC_TYPE_32BIT,
        .rx_channel_enabled = 0x0f,
        .tx_channel_enabled = 0x05,
        .adc_bits = RL_ADC_DATA_16_BIT,
        .adc_format = RL_ADC_FORMAT_COMPLEX_2X,
        .adc_iq_swap = 0,
        .packet0_type = MMWL_ESP_CSI_PACKET_0_ADC_ONLY,
        .packet0_virtual_channel = MMWL_ESP_CSI_VIRTUAL_CHANNEL(0),
        .packet1_type = MMWL_ESP_CSI_PACKET_1_NONE,
        .packet1_virtual_channel = MMWL_ESP_CSI_VIRTUAL_CHANNEL(1),
        .cq_size = {128, 128, 128},
        .data_rate = MMWL_ESP_CSI_DATA_RATE_600MBPS,
        .csi_lane_setting = 0x30021,
    };

    *stage = "Configure radar RF subsystem";
    result = mmwl_esp_pre_rf_init(device_map, &config);
    if (result != 0)
    {
        return result;
    }
    *stage = "Initialize and calibrate radar RF";
    result = rlRfInit(device_map);
    if (result == 0)
    {
        result = wait_event(XWR_ASYNC_EVENT_CALIB_DONE);
    }
    if (result != 0)
    {
        return result;
    }
    *stage = "Configure radar CSI output";
    result = mmwl_esp_post_rf_init(device_map, &config);
    if (result == 0)
    {
        char version[80];
        radar_version(version, sizeof(version));
        ESP_LOGI(tag, "%s", version);
    }

    return result;
}

int32_t radar_configure(const usb_config_t* config, const usb_chirp_t* chirps,
                        const char** stage)
{
    rlProfileCfg_t profile = {
        .profileId = 0,
        .pfVcoSelect = 0,
        .pfCalLutUpdate = 0,
        .startFreqConst = config->start_freq,
        .idleTimeConst = config->idle_time,
        .adcStartTimeConst = config->adc_start,
        .rampEndTime = config->ramp_end,
        .txOutPowerBackoffCode = 0,
        .txPhaseShifter = 0,
        .freqSlopeConst = config->slope,
        .txStartTime = config->tx_start,
        .numAdcSamples = config->samples,
        .digOutSampleRate = config->sample_rate,
        .hpfCornerFreq1 = RL_RX_HPF1_175_KHz,
        .hpfCornerFreq2 = RL_RX_HPF2_350_KHz,
        .txCalibEnCfg = 0,
        .rxGain = 0xa4,
    };
    *stage = "Set Profile 0";
    int32_t result = rlSetProfileConfig(device_map, 1, &profile);
    if (result != 0)
    {
        return result;
    }

    *stage = "Set chirp group";
    for (uint16_t index = 0; index < config->chirp_groups; ++index)
    {
        rlChirpCfg_t chirp = {
            .chirpStartIdx = chirps[index].start,
            .chirpEndIdx = chirps[index].end,
            .profileId = 0,
            .txEnable = chirps[index].tx_mask,
        };

        result = rlSetChirpConfig(device_map, 1, &chirp);
        if (result != 0)
        {
            ESP_LOGE(tag, "Chirp group %u (%u to %u) failed with code %" PRId32,
                     index + 1, chirp.chirpStartIdx, chirp.chirpEndIdx, result);
            return result;
        }
    }

    *stage = "Set frame";
    rlFrameCfg_t frame = {
        .chirpStartIdx = config->frame_start,
        .chirpEndIdx = config->frame_end,
        .numLoops = config->loops,
        .numFrames = config->frames,
        .numAdcSamples = config->frame_adc_words,
        .framePeriodicity = config->frame_period,
        .triggerSelect = 1,
        .frameTriggerDelay = 0,
    };
    return rlSetFrameConfig(device_map, &frame);
}

int32_t radar_trigger(bool start)
{
    rlFrameTrigger_t trigger = {
        .startStop = start ? 1 : 0,
    };
    const int32_t result = rlFrameStartStop(device_map, &trigger);
    if (!start && result == RL_RET_CODE_FRAME_ALREADY_ENDED)
    {
        xEventGroupSetBits(events, XWR_ASYNC_EVENT_FRAME_END);
        return 0;
    }
    return result;
}

int32_t radar_power_down(void)
{
    return rlDevicePowerOff();
}

void radar_clear_events(void)
{
    xEventGroupClearBits(events, XWR_ASYNC_EVENT_FRAME_END);
}

bool radar_frame_ended(void)
{
    return events && (xEventGroupGetBits(events) & XWR_ASYNC_EVENT_FRAME_END);
}

bool radar_faulted(void)
{
    return events && (xEventGroupGetBits(events) & XWR_ASYNC_EVENT_FAULT);
}

void radar_version(char* text, size_t size)
{
    if (!firmware_known)
    {
        snprintf(text, size, "Firmware version unavailable");
        return;
    }

    snprintf(text, size, "MSS %u.%u.%u.%u (%u/%02u/%02u) | Hardware %u.%u.%u",
             firmware.fwMajor, firmware.fwMinor, firmware.fwBuild, firmware.fwDebug,
             firmware.fwYear, firmware.fwMonth, firmware.fwDay, firmware.hwMajor,
             firmware.hwMinor, firmware.hwVarient);
}
