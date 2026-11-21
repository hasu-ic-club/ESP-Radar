// + ----------------------------------------------------------------- +
// - File Name    : capture.c
// - Description  : Radar capture orchestration and USB commands
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

#include "app/capture.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mmwavelink.h>

#include "app/radar.h"
#include "app/range_fft.h"
#include "periph/radar_csi.h"
#include "periph/usb_cdc.h"

#define FINAL_DMA_GRACE_US 20000
#define CSI_WATCHDOG_MARGIN_US 500000

static const char* const tag = "capture";
static usb_command_t command;
static usb_config_t active_config;
static usb_chirp_t chirp_groups[RADAR_CHIRP_MAX];
static _Alignas(16) float fft_output[RADAR_RX_COUNT * RADAR_ADC_MAX];

/// All control state belongs to app_main. Only CSI DMA and USB FIFOs use other tasks.
static struct
{
    radar_raw_frame_t raw;
    uint32_t request;
    uint32_t session;
    uint32_t chirps;
    uint32_t next_chirp;
    uint32_t frames_seen;
    int64_t last_frame_us;
    int64_t final_dma_us;
    int64_t stop_deadline_us;
    bool active;
    bool dma_stopped;
    bool user_stop;
} capture;

static const char* radar_stage;
static int32_t radar_error;
static esp_err_t csi_error;
static esp_err_t fft_error;
static bool configured;
static uint32_t command_session;

static uint32_t frame_chirps(const usb_config_t* config)
{
    return (config->frame_end - config->frame_start + 1U) * config->loops;
}

static uint32_t frame_period_us(void)
{
    return active_config.frame_period / 200;
}

static void initialize_radar(void)
{
    radar_error = radar_init(&radar_stage);
    if (radar_error != 0)
    {
        ESP_LOGE(tag, "%s failed with code %" PRId32, radar_stage, radar_error);
    }
}

static void reply(int32_t code, const char* text)
{
    if (code != 0)
    {
        ESP_LOGW(tag, "Command %u failed at %s with code %" PRId32, command.header.type,
                 text, code);
    }
    if (command_session != usb_cdc_session())
    {
        return;
    }
    const usb_reply_t message = {
        .code = code,
        .state = capture.active ? CAPTURE_RUNNING : CAPTURE_IDLE,
        .flags = (radar_error == 0 ? STATUS_RADAR_READY : 0) |
                 (configured ? STATUS_CONFIGURED : 0),
    };
    usb_cdc_send(USB_REPLY, command.header.request, &message, sizeof(message), text,
                 strlen(text));
}

static bool valid_config(const usb_config_t* config, size_t size)
{
    if (config->chirp_groups == 0 || config->chirp_groups > RADAR_CHIRP_MAX ||
        size != sizeof(*config) + config->chirp_groups * sizeof(usb_chirp_t) ||
        config->frame_start > config->frame_end ||
        config->frame_end >= RADAR_CHIRP_MAX || config->loops == 0 ||
        config->loops > 255 || config->samples < 2 || config->samples > RADAR_ADC_MAX ||
        config->frame_adc_words != config->samples * RADAR_COMPLEX_WORDS ||
        config->start_freq < 0x5471c71b || config->start_freq > 0x5a000000 ||
        config->idle_time > 524287 || config->adc_start > 4095 ||
        config->ramp_end > 500000 || config->slope < -5510 || config->slope > 5510 ||
        config->slope == 0 || config->tx_start < -4096 || config->tx_start > 4095 ||
        config->sample_rate < 2000 || config->sample_rate > 45000 ||
        config->frame_period < 60000 || config->frame_period > 268400000)
    {
        return false;
    }

    const uint32_t chirps = frame_chirps(config);
    if (chirps > RADAR_FRAME_CHIRPS_MAX ||
        chirps * config->samples * RADAR_ADC_BYTES_PER_SAMPLE > RADAR_FRAME_BYTES_MAX)
    {
        return false;
    }

    bool assigned[RADAR_CHIRP_MAX] = {false};
    for (unsigned i = 0; i < config->chirp_groups; ++i)
    {
        const usb_chirp_t* chirp = &chirp_groups[i];
        if (chirp->start > chirp->end || chirp->end >= RADAR_CHIRP_MAX ||
            (chirp->tx_mask & ~0x05) != 0)
        {
            return false;
        }
        for (unsigned j = chirp->start; j <= chirp->end; ++j)
        {
            if (assigned[j])
            {
                return false;
            }
            assigned[j] = true;
        }
    }
    for (unsigned i = config->frame_start; i <= config->frame_end; ++i)
    {
        if (!assigned[i])
        {
            return false;
        }
    }
    return true;
}

static void configure(void)
{
    if (capture.active)
    {
        reply(CAPTURE_ERR_STATE, "Stop the current capture before configuring");
        return;
    }
    if (command.header.size < sizeof(usb_config_t))
    {
        reply(CAPTURE_ERR_PROTOCOL, "Invalid configuration length");
        return;
    }

    usb_config_t config;
    memcpy(&config, command.payload, sizeof(config));
    memcpy(chirp_groups, command.payload + sizeof(config),
           command.header.size - sizeof(config));
    if (!valid_config(&config, command.header.size))
    {
        reply(CAPTURE_ERR_PROTOCOL, "Invalid radar configuration");
        return;
    }

    // A failed radar command may have partially changed its settings.
    configured = false;
    if (radar_error != 0 || radar_faulted())
    {
        initialize_radar();
        if (radar_error != 0)
        {
            reply(radar_error, radar_stage);
            return;
        }
    }

    const char* stage;
    const int32_t result = radar_configure(&config, chirp_groups, &stage);
    if (result == 0)
    {
        active_config = config;
        configured = true;
    }
    reply(result, result == 0 ? "Radar configuration applied" : stage);
}

static void reset_radar(int32_t code, const char* stage)
{
    radar_power_down();
    radar_error = code;
    radar_stage = stage;
    configured = false;
}

static void start_capture(void)
{
    if (capture.active || !configured || radar_error != 0)
    {
        reply(CAPTURE_ERR_STATE, "Apply a radar configuration before starting");
        return;
    }
    if (csi_error != ESP_OK || fft_error != ESP_OK)
    {
        reply(csi_error != ESP_OK ? csi_error : fft_error,
              csi_error != ESP_OK ? "Initialize CSI power" : "Initialize Range FFT");
        return;
    }

    const uint32_t chirps = frame_chirps(&active_config);
    const int32_t csi_result =
        radar_csi_start(active_config.samples * RADAR_ADC_BYTES_PER_SAMPLE, chirps);
    if (csi_result != 0)
    {
        reply(csi_result, "Start CSI DMA and PSRAM ring");
        return;
    }

    radar_clear_events();
    const int32_t result = radar_trigger(true);
    if (result != 0)
    {
        radar_csi_release();
        if (result < 0 || result == RL_RET_CODE_FRAME_ALREADY_STARTED)
        {
            reset_radar(result, "Start radar frame sampling");
        }
        reply(result, "Start radar frame sampling");
        return;
    }

    memset(&capture, 0, sizeof(capture));
    capture.active = true;
    capture.request = command.header.request;
    capture.session = command_session;
    capture.chirps = chirps;
    capture.last_frame_us = esp_timer_get_time();
    ESP_LOGI(tag, "Capture started");
    reply(0, "Capture started");
}

static void stop_capture(void)
{
    if (!capture.active || capture.user_stop || radar_frame_ended())
    {
        reply(0, "Capture is stopping or already idle");
        return;
    }

    const int32_t result = radar_trigger(false);
    if (result == 0)
    {
        capture.user_stop = true;
        capture.stop_deadline_us =
            esp_timer_get_time() + frame_period_us() + CSI_WATCHDOG_MARGIN_US;
    }
    reply(result, result == 0 ? "Stop requested" : "Stop radar frame sampling");
}

static void handle_command(void)
{
    command_session = usb_cdc_session();
    if (command.header.type != USB_CONFIGURE && command.header.size != 0)
    {
        reply(CAPTURE_ERR_PROTOCOL, "Unexpected command payload");
        return;
    }

    switch (command.header.type)
    {
    case USB_GET_STATUS: {
        char version[160];
        radar_version(version, sizeof(version));
        if (radar_error != 0)
        {
            const size_t used = strlen(version);
            snprintf(version + used, sizeof(version) - used, " | %s", radar_stage);
        }
        reply(radar_error, version);
        break;
    }
    case USB_CONFIGURE:
        configure();
        break;
    case USB_START:
        start_capture();
        break;
    case USB_STOP:
        stop_capture();
        break;
    default:
        reply(CAPTURE_ERR_PROTOCOL, "Unknown command");
        break;
    }
}

static void finish_capture(int32_t code, const char* text)
{
    const esp_err_t stop_result = radar_csi_stop();
    if (code == 0 && stop_result != ESP_OK)
    {
        code = CAPTURE_ERR_CSI;
        text = "Stop CSI DMA";
    }
    const usb_done_t done = {
        .code = code,
        .frames = radar_csi_frames(),
        .dropped = radar_csi_dropped(),
    };
    radar_csi_release();
    capture.active = false;

    if (usb_cdc_connected() && usb_cdc_session() == capture.session)
    {
        usb_cdc_send(USB_CAPTURE_DONE, capture.request, &done, sizeof(done), text,
                     strlen(text));
    }
    ESP_LOGI(tag, "%s (code %" PRId32 ", %" PRIu32 " CSI frames, %" PRIu32 " dropped)",
             text, code, done.frames, done.dropped);
}

static void abort_capture(int32_t code, const char* text)
{
    if (radar_faulted())
    {
        reset_radar(CAPTURE_ERR_RADAR_FAULT, "Radar reported a CPU fault");
    }
    else if (!radar_frame_ended())
    {
        const int32_t result = radar_trigger(false);
        const int64_t deadline =
            esp_timer_get_time() + frame_period_us() + CSI_WATCHDOG_MARGIN_US;
        while (result == 0 && !radar_frame_ended() && !radar_faulted() &&
               esp_timer_get_time() < deadline)
        {
            vTaskDelay(1);
        }
        if (result != 0 || !radar_frame_ended() || radar_faulted())
        {
            reset_radar(result != 0 ? result : code, "Reset radar after capture error");
        }
    }
    finish_capture(code, text);
}

/// Process at most one chirp before checking the USB control queue again.
static bool service_capture(void)
{
    if (!usb_cdc_connected() || usb_cdc_session() != capture.session)
    {
        abort_capture(CAPTURE_ERR_USB, "USB disconnected during capture");
        return true;
    }
    if (radar_faulted())
    {
        abort_capture(CAPTURE_ERR_RADAR_FAULT, "Radar reported a CPU fault");
        return true;
    }

    if (radar_csi_failed())
    {
        abort_capture(CAPTURE_ERR_CSI, "CSI DMA or cache synchronization failed");
        return true;
    }

    const int64_t now = esp_timer_get_time();
    const uint32_t frames = radar_csi_frames();
    if (capture.frames_seen != frames)
    {
        capture.frames_seen = frames;
        capture.last_frame_us = now;
    }

    if (radar_frame_ended() && capture.final_dma_us == 0)
    {
        capture.final_dma_us = now + FINAL_DMA_GRACE_US;
    }

    if (capture.final_dma_us != 0 && now >= capture.final_dma_us &&
        !capture.dma_stopped)
    {
        if (radar_csi_stop() != ESP_OK)
        {
            abort_capture(CAPTURE_ERR_CSI, "Unable to stop CSI DMA");
            return true;
        }
        capture.dma_stopped = true;
    }

    if (capture.final_dma_us == 0 &&
        (now - capture.last_frame_us > 2 * frame_period_us() + CSI_WATCHDOG_MARGIN_US ||
         (capture.stop_deadline_us != 0 && now >= capture.stop_deadline_us)))
    {
        abort_capture(CAPTURE_ERR_TIMEOUT,
                      "Timed out waiting for radar frames or frame end");
        return true;
    }

    if (!capture.raw.data && !radar_csi_take(&capture.raw))
    {
        if (capture.dma_stopped)
        {
            const bool missing = radar_csi_dropped() != 0 ||
                                 (!capture.user_stop && active_config.frames != 0 &&
                                  frames != active_config.frames);
            finish_capture(missing ? CAPTURE_ERR_OVERRUN : 0,
                           missing             ? "Capture ended with missing frames"
                           : capture.user_stop ? "Capture stopped"
                                               : "Capture complete");
            return true;
        }
        return false;
    }

    const size_t words_per_chirp =
        active_config.samples * RADAR_RX_COUNT * RADAR_COMPLEX_WORDS;
    const int16_t* samples = capture.raw.data + capture.next_chirp * words_per_chirp;
    const int32_t result = range_fft_chirp(samples, active_config.samples,
                                           active_config.slope < 0, fft_output);
    if (result != 0)
    {
        abort_capture(result, "Range FFT failed");
        return true;
    }
    const usb_fft_t metadata = {
        .frame_index = capture.raw.index,
        .chirp_index = capture.next_chirp,
        .bins = range_fft_bins(active_config.samples),
        .channels = RADAR_RX_COUNT,
    };
    const size_t bytes =
        metadata.bins * metadata.channels * RADAR_COMPLEX_WORDS * sizeof(fft_output[0]);
    if (!usb_cdc_send(USB_RANGE_FFT, capture.request, &metadata, sizeof(metadata),
                      fft_output, bytes))
    {
        abort_capture(CAPTURE_ERR_USB, "USB transmission failed during capture");
        return true;
    }

    if (++capture.next_chirp == capture.chirps)
    {
        radar_csi_return(&capture.raw);
        capture.raw.data = NULL;
        capture.next_chirp = 0;
    }
    return true;
}

void capture_run(void)
{
    const esp_err_t usb_error = usb_cdc_init();
    if (usb_error != ESP_OK)
    {
        ESP_LOGE(tag, "USB startup failed: %s", esp_err_to_name(usb_error));
        return;
    }

    csi_error = radar_csi_init();
    fft_error = range_fft_init();
    if (csi_error != ESP_OK || fft_error != ESP_OK)
    {
        ESP_LOGE(tag, "Capture peripheral startup failed (CSI: %s, FFT: %s)",
                 esp_err_to_name(csi_error), esp_err_to_name(fft_error));
    }
    initialize_radar();

    uint32_t session = usb_cdc_session();
    unsigned uninterrupted = 0;
    for (;;)
    {
        const uint32_t current_session = usb_cdc_session();
        if (current_session != session)
        {
            usb_cdc_discard();
            session = current_session;
        }
        if (usb_cdc_connected() && usb_cdc_receive(&command))
        {
            ESP_LOGI(tag, "USB control request %u", command.header.type);
            handle_command();
        }

        const bool busy = capture.active && service_capture();
        if (!busy || ++uninterrupted >= 32)
        {
            uninterrupted = 0;
            vTaskDelay(1);
        }
    }
}
