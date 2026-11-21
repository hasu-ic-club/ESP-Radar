// + ----------------------------------------------------------------- +
// - File Name    : protocol.h
// - Description  : USB protocol and capture geometry
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

#include <stdint.h>

#define USB_MAGIC UINT32_C(0x54464652)
#define USB_PROTOCOL_VERSION 1
#define RADAR_RX_COUNT 4
#define RADAR_ADC_MAX 1024
#define RADAR_CHIRP_MAX 512
#define RADAR_FRAME_BYTES_MAX (2U * 1024U * 1024U)
#define RADAR_FRAME_CHIRPS_MAX 4095
#define RADAR_COMPLEX_WORDS 2
#define RADAR_ADC_BYTES_PER_SAMPLE (RADAR_RX_COUNT * RADAR_COMPLEX_WORDS * 2)

typedef enum
{
    USB_GET_STATUS = 1,
    USB_CONFIGURE = 2,
    USB_START = 3,
    USB_STOP = 4,
    USB_REPLY = 0x80,
    USB_RANGE_FFT = 0x90,
    USB_CAPTURE_DONE = 0x91,
} usb_message_type_t;

typedef enum
{
    CAPTURE_IDLE = 1,
    CAPTURE_RUNNING = 2,
} capture_state_t;

enum
{
    STATUS_RADAR_READY = 1,
    STATUS_CONFIGURED = 2,
    CAPTURE_ERR_PROTOCOL = -10000,
    CAPTURE_ERR_STATE = -10001,
    CAPTURE_ERR_CSI = -10002,
    CAPTURE_ERR_OVERRUN = -10003,
    CAPTURE_ERR_TIMEOUT = -10004,
    CAPTURE_ERR_USB = -10005,
    CAPTURE_ERR_RADAR_FAULT = -10006,
};

/// Little-endian packet header shared by control replies and FFT data.
typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t reserved;
    uint32_t size;
    uint32_t request;
} usb_header_t;

/// Fields set by the host in the radar API's native units.
typedef struct __attribute__((packed))
{
    uint32_t start_freq;
    uint32_t idle_time;
    uint32_t adc_start;
    uint32_t ramp_end;
    int16_t slope;
    int16_t tx_start;
    uint16_t samples;
    uint16_t sample_rate;
    uint16_t frame_start;
    uint16_t frame_end;
    uint16_t loops;
    uint16_t frames;
    uint32_t frame_period;
    uint16_t frame_adc_words;
    uint16_t chirp_groups;
} usb_config_t;

typedef struct __attribute__((packed))
{
    uint16_t start;
    uint16_t end;
    uint16_t tx_mask;
} usb_chirp_t;

typedef struct __attribute__((packed))
{
    int32_t code;
    uint8_t state;
    uint8_t flags;
} usb_reply_t;

typedef struct
{
    uint32_t frame_index;
    uint32_t chirp_index;
    uint16_t bins;
    uint16_t channels;
} usb_fft_t;

typedef struct
{
    int32_t code;
    uint32_t frames;
    uint32_t dropped;
} usb_done_t;

_Static_assert(sizeof(usb_header_t) == 16, "Invalid USB header size");
_Static_assert(sizeof(usb_config_t) == 40, "Invalid USB configuration size");
_Static_assert(sizeof(usb_chirp_t) == 6, "Invalid USB chirp size");
_Static_assert(sizeof(usb_reply_t) == 6, "Invalid USB reply size");
_Static_assert(sizeof(usb_fft_t) == 12, "Invalid USB FFT metadata size");
_Static_assert(sizeof(usb_done_t) == 12, "Invalid USB completion size");
