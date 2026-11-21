// + ----------------------------------------------------------------- +
// - File Name    : usb_cdc.h
// - Description  : High-speed USB CDC transport
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

#include "app/protocol.h"

#define USB_COMMAND_MAX (sizeof(usb_config_t) + RADAR_CHIRP_MAX * sizeof(usb_chirp_t))

typedef struct
{
    usb_header_t header;
    uint8_t payload[USB_COMMAND_MAX];
} usb_command_t;

esp_err_t usb_cdc_init(void);
bool usb_cdc_connected(void);
uint32_t usb_cdc_session(void);
void usb_cdc_discard(void);

/// Read one complete command without waiting for USB traffic.
bool usb_cdc_receive(usb_command_t* command);

/// Send a packet with two payload segments and bounded backpressure.
bool usb_cdc_send(uint8_t type, uint32_t request, const void* prefix,
                  size_t prefix_size, const void* data, size_t data_size);
