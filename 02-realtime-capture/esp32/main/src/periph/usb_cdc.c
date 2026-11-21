// + ----------------------------------------------------------------- +
// - File Name    : usb_cdc.c
// - Description  : USB high-speed CDC framing
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

#include "periph/usb_cdc.h"

#include <stdatomic.h>
#include <string.h>

#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <tinyusb.h>
#include <tinyusb_cdc_acm.h>
#include <tinyusb_default_config.h>
#include <tusb.h>

#define CDC_PORT TINYUSB_CDC_ACM_0
#define TX_TIMEOUT_US 500000

static const char* const tag = "usb_cdc";
static uint8_t rx_bytes[sizeof(usb_header_t) + USB_COMMAND_MAX];
static size_t rx_size;
static atomic_bool dtr_asserted;
static atomic_uint session;

static void close_session(void)
{
    if (atomic_exchange(&dtr_asserted, false))
    {
        tud_cdc_n_read_flush(CDC_PORT);
        tud_cdc_n_write_clear(CDC_PORT);
        atomic_fetch_add(&session, 1);
    }
}

static void line_state_changed(int interface, cdcacm_event_t* event)
{
    if (event->line_state_changed_data.dtr)
    {
        atomic_store(&dtr_asserted, true);
    }
    else
    {
        close_session();
    }
}

static void device_event(tinyusb_event_t* event, void* context)
{
    if (event->id == TINYUSB_EVENT_DETACHED)
    {
        close_session();
    }
}

esp_err_t usb_cdc_init(void)
{
    const tinyusb_config_t config = TINYUSB_CONFIG_HIGH_SPEED(device_event, NULL);
    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&config), tag,
                        "USB high-speed initialization failed");

    const tinyusb_config_cdcacm_t cdc = {
        .cdc_port = CDC_PORT,
        .callback_line_state_changed = line_state_changed,
    };
    return tinyusb_cdcacm_init(&cdc);
}

bool usb_cdc_connected(void)
{
    return atomic_load(&dtr_asserted) && tud_cdc_n_connected(CDC_PORT);
}

uint32_t usb_cdc_session(void)
{
    return atomic_load(&session);
}

void usb_cdc_discard(void)
{
    rx_size = 0;
}

/// Disconnect after a partial packet if the host leaves the TX FIFO stalled.
static void restart_cdc(void)
{
    close_session();
    tud_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
    tud_connect();
}

bool usb_cdc_receive(usb_command_t* command)
{
    rx_size += tud_cdc_n_read(CDC_PORT, rx_bytes + rx_size, sizeof(rx_bytes) - rx_size);
    size_t offset = 0;

    while (rx_size - offset >= sizeof(usb_header_t))
    {
        usb_header_t header;
        memcpy(&header, rx_bytes + offset, sizeof(header));
        if (header.magic != USB_MAGIC || header.version != USB_PROTOCOL_VERSION ||
            header.reserved != 0 || header.size > USB_COMMAND_MAX ||
            header.request == 0)
        {
            ++offset;
            continue;
        }

        const size_t packet_size = sizeof(header) + header.size;
        if (rx_size - offset < packet_size)
        {
            break;
        }
        command->header = header;
        memcpy(command->payload, rx_bytes + offset + sizeof(header), header.size);
        offset += packet_size;
        rx_size -= offset;
        memmove(rx_bytes, rx_bytes + offset, rx_size);
        return true;
    }

    rx_size -= offset;
    memmove(rx_bytes, rx_bytes + offset, rx_size);
    return false;
}

static bool write_bytes(const void* data, size_t size, uint32_t current_session)
{
    const uint8_t* bytes = data;
    int64_t last_progress = esp_timer_get_time();
    while (size > 0)
    {
        if (!usb_cdc_connected() || usb_cdc_session() != current_session)
        {
            return false;
        }

        const size_t written = tud_cdc_n_write(CDC_PORT, bytes, size);
        bytes += written;
        size -= written;
        if (written != 0)
        {
            last_progress = esp_timer_get_time();
        }
        else if (esp_timer_get_time() - last_progress >= TX_TIMEOUT_US)
        {
            return false;
        }
        else
        {
            tud_cdc_n_write_flush(CDC_PORT);
            vTaskDelay(1);
        }
    }
    return true;
}

bool usb_cdc_send(uint8_t type, uint32_t request, const void* prefix,
                  size_t prefix_size, const void* data, size_t data_size)
{
    const uint32_t current_session = usb_cdc_session();
    const usb_header_t header = {
        .magic = USB_MAGIC,
        .version = USB_PROTOCOL_VERSION,
        .type = type,
        .size = prefix_size + data_size,
        .request = request,
    };
    if (!write_bytes(&header, sizeof(header), current_session) ||
        !write_bytes(prefix, prefix_size, current_session) ||
        !write_bytes(data, data_size, current_session))
    {
        if (usb_cdc_connected() && usb_cdc_session() == current_session)
        {
            restart_cdc();
        }
        return false;
    }

    tud_cdc_n_write_flush(CDC_PORT);
    return true;
}
