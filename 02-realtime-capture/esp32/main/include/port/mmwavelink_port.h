// + ----------------------------------------------------------------- +
// - File Name    : mmwavelink_port.h
// - Description  : Hardware porting implementations for mmWaveLink
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

#ifndef MMWAVELINK_PORT_H
#define MMWAVELINK_PORT_H

#include <driver/spi_master.h>

#include <freertos/FreeRTOS.h>

#include <mmwavelink_esp.h>

#define XWR_SPI_CS_DELAY_US                                                            \
    ((CONFIG_XWR_SPI_CS_DELAY_ORIGINAL + CONFIG_XWR_SPI_CLOCK_HZ - 1U) /               \
     CONFIG_XWR_SPI_CLOCK_HZ)

#define XWR_ASYNC_EVENT_RADAR_PWUP_DONE BIT0
#define XWR_ASYNC_EVENT_RF_PWUP_DONE BIT1
#define XWR_ASYNC_EVENT_CALIB_DONE BIT2
#define XWR_ASYNC_EVENT_FRAME_END BIT3
#define XWR_ASYNC_EVENT_FAULT BIT4

esp_err_t init_radar_peripherals(mmwl_esp_context_t* mmwl_handle);
void link_mmwl_handle(mmwl_esp_context_t* mmwl_handle,
                      EventGroupHandle_t* async_event_grp);

#endif // MMWAVELINK_PORT_H
