// + ----------------------------------------------------------------- +
// - File Name    : sdcard.c
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

#include <driver/sdmmc_host.h>
#include <driver/sdmmc_default_configs.h>
#include <sdmmc_cmd.h>
#include <esp_vfs_fat.h>

#if CONFIG_SDCARD_PWR_CTRL_LDO_INTERNAL
#include <sd_pwr_ctrl_by_on_chip_ldo.h>
#endif // CONFIG_SDCARD_PWR_CTRL_LDO_INTERNAL

#include "periph/sdcard.h"

static const char *TAG = "SD";

esp_err_t init_sdcard(sdmmc_card_t** sdcard_handle)
{
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = CONFIG_SDCARD_SLOT;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

#if CONFIG_SDCARD_PWR_CTRL_LDO_INTERNAL
    const sd_pwr_ctrl_ldo_config_t ldo_cfg = {
        .ldo_chan_id = CONFIG_SDCARD_PWR_CTRL_LDO_IO_ID
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = nullptr;
    esp_err_t err = sd_pwr_ctrl_new_on_chip_ldo(&ldo_cfg, &pwr_ctrl_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize LDO controller (%d).", err);
        return err;
    }

    host.pwr_ctrl_handle = pwr_ctrl_handle;
#endif // CONFIG_SDCARD_PWR_CTRL_LDO_INTERNAL

    sdmmc_slot_config_t slot_cfg = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_cfg.clk = CONFIG_SDCARD_SLOT_CLK_GPIO;
    slot_cfg.cmd = CONFIG_SDCARD_SLOT_CMD_GPIO;
    slot_cfg.d0 = CONFIG_SDCARD_SLOT_D0_GPIO;
    slot_cfg.d1 = CONFIG_SDCARD_SLOT_D1_GPIO;
    slot_cfg.d2 = CONFIG_SDCARD_SLOT_D2_GPIO;
    slot_cfg.d3 = CONFIG_SDCARD_SLOT_D3_GPIO;
    slot_cfg.width = 4;
    slot_cfg.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_vfs_fat_mount_config_t mount_cfg = VFS_FAT_MOUNT_DEFAULT_CONFIG();
    mount_cfg.format_if_mount_failed = false;
    mount_cfg.max_files = 2;

    err = esp_vfs_fat_sdmmc_mount("/sdcard", &host,
        &slot_cfg, &mount_cfg, sdcard_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount SD card (%d).", err);
        return err;
    }

    ESP_LOGI(TAG, "SD card mounted.");
    sdmmc_card_print_info(stdout, *sdcard_handle);
    return ESP_OK;
}
