// + ----------------------------------------------------------------- +
// - File Name    : radar_csi.c
// - Description  : CSI DMA and PSRAM capture ring
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

#include "periph/radar_csi.h"

#include <stdatomic.h>

#include <driver/isp_core.h>
#include <esp_cache.h>
#include <esp_check.h>
#include <esp_heap_caps.h>
#include <esp_ldo_regulator.h>
#include <esp_log.h>
#include <esp_private/dw_gdma.h>
#include <esp_private/esp_clk_tree_common.h>
#include <esp_private/mipi_csi_share_hw_ctrl.h>
#include <esp_private/periph_ctrl.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <hal/isp_ll.h>
#include <hal/mipi_csi_hal.h>
#include <hal/mipi_csi_ll.h>

#define RING_SLOTS 4
#define CSI_DMA_BURST_WORDS 512

static const char* const tag = "radar_csi";
static mipi_csi_hal_context_t csi;
static isp_proc_handle_t isp;
static esp_ldo_channel_handle_t ldo;
static dw_gdma_channel_handle_t dma;
static QueueHandle_t free_slots;
static QueueHandle_t full_slots;
static int16_t* buffers[RING_SLOTS];
static int16_t* discard_buffer;
static int16_t* dma_buffer;
static size_t dma_chunk_bytes;
static size_t dma_frame_words;
static atomic_bool running;
static atomic_bool dma_failed;
static atomic_uint frame_count;
static atomic_uint drop_count;

static esp_err_t arm_dma(int16_t* destination)
{
    const dw_gdma_block_transfer_config_t transfer = {
        .src =
            {
                .addr = MIPI_CSI_BRG_MEM_BASE,
                .burst_mode = DW_GDMA_BURST_MODE_FIXED,
                .burst_items = DW_GDMA_BURST_ITEMS_512,
                .burst_len = 16,
                .width = DW_GDMA_TRANS_WIDTH_64,
            },
        .dst =
            {
                .addr = (uint32_t)destination,
                .burst_mode = DW_GDMA_BURST_MODE_INCREMENT,
                .burst_items = DW_GDMA_BURST_ITEMS_512,
                .burst_len = 16,
                .width = DW_GDMA_TRANS_WIDTH_64,
            },
        .size = dma_frame_words,
    };
    const esp_err_t result = dw_gdma_channel_config_transfer(dma, &transfer);
    if (result != ESP_OK)
    {
        return result;
    }
    dma_buffer = destination;
    return dw_gdma_channel_enable_ctrl(dma, true);
}

/// Rearm DMA before publishing a completed frame to the CPU.
static bool IRAM_ATTR frame_ready(dw_gdma_channel_handle_t channel,
                                  const dw_gdma_trans_done_event_data_t* event,
                                  void* context)
{
    if (!atomic_load_explicit(&running, memory_order_relaxed))
    {
        return false;
    }

    int16_t* completed = dma_buffer;
    int16_t* next;
    if (xQueueReceiveFromISR(free_slots, &next, NULL) != pdTRUE)
    {
        next = discard_buffer;
    }
    if (arm_dma(next) != ESP_OK)
    {
        atomic_store_explicit(&dma_failed, true, memory_order_relaxed);
    }

    const uint32_t index =
        atomic_fetch_add_explicit(&frame_count, 1, memory_order_relaxed);
    if (completed == discard_buffer)
    {
        atomic_fetch_add_explicit(&drop_count, 1, memory_order_relaxed);
        return false;
    }
    const radar_raw_frame_t frame = {
        .data = completed,
        .index = index,
    };
    BaseType_t wake = pdFALSE;
    if (xQueueSendFromISR(full_slots, &frame, &wake) != pdTRUE)
    {
        atomic_fetch_add_explicit(&drop_count, 1, memory_order_relaxed);
        xQueueSendFromISR(free_slots, &completed, &wake);
    }
    return wake == pdTRUE;
}

esp_err_t radar_csi_init(void)
{
    const esp_ldo_channel_config_t ldo_config = {
        .chan_id = 3,
        .voltage_mv = 2500,
    };
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_config, &ldo), tag,
                        "Unable to power CSI PHY");

    // Keep the receive PHY up between captures because the radar can keep its clock
    // lane in high-speed mode after frame end.
    int bridge_id;
    ESP_RETURN_ON_ERROR(mipi_csi_brg_claim(MIPI_CSI_BRG_USER_CSI, &bridge_id), tag,
                        "Unable to reserve CSI bridge");
    const mipi_csi_phy_clock_source_t clock = MIPI_CSI_PHY_CLK_SRC_PLL_F25M;
    ESP_RETURN_ON_ERROR(esp_clk_tree_enable_src(clock, true), tag,
                        "Unable to enable CSI reference clock");
    PERIPH_RCC_ATOMIC()
    {
        mipi_csi_ll_enable_host_bus_clock(0, true);
        mipi_csi_ll_reset_host_clock(0);
        mipi_csi_ll_set_phy_clock_source(0, clock);
        mipi_csi_ll_enable_phy_config_clock(0, true);
    }

    const mipi_csi_hal_config_t receiver = {
        .frame_height = 1,
        .frame_width = 1,
        .lane_bit_rate_mbps = 600,
        .lanes_num = 2,
    };
    mipi_csi_hal_init(&csi, &receiver);
    mipi_csi_brg_ll_set_burst_len(csi.bridge_dev, CSI_DMA_BURST_WORDS);
    mipi_csi_brg_ll_set_data_type_min(csi.bridge_dev, 0x12);
    mipi_csi_brg_ll_set_data_type_max(csi.bridge_dev, 0x2f);
    mipi_csi_brg_ll_enable_color_conversion(csi.bridge_dev, true);
    mipi_csi_brg_ll_set_color_mode_bypass(csi.bridge_dev, true);

    // Configure the MIPI input path in ISP bypass mode as in the demo.
    const esp_isp_processor_cfg_t isp_config = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RAW8,
        .has_line_start_packet = true,
        .has_line_end_packet = true,
        .h_res = 4,
        .v_res = 1,
        .flags.bypass_isp = true,
    };
    ESP_RETURN_ON_ERROR(esp_isp_new_processor(&isp_config, &isp), tag,
                        "Unable to configure ISP bypass");

    const dw_gdma_channel_alloc_config_t dma_config = {
        .src =
            {
                .block_transfer_type = DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS,
                .role = DW_GDMA_ROLE_PERIPH_CSI,
                .handshake_type = DW_GDMA_HANDSHAKE_HW,
                .num_outstanding_requests = 5,
                .status_fetch_addr = MIPI_CSI_BRG_MEM_BASE,
            },
        .dst =
            {
                .block_transfer_type = DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS,
                .role = DW_GDMA_ROLE_MEM,
                .handshake_type = DW_GDMA_HANDSHAKE_HW,
                .num_outstanding_requests = 5,
            },
        .flow_controller = DW_GDMA_FLOW_CTRL_SRC,
        .chan_priority = 1,
    };
    ESP_RETURN_ON_ERROR(dw_gdma_new_channel(&dma_config, &dma), tag,
                        "Unable to reserve CSI DMA channel");
    dw_gdma_event_callbacks_t callbacks = {
        .on_full_trans_done = frame_ready,
    };
    return dw_gdma_channel_register_event_callbacks(dma, &callbacks, NULL);
}

static int16_t* allocate_buffer(size_t alignment, uint32_t caps)
{
    int16_t* buffer = heap_caps_aligned_alloc(alignment, dma_chunk_bytes, caps);
    if (buffer && esp_cache_msync(buffer, dma_chunk_bytes,
                                  ESP_CACHE_MSYNC_FLAG_DIR_M2C) != ESP_OK)
    {
        heap_caps_free(buffer);
        return NULL;
    }
    return buffer;
}

static esp_err_t allocate_buffers(size_t frame_size)
{
    const size_t alignment = CONFIG_CACHE_L2_CACHE_LINE_SIZE;
    dma_chunk_bytes = (frame_size + alignment - 1) & ~(alignment - 1);
    const uint32_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
    free_slots = xQueueCreate(RING_SLOTS, sizeof(buffers[0]));
    full_slots = xQueueCreate(RING_SLOTS, sizeof(radar_raw_frame_t));
    discard_buffer = allocate_buffer(alignment, caps);
    if (!free_slots || !full_slots || !discard_buffer)
    {
        return ESP_ERR_NO_MEM;
    }

    for (unsigned i = 0; i < RING_SLOTS; ++i)
    {
        buffers[i] = allocate_buffer(alignment, caps);
        if (!buffers[i])
        {
            return ESP_ERR_NO_MEM;
        }
        xQueueSend(free_slots, &buffers[i], 0);
    }
    return ESP_OK;
}

esp_err_t radar_csi_start(uint32_t bytes_per_chirp, uint32_t chirps_per_frame)
{
    if (!dma || atomic_load_explicit(&running, memory_order_relaxed))
    {
        return ESP_ERR_INVALID_STATE;
    }

    atomic_store_explicit(&frame_count, 0, memory_order_relaxed);
    atomic_store_explicit(&drop_count, 0, memory_order_relaxed);
    atomic_store_explicit(&dma_failed, false, memory_order_relaxed);
    const size_t frame_size = bytes_per_chirp * chirps_per_frame;
    esp_err_t result = allocate_buffers(frame_size);
    if (result != ESP_OK)
    {
        radar_csi_release();
        return result;
    }

    // The ESP32-P4 bridge width and the DW-GDMA count use 64-bit words.
    mipi_csi_brg_ll_set_intput_data_h_pixel_num(csi.bridge_dev, bytes_per_chirp / 8);
    mipi_csi_brg_ll_set_intput_data_v_row_num(csi.bridge_dev, chirps_per_frame);
    // Even with image processing bypassed, ISP tracks the input as 32-bit words.
    isp_ll_set_intput_data_h_pixel_num(&ISP, bytes_per_chirp / 4);
    isp_ll_set_intput_data_v_row_num(&ISP, chirps_per_frame);
    dma_frame_words = frame_size / 8;
    int16_t* first;
    xQueueReceive(free_slots, &first, 0);
    atomic_store_explicit(&running, true, memory_order_relaxed);
    result = arm_dma(first);
    if (result != ESP_OK)
    {
        radar_csi_release();
        return result;
    }
    mipi_csi_brg_ll_enable(csi.bridge_dev, true);
    return ESP_OK;
}

esp_err_t radar_csi_stop(void)
{
    if (!atomic_exchange_explicit(&running, false, memory_order_relaxed))
    {
        return ESP_OK;
    }
    mipi_csi_brg_ll_enable(csi.bridge_dev, false);
    return dw_gdma_channel_enable_ctrl(dma, false);
}

void radar_csi_release(void)
{
    if (radar_csi_stop() != ESP_OK)
    {
        ESP_LOGE(tag, "Unable to disable CSI DMA");
        return;
    }

    if (free_slots)
    {
        vQueueDelete(free_slots);
        free_slots = NULL;
    }
    if (full_slots)
    {
        vQueueDelete(full_slots);
        full_slots = NULL;
    }

    for (unsigned i = 0; i < RING_SLOTS; ++i)
    {
        heap_caps_free(buffers[i]);
        buffers[i] = NULL;
    }
    heap_caps_free(discard_buffer);
    discard_buffer = NULL;
    dma_buffer = NULL;
}

bool radar_csi_take(radar_raw_frame_t* frame)
{
    if (xQueueReceive(full_slots, frame, 0) != pdTRUE)
    {
        return false;
    }
    const esp_err_t result = esp_cache_msync((void*)frame->data, dma_chunk_bytes,
                                             ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    if (result != ESP_OK)
    {
        radar_csi_return(frame);
        frame->data = NULL;
        atomic_store_explicit(&dma_failed, true, memory_order_relaxed);
        return false;
    }
    return true;
}

void radar_csi_return(const radar_raw_frame_t* frame)
{
    xQueueSend(free_slots, &frame->data, 0);
}

uint32_t radar_csi_frames(void)
{
    return atomic_load_explicit(&frame_count, memory_order_relaxed);
}

uint32_t radar_csi_dropped(void)
{
    return atomic_load_explicit(&drop_count, memory_order_relaxed);
}

bool radar_csi_failed(void)
{
    return atomic_load_explicit(&dma_failed, memory_order_relaxed);
}
