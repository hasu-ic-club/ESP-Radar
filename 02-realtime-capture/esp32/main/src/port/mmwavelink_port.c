// + ----------------------------------------------------------------- +
// - File Name    : mmwavelink_port.c
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

#include "port/mmwavelink_port.h"

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

// ESP-IDF drivers
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>

// ESP-IDF libraries
#include <esp_check.h>
#include <esp_log.h>
#include <esp_rom_sys.h>

// mmWaveLink library
#include <mmwavelink.h>
#include <mmwavelink_esp.h>
#include <mmwl_esp_freertos.h>

static const char* TAG = "MMWL";
static const char* RADAR_TAG = "RDR";

static spi_device_handle_t xwr_spi;
static SemaphoreHandle_t spi_mutex;
static EventGroupHandle_t* async_event_grp_singleton;

static esp_err_t init_spi(void);
static esp_err_t init_gpio(void);
static esp_err_t init_host_intr(mmwl_esp_context_t* handle_to_pass);

// Callbacks for mmwave link library
static rlInt32_t xwr_print(const rlInt8_t* format, ...);
static uint8_t get_xwr_irq_pin_lvl(uint8_t device_id);
static void enable_host_irq(uint8_t device_id);
static void enable_device(mmwl_esp_context_t* context, uint8_t device_id);
static void disable_device(mmwl_esp_context_t* context, uint8_t device_id);
static int32_t write_data(mmwl_esp_context_t* context, uint8_t device_id,
                          uint8_t* buffer, uint16_t length);
static int32_t read_data(mmwl_esp_context_t* context, uint8_t device_id,
                         uint8_t* buffer, uint16_t length);

static void pwup_done_cb(uint8_t device_id, uint8_t* payload, uint16_t length);
static void rf_pwup_done_cb(uint8_t device_id, uint8_t* payload, uint16_t length);
static void init_calib_done_cb(uint8_t device_id, uint8_t* payload, uint16_t length);
static void frame_trig_rdy_cb(uint8_t device_id, uint8_t* payload, uint16_t length);

esp_err_t init_radar_peripherals(mmwl_esp_context_t* mmwl_handle)
{
    // Initialize SPI
    ESP_RETURN_ON_ERROR(init_spi(), TAG,
                        "Failed to initialize the SPI peripheral for radar.");

    // Initialize GPIO
    ESP_RETURN_ON_ERROR(init_gpio(), TAG,
                        "Failed to initialize the GPIO peripheral for radar.");

    // Initialize host interrupt
    ESP_RETURN_ON_ERROR(
        init_host_intr(mmwl_handle), TAG,
        "Failed to initialize interrupt GPIO / register the ISR for radar.");

    // Set initial states on GPIO
    gpio_set_level(CONFIG_XWR_SPI_CSN_GPIO, 1);
    gpio_set_level(CONFIG_XWR_RESET_N_GPIO, 0);

    return ESP_OK;
}

void link_mmwl_handle(mmwl_esp_context_t* mmwl_handle,
                      EventGroupHandle_t* async_event_grp)
{
    // Set singletons to communication with applications
    async_event_grp_singleton = async_event_grp;

    // Initialize the worker spawn task for mmwavelink driver
    mmwl_esp_worker_spawn_task_init();

    // Create mmwavelink handle
    mmwl_handle->ack_timeout_ms = 100;                 // Set 100 ms timeout
    mmwl_handle->default_crc_type = RL_CRC_TYPE_32BIT; // Use CRC-32
    mmwl_handle->dbg_level = RL_DBG_LEVEL_WARNING;     // Debug level: Warning
    mmwl_handle->dbg_print_cb = xwr_print;
    mmwl_handle->dev_type = RL_AR_DEVICETYPE_22XX; // Radar type: xWR22xx

    //  Hardware callbacks
    mmwl_handle->hw_cb.get_host_irq_pin_level = get_xwr_irq_pin_lvl;
    mmwl_handle->hw_cb.enable_host_irq = enable_host_irq;
    mmwl_handle->hw_cb.enable_device = enable_device;
    mmwl_handle->hw_cb.disable_device = disable_device;
    mmwl_handle->hw_cb.write_data = write_data;
    mmwl_handle->hw_cb.read_data = read_data;

    // Event callbacks
    mmwl_handle->rf_ae_event_cb.init_calib_status = init_calib_done_cb;
    mmwl_handle->dev_event_cb.power_up_done = pwup_done_cb;
    mmwl_handle->dev_event_cb.rf_power_up_done = rf_pwup_done_cb;
    mmwl_handle->rf_ae_event_cb.frame_trigger_ready = frame_trig_rdy_cb;

    mmwl_esp_client_connect(mmwl_handle);
}

/// @brief Interrupt handler
static void IRAM_ATTR xwr_irq_handler(void* arg)
{
    mmwl_esp_context_t* context = arg;
    mmwl_esp_host_irq_handler(context, 0);
}

static esp_err_t init_spi(void)
{
    // SPI initialize
    const spi_bus_config_t bus_config = {.mosi_io_num = CONFIG_XWR_SPI_MOSI_GPIO,
                                         .miso_io_num = CONFIG_XWR_SPI_MISO_GPIO,
                                         .sclk_io_num = CONFIG_XWR_SPI_SCK_GPIO,

                                         .quadhd_io_num = -1,
                                         .quadwp_io_num = -1,

                                         .max_transfer_sz =
                                             CONFIG_XWR_SPI_MAX_TRANSFER_SIZE};

    esp_err_t ret =
        spi_bus_initialize(CONFIG_XWR_SPI_HOST, &bus_config, SPI_DMA_DISABLED);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI bus initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t dev_config = {.clock_speed_hz =
                                                    CONFIG_XWR_SPI_CLOCK_HZ,
                                                .mode = CONFIG_XWR_SPI_MODE,
                                                .spics_io_num = -1,
                                                .queue_size = 1};

    ret = spi_bus_add_device(CONFIG_XWR_SPI_HOST, &dev_config, &xwr_spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI device initialization failed: %s", esp_err_to_name(ret));

        spi_bus_free(CONFIG_XWR_SPI_HOST);
        return ret;
    }

    spi_mutex = xSemaphoreCreateMutex();

    if (spi_mutex == NULL)
    {
        spi_bus_remove_device(xwr_spi);
        spi_bus_free(CONFIG_XWR_SPI_HOST);
        xwr_spi = NULL;

        return ESP_ERR_NO_MEM;
    }

    return ret;
}

static esp_err_t init_gpio(void)
{
    // Reset
    const gpio_config_t reset_gpio = {
        .pin_bit_mask = 1ULL << CONFIG_XWR_RESET_N_GPIO,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(gpio_config(&reset_gpio), TAG, "Reset GPIO init failed");

    // SPI CSn
    const gpio_config_t csn_gpio = {
        .pin_bit_mask = 1ULL << CONFIG_XWR_SPI_CSN_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(gpio_config(&csn_gpio), TAG, "CSN GPIO init failed");

    return ESP_OK;
}

static esp_err_t init_host_intr(mmwl_esp_context_t* handle_to_pass)
{
    const gpio_config_t intr_config = {
        .pin_bit_mask = 1ULL << CONFIG_XWR_HOST_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };

    ESP_RETURN_ON_ERROR(gpio_config(&intr_config), TAG, "Interrupt GPIO init failed.");

    const esp_err_t ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to install GPIO interrupt: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add(CONFIG_XWR_HOST_INT_GPIO, xwr_irq_handler, handle_to_pass),
        TAG, "Interrupt service routine registration failed.");

    return ESP_OK;
}

// Following functions are register to callbacks of mmwave-link library
static rlInt32_t xwr_print(const rlInt8_t* format, ...)
{
    if (xPortInIsrContext())
    {
        return 0;
    }

    va_list args;
    va_start(args, format);

    esp_log_writev(ESP_LOG_INFO, RADAR_TAG, (const char*)format, args);

    va_end(args);
    return 0;
}

static uint8_t get_xwr_irq_pin_lvl(uint8_t device_id)
{
    (void)device_id;
    return gpio_get_level(CONFIG_XWR_HOST_INT_GPIO) ? 1 : 0;
}

static void enable_host_irq(uint8_t device_id)
{
    if (device_id != 0)
    {
        ESP_LOGW(TAG, "Radar device %d is invalid.", device_id);
        return;
    }

    gpio_intr_enable(CONFIG_XWR_HOST_INT_GPIO);
    ESP_LOGI(TAG, "Host interrupt enabled.");
}

static void enable_device(mmwl_esp_context_t* context, uint8_t device_id)
{
    (void)context;
    if (device_id != 0)
    {
        ESP_LOGW(TAG, "Enable device: radar device %d is invalid.", device_id);
        return;
    }

    gpio_set_level(CONFIG_XWR_RESET_N_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void disable_device(mmwl_esp_context_t* context, uint8_t device_id)
{
    (void)context;
    if (device_id != 0)
    {
        ESP_LOGW(TAG, "Disable device: radar device %d is invalid.", device_id);
        return;
    }

    gpio_set_level(CONFIG_XWR_RESET_N_GPIO, 0);
    esp_rom_delay_us(20);
}

static __always_inline esp_err_t xwr_spi_transfer16(const uint8_t* tx, uint8_t* rx)
{
    spi_transaction_t trans = {0};
    trans.length = 16;

    if (tx != NULL)
    {
        trans.flags |= SPI_TRANS_USE_TXDATA;
        trans.tx_data[0] = tx[1];
        trans.tx_data[1] = tx[0];
    }

    if (rx != NULL)
    {
        trans.flags |= SPI_TRANS_USE_RXDATA;
    }

    gpio_set_level(CONFIG_XWR_SPI_CSN_GPIO, 0);
    esp_rom_delay_us(XWR_SPI_CS_DELAY_US);
    const esp_err_t ret = spi_device_polling_transmit(xwr_spi, &trans);
    gpio_set_level(CONFIG_XWR_SPI_CSN_GPIO, 1);

    if (ret != ESP_OK)
        return ret;

    if (rx != NULL)
    {
        rx[0] = trans.rx_data[1];
        rx[1] = trans.rx_data[0];
    }

    return ESP_OK;
}

static int32_t write_data(mmwl_esp_context_t* context, uint8_t device_id,
                          uint8_t* buffer, uint16_t length)
{
    if (device_id != 0 || buffer == NULL || (length & 1U))
        return -1;

    if (xSemaphoreTake(spi_mutex, portMAX_DELAY) != pdTRUE)
    {
        return -1;
    }

    esp_err_t ret = ESP_OK;

    esp_rom_delay_us(50);

    for (uint16_t bid = 0; bid < length; bid += 2)
    {
        ret = xwr_spi_transfer16(&buffer[bid], NULL);

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "xwr_spi_transfer16 failed: %x", ret);
            break;
        }
    }

    xSemaphoreGive(spi_mutex);
    return (ret == ESP_OK) ? (int32_t)length : -1;
}

static int32_t read_data(mmwl_esp_context_t* context, uint8_t device_id,
                         uint8_t* buffer, uint16_t length)
{
    if (device_id != 0 || buffer == NULL || (length & 1U))
        return -1;

    if (xSemaphoreTake(spi_mutex, portMAX_DELAY) != pdTRUE)
    {
        return -1;
    }

    const uint8_t dummy[2] = {0xFF, 0xFF};
    esp_err_t ret = ESP_OK;

    for (uint16_t bid = 0; bid < length; bid += 2)
    {
        ret = xwr_spi_transfer16(dummy, &buffer[bid]);

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "SPI read failed at offset %u: %s", bid,
                     esp_err_to_name(ret));
            break;
        }
    }

    xSemaphoreGive(spi_mutex);
    return (ret == ESP_OK) ? (int32_t)length : -1;
}

static void pwup_done_cb(uint8_t device_id, uint8_t* payload, uint16_t length)
{
    ESP_LOGI(TAG, "Radar power up done.");
    xEventGroupSetBits(*async_event_grp_singleton, XWR_ASYNC_EVENT_RADAR_PWUP_DONE);
}

static void rf_pwup_done_cb(uint8_t device_id, uint8_t* payload, uint16_t length)
{
    ESP_LOGI(TAG, "Radar RF power up done.");
    xEventGroupSetBits(*async_event_grp_singleton, XWR_ASYNC_EVENT_RF_PWUP_DONE);
}

static void init_calib_done_cb(uint8_t device_id, uint8_t* payload, uint16_t length)
{
    ESP_LOGI(TAG, "Radar calibration done.");
    xEventGroupSetBits(*async_event_grp_singleton, XWR_ASYNC_EVENT_CALIB_DONE);
}

static void frame_trig_rdy_cb(uint8_t device_id, uint8_t* payload, uint16_t length)
{
    ESP_LOGI(TAG, "Radar frame trig.");
}
