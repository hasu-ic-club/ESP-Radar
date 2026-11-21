idf_component_register(SRCS "src/main.c"
                            "src/periph/sdcard.c"
                            "src/periph/csi.c"
                            "src/port/mmwavelink_port.c"
                    INCLUDE_DIRS "include"
                    REQUIRES
                    mmwavelink-esp32
                    esp_driver_isp
                    esp_driver_cam
                    esp_mm
                    esp_hw_support
                    esp_driver_gpio
                    esp_driver_spi
                    esp_driver_sdmmc
                    fatfs)
