# Task 02: Real-Time Radar Capture

This project is the second in a series of projects demonstrating how to control
TI mmWave radar devices with an ESP32 and the mmWaveLink SDK. It builds on the
[Radar Initialization Demo](../01-init-demo/README.md) by adding desktop radar
configuration and a live range display.

## Overview

The ESP32-P4 configures the TI AWR2243 radar over **SPI** using mmWaveLink and
receives interleaved complex ADC samples over **MIPI CSI**. The firmware buffers
the raw radar frames in **PSRAM**, computes a **Range FFT** for each chirp and
receive channel, and streams the complex FFT bins over **USB High-Speed CDC**.
The PyQt desktop application uses the same USB connection to send radar
configuration and capture commands.

## Key Features

- **Radar Configuration** via the desktop application for Profile 0, chirp groups, and frame timing.
- **Continuous Acquisition** over two MIPI CSI data lanes, with DW-GDMA transferring frames into a PSRAM ring.
- **On-Device Range FFT** using the ESP-DSP kernel optimized for the ESP32-P4.
- **Real-Time Display** of four receive channels, with range in metres and magnitude plotted as `10 * log10(abs(fft))`.
- **HDF5 Recording** of complex Range FFT data from finite captures for offline analysis.

## What You Can Learn

By exploring the source code, you can see how to configure the radar from a
desktop application, capture raw CSI payloads while preserving the ADC sample
layout, and stream Range FFTs while acquisition continues.

### Capturing Radar Data Through MIPI CSI

The ESP32-P4 ISP normally interprets CSI payloads as image pixels. Its input
expects a supported image data type and matching line and frame dimensions.
The AWR2243 sends interleaved 16-bit I and Q values for four receive channels.
With a conventional image configuration, a mismatch in packet type or line
length can trigger ISP input errors and prevent complete radar frames from
being captured. ISP bypass skips image processing while keeping the CSI input
module active. That module still needs the correct line length and row count.
[ESP-IDF ISP Programming Guide](https://docs.espressif.com/projects/esp-idf/en/release-v6.0/esp32p4/api-reference/peripherals/isp.html#install-isp-driver) ·
[ESP-IDF 6.0.2 ISP driver](https://github.com/espressif/esp-idf/blob/v6.0.2/components/esp_driver_isp/src/isp_core.c#L138-L170)

The firmware maps one chirp to one CSI line and one radar frame to a set of
lines. The number of bytes per chirp is the ADC sample count multiplied by
16 bytes per sample across the four receive channels.

1. **CSI Bridge.** The firmware powers and clocks the CSI PHY, then calls
   `mipi_csi_hal_init()` for two data lanes at 600 Mbps per lane. The
   `mipi_csi_brg_ll_*` functions admit CSI data types in the range
   `0x12`–`0x2F`, set a DMA burst length of 512 64-bit words, and select
   color-conversion bypass. At capture start, the bridge width is set to
   `bytes_per_chirp / 8` in 64-bit words and the height to
   `chirps_per_frame`.
2. **ISP bypass.** `esp_isp_new_processor()` selects CSI input, enables
   line-start and line-end packet handling, and sets `flags.bypass_isp`.
   Both format fields are set to RAW8 to satisfy the bypass driver's
   matching-format requirement. The I/Q bytes pass through unchanged.
   At capture start, the `isp_ll_set_intput_data_*` functions replace the
   initial dimensions with `bytes_per_chirp / 4` in 32-bit words and
   `chirps_per_frame`.
3. **DW-GDMA.** The firmware uses ESP-IDF's internal `dw_gdma_*` driver
   to configure a source-controlled transfer with a CSI hardware handshake.
   The source address is fixed at `MIPI_CSI_BRG_MEM_BASE` and the PSRAM
   destination address increments. Each frame takes
   `bytes_per_chirp * chirps_per_frame / 8` 64-bit transfers. The firmware
   arms DMA before enabling the bridge with `mipi_csi_brg_ll_enable()`.

On completion, the DMA callback arms the next transfer before queuing the
completed buffer for processing. If all four PSRAM slots are busy, the next
frame goes into a separate discard buffer and is counted as dropped. The
firmware synchronizes the cache before reading a completed buffer.
[CSI capture source](esp32/main/src/periph/radar_csi.c)

## Hardware Requirements

- ESP32-P4 radar development board with a TI AWR2243 radar and PSRAM.
  The default firmware configuration targets ESP32-P4 revision v3.1 or later.
- USB High-Speed connection for radar control and data, plus a USB Serial/JTAG
  connection for flashing and firmware logs.

## Build and Run

Activate ESP-IDF 6.0.2. From the `02-realtime-capture` directory, initialize
the mmWaveLink submodule, then build and flash the firmware. Use
`idf.py -C esp32 menuconfig` to adjust board settings under
**mmWave Radar Subsystem Configuration**.

```sh
git submodule update --init esp32/components/mmwavelink-esp32
idf.py -C esp32 build
idf.py -C esp32 -p /dev/ttyACM0 flash monitor
```

Replace `/dev/ttyACM0` with your board's USB Serial/JTAG port if needed. In a
second terminal, use `uv` with Python 3.11 or newer to start the desktop
application.

```sh
cd host
uv sync --locked
uv run radar-capture
```

1. In **Radar Status**, select the **ESP Radar Capture** CDC port and click
   **Connect**.
2. Edit **Profile 0**, **Chirps**, and **Frame**. Chirp groups can enable TX0,
   TX2, both, or neither, and must cover the frame's chirp interval. Click
   **Apply Configuration**, then **Start**.
3. Set **Frame count** to zero for continuous display until you click
   **Stop**. A positive count records the received FFTs and offers
   **Save As…** when the capture ends. Use **Save Recording…** to save a
   recording later in the same session.

## Recorded Data

Each HDF5 recording contains the following datasets.

| Dataset | Contents |
| --- | --- |
| `range_fft` | Complex64 FFT values indexed by received chirp, RX channel, and range bin |
| `indices` | Zero-based frame index and chirp index within the frame for each received chirp |
| `range_m` | Range in metres for each FFT bin |

File attributes include the radar configuration, completion code, and frame
and drop counts. The firmware zero-pads each chirp to a power of two and
streams the half-spectrum selected by the chirp slope, starting at DC. The
range axis uses the programmed sample rate and quantized chirp slope.

## Acknowledgments

OpenAI Codex assisted in writing the code for this project.
