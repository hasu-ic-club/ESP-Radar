"""Validate AWR2243 settings and encode mmWaveLink's native units."""

from dataclasses import asdict, dataclass
from math import isfinite
from struct import Struct

import numpy as np

RX_CHANNELS = 4
MAX_ADC_SAMPLES = 1024
MAX_CHIRP_INDEX = 511
MAX_FRAME_CHIRPS = 4095
MAX_FRAME_BYTES = 2 * 1024 * 1024
ADC_BYTES_PER_SAMPLE = RX_CHANNELS * 4
TX_MASK = 0b101
START_FREQ_LSB_GHZ = 3.6 / 2**26
SLOPE_LSB_MHZ_US = 3.6e3 * 900 / 2**26
SPEED_OF_LIGHT = 299_792_458.0
# A high-speed bulk endpoint can carry at most 13 packets per 125 µs.
USB_HS_BULK_BYTES_PER_SECOND = 13 * 512 * 8000
FFT_PACKET_OVERHEAD_BYTES = 28

_CONFIG = Struct("<IIIIhhHHHHHHIHH")
_CHIRP = Struct("<HHH")


class ConfigError(ValueError):
    """A setting that must be corrected before sending configuration."""


@dataclass(frozen=True)
class Profile:
    start_ghz: float = 77.0
    idle_us: float = 20.0
    adc_start_us: float = 6.0
    ramp_end_us: float = 80.0
    slope_mhz_us: float = 49.970
    tx_start_us: float = 0.0
    adc_samples: int = 384
    sample_rate_ksps: int = 6000


@dataclass(frozen=True)
class ChirpGroup:
    start: int = 0
    end: int = 0
    tx_mask: int = 0b001


@dataclass(frozen=True)
class Frame:
    start: int = 0
    end: int = 0
    loops: int = 1
    count: int = 0
    period_ms: float = 40.0


@dataclass(frozen=True)
class RadarConfig:
    profile: Profile = Profile()
    chirps: tuple[ChirpGroup, ...] = (ChirpGroup(),)
    frame: Frame = Frame()

    @property
    def chirps_per_frame(self) -> int:
        return (self.frame.end - self.frame.start + 1) * self.frame.loops

    @property
    def fft_length(self) -> int:
        return 1 << (self.profile.adc_samples - 1).bit_length()

    @property
    def range_bins(self) -> int:
        return self.fft_length // 2

    def range_axis(self) -> np.ndarray:
        """Convert FFT bins to metres using the programmed slope and rate."""
        native_slope = round(self.profile.slope_mhz_us / SLOPE_LSB_MHZ_US)
        slope_hz_per_second = abs(native_slope * SLOPE_LSB_MHZ_US) * 1e12
        sample_rate_hz = self.profile.sample_rate_ksps * 1e3
        bin_hz = sample_rate_hz / self.fft_length
        bin_m = SPEED_OF_LIGHT * bin_hz / (2 * slope_hz_per_second)
        return np.arange(self.range_bins, dtype=np.float64) * bin_m

    def metadata(self) -> dict:
        """Return the source configuration for a saved capture."""
        return asdict(self)

    def encode(self) -> bytes:
        """Validate all settings before building the USB command."""
        profile = self.profile
        frame = self.frame
        _check("Start frequency (GHz)", profile.start_ghz, 76, 81)
        _check("Idle time (µs)", profile.idle_us, 0, 5242.87)
        _check("ADC start time (µs)", profile.adc_start_us, 0, 40.95)
        _check("Ramp end time (µs)", profile.ramp_end_us, 0.01, 5000)
        max_slope = 5510 * SLOPE_LSB_MHZ_US
        _check(
            "Chirp slope (MHz/µs)", profile.slope_mhz_us, -max_slope, max_slope
        )
        _check("TX start time (µs)", profile.tx_start_us, -40.96, 40.95)
        _check("ADC samples", profile.adc_samples, 2, MAX_ADC_SAMPLES)
        _check("ADC sample rate (kSps)", profile.sample_rate_ksps, 2000, 45000)

        start = round(profile.start_ghz / START_FREQ_LSB_GHZ)
        idle = round(profile.idle_us * 100)
        adc_start = round(profile.adc_start_us * 100)
        ramp = round(profile.ramp_end_us * 100)
        slope = round(profile.slope_mhz_us / SLOPE_LSB_MHZ_US)
        tx_start = round(profile.tx_start_us * 100)

        if slope == 0:
            raise ConfigError("Chirp slope must be nonzero for Range FFT.")
        # One 10 ns ramp tick at one slope LSB moves by nine RF-frequency LSBs.
        if not 0x5471C71B <= start + 9 * (ramp + 1) * slope <= 0x5A000000:
            raise ConfigError(
                "The programmed chirp must stay within 76–81 GHz."
            )
        if (ramp - adc_start) * profile.sample_rate_ksps < (
            profile.adc_samples * 100_000
        ):
            raise ConfigError("ADC sampling must finish before the ramp ends.")
        if not -idle <= tx_start < ramp:
            raise ConfigError("TX start time must fall within the chirp.")

        self._validate_chirps()
        _check("Frame loops", frame.loops, 1, 255)
        _check("Frame count", frame.count, 0, 65535)
        _check("Frame period (ms)", frame.period_ms, 0.3, 1342)
        period = round(frame.period_ms * 200_000)
        minimum = 2 * (idle + ramp + 1) * self.chirps_per_frame + 60_000
        if period < minimum:
            raise ConfigError(
                f"Frame period must be at least {minimum / 200_000:.5f} ms "
                "to fit all chirps and the 300 µs inter-frame interval."
            )

        frame_bytes = (
            self.chirps_per_frame * profile.adc_samples * ADC_BYTES_PER_SAMPLE
        )
        if (
            self.chirps_per_frame > MAX_FRAME_CHIRPS
            or frame_bytes > MAX_FRAME_BYTES
        ):
            raise ConfigError(
                "A frame must fit within 4,095 chirps and 2 MiB of raw data."
            )
        # A RAW8 CSI transfer may use at most the two 600 Mbit/s lanes.
        if profile.adc_samples * ADC_BYTES_PER_SAMPLE * 100 > (
            150 * (idle + ramp + 1)
        ):
            raise ConfigError(
                "Increase chirp duration or reduce ADC samples to fit CSI "
                "bandwidth."
            )

        fft_packet_bytes = (
            self.range_bins * RX_CHANNELS * 8 + FFT_PACKET_OVERHEAD_BYTES
        )
        if (
            fft_packet_bytes * self.chirps_per_frame * 200_000_000
            > USB_HS_BULK_BYTES_PER_SECOND * period
        ):
            raise ConfigError(
                "Increase the frame period or reduce the number of chirps "
                "or samples to fit USB High Speed bandwidth."
            )

        payload = _CONFIG.pack(
            start,
            idle,
            adc_start,
            ramp,
            slope,
            tx_start,
            profile.adc_samples,
            profile.sample_rate_ksps,
            frame.start,
            frame.end,
            frame.loops,
            frame.count,
            period,
            2 * profile.adc_samples,
            len(self.chirps),
        )
        return payload + b"".join(
            _CHIRP.pack(group.start, group.end, group.tx_mask)
            for group in self.chirps
        )

    def _validate_chirps(self) -> None:
        _check_chirp_indices("Frame", self.frame.start, self.frame.end)
        _check("Number of chirp groups", len(self.chirps), 1, 512)

        used = set()
        for row, group in enumerate(self.chirps, 1):
            name = f"Chirp group {row}"
            _check_chirp_indices(name, group.start, group.end)
            if group.tx_mask < 0 or group.tx_mask & ~TX_MASK:
                raise ConfigError(f"{name} can only enable TX0 and TX2.")
            indices = set(range(group.start, group.end + 1))
            if not used.isdisjoint(indices):
                raise ConfigError(f"{name} overlaps another chirp group.")
            used.update(indices)

        required = set(range(self.frame.start, self.frame.end + 1))
        if not required <= used:
            raise ConfigError(
                f"Frame chirp {min(required - used)} has no chirp group."
            )


def _check(name: str, value: float, low: float, high: float) -> None:
    if not isfinite(value) or not low <= value <= high:
        raise ConfigError(f"{name} must be between {low:g} and {high:g}.")


def _check_chirp_indices(name: str, start: int, end: int) -> None:
    _check(f"{name} start index", start, 0, MAX_CHIRP_INDEX)
    _check(f"{name} end index", end, start, MAX_CHIRP_INDEX)
