from dataclasses import dataclass


class VL53L4CDError(Exception):
    """Raised for VL53L4CD driver failures (timeouts, invalid arguments)."""


@dataclass
class Result:
    """One ranging measurement, decoded from the result registers."""

    status: int = 255                 # 0 -> valid measurement
    distance: int = 0                 # in mm
    ambient_noise_rate: int = 0       # in kcps
    ambient_noise_rate_spad: int = 0  # in kcps per SPAD
    signal_rate: int = 0              # in kcps
    signal_per_spad: int = 0          # in kcps per SPAD
    spad_count: int = 0
    std_dev: int = 0                  # in mm


@dataclass
class RangeTiming:
    """Range timing configuration, both values in ms."""

    timing_budget: int = 50
    inter_measurement_time: int = 0


@dataclass
class Threshold:
    """Distance threshold window in mm plus the window mode."""

    low: int = 0
    high: int = 0
    window: int = 0  # 0 = below low, 1 = above high, 2 = outside, 3 = inside
