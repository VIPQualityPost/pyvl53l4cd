"""Reference Python driver for the VL53L4CD Time-of-Flight sensor.

Port of the ST VL53L4CD Ultra Lite Driver API (registers, fixed-point
range-timing math and result decoding follow the ST ``VL53L4CD_ULD``
implementation); C error codes surface as ``VL53L4CDError`` exceptions.
"""

import logging
import time
from time import sleep

from .config import CfgBytes
from .registers import VL53L4CDAddress as REG
from .transport import Transport
from .types import RangeTiming, Result, Threshold, VL53L4CDError

logger = logging.getLogger(__name__)

# ST mapping from the raw range-status nibble to the user-visible status.
# 0 is the only guaranteed-valid status; values >= 6 are measurement
# quality flags (see the ST ULD/vl53l4cd_api.cpp table).
_RANGE_STATUS = (
    255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
    255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
    255, 255, 11, 12,
)


class VL53L4CD:
    """VL53L4CD ranging sensor on a register-level I2C transport.

    ``transport`` implements the :class:`~pyvl53l4cd.transport.Transport`
    protocol; default is :class:`~pyvl53l4cd.ftdi.FtdiI2cTransport`.
    """

    def __init__(self, transport: Transport | None = None) -> None:
        if transport is None:
            from .ftdi import FtdiI2cTransport

            transport = FtdiI2cTransport()
        self.dev = transport

    # ------------------------------------------------------------------
    # register helpers
    # ------------------------------------------------------------------
    def _read_u8(self, addr: int) -> int:
        return self.dev.read(addr, 1)[0]

    def _read_u16(self, addr: int) -> int:
        return int.from_bytes(self.dev.read(addr, 2), "big")

    def _read_u32(self, addr: int) -> int:
        return int.from_bytes(self.dev.read(addr, 4), "big")

    def _write_u16(self, addr: int, value: int) -> None:
        self.dev.write_bytes(addr, value.to_bytes(2, "big"))

    def _write_u32(self, addr: int, value: int) -> None:
        self.dev.write_bytes(addr, value.to_bytes(4, "big"))

    # ------------------------------------------------------------------
    # identification / addressing
    # ------------------------------------------------------------------
    def set_i2c_addr(self, new_address: int) -> None:
        """Move the sensor to ``new_address`` (8-bit form, e.g. 0x52)."""
        self.dev.write(REG.I2C_ADDR, new_address >> 1)
        device_addr = getattr(self.dev, "device_addr", None)
        if device_addr is not None:
            self.dev.device_addr = new_address >> 1

    def get_sensor_id(self) -> int:
        """Sensor identification word; VL53L4CD returns 0xEBAA."""
        return self._read_u16(REG.MODEL_ID)

    # ------------------------------------------------------------------
    # init / ranging control
    # ------------------------------------------------------------------
    def init_sensor(self) -> None:
        """Boot the sensor, load the default configuration and leave it
        stopped with a 50 ms range timing (ST ``VL53L4CD_SensorInit``)."""
        timeout = 0
        while self._read_u8(REG.FIRMWARE_SYS_STATUS) != 0x03:
            timeout += 1
            if timeout >= 100:  # ~1 s
                raise VL53L4CDError("Sensor init timed out!")
            sleep(0.01)

        self.load_default_config()

        # Initial VHV calibration pass, exactly like the ST driver.
        self.dev.write(REG.SYSTEM_START, 0x40)
        self.wait_ready()
        self.stop()

        self.dev.write(REG.VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09)
        self.dev.write(0x000B, 0x00)
        self._write_u16(0x0024, 0x0500)
        self.range_timing = RangeTiming(timing_budget=50, inter_measurement_time=0)

    def load_default_config(self) -> None:
        """Write the 91-byte default configuration to registers 0x2D..0x87."""
        self.dev.write_bytes(0x2D, bytes(CfgBytes()))

    def clear_interrupt(self) -> None:
        """Re-arm the data-ready interrupt for the next measurement."""
        self.dev.write(REG.SYSTEM_INTERRUPT_CLEAR, 0x01)

    def start(self) -> None:
        """Start ranging. Continuous (0x21) when the inter-measurement
        period is zero, single-shot (0x40) otherwise (ST semantics)."""
        inter_measurement = self._read_u32(REG.INTERMEASUREMENT_MS)
        if inter_measurement == 0:
            self.dev.write(REG.SYSTEM_START, 0x21)
        else:
            self.dev.write(REG.SYSTEM_START, 0x40)

    def stop(self) -> None:
        """Stop ranging."""
        self.dev.write(REG.SYSTEM_START, 0x00)

    def is_ready(self) -> bool:
        """True when a fresh measurement is available (data-ready line)."""
        polarity = self._read_u8(REG.GPIO_HV_MUX_CTRL) & 0x10
        int_pol = 0 if polarity else 1
        status = self._read_u8(REG.GPIO_TIO_HV_STATUS) & 0x01
        return status == int_pol

    def wait_ready(self, timeout: float = 1.0) -> None:
        """Block until a measurement is ready, then clear the interrupt."""
        deadline = time.monotonic() + timeout
        while not self.is_ready():
            if time.monotonic() >= deadline:
                raise VL53L4CDError("Sensor was not ready in timeout period!")
            sleep(0.01)
        self.clear_interrupt()

    def get_result(self) -> Result:
        """Decode the result registers into a :class:`Result`."""
        raw_status = self._read_u8(REG.RESULT_RANGE_STATUS) & 0x1F
        status = _RANGE_STATUS[raw_status] if raw_status < len(_RANGE_STATUS) else raw_status

        spad_count = self._read_u16(REG.RESULT_SPAD_NB) // 256
        if spad_count == 0:
            return Result(status=255)  # no measurement: force invalid status

        signal_rate = self._read_u16(REG.RESULT_SIGNAL_RATE) * 8
        ambient_rate = self._read_u16(REG.RESULT_AMBIENT_RATE) * 8
        std_dev = self._read_u16(REG.RESULT_SIGMA) // 4
        distance = self._read_u16(REG.RESULT_DISTANCE)

        return Result(
            status=status,
            distance=distance,
            ambient_noise_rate=ambient_rate,
            ambient_noise_rate_spad=ambient_rate // spad_count,
            signal_rate=signal_rate,
            signal_per_spad=signal_rate // spad_count,
            spad_count=spad_count,
            std_dev=std_dev,
        )

    # ------------------------------------------------------------------
    # range timing (ST fixed-point config writes)
    # ------------------------------------------------------------------
    @property
    def range_timing(self) -> RangeTiming:
        """Current timing budget / inter-measurement period, both in ms."""
        inter_measurement = self._read_u32(REG.INTERMEASUREMENT_MS)

        clock_pll = (self._read_u16(REG.RESULT_OSC_CAL_VAL) & 0x3FF) or 1
        clock_pll = int(1.065 * clock_pll)  # ST conversion factor
        inter_measurement_ms = inter_measurement // clock_pll

        osc_frequency = self._read_u16(REG.OSC_FREQ) or 1
        macro_period_us = (2304 * (0x40000000 // osc_frequency)) >> 6

        macrop = self._read_u16(REG.RANGE_CONFIG_A)
        ls_byte = (macrop & 0x00FF) << 4
        ms_byte = 4 - ((macrop & 0xFF00) >> 8)  # 4 - (ms - 1) - 1 per ST

        macro_period_us = macro_period_us * 16
        timing_budget_us = (
            ((ls_byte + 1) * (macro_period_us >> 6) - ((macro_period_us >> 6) >> 1)) >> 12
        )
        if ms_byte < 12:
            timing_budget_us >>= ms_byte

        if inter_measurement == 0:  # continuous mode
            timing_budget_us += 2500
        else:  # autonomous low-power mode
            timing_budget_us = timing_budget_us * 2 + 4300

        return RangeTiming(
            timing_budget=timing_budget_us // 1000,
            inter_measurement_time=inter_measurement_ms,
        )

    @range_timing.setter
    def range_timing(self, timing: RangeTiming) -> None:
        """Program the range timing. ``timing_budget`` must be 10..200 ms;
        ``inter_measurement_time`` must be 0 (continuous) or > timing budget
        (autonomous low-power mode), mirroring ST validation."""
        budget_ms = timing.timing_budget
        inter_ms = timing.inter_measurement_time
        if not 10 <= budget_ms <= 200:
            raise VL53L4CDError("timing budget must be between 10 ms and 200 ms")

        osc_frequency = self._read_u16(REG.OSC_FREQ)
        if osc_frequency == 0:
            raise VL53L4CDError("invalid oscillator frequency (0)")
        timing_budget_us = budget_ms * 1000
        macro_period_us = (2304 * (0x40000000 // osc_frequency)) >> 6

        if inter_ms == 0:  # continuous mode
            self._write_u32(REG.INTERMEASUREMENT_MS, 0)
            timing_budget_us -= 2500
        elif inter_ms > budget_ms:  # autonomous low-power mode
            clock_pll = self._read_u16(REG.RESULT_OSC_CAL_VAL) & 0x3FF
            self._write_u32(REG.INTERMEASUREMENT_MS, int(1.055 * inter_ms * clock_pll))
            timing_budget_us = (timing_budget_us - 4300) // 2
        else:
            raise VL53L4CDError(
                "inter-measurement must be 0 (continuous) or greater than the timing budget"
            )

        timing_budget_us <<= 12

        # Range config A: 16 * macro period
        tmp = macro_period_us * 16
        ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) // (tmp >> 6)) - 1
        ms_byte = 0
        while ls_byte & 0xFFFFFF00:
            ls_byte >>= 1
            ms_byte += 1
        self._write_u16(REG.RANGE_CONFIG_A, (ms_byte << 8) | (ls_byte & 0xFF))

        # Range config B: 12 * macro period
        tmp = macro_period_us * 12
        ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) // (tmp >> 6)) - 1
        ms_byte = 0
        while ls_byte & 0xFFFFFF00:
            ls_byte >>= 1
            ms_byte += 1
        self._write_u16(REG.RANGE_CONFIG_B, (ms_byte << 8) | (ls_byte & 0xFF))

    # ------------------------------------------------------------------
    # calibration: offset and crosstalk
    # ------------------------------------------------------------------
    @property
    def offset(self) -> int:
        """Current distance offset correction in mm."""
        temp = (self._read_u16(REG.RANGE_OFFSET_MM) << 3) & 0xFFFF
        offset = temp >> 5
        if offset > 1024:
            offset -= 2048
        return offset

    @offset.setter
    def offset(self, offset_mm: int) -> None:
        """Set the distance offset correction in mm (-1024..1023)."""
        self._write_u16(REG.RANGE_OFFSET_MM, (offset_mm * 4) & 0xFFFF)
        self._write_u16(REG.INNER_OFFSET_MM, 0x0000)
        self._write_u16(REG.OUTER_OFFSET_MM, 0x0000)

    @property
    def crosstalk(self) -> int:
        """Current crosstalk correction in kcps (0 = no cover glass)."""
        return (self._read_u16(REG.CROSSTALK_PLANE_OFFSET) * 1000) >> 9

    @crosstalk.setter
    def crosstalk(self, value_kcps: int) -> None:
        """Set the crosstalk correction in kcps (0..128)."""
        self._write_u16(REG.CROSSTALK_X_PLANE_GRADIENT, 0x0000)
        self._write_u16(REG.CROSSTALK_Y_PLANE_GRADIENT, 0x0000)
        self._write_u16(REG.CROSSTALK_PLANE_OFFSET, (value_kcps << 9) // 1000)

    # ------------------------------------------------------------------
    # thresholds
    # ------------------------------------------------------------------
    @property
    def detection_thresholds(self) -> Threshold:
        """Current distance threshold window (mm) and window mode."""
        high = self._read_u16(REG.THRESH_HIGH)
        low = self._read_u16(REG.THRESH_LOW)
        window = self._read_u8(REG.SYSTEM_INTERRUPT) & 0x07
        return Threshold(low=low, high=high, window=window)

    @detection_thresholds.setter
    def detection_thresholds(self, threshold: Threshold) -> None:
        """Set the distance threshold window (low/high mm) and mode:
        0 = below low, 1 = above high, 2 = outside, 3 = inside."""
        self.dev.write(REG.SYSTEM_INTERRUPT, threshold.window & 0x07)
        self._write_u16(REG.THRESH_HIGH, threshold.high)
        self._write_u16(REG.THRESH_LOW, threshold.low)

    @property
    def signal_threshold(self) -> int:
        """Minimum signal rate in kcps below which the measurement is invalid."""
        return self._read_u16(REG.MIN_COUNT_RATE_RTN_LIMIT_MCPS) << 3

    @signal_threshold.setter
    def signal_threshold(self, signal_kcps: int) -> None:
        """Set the minimum valid signal rate in kcps."""
        self._write_u16(REG.MIN_COUNT_RATE_RTN_LIMIT_MCPS, signal_kcps >> 3)

    @property
    def std_dev_threshold(self) -> int:
        """Sigma threshold in mm above which the measurement is invalid."""
        return self._read_u16(REG.RANGE_CONFIG_STD_DEV_THRESHOLD) >> 2

    @std_dev_threshold.setter
    def std_dev_threshold(self, sigma_mm: int) -> None:
        """Set the sigma threshold in mm (max 16383)."""
        if sigma_mm > 0xFFFF >> 2:
            raise VL53L4CDError("sigma threshold too large")
        self._write_u16(REG.RANGE_CONFIG_STD_DEV_THRESHOLD, sigma_mm << 2)

    # ------------------------------------------------------------------
    # temperature
    # ------------------------------------------------------------------
    def start_temperature_update(self) -> None:
        """Re-run VHV calibration; call when ambient temperature changes by
        more than about 8 C (ST ``VL53L4CD_StartTemperatureUpdate``)."""
        self.dev.write(REG.VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x81)
        self.dev.write(0x000B, 0x92)

        self.dev.write(REG.SYSTEM_START, 0x40)
        try:
            self.wait_ready()
        finally:
            self.stop()

        self.dev.write(REG.VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09)
        self.dev.write(0x000B, 0x00)
