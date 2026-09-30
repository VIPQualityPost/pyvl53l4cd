# pyvl53l4cd

Python reference driver for the ST VL53L4CD Time-of-Flight ranging sensor,
ported from the ST VL53L4CD Ultra Lite Driver (ULD) API.

## Install

```sh
uv sync --extra ftdi       # pulls pyftdi for the FTDI transport
```

## Quickstart

```python
from pyvl53l4cd import VL53L4CD

sensor = VL53L4CD()             # defaults to an FTDI adapter on ftdi:///1
sensor.init_sensor()            # boot, load defaults, 50 ms timing

sensor.start()
try:
    sensor.wait_ready()
    result = sensor.get_result()
    print(f"{result.distance} mm (status {result.status})")
finally:
    sensor.stop()
```

## Configuration

```python
sensor.range_timing = RangeTiming(timing_budget=20, inter_measurement_time=0)
sensor.offset = 30                      # mm
sensor.crosstalk = 8                    # kcps, cover glass
sensor.signal_threshold = 1000          # kcps
sensor.std_dev_threshold = 90           # mm
sensor.detection_thresholds = Threshold(low=100, high=300, window=3)
```

## Transports

The driver talks to a `Transport` (see `pyvl53l4cd/transport.py`) exposing
`write(addr, val)`, `write_bytes(addr, data)` and `read(addr, count)` over
the 16-bit register map. `FtdiI2cTransport` implements it through pyftdi
(FTDI USB adapters); any other I2C bridge can be substituted.
