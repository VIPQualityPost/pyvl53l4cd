class FtdiI2cTransport:
    """I2C transport for a VL53L4CD through an FTDI adapter, via pyftdi.

    The VL53L4CD has a 16-bit register map, so every transaction is a write
    of the two address bytes followed by the data. ``device_addr`` is the
    7-bit I2C address; the sensor's reset default is 0x29 (0x52 in 8-bit
    form as used by the ST ULD).
    """

    def __init__(
        self,
        path: str = "ftdi:///1",
        freq: float = 400e3,
        device_addr: int = 0x29,
    ) -> None:
        from pyftdi.i2c import I2cController

        self.device_addr = device_addr
        self.ftdi = I2cController()
        self.ftdi.configure(path, frequency=freq)

    def _register(self, addr: int) -> bytes:
        return addr.to_bytes(2, "big")

    def write(self, addr: int, val: int) -> None:
        self.ftdi.write(self.device_addr, self._register(addr) + bytes([val & 0xFF]))

    def write_bytes(self, addr: int, data: bytes) -> None:
        self.ftdi.write(self.device_addr, self._register(addr) + bytes(data))

    def read(self, addr: int, count: int) -> bytes:
        self.ftdi.write(self.device_addr, self._register(addr))
        return self.ftdi.read(self.device_addr, count)
