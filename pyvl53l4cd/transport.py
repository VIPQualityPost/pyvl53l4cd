from typing import Protocol


class Transport(Protocol):
    """Register-level transport for a VL53L4CD device.

    Addresses are 16-bit register addresses (see ``VL53L4CDAddress``).
    """

    def write(self, addr: int, val: int) -> None:
        """Write a single byte to register ``addr``."""
        ...

    def write_bytes(self, addr: int, data: bytes) -> None:
        """Write consecutive bytes starting at register ``addr``."""
        ...

    def read(self, addr: int, count: int) -> bytes:
        """Read ``count`` consecutive bytes starting at register ``addr``."""
        ...
