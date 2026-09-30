def CfgBytes(
    fast_mode: bool = True,
    pullup_1v8: bool = True,
    active_high_irq: bool = True,
    sigma_threshold: int = 0x00A0,
    min_rate_count: int = 0x0080,
    intermeasurement_period: int = 0x00000F89,
    distance_threshold: int = 0x00000000,
    clear_interrupt: bool = False,
    start_ranging: bool = False,
) -> list[int]:
    """Build the 91-byte default configuration block written to registers
    0x2D..0x87 by ``VL53L4CD_LoadDefaultConfig()``.

    The flash values are the ST VL53L4CD ULD default configuration; only the
    user-visible fields are parameterized.
    """
    fast_mode_byte = 0x12 if fast_mode else 0x00
    # 0x2E/0x2F bit 0: 0 = I2C/GPIO lines pulled up at 1.8 V (ST default),
    # 1 = pulled up at AVDD (2.8-3.5 V rail)
    pullup_byte = 0x00 if pullup_1v8 else 0x01
    # 0x30: bit 4 = 0 for active-high interrupt, 1 for active-low (bits 3:0 must be 0x1)
    interrupt_byte = 0x01 if active_high_irq else 0x11

    distance_high = (distance_threshold >> 16) & 0xFFFF
    distance_low = distance_threshold & 0xFFFF

    return [
        fast_mode_byte,        # 0x2D : fast plus mode (1 MHz I2C)
        pullup_byte,           # 0x2E : I2C pull-up
        pullup_byte,           # 0x2F : GPIO pull-up
        interrupt_byte,        # 0x30 : GPIO interrupt polarity
        0x02,                  # 0x31 : interrupt depending on polarity, use CheckForDataReady()
        0x00, 0x02, 0x08, 0x00, 0x08, 0x10,
        0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
        0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00,
        0x00, 0x00,
        0x20,                  # 0x46 : interrupt on new sample ready
        0x0B, 0x00, 0x00, 0x02, 0x14, 0x21,
        0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
        0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF,
        0x01, 0x00, 0x08, 0x00, 0x00, 0x01,
        0xCC, 0x07, 0x01, 0xF1, 0x05,
        (sigma_threshold >> 8) & 0xFF,      # 0x64 : sigma threshold MSB (14.2 format)
        sigma_threshold & 0xFF,             # 0x65 : sigma threshold LSB
        (min_rate_count >> 8) & 0xFF,       # 0x66 : min count rate MSB (9.7 format)
        min_rate_count & 0xFF,              # 0x67 : min count rate LSB
        0x08,
        0x38,
        0x00,
        0x00,
        (intermeasurement_period >> 24) & 0xFF,  # 0x6C : intermeasurement period (32 bits)
        (intermeasurement_period >> 16) & 0xFF,  # 0x6D
        (intermeasurement_period >> 8) & 0xFF,   # 0x6E
        intermeasurement_period & 0xFF,          # 0x6F
        0x00,
        0x00,
        (distance_high >> 8) & 0xFF,        # 0x72 : distance threshold high MSB
        distance_high & 0xFF,               # 0x73 : distance threshold high LSB
        (distance_low >> 8) & 0xFF,         # 0x74 : distance threshold low MSB
        distance_low & 0xFF,                # 0x75 : distance threshold low LSB
        0x00,
        0x01,
        0x07,
        0x05,
        0x06,
        0x06,
        0x00,
        0x00,
        0x02,
        0xC7,
        0xFF,
        0x9B,
        0x00,
        0x00,
        0x00,
        0x01,
        0x01 if clear_interrupt else 0x00,  # 0x86 : clear interrupt
        0x40 if start_ranging else 0x00,    # 0x87 : start ranging
    ]
