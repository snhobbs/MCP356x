# SPDX-FileCopyrightText: 2018 ladyada for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
:py:class:`~adafruit_mcp36xx.adafruit_mcp36xx.MCP36xx`
============================================================

CircuitPython Library for MCP36xx ADCs with SPI

* Author(s): ladyada, Brent Rubell

Implementation Notes
--------------------

**Hardware:**

* Adafruit `MCP3564 8-Channel 10-Bit ADC with SPI
  <https://www.adafruit.com/product/856>`_ (Product ID: 856)

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

.. note:: The ADC chips' input pins (AKA "channels") are aliased in this library
    as integer variables whose names start with "P" (eg ``MCP3564.P0`` is channel 0 on the MCP3568
    chip). Each module that contains a driver class for a particular ADC chip has these aliases
    predefined accordingly. This is done for code readability and prevention of erroneous SPI
    commands.

.. important::
    The differential reads (comparisons done by the ADC chip) are limited to certain pairs of
    channels. These predefined pairs are referenced in this documentation as differential
    channel mappings. Please refer to the driver class of your ADC chip (`MCP3564`_,
    `MCP3562`_, `MCP3561`_) for a list of available differential channel mappings.
"""

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_MCP36xx.git"

import math
from adafruit_bus_device.spi_device import SPIDevice
from enum import IntEnum


'''
Takes a bytearray from an adc channel reading
and return a signed int properly masked
'''
def buffer_to_nbit_int(buffer : bytearray, byteorder: str, signed: bool):
    return int.from_bytes(buffer, byteorder=byteorder, signed=signed)



class InternalRegister:
    def __init__(self, address: int, name: str, nbits: int, rw: str):
        self.address = address
        self.name = name
        self.nbits = nbits
        self.rw = rw

class Register(IntEnum):
    ADCDATA   = 0x00 #   // (it's complicated)  R
    CONFIG0   = 0x01 #   // 8-bit               RW
    CONFIG1   = 0x02 #   // 8-bit               RW
    CONFIG2   = 0x03 #   // 8-bit               RW
    CONFIG3   = 0x04 #   // 8-bit               RW
    IRQ       = 0x05 #   // 8-bit               RW
    MUX       = 0x06 #   // 8-bit               RW
    SCAN      = 0x07 #   // 24-bit              RW
    TIMER     = 0x08 #   // 24-bit              RW
    OFFSETCAL = 0x09 #   // 24-bit              RW
    GAINCAL   = 0x0A #   // 24-bit              RW
    RESERVED0 = 0x0B #   // 24-bit              RW
    RESERVED1 = 0x0C #   // 8-bit               RW
    LOCK      = 0x0D #   // 8-bit               RW
    RESERVED2 = 0x0E #  // 16-bit              RW
    CRCCFG    = 0x0F #   // 16-bit              R

class CommandType(IntEnum):
    kDefault = 0x00
    kStaticReadRegister = 0b01
    kIncrimentalWriteRegister = 0b10
    kIncrimentalReadRegister = 0b11

class Channel(IntEnum):
    SE_0   = 0x00
    SE_1   = 0x01
    SE_2   = 0x02
    SE_3   = 0x03
    SE_4   = 0x04
    SE_5   = 0x05
    SE_6   = 0x06
    SE_7   = 0x07
    DIFF_A = 0x08
    DIFF_B = 0x09
    DIFF_C = 0x0A
    DIFF_D = 0x0B
    TEMP   = 0x0C  # Internal temperature sensor.
    AVDD   = 0x0D
    VCM    = 0x0E
    OFFSET = 0x0F

class Mode(IntEnum):
    GAIN_ONETHIRD = 0
    GAIN_1        = 1
    GAIN_2        = 2
    GAIN_4        = 3
    GAIN_8        = 4
    GAIN_16       = 5
    GAIN_32       = 6
    GAIN_64       = 7

class BiasCurrent(IntEnum):
    NONE           = 0
    NANOAMPS_900   = 1
    NANOAMPS_3700  = 2
    NANOAMPS_15000 = 3

class BiasBoost(IntEnum):
    HALF      = 0
    TWOTHIRDS = 1
    ONE       = 2
    DOUBLE    = 3

class OversamplingRatio(IntEnum):
    OSR_32    = 0x00,
    OSR_64    = 0x01
    OSR_128   = 0x02
    OSR_256   = 0x03  # Default on reset.
    OSR_512   = 0x04
    OSR_1024  = 0x05
    OSR_2048  = 0x06
    OSR_4096  = 0x07
    OSR_8192  = 0x08
    OSR_16384 = 0x09
    OSR_20480 = 0x0A
    OSR_24576 = 0x0B
    OSR_40960 = 0x0C
    OSR_49152 = 0x0D
    OSR_81920 = 0x0E
    OSR_98304 = 0x0F

class AMCLKPrescale(IntEnum):
    OVER_1 = 0
    OVER_2 = 1
    OVER_4 = 2
    OVER_8 = 3

class MCP35xx:
    _nbits = 24
    _internal_vref = 2.4
    _transfer_bytes = int(math.log(_nbits, 2))
    _registers = (\
        InternalRegister(0, "ADCDATA", 32, "r"),
        InternalRegister(1, "CONFIG0", 8, "rw"),
        InternalRegister(2, "CONFIG1", 8, "rw"),
        InternalRegister(3, "CONFIG2", 8, "rw"),
        InternalRegister(4, "CONFIG3", 8, "rw"),
        InternalRegister(5, "IRQ", 8, "rw"),
        InternalRegister(6, "MUX", 8, "rw"),
        InternalRegister(7, "SCAN", 24, "rw"),
        InternalRegister(8, "TIMER", 24, "rw"),
        InternalRegister(9, "OFFSETCAL", 24, "rw"),
        InternalRegister(0xa, "GAINCAL", 24, "rw"),
        InternalRegister(0xb, "RESERVED", 24, "rw"),
        InternalRegister(0xc, "RESERVED", 8, "rw"),
        InternalRegister(0xd, "LOCK", 8, "rw"),
        InternalRegister(0xe, "RESERVED", 16, "rw"),
        InternalRegister(0xf, "CRCCFG", 16, "r"))

    def __init__(self, spi_bus, cs, ref_voltage=None):
        if ref_voltage is None:
            ref_voltage = self._internal_vref
        self._spi_device = SPIDevice(spi_bus, cs, baudrate=int(1e6))
        self._out_buf = bytearray(self._transfer_bytes)
        self._in_buf = bytearray(self._transfer_bytes)
        self._ref_voltage = ref_voltage

    @property
    def bits(self):
        return self._nbits

    @property
    def reference_voltage(self):
        """Returns the MCP36xx's reference voltage. (read-only)"""
        return self._ref_voltage

    def write_default_registers(self):
        with self._spi_device as spi:
            spi.write(self._out_buf, start=0, end=1)

    def read(self, pin, is_differential=False):
        """SPI Interface for MCP36xx-based ADCs reads. Due to 10-bit accuracy, the returned
        value ranges [0, 1023].

        :param int pin: individual or differential pin.
        :param bool is_differential: single-ended or differential read.

        .. note:: This library offers a helper class called `AnalogIn`_ for both single-ended
            and differential reads. If you opt to not implement `AnalogIn`_ during differential
            reads, then the ``pin`` parameter should be the first of the two pins associated with
            the desired differential channel mapping.
        """
        assert(0)
        self._out_buf[1] = ((not is_differential) << 7) | (pin << 4)
        with self._spi_device as spi:
            # pylint: disable=no-member
            spi.write_readinto(self._out_buf, self._in_buf)

        return buffer_to_nbit_int(buffer=self._in_buf, byteorder="big", signed=True)
