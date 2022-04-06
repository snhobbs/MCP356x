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
    as integer variables whose names start with "P" (eg ``MCP3564.P0`` is channel 0 on the MCP3008
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

def buffer_to_nbit_int(buffer, byteorder, signed):
    return int.from_bytes(buffer, byteorder, signed)

class InternalRegister:
    def __init__(self, address: int, name: str, nbits: int, rw: str):
        self.address = address
        self.name = name
        self.nbits = nbits
        self.rw = rw


class MCP36xx:
    """
    This abstract base class is meant to be inherited by `MCP3564`_, `MCP3562`_,
    or `MCP3561`_ child classes.

    :param ~adafruit_bus_device.spi_device.SPIDevice spi_bus: SPI bus the ADC is connected to.
    :param ~digitalio.DigitalInOut cs: Chip Select Pin.
    :param float ref_voltage: Voltage into (Vin) the ADC.
    """
    _nbits = 24
    _internal_vref = 2.4
    _transfer_bytes = int(math.log(_nbits, 2))
    _registers = (
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
        InternalRegister(0xf, "CRCCFG", 16, "r"),
    )

    def __init__(self, spi_bus, cs, ref_voltage=self._internal_vref):
        self._spi_device = SPIDevice(spi_bus, cs)
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
        self._out_buf[1] = ((not is_differential) << 7) | (pin << 4)
        with self._spi_device as spi:
            # pylint: disable=no-member
            spi.write_readinto(self._out_buf, self._in_buf)

        return buffer_to_nbit_int(buffer=self._in_buf, byteorder="big", signed=True)
