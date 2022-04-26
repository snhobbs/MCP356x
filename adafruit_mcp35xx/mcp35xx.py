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

from typing import Union, Optional, Tuple
from adafruit_bus_device.spi_device import SPIDevice
from . import objects

def write_register(spi, data: Union[bytes, bytearray, int],
                   register: objects.InternalRegisterAddress,
                   address: int) -> int:
    cmd = bytearray(5)
    datain = bytearray(5)
    cmd[0] = objects.make_command_byte(
        address=address,
        body=register,
        command=objects.CommandType.kIncrimentalWrite)

    if isinstance(data, int):
        data = bytes([data])

    for i, pt in enumerate(data):
        cmd[i+1] = pt

    size = len(data)
    spi.write_readinto(buffer_out=cmd, buffer_in=datain, out_end=size, in_end=size)
    return datain[0]


def read_register(spi, register: Union[objects.InternalRegister, objects.InternalRegisterAddress],
                  address: int) -> Tuple[int, bytearray]:
    if isinstance(register, objects.InternalRegisterAddress):
        register = objects.internal_registers[register]
    nbytes = register.nbytes + 1
    cmd = bytearray(nbytes)
    datain = bytearray(nbytes)
    cmd[0] = objects.make_command_byte(
        address=address,
        body=register.address,
        # command=objects.CommandType.kStaticRead)
        command=objects.CommandType.kIncrimentalRead)
    spi.write_readinto(buffer_out=cmd, buffer_in=datain)
    status = datain[0]
    return status, datain[1:]


def send_command(spi, command: objects.CommandOperationType,
                   address: int) -> int:
    cmd = bytearray(2)
    datain = bytearray(2)
    cmd[0] = objects.make_command_byte(
        address=address,
        body=command,
        command=objects.CommandType.kCommand)

    size = len(cmd)
    spi.write_readinto(buffer_out=cmd, buffer_in=datain, out_end=size, in_end=size)
    return datain[0]


class MCP35xx:
    _nbits = 24
    _internal_vref = 2.4

    def __init__(self, spi_bus, cs, ref_voltage=None, address=0x01):
        self._baudrate = int(1e6)
        if ref_voltage is None:
            ref_voltage = self._internal_vref
        self._spi_device = SPIDevice(spi_bus, cs, baudrate=self._baudrate)
        self._address = address
        self._ref_voltage = ref_voltage
        self._register_settings = None

    @property
    def register_settings(self) -> dict:
        return self._register_settings

    def setup(self, config: Optional[Union[dict, objects.ConfigurationTable]]) -> None:
        if isinstance(config, objects.ConfigurationTable):
            self._register_settings = config.register_settings
        elif config is None:
            self._register_settings = objects.ConfigurationTable().register_settings
        else:
            self._register_settings = config

        for key, value in self.register_settings.items():
            self.write_register(register=key, data=value)

    @property
    def bits(self) -> int:
        return self._nbits

    @property
    def reference_voltage(self) -> float:
        """Returns the MCP36xx's reference voltage. (read-only)"""
        return self._ref_voltage

    @property
    def data_ready(self) -> bool:
        status, irq = self.read_register(objects.InternalRegisterAddress.kIrq)
        return ((irq[0] >> 6)&0x1) == 0

    def read_register(self, register: Union[objects.InternalRegister, objects.InternalRegisterAddress]) -> Tuple[int, bytearray]:
        with self._spi_device as spi:
            status, data = read_register(spi, register, address=self._address)
        return status, data

    def write_register(self, data: Union[bytes, bytearray, int],
                   register: objects.InternalRegisterAddress) -> int:
        with self._spi_device as spi:
            status = write_register(spi, data, register, address=self._address)
        return status

    def send_command(self, command: objects.CommandOperationType) -> int:
        with self._spi_device as spi:
            status = send_command(spi, command, self._address)
        return status

    def set_mux_channel(self, mux_positive: objects.MuxChannel, mux_negative: objects.MuxChannel) -> int:
        setting = objects.pack_mux(
            vinp=mux_positive,
            vinn=mux_negative)

        status = self.write_register(setting, objects.InternalRegisterAddress.kMux)
        return status

    def read_adc_data(self, nbytes=4) -> Tuple[int, bytearray]:
        assert nbytes==4
        status, data = self.read_register(objects.InternalRegisterAddress.kAdcData)
        return status, data

    def read_status(self) -> int:
        status, _ = self.read_register(objects.InternalRegisterAddress.kConfig0)
        return status

    def read_all_registers(self) -> dict:
        '''Cycle through all registers and return dict of values.'''
        register_readings = dict()
        for register in objects.internal_registers.values():
            _, data = self.read_register(register)
            register_readings[register.address] = data
        return register_readings
