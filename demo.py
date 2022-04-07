# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import busio
import digitalio
import board
from adafruit_bus_device.spi_device import SPIDevice
from time import sleep
import adafruit_mcp35xx.mcp35xx as MCP
from adafruit_mcp35xx import CommandType, Register
# create the spi bus
spi_bus = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D4)
spi_device = SPIDevice(spi_bus, cs, baudrate=int(1e6))

out_buf = bytearray(4)
in_buf = bytearray(4)

regs = dict(((reg.name, reg) for reg in MCP.MCP35xx._registers))
settings = {
    "CONFIG0": (0b1<<7) | (0b1 << 6) | (0b11<<4) | (0b00 << 2) | (0b11 << 0),
    "CONFIG1": (0b00<<6) | (0b0000<<2),
    "CONFIG2": (0b10<<6) | (0b001<<3) | (0b0 << 2) | (0b0 << 1) | (0b1 << 0),
    "CONFIG3": (0b11<<6) | (0b11<<4) | (0b0 << 3) | (0b0 << 2) | (0b0 << 1) | (0b0 << 0),
}

def make_command_byte(device_address: int, command_bits: int, command_type: CommandType):
    byte = ((device_address & 0x3) << 6) | ((command_bits & 0xf) << 2) | ((command_type & 0x3) << 0)
    print(bin(byte))
    return byte

device_address = 0b01
loop = True
while loop:
    with spi_device as spi:
        spi.configure(baudrate=int(1e6), polarity=False, phase=False)
        #  write command
        for key, value in settings.items():
            print(key, bin(value))
            out_buf[0] = make_command_byte(device_address, regs[key].address, CommandType.kIncrimentalWriteRegister)
            out_buf[1] = value
            spi.write_readinto(out_buf, in_buf, out_end=2, in_end=2)
            sleep(0.1)
            print(value)
        loop = False

        while True:
            sleep(0.1)
            out_buf[0] = make_command_byte(device_address, 0x00, CommandType.kIncrimentalReadRegister)
            out_buf[1] = 0
            spi.write_readinto(buffer_out=out_buf, buffer_in=in_buf, out_end=4, in_end=4)
            print(in_buf, [hex(p) for p in out_buf])
