'''
Write config1, read back from it, confirm the change has occured
'''

from adafruit_mcp35xx import objects, mcp35xx, MCP35xx
from typing import Union
import busio
import digitalio
import board


def main():
    # create the spi bus
    spi_bus = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

    # create the cs (chip select)
    cs = digitalio.DigitalInOut(board.D21)
    cs.switch_to_output()

    clk_prescale = 0x3
    osr = 0b0111

    value = objects.pack_config1(
        clk_prescale=clk_prescale,  # 8
        osr=osr)  # 4096

    mcp = MCP35xx(spi_bus=spi_bus, cs=cs, address=0x1)
    mcp.write_register(data=value, register=objects.InternalRegisterAddress.kConfig1)
    status, reading = mcp.read_register(register=objects.internal_registers[objects.InternalRegisterAddress.kConfig1])
    clk_prescale_read, osr_read = objects.unpack_config1(reading)
    print(f"Preset Set: {clk_prescale}")
    print(f"Preset Read: {clk_prescale_read}")
    print(f"OSR Set: {osr}")
    print(f"OSR Read: {osr_read}")
    if osr == osr_read and clk_prescale == clk_prescale_read:
        print("SUCCESS")
    else:
        print("Settings do not match")


if __name__ == "__main__":
    main()
