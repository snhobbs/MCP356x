'''
Read all the internal registers of the chip, print, and exit.
'''
from time import sleep
from typing import Union
import busio
import numpy as np
import digitalio
import board
from adafruit_mcp35xx import objects, mcp35xx, MCP35xx, tui
import rich
from rich.table import Table
from rich import console
from rich.console import Console
from rich.pretty import pprint


def generate_table(mcp) -> Table:
    """Make a new table."""
    register_readings = mcp.read_all_registers()
    return tui.generate_settings_table(register_readings)


def main():
    # create the spi bus
    spi_bus = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

    # create the cs (chip select)
    cs = digitalio.DigitalInOut(board.D21)
    cs.switch_to_output()

    mcp = MCP35xx(spi_bus=spi_bus, cs=cs, address=0x1)

    console = Console()
    console.print(generate_table(mcp))

if __name__ == "__main__":
    main()
