'''
Basic setup sequence to read one channel
Simplest setup is used.
No scan mode, single channel
All auto zero is disabled
No crc
No calibration
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

    config = objects.ConfigurationTable()

    mcp = MCP35xx(spi_bus=spi_bus, cs=cs, address=0x1)
    mcp.send_command(objects.CommandOperationType.kFullReset)
    mcp.setup(config)
    print("Setup Complete")

    console = Console()
    console.print(generate_table(mcp))

    t = objects.internal_registers_to_configuration_table(config.register_settings)
    for key, value in t._settings.items():
        assert value == config[key]

if __name__ == "__main__":
    main()
