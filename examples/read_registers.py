'''
Read internal registers, print, and exit
'''
from time import sleep
from typing import Union
import numpy as np
import busio
import digitalio
import board
from adafruit_mcp35xx import objects, mcp35xx, MCP35xx, tui
import rich
from rich.console import Console
from rich.live import Live
from rich.table import Table


def generate_table(mcp) -> Table:
    """Make a new table."""""
    table = Table(title="MCP35xx Registers", box=rich.box.MINIMAL_DOUBLE_HEAD)
    table.add_column("Register")
    table.add_column("Reading")
    table.add_column("Status")

    register_readings = dict()
    for register in objects.internal_registers.values():
        row = [register.address.name]

        status, data = mcp.read_register(register)
        register_readings[register.address] = data

        row.append(str([hex(pt) for pt in data]))
        row.append(bin(status))
        table.add_row(*np.array(row, dtype=str))
    return table


def main():
    # create the spi bus
    spi_bus = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

    # create the cs (chip select)
    cs = digitalio.DigitalInOut(board.D21)
    cs.switch_to_output()

    mcp = MCP35xx(spi_bus=spi_bus, cs=cs, address=0x1)
    console = Console()
    register_readings = mcp.read_all_registers()
    console.print(tui.generate_settings_table(register_readings))

    table = generate_table(mcp)
    console.print(table)


if __name__ == "__main__":
    main()
