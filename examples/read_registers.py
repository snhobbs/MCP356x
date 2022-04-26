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
import numpy as np
import busio
import digitalio
import board
from adafruit_mcp35xx import objects, mcp35xx, MCP35xx, tui
import rich
from rich.console import Console
from rich.live import Live
from rich.table import Table


loop = 0
def generate_table(mcp) -> Table:
    """Make a new table."""""
    global loop
    loop = loop+1
    table = Table(title=str(loop), box=rich.box.MINIMAL_DOUBLE_HEAD)
    table.add_column("Register")
    table.add_column("Setting")
    table.add_column("Reading")
    table.add_column("Status")

    register_readings = dict()
    for register in objects.internal_registers.values():
        row = [register.address.name]
        setting = ""
        if register.address in mcp.register_settings:
            setting = mcp.register_settings[register.address]

        status, data = mcp.read_register(register)
        register_readings[register.address] = data

        row.append(str(np.array(setting)))
        row.append(str(np.array(data)))
        row.append(bin(status))
        table.add_row(*np.array(row, dtype=str))
    return table


def main():
    # create the spi bus
    spi_bus = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

    # create the cs (chip select)
    cs = digitalio.DigitalInOut(board.D21)
    cs.switch_to_output()

    config = objects.ConfigurationTable()
    config.vinp=objects.MuxChannel.kChannel0
    config.vinn=objects.MuxChannel.kAGnd

    mcp = MCP35xx(spi_bus=spi_bus, cs=cs, address=0x1)
    mcp.setup(config)
    print("Setup Complete")
    console = Console()
    register_readings = mcp.read_all_registers()
    console.print(tui.generate_settings_table(register_readings))

    with Live(generate_table(mcp), refresh_per_second=4) as live:
        while True:
            sleep(0.4)
            live.update(generate_table(mcp))


if __name__ == "__main__":
    main()
