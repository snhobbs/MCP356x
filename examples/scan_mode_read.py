'''
Basic setup sequence to read one channel
Simplest setup is used.
No scan mode, single channel
All auto zero is disabled
No crc
No calibration
'''
from time import sleep
import time
from typing import Union
import busio
import digitalio
import board
import numpy as np
from adafruit_mcp35xx import objects, MCP35xx, tui
import rich
from rich.live import Live
from rich.table import Table
from rich import console
from rich.console import Console
from rich.pretty import pprint

data_format = objects.DataFormat.kChannelPlusSignExtended
loop = 0
def generate_table(mcp: MCP35xx) -> Table:
    """Make a new table."""""
    global loop
    loop = loop+1
    table = Table(title=str(loop), box=rich.box.MINIMAL_DOUBLE_HEAD)
    table.add_column("Scan Channel")
    table.add_column("Reading")
    table.add_column("Reading (lsb)")
    table.add_column("Reading (Bin)")
    table.add_column("Status")


    #  Run through single scan cycle

    start = time.time()  #  call it over after 2 seconds
    mcp.send_command(objects.CommandOperationType.kConversionStart)
    last_channel = None
    run = True
    while run:
        while True:
            if mcp.data_ready or not run:
                break

            _, config0 = mcp.read_register(objects.InternalRegisterAddress.kConfig0)
            if objects.unpack_config0(config0)[-1] != objects.AdcOperatingMode.kConversion:
                run = False

        _ , data = mcp.read_adc_data()
        status, config0_end = mcp.read_register(objects.InternalRegisterAddress.kConfig0)

        channel_int, reading = objects.read_adc_output(mode=data_format, data=data)
        channel = objects.ScanChannel(channel_int)
        if channel != last_channel:
            last_channel = channel
        else:
            continue

        row = (
            channel.name,
            reading*2.4/(1<<23),
            reading,
            str([bin(pt) for pt in data]),
            bin(status),
        )
        table.add_row(*np.array(row, dtype=str))
    return table


def main():
    # create the spi bus
    spi_bus = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

    # create the cs (chip select)
    cs = digitalio.DigitalInOut(board.D21)
    cs.switch_to_output()
    #spi_device = SPIDevice(spi_bus, chip_select=cs)

    config = objects.ConfigurationTable()
    ScanChannel = objects.ScanChannel
    config["scan_channels"] = [
        ScanChannel.kSingleChannel0,
        ScanChannel.kSingleChannel1,
        #ScanChannel.kSingleChannel2,
        #ScanChannel.kSingleChannel3,
        #ScanChannel.kSingleChannel4,
        #ScanChannel.kSingleChannel5,
        #ScanChannel.kSingleChannel6,
        #ScanChannel.kSingleChannel7,
        ScanChannel.kDifferentialChannelA,
        #ScanChannel.kDifferentialChannelB,
        #ScanChannel.kDifferentialChannelC,
        #ScanChannel.kDifferentialChannelD,
        ScanChannel.kTemp,
        ScanChannel.kVdd,
        ScanChannel.kVcm,
        ScanChannel.kOffset]
    config["data_format"] = data_format

    mcp = MCP35xx(spi_bus=spi_bus, cs=cs, address=0x1)
    mcp.setup(config)
    mcp.send_command(objects.CommandOperationType.kFullReset)
    mcp.setup(config)
    print("Setup Complete")
    console = Console()
    register_readings = mcp.read_all_registers()
    console.print(tui.generate_settings_table(register_readings))

    rate = 1
    with Live(generate_table(mcp), refresh_per_second=rate) as live:
        while True:
            sleep(0.9/rate)
            live.update(generate_table(mcp))


if __name__ == "__main__":
    main()
