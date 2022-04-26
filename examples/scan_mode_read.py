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

ScanChannel = objects.ScanChannel
channels = [
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

def init(mcp):
    config = objects.ConfigurationTable()
    config["data_format"] = data_format
    config["vref_sel"] = objects.VRefSelection.kInternal
    config["boost"]=objects.BoostSetting.k2
    config["osr"]=0b1100
    config["clk_prescale"]=0x3

    config["scan_channels"] = channels
    config["data_format"] = data_format
    config["adc_mode"]=objects.AdcOperatingMode.kConversion
    config["conv_mode"]=objects.ConversionMode.kOneShotStandby


    mcp.setup(config)
    mcp.send_command(objects.CommandOperationType.kFullReset)
    mcp.setup(config)

loop = 0
start=time.time()
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

    mcp.send_command(objects.CommandOperationType.kConversionStart)
    last_channel = None
    run = True
    while run:
        while True:
            mode = mcp.operating_status
            print(mode)
            run = mode == objects.AdcOperatingMode.kConversion
            if mcp.data_ready or not run:
                print("1")
                break
            else:
                #print("0")
                pass

        status, data = mcp.read_adc_data()
        channel, reading = objects.read_adc_output(mode=data_format, data=data)

        channel = objects.ScanChannel(channel)
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

    mcp = MCP35xx(spi_bus=spi_bus, cs=cs, address=0x1)
    init(mcp)
    print("Setup Complete")
    console = Console()
    register_readings = mcp.read_all_registers()
    console.print(tui.generate_settings_table(register_readings))

    rate = 0.5
    with Live(generate_table(mcp), refresh_per_second=rate) as live:
        while True:
            sleep(0.9/rate)
            live.update(generate_table(mcp))


if __name__ == "__main__":
    main()
