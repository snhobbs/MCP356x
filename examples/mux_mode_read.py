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

data_format = objects.DataFormat.kSignExtended

def init(mcp):
    config = objects.ConfigurationTable()
    config["data_format"] = data_format
    config["vref_sel"] = objects.VRefSelection.kInternal
    config["boost"]=objects.BoostSetting.k2
    config["adc_mode"]=objects.AdcOperatingMode.kConversion
    config["osr"]=0b100
    config["clk_prescale"]=0x1

    mcp.setup(config)
    mcp.send_command(objects.CommandOperationType.kFullReset)
    mcp.setup(config)

loop = 0
def generate_table(mcp: MCP35xx) -> Table:
    """Make a new table."""""
    global loop
    loop = loop+1
    table = Table(title=str(loop), box=rich.box.MINIMAL_DOUBLE_HEAD)
    table.add_column("Channel Set+")
    table.add_column("Channel+")
    table.add_column("Channel-")
    table.add_column("Reading")
    table.add_column("Reading (lsb)")
    table.add_column("ADC Mode Start")
    table.add_column("ADC Mode End")
    table.add_column("Status")


    channels = [
        #objects.MuxChannel.kChannel0,
        #objects.MuxChannel.kChannel1,
        #objects.MuxChannel.kChannel2,
        #objects.MuxChannel.kChannel3,
        #objects.MuxChannel.kChannel4,
        #objects.MuxChannel.kChannel5,
        #objects.MuxChannel.kChannel6,
        #objects.MuxChannel.kChannel7,
        objects.MuxChannel.kAGnd,
        objects.MuxChannel.kAVdd,
        #objects.MuxChannel.kReserved ,
        objects.MuxChannel.kVref ,
        objects.MuxChannel.kVrefn ,
        objects.MuxChannel.kTempp ,
        objects.MuxChannel.kTempn ,
        objects.MuxChannel.kVcm ,
    ]
    for channel in channels:
        status = mcp.set_mux_channel(channel, objects.MuxChannel.kAGnd)

        _ , mux_data = mcp.read_register(objects.InternalRegisterAddress.kMux)
        muxp, muxn = objects.unpack_mux(mux_data)

        mcp.send_command(objects.CommandOperationType.kConversionStart)
        _, config0_start = mcp.read_register(objects.InternalRegisterAddress.kConfig0)

        while(not mcp.data_ready):  #  Wait for the data ready
            continue

        _ , data = mcp.read_adc_data()
        status, config0_end = mcp.read_register(objects.InternalRegisterAddress.kConfig0)

        config0_start = objects.unpack_config0(config0_start)
        config0_end = objects.unpack_config0(config0_end)
        _, reading = objects.read_adc_output(mode=data_format, data=data)

        row = (
            channel.name,
            muxp.name,
            muxn.name,
            reading*2.4/(1<<23),
            reading,
            #str([bin(pt) for pt in data]),
            (config0_start[-1].name),
            (config0_end[-1].name),
            bin(status)
        )
        config0_start = None
        config1_start = None
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

    rate = 0.25
    with Live(generate_table(mcp), refresh_per_second=rate) as live:
        while True:
            sleep(0.5/rate)
            print("update")
            live.update(generate_table(mcp))


if __name__ == "__main__":
    main()
