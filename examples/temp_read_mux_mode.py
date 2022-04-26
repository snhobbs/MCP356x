'''
Read the temperature channel.
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

#data_format = objects.DataFormat.kRightPaddedZero
#data_format = objects.DataFormat.kSignExtended
#data_format = objects.DataFormat.kDataSign
data_format = objects.DataFormat.kChannelPlusSignExtended

loop = 0
def generate_table(mcp: MCP35xx) -> Table:
    """Make a new table."""""
    global loop
    loop = loop+1
    table = Table(title=str(loop), box=rich.box.MINIMAL_DOUBLE_HEAD)
    table.add_column("Channel+")
    table.add_column("Channel-")
    table.add_column("Reading (LSB)")
    table.add_column("Reading (V)")
    table.add_column("Temperature")
    table.add_column("Bin")
    table.add_column("Status")


    status = mcp.set_mux_channel(objects.MuxChannel.kTempp, objects.MuxChannel.kTempn)

    _ , mux_data = mcp.read_register(objects.InternalRegisterAddress.kMux)
    muxp, muxn = objects.unpack_mux(mux_data)
    assert mux_data[0] == 0xde

    mcp.send_command(objects.CommandOperationType.kConversionStart)
    _, config0_start = mcp.read_register(objects.InternalRegisterAddress.kConfig0)

    while((mcp.read_status()) & (1<<2)):  #  Wait for the data ready
        continue

    _ , data = mcp.read_adc_data()
    status, config0_end = mcp.read_register(objects.InternalRegisterAddress.kConfig0)

    config0_start = objects.unpack_config0(config0_start)
    config0_end = objects.unpack_config0(config0_end)
    assert config0_end[-1] == objects.AdcOperatingMode.kStandby
    _, reading = objects.read_adc_output(mode=data_format, data=data)

    vref = 2.4
    temperature = objects.first_order_temperature_sensor_transfer_to_c(reading, vref=vref)

    v = reading*vref/(1<<23)
    temperature = objects.first_order_temperature_sensor_transfer_v_to_c(vin=v, vref=vref)
    row = (
        muxp.name,
        muxn.name,
        reading,
        v,
        temperature,
        str([bin(pt) for pt in data]),
        bin(status)
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
    config["data_format"] = data_format

    mcp = MCP35xx(spi_bus=spi_bus, cs=cs, address=0x1)
    mcp.send_command(objects.CommandOperationType.kFullReset)
    mcp.setup(config)
    mcp.send_command(objects.CommandOperationType.kFullReset)
    mcp.setup(config)
    print("Setup Complete")
    console = Console()
    register_readings = mcp.read_all_registers()
    console.print(tui.generate_settings_table(register_readings))

    rate = 10  # hz
    with Live(generate_table(mcp), refresh_per_second=rate) as live:
        while True:
            sleep(0.9/rate)
            live.update(generate_table(mcp))


if __name__ == "__main__":
    main()
