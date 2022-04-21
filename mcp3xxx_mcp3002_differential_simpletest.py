# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import busio
import digitalio
import board
import adafruit_mcp35xx.mcp3561 as MCP
from adafruit_mcp35xx.analog_in import AnalogIn

# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D4)

# create the mcp object
mcp = MCP.MCP3561(spi, cs)

# create a differential ADC channel between Pin 0 and Pin 1
chan = AnalogIn(mcp, MCP.P0)#, MCP.P1)
from time import sleep
while True:
    print("Differential ADC Value: ", chan.value)
    sleep(0.1)
    #print("Differential ADC Voltage: " + str(chan.voltage) + "V")
