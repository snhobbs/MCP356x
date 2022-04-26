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
from adafruit_bus_device.spi_device import SPIDevice


if __name__ == "__main__":
    spi_bus = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
    spi_bus.try_lock()
    spi_bus.configure(baudrate=int(1e5))
    cs = digitalio.DigitalInOut(board.D21)
    cs.switch_to_output()

    #with spi_bus as spi:
    #    spi.try_lock()
    #    spi.configure(baudrate=int(1e6))

    spi_device = SPIDevice(spi_bus, chip_select=cs)
    # write_register(spi, address=address, data=value, register=key)
    while(1):
        outbuff = bytearray(1)
        inbuff = bytearray(1)
        read_config0 = 0b00000101  # read config0
        try:
            #with spi_device as spi:
            if True:
                spi = spi_bus
                outbuff[0] = read_config0
                address = 0x01
                outbuff[0] |= (address << 6)
                cs.value = 0
                sleep(25e-6)
                spi.write_readinto(outbuff, inbuff)
                print((outbuff[0] >>6), outbuff, inbuff)
                sleep(25e-6)
                cs.value = 1
                sleep(0.1)
        except AssertionError as e:
            raise
