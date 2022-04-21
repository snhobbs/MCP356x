'''
Basic setup sequence to read one channel
Simplest setup is used.
No scan mode, single channel
All auto zero is disabled
No crc
No calibration
'''
from adafruit_mcp36xx import objects
from time import sleep
from typing import Union


def init(spi, address=0x1):
    '''
    Set each of the resisters
    '''

    data_registers = dict()
    data_registers[objects.InternalRegisterAddress.kConfig0] = objects.pack_config0(
        partial_shutdown=False,
        vref_sel=objects.VRefSelection.kInput,
        clk_sel=objects.ClockSelection.kInternal,
        cs_sel=objects.BurnoutCurrentSourceSetting.k0,
        adc_mode=objects.AdcOperatingMode.kConversion)

    data_registers[objects.InternalRegisterAddress.kConfig1] = objects.pack_config1(
        clk_prescale=0x3,  # 8
        osr=0b0111)  # 4096

    data_registers[objects.InternalRegisterAddress.kConfig2] = objects.pack_config2(
        boost=objects.BoostSetting.k1,
        gain=objects.AdcGainSetting.k1,
        az_mux=False,
        az_ref=False)

    data_registers[objects.InternalRegisterAddress.kConfig3] = objects.pack_config3(
        conv_mode=objects.ConversionMode.kOneShotStandby,
        data_format=objects.DataFormat.kSignExtended,
        crc_format=objects.CRCFormat.k16,
        enable_crc=False,
        enable_offsetcal=False,
        enable_gaincal=False)

    data_registers[objects.InternalRegisterAddress.kIrq] = objects.pack_irq(
        mdat_irq=True,
        irq_inactive_state=True,
        enable_fastcmd=True,
        enable_conversion_start_interrupt=True)

    #  Start off with the channel0 single ended
    data_registers[objects.InternalRegisterAddress.kMux] = objects.pack_mux(
        vinp=objects.MuxChannel.kSingleChannel0,
        vinn=objects.MuxChannel.kAGnd)

    # Scan mode set to 0 sets mux mode
    data_registers[objects.InternalRegisterAddress.kScan] = objects.pack_scan(
        delay=objects.InnerScanDelay,
        channels=[])

    data_registers[objects.InternalRegisterAddress.kTimer] = objects.pack_timer(1024)

    data_registers[objects.InternalRegisterAddress.kOffsetcal] = objects.pack_offsetcal(0x0)
    data_registers[objects.InternalRegisterAddress.kGaincal] = objects.pack_gaincal(0x800000) # Gain of 1, this is the default

    data_registers[objects.InternalRegisterAddress.kLock] = objects.pack_lock(0xA5)  # leave unlocked

    # Set each register
    def write_register(spi, data: Union[bytes, bytearray, int],
                       register: objects.InternalRegisterAddress,
                       address=0x1) -> None:
        cmd = bytearray(5)
        cmd[0] = objects.make_command_byte(
            address=address,
            body=register,
            command=objects.CommandType.kIncrimentalWrite)
        for i, pt in enumerate(data):
            cmd[i+1] = pt

        spi.write(cmd, end=len(data)+1)

    for key, value in data_registers.items():
        write_register(spi, address=address, data=value, register=key)


def print_registers(spi, address=0x1):
    for register in objects.InternalRegisterAddress:
        cmd = bytearray(4)
        datain = bytearray(4)
        cmd[0] = objects.make_command_byte(
            address=address,
            body=register,
            command=objects.CommandType.kIncrimentalRead)
        spi.write_readinto(cmd, datain)
        print(register, datain)


def reset(spi):
    # FIXME impliment
    pass


def main():
    with SPIDevice() as spi:
        sleep(1)
        reset(spi)
        sleep(1)
        init(spi)
        sleep(1)
        print_registers(spi)
