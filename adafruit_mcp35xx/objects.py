'''
Model of an MCP356x
This should have all the internal state and respond as the chip would according to the data sheet.
The inputs can be set as an externally updated object. The interface should also be seperately modeled.
Interacting can then be done like a model + view + controller.
'''

import enum
from typing import Union, List
from functools import partial


class InputMuxSetting(enum.IntEnum):
    '''Table 5-1
    page 38.'''
    kCH0 = 0
    kCH1 = 1
    kCH2 = 2
    kCH3 = 3
    kCH4 = 4
    kCH5 = 5
    kCH6 = 6
    kCH7 = 7
    kAGnd = 8
    kAVdd = 9
    kReserved = 10
    kRefP = 11
    kRefM = 12
    kTempP = 13
    kTempM = 14
    kVCm = 15


class BurnoutCurrentSourceSetting(enum.IntEnum):
    '''
    Table 5-2
    page 39.'''
    k0 = 0
    k0_9u = 1
    k3_7u = 2
    k15u = 3


def first_order_temperature_sensor_transfer_to_c(lsb: int, vref: float) -> float:
    '''Eq. 5-1
    page 40.
    gain=1, mclk=4.9152MHz.'''
    return (4.0096e-4*lsb*vref)-269.13


def first_order_temperature_sensor_transfer_to_vin(temp: float) -> float:
    '''Eq. 5-1
    page 40.
    gain=1, mclk=4.9152MHz.'''
    return (temp*0.2973+80)/1000


'''
ADC Offset Cancellation
EQ 5-2
page 40
FIXME ADD
'''


class AdcGainSetting(enum.IntEnum):
    '''Table 5-3
    page 41.'''
    k0_333 = 0
    k1 = 1
    k2 = 2
    k4 = 3
    k8 = 4
    k16 = 5
    k32 = 6
    k64 = 7


'''
table 5-4
page 43
Modulator mode
FIXME add
'''


class BoostSetting(enum.IntEnum):
    '''Boost Modes
    table 5-5
    page 43.'''
    k0_5 = 0
    k0_66 = 1
    k1 = 2
    k2 = 3


'''
Digital decimation filter transfer function
EQ 5-3
page 44
FIXME add and look into
'''


def get_conversion_time(osr1: int, osr3: int, dmclk_freq: int) -> float:
    '''Conversion time
    EQ 5-4
    page 44.'''
    return (3*osr3 + (osr1-1)*osr3)/dmclk_freq


def get_osr3(osr: int) -> int:
    '''Oversampling ratio
    table 5-6
    page 46.'''
    assert osr <= 0xf
    if osr < 5:
        return 1 << (osr+5)
    return 512


def get_osr1(osr: int) -> int:
    '''Oversampling ratio
    table 5-6
    page 46.'''
    assert osr <= 0xf
    assert osr <= 0xf
    lookup_table = {
        0: 1,
        1: 1,
        2: 1,
        3: 1,
        4: 1,
        5: 2,
        6: 4,
        7: 8,
        8: 16,
        9: 32,
        10: 40,
        11: 48,
        12: 80,
        13: 96,
        14: 160,
        15: 192,
    }
    return lookup_table[osr]

def calc_lsb_for_dc_input(vinp: float, vinn: float, vrefp: float, vrefn : float = 0, gain: float = 1) -> int:
    '''
    EQ 5-5
    page 48.'''
    lsb_float = (vinp-vinn)/(vrefp-vrefn)*8388608*gain
    return int(round(lsb_float))


class DataFormat(enum.IntEnum):
    '''Adc output format
    figure 5-8
    page 48.'''
    kDataSign = 0
    kLeftZeroPaddedDataSign = 1
    kSignExtended = 2
    kChannelPlusSignExtended = 3

def format_adc_output(mode: DataFormat, data: int, channel: int = 0) -> int:
    sign = int(data < 0)
    out = None
    if mode == DataFormat.kDataSign:
        dout = abs(data) | (sign<<23)
        out = dout.to_bytes(length=3, byteorder="little")
    elif mode == DataFormat.kLeftZeroPaddedDataSign:
        dout = (abs(data) | (sign<<23)) << 8
        out = dout.to_bytes(length=4, byteorder="little")
    elif mode == DataFormat.kSignExtended:
        out = (data<<8).to_bytes(length=4, byteorder="little")
    elif mode == DataFormat.kChannelPlusSignExtended:
        raise NotImplementedError
    return out


class VRefSelection(enum.IntEnum):
    '''Table 5-9
    page 50.'''
    kOutput = 0
    kInput = 1


class AdcOperatingMode(enum.IntEnum):
    '''Table 5-10
    page 53.'''
    kShutdown = 0
    kShutdownB = 1
    kStandby = 2
    kConversion = 3


class ClockSelection(enum.IntEnum):
    '''Table 5-13
    page 56.'''
    kExternal = 0
    kExternalB = 1
    kInternal = 2
    kInternalWithClockOutput = 3


def calc_lsb_post_calibration(lsb, offset, gaincal):
    '''EQ 5-6
    page 57.'''
    return (lsb+offset)*gaincal


def calc_offset_calibration_v(vref, offset, gain):
    '''EQ 5-7
    page 57.'''
    return vref*offset/(8388608*gain)


def calc_gaincal_v(gaincal):
    '''EQ 5-8
    page 58.'''
    return gaincal/8388608


class ConversionMode(enum.IntEnum):
    '''Table 5-14
    page 59.'''
    kOneShotShutdown = 0
    kOneShotStandby = 2
    kContinuous = 3


class ScanChannel(enum.IntEnum):
    '''Table 5-15
    page 62.
    Scan mode is enabled when the scan register is not 0x00.
    '''
    kSingleChannel0 = 0
    kSingleChannel1 = 1
    kSingleChannel2 = 2
    kSingleChannel3 = 3
    kSingleChannel4 = 4
    kSingleChannel5 = 5
    kSingleChannel6 = 6
    kSingleChannel7 = 7
    kDifferentialChannelA = 8
    kDifferentialChannelB = 9
    kDifferentialChannelC = 10
    kDifferentialChannelD = 11
    kTemp = 12
    kVdd = 13
    kVcm = 14
    kOffset = 15


class MuxChannel(enum.IntEnum):
    '''Table 5-15
    page 62.
    Note that any mux combination can be set, the only
    restriction is what the scan sets. When in scan mode all
    settings of the MUX register are DNC.'''
    kChannel0 = 0
    kChannel1 = 1
    kChannel2 = 2
    kChannel3 = 3
    kChannel4 = 4
    kChannel5 = 5
    kChannel6 = 6
    kChannel7 = 7
    kAGnd = 8
    kAVdd = 9
    kReserved = 10
    kVref = 11
    kVrefn = 12
    kTempp = 13
    kTempn = 14
    kVcm = 15


class InnerScanDelay(enum.IntEnum):
    '''Table 5-16
    page 63.'''
    k0 = 0
    k8 = 1
    k16 = 2
    k32 = 3
    k64 = 4
    k128 = 5
    k256 = 6
    k512 = 7


'''
Delay between scans
table 5-17
page 64
'''

class CommandType(enum.IntEnum):
    '''CommandType
    table 6-2
    page 68.'''
    kCommand = 0
    kStaticRead = 1
    kIncrimentalWrite = 2
    kIncrimentalRead = 3


class CommandOperationType(enum.IntEnum):
    '''Table 6-2
    page 68.'''
    kDNC = 0
    kConversionStart = 0b1010
    kFastStandby = 0b1011
    kFastShutdown = 0b1100
    kFullShutdown = 0b1101
    kFastReset = 0b1101


def read_status_byte(value: int) -> tuple:
    '''Read status byte
    Figure 6-1
    page 69.'''
    address = (value & 0xc0)>>3
    irq = (value & 0x7)
    return address, irq


class CRCFormat(enum.IntEnum):
    '''CRC Setting
Figure 6-10
page 77.'''

    k16 = 0
    k32 = 1  # still a crc16, just padded to 32 bits


class InternalRegisterAddress(enum.IntEnum):
    '''Internal Registers
table 8-1
page 89.'''

    kAdcData = 0
    kConfig0 = 1
    kConfig1 = 2
    kConfig2 = 3
    kConfig3 = 4
    kIrq = 5
    kMux = 6
    kScan = 7
    kTimer = 8
    kOffsetcal = 9
    kGaincal = 10
    # kReserved0 = 11
    # kReserved1 = 12
    kLock = 13
    # kReserved2 = 14
    kCrccfg = 15


def make_command_byte(address: int, body: Union[CommandOperationType, InternalRegisterAddress], command: CommandType) -> int:
    '''Command byte
    table 6-1
    page 67.'''
    value = (address & 0b11)<<6 | (int(body)&0b1111)<<2 | (int(command)&0b11)
    assert value <= 0xff
    return value


def pack_config0(partial_shutdown: bool, vref_sel: VRefSelection, clk_sel: ClockSelection, cs_sel: BurnoutCurrentSourceSetting, adc_mode: AdcOperatingMode) -> int:
    return int(partial_shutdown) << 7 |\
        int(vref_sel) << 6 | int(clk_sel) << 4 |\
        int(cs_sel) << 2 | int(adc_mode)


def unpack_config0(value) -> tuple:
    partial_shutdown = bool(value & 0x7)
    vref_sel = VRefSelection((value >> 6) & 0x1)
    clk_sel = ClockSelection((value >> 4) & 0x3)
    cs_sel = BurnoutCurrentSourceSetting((value >> 2) & 0x3)
    adc_mode = AdcOperatingMode(value & 0x3)
    return partial_shutdown, vref_sel, clk_sel, cs_sel, adc_mode


def pack_config1(clk_prescale: int, osr: int) -> int:
    assert clk_prescale < 0x3
    assert clk_prescale >= 0
    assert osr < 0xf
    assert osr >= 0
    return (clk_prescale << 6) | (osr << 2)


def unpack_config1(value) -> tuple:
    clk_prescale = (value >> 6)&0x3
    osr = (value >> 2) & 0xf
    return clk_prescale, osr


def pack_config2(boost: BoostSetting, gain: AdcGainSetting, az_mux: bool, az_ref: bool) -> int:
    return int(boost)<<6 | int(gain) << 3 | int(az_mux)<<2 | int(az_ref) << 1


def unpack_config2(value) -> tuple:
    boost = BoostSetting((value>>6)&0x3)
    gain = AdcGainSetting((value>>3)&0b111)
    az_mux = (value >> 2)&0x1
    az_ref = (value >> 1)&0x1
    return boost, gain, az_mux, az_ref


def pack_config3(conv_mode: ConversionMode, data_format: DataFormat, crc_format: CRCFormat, enable_crc: bool, enable_offsetcal: bool, enable_gaincal: bool) -> int:
    return int(conv_mode) << 6 | int(data_format) << 4 | int(crc_format) << 3 | int(enable_crc) << 2 | int(enable_offsetcal) << 1 | int(enable_gaincal)


def unpack_config3(value) -> tuple:
    conv_mode = ConversionMode((value >> 6)&0x3)
    data_format = DataFormat((value >> 4)&0x3)
    crc_format = CRCFormat((value >> 3)&0x1)
    enable_crc = (value >> 2)&0x1 == 1
    enable_offsetcal = (value >> 1)&0x1 == 1
    enable_gaincal = (value&0x1) == 1
    return conv_mode, data_format, crc_format, enable_crc, enable_offsetcal, enable_gaincal


def pack_irq(mdat_irq: bool, irq_inactive_state: bool, enable_fastcmd: bool, enable_conversion_start_interrupt: bool) -> int:
    return int(mdat_irq) << 3 |\
        int(irq_inactive_state) << 2 | int(enable_fastcmd) << 1 |\
        int(enable_conversion_start_interrupt)


def unpack_irq(value) -> tuple:
    data_ready = (value >> 6)&0x1 == 1
    crccfg_status = (value >> 5)&0x1 == 1
    por_status = (value >> 4)&0x1 == 1
    mdat_irq = (value >> 3)&0x1 == 1
    irq_inactive_state = (value>>2)&0x1 == 1
    enable_fastcmd = (value >> 1)&0x1 == 1
    enable_conversion_start_interrupt = (value)&0x1 == 1
    return data_ready, crccfg_status, por_status, mdat_irq, irq_inactive_state,\
        enable_fastcmd, enable_conversion_start_interrupt


# register 8-7
# page 96


def pack_mux(vinp: MuxChannel, vinn: MuxChannel) -> int:
    return int(vinp) << 4 | int(vinn)


def unpack_mux(value: int) -> tuple:
    vinp = MuxChannel(value >> 4)
    vinn = MuxChannel(value&0xf)
    return vinp, vinn


# Scan register
# page 97


def pack_scan(delay: InnerScanDelay, channels: List[MuxChannel]) -> bytearray:
    channel_int = 0
    for channel in channels:
        channel_int |= (1<<channel)
    return bytearray([int(delay)<<5, int(channel_int)>>8, int(channel_int)&0xff])


def unpack_scan(datain: Union[bytes, bytearray]) -> tuple:
    return datain[0] >> 5, (datain[1] << 8 | datain[2])


def pack_int(value: int, nbytes: int) -> bytearray:
    '''Packs an integer of nbytes length into bytearray.
Used by timer, offsetcal, gaincal, lock, reserved3, crccfg register.'''
    array = int(value).to_bytes(length=nbytes, byteorder="big")
    return bytearray(array)


def unpack_int(datain: Union[bytes, bytearray]) -> tuple:
    '''Packs an integer of nbytes length into bytearray.
Used by timer, offsetcal, gaincal, lock, reserved3, crccfg register.'''

    value = int.from_bytes(datain, byteorder="big")
    return (value,)

pack_timer = partial(pack_int, nbytes=3)
unpack_timer = unpack_int

pack_offsetcal = partial(pack_int, nbytes=3)
unpack_offsetcal = unpack_int

pack_gaincal = partial(pack_int, nbytes=3)
unpack_gaincal = unpack_int

pack_lock = partial(pack_int, nbytes=1)
unpack_lock = unpack_int

unpack_chip_id = unpack_int  #  reserved register that identifies chip

unpack_crccfg = unpack_int
