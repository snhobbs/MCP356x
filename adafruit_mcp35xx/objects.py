'''
Model of an MCP356x
This should have all the internal state and respond as the chip would according to the data sheet.
The inputs can be set as an externally updated object. The interface should also be seperately modeled.
Interacting can then be done like a model + view + controller.
'''

import enum
from typing import Union, List, Literal, Tuple
from functools import partial
import numpy as np


'''
Takes a bytearray from an adc channel reading
and return a signed int properly masked
'''
def buffer_to_nbit_int(buffer : bytearray, byteorder: Literal['little', 'big'], signed: bool):
    return int.from_bytes(buffer, byteorder=byteorder, signed=signed)


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


def first_order_temperature_sensor_transfer_v_to_c(vin: float, vref: float) -> float:
    '''Eq. 5-1
    page 40.
    gain=1, mclk=4.9152MHz.'''
    return (vin*1000-80)/0.2973


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
    kRightPaddedZero = 1
    kSignExtended = 2
    kChannelPlusSignExtended = 3


def read_adc_output(mode: DataFormat, data: Union[bytes, bytearray]) -> Tuple[int, int]:
    channel = -1
    value = 0
    data = bytearray(data)

    def sign_magnitude_to_int(data, sign) -> int:
        assert len(data) == 3
        abs_value = (data[0]<<16) | (data[1]<<8) | (data[2])
        assert abs_value < (1<<23)
        assert abs_value >= 0
        mult = 1
        if sign:
            mult = -1
        value = abs_value * mult
        return value

    if mode == DataFormat.kDataSign:
        assert len(data) == 3
        data[0] = data[0]&0x7f
        sign = data[0]&(1<<7)
        value = sign_magnitude_to_int(data, sign=sign)

    elif mode == DataFormat.kRightPaddedZero:
        assert len(data) == 4
        assert(data[-1] == 0)
        data = data[:-1] # drop last byte
        sign = data[0]&(1<<7)
        value = sign_magnitude_to_int(data, sign=sign)

    elif mode == DataFormat.kSignExtended:
        assert len(data) == 4
        value = int.from_bytes(data, byteorder="big", signed=True)

    elif mode == DataFormat.kChannelPlusSignExtended:
        assert len(data) == 4
        channel = (data[0] >> 4) & 0x0f
        if data[0]&0x0f == 0x0f:
            data[0] = 0xff
        else:
            data[0] = 0
        value = int.from_bytes(data, byteorder="big", signed=True)
    else:
        assert 0

    return channel, value

def format_adc_output(mode: DataFormat, data: int, channel: int = 0) -> bytes:
    sign = int(data < 0)
    out = None
    if mode == DataFormat.kDataSign:
        dout = abs(data) | (sign<<23)
        out = dout.to_bytes(length=3, byteorder="big", signed=True)
    elif mode == DataFormat.kRightPaddedZero:
        dout = (abs(data) | (sign<<23)) << 8
        out = dout.to_bytes(length=4, byteorder="big", signed=True)
    elif mode == DataFormat.kSignExtended:
        out = data.to_bytes(length=4, byteorder="big", signed=True)
    elif mode == DataFormat.kChannelPlusSignExtended:
        out = bytearray(data.to_bytes(length=4, byteorder="big", signed=True))
        out[0] &= 0x0f
        out[0] |= (channel & 0xf)<<4
    return bytes(out)


class VRefSelection(enum.IntEnum):
    '''Table 5-9
    page 50.'''
    kInternal = 1
    kExternal = 0


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
    kFullReset = 0b1110


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
    kReserved0 = 11
    kReserved1 = 12
    kLock = 13
    kReserved2 = 14
    kCrccfg = 15


class InternalRegister:
    def __init__(self, address: InternalRegisterAddress, nbytes: int):
        self.address = address
        self.nbytes = nbytes

    @property
    def name(self):
        return self.address.name


internal_registers = {
    InternalRegisterAddress.kAdcData: InternalRegister(InternalRegisterAddress.kAdcData, 4), # FIXME this can change
    InternalRegisterAddress.kConfig0: InternalRegister(InternalRegisterAddress.kConfig0, 1),
    InternalRegisterAddress.kConfig1: InternalRegister(InternalRegisterAddress.kConfig1, 1),
    InternalRegisterAddress.kConfig2: InternalRegister(InternalRegisterAddress.kConfig2, 1),
    InternalRegisterAddress.kConfig3: InternalRegister(InternalRegisterAddress.kConfig3, 1),
    InternalRegisterAddress.kIrq: InternalRegister(InternalRegisterAddress.kIrq, 1),
    InternalRegisterAddress.kMux: InternalRegister(InternalRegisterAddress.kMux, 1),
    InternalRegisterAddress.kScan: InternalRegister(InternalRegisterAddress.kScan, 3),
    InternalRegisterAddress.kTimer: InternalRegister(InternalRegisterAddress.kTimer, 3),
    InternalRegisterAddress.kOffsetcal: InternalRegister(InternalRegisterAddress.kOffsetcal, 3),
    InternalRegisterAddress.kGaincal: InternalRegister(InternalRegisterAddress.kGaincal, 3),
    InternalRegisterAddress.kReserved0: InternalRegister(InternalRegisterAddress.kReserved0, 3),
    InternalRegisterAddress.kReserved1: InternalRegister(InternalRegisterAddress.kReserved1, 1),
    InternalRegisterAddress.kLock: InternalRegister(InternalRegisterAddress.kLock, 1),
    InternalRegisterAddress.kReserved2: InternalRegister(InternalRegisterAddress.kReserved2, 2),
    InternalRegisterAddress.kCrccfg: InternalRegister(InternalRegisterAddress.kCrccfg, 2),
}


def make_command_byte(address: int, body: Union[CommandOperationType, InternalRegisterAddress], command: CommandType) -> int:
    '''Command byte
    table 6-1
    page 67.'''
    value = (address & 0b11)<<6 | (int(body)&0b1111)<<2 | (int(command)&0b11)
    assert value <= 0xff
    return value

CONFIG0_PARTIAL_SHUTDOWN_POSITION = 7
CONFIG0_VREF_SEL_POSITION = 6
CONFIG0_CLK_SEL_POSITION = 4
CONFIG0_CS_SEL_POSITION = 2
CONFIG0_ADC_MODE_POSITION = 0
def pack_config0(partial_shutdown: bool, vref_sel: VRefSelection, clk_sel: ClockSelection, cs_sel: BurnoutCurrentSourceSetting, adc_mode: AdcOperatingMode) -> int:
    return int(partial_shutdown) << CONFIG0_PARTIAL_SHUTDOWN_POSITION |\
        int(vref_sel) << CONFIG0_VREF_SEL_POSITION | int(clk_sel) <<  CONFIG0_CLK_SEL_POSITION|\
        int(cs_sel) << CONFIG0_CS_SEL_POSITION | int(adc_mode) << CONFIG0_ADC_MODE_POSITION


def unpack_config0(value: Union[bytes, bytearray, int]) -> tuple:
    if not isinstance(value, int):
        value = value[0]

    partial_shutdown = ((value >> CONFIG0_PARTIAL_SHUTDOWN_POSITION) & 0x1) == 1
    vref_sel = VRefSelection((value >> CONFIG0_VREF_SEL_POSITION) & 0x1)
    clk_sel = ClockSelection((value >> CONFIG0_CLK_SEL_POSITION) & 0x3)
    cs_sel = BurnoutCurrentSourceSetting((value >> CONFIG0_CS_SEL_POSITION) & 0x3)
    adc_mode = AdcOperatingMode((value >> CONFIG0_ADC_MODE_POSITION) & 0x3)
    return partial_shutdown, vref_sel, clk_sel, cs_sel, adc_mode


CONFIG1_CLK_PRESCALE_POSITION = 6
CONFIG1_OSR_POSITION = 2
def pack_config1(clk_prescale: int, osr: int) -> int:
    assert clk_prescale <= 0x3
    assert clk_prescale >= 0
    assert osr < 0xf
    assert osr >= 0
    return (clk_prescale << CONFIG1_CLK_PRESCALE_POSITION) | (osr << CONFIG1_OSR_POSITION)


def unpack_config1(value: Union[bytes, bytearray, int]) -> tuple:
    if not isinstance(value, int):
        value = value[0]
    clk_prescale = (value >> CONFIG1_CLK_PRESCALE_POSITION)&0x3
    osr = (value >> CONFIG1_OSR_POSITION) & 0xf
    return clk_prescale, osr


def pack_config2(boost: BoostSetting, gain: AdcGainSetting, az_mux: bool, az_ref: bool) -> int:
    return int(boost)<<6 | int(gain) << 3 | int(az_mux)<<2 | int(az_ref) << 1


def unpack_config2(value: Union[bytes, bytearray, int]) -> tuple:
    if not isinstance(value, int):
        value = value[0]
    boost = BoostSetting((value>>6)&0x3)
    gain = AdcGainSetting((value>>3)&0b111)
    az_mux = (value >> 2)&0x1
    az_ref = (value >> 1)&0x1
    return boost, gain, az_mux, az_ref


def pack_config3(conv_mode: ConversionMode, data_format: DataFormat, crc_format: CRCFormat, enable_crc: bool, enable_offsetcal: bool, enable_gaincal: bool) -> int:
    return int(conv_mode) << 6 | int(data_format) << 4 | int(crc_format) << 3 | int(enable_crc) << 2 | int(enable_offsetcal) << 1 | int(enable_gaincal)


def unpack_config3(value: Union[bytes, bytearray, int]) -> tuple:
    if not isinstance(value, int):
        value = value[0]
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


def unpack_irq(value: Union[bytes, bytearray, int]) -> tuple:
    if not isinstance(value, int):
        value = value[0]
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


def unpack_mux(value: Union[bytes, bytearray, int]) -> tuple:
    if not isinstance(value, int):
        value = value[0]
    vinp = MuxChannel(value >> 4)
    vinn = MuxChannel(value&0xf)
    return vinp, vinn


# Scan register
# page 97


def pack_scan(delay: InnerScanDelay, channels: List[MuxChannel]) -> bytearray:
    channel_int = 0
    for channel in channels:
        channel_int |= (1<<int(channel))
    return bytearray([int(delay)<<5, int(channel_int)>>8, int(channel_int)&0xff])


def unpack_scan(datain: Union[bytes, bytearray]) -> tuple:
    channels = []
    channel_int = (datain[1] << 8 | datain[2])
    for i in range(16):
        if (channel_int >> i) & 0x1:
            channels.append(MuxChannel(i))
    return datain[0] >> 5, channels


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


class ConfigurationTable:
    '''
    Holds all settings for the chip. This is passed to the setup function of the chip.
    '''
    def __init__(self):
        self._settings = dict()

        self._settings["partial_shutdown"]=False
        self._settings["vref_sel"]=VRefSelection.kInternal
        self._settings["clk_sel"]=ClockSelection.kInternalWithClockOutput
        self._settings["cs_sel"]=BurnoutCurrentSourceSetting.k0
        self._settings["adc_mode"]=AdcOperatingMode.kStandby

        self._settings["clk_prescale"]=0x3  # 8
        self._settings["osr"]=0b1010

        self._settings["boost"]=BoostSetting.k1
        self._settings["gain"]=AdcGainSetting.k1
        self._settings["az_mux"]=False
        self._settings["az_ref"]=False

        self._settings["conv_mode"]=ConversionMode.kOneShotStandby
        self._settings["data_format"]=DataFormat.kSignExtended
        self._settings["crc_format"]=CRCFormat.k16
        self._settings["enable_crc"]=False
        self._settings["enable_offsetcal"]=False
        self._settings["enable_gaincal"]=False

        self._settings["mdat_irq"]=False  #  Regular data mode, changing this put the data into 4 bit MDAT mode!!
        self._settings["irq_inactive_state"]=True
        self._settings["enable_fastcmd"]=True
        self._settings["enable_conversion_start_interrupt"]=True

        self._settings["vinp"]=MuxChannel.kChannel0
        self._settings["vinn"]=MuxChannel.kAGnd

        self._settings["delay"]=InnerScanDelay.k0
        self._settings["scan_channels"]=[]

        self._settings["timer"]= 1024

        self._settings["offsetcal"]= 0x0
        self._settings["gaincal"]= 0x800000 # Gain of 1, this is the default

        self._settings["lock"]= 0xA5  # leave unlocked

    def __setitem__(self, key, value):
        if key not in self._settings:
            raise KeyError(f"Key {key} not found")
        self._settings[key] = value

    def __getitem__(self, key):
        return self._settings[key]

    @property
    def register_settings(self):
        '''
        Returns the internal state as a dictionary of register settings.
        '''

        registers = dict()
        registers[InternalRegisterAddress.kConfig0] = pack_config0(
            partial_shutdown=self["partial_shutdown"],
            vref_sel=self["vref_sel"],
            clk_sel=self["clk_sel"],
            cs_sel=self["cs_sel"],
            adc_mode=self["adc_mode"])

        registers[InternalRegisterAddress.kConfig1] = pack_config1(
            clk_prescale=self["clk_prescale"],
            osr=self["osr"])

        registers[InternalRegisterAddress.kConfig2] = pack_config2(
            boost=self["boost"],
            gain=self["gain"],
            az_mux=self["az_mux"],
            az_ref=self["az_ref"])

        registers[InternalRegisterAddress.kConfig3] = pack_config3(
            conv_mode=self["conv_mode"],
            data_format=self["data_format"],
            crc_format=self["crc_format"],
            enable_crc=self["enable_crc"],
            enable_offsetcal=self["enable_offsetcal"],
            enable_gaincal=self["enable_gaincal"])

        registers[InternalRegisterAddress.kIrq] = pack_irq(
            mdat_irq=self["mdat_irq"],
            irq_inactive_state=self["irq_inactive_state"],
            enable_fastcmd=self["enable_fastcmd"],
            enable_conversion_start_interrupt=self["enable_conversion_start_interrupt"])

        #  Start off with the channel0 single ended
        registers[InternalRegisterAddress.kMux] = pack_mux(
            vinp=self["vinp"],
            vinn=self["vinn"])

        # Scan mode set to 0 sets mux mode
        registers[InternalRegisterAddress.kScan] = pack_scan(
            delay=self["delay"],
            channels=self["scan_channels"])

        registers[InternalRegisterAddress.kTimer] = pack_timer(self["timer"])

        registers[InternalRegisterAddress.kOffsetcal] = pack_offsetcal(self["offsetcal"])
        registers[InternalRegisterAddress.kGaincal] = pack_gaincal(self["gaincal"]) # Gain of 1, this is the default

        registers[InternalRegisterAddress.kLock] = pack_lock(self["lock"])  # leave unlocked
        return registers


def internal_registers_to_configuration_table(registers: dict) -> ConfigurationTable:
    table = ConfigurationTable()

    (table["partial_shutdown"],
     table["vref_sel"],
     table["clk_sel"],
     table["cs_sel"],
     table["adc_mode"]) = unpack_config0(registers[InternalRegisterAddress.kConfig0])

    (table["clk_prescale"],
     table["osr"]) = unpack_config1(registers[InternalRegisterAddress.kConfig1])

    (table["boost"],
     table["gain"],
     table["az_mux"],
     table["az_ref"]) = unpack_config2(registers[InternalRegisterAddress.kConfig2])

    t = unpack_config3(registers[InternalRegisterAddress.kConfig3])

    (table["conv_mode"],
     table["data_format"],
     table["crc_format"],
     table["enable_crc"],
     table["enable_offsetcal"],
     table["enable_gaincal"]) = t

    t = unpack_irq(registers[InternalRegisterAddress.kIrq])

    (data_ready,crccfg_status, por_status,
     table["mdat_irq"],
     table["irq_inactive_state"],
     table["enable_fastcmd"],
     table["enable_conversion_start_interrupt"]) = t

    t = unpack_mux(registers[InternalRegisterAddress.kMux])
    (table["vinp"],
     table["vinn"]) = t

    t = unpack_scan(registers[InternalRegisterAddress.kScan])
    table["delay"], table["scan_channels"] = t

    table["timer"] = unpack_timer(registers[InternalRegisterAddress.kTimer])[0]

    table["offsetcal"] = unpack_offsetcal(registers[InternalRegisterAddress.kOffsetcal])[0]
    table["gaincal"] = unpack_gaincal(registers[InternalRegisterAddress.kGaincal])[0]

    table["lock"] = unpack_lock(registers[InternalRegisterAddress.kLock])[0]
    return table

