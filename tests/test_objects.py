import unittest
from adafruit_mcp36xx import objects

class TestBasic(unittest.TestCase):
    def setUp(self):
        pass

    def test_temperature_transfer(self):
        vref = 2.4
        bits = 24
        for lsb in [1<<17, 1<<18, 1<<19, 1<<20, 1<<21, 1<<22]:
            temp = objects.first_order_temperature_sensor_transfer_to_c(lsb=lsb, vref=vref)
            vin = objects.first_order_temperature_sensor_transfer_to_vin(temp=temp)
            self.assertLessEqual(vin, vref)
            self.assertGreaterEqual(vin, 0)

            lsbm = objects.calc_lsb_for_dc_input(vinp=(vref/2+vin), vinn=vref/2, vrefp=vref)
            self.assertGreaterEqual(lsbm, 0)
            self.assertLessEqual(lsbm, ((1<<bits)-1))
            self.assertAlmostEqual(lsb, lsbm, -3) # FIXME this is a big difference


    def test_get_conversion_time(self):
        dmclk_freq = 4.192e6
        for osr in range(0, 0x10):
            osr1 = objects.get_osr1(osr)
            osr3 = objects.get_osr3(osr)
            self.assertLessEqual(osr1, 192)
            self.assertLessEqual(osr3, 512)

    def test_make_command_byte(self):
        address = 0x1
        body = objects.InternalRegisterAddress.kConfig0
        command = objects.CommandType.kIncrimentalWrite
        byte = objects.make_command_byte(address, body, command)
        self.assertEqual(byte&0x3, command)
        self.assertEqual((byte>>2)&0xf, body)
        self.assertEqual((byte>>6)&0x3, address)

    def test_pack_config0(self):
        partial_shutdown=True
        vref_sel = objects.VRefSelection.kOutput
        clk_sel = objects.ClockSelection.kExternal
        cs_sel = objects.BurnoutCurrentSourceSetting.k0
        adc_mode = objects.AdcOperatingMode.kConversion

        byte = objects.pack_config0(partial_shutdown, vref_sel, clk_sel, cs_sel, adc_mode)
        self.assertEqual(byte, 0b10000011)

    def test_pack_unpack_config0(self):
        partial_shutdown=True
        vref_sel = objects.VRefSelection.kInput
        clk_sel = objects.ClockSelection.kExternal
        cs_sel = objects.BurnoutCurrentSourceSetting.k15u
        adc_mode = objects.AdcOperatingMode.kConversion

        byte = objects.pack_config0(partial_shutdown, vref_sel, clk_sel, cs_sel, adc_mode)
        unpack = objects.unpack_config0(byte)
        self.assertEqual(unpack, (
            partial_shutdown,
            vref_sel,
            clk_sel,
            cs_sel,
            adc_mode
        ))

    def test_pack_unpack_config1(self):
        clk_prescale = 0x2
        osr = 0b1010
        byte = objects.pack_config1(clk_prescale, osr)
        self.assertEqual(byte, 0b10101000)

        unpack = objects.unpack_config1(byte)
        self.assertEqual(unpack, (
            clk_prescale, osr
        ))

    def test_pack_unpack_config2(self):
        boost = objects.BoostSetting.k2
        gain = objects.AdcGainSetting.k1
        az_mux = False
        az_ref = True
        byte = objects.pack_config2(boost, gain, az_mux, az_ref)
        self.assertEqual(byte, 0b11001010)

        unpack = objects.unpack_config2(byte)
        self.assertEqual(unpack, (
            boost, gain, az_mux, az_ref
        ))

    def test_pack_unpack_config3(self):
        conv_mode = objects.ConversionMode.kOneShotShutdown
        data_format = objects.DataFormat.kDataSign
        crc_format = objects.CRCFormat.k16
        enable_crc = False
        enable_offsetcal = True
        enable_gaincal = False

        t = (
            conv_mode,
            data_format,
            crc_format,
            enable_crc,
            enable_offsetcal,
            enable_gaincal
        )

        byte = objects.pack_config3(*t)
        self.assertEqual(byte, 0b00000010)

        unpack = objects.unpack_config3(byte)
        self.assertEqual(unpack, t)


    def test_pack_unpack_irq(self):
        mdat_irq = False
        irq_inactive_state = False
        enable_fastcmd = False
        enable_conversion_start_interrupt = False

        t = (
            mdat_irq,
            irq_inactive_state,
            enable_fastcmd,
            enable_conversion_start_interrupt
        )

        t_out = [False, False, False]
        for pt in t:
            t_out.append(pt)

        byte = objects.pack_irq(*t)
        self.assertEqual(byte, 0b00000000)

        unpack = objects.unpack_irq(byte)
        self.assertEqual(unpack, tuple(t_out))


    def test_pack_unpack_mux(self):
        vinp = objects.MuxChannel.kSingleChannel0
        vinn = objects.MuxChannel.kSingleChannel1

        t = (
            vinp,
            vinn
        )

        byte = objects.pack_mux(*t)
        self.assertEqual(byte, 0b00000001)

        unpack = objects.unpack_mux(byte)
        self.assertEqual(unpack, t)


    def test_pack_unpack_scan(self):
        delay = objects.InnerScanDelay.k0
        channel = objects.MuxChannel.kSingleChannel1

        t = (
            delay,
            channel
        )

        data = objects.pack_scan(*t)
        self.assertEqual(data[0], 0b00000000)
        self.assertEqual(data[1], 0b00000000)
        self.assertEqual(data[2], 0b00000001)

        unpack = objects.unpack_scan(data)
        self.assertEqual(unpack, t)

    def test_pack_unpack_int_single_byte(self):
        value = 0xfa

        t = (
            value,
        )

        data = objects.pack_int(*t, nbytes=1)
        self.assertEqual(data[0], value)

        unpack = objects.unpack_int(data, nbytes=1)
        self.assertEqual(unpack, t)

    def test_pack_unpack_int(self):
        value = 0xfa

        t = (
            value,
        )

        data = objects.pack_int(*t, nbytes=3)
        self.assertEqual(data[0], 0)
        self.assertEqual(data[1], 0)
        self.assertEqual(data[2], value)

        unpack = objects.unpack_int(data, nbytes=3)
        self.assertEqual(unpack, t)


if __name__ == '__main__':
    unittest.main()
