/*
 * librtlsdrk - Kotlin port of librtlsdr (RTL-SDR Blog fork) for Android USB host
 *
 * Based on tuner_fc2580.c (FCI FC2580 driver, taken from the kernel
 * driver found on http://linux.terratec.de/tv_en.html)
 *
 * Kotlin port: Copyright (C) 2026 Isak Ruas <isakruas@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

package com.isaklab.librtlsdrk

/**
 * FCI FC2580 tuner driver.
 *
 * Kotlin port of tuner_fc2580.c. The C driver ignores the RTL2832U clock and
 * uses the tuner's own 16.384 MHz crystal (at least on the Logilink VG0002A).
 * The C code accumulates FCI_SUCCESS(1)/FCI_FAIL(0) with `&=`; here that is a
 * Boolean `ok` accumulator.
 */
class Fc2580Tuner(private val ctx: TunerContext) : RtlTuner {

    companion object {
        const val FC2580_I2C_ADDR = 0xac

        /* 16.384 MHz (at least on the Logilink VG0002A) */
        private const val CRYSTAL_FREQ = 16_384_000L
        private const val XTAL_KHZ = ((CRYSTAL_FREQ + 500) / 1000).toInt()

        /* 2.6 GHz: border between low and high VCO */
        private const val BORDER_FREQ = 2_600_000L
        private const val USE_EXT_CLK = 0

        private const val BAND_UHF = 0
        private const val BAND_L = 1
        private const val BAND_VHF = 2
    }

    override val name = "FCI FC2580"
    override val gains = intArrayOf(0) /* no gain values */

    private fun wr(reg: Int, value: Int): Boolean =
        ctx.i2cWriteReg(FC2580_I2C_ADDR, reg, value and 0xff) >= 0

    private fun rd(reg: Int): Int = ctx.i2cReadReg(FC2580_I2C_ADDR, reg)

    override fun init(): Int {
        /* fc2580_set_init with FC2580_AGC_EXTERNAL */
        var ok = wr(0x00, 0x00) /* Confidential */
        ok = wr(0x12, 0x86) && ok
        ok = wr(0x14, 0x5C) && ok
        ok = wr(0x16, 0x3C) && ok
        ok = wr(0x1F, 0xD2) && ok
        ok = wr(0x09, 0xD7) && ok
        ok = wr(0x0B, 0xD5) && ok
        ok = wr(0x0C, 0x32) && ok
        ok = wr(0x0E, 0x43) && ok
        ok = wr(0x21, 0x0A) && ok
        ok = wr(0x22, 0x82) && ok
        /* Voltage Control Mode (external AGC) */
        ok = wr(0x45, 0x20) && ok
        ok = wr(0x4C, 0x02) && ok /* HOLD_AGC polarity */
        ok = wr(0x3F, 0x88) && ok
        ok = wr(0x02, 0x0E) && ok
        ok = wr(0x58, 0x14) && ok
        ok = setFilter(8) && ok /* BW = 7.8 MHz */

        return if (ok) 0 else -1
    }

    override fun exit(): Int = 0

    override fun setFreq(freqHz: Long): Int {
        /* fc2580_set_freq works in kHz */
        val fLo = (freqHz + 500) / 1000
        val xtal = XTAL_KHZ.toLong()

        val band = if (fLo > 1_000_000L) BAND_L else if (fLo > 400_000L) BAND_UHF else BAND_VHF

        var ok = true
        var data0x02 = (USE_EXT_CLK shl 5) or 0x0E

        val fVco = when (band) {
            BAND_UHF -> fLo * 4
            BAND_L -> fLo * 2
            else -> fLo * 12
        }
        val rVal = if (fVco >= 2 * 76 * xtal) 1 else if (fVco >= 76 * xtal) 2 else 4
        val fComp = xtal / rVal
        val nVal = (fVco / 2) / fComp

        val preShiftBits = 4
        val fDiff = fVco - 2 * fComp * nVal
        val fDiffShifted = fDiff shl (20 - preShiftBits)
        var kVal = fDiffShifted / ((2 * fComp) shr preShiftBits)
        if (fDiffShifted - kVal * ((2 * fComp) shr preShiftBits) >= (fComp shr preShiftBits)) {
            kVal++
        }

        /* Select VCO band */
        data0x02 = if (fVco >= BORDER_FREQ) data0x02 or 0x08 else data0x02 and 0xF7

        when (band) {
            BAND_UHF -> {
                data0x02 = data0x02 and 0x3F

                ok = wr(0x25, 0xF0) && ok
                ok = wr(0x27, 0x77) && ok
                ok = wr(0x28, 0x53) && ok
                ok = wr(0x29, 0x60) && ok
                ok = wr(0x30, 0x09) && ok
                ok = wr(0x50, 0x8C) && ok
                ok = wr(0x53, 0x50) && ok

                ok = wr(0x5F, if (fLo < 538_000L) 0x13 else 0x15) && ok

                if (fLo < 538_000L) {
                    ok = wr(0x61, 0x07) && ok
                    ok = wr(0x62, 0x06) && ok
                    ok = wr(0x67, 0x06) && ok
                    ok = wr(0x68, 0x08) && ok
                    ok = wr(0x69, 0x10) && ok
                    ok = wr(0x6A, 0x12) && ok
                } else if (fLo < 794_000L) {
                    ok = wr(0x61, 0x03) && ok
                    ok = wr(0x62, 0x03) && ok
                    ok = wr(0x67, 0x03) && ok /* ACI improve */
                    ok = wr(0x68, 0x05) && ok /* ACI improve */
                    ok = wr(0x69, 0x0C) && ok
                    ok = wr(0x6A, 0x0E) && ok
                } else {
                    ok = wr(0x61, 0x07) && ok
                    ok = wr(0x62, 0x06) && ok
                    ok = wr(0x67, 0x07) && ok
                    ok = wr(0x68, 0x09) && ok
                    ok = wr(0x69, 0x10) && ok
                    ok = wr(0x6A, 0x12) && ok
                }

                ok = wr(0x63, 0x15) && ok

                ok = wr(0x6B, 0x0B) && ok
                ok = wr(0x6C, 0x0C) && ok
                ok = wr(0x6D, 0x78) && ok
                ok = wr(0x6E, 0x32) && ok
                ok = wr(0x6F, 0x14) && ok
                ok = setFilter(8) && ok /* BW = 7.8 MHz */
            }
            BAND_VHF -> {
                data0x02 = (data0x02 and 0x3F) or 0x80
                ok = wr(0x27, 0x77) && ok
                ok = wr(0x28, 0x33) && ok
                ok = wr(0x29, 0x40) && ok
                ok = wr(0x30, 0x09) && ok
                ok = wr(0x50, 0x8C) && ok
                ok = wr(0x53, 0x50) && ok
                ok = wr(0x5F, 0x0F) && ok
                ok = wr(0x61, 0x07) && ok
                ok = wr(0x62, 0x00) && ok
                ok = wr(0x63, 0x15) && ok
                ok = wr(0x67, 0x03) && ok
                ok = wr(0x68, 0x05) && ok
                ok = wr(0x69, 0x10) && ok
                ok = wr(0x6A, 0x12) && ok
                ok = wr(0x6B, 0x08) && ok
                ok = wr(0x6C, 0x0A) && ok
                ok = wr(0x6D, 0x78) && ok
                ok = wr(0x6E, 0x32) && ok
                ok = wr(0x6F, 0x54) && ok
                ok = setFilter(7) && ok /* BW = 6.8 MHz */
            }
            BAND_L -> {
                data0x02 = (data0x02 and 0x3F) or 0x40
                ok = wr(0x2B, 0x70) && ok
                ok = wr(0x2C, 0x37) && ok
                ok = wr(0x2D, 0xE7) && ok
                ok = wr(0x30, 0x09) && ok
                ok = wr(0x44, 0x20) && ok
                ok = wr(0x50, 0x8C) && ok
                ok = wr(0x53, 0x50) && ok
                ok = wr(0x5F, 0x0F) && ok
                ok = wr(0x61, 0x0F) && ok
                ok = wr(0x62, 0x00) && ok
                ok = wr(0x63, 0x13) && ok
                ok = wr(0x67, 0x00) && ok
                ok = wr(0x68, 0x02) && ok
                ok = wr(0x69, 0x0C) && ok
                ok = wr(0x6A, 0x0E) && ok
                ok = wr(0x6B, 0x08) && ok
                ok = wr(0x6C, 0x0A) && ok
                ok = wr(0x6D, 0xA0) && ok
                ok = wr(0x6E, 0x50) && ok
                ok = wr(0x6F, 0x14) && ok
                ok = setFilter(1) && ok /* BW = 1.53 MHz */
            }
        }

        /* AGC clock's pre-divide ratio */
        if (XTAL_KHZ >= 28_000) {
            ok = wr(0x4B, 0x22) && ok
        }

        /* VCO band and PLL setting */
        ok = wr(0x02, data0x02) && ok
        val data0x18 = (if (rVal == 1) 0x00 else if (rVal == 2) 0x10 else 0x20) +
            ((kVal shr 16).toInt() and 0xff)
        ok = wr(0x18, data0x18) && ok             /* 'R' + high part of 'K' */
        ok = wr(0x1A, (kVal shr 8).toInt()) && ok /* middle part of 'K' */
        ok = wr(0x1B, kVal.toInt()) && ok         /* lower part of 'K' */
        ok = wr(0x1C, nVal.toInt()) && ok         /* 'N' */

        /* UHF LNA load cap */
        if (band == BAND_UHF) {
            ok = wr(0x2D, if (fLo <= 794_000L) 0x9F else 0x8F) && ok
        }

        return if (ok) 0 else -1
    }

    /** librtlsdr always calls fc2580_SetBandwidthMode(dev, 1) → 1.53 MHz. */
    override fun setBandwidth(bwHz: Int): Int = if (setFilter(1)) 0 else -1

    /**
     * fc2580_set_filter. filter_bw: 1 = 1.53 MHz, 6 = 6 MHz, 7 = 6.8 MHz,
     * 8 = 7.8 MHz. Includes the calibration monitor retry loop.
     */
    private fun setFilter(filterBw: Int): Boolean {
        var ok = true

        when (filterBw) {
            1 -> {
                ok = wr(0x36, 0x1C) && ok
                ok = wr(0x37, (4151L * XTAL_KHZ / 1_000_000).toInt()) && ok
                ok = wr(0x39, 0x00) && ok
                ok = wr(0x2E, 0x09) && ok
            }
            6 -> {
                ok = wr(0x36, 0x18) && ok
                ok = wr(0x37, (4400L * XTAL_KHZ / 1_000_000).toInt()) && ok
                ok = wr(0x39, 0x00) && ok
                ok = wr(0x2E, 0x09) && ok
            }
            7 -> {
                ok = wr(0x36, 0x18) && ok
                ok = wr(0x37, (3910L * XTAL_KHZ / 1_000_000).toInt()) && ok
                ok = wr(0x39, 0x80) && ok
                ok = wr(0x2E, 0x09) && ok
            }
            8 -> {
                ok = wr(0x36, 0x18) && ok
                ok = wr(0x37, (3300L * XTAL_KHZ / 1_000_000).toInt()) && ok
                ok = wr(0x39, 0x80) && ok
                ok = wr(0x2E, 0x09) && ok
            }
        }

        /* calibration check; C's wait_msec is a no-op (USB latency suffices) */
        for (i in 0 until 5) {
            val calMon = rd(0x2F)
            ok = (calMon >= 0) && ok
            if (calMon < 0 || (calMon and 0xC0) != 0xC0) {
                ok = wr(0x2E, 0x01) && ok
                ok = wr(0x2E, 0x09) && ok
            } else {
                break
            }
        }

        ok = wr(0x2E, 0x01) && ok

        return ok
    }
}
