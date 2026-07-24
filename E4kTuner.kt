/*
 * librtlsdrk - Kotlin port of librtlsdr (RTL-SDR Blog fork) for Android USB host
 *
 * Based on tuner_e4k.c (Elonics E4000 driver):
 * Copyright (C) 2011-2012 by Harald Welte <laforge@gnumonks.org>
 * Copyright (C) 2012 by Sylvain Munaut <tnt@246tNt.com>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
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

import android.util.Log
import kotlin.math.min

/**
 * Elonics E4000 tuner driver.
 *
 * Faithful Kotlin port of tuner_e4k.c plus the e4000_* wrapper glue from
 * librtlsdr.c. The chip crystal (vco.fosc in the C code) is read live from
 * [TunerContext.getTunerClock], so PPM-correction changes are picked up on the
 * next tune, matching librtlsdr's behavior of refreshing e4k_s.vco.fosc.
 *
 * Methods return 0 on success and a negative value on failure, mirroring the
 * C code (-EINVAL is represented as [EINVAL_NEG]).
 */
class E4kTuner(private val ctx: TunerContext) : RtlTuner {

    companion object {
        private const val TAG = "E4kTuner"

        const val E4K_I2C_ADDR = 0xc8

        private const val EINVAL_NEG = -22  /* -EINVAL */

        /* enum e4k_reg (only the registers the driver uses) */
        private const val REG_MASTER1 = 0x00
        private const val REG_CLK_INP = 0x05
        private const val REG_REF_CLK = 0x06
        private const val REG_SYNTH1 = 0x07
        private const val REG_SYNTH3 = 0x09
        private const val REG_SYNTH4 = 0x0a
        private const val REG_SYNTH5 = 0x0b
        private const val REG_SYNTH7 = 0x0d
        private const val REG_FILT1 = 0x10
        private const val REG_FILT2 = 0x11
        private const val REG_FILT3 = 0x12
        private const val REG_GAIN1 = 0x14
        private const val REG_GAIN2 = 0x15
        private const val REG_GAIN3 = 0x16
        private const val REG_GAIN4 = 0x17
        private const val REG_AGC1 = 0x1a
        private const val REG_AGC4 = 0x1d
        private const val REG_AGC5 = 0x1e
        private const val REG_AGC6 = 0x1f
        private const val REG_AGC7 = 0x20
        private const val REG_AGC11 = 0x24
        private const val REG_DC5 = 0x2d
        private const val REG_DCTIME1 = 0x70
        private const val REG_DCTIME2 = 0x71
        private const val REG_BIAS = 0x78
        private const val REG_CLKOUT_PWDN = 0x7a

        private const val MASTER1_RESET = 1 shl 0
        private const val MASTER1_NORM_STBY = 1 shl 1
        private const val MASTER1_POR_DET = 1 shl 2

        private const val FILT3_DISABLE = 1 shl 5

        private const val AGC1_MOD_MASK = 0xF
        private const val AGC_MOD_SERIAL = 0x0
        private const val AGC_MOD_IF_SERIAL_LNA_AUTON = 0x9

        private const val AGC7_MIX_GAIN_AUTO = 1 shl 0
        private const val AGC11_LNA_GAIN_ENH = 1 shl 0

        /* enum e4k_band */
        private const val BAND_VHF2 = 0
        private const val BAND_VHF3 = 1
        private const val BAND_UHF = 2
        private const val BAND_L = 3

        /* enum e4k_if_filter */
        private const val IF_FILTER_MIX = 0
        private const val IF_FILTER_CHAN = 1
        private const val IF_FILTER_RC = 2

        private const val E4K_PLL_Y = 65536L

        /* look-up table bit-width -> mask */
        private val WIDTH2MASK = intArrayOf(0, 1, 3, 7, 0xf, 0x1f, 0x3f, 0x7f, 0xff)

        private fun mhz(x: Int) = x * 1000L * 1000L
        private fun khz(x: Int) = x * 1000L

        private val RF_FILT_CENTER_UHF = longArrayOf(
            mhz(360), mhz(380), mhz(405), mhz(425),
            mhz(450), mhz(475), mhz(505), mhz(540),
            mhz(575), mhz(615), mhz(670), mhz(720),
            mhz(760), mhz(840), mhz(890), mhz(970)
        )

        private val RF_FILT_CENTER_L = longArrayOf(
            mhz(1300), mhz(1320), mhz(1360), mhz(1410),
            mhz(1445), mhz(1460), mhz(1490), mhz(1530),
            mhz(1560), mhz(1590), mhz(1640), mhz(1660),
            mhz(1680), mhz(1700), mhz(1720), mhz(1750)
        )

        /* Mixer Filter */
        private val MIX_FILTER_BW = longArrayOf(
            khz(27000), khz(27000), khz(27000), khz(27000),
            khz(27000), khz(27000), khz(27000), khz(27000),
            khz(4600), khz(4200), khz(3800), khz(3400),
            khz(3300), khz(2700), khz(2300), khz(1900)
        )

        /* IF RC Filter */
        private val IFRC_FILTER_BW = longArrayOf(
            khz(21400), khz(21000), khz(17600), khz(14700),
            khz(12400), khz(10600), khz(9000), khz(7700),
            khz(6400), khz(5300), khz(4400), khz(3400),
            khz(2600), khz(1800), khz(1200), khz(1000)
        )

        /* IF Channel Filter */
        private val IFCH_FILTER_BW = longArrayOf(
            khz(5500), khz(5300), khz(5000), khz(4800),
            khz(4600), khz(4400), khz(4300), khz(4100),
            khz(3900), khz(3800), khz(3700), khz(3600),
            khz(3400), khz(3300), khz(3200), khz(3100),
            khz(3000), khz(2950), khz(2900), khz(2800),
            khz(2750), khz(2700), khz(2600), khz(2550),
            khz(2500), khz(2450), khz(2400), khz(2300),
            khz(2280), khz(2240), khz(2200), khz(2150)
        )

        /* indexed by e4k_if_filter: MIX, CHAN, RC */
        private val IF_FILTER_BW = arrayOf(MIX_FILTER_BW, IFCH_FILTER_BW, IFRC_FILTER_BW)

        private data class RegField(val reg: Int, val shift: Int, val width: Int)

        /* indexed by e4k_if_filter: MIX, CHAN, RC */
        private val IF_FILTER_FIELDS = arrayOf(
            RegField(REG_FILT2, 4, 4),
            RegField(REG_FILT3, 0, 5),
            RegField(REG_FILT2, 0, 4),
        )

        private class PllSettings(val freq: Long, val regSynth7: Int, val mult: Int)

        private val PLL_VARS = arrayOf(
            PllSettings(khz(72400), (1 shl 3) or 7, 48),
            PllSettings(khz(81200), (1 shl 3) or 6, 40),
            PllSettings(khz(108300), (1 shl 3) or 5, 32),
            PllSettings(khz(162500), (1 shl 3) or 4, 24),
            PllSettings(khz(216600), (1 shl 3) or 3, 16),
            PllSettings(khz(325000), (1 shl 3) or 2, 12),
            PllSettings(khz(350000), (1 shl 3) or 1, 8),
            PllSettings(khz(432000), (0 shl 3) or 3, 8),
            PllSettings(khz(667000), (0 shl 3) or 2, 6),
            PllSettings(khz(1200000), (0 shl 3) or 1, 4),
        )

        /* IF stage gain tables, in dB (index 0 unused) */
        private val IF_STAGE1_GAIN = intArrayOf(-3, 6)
        private val IF_STAGE23_GAIN = intArrayOf(0, 3, 6, 9)
        private val IF_STAGE4_GAIN = intArrayOf(0, 1, 2, 2)
        private val IF_STAGE56_GAIN = intArrayOf(3, 6, 9, 12, 15, 15, 15, 15)

        private val IF_STAGE_GAIN = arrayOf(
            intArrayOf(),
            IF_STAGE1_GAIN,
            IF_STAGE23_GAIN,
            IF_STAGE23_GAIN,
            IF_STAGE4_GAIN,
            IF_STAGE56_GAIN,
            IF_STAGE56_GAIN,
        )

        private val IF_STAGE_GAIN_REGS = arrayOf(
            RegField(0, 0, 0),
            RegField(REG_GAIN3, 0, 1),
            RegField(REG_GAIN3, 1, 2),
            RegField(REG_GAIN3, 3, 2),
            RegField(REG_GAIN3, 5, 2),
            RegField(REG_GAIN4, 0, 3),
            RegField(REG_GAIN4, 3, 3),
        )

        /* pairs of { gain in tenth dB, register value } */
        private val LNA_GAIN = intArrayOf(
            -50, 0,
            -25, 1,
            0, 4,
            25, 5,
            50, 6,
            75, 7,
            100, 8,
            125, 9,
            150, 10,
            175, 11,
            200, 12,
            250, 13,
            300, 14,
        )

        val GAINS = intArrayOf(
            -10, 15, 40, 65, 90, 115, 140, 165, 190, 215, 240, 290, 340, 420
        )
    }

    /** PLL state (struct e4k_pll_params vco). */
    private var band = BAND_VHF2
    private var vcoFosc = 0L
    private var vcoFlo = 0L
    private var vcoIntendedFlo = 0L
    private var vcoX = 0
    private var vcoZ = 0
    private var vcoR = 2
    private var vcoRIdx = 0
    private var vcoThreephase = false

    override val name = "Elonics E4000"
    override val gains: IntArray get() = GAINS

    // ==================== Register access ====================

    private fun regWrite(reg: Int, value: Int): Int {
        val rc = ctx.i2cWrite(E4K_I2C_ADDR, byteArrayOf(reg.toByte(), value.toByte()))
        return if (rc == 2) 0 else -1
    }

    private fun regRead(reg: Int): Int {
        if (ctx.i2cWrite(E4K_I2C_ADDR, byteArrayOf(reg.toByte())) < 1) return -1
        val data = ctx.i2cRead(E4K_I2C_ADDR, 1) ?: return -1
        if (data.isEmpty()) return -1
        return data[0].toInt() and 0xff
    }

    private fun regSetMask(reg: Int, mask: Int, value: Int): Int {
        /* like the C, a failed read (-1) is truncated to 0xff before masking */
        val tmp = regRead(reg) and 0xff
        if ((tmp and mask) == value) return 0
        return regWrite(reg, (tmp and mask.inv()) or (value and mask))
    }

    private fun fieldWrite(field: RegField, value: Int): Int {
        val rc = regRead(field.reg)
        if (rc < 0) return rc
        val mask = WIDTH2MASK[field.width] shl field.shift
        return regSetMask(field.reg, mask, value shl field.shift)
    }

    // ==================== Filter control ====================

    private fun closestArrIdx(arr: LongArray, freq: Long): Int {
        var bi = 0
        var bestDelta = Long.MAX_VALUE
        for (i in arr.indices) {
            val delta = if (freq > arr[i]) freq - arr[i] else arr[i] - freq
            if (delta < bestDelta) {
                bestDelta = delta
                bi = i
            }
        }
        return bi
    }

    /* return 4-bit index as to which RF filter to select */
    private fun chooseRfFilter(band: Int, freq: Long): Int = when (band) {
        BAND_VHF2, BAND_VHF3 -> 0
        BAND_UHF -> closestArrIdx(RF_FILT_CENTER_UHF, freq)
        BAND_L -> closestArrIdx(RF_FILT_CENTER_L, freq)
        else -> EINVAL_NEG
    }

    private fun rfFilterSet(): Int {
        val rc = chooseRfFilter(band, vcoFlo)
        if (rc < 0) return rc
        return regSetMask(REG_FILT1, 0xF, rc)
    }

    private fun ifFilterBwSet(filter: Int, bandwidth: Long): Int {
        if (filter >= IF_FILTER_BW.size) return EINVAL_NEG
        val bwIdx = closestArrIdx(IF_FILTER_BW[filter], bandwidth)
        return fieldWrite(IF_FILTER_FIELDS[filter], bwIdx)
    }

    private fun ifFilterChanEnable(on: Boolean): Int =
        regSetMask(REG_FILT3, FILT3_DISABLE, if (on) 0 else FILT3_DISABLE)

    // ==================== Frequency control ====================

    private fun isFoscValid(fosc: Long): Boolean {
        if (fosc < mhz(16) || fosc > mhz(30)) {
            Log.e(TAG, "Fosc $fosc invalid")
            return false
        }
        return true
    }

    /* Fvco = Fosc * Z + (Fosc * X)/Y, 64-bit to avoid overflow */
    private fun computeFvco(fosc: Long, z: Int, x: Int): Long =
        fosc * z + (fosc * x) / E4K_PLL_Y

    private fun computeFlo(fosc: Long, z: Int, x: Int, r: Int): Long =
        computeFvco(fosc, z, x) / r

    private fun bandSet(newBand: Int): Int {
        when (newBand) {
            BAND_VHF2, BAND_VHF3, BAND_UHF -> regWrite(REG_BIAS, 3)
            BAND_L -> regWrite(REG_BIAS, 0)
        }

        /* workaround: reset the register before writing to it, otherwise
         * there is a gap between 325-350 MHz */
        regSetMask(REG_SYNTH1, 0x06, 0)
        val rc = regSetMask(REG_SYNTH1, 0x06, newBand shl 1)
        if (rc >= 0) band = newBand
        return rc
    }

    /**
     * e4k_compute_pll_params: fills the vco* fields on success and returns the
     * actual PLL frequency, or 0 in case of error.
     */
    private fun computePllParams(fosc: Long, intendedFlo: Long): Long {
        var r = 2
        var rIdx = 0
        var threephase = false

        if (!isFoscValid(fosc)) return 0

        for (v in PLL_VARS) {
            if (intendedFlo < v.freq) {
                threephase = (v.regSynth7 and 0x08) != 0
                rIdx = v.regSynth7
                r = v.mult
                break
            }
        }

        /* flo(max) = 1700MHz, R(max) = 48 -> 64-bit */
        val intendedFvco = intendedFlo * r

        /* integral component of the multiplier (uint8_t in the C struct) */
        val z = (intendedFvco / fosc).toInt() and 0xff

        /* fractional part; remainder(max) = 30MHz * 65536 -> 64-bit */
        val remainder = intendedFvco - fosc * (intendedFvco / fosc)
        val x = ((remainder * E4K_PLL_Y) / fosc).toInt()

        val flo = computeFlo(fosc, z, x, r)

        vcoFosc = fosc
        vcoFlo = flo
        vcoIntendedFlo = intendedFlo
        vcoR = r
        vcoRIdx = rIdx
        vcoThreephase = threephase
        vcoX = x
        vcoZ = z

        return flo
    }

    /** e4k_tune_params: programs the PLL and sets band + RF filter. */
    private fun tuneParams(): Long {
        /* program R + 3phase/2phase */
        regWrite(REG_SYNTH7, vcoRIdx)
        /* program Z */
        regWrite(REG_SYNTH3, vcoZ)
        /* program X */
        regWrite(REG_SYNTH4, vcoX and 0xff)
        regWrite(REG_SYNTH5, (vcoX shr 8) and 0xff)

        /* we're in auto calibration mode, so there's no need to trigger it */

        /* set the band */
        when {
            vcoFlo < mhz(140) -> bandSet(BAND_VHF2)
            vcoFlo < mhz(350) -> bandSet(BAND_VHF3)
            vcoFlo < mhz(1135) -> bandSet(BAND_UHF)
            else -> bandSet(BAND_L)
        }

        /* select and set proper RF filter */
        rfFilterSet()

        return vcoFlo
    }

    /** e4k_tune_freq. */
    private fun tuneFreq(freq: Long): Int {
        /* determine PLL parameters (fosc refreshed with current PPM corr) */
        val flo = computePllParams(ctx.getTunerClock(), freq)
        if (flo == 0L) return EINVAL_NEG

        /* actually tune to those parameters */
        tuneParams()

        /* check PLL lock */
        val rc = regRead(REG_SYNTH1)
        if ((rc and 0x01) == 0) {
            Log.e(TAG, "PLL not locked for $freq Hz!")
            return -1
        }

        return 0
    }

    // ==================== Gain control ====================

    private fun setLnaGain(gain: Int): Int {
        for (i in 0 until LNA_GAIN.size / 2) {
            if (LNA_GAIN[i * 2] == gain) {
                regSetMask(REG_GAIN1, 0xf, LNA_GAIN[i * 2 + 1])
                return gain
            }
        }
        return EINVAL_NEG
    }

    private fun enableManualGain(manual: Boolean): Int {
        if (manual) {
            /* Set LNA mode to manual */
            regSetMask(REG_AGC1, AGC1_MOD_MASK, AGC_MOD_SERIAL)
            /* Set Mixer Gain Control to manual */
            regSetMask(REG_AGC7, AGC7_MIX_GAIN_AUTO, 0)
        } else {
            /* Set LNA mode to auto */
            regSetMask(REG_AGC1, AGC1_MOD_MASK, AGC_MOD_IF_SERIAL_LNA_AUTON)
            /* Set Mixer Gain Control to auto */
            regSetMask(REG_AGC7, AGC7_MIX_GAIN_AUTO, 1)
            regSetMask(REG_AGC11, 0x7, 0)
        }
        return 0
    }

    private fun findStageGain(stage: Int, value: Int): Int {
        if (stage >= IF_STAGE_GAIN.size) return EINVAL_NEG
        val arr = IF_STAGE_GAIN[stage]
        for (i in arr.indices) {
            if (arr[i] == value) return i
        }
        return EINVAL_NEG
    }

    /** e4k_if_gain_set: [stage] 1..6, [value] gain in dB. */
    private fun ifGainSet(stage: Int, value: Int): Int {
        val rc = findStageGain(stage, value)
        if (rc < 0) return rc
        val field = IF_STAGE_GAIN_REGS[stage]
        val mask = WIDTH2MASK[field.width] shl field.shift
        return regSetMask(field.reg, mask, rc shl field.shift)
    }

    private fun mixerGainSet(value: Int): Int {
        val bit = when (value) {
            4 -> 0
            12 -> 1
            else -> return EINVAL_NEG
        }
        return regSetMask(REG_GAIN2, 1, bit)
    }

    // ==================== RtlTuner interface (e4000_* glue) ====================

    /** e4k_init. */
    override fun init(): Int {
        /* make a dummy i2c read command, will not be ACKed! */
        regRead(0)

        /* reset everything and clear POR indicator */
        regWrite(REG_MASTER1, MASTER1_RESET or MASTER1_NORM_STBY or MASTER1_POR_DET)

        /* configure clock input */
        regWrite(REG_CLK_INP, 0x00)

        /* disable clock output */
        regWrite(REG_REF_CLK, 0x00)
        regWrite(REG_CLKOUT_PWDN, 0x96)

        /* magic values */
        regWrite(0x7e, 0x01)
        regWrite(0x7f, 0xfe)
        regWrite(0x82, 0x00)
        regWrite(0x86, 0x50) /* polarity A */
        regWrite(0x87, 0x20)
        regWrite(0x88, 0x01)
        regWrite(0x9f, 0x7f)
        regWrite(0xa0, 0x07)

        /* set LNA mode to manual */
        regWrite(REG_AGC4, 0x10) /* high threshold */
        regWrite(REG_AGC5, 0x04) /* low threshold */
        regWrite(REG_AGC6, 0x1a) /* LNA calib + loop rate */

        regSetMask(REG_AGC1, AGC1_MOD_MASK, AGC_MOD_SERIAL)

        /* set Mixer Gain Control to manual */
        regSetMask(REG_AGC7, AGC7_MIX_GAIN_AUTO, 0)

        /* use auto-gain as default */
        enableManualGain(false)

        /* select moderate gain levels */
        ifGainSet(1, 6)
        ifGainSet(2, 0)
        ifGainSet(3, 0)
        ifGainSet(4, 0)
        ifGainSet(5, 9)
        ifGainSet(6, 9)

        /* set the most narrow filter we can possibly use */
        ifFilterBwSet(IF_FILTER_MIX, khz(1900))
        ifFilterBwSet(IF_FILTER_RC, khz(1000))
        ifFilterBwSet(IF_FILTER_CHAN, khz(2150))
        ifFilterChanEnable(true)

        /* disable time variant DC correction and LUT */
        regSetMask(REG_DC5, 0x03, 0)
        regSetMask(REG_DCTIME1, 0x03, 0)
        regSetMask(REG_DCTIME2, 0x03, 0)

        return 0
    }

    /** e4000_exit = e4k_standby(1). */
    override fun exit(): Int {
        regSetMask(REG_MASTER1, MASTER1_NORM_STBY, 0)
        return 0
    }

    override fun setFreq(freqHz: Long): Int = tuneFreq(freqHz)

    /** e4000_set_bw: applies [bwHz] to the MIX, RC and CHAN filters. */
    override fun setBandwidth(bwHz: Int): Int {
        var r = 0
        r = r or ifFilterBwSet(IF_FILTER_MIX, bwHz.toLong())
        r = r or ifFilterBwSet(IF_FILTER_RC, bwHz.toLong())
        r = r or ifFilterBwSet(IF_FILTER_CHAN, bwHz.toLong())
        return if (r < 0) -1 else 0
    }

    /** e4000_set_gain: splits [gainTenthDb] into mixer + LNA gain. */
    override fun setGain(gainTenthDb: Int): Int {
        val mixGain = if (gainTenthDb > 340) 12 else 4
        if (setLnaGain(min(300, gainTenthDb - mixGain * 10)) == EINVAL_NEG) return -1
        if (mixerGainSet(mixGain) == EINVAL_NEG) return -1
        return 0
    }

    /** e4000_set_if_gain. */
    override fun setIfGain(stage: Int, gainTenthDb: Int): Int =
        ifGainSet(stage, gainTenthDb / 10)

    /** e4000_set_gain_mode = e4k_enable_manual_gain. */
    override fun setGainMode(manual: Boolean): Int = enableManualGain(manual)
}
