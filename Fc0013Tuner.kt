/*
 * librtlsdrk - Kotlin port of librtlsdr (RTL-SDR Blog fork) for Android USB host
 *
 * Based on tuner_fc0013.c (Fitipower FC0013 driver):
 * Copyright (C) 2012 Hans-Frieder Vogt <hfvogt@gmx.net>
 * Copyright (C) 2010 Fitipower Integrated Technology Inc
 * Copyright (C) 2012 Steve Markgraf <steve@steve-m.de>
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

/**
 * Fitipower FC0013 tuner driver.
 *
 * Faithful Kotlin port of tuner_fc0013.c from the RTL-SDR Blog fork of
 * librtlsdr, including the librtlsdr wrapper semantics (bandwidth fixed at
 * 6 MHz, gain via the LNA gain table, gain mode via LNA forcing bit).
 * Methods return 0 on success and a negative value on failure.
 */
class Fc0013Tuner(private val ctx: TunerContext) : RtlTuner {

    companion object {
        private const val TAG = "Fc0013Tuner"

        const val FC0013_I2C_ADDR = 0xc6

        /* Initial register values, indices 0x01..0x15 (reg. 0 is a dummy). */
        private val INIT_REGS = intArrayOf(
            0x00,   /* reg. 0x00: dummy */
            0x09,   /* reg. 0x01 */
            0x16,   /* reg. 0x02 */
            0x00,   /* reg. 0x03 */
            0x00,   /* reg. 0x04 */
            0x17,   /* reg. 0x05 */
            0x02,   /* reg. 0x06: LPF bandwidth */
            0x0a,   /* reg. 0x07: CHECK */
            0xff,   /* reg. 0x08: AGC Clock divide by 256, AGC gain 1/256, Loop Bw 1/8 */
            0x6e,   /* reg. 0x09: Disable LoopThrough */
            0xb8,   /* reg. 0x0a: Disable LO Test Buffer */
            0x82,   /* reg. 0x0b: CHECK */
            0xfc,   /* reg. 0x0c: depending on AGC Up-Down mode, may need 0xf8 */
            0x01,   /* reg. 0x0d: AGC Not Forcing & LNA Forcing, may need 0x02 */
            0x00,   /* reg. 0x0e */
            0x00,   /* reg. 0x0f */
            0x00,   /* reg. 0x10 */
            0x00,   /* reg. 0x11 */
            0x00,   /* reg. 0x12 */
            0x00,   /* reg. 0x13 */
            0x50,   /* reg. 0x14: DVB-t High Gain, UHF. Middle: 0x48, Low: 0x40 */
            0x01,   /* reg. 0x15 */
        )

        /* gain (tenths of dB) -> reg 0x14 LNA bits */
        private val LNA_GAINS = intArrayOf(
            -99, 0x02,
            -73, 0x03,
            -65, 0x05,
            -63, 0x04,
            -63, 0x00,
            -60, 0x07,
            -58, 0x01,
            -54, 0x06,
            58, 0x0f,
            61, 0x0e,
            63, 0x0d,
            65, 0x0c,
            67, 0x0b,
            68, 0x0a,
            70, 0x09,
            71, 0x08,
            179, 0x17,
            181, 0x16,
            182, 0x15,
            184, 0x14,
            186, 0x13,
            188, 0x12,
            191, 0x11,
            197, 0x10,
        )
        private val GAIN_CNT = LNA_GAINS.size / 2

        private val GAINS = intArrayOf(
            -99, -73, -65, -63, -60, -58, -54, 58, 61,
            63, 65, 67, 68, 70, 71, 179, 181, 182,
            184, 186, 188, 191, 197
        )
    }

    override val name: String get() = "Fitipower FC0013"
    override val gains: IntArray get() = GAINS

    private fun writeReg(reg: Int, value: Int): Int =
        if (ctx.i2cWriteReg(FC0013_I2C_ADDR, reg, value and 0xff) < 0) -1 else 0

    /** Returns the register byte (0..255) or -1 on failure. */
    private fun readReg(reg: Int): Int = ctx.i2cReadReg(FC0013_I2C_ADDR, reg)

    override fun init(): Int {
        val reg = INIT_REGS.copyOf()

        /* 28.8 MHz crystal input */
        reg[0x07] = reg[0x07] or 0x20

        /* dual master */
        reg[0x0c] = reg[0x0c] or 0x02

        for (i in 1 until reg.size) {
            val ret = writeReg(i, reg[i])
            if (ret < 0) return ret
        }
        return 0
    }

    override fun exit(): Int = 0

    override fun setFreq(freqHz: Long): Int = setParams(freqHz, 6_000_000)

    /** Port of fc0013_rc_cal_add. */
    fun rcCalAdd(rcVal: Int): Int {
        /* push rc_cal value, get rc_cal value */
        var ret = writeReg(0x10, 0x00)
        if (ret != 0) return ret

        /* get rc_cal value */
        val rcCal = readReg(0x10)
        if (rcCal < 0) return -1

        val value = (rcCal and 0x0f) + rcVal

        /* forcing rc_cal */
        ret = writeReg(0x0d, 0x11)
        if (ret != 0) return ret

        /* modify rc_cal value */
        return when {
            value > 15 -> writeReg(0x10, 0x0f)
            value < 0 -> writeReg(0x10, 0x00)
            else -> writeReg(0x10, value)
        }
    }

    /** Port of fc0013_rc_cal_reset. */
    fun rcCalReset(): Int {
        var ret = writeReg(0x0d, 0x01)
        if (ret == 0) ret = writeReg(0x10, 0x00)
        return ret
    }

    /** Port of fc0013_set_vhf_track. */
    private fun setVhfTrack(freq: Long): Int {
        val r = readReg(0x1d)
        if (r < 0) return -1
        val tmp = r and 0xe3
        return when {
            freq <= 177_500_000L -> writeReg(0x1d, tmp or 0x1c)  /* VHF Track: 7 */
            freq <= 184_500_000L -> writeReg(0x1d, tmp or 0x18)  /* VHF Track: 6 */
            freq <= 191_500_000L -> writeReg(0x1d, tmp or 0x14)  /* VHF Track: 5 */
            freq <= 198_500_000L -> writeReg(0x1d, tmp or 0x10)  /* VHF Track: 4 */
            freq <= 205_500_000L -> writeReg(0x1d, tmp or 0x0c)  /* VHF Track: 3 */
            freq <= 219_500_000L -> writeReg(0x1d, tmp or 0x08)  /* VHF Track: 2 */
            freq < 300_000_000L -> writeReg(0x1d, tmp or 0x04)   /* VHF Track: 1 */
            else -> writeReg(0x1d, tmp or 0x1c)                  /* UHF and GPS */
        }
    }

    /** Port of fc0013_set_params. */
    private fun setParams(freq: Long, bandwidth: Int): Int {
        val reg = IntArray(7)
        var ret: Int
        var tmp: Int
        var vcoSelect = false

        val xtalFreqDiv2 = ctx.getTunerClock() / 2

        /* set VHF track */
        ret = setVhfTrack(freq)
        if (ret != 0) return ret

        if (freq < 300_000_000L) {
            /* enable VHF filter */
            tmp = readReg(0x07)
            if (tmp < 0) return -1
            ret = writeReg(0x07, tmp or 0x10)
            if (ret != 0) return ret

            /* disable UHF & disable GPS */
            tmp = readReg(0x14)
            if (tmp < 0) return -1
            ret = writeReg(0x14, tmp and 0x1f)
            if (ret != 0) return ret
        } else {
            /* disable VHF filter */
            tmp = readReg(0x07)
            if (tmp < 0) return -1
            ret = writeReg(0x07, tmp and 0xef)
            if (ret != 0) return ret

            /* enable UHF & disable GPS */
            tmp = readReg(0x14)
            if (tmp < 0) return -1
            ret = writeReg(0x14, (tmp and 0x1f) or 0x40)
            if (ret != 0) return ret
        }

        /* select frequency divider and the frequency of VCO */
        val multi: Int
        when {
            freq < 37_084_000L -> {         /* freq * 96 < 3560000000 */
                multi = 96; reg[5] = 0x82; reg[6] = 0x00
            }
            freq < 55_625_000L -> {         /* freq * 64 < 3560000000 */
                multi = 64; reg[5] = 0x02; reg[6] = 0x02
            }
            freq < 74_167_000L -> {         /* freq * 48 < 3560000000 */
                multi = 48; reg[5] = 0x42; reg[6] = 0x00
            }
            freq < 111_250_000L -> {        /* freq * 32 < 3560000000 */
                multi = 32; reg[5] = 0x82; reg[6] = 0x02
            }
            freq < 148_334_000L -> {        /* freq * 24 < 3560000000 */
                multi = 24; reg[5] = 0x22; reg[6] = 0x00
            }
            freq < 222_500_000L -> {        /* freq * 16 < 3560000000 */
                multi = 16; reg[5] = 0x42; reg[6] = 0x02
            }
            freq < 296_667_000L -> {        /* freq * 12 < 3560000000 */
                multi = 12; reg[5] = 0x12; reg[6] = 0x00
            }
            freq < 445_000_000L -> {        /* freq * 8 < 3560000000 */
                multi = 8; reg[5] = 0x22; reg[6] = 0x02
            }
            freq < 593_334_000L -> {        /* freq * 6 < 3560000000 */
                multi = 6; reg[5] = 0x0a; reg[6] = 0x00
            }
            freq < 950_000_000L -> {        /* freq * 4 < 3800000000 */
                multi = 4; reg[5] = 0x12; reg[6] = 0x02
            }
            else -> {
                multi = 2; reg[5] = 0x0a; reg[6] = 0x02
            }
        }

        val fVco = freq * multi

        if (fVco >= 3_060_000_000L) {
            reg[6] = reg[6] or 0x08
            vcoSelect = true
        }

        /* From divided value (XDIV) determine the FA and FP value */
        var xdiv = (fVco / xtalFreqDiv2).toInt() and 0xffff
        if (fVco - xdiv * xtalFreqDiv2 >= xtalFreqDiv2 / 2) xdiv++

        var pm = (xdiv / 8) and 0xff
        var am = (xdiv - 8 * pm) and 0xff

        if (am < 2) {
            am += 8
            pm = (pm - 1) and 0xff
        }

        if (pm > 31) {
            reg[1] = (am + 8 * (pm - 31)) and 0xff
            reg[2] = 31
        } else {
            reg[1] = am
            reg[2] = pm
        }

        if (reg[1] > 15 || reg[2] < 0x0b) {
            Log.e(TAG, "no valid PLL combination found for $freq Hz!")
            return -1
        }

        /* fix clock out */
        reg[6] = reg[6] or 0x20

        /* From VCO frequency determine XIN (fractional part of Delta
           Sigma PLL) and divided value (XDIV) */
        var xin = (((fVco - (fVco / xtalFreqDiv2) * xtalFreqDiv2) / 1000).toInt()) and 0xffff
        xin = ((xin shl 15) / (xtalFreqDiv2 / 1000).toInt()) and 0xffff
        if (xin >= 16384) xin += 32768

        reg[3] = (xin shr 8) and 0xff
        reg[4] = xin and 0xff

        reg[6] = reg[6] and 0x3f        /* bits 6 and 7 describe the bandwidth */
        when (bandwidth) {
            6_000_000 -> reg[6] = reg[6] or 0x80
            7_000_000 -> reg[6] = reg[6] or 0x40
            else -> {}                  /* 8 MHz */
        }

        /* modified for Realtek demod */
        reg[5] = reg[5] or 0x07

        for (i in 1..6) {
            ret = writeReg(i, reg[i])
            if (ret != 0) return ret
        }

        tmp = readReg(0x11)
        if (tmp < 0) return -1
        ret = if (multi == 64) {
            writeReg(0x11, tmp or 0x04)
        } else {
            writeReg(0x11, tmp and 0xfb)
        }
        if (ret != 0) return ret

        /* VCO Calibration */
        ret = writeReg(0x0e, 0x80)
        if (ret == 0) ret = writeReg(0x0e, 0x00)

        /* VCO Re-Calibration if needed */
        if (ret == 0) ret = writeReg(0x0e, 0x00)

        if (ret == 0) {
            tmp = readReg(0x0e)
            if (tmp < 0) return -1
        }
        if (ret != 0) return ret

        /* vco selection */
        tmp = tmp and 0x3f

        if (vcoSelect) {
            if (tmp > 0x3c) {
                reg[6] = reg[6] and 0x08.inv()
                ret = writeReg(0x06, reg[6])
                if (ret == 0) ret = writeReg(0x0e, 0x80)
                if (ret == 0) ret = writeReg(0x0e, 0x00)
            }
        } else {
            if (tmp < 0x02) {
                reg[6] = reg[6] or 0x08
                ret = writeReg(0x06, reg[6])
                if (ret == 0) ret = writeReg(0x0e, 0x80)
                if (ret == 0) ret = writeReg(0x0e, 0x00)
            }
        }

        return ret
    }

    /** Port of fc0013_set_gain_mode (manual = LNA forcing + fixed IF gain). */
    override fun setGainMode(manual: Boolean): Int {
        var ret = 0

        /* C starts from 0 when the read fails but still records the error */
        val r = readReg(0x0d)
        var tmp = if (r < 0) 0 else r
        if (r < 0) ret = -1

        tmp = if (manual) {
            tmp or (1 shl 3)
        } else {
            tmp and (1 shl 3).inv()
        }

        if (writeReg(0x0d, tmp) < 0) ret = -1

        /* set a fixed IF-gain for now */
        if (writeReg(0x13, 0x0a) < 0) ret = -1

        return ret
    }

    /** Port of fc0013_set_lna_gain (gain in tenths of dB, snapped to table). */
    override fun setGain(gainTenthDb: Int): Int {
        var ret = 0

        val r = readReg(0x14)
        var tmp = if (r < 0) 0 else r
        if (r < 0) ret = -1

        /* mask bits off */
        tmp = tmp and 0xe0

        for (i in 0 until GAIN_CNT) {
            if (LNA_GAINS[i * 2] >= gainTenthDb || i + 1 == GAIN_CNT) {
                tmp = tmp or LNA_GAINS[i * 2 + 1]
                break
            }
        }

        /* set gain */
        if (writeReg(0x14, tmp) < 0) ret = -1

        return ret
    }
}
