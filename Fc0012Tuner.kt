/*
 * librtlsdrk - Kotlin port of librtlsdr (RTL-SDR Blog fork) for Android USB host
 *
 * Based on tuner_fc0012.c (Fitipower FC0012 driver):
 * Copyright (C) 2012 Hans-Frieder Vogt <hfvogt@gmx.net>
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
 * Fitipower FC0012 tuner driver.
 *
 * Faithful Kotlin port of tuner_fc0012.c from the RTL-SDR Blog fork of
 * librtlsdr, including the librtlsdr wrapper semantics (fc0012_set_freq:
 * GPIO 6 selects the V-band/U-band filter, bandwidth fixed at 6 MHz).
 * Methods return 0 on success and a negative value on failure.
 */
class Fc0012Tuner(private val ctx: TunerContext) : RtlTuner {

    companion object {
        private const val TAG = "Fc0012Tuner"

        const val FC0012_I2C_ADDR = 0xc6

        /* Initial register values, indices 0x01..0x15 (reg. 0 is a dummy). */
        private val INIT_REGS = intArrayOf(
            0x00,   /* dummy reg. 0 */
            0x05,   /* reg. 0x01 */
            0x10,   /* reg. 0x02 */
            0x00,   /* reg. 0x03 */
            0x00,   /* reg. 0x04 */
            0x0f,   /* reg. 0x05: may also be 0x0a */
            0x00,   /* reg. 0x06: divider 2, VCO slow */
            0x00,   /* reg. 0x07: may also be 0x0f */
            0xff,   /* reg. 0x08: AGC Clock divide by 256, AGC gain 1/256, Loop Bw 1/8 */
            0x6e,   /* reg. 0x09: Disable LoopThrough */
            0xb8,   /* reg. 0x0a: Disable LO Test Buffer */
            0x82,   /* reg. 0x0b: Output Clock is same as clock frequency */
            0xfc,   /* reg. 0x0c: depending on AGC Up-Down mode, may need 0xf8 */
            0x02,   /* reg. 0x0d: AGC Not Forcing & LNA Forcing, 0x02 for DVB-T */
            0x00,   /* reg. 0x0e */
            0x00,   /* reg. 0x0f */
            0x00,   /* reg. 0x10: may also be 0x0d */
            0x00,   /* reg. 0x11 */
            0x1f,   /* reg. 0x12: Set to maximum gain */
            0x08,   /* reg. 0x13: Middle Gain: 0x08, Low: 0x00, High: 0x10 */
            0x00,   /* reg. 0x14 */
            0x04,   /* reg. 0x15: Enable LNA COMPS */
        )

        private val GAINS = intArrayOf(-99, -40, 71, 179, 192)
    }

    override val name: String get() = "Fitipower FC0012"
    override val gains: IntArray get() = GAINS

    private fun writeReg(reg: Int, value: Int): Int =
        if (ctx.i2cWriteReg(FC0012_I2C_ADDR, reg, value and 0xff) < 0) -1 else 0

    /** Returns the register byte (0..255) or -1 on failure. */
    private fun readReg(reg: Int): Int = ctx.i2cReadReg(FC0012_I2C_ADDR, reg)

    override fun init(): Int {
        val reg = INIT_REGS.copyOf()

        /* 28.8 MHz crystal input */
        reg[0x07] = reg[0x07] or 0x20

        /* dual master */
        reg[0x0c] = reg[0x0c] or 0x02

        for (i in 1 until reg.size) {
            val ret = writeReg(i, reg[i])
            if (ret != 0) return ret
        }
        return 0
    }

    override fun exit(): Int = 0

    override fun setFreq(freqHz: Long): Int {
        /* select V-band/U-band filter (librtlsdr fc0012_set_freq wrapper) */
        ctx.setGpioBit(6, freqHz > 300_000_000L)
        return setParams(freqHz, 6_000_000)
    }

    /** Port of fc0012_set_params. */
    private fun setParams(freq: Long, bandwidth: Int): Int {
        val reg = IntArray(7)
        var ret: Int
        var vcoSelect = false

        val xtalFreqDiv2 = ctx.getTunerClock() / 2

        /* select frequency divider and the frequency of VCO */
        val multi: Int
        when {
            freq < 37_084_000L -> {         /* freq * 96 < 3560000000 */
                multi = 96; reg[5] = 0x82; reg[6] = 0x00
            }
            freq < 55_625_000L -> {         /* freq * 64 < 3560000000 */
                multi = 64; reg[5] = 0x82; reg[6] = 0x02
            }
            freq < 74_167_000L -> {         /* freq * 48 < 3560000000 */
                multi = 48; reg[5] = 0x42; reg[6] = 0x00
            }
            freq < 111_250_000L -> {        /* freq * 32 < 3560000000 */
                multi = 32; reg[5] = 0x42; reg[6] = 0x02
            }
            freq < 148_334_000L -> {        /* freq * 24 < 3560000000 */
                multi = 24; reg[5] = 0x22; reg[6] = 0x00
            }
            freq < 222_500_000L -> {        /* freq * 16 < 3560000000 */
                multi = 16; reg[5] = 0x22; reg[6] = 0x02
            }
            freq < 296_667_000L -> {        /* freq * 12 < 3560000000 */
                multi = 12; reg[5] = 0x12; reg[6] = 0x00
            }
            freq < 445_000_000L -> {        /* freq * 8 < 3560000000 */
                multi = 8; reg[5] = 0x12; reg[6] = 0x02
            }
            freq < 593_334_000L -> {        /* freq * 6 < 3560000000 */
                multi = 6; reg[5] = 0x0a; reg[6] = 0x00
            }
            else -> {
                multi = 4; reg[5] = 0x0a; reg[6] = 0x02
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

        reg[3] = (xin shr 8) and 0xff   /* xin with 9 bit resolution */
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

        /* VCO Calibration */
        ret = writeReg(0x0e, 0x80)
        if (ret == 0) ret = writeReg(0x0e, 0x00)

        /* VCO Re-Calibration if needed */
        if (ret == 0) ret = writeReg(0x0e, 0x00)

        var tmp = 0
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

    /** Port of fc0012_set_gain (gain in tenths of dB, snapped to the table). */
    override fun setGain(gainTenthDb: Int): Int {
        /* C ignores a failed read here and starts from 0 */
        val r = readReg(0x13)
        var tmp = if (r < 0) 0 else r

        /* mask bits off */
        tmp = tmp and 0xe0

        when (gainTenthDb) {
            -99 -> tmp = tmp or 0x02    /* -9.9 dB */
            -40 -> {}                   /* -4 dB */
            71 -> tmp = tmp or 0x08     /* 7.1 dB */
            179 -> tmp = tmp or 0x17    /* 17.9 dB */
            else -> tmp = tmp or 0x10   /* 19.2 dB */
        }

        return writeReg(0x13, tmp)
    }
}
