/*
 * librtlsdrk - Kotlin port of librtlsdr (RTL-SDR Blog fork) for Android USB host
 *
 * Based on tuner_r82xx.c (Rafael Micro R820T/R828D driver):
 * Copyright (C) 2013 Mauro Carvalho Chehab <mchehab@redhat.com>
 * Copyright (C) 2013 Steve Markgraf <steve@steve-m.de>
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
 * Rafael Micro R820T/R828D tuner driver.
 *
 * Faithful Kotlin port of tuner_r82xx.c from the RTL-SDR Blog fork of librtlsdr,
 * including the Blog modifications (max VCO current, 2.0V PLL dropout, and
 * RTL-SDR Blog V4 / V4L input switching with built-in upconverter support).
 *
 * All I2C access goes through the callbacks supplied by the RTL2832U driver;
 * the caller is responsible for enabling the RTL2832U I2C repeater around calls.
 * Methods return 0 on success and a negative value on failure, mirroring the C code.
 */
class R82xxTuner(
    private val chip: Chip,
    /** Corrected tuner crystal frequency in Hz (PPM correction already applied). */
    private val xtalFreq: () -> Long,
    /** Writes [data] to the tuner I2C address; returns bytes written or negative on error. */
    private val i2cWrite: (data: ByteArray) -> Int,
    /** Reads [len] bytes from the tuner I2C address into a new array, or null on error. */
    private val i2cRead: (len: Int) -> ByteArray?,
    /** rtlsdr_set_bias_tee_gpio equivalent, used for the Blog V4 upconverter switch. */
    private val setGpio: (gpio: Int, on: Boolean) -> Unit = { _, _ -> },
    /** True for RTL-SDR Blog V4 (R828D based, 28.8 MHz xtal, HF upconverter). */
    private val isBlogV4: Boolean = false,
    /** True for RTL-SDR Blog V4 Lite. */
    private val isBlogV4L: Boolean = false,
) : RtlTuner {
    enum class Chip { R820T, R828D }

    companion object {
        private const val TAG = "R82xxTuner"

        const val R820T_I2C_ADDR = 0x34
        const val R828D_I2C_ADDR = 0x74
        const val R82XX_CHECK_VAL = 0x69
        const val R82XX_IF_FREQ = 3_570_000

        private const val REG_SHADOW_START = 5
        private const val NUM_REGS = 30
        private const val VER_NUM = 49
        private const val MAX_I2C_MSG_LEN = 8

        // Band constants for the Blog V4/V4L input switch (HF/VHF/UHF in the C code).
        private const val BAND_HF = 1
        private const val BAND_VHF = 2
        private const val BAND_UHF = 3

        private const val MHZ_28_8 = 28_800_000L

        /** Initial register values, starting at REG_SHADOW_START (regs 0x05..0x1f). */
        private val INIT_ARRAY = intArrayOf(
            0x83, 0x30, 0x75,                   /* 05 to 07 */
            0xc0, 0x40, 0xd6, 0x6c,             /* 08 to 0b */
            0xf5, 0x63, 0x75, 0x68,             /* 0c to 0f */
            0x6c, 0x83, 0x80, 0x00,             /* 10 to 13 */
            0x0f, 0x00, 0xc0, 0x30,             /* 14 to 17 */
            0x48, 0xcc, 0x60, 0x00,             /* 18 to 1b */
            0x54, 0xae, 0x4a, 0xc0              /* 1c to 1f */
        )

        private data class FreqRange(
            val freqMhz: Int,
            val openD: Int,
            val rfMuxPloy: Int,
            val tfC: Int,
            val xtalCap20p: Int,
            val xtalCap10p: Int,
            val xtalCap0p: Int,
        )

        private val FREQ_RANGES = arrayOf(
            FreqRange(0, 0x08, 0x02, 0xdf, 0x02, 0x01, 0x00),
            FreqRange(50, 0x08, 0x02, 0xbe, 0x02, 0x01, 0x00),
            FreqRange(55, 0x08, 0x02, 0x8b, 0x02, 0x01, 0x00),
            FreqRange(60, 0x08, 0x02, 0x7b, 0x02, 0x01, 0x00),
            FreqRange(65, 0x08, 0x02, 0x69, 0x02, 0x01, 0x00),
            FreqRange(70, 0x08, 0x02, 0x58, 0x02, 0x01, 0x00),
            FreqRange(75, 0x00, 0x02, 0x44, 0x02, 0x01, 0x00),
            FreqRange(80, 0x00, 0x02, 0x44, 0x02, 0x01, 0x00),
            FreqRange(90, 0x00, 0x02, 0x34, 0x01, 0x01, 0x00),
            FreqRange(100, 0x00, 0x02, 0x34, 0x01, 0x01, 0x00),
            FreqRange(110, 0x00, 0x02, 0x24, 0x01, 0x01, 0x00),
            FreqRange(120, 0x00, 0x02, 0x24, 0x01, 0x01, 0x00),
            FreqRange(140, 0x00, 0x02, 0x14, 0x01, 0x01, 0x00),
            FreqRange(180, 0x00, 0x02, 0x13, 0x00, 0x00, 0x00),
            FreqRange(220, 0x00, 0x02, 0x13, 0x00, 0x00, 0x00),
            FreqRange(250, 0x00, 0x02, 0x11, 0x00, 0x00, 0x00),
            FreqRange(280, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00),
            FreqRange(310, 0x00, 0x41, 0x00, 0x00, 0x00, 0x00),
            FreqRange(450, 0x00, 0x41, 0x00, 0x00, 0x00, 0x00),
            FreqRange(588, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00),
            FreqRange(650, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00),
        )

        /* Measured gain steps (tenths of dB), see tuner_r82xx.c. */
        private val LNA_GAIN_STEPS = intArrayOf(
            0, 9, 13, 40, 38, 13, 31, 22, 26, 31, 26, 14, 19, 5, 35, 13
        )
        private val MIXER_GAIN_STEPS = intArrayOf(
            0, 5, 10, 10, 19, 9, 10, 25, 17, 10, 8, 16, 13, 6, 3, -8
        )

        val GAINS = intArrayOf(
            0, 9, 14, 27, 37, 77, 87, 125, 144, 157,
            166, 197, 207, 229, 254, 280, 297, 328,
            338, 364, 372, 386, 402, 421, 434, 439,
            445, 480, 496
        )

        /* Bandwidth contribution by low-pass filter. */
        private val IF_LOW_PASS_BW_TABLE = intArrayOf(
            1_700_000, 1_600_000, 1_550_000, 1_450_000, 1_200_000,
            900_000, 700_000, 550_000, 450_000, 350_000
        )
        private const val FILT_HP_BW1 = 350_000
        private const val FILT_HP_BW2 = 380_000

        private val BITREV_LUT = intArrayOf(
            0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
            0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf
        )

        private fun bitrev(b: Int): Int =
            (BITREV_LUT[b and 0x0f] shl 4) or BITREV_LUT[(b shr 4) and 0x0f]
    }

    /** Shadow copy of registers REG_SHADOW_START..REG_SHADOW_START+NUM_REGS-1. */
    private val regs = IntArray(NUM_REGS)

    /** Current IF frequency in Hz (int_freq in the C code). */
    var intFreq: Int = R82XX_IF_FREQ
        private set

    var hasLock = false
        private set
    var initDone = false
        private set

    private var filCalCode = 0
    private var input = 0

    /** Last manual gain, reapplied when switching AGC back to manual. */
    private var lastManualGain = 0

    override val name: String
        get() = if (chip == Chip.R828D) "Rafael Micro R828D" else "Rafael Micro R820T"
    override val gains: IntArray get() = GAINS

    override fun exit(): Int = standby()

    override fun setGain(gainTenthDb: Int): Int {
        lastManualGain = gainTenthDb
        return setGain(true, gainTenthDb)
    }

    /* librtlsdr passes 0 here; reapplying the last manual gain instead
     * restores the previous level when the user toggles AGC off */
    override fun setGainMode(manual: Boolean): Int = setGain(manual, lastManualGain)

    // ==================== I2C / shadow register helpers ====================

    private fun shadowStore(reg: Int, values: IntArray, offset: Int, length: Int) {
        var r = reg - REG_SHADOW_START
        var pos = offset
        var len = length
        if (r < 0) {
            pos -= r
            len += r
            r = 0
        }
        if (len <= 0) return
        if (len > NUM_REGS - r) len = NUM_REGS - r
        for (i in 0 until len) {
            regs[r + i] = values[pos + i] and 0xff
        }
    }

    private fun write(reg: Int, values: IntArray, offset: Int = 0, length: Int = values.size): Int {
        shadowStore(reg, values, offset, length)

        var currentReg = reg
        var pos = offset
        var len = length
        do {
            val size = if (len > MAX_I2C_MSG_LEN - 1) MAX_I2C_MSG_LEN - 1 else len

            val buf = ByteArray(size + 1)
            buf[0] = currentReg.toByte()
            for (i in 0 until size) {
                buf[1 + i] = values[pos + i].toByte()
            }

            val rc = i2cWrite(buf)
            if (rc != size + 1) {
                Log.e(TAG, "i2c wr failed=$rc reg=${currentReg.toString(16)} len=$size")
                return if (rc < 0) rc else -1
            }

            currentReg += size
            len -= size
            pos += size
        } while (len > 0)

        return 0
    }

    private fun writeReg(reg: Int, value: Int): Int = write(reg, intArrayOf(value and 0xff))

    private fun readCacheReg(reg: Int): Int {
        val r = reg - REG_SHADOW_START
        return if (r in 0 until NUM_REGS) regs[r] else -1
    }

    private fun writeRegMask(reg: Int, value: Int, bitMask: Int): Int {
        val rc = readCacheReg(reg)
        if (rc < 0) return rc
        val v = (rc and bitMask.inv()) or (value and bitMask)
        return write(reg, intArrayOf(v and 0xff))
    }

    /**
     * Reads [len] registers starting at reg 0. The R82xx returns data with
     * reversed bit order, so every byte is bit-reversed before being returned.
     */
    private fun read(reg: Int, len: Int): IntArray? {
        val rc = i2cWrite(byteArrayOf(reg.toByte()))
        if (rc != 1) {
            Log.e(TAG, "i2c wr (read setup) failed=$rc reg=${reg.toString(16)}")
            return null
        }
        val data = i2cRead(len) ?: run {
            Log.e(TAG, "i2c rd failed reg=${reg.toString(16)} len=$len")
            return null
        }
        if (data.size < len) return null
        return IntArray(len) { bitrev(data[it].toInt() and 0xff) }
    }

    // ==================== Tuning logic ====================

    private fun setMux(freq: Long): Int {
        val freqMhz = freq / 1_000_000
        var idx = 0
        while (idx < FREQ_RANGES.size - 1) {
            if (freqMhz < FREQ_RANGES[idx + 1].freqMhz) break
            idx++
        }
        val range = FREQ_RANGES[idx]

        /* Open Drain */
        var rc = writeRegMask(0x17, range.openD, 0x08)
        if (rc < 0) return rc

        /* RF_MUX, Polymux */
        rc = writeRegMask(0x1a, range.rfMuxPloy, 0xc3)
        if (rc < 0) return rc

        /* TF BAND */
        rc = writeReg(0x1b, range.tfC)
        if (rc < 0) return rc

        /* XTAL CAP & Drive: xtal_cap_sel is always XTAL_HIGH_CAP_0P in librtlsdr */
        rc = writeRegMask(0x10, range.xtalCap0p or 0x00, 0x0b)
        if (rc < 0) return rc

        rc = writeRegMask(0x08, 0x00, 0x3f)
        if (rc < 0) return rc

        return writeRegMask(0x09, 0x00, 0x3f)
    }

    private fun setPll(freq: Long): Int {
        val vcoMin = 1_770_000L /* kHz */
        val vcoMax = vcoMin * 2

        val freqKhz = (freq + 500) / 1000
        val pllRef = xtalFreq()
        val pllRefKhz = (pllRef + 500) / 1000

        /* refdiv2 = 0 */
        var rc = writeRegMask(0x10, 0x00, 0x10)
        if (rc < 0) return rc

        /* set pll autotune = 128kHz */
        rc = writeRegMask(0x1a, 0x00, 0x0c)
        if (rc < 0) return rc

        /* RTL-SDR Blog modification: set VCO current to max */
        rc = writeRegMask(0x12, 0x06, 0xff)
        if (rc < 0) return rc

        /* Calculate divider */
        var mixDiv = 2
        var divNum = 0
        while (mixDiv <= 64) {
            if (freqKhz * mixDiv >= vcoMin && freqKhz * mixDiv < vcoMax) {
                var divBuf = mixDiv
                while (divBuf > 2) {
                    divBuf = divBuf shr 1
                    divNum++
                }
                break
            }
            mixDiv = mixDiv shl 1
        }

        var data = read(0x00, 5) ?: return -1

        var vcoPowerRef = 2
        if (chip == Chip.R828D || isBlogV4L) vcoPowerRef = 1

        val vcoFineTune = (data[4] and 0x30) shr 4
        if (vcoFineTune > vcoPowerRef) divNum--
        else if (vcoFineTune < vcoPowerRef) divNum++

        rc = writeRegMask(0x10, (divNum shl 5) and 0xff, 0xe0)
        if (rc < 0) return rc

        val vcoFreq = freq * mixDiv
        val nint = (vcoFreq / (2 * pllRef)).toInt()
        var vcoFra = ((vcoFreq - 2 * pllRef * nint) / 1000).toInt()

        if (nint > (128 / vcoPowerRef) - 1) {
            Log.e(TAG, "No valid PLL values for $freq Hz!")
            return -1
        }

        val ni = (nint - 13) / 4
        val si = nint - 4 * ni - 13

        rc = writeReg(0x14, (ni + (si shl 6)) and 0xff)
        if (rc < 0) return rc

        /* pw_sdm */
        rc = writeRegMask(0x12, if (vcoFra == 0) 0x08 else 0x00, 0x08)
        if (rc < 0) return rc

        /* sdm calculator */
        var nSdm = 2
        var sdm = 0
        while (vcoFra > 1) {
            if (vcoFra > (2 * pllRefKhz / nSdm).toInt()) {
                sdm += 32768 / (nSdm / 2)
                vcoFra -= (2 * pllRefKhz / nSdm).toInt()
                if (nSdm >= 0x8000) break
            }
            nSdm = nSdm shl 1
        }

        rc = writeReg(0x16, (sdm shr 8) and 0xff)
        if (rc < 0) return rc
        rc = writeReg(0x15, sdm and 0xff)
        if (rc < 0) return rc

        /* Check if PLL has locked */
        var locked = false
        for (i in 0 until 2) {
            data = read(0x00, 3) ?: return -1
            if ((data[2] and 0x40) != 0) {
                locked = true
                break
            }
            if (i == 0) {
                /* Didn't lock. RTL-SDR Blog hack: set max current */
                rc = writeRegMask(0x12, 0x06, 0xff)
                if (rc < 0) return rc
            }
        }

        if (!locked) {
            Log.w(TAG, "PLL not locked at $freq Hz!")
            hasLock = false
            return 0
        }
        hasLock = true

        /* set pll autotune = 8kHz */
        return writeRegMask(0x1a, 0x08, 0x08)
    }

    /**
     * r82xx_sysfreq_sel for the SYS_DVBT / TUNER_DIGITAL_TV case used by librtlsdr.
     */
    private fun sysfreqSel(): Int {
        val mixerTop = 0x24     /* mixer top:13, top-1, low-discharge */
        val lnaTop = 0xe5       /* detect bw 3, lna top:4, predet top:2 */
        val lnaVthL = 0x53      /* lna vth 0.84, vtl 0.64 */
        val mixerVthL = 0x75    /* mixer vth 1.04, vtl 0.84 */
        val airCable1In = 0x00
        val cable2In = 0x00
        val lnaDischarge = 14
        val cpCur = 0x38        /* 111, auto */
        val filterCur = 0x40    /* 10, low */
        /* RTL-SDR Blog hack: improve L-band performance, PLL dropout 2.0V */
        val divBufCur = 0xa0

        var rc = writeRegMask(0x1d, lnaTop, 0xc7)
        if (rc < 0) return rc
        rc = writeRegMask(0x1c, mixerTop, 0xf8)
        if (rc < 0) return rc
        rc = writeReg(0x0d, lnaVthL)
        if (rc < 0) return rc
        rc = writeReg(0x0e, mixerVthL)
        if (rc < 0) return rc

        input = airCable1In

        /* Air-IN only for Astrometa */
        rc = writeRegMask(0x05, airCable1In, 0x60)
        if (rc < 0) return rc
        rc = writeRegMask(0x06, cable2In, 0x08)
        if (rc < 0) return rc

        rc = writeRegMask(0x11, cpCur, 0x38)
        if (rc < 0) return rc
        rc = writeRegMask(0x17, divBufCur, 0x30)
        if (rc < 0) return rc
        rc = writeRegMask(0x0a, filterCur, 0x60)
        if (rc < 0) return rc

        /* LNA setup, digital TV path */
        rc = writeRegMask(0x1d, 0, 0x38)
        if (rc < 0) return rc
        rc = writeRegMask(0x1c, 0, 0x04)
        if (rc < 0) return rc
        rc = writeRegMask(0x06, 0, 0x40)
        if (rc < 0) return rc

        /* agc clk 250hz */
        rc = writeRegMask(0x1a, 0x30, 0x30)
        if (rc < 0) return rc

        /* write LNA TOP = 3 */
        rc = writeRegMask(0x1d, 0x18, 0x38)
        if (rc < 0) return rc

        /* write discharge mode */
        rc = writeRegMask(0x1c, mixerTop, 0x04)
        if (rc < 0) return rc

        /* LNA discharge current */
        rc = writeRegMask(0x1e, lnaDischarge, 0x1f)
        if (rc < 0) return rc

        /* agc clk 60hz */
        return writeRegMask(0x1a, 0x20, 0x30)
    }

    /**
     * r82xx_set_tv_standard for the "BW < 6 MHz" digital-TV profile used by librtlsdr.
     * Note: like the C code, a PLL lock failure during filter calibration aborts the
     * remaining configuration but is not treated as a fatal init error.
     */
    private fun setTvStandard(): Int {
        val filtCalLo = 56000       /* kHz */
        val filtGain = 0x30         /* +3db, 6mhz on */
        val imgR = 0x00             /* image negative */
        val filtQ = 0x10            /* r10[4]:low q(1'b1) */
        val hpCor = 0x6b            /* 1.7m disable, +2cap, 1.0mhz */
        val extEnable = 0x60        /* r30[6]=1 ext enable */
        val loopThrough = 0x80      /* r5[7], lt off */
        val ltAtt = 0x00            /* r31[7], lt att enable */
        val fltExtWidest = 0x00     /* r15[7]: flt_ext_wide off */
        val polyfilCur = 0x60       /* r25[6:5]:min */

        /* Initialize the shadow registers */
        shadowStore(REG_SHADOW_START, INIT_ARRAY, 0, INIT_ARRAY.size)

        /* Init Flag & Xtal_check Result */
        var rc = writeRegMask(0x0c, 0x00, 0x0f)
        if (rc < 0) return rc

        /* version */
        rc = writeRegMask(0x13, VER_NUM, 0x3f)
        if (rc < 0) return rc

        /* for LT Gain test */
        rc = writeRegMask(0x1d, 0x00, 0x38)
        if (rc < 0) return rc

        intFreq = 3570 * 1000

        /* Filter calibration */
        for (i in 0 until 2) {
            /* Set filt_cap */
            rc = writeRegMask(0x0b, hpCor, 0x60)
            if (rc < 0) return rc

            /* set cali clk = on */
            rc = writeRegMask(0x0f, 0x04, 0x04)
            if (rc < 0) return rc

            /* X'tal cap 0pF for PLL */
            rc = writeRegMask(0x10, 0x00, 0x03)
            if (rc < 0) return rc

            rc = setPll(filtCalLo * 1000L)
            if (rc < 0 || !hasLock) return rc

            /* Start Trigger */
            rc = writeRegMask(0x0b, 0x10, 0x10)
            if (rc < 0) return rc

            /* Stop Trigger */
            rc = writeRegMask(0x0b, 0x00, 0x10)
            if (rc < 0) return rc

            /* set cali clk = off */
            rc = writeRegMask(0x0f, 0x00, 0x04)
            if (rc < 0) return rc

            /* Check if calibration worked */
            val data = read(0x00, 5) ?: return -1
            filCalCode = data[4] and 0x0f
            if (filCalCode != 0 && filCalCode != 0x0f) break
        }

        /* narrowest */
        if (filCalCode == 0x0f) filCalCode = 0

        rc = writeRegMask(0x0a, filtQ or filCalCode, 0x1f)
        if (rc < 0) return rc

        /* Set BW, Filter_gain & HP corner */
        rc = writeRegMask(0x0b, hpCor, 0xef)
        if (rc < 0) return rc

        /* Set Img_R */
        rc = writeRegMask(0x07, imgR, 0x80)
        if (rc < 0) return rc

        /* Set filt_3dB, V6MHz */
        rc = writeRegMask(0x06, filtGain, 0x30)
        if (rc < 0) return rc

        /* channel filter extension */
        rc = writeRegMask(0x1e, extEnable, 0x60)
        if (rc < 0) return rc

        /* Loop through */
        rc = writeRegMask(0x05, loopThrough, 0x80)
        if (rc < 0) return rc

        /* Loop through attenuation */
        rc = writeRegMask(0x1f, ltAtt, 0x80)
        if (rc < 0) return rc

        /* filter extension widest */
        rc = writeRegMask(0x0f, fltExtWidest, 0x80)
        if (rc < 0) return rc

        /* RF poly filter current */
        return writeRegMask(0x19, polyfilCur, 0x60)
    }

    /** Reads the currently applied gain, in tenths of dB steps (r82xx_read_gain). */
    fun readGain(): Int {
        val data = read(0x00, 4) ?: return -1
        return ((data[3] and 0x0f) shl 1) + ((data[3] and 0xf0) shr 4)
    }

    private fun setVgaGain(): Int {
        /* Fixed VGA gain (16.3 dB), see tuner_r82xx.c */
        return writeRegMask(0x0c, 0x08, 0x9f)
    }

    /**
     * r82xx_set_gain: manual gain in tenths of dB, or hardware AGC.
     */
    fun setGain(manualGain: Boolean, gain: Int = 0): Int {
        var rc: Int
        if (manualGain) {
            /* LNA auto off */
            rc = writeRegMask(0x05, 0x10, 0x10)
            if (rc < 0) return rc

            /* Mixer auto off */
            rc = writeRegMask(0x07, 0, 0x10)
            if (rc < 0) return rc

            read(0x00, 4) ?: return -1

            rc = setVgaGain()
            if (rc < 0) return rc

            var totalGain = 0
            var lnaIndex = 0
            var mixIndex = 0
            for (i in 0 until 15) {
                if (totalGain >= gain) break
                totalGain += LNA_GAIN_STEPS[++lnaIndex]
                if (totalGain >= gain) break
                totalGain += MIXER_GAIN_STEPS[++mixIndex]
            }

            /* set LNA gain */
            rc = writeRegMask(0x05, lnaIndex, 0x0f)
            if (rc < 0) return rc

            /* set Mixer gain */
            rc = writeRegMask(0x07, mixIndex, 0x0f)
            if (rc < 0) return rc
        } else {
            /* LNA auto on */
            rc = writeRegMask(0x05, 0, 0x10)
            if (rc < 0) return rc

            /* Mixer auto on */
            rc = writeRegMask(0x07, 0x10, 0x10)
            if (rc < 0) return rc

            /* set fixed VGA gain for now (26.5 dB) */
            rc = writeRegMask(0x0c, 0x0b, 0x9f)
            if (rc < 0) return rc
        }
        return 0
    }

    /**
     * r82xx_set_bandwidth. Returns the new IF frequency (Hz) on success,
     * negative on failure. The caller must reprogram the RTL2832U IF frequency
     * and retune afterwards.
     */
    override fun setBandwidth(bwHz: Int): Int {
        var bw = bwHz
        val reg0a: Int
        var reg0b: Int

        if (bw > 7_000_000) {
            /* BW: 8 MHz */
            reg0a = 0x10
            reg0b = 0x0b
            intFreq = 4_570_000
        } else if (bw > 6_000_000) {
            /* BW: 7 MHz */
            reg0a = 0x10
            reg0b = 0x2a
            intFreq = 4_570_000
        } else if (bw > IF_LOW_PASS_BW_TABLE[0] + FILT_HP_BW1 + FILT_HP_BW2) {
            /* BW: 6 MHz */
            reg0a = 0x10
            reg0b = 0x6b
            intFreq = 3_570_000
        } else {
            reg0a = 0x00
            reg0b = 0x80
            intFreq = 2_300_000
            var realBw = 0

            if (bw > IF_LOW_PASS_BW_TABLE[0] + FILT_HP_BW1) {
                bw -= FILT_HP_BW2
                intFreq += FILT_HP_BW2
                realBw += FILT_HP_BW2
            } else {
                reg0b = reg0b or 0x20
            }

            if (bw > IF_LOW_PASS_BW_TABLE[0]) {
                bw -= FILT_HP_BW1
                intFreq += FILT_HP_BW1
                realBw += FILT_HP_BW1
            } else {
                reg0b = reg0b or 0x40
            }

            /* find low-pass filter */
            var i = 0
            while (i < IF_LOW_PASS_BW_TABLE.size) {
                if (bw > IF_LOW_PASS_BW_TABLE[i]) break
                i++
            }
            i--
            reg0b = reg0b or (15 - i)
            realBw += IF_LOW_PASS_BW_TABLE[i]

            intFreq -= realBw / 2
        }

        var rc = writeRegMask(0x0a, reg0a, 0x10)
        if (rc < 0) return rc
        rc = writeRegMask(0x0b, reg0b, 0xef)
        if (rc < 0) return rc

        return intFreq
    }

    /**
     * r82xx_set_freq: tunes to [freqHz] (RF frequency, before IF offset).
     */
    override fun setFreq(freqHz: Long): Int {
        /* RTL-SDR Blog V4/V4L: automatically upconvert by 28.8 MHz on HF */
        val upconvertFreq = if (isBlogV4 || isBlogV4L) {
            if (freqHz < MHZ_28_8) freqHz + MHZ_28_8 else freqHz
        } else {
            freqHz
        }

        val loFreq = upconvertFreq + intFreq

        var rc = setMux(loFreq)
        if (rc < 0) return err(rc)

        rc = setVgaGain()
        if (rc < 0) return err(rc)

        rc = setPll(loFreq)
        if (rc < 0 || !hasLock) return err(rc)

        if (isBlogV4) {
            /* Notches OFF when tuned inside a notch band, ON outside */
            val openD = if (freqHz <= 2_200_000L ||
                (freqHz in 85_000_000L..112_000_000L) ||
                (freqHz in 172_000_000L..242_000_000L)
            ) 0x00 else 0x08
            rc = writeRegMask(0x17, openD, 0x08)
            if (rc < 0) return rc

            val band = when {
                freqHz <= MHZ_28_8 -> BAND_HF
                freqHz < 250_000_000L -> BAND_VHF
                else -> BAND_UHF
            }

            /* Bypass tracking filter on HF (upconverter path) */
            if (band == BAND_HF) {
                rc = writeRegMask(0x1a, 0x40, 0xc3)
                if (rc < 0) return err(rc)
                rc = writeReg(0x1b, 0x00)
                if (rc < 0) return err(rc)
            }

            if (band != input) {
                input = band

                /* activate cable 2 (HF input) */
                val cable2In = if (band == BAND_HF) 0x08 else 0x00
                rc = writeRegMask(0x06, cable2In, 0x08)
                if (rc < 0) return err(rc)

                /* Control upconverter GPIO switch on newer batches */
                setGpio(5, cable2In == 0)

                /* activate cable 1 (VHF input) */
                rc = writeRegMask(0x05, if (band == BAND_VHF) 0x40 else 0x00, 0x40)
                if (rc < 0) return err(rc)

                /* activate air_in (UHF input) */
                rc = writeRegMask(0x05, if (band == BAND_UHF) 0x00 else 0x20, 0x20)
                if (rc < 0) return err(rc)
            }
        } else if (isBlogV4L) {
            val band = if (freqHz <= MHZ_28_8) BAND_HF else BAND_UHF

            /* Bypass tracking filter on HF */
            if (band == BAND_HF) {
                rc = writeRegMask(0x1a, 0x40, 0xc3)
                if (rc < 0) return err(rc)
                rc = writeReg(0x1b, 0x00)
                if (rc < 0) return err(rc)
            }

            if (band != input) {
                input = band

                val cable1In = if (band == BAND_HF) 0x40 else 0x00
                setGpio(5, cable1In == 0)

                /* activate cable 1 (HF input) */
                rc = writeRegMask(0x05, cable1In, 0x40)
                if (rc < 0) return err(rc)

                /* activate air_in (UHF input) */
                rc = writeRegMask(0x05, if (band == BAND_UHF) 0x00 else 0x20, 0x20)
                if (rc < 0) return err(rc)
            }
        } else {
            /* Standard R828D dongle: switch between 'Cable1' and 'Air-In'
             * at 345 MHz, where both have similar noise floors. */
            val airCable1In = if (freqHz > 345_000_000L) 0x00 else 0x60
            if (chip == Chip.R828D && airCable1In != input) {
                input = airCable1In
                rc = writeRegMask(0x05, airCable1In, 0x60)
                if (rc < 0) return err(rc)
            }
        }

        return rc
    }

    private fun err(rc: Int): Int {
        if (rc < 0) Log.e(TAG, "tuner operation failed=$rc")
        return rc
    }

    /** r82xx_standby: puts the tuner in low-power state. */
    fun standby(): Int {
        if (!initDone) return 0

        var rc = writeReg(0x06, 0xb1)
        if (rc < 0) return rc
        rc = writeReg(0x05, 0xa0)
        if (rc < 0) return rc
        rc = writeReg(0x07, 0x3a)
        if (rc < 0) return rc
        rc = writeReg(0x08, 0x40)
        if (rc < 0) return rc
        rc = writeReg(0x09, 0xc0)
        if (rc < 0) return rc
        rc = writeReg(0x0a, 0x36)
        if (rc < 0) return rc
        rc = writeReg(0x0c, 0x35)
        if (rc < 0) return rc
        rc = writeReg(0x0f, 0x68)
        if (rc < 0) return rc
        rc = writeReg(0x11, 0x03)
        if (rc < 0) return rc
        rc = writeReg(0x17, 0xf4)
        if (rc < 0) return rc
        return writeReg(0x19, 0x0c)
    }

    /** r82xx_init: full initialization sequence. */
    override fun init(): Int {
        /* Initialize registers */
        write(REG_SHADOW_START, INIT_ARRAY)

        var rc = setTvStandard()
        if (rc < 0) return err(rc)

        rc = sysfreqSel()
        initDone = true

        return err(rc)
    }
}
