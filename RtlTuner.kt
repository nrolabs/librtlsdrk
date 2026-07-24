/*
 * librtlsdrk - Kotlin port of librtlsdr (RTL-SDR Blog fork) for Android USB host
 *
 * Tuner interface derived from rtlsdr_tuner_iface_t in librtlsdr.c:
 * Copyright (C) 2012-2014 by Steve Markgraf <steve@steve-m.de>
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
 * Common tuner interface, mirroring rtlsdr_tuner_iface_t in librtlsdr.c.
 * All methods return 0 on success, negative on failure. The RTL2832U driver
 * manages the I2C repeater around every call.
 */
interface RtlTuner {
    /** Human-readable name for logs/UI. */
    val name: String

    /** Available gain values, in tenths of a dB (rtlsdr_get_tuner_gains). */
    val gains: IntArray

    fun init(): Int

    /** Standby / power down (tuner->exit). */
    fun exit(): Int

    /** Tunes to [freqHz] (RF frequency, before any IF offset). */
    fun setFreq(freqHz: Long): Int

    /**
     * Sets the IF filter bandwidth. Returns the tuner's new IF frequency in Hz
     * when the RTL2832U IF must be reprogrammed afterwards (R82xx), 0 when no
     * IF change is needed, or negative on error.
     */
    fun setBandwidth(bwHz: Int): Int = 0

    /** Manual gain in tenths of dB. */
    fun setGain(gainTenthDb: Int): Int = 0

    fun setIfGain(stage: Int, gainTenthDb: Int): Int = 0

    fun setGainMode(manual: Boolean): Int = 0
}

/**
 * Hardware access callbacks handed to tuner drivers by the RTL2832U driver.
 * Equivalent to the rtlsdr_i2c_*_fn / rtlsdr_set_gpio_* functions the C tuner
 * drivers call back into librtlsdr with.
 *
 * I2C addresses are the 8-bit form used by the C headers (e.g. E4K 0xc8).
 */
class TunerContext(
    /** Raw I2C write; returns bytes written (including none on failure <0). */
    val i2cWrite: (addr: Int, data: ByteArray) -> Int,
    /** Raw I2C read of [len] bytes, or null on failure. */
    val i2cRead: (addr: Int, len: Int) -> ByteArray?,
    /** rtlsdr_i2c_write_reg. Returns bytes written, <0 on failure. */
    val i2cWriteReg: (addr: Int, reg: Int, value: Int) -> Int,
    /** rtlsdr_i2c_read_reg. Returns the byte, or -1 on failure. */
    val i2cReadReg: (addr: Int, reg: Int) -> Int,
    /** rtlsdr_get_tuner_clock: PPM-corrected tuner crystal frequency in Hz. */
    val getTunerClock: () -> Long,
    /** rtlsdr_set_gpio_bit. */
    val setGpioBit: (gpio: Int, on: Boolean) -> Unit,
    /** rtlsdr_set_gpio_output. */
    val setGpioOutput: (gpio: Int) -> Unit,
)
