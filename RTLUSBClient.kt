/*
 * librtlsdrk - Kotlin port of librtlsdr (RTL-SDR Blog fork) for Android USB host
 *
 * Based on librtlsdr.c:
 * Copyright (C) 2012-2014 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Dimitri Stolnikov <horiz0n@gmx.net>
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

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbConstants
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbDeviceConnection
import android.hardware.usb.UsbEndpoint
import android.hardware.usb.UsbInterface
import android.hardware.usb.UsbManager
import android.util.Log
import androidx.core.content.ContextCompat
import com.isaklab.isdrdrivers.core.FFTProcessor
import com.isaklab.isdrdrivers.core.SDRConfig
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.asCoroutineDispatcher
import kotlinx.coroutines.cancel
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.withContext
import java.util.concurrent.Executors
import kotlin.coroutines.resume
import kotlinx.coroutines.suspendCancellableCoroutine

/**
 * rtl_tcp compatible command, shared by the TCP and USB backends.
 */
data class RTLCommand(val cmd: Byte, val param: Int) {
    companion object {
        const val CMD_SET_FREQUENCY: Byte = 0x01
        const val CMD_SET_SAMPLE_RATE: Byte = 0x02
        const val CMD_SET_GAIN_MODE: Byte = 0x03
        const val CMD_SET_GAIN: Byte = 0x04
        const val CMD_SET_PPM: Byte = 0x05
        const val CMD_SET_AGC: Byte = 0x08
        const val CMD_SET_DIRECT_SAMPLING: Byte = 0x09
        const val CMD_SET_BIAS_TEE: Byte = 0x0e

        fun SetFrequency(freqHz: Long) = RTLCommand(CMD_SET_FREQUENCY, freqHz.toInt())
        fun SetSampleRate(rateHz: Long) = RTLCommand(CMD_SET_SAMPLE_RATE, rateHz.toInt())
        fun SetGainMode(manual: Boolean) = RTLCommand(CMD_SET_GAIN_MODE, if (manual) 1 else 0)
        fun SetGain(gainTenthsOfDb: Int) = RTLCommand(CMD_SET_GAIN, gainTenthsOfDb)
        fun SetPPMCorrection(ppm: Int) = RTLCommand(CMD_SET_PPM, ppm)
        fun SetAGC(enabled: Boolean) = RTLCommand(CMD_SET_AGC, if (enabled) 1 else 0)
        fun SetBiasTee(enabled: Boolean) = RTLCommand(CMD_SET_BIAS_TEE, if (enabled) 1 else 0)
    }
}

/**
 * RTL2832U USB driver.
 *
 * Kotlin port of librtlsdr.c (RTL-SDR Blog fork) on top of the Android USB host
 * API. All register access is serialized on a single-threaded dispatcher; the
 * bulk IQ stream runs on its own IO coroutine, which is safe because control
 * and bulk transfers use different endpoints.
 *
 * Instances are single-use: after [disconnect] (or a failed [connect]) the
 * internal executor is shut down and a new instance must be created.
 */
class RTLUSBClient(
    private val context: Context,
    /** (power spectrum in dB, interleaved IQ samples i0,q0,i1,q1,... in [-1,1]) */
    private val onDataReceived: (FloatArray, FloatArray) -> Unit,
    private val onConnectionStatusChanged: (Boolean, String) -> Unit,
) {
    companion object {
        private const val TAG = "RTLUSBClient"
        private val EMPTY_SPECTRUM = FloatArray(0)

        private const val DEF_RTL_XTAL_FREQ = 28_800_000
        private const val R828D_XTAL_FREQ = 16_000_000
        private const val R82XX_IF_FREQ = R82xxTuner.R82XX_IF_FREQ

        private const val CTRL_TIMEOUT = 300
        private const val BULK_TIMEOUT = 500
        private const val BULK_BUFFER_SIZE = 64 * 1024  /* multiple of 512 */

        private const val EEPROM_ADDR = 0xa0
        private const val EEPROM_SIZE = 256

        private val CTRL_IN = UsbConstants.USB_TYPE_VENDOR or UsbConstants.USB_DIR_IN
        private val CTRL_OUT = UsbConstants.USB_TYPE_VENDOR or UsbConstants.USB_DIR_OUT

        /* usb_reg */
        private const val USB_SYSCTL = 0x2000
        private const val USB_EPA_CTL = 0x2148
        private const val USB_EPA_MAXPKT = 0x2158

        /* sys_reg */
        private const val DEMOD_CTL = 0x3000
        private const val GPO = 0x3001
        private const val GPOE = 0x3003
        private const val GPD = 0x3004
        private const val DEMOD_CTL_1 = 0x300b

        /* blocks */
        private const val USBB = 1
        private const val SYSB = 2
        private const val IICB = 6

        /* tuner probe addresses */
        private const val E4K_I2C_ADDR = 0xc8
        private const val E4K_CHECK_ADDR = 0x02
        private const val E4K_CHECK_VAL = 0x40
        private const val FC0012_I2C_ADDR = 0xc6
        private const val FC0012_CHECK_ADDR = 0x00
        private const val FC0012_CHECK_VAL = 0xa1
        private const val FC0013_I2C_ADDR = 0xc6
        private const val FC0013_CHECK_ADDR = 0x00
        private const val FC0013_CHECK_VAL = 0xa3
        private const val FC2580_I2C_ADDR = 0xac
        private const val FC2580_CHECK_ADDR = 0x01
        private const val FC2580_CHECK_VAL = 0x56

        /*
         * FIR coefficients, running at XTal frequency (see librtlsdr.c).
         * First 8 are 8-bit signed, next 8 are 12-bit signed.
         */
        private val FIR_DEFAULT = intArrayOf(
            -54, -36, -41, -40, -32, -14, 14, 53,
            101, 156, 215, 273, 327, 372, 404, 421
        )

        data class Dongle(val vid: Int, val pid: Int, val name: String)

        /** known_devices from librtlsdr.c */
        val KNOWN_DEVICES = listOf(
            Dongle(0x0bda, 0x2832, "Generic RTL2832U"),
            Dongle(0x0bda, 0x2838, "Generic RTL2832U OEM"),
            Dongle(0x0413, 0x6680, "DigitalNow Quad DVB-T PCI-E card"),
            Dongle(0x0413, 0x6f0f, "Leadtek WinFast DTV Dongle mini D"),
            Dongle(0x0458, 0x707f, "Genius TVGo DVB-T03 USB dongle (Ver. B)"),
            Dongle(0x0ccd, 0x00a9, "Terratec Cinergy T Stick Black (rev 1)"),
            Dongle(0x0ccd, 0x00b3, "Terratec NOXON DAB/DAB+ USB dongle (rev 1)"),
            Dongle(0x0ccd, 0x00b4, "Terratec Deutschlandradio DAB Stick"),
            Dongle(0x0ccd, 0x00b5, "Terratec NOXON DAB Stick - Radio Energy"),
            Dongle(0x0ccd, 0x00b7, "Terratec Media Broadcast DAB Stick"),
            Dongle(0x0ccd, 0x00b8, "Terratec BR DAB Stick"),
            Dongle(0x0ccd, 0x00b9, "Terratec WDR DAB Stick"),
            Dongle(0x0ccd, 0x00c0, "Terratec MuellerVerlag DAB Stick"),
            Dongle(0x0ccd, 0x00c6, "Terratec Fraunhofer DAB Stick"),
            Dongle(0x0ccd, 0x00d3, "Terratec Cinergy T Stick RC (Rev.3)"),
            Dongle(0x0ccd, 0x00d7, "Terratec T Stick PLUS"),
            Dongle(0x0ccd, 0x00e0, "Terratec NOXON DAB/DAB+ USB dongle (rev 2)"),
            Dongle(0x1554, 0x5020, "PixelView PV-DT235U(RN)"),
            Dongle(0x15f4, 0x0131, "Astrometa DVB-T/DVB-T2"),
            Dongle(0x15f4, 0x0133, "HanfTek DAB+FM+DVB-T"),
            Dongle(0x185b, 0x0620, "Compro Videomate U620F"),
            Dongle(0x185b, 0x0650, "Compro Videomate U650F"),
            Dongle(0x185b, 0x0680, "Compro Videomate U680F"),
            Dongle(0x1b80, 0xd393, "GIGABYTE GT-U7300"),
            Dongle(0x1b80, 0xd394, "DIKOM USB-DVBT HD"),
            Dongle(0x1b80, 0xd395, "Peak 102569AGPK"),
            Dongle(0x1b80, 0xd397, "KWorld KW-UB450-T USB DVB-T Pico TV"),
            Dongle(0x1b80, 0xd398, "Zaapa ZT-MINDVBZP"),
            Dongle(0x1b80, 0xd39d, "SVEON STV20 DVB-T USB & FM"),
            Dongle(0x1b80, 0xd3a4, "Twintech UT-40"),
            Dongle(0x1b80, 0xd3a8, "ASUS U3100MINI_PLUS_V2"),
            Dongle(0x1b80, 0xd3af, "SVEON STV27 DVB-T USB & FM"),
            Dongle(0x1b80, 0xd3b0, "SVEON STV21 DVB-T USB & FM"),
            Dongle(0x1d19, 0x1101, "Dexatek DK DVB-T Dongle (Logilink VG0002A)"),
            Dongle(0x1d19, 0x1102, "Dexatek DK DVB-T Dongle (MSI DigiVox mini II V3.0)"),
            Dongle(0x1d19, 0x1103, "Dexatek Technology Ltd. DK 5217 DVB-T Dongle"),
            Dongle(0x1d19, 0x1104, "MSI DigiVox Micro HD"),
            Dongle(0x1f4d, 0xa803, "Sweex DVB-T USB"),
            Dongle(0x1f4d, 0xb803, "GTek T803"),
            Dongle(0x1f4d, 0xc803, "Lifeview LV5TDeluxe"),
            Dongle(0x1f4d, 0xd286, "MyGica TD312"),
            Dongle(0x1f4d, 0xd803, "PROlectrix DV107669"),
        )

        fun findKnownDevice(vid: Int, pid: Int): Dongle? =
            KNOWN_DEVICES.find { it.vid == vid && it.pid == pid }
    }

    enum class Tuner { UNKNOWN, E4000, FC0012, FC0013, FC2580, R820T, R828D }

    // ==================== State (mirrors struct rtlsdr_dev) ====================

    private val usbManager = context.getSystemService(Context.USB_SERVICE) as UsbManager
    private var usbDevice: UsbDevice? = null
    private var connection: UsbDeviceConnection? = null
    private var usbInterface: UsbInterface? = null
    private var bulkEndpoint: UsbEndpoint? = null

    @Volatile private var isOpen = false
    @Volatile private var devLost = false
    @Volatile private var isConnected = false

    private var rate = 0                       /* Hz */
    private var rtlXtal = DEF_RTL_XTAL_FREQ    /* Hz */
    private var tunXtal = DEF_RTL_XTAL_FREQ    /* Hz */
    private var freq = 0L                      /* Hz */
    private var bw = 0
    private var offsFreq = 0                   /* Hz */
    private var corr = 0                       /* ppm */
    private var gain = 0                       /* tenth dB */
    private var directSampling = 0
    private var directSamplingMode = 0
    private var forceBiasTee = false
    private var manufact = ""
    private var product = ""

    private var tunerType = Tuner.UNKNOWN
    private var tuner: RtlTuner? = null

    private val fir = FIR_DEFAULT.clone()

    /** All control transfers are funneled through this single-threaded dispatcher. */
    private val usbExecutor = Executors.newSingleThreadExecutor { r ->
        Thread(r, "rtlsdr-usb").apply { priority = Thread.NORM_PRIORITY + 1 }
    }
    private val usbDispatcher = usbExecutor.asCoroutineDispatcher()
    private val scope = CoroutineScope(SupervisorJob() + usbDispatcher)

    private var streamJob: Job? = null
    private var fftProcessor: FFTProcessor? = null

    /**
     * When false, IQ blocks are delivered with an empty spectrum and the FFT
     * is skipped (the host has no visible spectrum consumer). Audio delivery
     * is unaffected.
     */
    @Volatile var spectrumEnabled: Boolean = true

    /** Coalesces rapid tuning requests from the UI. */
    @Volatile private var latestRequestedFreq = 0L

    private var permissionReceiver: BroadcastReceiver? = null
    @Volatile private var pendingPermission: kotlinx.coroutines.CancellableContinuation<Boolean>? = null
    @Volatile private var released = false
    private val actionUsbPermission = "${context.packageName}.USB_PERMISSION"

    // ==================== Public API ====================

    suspend fun connect(): Boolean = withContext(Dispatchers.IO) {
        if (released) {
            Log.e(TAG, "connect() called on a released client; create a new instance")
            return@withContext false
        }
        try {
            onConnectionStatusChanged(false, "Searching for RTL-SDR devices...")

            val device = usbManager.deviceList.values.firstOrNull {
                findKnownDevice(it.vendorId, it.productId) != null
            }
            if (device == null) {
                onConnectionStatusChanged(false, "No RTL-SDR devices found")
                release()
                return@withContext false
            }

            val name = findKnownDevice(device.vendorId, device.productId)!!.name
            Log.i(TAG, "Found $name (${device.vendorId.toString(16)}:${device.productId.toString(16)})")
            onConnectionStatusChanged(false, "Found: $name")

            if (!usbManager.hasPermission(device) && !requestPermission(device)) {
                onConnectionStatusChanged(false, "USB permission denied")
                release()
                return@withContext false
            }

            usbDevice = device
            onConnectionStatusChanged(false, "Opening device...")

            val opened = withContext(usbDispatcher) { open() }
            if (!opened) {
                onConnectionStatusChanged(false, "Failed to open device")
                release()
                return@withContext false
            }

            withContext(usbDispatcher) { applyStreamingDefaults() }

            fftProcessor = FFTProcessor(SDRConfig.FFT_SIZE)
            isConnected = true
            startStreaming()

            onConnectionStatusChanged(true, "Connected - Tuner: ${tunerType.name}")
            true
        } catch (e: Exception) {
            Log.e(TAG, "Connection failed", e)
            onConnectionStatusChanged(false, "Failed: ${e.message}")
            disconnect()
            false
        }
    }

    fun disconnect() {
        isConnected = false
        if (released) return
        scope.launch {
            try {
                close()
                onConnectionStatusChanged(false, "Disconnected")
            } catch (e: Exception) {
                Log.e(TAG, "Error during disconnect", e)
            } finally {
                release()
            }
        }
    }

    /** Frees the worker thread and cancels any pending permission request. */
    private fun release() {
        if (released) return
        released = true
        pendingPermission?.let {
            pendingPermission = null
            if (it.isActive) it.resume(false)
        }
        unregisterPermissionReceiver()
        scope.cancel()
        usbExecutor.shutdown()
    }

    /**
     * Queues an rtl_tcp style command; executed on the USB dispatcher without
     * interrupting the IQ stream.
     */
    fun sendCommand(command: RTLCommand) {
        if (!isOpen) return

        if (command.cmd == RTLCommand.CMD_SET_FREQUENCY) {
            latestRequestedFreq = command.param.toLong() and 0xffffffffL
        }

        scope.launch {
            if (!isOpen || devLost) return@launch
            try {
                executeCommand(command)
            } catch (e: Exception) {
                Log.e(TAG, "Command 0x${command.cmd.toString(16)} failed", e)
            }
        }
    }

    fun setFrequency(frequencyHz: Long) = sendCommand(RTLCommand.SetFrequency(frequencyHz))
    fun setGainMode(manual: Boolean) = sendCommand(RTLCommand.SetGainMode(manual))
    fun setGain(gainTenthsOfDb: Int) = sendCommand(RTLCommand.SetGain(gainTenthsOfDb))
    fun setDirectSamplingMode(mode: Int) = sendCommand(RTLCommand(RTLCommand.CMD_SET_DIRECT_SAMPLING, mode))

    fun getCenterFrequency(): Long = freq
    fun getSampleRate(): Double = if (rate != 0) rate.toDouble() else SDRConfig.DEFAULT_SAMPLE_RATE_HZ.toDouble()
    fun getTunerGain(): Int = gain
    fun getFrequencyCorrection(): Int = corr
    fun getTunerType(): Tuner = tunerType
    fun getDirectSampling(): Int = directSampling

    /** All gain values are expressed in tenths of a dB (rtlsdr_get_tuner_gains). */
    fun getTunerGains(): IntArray = tuner?.gains ?: intArrayOf(0)

    fun setSmoothingFactor(alpha: Float) {
        fftProcessor?.setSmoothingFactor(alpha)
    }

    fun getStatusSummary(): String = if (isConnected) {
        "Connected - Tuner: ${tunerType.name}, Freq: ${freq / 1e6} MHz, " +
            "Gain: ${gain / 10.0} dB, Rate: $rate Hz"
    } else {
        "Not connected"
    }

    // ==================== Command execution (USB dispatcher only) ====================

    private fun executeCommand(command: RTLCommand) {
        when (command.cmd) {
            RTLCommand.CMD_SET_FREQUENCY -> {
                val requested = command.param.toLong() and 0xffffffffL
                /* skip stale tuning requests that were superseded meanwhile */
                if (requested != latestRequestedFreq) return
                fftProcessor?.resetSmoothing()
                setCenterFreq(requested)
            }
            RTLCommand.CMD_SET_SAMPLE_RATE -> {
                fftProcessor?.resetSmoothing()
                setSampleRate(command.param)
            }
            RTLCommand.CMD_SET_GAIN_MODE -> setTunerGainMode(command.param != 0)
            RTLCommand.CMD_SET_GAIN -> setTunerGain(command.param)
            RTLCommand.CMD_SET_PPM -> setFreqCorrection(command.param)
            RTLCommand.CMD_SET_AGC -> setAgcMode(command.param != 0)
            RTLCommand.CMD_SET_DIRECT_SAMPLING -> setDirectSampling(command.param)
            RTLCommand.CMD_SET_BIAS_TEE -> setBiasTee(command.param != 0)
            else -> Log.w(TAG, "Unsupported command 0x${command.cmd.toString(16)}")
        }
    }

    // ==================== USB permission ====================

    private suspend fun requestPermission(device: UsbDevice): Boolean =
        suspendCancellableCoroutine { continuation ->
            pendingPermission = continuation
            val receiver = object : BroadcastReceiver() {
                override fun onReceive(ctx: Context, intent: Intent) {
                    if (intent.action != actionUsbPermission) return
                    @Suppress("DEPRECATION")
                    val grantedDevice: UsbDevice? =
                        intent.getParcelableExtra(UsbManager.EXTRA_DEVICE)
                    if (grantedDevice != null && grantedDevice.deviceName != device.deviceName) {
                        return  /* grant for a different device */
                    }
                    unregisterPermissionReceiver()
                    pendingPermission = null
                    val granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)
                    if (continuation.isActive) continuation.resume(granted)
                }
            }
            permissionReceiver = receiver
            ContextCompat.registerReceiver(
                context, receiver, IntentFilter(actionUsbPermission),
                ContextCompat.RECEIVER_NOT_EXPORTED
            )

            /* The system fills in extras, so the PendingIntent must be mutable
             * and the intent explicit (package-scoped) on modern Android. */
            val flags = if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.S) {
                PendingIntent.FLAG_MUTABLE
            } else {
                0
            }
            val pendingIntent = PendingIntent.getBroadcast(
                context, 0,
                Intent(actionUsbPermission).setPackage(context.packageName),
                flags
            )
            usbManager.requestPermission(device, pendingIntent)

            continuation.invokeOnCancellation {
                pendingPermission = null
                unregisterPermissionReceiver()
            }
        }

    private fun unregisterPermissionReceiver() {
        permissionReceiver?.let {
            try {
                context.unregisterReceiver(it)
            } catch (_: Exception) {
                /* already unregistered */
            }
            permissionReceiver = null
        }
    }

    // ==================== Open / close (port of rtlsdr_open/rtlsdr_close) ====================

    private fun open(): Boolean {
        if (isOpen) return true
        val device = usbDevice ?: return false

        val conn = usbManager.openDevice(device) ?: run {
            Log.e(TAG, "openDevice returned null")
            return false
        }
        connection = conn

        if (device.interfaceCount == 0) return false
        val iface = device.getInterface(0)
        if (!conn.claimInterface(iface, true)) {
            Log.e(TAG, "claimInterface failed")
            conn.close()
            connection = null
            return false
        }
        usbInterface = iface

        bulkEndpoint = (0 until iface.endpointCount)
            .map { iface.getEndpoint(it) }
            .firstOrNull {
                it.type == UsbConstants.USB_ENDPOINT_XFER_BULK &&
                    it.direction == UsbConstants.USB_DIR_IN
            }
        if (bulkEndpoint == null) {
            Log.e(TAG, "No bulk IN endpoint found")
            conn.releaseInterface(iface)
            conn.close()
            connection = null
            return false
        }

        rtlXtal = DEF_RTL_XTAL_FREQ
        devLost = false

        /* Dummy write; if it fails the device may need a reset. */
        if (writeReg(USBB, USB_SYSCTL, 0x09, 1) < 0) {
            Log.w(TAG, "Initial write failed, device may be in a bad state")
        }

        initBaseband()

        manufact = device.manufacturerName ?: ""
        product = device.productName ?: ""
        Log.i(TAG, "USB strings: manufact='$manufact' product='$product'")

        probeTunerAndInit()

        isOpen = true
        Log.i(TAG, "Device opened, tuner=${tunerType.name}, tunXtal=$tunXtal")
        return true
    }

    private fun close() {
        stopStreamingBlocking()

        if (isOpen && !devLost) {
            deinitBaseband()
        }
        isOpen = false

        connection?.let { conn ->
            usbInterface?.let { conn.releaseInterface(it) }
            conn.close()
        }
        connection = null
        usbInterface = null
        bulkEndpoint = null
        tuner = null
        tunerType = Tuner.UNKNOWN
        unregisterPermissionReceiver()
        Log.i(TAG, "Device closed")
    }

    /**
     * Tuner probing and initialization, mirroring the tail of rtlsdr_open().
     * The I2C repeater stays enabled for the whole sequence.
     */
    private fun probeTunerAndInit() {
        setI2cRepeater(true)

        tunerType = probeTuner()

        /* use the rtl clock value by default */
        tunXtal = rtlXtal

        val isBlogV4 = checkDongleModel("RTLSDRBlog", "Blog V4")
        val isBlogV4L = checkDongleModel("RTLSDRBlog", "Blog V4L")
        if (isBlogV4) Log.i(TAG, "RTL-SDR Blog V4 detected")
        if (isBlogV4L) Log.i(TAG, "RTL-SDR Blog V4 Lite detected")

        when (tunerType) {
            Tuner.R828D, Tuner.R820T -> {
                if (tunerType == Tuner.R828D && !isBlogV4) {
                    /* Typical R828D 16 MHz crystal (Blog V4 keeps 28.8 MHz) */
                    tunXtal = R828D_XTAL_FREQ
                }

                /* disable Zero-IF mode */
                demodWriteReg(1, 0xb1, 0x1a, 1)
                /* only enable In-phase ADC input */
                demodWriteReg(0, 0x08, 0x4d, 1)
                /* the R82xx uses a 3.57 MHz IF for the DVB-T 6 MHz mode */
                setIfFreq(R82XX_IF_FREQ)
                /* enable spectrum inversion */
                demodWriteReg(1, 0x15, 0x01, 1)
            }
            Tuner.UNKNOWN -> {
                Log.w(TAG, "No supported tuner found, enabling direct sampling")
            }
            else -> {}
        }

        /* Force-bias-tee hack: IR-endpoint bit 0 in EEPROM byte 7. Only trust
         * the flag if the EEPROM actually read back successfully. */
        val eeprom = ByteArray(EEPROM_SIZE)
        forceBiasTee = readEeprom(eeprom, 0, EEPROM_SIZE) &&
            (eeprom[7].toInt() and 0x02) == 0
        if (forceBiasTee) {
            Log.i(TAG, "EEPROM requests forced bias tee")
            setBiasTee(true)
        }

        val ctx = TunerContext(
            i2cWrite = { addr, data -> i2cWrite(addr, data) },
            i2cRead = { addr, len -> i2cRead(addr, len) },
            i2cWriteReg = { addr, reg, value -> i2cWriteReg(addr, reg, value) },
            i2cReadReg = { addr, reg -> i2cReadReg(addr, reg) },
            getTunerClock = { getTunerXtalFreq() },
            setGpioBit = { gpio, on -> setGpioBit(gpio, on) },
            setGpioOutput = { gpio -> setGpioOutput(gpio) },
        )

        tuner = when (tunerType) {
            Tuner.R820T, Tuner.R828D -> {
                val chip = if (tunerType == Tuner.R828D) R82xxTuner.Chip.R828D else R82xxTuner.Chip.R820T
                val i2cAddr = if (tunerType == Tuner.R828D) R82xxTuner.R828D_I2C_ADDR else R82xxTuner.R820T_I2C_ADDR
                R82xxTuner(
                    chip = chip,
                    xtalFreq = { getTunerXtalFreq() },
                    i2cWrite = { data -> i2cWrite(i2cAddr, data) },
                    i2cRead = { len -> i2cRead(i2cAddr, len) },
                    setGpio = { gpio, on -> setBiasTeeGpio(gpio, on) },
                    isBlogV4 = isBlogV4,
                    isBlogV4L = isBlogV4L,
                )
            }
            Tuner.E4000 -> E4kTuner(ctx)
            Tuner.FC0012 -> Fc0012Tuner(ctx)
            Tuner.FC0013 -> Fc0013Tuner(ctx)
            Tuner.FC2580 -> Fc2580Tuner(ctx)
            Tuner.UNKNOWN -> null
        }

        val t = tuner
        if (t != null) {
            if (t.init() < 0) {
                Log.e(TAG, "${t.name} tuner initialization failed")
            } else {
                Log.i(TAG, "${t.name} tuner initialized")
            }
        } else {
            setDirectSampling(1)
        }

        setI2cRepeater(false)
    }

    /** Tuner detection, mirroring the probe sequence in rtlsdr_open(). */
    private fun probeTuner(): Tuner {
        var reg = i2cReadReg(E4K_I2C_ADDR, E4K_CHECK_ADDR)
        if (reg == E4K_CHECK_VAL) {
            Log.i(TAG, "Found Elonics E4000 tuner")
            return Tuner.E4000
        }

        reg = i2cReadReg(FC0013_I2C_ADDR, FC0013_CHECK_ADDR)
        if (reg == FC0013_CHECK_VAL) {
            Log.i(TAG, "Found Fitipower FC0013 tuner")
            return Tuner.FC0013
        }

        reg = i2cReadReg(R82xxTuner.R820T_I2C_ADDR, 0x00)
        if (reg == R82xxTuner.R82XX_CHECK_VAL) {
            Log.i(TAG, "Found Rafael Micro R820T tuner")
            return Tuner.R820T
        }

        reg = i2cReadReg(R82xxTuner.R828D_I2C_ADDR, 0x00)
        if (reg == R82xxTuner.R82XX_CHECK_VAL) {
            Log.i(TAG, "Found Rafael Micro R828D tuner")
            return Tuner.R828D
        }

        /* initialise GPIOs and reset tuner before probing FC2580/FC0012 */
        setGpioOutput(4)
        setGpioBit(4, true)
        setGpioBit(4, false)

        reg = i2cReadReg(FC2580_I2C_ADDR, FC2580_CHECK_ADDR)
        if ((reg and 0x7f) == FC2580_CHECK_VAL) {
            Log.i(TAG, "Found FCI 2580 tuner")
            return Tuner.FC2580
        }

        reg = i2cReadReg(FC0012_I2C_ADDR, FC0012_CHECK_ADDR)
        if (reg == FC0012_CHECK_VAL) {
            Log.i(TAG, "Found Fitipower FC0012 tuner")
            setGpioOutput(6)
            return Tuner.FC0012
        }

        Log.w(TAG, "No supported tuner found")
        return Tuner.UNKNOWN
    }

    private fun checkDongleModel(manufactCheck: String, productCheck: String): Boolean =
        manufact == manufactCheck && product == productCheck

    /** Sensible initial configuration after open, before streaming starts. */
    private fun applyStreamingDefaults() {
        setSampleRate(SDRConfig.DEFAULT_SAMPLE_RATE_HZ)
        setCenterFreq(100_000_000L)
        setTunerGainMode(false)   /* tuner AGC */
        setAgcMode(true)          /* RTL digital AGC */
        resetBuffer()
    }

    // ==================== Baseband (port of rtlsdr_init_baseband) ====================

    private fun initBaseband() {
        /* initialize USB */
        writeReg(USBB, USB_SYSCTL, 0x09, 1)
        writeReg(USBB, USB_EPA_MAXPKT, 0x0002, 2)
        writeReg(USBB, USB_EPA_CTL, 0x1002, 2)

        /* power on demod */
        writeReg(SYSB, DEMOD_CTL_1, 0x22, 1)
        writeReg(SYSB, DEMOD_CTL, 0xe8, 1)

        /* reset demod (bit 3, soft_rst) */
        demodWriteReg(1, 0x01, 0x14, 1)
        demodWriteReg(1, 0x01, 0x10, 1)

        /* disable spectrum inversion and adjacent channel rejection */
        demodWriteReg(1, 0x15, 0x00, 1)
        demodWriteReg(1, 0x16, 0x0000, 2)

        /* clear both DDC shift and IF frequency registers */
        for (i in 0 until 6) {
            demodWriteReg(1, 0x16 + i, 0x00, 1)
        }

        setFir()

        /* enable SDR mode, disable DAGC (bit 5) */
        demodWriteReg(0, 0x19, 0x05, 1)

        /* init FSM state-holding register */
        demodWriteReg(1, 0x93, 0xf0, 1)
        demodWriteReg(1, 0x94, 0x0f, 1)

        /* disable AGC (en_dagc, bit 0) */
        demodWriteReg(1, 0x11, 0x00, 1)

        /* disable RF and IF AGC loop */
        demodWriteReg(1, 0x04, 0x00, 1)

        /* disable PID filter */
        demodWriteReg(0, 0x61, 0x60, 1)

        /* opt_adc_iq = 0, default ADC_I/ADC_Q datapath */
        demodWriteReg(0, 0x06, 0x80, 1)

        /* Enable Zero-IF mode, DC cancellation, IQ estimation/compensation */
        demodWriteReg(1, 0xb1, 0x1b, 1)

        /* disable 4.096 MHz clock output on pin TP_CK0 */
        demodWriteReg(0, 0x0d, 0x83, 1)
    }

    private fun deinitBaseband() {
        tuner?.let {
            setI2cRepeater(true)
            it.exit()
            setI2cRepeater(false)
        }
        /* poweroff demodulator and ADCs */
        writeReg(SYSB, DEMOD_CTL, 0x20, 1)
    }

    private fun setFir(): Int {
        val packed = IntArray(20)

        /* format: int8_t[8] */
        for (i in 0 until 8) {
            val v = fir[i]
            if (v < -128 || v > 127) return -1
            packed[i] = v and 0xff
        }
        /* format: int12_t[8] */
        for (i in 0 until 8 step 2) {
            val v0 = fir[8 + i]
            val v1 = fir[8 + i + 1]
            if (v0 < -2048 || v0 > 2047 || v1 < -2048 || v1 > 2047) return -1
            packed[8 + i * 3 / 2] = (v0 shr 4) and 0xff
            packed[8 + i * 3 / 2 + 1] = ((v0 shl 4) or ((v1 shr 8) and 0x0f)) and 0xff
            packed[8 + i * 3 / 2 + 2] = v1 and 0xff
        }

        for (i in packed.indices) {
            if (demodWriteReg(1, 0x1c + i, packed[i], 1) != 0) return -1
        }
        return 0
    }

    // ==================== Frequency / rate / gain ====================

    /** rtlsdr_get_xtal_freq: xtal values with PPM correction applied. */
    private fun getRtlXtalFreq(): Long = (rtlXtal * (1.0 + corr / 1e6)).toLong()
    private fun getTunerXtalFreq(): Long = (tunXtal * (1.0 + corr / 1e6)).toLong()

    private fun setIfFreq(freqHz: Int): Int {
        val rtlXtalCorr = getRtlXtalFreq()
        val ifFreq = -((freqHz.toLong() shl 22) / rtlXtalCorr)

        var r = demodWriteReg(1, 0x19, ((ifFreq shr 16) and 0x3f).toInt(), 1)
        r = r or demodWriteReg(1, 0x1a, ((ifFreq shr 8) and 0xff).toInt(), 1)
        r = r or demodWriteReg(1, 0x1b, (ifFreq and 0xff).toInt(), 1)
        return r
    }

    private fun setSampleFreqCorrection(ppm: Int): Int {
        val offs = (ppm * -1 * (1L shl 24) / 1_000_000).toInt()
        var r = demodWriteReg(1, 0x3f, offs and 0xff, 1)
        r = r or demodWriteReg(1, 0x3e, (offs shr 8) and 0x3f, 1)
        return r
    }

    /** Port of rtlsdr_set_center_freq, including automatic direct sampling. */
    private fun setCenterFreq(freqHz: Long): Int {
        val lastDs = directSampling

        /* Auto-switch to direct sampling below 24 MHz on R820T (except Blog V4L,
         * which has a built-in upconverter). Only when ds mode is 'standard'. */
        if (directSamplingMode == 0) {
            directSampling = if (freqHz < 24_000_000L && tunerType == Tuner.R820T &&
                !checkDongleModel("RTLSDRBlog", "Blog V4L")
            ) 2 else 0
        }

        val t = tuner
        val r: Int = if (directSampling != 0) {
            setI2cRepeater(false)
            setIfFreq(freqHz.toInt())
        } else if (t != null) {
            setI2cRepeater(true)
            t.setFreq(freqHz - offsFreq)
        } else {
            /* no tuner driver: tune via demod DDC only */
            setIfFreq((freqHz - offsFreq).toInt())
        }

        freq = if (r == 0) freqHz else 0L

        /* Apply the demod-side direct sampling transition after freq is updated */
        if (lastDs != directSampling) {
            return applyDirectSampling(directSampling)
        }
        return r
    }

    /** Port of rtlsdr_set_sample_rate. */
    private fun setSampleRate(sampRate: Int): Int {
        /* check if the rate is supported by the resampler */
        if (sampRate <= 225_000 || sampRate > 3_200_000 ||
            (sampRate in 300_001..900_000)
        ) {
            Log.e(TAG, "Invalid sample rate: $sampRate Hz")
            return -1
        }

        var rsampRatio = ((rtlXtal.toLong() shl 22) / sampRate).toInt()
        rsampRatio = rsampRatio and 0x0ffffffc

        val realRsampRatio = rsampRatio or ((rsampRatio and 0x08000000) shl 1)
        val realRate = (rtlXtal.toDouble() * (1L shl 22)) / realRsampRatio
        if (sampRate.toDouble() != realRate) {
            Log.i(TAG, "Exact sample rate is $realRate Hz")
        }
        rate = realRate.toInt()

        /* configure tuner IF bandwidth for the new rate */
        if (tuner != null) {
            setI2cRepeater(true)
            tunerSetBandwidth(if (bw > 0) bw else rate)
            setI2cRepeater(false)
        }

        var r = demodWriteReg(1, 0x9f, (rsampRatio shr 16) and 0xffff, 2)
        r = r or demodWriteReg(1, 0xa1, rsampRatio and 0xffff, 2)
        r = r or setSampleFreqCorrection(corr)

        /* reset demod (bit 3, soft_rst) */
        r = r or demodWriteReg(1, 0x01, 0x14, 1)
        r = r or demodWriteReg(1, 0x01, 0x10, 1)

        /* recalculate offset frequency if offset tuning is enabled */
        if (offsFreq != 0) setOffsetTuning(true)

        return r
    }

    /**
     * Port of the tuner set_bw wrappers: sets the tuner filter and, when the
     * tuner reports a new IF (R82xx), reprograms the RTL2832U IF and retunes.
     */
    private fun tunerSetBandwidth(bwHz: Int): Int {
        val t = tuner ?: return 0
        val newIf = t.setBandwidth(bwHz)
        if (newIf < 0) return newIf
        if (newIf == 0) return 0
        var r = setIfFreq(newIf)
        if (r != 0) return r
        if (freq != 0L) {
            r = setCenterFreq(freq)
        }
        return r
    }

    /** rtlsdr_set_tuner_bandwidth. */
    fun setTunerBandwidth(bwHz: Int): Int {
        if (!isOpen) return -1
        setI2cRepeater(true)
        val r = tunerSetBandwidth(if (bwHz > 0) bwHz else rate)
        setI2cRepeater(false)
        if (r == 0) bw = bwHz
        return r
    }

    private fun setTunerGain(gainTenthDb: Int): Int {
        val t = tuner ?: return -1
        setI2cRepeater(true)
        val r = t.setGain(gainTenthDb)
        gain = if (r == 0) gainTenthDb else 0
        return r
    }

    private fun setTunerGainMode(manual: Boolean): Int {
        val t = tuner ?: return -1
        setI2cRepeater(true)
        return t.setGainMode(manual)
    }

    private fun setAgcMode(on: Boolean): Int =
        demodWriteReg(0, 0x19, if (on) 0x25 else 0x05, 1)

    fun setTestMode(on: Boolean): Int =
        demodWriteReg(0, 0x19, if (on) 0x03 else 0x05, 1)

    /** Port of rtlsdr_set_freq_correction. */
    private fun setFreqCorrection(ppm: Int): Int {
        if (corr == ppm) return 0
        corr = ppm

        var r = setSampleFreqCorrection(ppm)
        /* the tuner reads the corrected xtal via getTunerXtalFreq() */
        if (freq != 0L) {
            r = r or setCenterFreq(freq)
        }
        return r
    }

    /** Port of rtlsdr_set_direct_sampling (remembers the UI-selected mode). */
    private fun setDirectSampling(on: Int): Int {
        directSamplingMode = on
        return applyDirectSampling(on)
    }

    /** Port of _rtlsdr_set_direct_sampling. */
    private fun applyDirectSampling(on: Int): Int {
        var r = 0

        if (on != 0) {
            tuner?.let {
                setI2cRepeater(true)
                it.exit()
                setI2cRepeater(false)
            }

            /* disable Zero-IF mode */
            r = r or demodWriteReg(1, 0xb1, 0x1a, 1)
            /* disable spectrum inversion */
            r = r or demodWriteReg(1, 0x15, 0x00, 1)
            /* only enable In-phase ADC input */
            r = r or demodWriteReg(0, 0x08, 0x4d, 1)
            /* swap I and Q ADC to select between the two inputs */
            r = r or demodWriteReg(0, 0x06, if (on > 1) 0x90 else 0x80, 1)

            Log.i(TAG, "Enabled direct sampling mode, input $on")
            directSampling = on
        } else {
            tuner?.let {
                setI2cRepeater(true)
                it.init()
                setI2cRepeater(false)
            }

            if (tunerType == Tuner.R820T || tunerType == Tuner.R828D) {
                r = r or setIfFreq(R82XX_IF_FREQ)
                /* enable spectrum inversion */
                r = r or demodWriteReg(1, 0x15, 0x01, 1)
            } else {
                r = r or setIfFreq(0)
                /* enable In-phase + Quadrature ADC input */
                r = r or demodWriteReg(0, 0x08, 0xcd, 1)
                /* Enable Zero-IF mode */
                r = r or demodWriteReg(1, 0xb1, 0x1b, 1)
            }

            /* opt_adc_iq = 0, default ADC_I/ADC_Q datapath */
            r = r or demodWriteReg(0, 0x06, 0x80, 1)

            Log.i(TAG, "Disabled direct sampling mode")
            directSampling = 0
        }

        if (freq != 0L) {
            r = r or setCenterFreq(freq)
        }
        return r
    }

    /** Port of rtlsdr_set_offset_tuning. */
    fun setOffsetTuning(on: Boolean): Int {
        if (tunerType == Tuner.R820T || tunerType == Tuner.R828D) {
            /* RTL-SDR Blog hack: reuse "offset tuning" as a bias tee toggle,
             * since the R82xx doesn't use offset tuning. */
            setBiasTee(on)
            return -2
        }

        if (directSampling != 0) return -3

        /* based on keenerds 1/f noise measurements */
        offsFreq = if (on) (rate / 2) * 170 / 100 else 0
        var r = setIfFreq(offsFreq)

        if (freq > offsFreq) {
            r = r or setCenterFreq(freq)
        }
        return r
    }

    // ==================== Bias tee / GPIO ====================

    /** Port of rtlsdr_set_bias_tee_gpio. */
    private fun setBiasTeeGpio(gpio: Int, on: Boolean): Int {
        /* Never allow the forced bias tee to be turned off. */
        val value = if (gpio == 0 && forceBiasTee) true else on
        setGpioOutput(gpio)
        setGpioBit(gpio, value)
        return 0
    }

    fun setBiasTee(on: Boolean): Int = setBiasTeeGpio(0, on)

    private fun setGpioBit(gpio: Int, value: Boolean) {
        val mask = 1 shl gpio
        var r = readReg(SYSB, GPO, 1)
        if (r < 0) return  /* don't clobber the other GPIO bits on a failed read */
        r = if (value) r or mask else r and mask.inv()
        writeReg(SYSB, GPO, r, 1)
    }

    private fun setGpioOutput(gpio: Int) {
        val mask = 1 shl gpio
        val d = readReg(SYSB, GPD, 1)
        if (d < 0) return
        writeReg(SYSB, GPD, d and mask.inv(), 1)
        val e = readReg(SYSB, GPOE, 1)
        if (e < 0) return
        writeReg(SYSB, GPOE, e or mask, 1)
    }

    private fun setI2cRepeater(on: Boolean) {
        demodWriteReg(1, 0x01, if (on) 0x18 else 0x10, 1)
    }

    // ==================== Streaming ====================

    private fun resetBuffer(): Int {
        var r = writeReg(USBB, USB_EPA_CTL, 0x1002, 2)
        r = minOf(r, writeReg(USBB, USB_EPA_CTL, 0x0000, 2))
        return if (r < 0) -1 else 0
    }

    private fun startStreaming() {
        streamJob = CoroutineScope(Dispatchers.IO).launch {
            // Audio priority: block delivery must not lose CPU to rendering.
            try {
                android.os.Process.setThreadPriority(
                    android.os.Process.THREAD_PRIORITY_URGENT_AUDIO
                )
            } catch (_: Throwable) {}
            val conn = connection
            val endpoint = bulkEndpoint
            if (conn == null || endpoint == null) {
                onConnectionStatusChanged(false, "Device not ready")
                return@launch
            }

            val buffer = ByteArray(BULK_BUFFER_SIZE)
            var consecutiveErrors = 0
            val maxConsecutiveErrors = 10

            while (isConnected && !devLost) {
                val bytesRead = try {
                    conn.bulkTransfer(endpoint, buffer, buffer.size, BULK_TIMEOUT)
                } catch (e: Exception) {
                    -1
                }

                when {
                    bytesRead > 0 -> {
                        consecutiveErrors = 0
                        processIQData(buffer, bytesRead)
                    }
                    else -> {
                        consecutiveErrors++
                        if (consecutiveErrors >= maxConsecutiveErrors) {
                            devLost = true
                            Log.e(TAG, "Too many bulk transfer errors, device lost")
                            onConnectionStatusChanged(false, "Device lost")
                            break
                        }
                        /* flush the endpoint FIFO and retry */
                        withContext(usbDispatcher) {
                            if (isOpen) resetBuffer()
                        }
                    }
                }
            }

            /* Unplugged mid-stream: free the interface, fd and worker thread. */
            if (devLost) {
                isConnected = false
                scope.launch {
                    try {
                        close()
                    } finally {
                        release()
                    }
                }
            }
        }
    }

    private fun stopStreamingBlocking() {
        isConnected = false
        streamJob?.let {
            it.cancel()
            runBlocking { it.join() }
        }
        streamJob = null
    }

    /** UI renders at ~10 fps; skip redundant FFTs on intermediate chunks. */
    private var lastSpectrum: FloatArray? = null
    private var lastFftTimeMs = 0L
    private val fftIntervalMs = 80L

    private fun processIQData(buffer: ByteArray, length: Int) {
        val pairs = length / 2
        if (pairs < SDRConfig.FFT_SIZE / 4) return

        /* unsigned 8-bit IQ centered at 128 → interleaved floats in [-1, 1] */
        val iq = FloatArray(pairs * 2)
        for (i in 0 until pairs * 2) {
            iq[i] = ((buffer[i].toInt() and 0xff) - 128) / 128.0f
        }

        /* Host has no visible spectrum consumer: skip the FFT, keep the IQ
           flowing (audio must never depend on the FFT path). */
        if (!spectrumEnabled) {
            onDataReceived(EMPTY_SPECTRUM, iq)
            return
        }

        /* computePowerSpectrum windows the first FFT_SIZE pairs itself */
        val now = System.currentTimeMillis()
        val spectrum = if (lastSpectrum == null || now - lastFftTimeMs >= fftIntervalMs) {
            lastFftTimeMs = now
            fftProcessor?.computePowerSpectrum(iq, pairs)?.also { lastSpectrum = it }
        } else {
            lastSpectrum
        } ?: return

        onDataReceived(spectrum, iq)
    }

    // ==================== Low-level register access ====================

    private fun readArray(block: Int, addr: Int, len: Int): ByteArray? {
        val conn = connection ?: return null
        val buffer = ByteArray(len)
        val index = block shl 8
        val r = conn.controlTransfer(CTRL_IN, 0, addr, index, buffer, len, CTRL_TIMEOUT)
        return if (r >= 0) buffer.copyOf(r.coerceAtMost(len)) else null
    }

    private fun writeArray(block: Int, addr: Int, data: ByteArray, len: Int = data.size): Int {
        val conn = connection ?: return -1
        val index = (block shl 8) or 0x10
        return conn.controlTransfer(CTRL_OUT, 0, addr, index, data, len, CTRL_TIMEOUT)
    }

    /** Returns the register value, or -1 if the control transfer failed. */
    private fun readReg(block: Int, addr: Int, len: Int): Int {
        val data = readArray(block, addr, len) ?: return -1
        return when {
            data.size >= 2 -> ((data[1].toInt() and 0xff) shl 8) or (data[0].toInt() and 0xff)
            data.size == 1 -> data[0].toInt() and 0xff
            else -> -1
        }
    }

    private fun writeReg(block: Int, addr: Int, value: Int, len: Int): Int {
        val data = if (len == 1) {
            byteArrayOf((value and 0xff).toByte())
        } else {
            byteArrayOf(((value shr 8) and 0xff).toByte(), (value and 0xff).toByte())
        }
        return writeArray(block, addr, data)
    }

    private fun demodReadReg(page: Int, addr: Int, len: Int): Int {
        val conn = connection ?: return 0
        val data = ByteArray(2)
        val address = (addr shl 8) or 0x20
        conn.controlTransfer(CTRL_IN, 0, address, page, data, len, CTRL_TIMEOUT)
        return ((data[1].toInt() and 0xff) shl 8) or (data[0].toInt() and 0xff)
    }

    private fun demodWriteReg(page: Int, addr: Int, value: Int, len: Int): Int {
        val conn = connection ?: return -1
        val data = if (len == 1) {
            byteArrayOf((value and 0xff).toByte())
        } else {
            byteArrayOf(((value shr 8) and 0xff).toByte(), (value and 0xff).toByte())
        }
        val address = (addr shl 8) or 0x20
        val index = 0x10 or page
        val r = conn.controlTransfer(CTRL_OUT, 0, address, index, data, len, CTRL_TIMEOUT)

        demodReadReg(0x0a, 0x01, 1)

        return if (r == len) 0 else -1
    }

    // ==================== I2C ====================

    private fun i2cWrite(i2cAddr: Int, buffer: ByteArray): Int =
        writeArray(IICB, i2cAddr, buffer)

    private fun i2cRead(i2cAddr: Int, len: Int): ByteArray? {
        val data = readArray(IICB, i2cAddr, len) ?: return null
        return if (data.size == len) data else null
    }

    private fun i2cWriteReg(i2cAddr: Int, reg: Int, value: Int): Int =
        writeArray(IICB, i2cAddr, byteArrayOf(reg.toByte(), value.toByte()))

    private fun i2cReadReg(i2cAddr: Int, reg: Int): Int {
        writeArray(IICB, i2cAddr, byteArrayOf(reg.toByte()))
        val data = readArray(IICB, i2cAddr, 1) ?: return -1
        return if (data.isNotEmpty()) data[0].toInt() and 0xff else -1
    }

    // ==================== EEPROM ====================

    /** Port of rtlsdr_read_eeprom; returns true only if every byte read back. */
    private fun readEeprom(data: ByteArray, offset: Int, len: Int): Boolean {
        if (len + offset > EEPROM_SIZE) return false

        if (writeArray(IICB, EEPROM_ADDR, byteArrayOf(offset.toByte())) < 0) return false

        for (i in 0 until len) {
            val b = readArray(IICB, EEPROM_ADDR, 1) ?: return false
            if (b.isEmpty()) return false
            data[i] = b[0]
        }
        return true
    }
}
