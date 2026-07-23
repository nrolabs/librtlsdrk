/*
 * librtlsdrk - Kotlin port of librtlsdr (RTL-SDR Blog fork) for Android USB host
 *
 * rtl_tcp protocol client, protocol defined by rtl_tcp.c:
 * Copyright (C) 2012-2013 by Steve Markgraf <steve@steve-m.de>
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

import android.util.Log
import com.isaklab.isdrdrivers.core.FFTProcessor
import kotlinx.coroutines.*
import java.io.*
import java.net.Socket
import java.nio.ByteBuffer
import java.nio.ByteOrder

class RTLTCPClient(
    private val host: String,
    private val port: Int,
    /** (power spectrum in dB, interleaved IQ samples i0,q0,i1,q1,... in [-1,1]) */
    private val onDataReceived: (FloatArray, FloatArray) -> Unit,
    private val onConnectionStatusChanged: (Boolean, String) -> Unit
) {
    companion object {
        private val EMPTY_SPECTRUM = FloatArray(0)

        /**
         * Convert [byteCount] bytes of a raw rtl_tcp payload — unsigned 8-bit
         * interleaved I/Q centred at 128 — into interleaved floats in [-1, 1).
         * Pure and side-effect free so the receive path can be unit-tested.
         */
        fun u8ToIq(buffer: ByteArray, byteCount: Int): FloatArray {
            val iq = FloatArray(byteCount)
            for (i in 0 until byteCount) {
                iq[i] = ((buffer[i].toInt() and 0xFF) - 128) / 128.0f
            }
            return iq
        }
    }

    private var socket: Socket? = null
    private var inputStream: DataInputStream? = null
    private var outputStream: DataOutputStream? = null
    private var isConnected = false
    private var receiveJob: Job? = null
    private var fftProcessor: FFTProcessor? = null

    /**
     * When false, IQ blocks are delivered with an empty spectrum and the FFT
     * is skipped (the host has no visible spectrum consumer). Audio delivery
     * is unaffected.
     */
    @Volatile var spectrumEnabled: Boolean = true
    private var sampleRate: Double = 2.048e6
    private val scope = CoroutineScope(Dispatchers.IO + SupervisorJob())

    suspend fun connect(): Boolean = withContext(Dispatchers.IO) {
        try {
            onConnectionStatusChanged(false, "Connecting...")
            socket = Socket(host, port)
            socket!!.soTimeout = 10000
            socket!!.tcpNoDelay = true
            socket!!.keepAlive = true
            inputStream = DataInputStream(socket!!.getInputStream())
            outputStream = DataOutputStream(socket!!.getOutputStream())
            val magic = ByteArray(4)
            inputStream!!.readFully(magic)
            val magicStr = String(magic, Charsets.UTF_8)
            if (magicStr != "RTL0") {
                throw IOException("Invalid server response")
            }
            val tunerType = Integer.reverseBytes(inputStream!!.readInt())
            val tunerGainCount = Integer.reverseBytes(inputStream!!.readInt())
            Log.i("RTLTCPClient", "Connected - Tuner type: $tunerType, Gain count: $tunerGainCount")
            isConnected = true
            fftProcessor = FFTProcessor(800)
            onConnectionStatusChanged(true, "Connected")
            startReceiving()
            true
        } catch (e: Exception) {
            Log.e("RTLTCPClient", "Connection failed: ${e.message}")
            onConnectionStatusChanged(false, "Connection failed")
            disconnect()
            false
        }
    }

    fun disconnect() {
        scope.launch {
            try {
                isConnected = false
                receiveJob?.cancel()
                inputStream?.close()
                outputStream?.close()
                socket?.close()
                onConnectionStatusChanged(false, "Disconnected")
            } catch (e: Exception) {
            }
        }
    }

    private fun startReceiving() {
        receiveJob = scope.launch {
            // Audio priority: this loop feeds the audio pipeline and must not
            // lose CPU to spectrum/waterfall rendering on a low-end phone.
            try {
                android.os.Process.setThreadPriority(
                    android.os.Process.THREAD_PRIORITY_URGENT_AUDIO
                )
            } catch (_: Throwable) {}
            val buffer = ByteArray(32768)
            var packetsReceived = 0
            var lastLogTime = System.currentTimeMillis()
            Log.d("RTLTCPClient", "Starting data reception loop...")
            while (isConnected && !currentCoroutineContext().job.isCancelled) {
                try {
                    val bytesRead = inputStream!!.read(buffer)
                    if (bytesRead > 0) {
                        packetsReceived++
                        val currentTime = System.currentTimeMillis()
                        if (packetsReceived % 100 == 0 || (currentTime - lastLogTime) > 2000) {
                            Log.d("RTLTCPClient", "Received $packetsReceived packets, last: $bytesRead bytes")
                            lastLogTime = currentTime
                        }
                        processIQData(buffer, bytesRead)
                    } else if (bytesRead == -1) {
                        Log.w("RTLTCPClient", "Server closed connection (EOF)")
                        onConnectionStatusChanged(false, "Server closed")
                        isConnected = false
                        break
                    }
                } catch (e: java.net.SocketTimeoutException) {
                    Log.w("RTLTCPClient", "Socket timeout - no data received")
                    onConnectionStatusChanged(false, "Timeout")
                    isConnected = false
                    break
                } catch (e: Exception) {
                    if (isConnected) {
                        Log.e("RTLTCPClient", "Data reception error: ${e.javaClass.simpleName}: ${e.message}")
                        onConnectionStatusChanged(false, "Error")
                        isConnected = false
                    }
                    break
                }
            }
            Log.d("RTLTCPClient", "Data reception loop ended. Total packets: $packetsReceived")
        }
    }

    private var lastSpectrum: FloatArray? = null
    private var lastFftTimeMs = 0L
    private val fftIntervalMs = 80L

    private suspend fun processIQData(buffer: ByteArray, length: Int) = withContext(Dispatchers.Default) {
        try {
            val pairsAvailable = length / 2
            if (pairsAvailable < 100) {
                return@withContext
            }
            /* unsigned 8-bit IQ centered at 128 → interleaved floats in [-1, 1] */
            val iq = u8ToIq(buffer, pairsAvailable * 2)
            /* Host has no visible spectrum consumer: skip the FFT, keep the
               IQ flowing (audio must never depend on the FFT path). */
            if (!spectrumEnabled) {
                onDataReceived(EMPTY_SPECTRUM, iq)
                return@withContext
            }
            /* The display renders ~10 fps: computing an FFT for every block
               only heated the CPU — recompute on the display cadence and
               re-serve the cached spectrum in between (same as the USB client). */
            val now = System.currentTimeMillis()
            val powerSpectrum = if (lastSpectrum == null || now - lastFftTimeMs >= fftIntervalMs) {
                lastFftTimeMs = now
                fftProcessor?.computePowerSpectrum(iq, pairsAvailable)?.also { lastSpectrum = it }
            } else {
                lastSpectrum
            } ?: return@withContext
            onDataReceived(powerSpectrum, iq)
        } catch (e: Exception) {
            Log.e("RTLTCPClient", "Error processing IQ data: ${e.message}")
        }
    }

    fun sendCommand(command: RTLCommand) {
        scope.launch {
            try {
                if (isConnected && outputStream != null) {
                    val buffer = ByteBuffer.allocate(5)
                    buffer.order(ByteOrder.BIG_ENDIAN)
                    buffer.put(command.cmd)
                    buffer.putInt(command.param)
                    outputStream!!.write(buffer.array())
                    outputStream!!.flush()
                    Log.d("RTLTCPClient", "Sent command: 0x${String.format("%02x", command.cmd)} param: ${command.param}")
                }
            } catch (e: Exception) {
                Log.e("RTLTCPClient", "Failed to send command: ${e.message}")
            }
        }
    }

    fun setFrequency(frequencyHz: Long) {
        fftProcessor?.resetSmoothing()
        sendCommand(RTLCommand(0x01.toByte(), frequencyHz.toInt()))
    }

    fun setSampleRate(sampleRateHz: Int) {
        sampleRate = sampleRateHz.toDouble()
        fftProcessor?.resetSmoothing()
        sendCommand(RTLCommand(0x02.toByte(), sampleRateHz))
    }

    fun setGainMode(manual: Boolean) {
        sendCommand(RTLCommand(0x03.toByte(), if (manual) 1 else 0))
    }

    fun setGain(gainTenthsOfDb: Int) {
        sendCommand(RTLCommand(0x04.toByte(), gainTenthsOfDb))
    }

    fun setFrequencyCorrection(ppm: Int) {
        sendCommand(RTLCommand(0x05.toByte(), ppm))
    }

    fun setBiasTee(enabled: Boolean) {
        Log.i("RTLTCPClient", "Setting Bias-T: ${if (enabled) "ENABLED" else "DISABLED"}")
        sendCommand(RTLCommand(0x0e.toByte(), if (enabled) 1 else 0))
    }

    fun setDirectSampling(mode: Int) {
        sendCommand(RTLCommand(0x09.toByte(), mode))
    }

    fun setAgcMode(enabled: Boolean) {
        Log.i("RTLTCPClient", "Setting AGC: ${if (enabled) "ENABLED" else "DISABLED"}")
        sendCommand(RTLCommand(0x08.toByte(), if (enabled) 1 else 0))
    }

    /**
     * Set FFT smoothing factor (0.0 = maximum smoothing, 1.0 = no smoothing)
     */
    fun setSmoothingFactor(alpha: Float) {
        fftProcessor?.setSmoothingFactor(alpha)
    }
    
    /**
     * Get current sample rate
     */
    fun getSampleRate(): Double = sampleRate
}