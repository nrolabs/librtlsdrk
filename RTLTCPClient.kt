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
import com.isaklab.isdrdrivers.core.SpectrumWorker
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
         * Delivery block: bytes accumulated before one conversion+delivery.
         * 16384 IQ pairs = 8 ms at 2.048 MSps. Short TCP reads (slow links,
         * chaos stalls) accumulate here instead of being discarded — the old
         * path silently dropped any read under 100 pairs, losing real samples.
         */
        private const val BLOCK_BYTES = 32768

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
    // The Welch FFT runs on its own thread: computing it inline on the receive
    // thread stalls the socket read and drops samples at high rates. The
    // receive thread only submits a block on the display cadence and delivers
    // the last cached spectrum — audio never waits on the FFT.
    private var spectrumWorker: SpectrumWorker? = null

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
            spectrumWorker = SpectrumWorker(fftProcessor!!)
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
                spectrumWorker?.stop()
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
            spectrumWorker?.start()
            // Reads land directly in the block accumulator at [fill]; a block
            // is converted and delivered only when full. No thread hop per
            // read: the conversion is a linear table-free loop, far cheaper
            // than a withContext(Default) dispatch per TCP read was.
            val buffer = ByteArray(BLOCK_BYTES)
            var fill = 0
            var packetsReceived = 0
            var lastLogTime = System.currentTimeMillis()
            Log.d("RTLTCPClient", "Starting data reception loop...")
            while (isConnected && !currentCoroutineContext().job.isCancelled) {
                try {
                    val bytesRead = inputStream!!.read(buffer, fill, buffer.size - fill)
                    if (bytesRead > 0) {
                        fill += bytesRead
                        packetsReceived++
                        val currentTime = System.currentTimeMillis()
                        if (packetsReceived % 100 == 0 || (currentTime - lastLogTime) > 2000) {
                            Log.d("RTLTCPClient", "Received $packetsReceived packets, last: $bytesRead bytes")
                            lastLogTime = currentTime
                        }
                        if (fill == buffer.size) {
                            processIQData(buffer, fill)
                            fill = 0
                        }
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

    private var lastFftTimeMs = 0L
    private val fftIntervalMs = 80L

    /**
     * Fixed-size conversion scratch, reused every block: the old path built a
     * fresh FloatArray per TCP read (~4 MB/s of garbage at 2 MSps, GC pauses
     * read as audio stutter). The SAME array instance is handed to
     * [onDataReceived] each time — the consumer (DriverSession) serializes it
     * before returning, per the driver callback contract.
     */
    private val iqScratch = FloatArray(BLOCK_BYTES)

    private fun processIQData(buffer: ByteArray, length: Int) {
        try {
            /* unsigned 8-bit IQ centered at 128 → interleaved floats in [-1, 1] */
            for (i in 0 until length) {
                iqScratch[i] = ((buffer[i].toInt() and 0xFF) - 128) / 128.0f
            }
            /* Host has no visible spectrum consumer: skip the FFT, keep the
               IQ flowing (audio must never depend on the FFT path). */
            if (!spectrumEnabled) {
                onDataReceived(EMPTY_SPECTRUM, iqScratch)
                return
            }
            /* The display renders ~10 fps: on that cadence hand a copy of the
               block to the off-thread worker (submit copies internally); never
               compute the FFT on this receive thread. Deliver the IQ NOW with
               the last cached spectrum — audio must never wait on the FFT
               (spectrum may lag one frame, imperceptible at 10 fps). */
            val now = System.currentTimeMillis()
            if (now - lastFftTimeMs >= fftIntervalMs) {
                lastFftTimeMs = now
                spectrumWorker?.submit(iqScratch, length / 2)
            }
            onDataReceived(spectrumWorker?.latest ?: EMPTY_SPECTRUM, iqScratch)
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
        spectrumWorker?.resetSmoothing()
        sendCommand(RTLCommand(0x01.toByte(), frequencyHz.toInt()))
    }

    fun setSampleRate(sampleRateHz: Int) {
        sampleRate = sampleRateHz.toDouble()
        spectrumWorker?.resetSmoothing()
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