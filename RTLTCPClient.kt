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
import com.isaklab.isdrdrivers.core.AntennaPowerCapable
import com.isaklab.isdrdrivers.core.RadioClient

import android.util.Log
import com.isaklab.isdrdrivers.core.DspThread
import com.isaklab.isdrdrivers.core.FFTProcessor
import com.isaklab.isdrdrivers.core.SpectrumWorker
import kotlinx.coroutines.*
import java.io.*
import java.net.Socket
import java.nio.ByteBuffer
import java.nio.ByteOrder

/** rtl_tcp's 12-byte greeting; both u32 values are network byte order. */
internal data class RtlTcpHeader(val tunerType: Int, val tunerGainCount: Int)

@Throws(IOException::class)
internal fun readRtlTcpHeader(input: DataInputStream): RtlTcpHeader {
    val magic = ByteArray(4)
    input.readFully(magic)
    val expectedMagic = byteArrayOf(
        'R'.code.toByte(), 'T'.code.toByte(), 'L'.code.toByte(), '0'.code.toByte(),
    )
    if (!magic.contentEquals(expectedMagic)) {
        throw IOException("Invalid rtl_tcp greeting")
    }
    // DataInputStream.readInt is already big-endian, exactly matching htonl()
    // in rtl_tcp. Reversing these values turns tuner 5 into 0x05000000.
    val tunerType = input.readInt()
    val gainCount = input.readInt()
    if (tunerType < 0) throw IOException("Invalid rtl_tcp tuner type $tunerType")
    if (gainCount !in 0..MAX_RTL_GAIN_STEPS) {
        throw IOException("Invalid rtl_tcp gain count $gainCount")
    }
    return RtlTcpHeader(tunerType, gainCount)
}

/**
 * RTL-TCP client implementation.
 *
 * Kotlin port of the rtl_tcp client protocol originally defined in `rtl_tcp.c`.
 * This class establishes a persistent TCP socket to a remote `rtl_tcp` server,
 * issues commands (tuning, gain, sample rate) using the standard 5-byte `RTLCommand` protocol,
 * and maintains a high-priority read loop to ingest interleaved 8-bit unsigned IQ data.
 *
 * Why: Decoupling the SDR hardware from the DSP processing host is crucial for distributed
 * setups (e.g., remote antenna streaming to an Android device). The client must handle
 * potentially erratic network behavior (short reads, stalls) without dropping the DSP thread
 * or causing audio underruns.
 *
 * To maintain DSP stability, the high-priority receive thread buffers incoming packets into
 * `BLOCK_BYTES` chunks. The conversion of unsigned 8-bit to normalized floats is done inline
 * without garbage allocation. Spectrum FFTs are explicitly offloaded to a separate
 * `SpectrumWorker` thread, guaranteeing that the UI/waterfall cadence does not stall the
 * critical audio stream path.
 */
class RTLTCPClient(
    private val host: String,
    private val port: Int,
    /** (power spectrum in dB, interleaved IQ samples i0,q0,i1,q1,... in [-1,1]) */
    private val onDataReceived: (FloatArray, FloatArray) -> Unit,
    private val onConnectionStatusChanged: (Boolean, String) -> Unit
) : RadioClient, AntennaPowerCapable {
    companion object {
        private val EMPTY_SPECTRUM = FloatArray(0)
        internal val SUPPORTED_SAMPLE_RATES = setOf(
            240_000, 300_000, 960_000, 1_024_000, 1_152_000, 1_200_000,
            1_440_000, 1_600_000, 1_800_000, 2_048_000, 2_400_000,
            2_560_000, 2_880_000, 3_200_000,
        )

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
    // Read by the receive thread as its loop guard and written by disconnect()
    // from another thread: the write has to be visible without a lock.
    @Volatile private var isConnected = false
    @Volatile private var tunerMetadata = RtlTunerInfo.unknown()
    // Its own thread (see DspThread): raising an IO-pool worker to audio
    // priority leaked that priority to every later coroutine on it.
    private var receiveThread: Thread? = null
    /** One-shot latch: the teardown of one connection runs exactly once,
     *  whichever of the operator path and the failure path reaches it first.
     *  Cleared when a new receive loop starts. */
    private val teardownDone = java.util.concurrent.atomic.AtomicBoolean(false)
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
    @Volatile override var spectrumEnabled: Boolean = true

    /**
     * Narrow the panadapter's span before the transform; returns the
     * decimation actually in force. With no session there is nothing to hold
     * it, and the answer is an honest 1 rather than the request echoed back.
     */
    fun setSpectrumZoom(decimation: Int, offsetHz: Long): Int =
        spectrumWorker?.setZoom(decimation, offsetHz.toDouble(), sampleRate.toDouble()) ?: 1

    private var sampleRate: Double = 2.048e6
    @Volatile private var freqHz = 0L
    private val scope = CoroutineScope(Dispatchers.IO + SupervisorJob())

    override suspend fun connect(): Boolean = withContext(Dispatchers.IO) {
        try {
            onConnectionStatusChanged(false, "Connecting...")
            socket = Socket(host, port)
            socket!!.soTimeout = 10000
            socket!!.tcpNoDelay = true
            socket!!.keepAlive = true
            inputStream = DataInputStream(socket!!.getInputStream())
            outputStream = DataOutputStream(socket!!.getOutputStream())
            val header = readRtlTcpHeader(inputStream!!)
            tunerMetadata = rtlTcpTunerInfo(header.tunerType, header.tunerGainCount)
            Log.i(
                "RTLTCPClient",
                "Connected - Tuner type: ${header.tunerType}, " +
                    "Gain count: ${header.tunerGainCount}",
            )
            isConnected = true
            fftProcessor = FFTProcessor(800)
            spectrumWorker = SpectrumWorker(fftProcessor!!)
            onConnectionStatusChanged(true, "Connected - ${tunerMetadata.tunerName}")
            startReceiving()
            true
        } catch (e: Exception) {
            Log.e("RTLTCPClient", "Connection failed: ${e.message}")
            onConnectionStatusChanged(false, "Connection failed")
            disconnect()
            false
        }
    }

    override fun disconnect() {
        scope.launch { teardown("Disconnected") }
    }

    /**
     * Closes the link and tells the host, exactly once per connection.
     *
     * Shared by [disconnect] and [failLink] so a failure exit releases the
     * same resources an operator disconnect does — a link that is torn down
     * silently leaves the socket, the FFT worker thread and the host's
     * "connected" state behind.
     *
     * Never joins the receive thread from the receive thread itself: the
     * failure path runs INSIDE that thread, and a self-join would burn the
     * full join timeout on every failed link.
     */
    private fun teardown(statusMessage: String) {
        if (!teardownDone.compareAndSet(false, true)) return
        isConnected = false
        val self = Thread.currentThread()
        // Closing the stream is what unblocks the blocking read.
        runCatching { inputStream?.close() }
        runCatching { outputStream?.close() }
        runCatching { socket?.close() }
        receiveThread?.takeIf { it !== self }?.let { DspThread.stop(it) }
        receiveThread = null
        spectrumWorker?.stop()
        onConnectionStatusChanged(false, statusMessage)
    }

    /**
     * Retire the link after the receive thread died on an unhandled
     * throwable. DspThread has already logged it at ERROR with the stack; the
     * job here is to make the death VISIBLE and leave nothing running.
     *
     * Without this the thread is gone while `isConnected` and the host's
     * "Connected" status survive: the display freezes on the last block and
     * no reconnect is ever attempted, because as far as everyone above is
     * concerned the radio is still streaming. A throwable raised while the
     * teardown is already under way is that teardown closing the socket under
     * the read, so the status text follows whichever path got here first.
     */
    private fun failLink() {
        teardown(if (isConnected) "Error" else "Disconnected")
    }

    private fun startReceiving() {
        teardownDone.set(false)
        // Audio priority: this loop feeds the audio pipeline and must not
        // lose CPU to spectrum/waterfall rendering on a low-end phone.
        receiveThread = DspThread.start("rtltcp-rx", DspThread.PRIORITY_RADIO, onFailure = { failLink() }) {
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
            while (isConnected) {
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
                        // Full teardown, not just a status change: leaving the
                        // socket open and the FFT worker alive held both until
                        // the next connect.
                        teardown("Server closed")
                        break
                    }
                } catch (e: java.net.SocketTimeoutException) {
                    Log.w("RTLTCPClient", "Socket timeout - no data received")
                    teardown("Timeout")
                    break
                } catch (e: Exception) {
                    if (isConnected) {
                        Log.e("RTLTCPClient", "Data reception error: ${e.javaClass.simpleName}: ${e.message}")
                        teardown("Error")
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

    @Synchronized
    fun sendCommand(command: RTLCommand) {
        check(isConnected) { "rtl_tcp control transport is not connected" }
        val output = outputStream
            ?: throw IllegalStateException("rtl_tcp control stream is unavailable")
        val buffer = ByteBuffer.allocate(5).order(ByteOrder.BIG_ENDIAN)
        buffer.put(command.cmd)
        buffer.putInt(command.param)
        try {
            output.write(buffer.array())
            output.flush()
        } catch (failure: Exception) {
            Log.e("RTLTCPClient", "Failed to send command: ${failure.message}")
            failLink()
            throw failure
        }
        Log.d(
            "RTLTCPClient",
            "Sent command: 0x${String.format("%02x", command.cmd)} param: ${command.param}",
        )
    }

    override fun setFrequency(hz: Long) {
        require(hz in 500_000L..1_766_000_000L) {
            "rtl_tcp frequency $hz Hz is outside 500000..1766000000 Hz"
        }
        sendCommand(RTLCommand(0x01.toByte(), hz.toInt()))
        spectrumWorker?.resetSmoothing()
        freqHz = hz
    }

    /**
     * rtl_tcp has no reply channel, so the last tune put on the wire is the
     * closest available truth; zero (= cannot say) until the first tune,
     * because the far end powers up on its own default and never says so.
     */
    override fun frequencyHz(): Long = freqHz

    override fun sampleRateHz(): Int = sampleRate.toInt()

    override fun setSampleRate(hz: Int) {
        require(hz in SUPPORTED_SAMPLE_RATES) {
            "rtl_tcp sample rate $hz is not an exact supported choice"
        }
        sendCommand(RTLCommand(0x02.toByte(), hz))
        sampleRate = hz.toDouble()
        spectrumWorker?.resetSmoothing()
    }

    fun setGainMode(manual: Boolean) {
        sendCommand(RTLCommand(0x03.toByte(), if (manual) 1 else 0))
    }

    fun setGain(gainTenthsOfDb: Int) {
        val metadata = tunerMetadata
        require(metadata.gainTableKnown) {
            "rtl_tcp server did not provide a verifiable tuner gain table"
        }
        require(gainTenthsOfDb in metadata.gainsTenthsDb) {
            "rtl_tcp tuner does not advertise exact gain $gainTenthsOfDb"
        }
        sendCommand(RTLCommand(0x04.toByte(), gainTenthsOfDb))
    }

    fun tunerInfo(): RtlTunerInfo = tunerMetadata.copy(
        gainsTenthsDb = tunerMetadata.gainsTenthsDb.toList(),
    )

    fun setFrequencyCorrection(ppm: Int) {
        require(ppm in -488..488) { "rtl_tcp correction $ppm ppm is outside -488..488" }
        sendCommand(RTLCommand(0x05.toByte(), ppm))
    }

    /** Contract name for the bias tee: one concept, one name across radios. */
    override fun setAntennaPower(on: Boolean) = setBiasTee(on)

    fun setBiasTee(enabled: Boolean) {
        Log.i("RTLTCPClient", "Setting Bias-T: ${if (enabled) "ENABLED" else "DISABLED"}")
        sendCommand(RTLCommand(0x0e.toByte(), if (enabled) 1 else 0))
    }

    fun setDirectSampling(mode: Int) {
        require(mode in 0..2) { "rtl_tcp direct-sampling mode $mode is outside 0..2" }
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
