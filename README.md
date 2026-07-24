# librtlsdrk

Kotlin port of [librtlsdr (RTL-SDR Blog fork)](https://github.com/rtlsdrblog/rtl-sdr-blog)
for Android, built directly on the Android USB host API — no NDK, no libusb,
no root required.

Maintained by Isak — **PU3IAR**. Brought to you by [id.qsl.br](https://id.qsl.br),
a platform with tools for amateur radio operators.
YouTube: [@qraisak](https://www.youtube.com/@qraisak)

## Features

- **RTL2832U driver** (`RTLUSBClient`): faithful port of `librtlsdr.c` —
  baseband init, sample rate/resampler, IF/DDC tuning, PPM correction,
  direct sampling (HF), offset tuning, bias tee (including the forced
  bias-tee EEPROM hack), EEPROM read, rtl_tcp-style command interface.
- **All five tuner drivers**, ported line by line from the C sources:
  - `R82xxTuner` — Rafael Micro R820T/R828D, including the RTL-SDR Blog
    hacks (max VCO current, 2.0 V PLL dropout) and **RTL-SDR Blog V4 / V4L**
    support (built-in HF upconverter, notch switching, input selection)
  - `E4kTuner` — Elonics E4000
  - `Fc0012Tuner` / `Fc0013Tuner` — Fitipower FC0012/FC0013
  - `Fc2580Tuner` — FCI FC2580
- **`RTLTCPClient`**: client for remote `rtl_tcp` servers with the same
  callback interface as the USB driver.
- IQ samples are delivered as interleaved `FloatArray` (`i0, q0, i1, q1, …`
  in `[-1, 1]`) — no per-sample object allocation on the streaming path.
- All USB register access serialized on a single worker thread; commands
  are applied between bulk transfers without interrupting the IQ stream.

## Usage

```kotlin
val client = RTLUSBClient(
    context = context,
    onDataReceived = { spectrum, iq ->
        // spectrum: FloatArray of dB values (FFT), iq: interleaved samples
    },
    onConnectionStatusChanged = { connected, status -> /* UI feedback */ }
)

lifecycleScope.launch {
    if (client.connect()) {
        client.setFrequency(100_000_000L)          // 100 MHz
        client.sendCommand(RTLCommand.SetSampleRate(2_048_000L))
        client.sendCommand(RTLCommand.SetGain(280)) // tenths of dB
    }
}

// later
client.disconnect()  // instances are single-use; create a new one to reconnect
```

The app must declare USB host support and (optionally) a `device_filter.xml`
with the RTL2832U vendor/product IDs to get plug-in intents. See the
[iSDR app](https://github.com/nrolabs) for a complete integration,
including the recommended manifest entries.

## Using as a git submodule

The package path is `com.isaklab.librtlsdrk`. Add the repository as a
submodule at the matching source directory:

```sh
git submodule add https://github.com/nrolabs/librtlsdrk \
    app/src/main/java/com/isaklab/librtlsdrk
```

## License

GNU General Public License v2 or later — the same license as the original
C library. See [LICENSE](LICENSE) for the full text and [COPYING.md](COPYING.md)
for provenance. Original authors are credited in each file's header:
Steve Markgraf, Dimitri Stolnikov, Mauro Carvalho Chehab, Harald Welte,
Sylvain Munaut, Hoernchen, Hans-Frieder Vogt, Fitipower Integrated
Technology and contributors.
