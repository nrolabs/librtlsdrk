/*
 * librtlsdrk - GPL driver metadata contract for RTL-SDR tuners
 * Copyright (C) 2026 Isak Ruas <isakruas@gmail.com>
 */
package com.isaklab.librtlsdrk

import com.isaklab.isdrproto.DriverProto

/** Defensive ceiling shared by host encoding and app decoding. */
const val MAX_RTL_GAIN_STEPS = DriverProto.RTL_INFO_MAX_GAIN_STEPS

/** Stable tuner ids defined by the original rtl_tcp greeting. */
object RtlTunerType {
    const val UNKNOWN = 0
    const val E4000 = 1
    const val FC0012 = 2
    const val FC0013 = 3
    const val FC2580 = 4
    const val R820T = 5
    const val R828D = 6

    fun name(type: Int): String = when (type) {
        UNKNOWN -> "Unknown tuner"
        E4000 -> "Elonics E4000"
        FC0012 -> "Fitipower FC0012"
        FC0013 -> "Fitipower FC0013"
        FC2580 -> "FCI FC2580"
        R820T -> "Rafael Micro R820T"
        R828D -> "Rafael Micro R828D"
        else -> "Unknown tuner ($type)"
    }
}

/**
 * Exact tuner metadata. [gainTableKnown] is independent from table size: a
 * known empty table is valid, while an unknown table must carry no values.
 */
data class RtlTunerInfo(
    val tunerType: Int,
    val tunerName: String,
    val gainTableKnown: Boolean,
    val gainsTenthsDb: List<Int>,
) {
    init {
        require(gainsTenthsDb.size <= MAX_RTL_GAIN_STEPS) {
            "RTL gain table exceeds $MAX_RTL_GAIN_STEPS entries"
        }
        require(gainTableKnown || gainsTenthsDb.isEmpty()) {
            "unknown RTL gain table must not carry invented values"
        }
        require(gainsTenthsDb.distinct().size == gainsTenthsDb.size) {
            "RTL gain table contains duplicate values"
        }
        require(gainsTenthsDb.zipWithNext().all { (a, b) -> a < b }) {
            "RTL gain table must be strictly increasing"
        }
    }

    companion object {
        fun unknown(tunerType: Int = RtlTunerType.UNKNOWN): RtlTunerInfo =
            RtlTunerInfo(tunerType, RtlTunerType.name(tunerType), false, emptyList())
    }
}

/**
 * rtl_tcp v1 reports tuner id and gain count, but not the values. A standard
 * table is authoritative only when both the tuner id and advertised count
 * match exactly; a custom/extended server remains explicitly unknown.
 */
internal fun rtlTcpTunerInfo(tunerType: Int, advertisedGainCount: Int): RtlTunerInfo {
    require(advertisedGainCount in 0..MAX_RTL_GAIN_STEPS) {
        "rtl_tcp advertised invalid gain count $advertisedGainCount"
    }
    val standard = when (tunerType) {
        RtlTunerType.E4000 -> E4kTuner.GAINS
        RtlTunerType.FC0012 -> Fc0012Tuner.GAINS
        RtlTunerType.FC0013 -> Fc0013Tuner.GAINS
        RtlTunerType.FC2580 -> Fc2580Tuner.GAINS
        RtlTunerType.R820T,
        RtlTunerType.R828D -> R82xxTuner.GAINS
        else -> null
    }
    val known = standard != null && standard.size == advertisedGainCount
    return RtlTunerInfo(
        tunerType = tunerType,
        tunerName = RtlTunerType.name(tunerType),
        gainTableKnown = known,
        gainsTenthsDb = if (known) standard!!.toList() else emptyList(),
    )
}
