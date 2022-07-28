package com.alphago.agDistanceLocalization.filters

class LowPassFilter(@JvmField var gain: Double) {
    private var previousEstimate = 0.0

    init {
        if (gain !in 0.0..1.0)
            throw IllegalArgumentException("Gain must be between 0.0 and 1.0")
    }

    infix fun estimate(measurement: Double): Double {
        val estimate = (gain * previousEstimate) + (1.0 - gain) * measurement
        previousEstimate = estimate
        return estimate
    }
}