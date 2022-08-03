package com.alphago.agDistanceLocalization.filters

class MedianFilter(private val n: Int) {
    private val window = ArrayList<Double>()

    init { if (n <= 0) throw IllegalArgumentException("n must not be <= 0") }

    fun push(value: Double): MedianFilter {
        window.add(value)
        if(window.size > n) window.removeAt(0)
        return this
    }

    fun median(): Double {
        val sorted = window.sorted()
        val size = sorted.size
        if(size == 0) return 0.0
        return if(size % 2 == 0)
            (sorted[(size / 2)] + sorted[(size / 2 - 1)]) / 2.0
        else
            sorted[(size / 2)]
    }
}