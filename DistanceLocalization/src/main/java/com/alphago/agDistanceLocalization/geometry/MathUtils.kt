package com.alphago.agDistanceLocalization.geometry

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.round

const val TAU = 2.0*PI

infix fun Double.fmod(other: Double) = ((this % other) + other) % other
fun Double.contain2PI(): Double {
    var corrected = (this fmod (2.0* PI))
    if (corrected < 0.0) corrected += 2.0* PI
    return corrected
}

infix fun Double.round(decimals: Int): Double {
    var multiplier = 1.0
    repeat(decimals) { multiplier *= 10 }
    return round(this * multiplier) / multiplier
}

private fun findClosest(value: Double, list: ArrayList<Double>): Double {
    var dp = arrayListOf(0.0, Double.MAX_VALUE)
    for (i in list)
        if (abs(value - i) < dp[1]) dp = arrayListOf(list.indexOf(i).toDouble(), abs(value - i))
    return if (list[dp[0].toInt()] == 2.0*PI) 0.0 else list[dp[0].toInt()]
}

fun closestCardinal(rad: Double): Double {
    val closest = findClosest(rad, arrayListOf(0.0, PI/2.0, PI, 3.0*PI/2.0, 2.0* PI))
    return if (closest == 2.0*PI) 0.0 else closest
}

fun closestHalfCardinal(rad: Double): Double {
    return findClosest(rad, arrayListOf(PI/4.0, 3.0*PI/4.0, 5.0*PI/4.0, 7.0*PI/4.0))
}

val Double.toRadians get() = Math.toRadians(this)
val Double.toDegrees get() = Math.toDegrees(this)

infix fun Double.difference(other: Double) = (this - other)