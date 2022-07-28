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

fun closestCardinal(rad: Double): Double {
    val cardinalAngles = arrayListOf(0.0, PI /2, PI, 3.0* PI /2.0, 2.0* PI); var dp = arrayListOf(0.0, 10.0)
    for (i in cardinalAngles)
        if (abs(rad - i) < dp[1]) dp = arrayListOf(cardinalAngles.indexOf(i).toDouble(), abs(rad - i))
    return if (cardinalAngles[dp[0].toInt()] == 2.0*PI) 0.0 else cardinalAngles[dp[0].toInt()]
}

val Double.toRadians get() = Math.toRadians(this)
val Double.toDegrees get() = Math.toDegrees(this)