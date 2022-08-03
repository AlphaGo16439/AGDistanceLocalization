package com.alphago.agDistanceLocalization.geometry

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.round

const val TAU = 2.0*PI

infix fun Double.fmod(other: Double) = ((this % other) + other) % other
fun Double.contain2PI(): Double {
    var corrected = (this fmod TAU)
    if (corrected < 0.0) corrected += TAU
    return corrected
}

infix fun Double.round(decimals: Int): Double {
    var multiplier = 1.0
    repeat(decimals) { multiplier *= 10 }
    return round(this * multiplier) / multiplier
}

private fun findClosest(value: Double, list: ArrayList<Double>): Double {
    var dp = arrayListOf(0.0, Double.MAX_VALUE)
    for (i in list) if (abs(value - i) < dp[1]) dp = arrayListOf(list.indexOf(i).toDouble(), abs(value - i))
    return list[dp[0].toInt()]
}

fun closestCardinal(rad: Double): Double {
    val closest = findClosest(rad, arrayListOf(0.0, PI/2.0, PI, 3.0*PI/2.0, TAU))
    return if (closest == TAU) 0.0 else closest
}

fun closestHalfCardinal(rad: Double): Double
    = findClosest(rad, arrayListOf(PI/4.0, 3.0*PI/4.0, 5.0*PI/4.0, 7.0*PI/4.0))

val Double.toRadians get() = Math.toRadians(this)
val Double.toDegrees get() = Math.toDegrees(this)

infix fun Double.difference(other: Double) = (this - other)
fun dEquals(d1: Double, d2: Double) = abs(d1 difference d2) < 1.0
fun Double.checkRelZero(greater: () -> Unit, equal: () -> Unit, lesser: () -> Unit) {
    when {
        this > 0.0 -> greater.invoke()
        this == 0.0 -> equal.invoke()
        this < 0.0 -> lesser.invoke()
    }
}

fun within(value: Double, baseMin: Double, baseMax: Double): Boolean {
    val list = listOf(baseMin.contain2PI(), baseMax.contain2PI())
    var mValue = value; if (abs(value - list[0]) < 0.001) mValue = list[0] else if (abs(value - list[1]) < 0.001) mValue = list[1]
    val min = list.min(); val max = list.max()
    val path = if ((min - max).contain2PI() < (max - min).contain2PI()) max..min else min..max
    val pStart = path.start; var pEnd = path.endInclusive
    if (pStart in 3.0*PI/2.0..2.0*PI && pEnd in 0.0..PI/2.0) pEnd += TAU
    return mValue in pStart..pEnd
}

inline fun Boolean.ifTrue(block: Boolean.() -> Unit): Boolean {
    if (this) block()
    return this
}