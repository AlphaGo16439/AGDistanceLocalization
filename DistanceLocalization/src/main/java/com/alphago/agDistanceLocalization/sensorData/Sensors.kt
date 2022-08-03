package com.alphago.agDistanceLocalization.sensorData

import com.alphago.agDistanceLocalization.geometry.Pose
import kotlin.math.*

enum class CalcPosition {
    X, Y, Nothing;
    fun isX() = (this == X); fun isY() = (this == Y); fun isNothing() = (this == Nothing)
}

abstract class Sensors(position: Pose) {
    abstract val id: String
    var distance: Double = 0.0
    var theta: Double = 0.0

    val x = position.x
    val y = position.y
    val rad = position.rad

    var inRange = false
    var calculateFor = CalcPosition.Nothing

    abstract fun update(distance: Double, theta: Double)
    abstract fun vertical(): Double
    abstract fun horizontal(): Double
}

class LeftSensor(position: Pose): Sensors(position) {
    override val id: String = "left"

    override fun update(distance: Double, theta: Double) {
        this.distance = distance
        this.theta = theta
    }

    override fun vertical(): Double {
       return if (y >= 0.0) abs(x * sin(theta) + y * cos(theta)) + abs(distance * sin(theta)) else ((x * sin(theta) + y * cos(theta)) - (distance * sin(theta))).absoluteValue
    }

    override fun horizontal(): Double {
        return abs(x * cos(theta) - y * sin(theta)) + abs(distance * cos(theta))
    }
}

class FrontSensor(position: Pose): Sensors(position) {
    override val id: String = "front"

    override fun update(distance: Double, theta: Double) {
        this.distance = distance
        this.theta = theta
    }

    override fun vertical(): Double {
        return abs(x * sin(theta) + y * cos(theta)) + abs(distance * cos(theta))
    }

    override fun horizontal(): Double {
        return abs(x * cos(theta) - y * sin(theta)) + abs(distance * sin(theta))
    }
}

class RightSensor(position: Pose): Sensors(position) {
    override val id: String = "right"

    override fun update(distance: Double, theta: Double) {
        this.distance = distance
        this.theta = theta
    }

    override fun vertical(): Double {
        return if (y >= 0.0) abs(x * sin(theta) + y * cos(theta)) + abs(distance * sin(theta)) else ((x * sin(theta) + y * cos(theta)) + (distance * sin(theta))).absoluteValue
    }

    override fun horizontal(): Double {
        return abs(x * cos(theta) - y * sin(theta)) + abs(distance * cos(theta))
    }
}