package com.alphago.agDistanceLocalization.sensorData

import com.alphago.agDistanceLocalization.geometry.Pose
import kotlin.math.*

enum class CalcPosition { X, Y, Nothing }

abstract class Sensors(val position: Pose) {
    abstract val id: String
    abstract var distance: Double
    abstract var theta: Double

    val x = position.x
    val y = position.y

    var inRange = false
    var calculateFor = CalcPosition.Nothing

    abstract fun update(distance: Double, theta: Double): Sensors
    abstract fun verticalComponent(): Double
    abstract fun horizontalComponent(): Double
}

class LeftSensor(position: Pose): Sensors(position) {
    override val id: String = "left"
    override var distance: Double = 0.0
    override var theta: Double = 0.0

    override fun update(distance: Double, theta: Double): Sensors {
        this.distance = distance
        this.theta = theta
        return this
    }

    override fun verticalComponent(): Double {
       return if (y >= 0.0) abs(x * sin(theta) + y * cos(theta)) + abs(distance * sin(theta)) else ((x * sin(theta) + y * cos(theta)) - (distance * sin(theta))).absoluteValue
    }

    override fun horizontalComponent(): Double {
        return abs(x * cos(theta) - y * sin(theta)) + abs(distance * cos(theta))
    }
}

class FrontSensor(position: Pose): Sensors(position) {
    override val id: String = "front"
    override var distance: Double = 0.0
    override var theta: Double = 0.0

    override fun update(distance: Double, theta: Double): Sensors {
        this.distance = distance
        this.theta = theta
        return this
    }

    override fun verticalComponent(): Double {
        return abs(x * sin(theta) + y * cos(theta)) + abs(distance * cos(theta))
    }

    override fun horizontalComponent(): Double {
        return abs(x * cos(theta) - y * sin(theta)) + abs(distance * sin(theta))
    }
}

class RightSensor(position: Pose): Sensors(position) {
    override val id: String = "right"
    override var distance: Double = 0.0
    override var theta: Double = 0.0

    override fun update(distance: Double, theta: Double): Sensors {
        this.distance = distance
        this.theta = theta
        return this
    }

    override fun verticalComponent(): Double {
        return if (y >= 0.0) abs(x * sin(theta) + y * cos(theta)) + abs(distance * sin(theta)) else ((x * sin(theta) + y * cos(theta)) + (distance * sin(theta))).absoluteValue
    }

    override fun horizontalComponent(): Double {
        return abs(x * cos(theta) - y * sin(theta)) + abs(distance * cos(theta))
    }
}