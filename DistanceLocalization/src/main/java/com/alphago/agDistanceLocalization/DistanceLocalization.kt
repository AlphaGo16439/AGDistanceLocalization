package com.alphago.agDistanceLocalization

import com.alphago.agDistanceLocalization.geometry.*
import com.alphago.agDistanceLocalization.sensorData.*
import kotlin.math.*

class DistanceLocalization(
    leftSensorPosition: Pose,
    frontSensorPosition: Pose,
    rightSensorPosition: Pose,
    private val sensorDistanceSafety: Double,
    private val eval: Boolean = false
) {
    private val ls = LeftSensor(leftSensorPosition)
    private val fs = FrontSensor(frontSensorPosition)
    private val rs = RightSensor(rightSensorPosition)
    private val sensors get() = listOf(ls, fs, rs)

    private var theta: Double = 0.0

    private val xList = mutableListOf<Double>()
    private val yList = mutableListOf<Double>()

    private val q1Max = 144.0

    private var poseEstimate = Pose(0.0, 0.0, 0.0)

    private enum class FinalPosition {ZERO_BASED, MAX_BASED}
    private var finalXOverride: FinalPosition? = null
    private var finalYOverride: FinalPosition? = null

    fun update(leftSensorDistance: Double, frontSensorDistance: Double, rightSensorDistance: Double, theta: Double): Pose {
        this.theta = theta
        sensors.forEach { sensor ->
            sensor.update(
                when(sensor.id) {
                    "left" -> leftSensorDistance
                    "front" -> frontSensorDistance
                    else -> rightSensorDistance
                }, (this.theta - PI/2.0).contain2PI()
            )
        }
        run()
        return poseEstimate
    }

    private fun readySensors() {
        sensors.forEach { sensor ->
            sensor.apply {
                inRange = distance in 0.25..sensorDistanceSafety
                if (inRange) calculateFor = when(closestCardinal((position.rad + theta).contain2PI())) {
                    0.0 -> CalcPosition.X; PI/2.0 -> CalcPosition.Y; PI -> CalcPosition.X; else -> CalcPosition.Y
                }
            }
        }
    }

    private fun run() {
        readySensors()
        xList.clear(); yList.clear()
        finalXOverride = null; finalYOverride = null
        sensors.forEach { sensor ->
            sensor.apply {
                if (inRange)
                    if (numberOfSensorsInUse() != 3) calcRawPosition(this) else calcRawPositionAllSensors(this)
            }
        }
        if (xList.isEmpty()) xList.add(Double.NaN); if (yList.isEmpty()) yList.add(Double.NaN)
        poseEstimate = Pose(
            (if(calcFinalX() == FinalPosition.MAX_BASED) (q1Max - xList.average()) else (xList.average())) round 3,
            (if(calcFinalY() == FinalPosition.MAX_BASED) (q1Max - yList.average()) else (yList.average())) round 3,
            theta
        )
        if (eval) evaluate()
    }

    private fun numberOfSensorsInUse(): Int {
        var count = 0
        sensors.forEach { if (it.calculateFor != CalcPosition.Nothing) count++ }
        return count
    }

    private fun calcRawPosition(sensor: Sensors) {
        sensor.apply {
            if (calculateFor == CalcPosition.X)
                xList.add(horizontalComponent())
            else if (calculateFor == CalcPosition.Y)
                yList.add(verticalComponent())
        }
    }

    private fun calcRawPositionAllSensors(sensor: Sensors) {
        when(val closestHC = closestHalfCardinal(theta)) {
            PI/4.0, 5.0*PI/4.0 -> {
                sensor.apply {
                    if (id == "left") yList.add(verticalComponent())
                    if (id == "front") {
                        if ((verticalComponent() difference ls.verticalComponent()).absoluteValue < (horizontalComponent() difference rs.horizontalComponent()).absoluteValue)
                            yList.add(verticalComponent())
                        else if ((horizontalComponent() difference rs.horizontalComponent()).absoluteValue < (verticalComponent() difference ls.verticalComponent()).absoluteValue)
                            xList.add(horizontalComponent())
                    }
                    if (id == "right") xList.add(horizontalComponent())
                }
                finalXOverride = if (closestHC == PI/4.0) FinalPosition.MAX_BASED else FinalPosition.ZERO_BASED
                finalYOverride = if (closestHC == PI/4.0) FinalPosition.MAX_BASED else FinalPosition.ZERO_BASED
            }

            3.0*PI/4.0, 7.0*PI/4.0 -> {
                sensor.apply {
                    if (id == "left") xList.add(horizontalComponent())
                    if (id == "front") {
                        if ((verticalComponent() difference rs.verticalComponent()).absoluteValue < (horizontalComponent() difference ls.horizontalComponent()).absoluteValue)
                            yList.add(verticalComponent())
                        else if ((horizontalComponent() difference ls.horizontalComponent()).absoluteValue < (verticalComponent() difference rs.verticalComponent()).absoluteValue)
                            xList.add(horizontalComponent())
                    }
                    if (id == "right") yList.add(verticalComponent())
                }
                finalXOverride = if (closestHC == 3.0*PI/4.0) FinalPosition.ZERO_BASED else FinalPosition.MAX_BASED
                finalYOverride = if (closestHC == 3.0*PI/4.0) FinalPosition.MAX_BASED else FinalPosition.ZERO_BASED
            }
            else -> calcRawPosition(sensor)
        }
    }

    private fun calcFinalX(): FinalPosition {
        finalXOverride?.apply { return this }
        if ((ls.calculateFor == CalcPosition.X && theta in (PI/4.0)..(3.0*PI/4.0))
            || (fs.calculateFor == CalcPosition.X && theta in (3.0*PI/4.0)..(5.0*PI/4.0))
            || (rs.calculateFor == CalcPosition.X && theta in (5.0*PI/4.0)..(7.0*PI/4.0))
            || xList.contains(Double.NaN)) return FinalPosition.ZERO_BASED
        return FinalPosition.MAX_BASED
    }

    private fun calcFinalY(): FinalPosition {
        finalYOverride?.apply { return this }
        if ((ls.calculateFor == CalcPosition.Y && theta in (PI/4.0)..(7.0*PI/4.0))
            || (fs.calculateFor == CalcPosition.Y && theta in (7.0*PI/4.0)..(5.0*PI/4.0))
            || (rs.calculateFor == CalcPosition.Y && theta in (5.0*PI/4.0)..(3.0*PI/4.0))
            || yList.contains(Double.NaN)) return FinalPosition.ZERO_BASED
        return FinalPosition.MAX_BASED
    }

    private fun evaluate() {
        sensors.forEach { println("${it.id} is calculating for ${it.calculateFor} with distance ${it.distance}") }
        println()
        println("x-list: $xList")
        println("y-list: $yList")
        println()
        println("x-list avg: ${xList.average() round 3}")
        println("y-list avg: ${yList.average() round 3}")
        println()
    }
}