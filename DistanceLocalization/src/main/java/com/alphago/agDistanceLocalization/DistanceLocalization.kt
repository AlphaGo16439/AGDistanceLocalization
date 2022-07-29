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

    private fun assignSensors() {
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
        assignSensors()
        xList.clear(); yList.clear()
        sensors.forEach { sensor ->
            sensor.apply {
                if (inRange)
                    if (numberOfSensorsInUse() != 3) calcXY(this) else calcXYSpecialCondition(this)
            }
        }
        if (xList.isEmpty()) xList.add(Double.NaN); if (yList.isEmpty()) yList.add(Double.NaN)
        poseEstimate = Pose(
            (if(calcXpe()) (q1Max - xList.average()) else (xList.average())) round 3,
            (if(calcYpe()) (q1Max - yList.average()) else (yList.average())) round 3,
            theta
        )
        if (eval) evaluate()
    }

    private fun numberOfSensorsInUse(): Int {
        var count = 0
        sensors.forEach { if (it.calculateFor != CalcPosition.Nothing) count++ }
        return count
    }

    private fun calcXY(sensor: Sensors) {
        sensor.apply {
            if (calculateFor == CalcPosition.X)
                xList.add(horizontalComponent())
            else if (calculateFor == CalcPosition.Y)
                yList.add(verticalComponent())
        }
    }

    private fun calcXYSpecialCondition(sensor: Sensors) {
        when(closestHalfCardinal(theta)) {
            PI/4.0, 5.0*PI/4.0 -> {
                sensor.apply {
                    if (id == "left") yList.add(verticalComponent())
                    if (id == "front") {
                        if ((verticalComponent() difference ls.verticalComponent()) threshold 1.0)
                            yList.add(verticalComponent())
                        else if ((horizontalComponent() difference rs.horizontalComponent()) threshold 1.0)
                            xList.add(horizontalComponent())
                    }
                    if (id == "right") xList.add(horizontalComponent())
                }
            }
            3.0*PI/4.0, 7.0*PI/4.0 -> {
                sensor.apply {
                    if (id == "left") xList.add(horizontalComponent())
                    if (id == "front") {
                        if ((verticalComponent() difference rs.verticalComponent()) threshold 1.0)
                            yList.add(verticalComponent())
                        else if ((horizontalComponent() difference ls.horizontalComponent()) threshold 1.0)
                            xList.add(horizontalComponent())
                    }
                    if (id == "right") yList.add(verticalComponent())
                }
            }
            else -> calcXY(sensor)
        }
    }

    private fun calcXpe(): Boolean {
        if ((ls.calculateFor == CalcPosition.X && theta in (PI/4.0)..(3.0*PI/4.0))
            || (fs.calculateFor == CalcPosition.X && theta in (3.0*PI/4.0)..(5.0*PI/4.0))
            || (rs.calculateFor == CalcPosition.X && theta in (5.0*PI/4.0)..(7.0*PI/4.0))
            || xList.contains(Double.NaN)) return false
        return true //true means (144 - x), false means x
    }

    private fun calcYpe(): Boolean {
        if ((ls.calculateFor == CalcPosition.Y && theta in (PI/4.0)..(7.0*PI/4.0))
            || (fs.calculateFor == CalcPosition.Y && theta in (7.0*PI/4.0)..(5.0*PI/4.0))
            || (rs.calculateFor == CalcPosition.Y && theta in (5.0*PI/4.0)..(3.0*PI/4.0))
            || yList.contains(Double.NaN)) return false
        return true //true means (144 - y), false means y
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