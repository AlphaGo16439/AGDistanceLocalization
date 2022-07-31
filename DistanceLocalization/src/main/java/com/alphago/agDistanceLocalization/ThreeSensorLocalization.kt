package com.alphago.agDistanceLocalization

import com.alphago.agDistanceLocalization.geometry.*
import com.alphago.agDistanceLocalization.sensorData.*
import kotlin.math.*

class ThreeSensorLocalization(
    leftSensorPosition: Pose,
    frontSensorPosition: Pose,
    rightSensorPosition: Pose,
    private val sensorDistanceSafety: Double
) {
    private val ls = LeftSensor(leftSensorPosition)
    private val fs = FrontSensor(frontSensorPosition)
    private val rs = RightSensor(rightSensorPosition)
    private val sensors = listOf(ls, fs, rs)
    private var numberOfSensorsInUse = 0

    private var theta: Double = 0.0

    private val xList = mutableListOf<Double>()
    private val yList = mutableListOf<Double>()

    private var fieldMaxX = 144.0
    private var fieldMaxY = 144.0

    private var poseEstimate = Pose(0.0, 0.0, 0.0)

    private enum class FinalPosition { ZERO_BASED, MAX_BASED }
    private var finalXOverride: FinalPosition? = null
    private var finalYOverride: FinalPosition? = null
    
    private val componentDifferenceThreshold = 1.0

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

    private fun resetVars() { xList.clear(); yList.clear(); finalXOverride = null; finalYOverride = null; numberOfSensorsInUse = 0 }

    private fun readySensors() {
        resetVars()
        sensors.forEach { sensor ->
            sensor.apply {
                inRange = distance in 0.25..sensorDistanceSafety
                if (inRange) {
                    calculateFor = when (closestCardinal((position.rad + theta).contain2PI())) {
                        0.0 -> CalcPosition.X; PI/2.0 -> CalcPosition.Y; PI -> CalcPosition.X; else -> CalcPosition.Y
                    }
                    numberOfSensorsInUse++
                }
            }
        }
        if (numberOfSensorsInUse == 2) adjustPositionCalculationTwoSensors()
    }

    private fun run() {
        readySensors()
        sensors.forEach { sensor ->
            sensor.apply {
                if (inRange) { if (numberOfSensorsInUse != 3) calcRawPosition(this) else calcRawPositionAllSensors(this) }
            }
        }
        if (xList.isEmpty()) xList.add(Double.NaN); if (yList.isEmpty()) yList.add(Double.NaN)
        poseEstimate = Pose(
            (if(calcFinalX() == FinalPosition.MAX_BASED) (fieldMaxX - xList.average()) else (xList.average())) round 3,
            (if(calcFinalY() == FinalPosition.MAX_BASED) (fieldMaxY - yList.average()) else (yList.average())) round 3,
            theta
        )
    }

    private fun adjustPositionCalculationTwoSensors() {
        when (val chc = closestHalfCardinal(theta)) {
            PI/4.0, 5.0*PI/4.0 -> {
                if (ls.inRange) {
                    if ((theta - chc) > 0.0 && (fs.verticalComponent() difference ls.verticalComponent()).absoluteValue < componentDifferenceThreshold)
                        ls.calculateFor = CalcPosition.Y
                    else if ((theta - chc) < 0.0 && (ls.verticalComponent() difference fs.verticalComponent()).absoluteValue < componentDifferenceThreshold)
                        fs.calculateFor = CalcPosition.Y
                    else if (theta - chc == 0.0 && (ls.verticalComponent() difference fs.verticalComponent()).absoluteValue < componentDifferenceThreshold)
                        fs.calculateFor = CalcPosition.Y
                } else if (rs.inRange) {
                    if ((theta - chc) > 0.0 && (rs.horizontalComponent() difference fs.horizontalComponent()).absoluteValue < componentDifferenceThreshold)
                        fs.calculateFor = CalcPosition.X
                    else if ((theta - chc) < 0.0 && (fs.horizontalComponent() difference rs.horizontalComponent()).absoluteValue < componentDifferenceThreshold)
                        rs.calculateFor = CalcPosition.X
                    else if (theta == chc && (rs.horizontalComponent() difference fs.horizontalComponent()).absoluteValue < componentDifferenceThreshold)
                        fs.calculateFor = CalcPosition.X
                }
            }
            3.0*PI/4.0, 7.0*PI/4.0 -> {
                if (ls.inRange) {
                    if ((theta - chc) > 0.0 && (fs.horizontalComponent() difference ls.horizontalComponent()).absoluteValue < componentDifferenceThreshold)
                        ls.calculateFor = CalcPosition.X
                    else if ((theta - chc) < 0.0 && (ls.horizontalComponent() difference fs.horizontalComponent()).absoluteValue < componentDifferenceThreshold)
                        fs.calculateFor = CalcPosition.X
                    else if (theta == chc && (ls.horizontalComponent() difference fs.horizontalComponent()).absoluteValue < componentDifferenceThreshold)
                        fs.calculateFor = CalcPosition.X
                } else if (rs.inRange) {
                    if ((theta - chc) > 0.0 && (rs.verticalComponent() difference fs.verticalComponent()).absoluteValue < componentDifferenceThreshold)
                        fs.calculateFor = CalcPosition.Y
                    else if ((theta - chc) < 0.0 && (fs.verticalComponent() difference rs.verticalComponent()).absoluteValue < componentDifferenceThreshold)
                        rs.calculateFor = CalcPosition.Y
                    else if (theta == chc && (rs.verticalComponent() difference fs.verticalComponent()).absoluteValue < componentDifferenceThreshold)
                        fs.calculateFor = CalcPosition.Y
                }
            }
        }
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
            || (fs.calculateFor == CalcPosition.Y && theta in (5.0*PI/4.0)..(7.0*PI/4.0))
            || (rs.calculateFor == CalcPosition.Y && theta in (3.0*PI/4.0)..(5.0*PI/4.0))
            || yList.contains(Double.NaN)) return FinalPosition.ZERO_BASED
        return FinalPosition.MAX_BASED
    }

    fun overrideFieldMax(xMax: Double, yMax: Double) {
        fieldMaxX = xMax
        fieldMaxY = yMax
    }

    fun debug(): String {
        return """
            Left is calculating for ${ls.calculateFor} with distance ${ls.distance}
            Front is calculating for ${fs.calculateFor} with distance ${fs.distance}
            Right is calculating for ${rs.calculateFor} with distance ${rs.distance}
            
            x-list: $xList
            y-list: $yList
            
            x-list avg ${xList.average()}
            y-list avg ${yList.average()}
        """.trimIndent()
    }
}