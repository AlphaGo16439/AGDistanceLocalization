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

    private val fieldMaxX = 144.0
    private val fieldMaxY = 144.0

    private var poseEstimate = Pose(0.0, 0.0, 0.0)

    private enum class FinalPosition { ZERO_BASED, MAX_BASED; fun isMaxBased() = this == MAX_BASED }

    /**
     * Resets all variables to their default values
     */
    private fun resetVars() { xList.clear(); yList.clear(); numberOfSensorsInUse = 0 }

    /**
     * Update function to be called every loop you would like to calculate the pose estimate
     * @param leftSensorDistance the distance reported by the left sensor in INCHES
     * @param frontSensorDistance the distance reported by the front sensor in INCHES
     * @param rightSensorDistance the distance reported by the right sensor in INCHES
     * @param theta the angle of the robot in unit circle RADIANS
     * @return the pose estimate of the robot
     */
    fun update(leftSensorDistance: Double, frontSensorDistance: Double, rightSensorDistance: Double, theta: Double): Pose {
        resetVars()
        this.theta = theta

        readySensors(leftSensorDistance, frontSensorDistance, rightSensorDistance)
        calculate()
        estimate()

        return poseEstimate
    }

    /**
     * Readies the sensors to be used for calculations -
     * updates wrapper class, sensor in range, number of sensors in use,
     * and initial calculate position
     */
    private fun readySensors(left: Double, front: Double, right: Double) {
        val normalized = (theta - PI/2.0)
        sensors.forEach { s ->
            s.update(when(s.id) {"left" -> left; "front" -> front; else -> right }, normalized.contain2PI())
            s.inRange = (s.distance in 0.1..sensorDistanceSafety).ifTrue {
                numberOfSensorsInUse++
                s.calculateFor = when (closestCardinal((s.rad + normalized).contain2PI())) {
                    0.0 -> CalcPosition.X; PI/2.0 -> CalcPosition.Y; PI -> CalcPosition.X; else -> CalcPosition.Y
                }
            }
        }
    }

    /**
     * Adjusts which position the sensor calculates for based on
     * the angle of the robot and number of sensors in use
     */
    private fun calculate() {
        val chc = closestHalfCardinal(theta)
        if (numberOfSensorsInUse == 3) {
            fs.calculateFor = CalcPosition.Nothing
            when(chc) {
                PI/4.0, 5.0*PI/4.0 -> { ls.calculateFor = CalcPosition.Y; rs.calculateFor = CalcPosition.X }
                3.0*PI/4.0, 7.0*PI/4.0 -> { ls.calculateFor = CalcPosition.X; rs.calculateFor = CalcPosition.Y }
            }
        } else if (numberOfSensorsInUse == 2) {
            val normalized = (theta - chc)
            when(chc) {
                PI/4.0, 5.0*PI/4.0 -> {
                    if (ls.inRange && !rs.inRange && dEquals(fs.vertical(), ls.vertical())) normalized.checkRelZero(
                        { ls.calculateFor = CalcPosition.Y }, { fs.calculateFor = CalcPosition.Y }, { fs.calculateFor = CalcPosition.Y }
                    ) else if (rs.inRange && !ls.inRange && dEquals(rs.horizontal(), fs.horizontal())) normalized.checkRelZero(
                        { fs.calculateFor = CalcPosition.X }, { fs.calculateFor = CalcPosition.X }, { rs.calculateFor = CalcPosition.X }
                    )
                }
                3.0*PI/4.0, 7.0*PI/4.0 -> {
                    if (ls.inRange && !rs.inRange && dEquals(fs.horizontal(), ls.horizontal())) normalized.checkRelZero(
                        { ls.calculateFor = CalcPosition.X }, { fs.calculateFor = CalcPosition.X }, { fs.calculateFor = CalcPosition.X }
                    ) else if (rs.inRange && !ls.inRange && dEquals(rs.vertical(), fs.vertical())) normalized.checkRelZero(
                        { fs.calculateFor = CalcPosition.Y }, { fs.calculateFor = CalcPosition.Y }, { rs.calculateFor = CalcPosition.Y }
                    )
                }
            }
        }
    }

    /**
     * Identifies which side of the field the robot is on and estimates the position of the robot
     */
    private fun estimate() {
        sensors.filter { !it.calculateFor.isNothing() }.forEach { s ->
            var x = Double.NaN; var y = Double.NaN; var fp = FinalPosition.MAX_BASED

            if (s.calculateFor.isX()) x = s.horizontal() else if (s.calculateFor.isY()) y = s.vertical()

            if (s.calculateFor.isX() && within(theta, (-s.rad + 13.0*PI/12.0), (-s.rad + 23.0*PI/12.0)) ||
                s.calculateFor.isY() && within(theta, (-s.rad + 19.0*PI/12.0), (-s.rad + 5.0*PI/12.0))) {
                fp = FinalPosition.ZERO_BASED
            } else if (s.calculateFor.isX() && within(theta, (-s.rad + PI/12.0), (-s.rad + 11.0*PI/12.0)) ||
                s.calculateFor.isY() && within(theta, (-s.rad + 7.0*PI/12.0), (-s.rad + 17.0*PI/12.0))) {
                fp = FinalPosition.MAX_BASED
            }

            if (!x.isNaN()) xList.add(if (fp.isMaxBased()) fieldMaxX - x else x)
            if (!y.isNaN()) yList.add(if (fp.isMaxBased()) fieldMaxY - y else y)
        }

        if (xList.isEmpty()) xList.add(Double.NaN); if (yList.isEmpty()) yList.add(Double.NaN)

        poseEstimate = Pose(
            xList.average() round 3,
            yList.average() round 3,
            theta
        )
    }

    /**
     * Reports debug information about the current pose estimate
     */
    fun debug(): String {
        return """
            Left is calculating for ${ls.calculateFor}, with distance: ${ls.distance}
            Front is calculating for ${fs.calculateFor}, with distance: ${fs.distance}
            Right is calculating for ${rs.calculateFor}, with distance: ${rs.distance}
            
            x-list: $xList
            y-list: $yList
            
            x-list avg: ${xList.average()}
            y-list avg: ${yList.average()}
            
            $poseEstimate
        """.trimIndent()
    }
}