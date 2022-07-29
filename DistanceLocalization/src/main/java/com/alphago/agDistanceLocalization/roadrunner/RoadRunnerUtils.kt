package com.alphago.agDistanceLocalization.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.alphago.agDistanceLocalization.geometry.Pose
import com.alphago.agDistanceLocalization.geometry.TAU
import kotlin.math.PI

//converts Quadrant 1 positions to roadrunner positions
fun asRoadRunnerCoords(pose: Pose): Pose2d {
    return Pose2d(pose.y - 72.0, 72.0 - pose.x, asRoadRunnerHeading(pose.rad))
}

/**
 * converts unit circle(0 to 360 with 90 facing up) heading
 * to roadrunner heading (180 to -180 with 0.0 facing up and positive to the left)
 * USED WHEN CALLING .asRoadRunnerCoords()
 */
fun asRoadRunnerHeading(rad: Double): Double {
    var angle = rad - (PI/2.0)
    if (angle < 0.0) angle += TAU
    return when {
        angle < -Math.PI -> angle + TAU
        angle > Math.PI -> angle - TAU
        else -> angle
    }
}

/**
 * converts roadrunner (180 to -180 with 0.0 facing up and positive to the left) heading
 * to unit circle heading (0 to 360 with 90 facing up)
 * USED WHEN CALLING .update()
 */
fun asUnitCircleHeading(rad: Double): Double {
    var angle = when {
        rad < 0 -> rad + TAU
        rad > TAU -> rad - TAU
        else -> rad
    } + PI/2.0
    if (angle >= 2* PI) angle -= 2*PI
    return angle
}