package com.alphago.agDistanceLocalization.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.alphago.agDistanceLocalization.geometry.Pose
import kotlin.math.PI

fun asRoadRunnerCoords(pose: Pose): Pose2d {
    return Pose2d(pose.y - 72.0, 72.0 - pose.x, PI /2 + pose.rad)
}

fun asUnitCircleHeading(rad: Double): Double {
    var theta = rad
    if (theta >= PI * 2.0) theta -= PI * 2.0
    if (theta < 0.0) theta += PI * 2.0
    return theta
}