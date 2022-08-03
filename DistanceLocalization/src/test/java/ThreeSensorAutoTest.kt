import com.alphago.agDistanceLocalization.ThreeSensorLocalization
import com.alphago.agDistanceLocalization.geometry.*
import kotlin.math.PI
import kotlin.math.absoluteValue

fun main() {
    val time = System.currentTimeMillis()
    val leftPose1 = Pose(-5.0, 0.0, PI)
    val leftPose2 = Pose(-8.25, 7.5, PI)
    val leftPose3 = Pose(-7.0, -5.0, PI)

    val frontPose1 = Pose(0.0, 5.0, PI/2.0)
    val frontPose2 = Pose(-3.5, 8.0, PI/2.0)
    val frontPose3 = Pose(5.0, 6.0, PI/2.0)

    val rightPose1 = Pose(5.0, 0.0, 0.0)
    val rightPose2 = Pose(8.25, 7.5, 0.0)
    val rightPose3 = Pose(9.0, 5.0, 0.0)

    //normal tests
    println(TestCase(leftPose1, frontPose1, rightPose1, 45.0, 0.0, 20.88190, 28.12884, 255.0, 32.0, 25.0))
    println(TestCase(leftPose2, frontPose2, rightPose2, 45.0, 0.0, 22.94195, 37.25387, 68.0, 144.0 - 45.0, 144.0 - 30.0))
    println(TestCase(leftPose3, frontPose3, rightPose3, 45.0, 22.71854, 16.04527, 0.0, 75.0, 30.0, 144.0 - 20.0))
    println()

    //two sensor same wall testing
    println(TestCase(leftPose1, frontPose1, rightPose1, 45.0, 12.68768, 23.30620, 0.0, 32.0, Double.NaN, 144.0 - 15.0))
    println(TestCase(leftPose2, frontPose2, rightPose2, 45.0, 0.0, 31.48825, 26.37212, 40.0, 144.0 - 28.0, Double.NaN))
    println(TestCase(leftPose3, frontPose3, rightPose3, 45.0, 29.11270, 30.11270, 0.0, 225.0, Double.NaN, 22.0))
    println()

    //all sensor corner condition
    println(TestCase(leftPose1, frontPose1, rightPose1, 45.0, 21.83257, 18.33267, 33.83208, 301.0, 144.0 - 23.0, 20.0))
    println(TestCase(leftPose2, frontPose2, rightPose2, 45.0, 16.65756, 11.57279, 33.30628, 105.0, 26.0, 144.0 - 18.0))
    println(TestCase(leftPose3, frontPose3, rightPose3, 45.0, 35.80870, 29.83249, 26.18221, 229.5, 30.0, 24.0))
    println((System.currentTimeMillis() - time) / 1000.0)
}

class TestCase(
    lsPos: Pose, fsPos: Pose, rsPose: Pose, maxDistance: Double,
    left: Double, front: Double, right: Double,
    thetaDEG: Double, private val expX: Double, private val expY: Double
) {
    private var success = false
    private val tsl: ThreeSensorLocalization
    private val pose: Pose

    init {
        tsl = ThreeSensorLocalization(lsPos, fsPos, rsPose, maxDistance)
        pose = tsl.update(left, front, right, thetaDEG.toRadians)
        //println(tsl.debug())

        if(!expX.isNaN() && !expY.isNaN())
            success = ((pose.x difference expX).absoluteValue < 0.01 && (pose.y difference expY).absoluteValue < 0.01)
        else if (expX.isNaN())
            success = (pose.x.isNaN() && (pose.y difference expY).absoluteValue < 0.01)
        else if (expY.isNaN())
            success = ((pose.x difference expX).absoluteValue < 0.01 && pose.y.isNaN())
    }
    override fun toString(): String = if (success) "SUCCESS" else "FAILED: calculated: ${pose.point}, expected: ${Point(expX, expY)}"
}

/* some testing stuff...ignore
fun main() {
    //val rad = (-90.0).toRadians
    //left x zero based

    */
/*val rotated = rad - (rad - PI/2.0)
    val max = (rotated + (PI - rad + PI/2.0)).contain2PI()
    val min = (rotated + (PI - rad - PI/2.0)).contain2PI()*//*


    //val max = (-rad + 5.0*PI/4.0).contain2PI()
    //val min = (-rad + 3.0*PI/4.0).contain2PI()

    */
/*val rotated = rad - (rad - PI/2.0)
    val max = (-rad + 3.0*PI/4.0 - PI/4.0).contain2PI()
    val min = (-rad + 5.0*PI/4.0 + PI/4.0).contain2PI()

    println(max.toDegrees round 3)
    println(min.toDegrees round 3)*//*


    val rad = (0.0).toRadians
    println(within((301.0).toRadians, (-rad + 7.0*PI/4.0 - PI/6.0), (-rad + PI/4.0 + PI/6.0)))
}*/
