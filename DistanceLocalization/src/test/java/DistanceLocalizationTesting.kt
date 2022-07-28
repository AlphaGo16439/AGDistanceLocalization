import com.alphago.agDistanceLocalization.DistanceLocalization
import com.alphago.agDistanceLocalization.geometry.Pose
import kotlin.math.PI

fun main() {
    val time = System.currentTimeMillis()
    val dl = DistanceLocalization(
        Pose(-8.25, 7.5, PI),
        Pose(0.0, 8.0, PI/2.0),
        Pose(8.25, 7.5, 0.0),
        45.0,
        Math.toRadians(10.0)
    ).update(0.0, 30.0, 30.0, PI/2.0)
    println(dl)
    println((System.currentTimeMillis() - time) / 1000.0)
}