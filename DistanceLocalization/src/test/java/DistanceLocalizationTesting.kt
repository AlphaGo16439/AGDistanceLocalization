import com.alphago.agDistanceLocalization.DistanceLocalization
import com.alphago.agDistanceLocalization.geometry.Pose
import com.alphago.agDistanceLocalization.geometry.toRadians
import com.alphago.agDistanceLocalization.roadrunner.asRoadRunnerCoords
import kotlin.math.PI

fun main() {
    val time = System.currentTimeMillis()
    val dl = DistanceLocalization(
        Pose(5.0, 0.0, PI),
        Pose(0.0, 5.0, PI/2.0),
        Pose(5.0, 0.0, 0.0),
        45.0,
        Math.toRadians(10.0)
    ).update(0.0, 19.0, 19.0, (135.0).toRadians)
    println(dl)
    println(asRoadRunnerCoords(dl))
    println((System.currentTimeMillis() - time) / 1000.0)
}