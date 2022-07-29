import com.alphago.agDistanceLocalization.DistanceLocalization
import com.alphago.agDistanceLocalization.geometry.Pose
import com.alphago.agDistanceLocalization.geometry.closestHalfCardinal
import com.alphago.agDistanceLocalization.geometry.toDegrees
import com.alphago.agDistanceLocalization.geometry.toRadians
import com.alphago.agDistanceLocalization.roadrunner.asRoadRunnerCoords
import kotlin.math.PI

fun main() {
    val dl = DistanceLocalization(
        Pose(-5.0, 0.0, PI), Pose(0.0, 5.0, PI/2.0), Pose(5.0, 0.0, 0.0),
        45.0, false
    )
    println(dl.update(18.09401, 12.32051, 25.00000, (300.0).toRadians))
}