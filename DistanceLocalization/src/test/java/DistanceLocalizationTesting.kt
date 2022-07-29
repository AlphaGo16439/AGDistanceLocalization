import com.alphago.agDistanceLocalization.ThreeSensorLocalization
import com.alphago.agDistanceLocalization.geometry.Pose
import com.alphago.agDistanceLocalization.geometry.toRadians
import kotlin.math.PI

fun main() {
    val dl = ThreeSensorLocalization(
        Pose(-5.0, 0.0, PI), Pose(0.0, 5.0, PI/2.0), Pose(5.0, 0.0, 0.0),
        45.0, true
    )
    println(dl.update(8.05407, 10.55724, 18.33586, (130.0).toRadians))
}