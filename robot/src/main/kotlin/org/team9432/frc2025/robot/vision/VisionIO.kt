package org.team9432.frc2025.robot.vision

import edu.wpi.first.math.geometry.Pose3d
import org.team9432.annotation.Logged

interface VisionIO {
    @Logged
    open class VisionIOInputs {
        var connected: Boolean = false
        var poseObservations: Array<PoseObservation> = emptyArray()
        var tagIds: IntArray = IntArray(0)
    }

    /** Represents a robot pose sample used for pose estimation. */
    @JvmRecord
    data class PoseObservation(
        val timestamp: Double,
        val pose: Pose3d,
        val ambiguity: Double,
        val tagCount: Int,
        val averageTagDistance: Double,
    )

    val config: CameraConfig

    fun updateInputs(inputs: VisionIOInputs) {}
}
