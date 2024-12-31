package org.team9432.frc2025.robot.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import org.team9432.annotation.Logged

interface VisionIO {
    @Logged
    open class VisionIOInputs {
        var connected: Boolean = false
        var latestTargetObservation: TargetObservation = TargetObservation(Rotation2d(), Rotation2d())
        var poseObservations: Array<PoseObservation> = emptyArray()
        var tagIds: IntArray = IntArray(0)
    }

    /** Represents the angle as a simple target, not used for pose estimation. */
    @JvmRecord data class TargetObservation(val tx: Rotation2d, val ty: Rotation2d)

    /** Represents a robot pose sample used for pose estimation. */
    @JvmRecord
    data class PoseObservation(
        val timestamp: Double,
        val pose: Pose3d,
        val ambiguity: Double,
        val tagCount: Int,
        val averageTagDistance: Double,
        val type: PoseObservationType,
    )

    enum class PoseObservationType {
        PHOTONVISION
    }

    fun updateInputs(inputs: VisionIOInputs) {}
}
