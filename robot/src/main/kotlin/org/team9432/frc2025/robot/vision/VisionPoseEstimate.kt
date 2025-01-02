package org.team9432.frc2025.robot.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.util.Units
import kotlin.math.abs
import org.team9432.frc2025.robot.vision.VisionConstants.aprilTagLayout

@JvmRecord
data class VisionPoseEstimate(
    val pose: Pose3d,
    val timestamp: Double,
    val trackedTags: AprilTagList,
    val ambiguity: Double,
    val maxDistance: Double,
    val avgDistance: Double,
) {
    fun getFaults(): VisionEstimateFault {
        val outOfBounds =
            pose.x < 0.0 || pose.x > aprilTagLayout.fieldLength || pose.y < 0.0 || pose.y > aprilTagLayout.fieldWidth

        return VisionEstimateFault(
            outOfBounds,
            ambiguity > VisionConstants.MAX_AMBIGUITY,
            trackedTags.tags.isEmpty(),
            abs(pose.translation.z) > VisionConstants.MAX_Z_ERROR,
            abs(pose.rotation.y) > Units.degreesToRadians(VisionConstants.MAX_ANGLE_ERROR),
            abs(pose.rotation.x) > Units.degreesToRadians(VisionConstants.MAX_ANGLE_ERROR),
            VisionConstants.DISABLED,
        )
    }
}
