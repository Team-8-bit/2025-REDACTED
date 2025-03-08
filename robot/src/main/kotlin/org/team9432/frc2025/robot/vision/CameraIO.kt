package org.team9432.frc2025.robot.vision

import edu.wpi.first.math.geometry.Pose3d
import org.team9432.annotation.Logged

interface CameraIO {
    @Logged
    open class CameraIOInputs {
        var connected: Boolean = false
        var poseObservations: Array<VisionData> = emptyArray()
        var txTyObservations: Array<TxTyData> = emptyArray()
    }

    fun updateInputs(inputs: CameraIOInputs) {}

    @JvmRecord
    data class VisionData(
        val cameraPose: Pose3d,
        val robotPose: Pose3d,
        val tagList: AprilTagList,
        val timestamp: Double,
    )

    @JvmRecord
    data class TxTyData(val tagId: Int, val tx: Double, val ty: Double, val distance: Double, val timestamp: Double)
}
