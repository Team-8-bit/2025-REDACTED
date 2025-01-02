package org.team9432.frc2025.robot.vision

import org.team9432.annotation.Logged

interface CameraIO {
    @Logged
    open class CameraIOInputs {
        var connected: Boolean = false
        var poseObservations: Array<VisionPoseEstimate> = emptyArray()
    }

    fun updateInputs(inputs: CameraIOInputs) {}
}
