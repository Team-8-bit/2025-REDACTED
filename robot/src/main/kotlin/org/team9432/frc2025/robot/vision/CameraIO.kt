package org.team9432.frc2025.robot.vision

import org.team9432.annotation.Logged
import org.team9432.frc2025.robot.Localizer

interface CameraIO {
    @Logged
    open class CameraIOInputs {
        var connected: Boolean = false
        var poseObservations: Array<Localizer.VisionObservation> = emptyArray()
        var txTyObservations: Array<Localizer.TxTyObservation> = emptyArray()
    }

    fun updateInputs(inputs: CameraIOInputs) {}
}
