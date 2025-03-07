package org.team9432.frc2025.robot.vision

import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.Localizer

class Camera(
    private val io: CameraIO,
    private val constants: VisionConstants.CameraConstants,
    private val localizer: Localizer,
) : SubsystemBase() {
    private val inputs = LoggedCameraIOInputs()
    private val alert = Alert("${constants.cameraName} is disconnected!", Alert.AlertType.kError)

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Vision/${constants.cameraName}", inputs)
        alert.set(!inputs.connected)

        // Send observations
        for (observation in inputs.poseObservations) {
            localizer.addVisionObservation(observation)
        }

        for (observation in inputs.txTyObservations) {
            localizer.addTxTyObservation(observation)
        }
    }
}
