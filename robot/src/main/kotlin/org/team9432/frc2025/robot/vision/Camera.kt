package org.team9432.frc2025.robot.vision

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs
import kotlin.math.pow
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.FieldConstants
import org.team9432.frc2025.robot.Localizer
import org.team9432.frc2025.robot.vision.VisionConstants.FIELD_BORDER_MARGIN
import org.team9432.frc2025.robot.vision.VisionConstants.aprilTagLayout

class Camera(
    private val io: CameraIO,
    private val cameraConstants: VisionConstants.CameraConstants,
    private val localizer: Localizer,
) : SubsystemBase() {
    private val inputs = LoggedCameraIOInputs()
    private val alert = Alert("${cameraConstants.cameraName} is disconnected!", Alert.AlertType.kError)

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Vision/${cameraConstants.cameraName}", inputs)
        alert.set(!inputs.connected)

        for (observation in inputs.poseObservations) {
            val robotPose = observation.cameraPose.plus(cameraConstants.robotToCamera.inverse())
            val tagPoses = observation.tagList.tags.mapNotNull { aprilTagLayout.getTagPose(it).getOrNull() }

            // If the robot pose is invalid, continue
            if (
                robotPose.x < -FIELD_BORDER_MARGIN ||
                    robotPose.x > FieldConstants.fieldLength + FIELD_BORDER_MARGIN ||
                    robotPose.y < -FIELD_BORDER_MARGIN ||
                    robotPose.y > FieldConstants.fieldWidth + FIELD_BORDER_MARGIN ||
                    abs(robotPose.translation.z) > VisionConstants.MAX_Z_ERROR ||
                    abs(robotPose.rotation.y) > Units.degreesToRadians(VisionConstants.MAX_ANGLE_ERROR) ||
                    abs(robotPose.rotation.x) > Units.degreesToRadians(VisionConstants.MAX_ANGLE_ERROR)
            )
                continue

            // Calculate standard deviations of the estimate
            val averageTagDistance =
                tagPoses.map { it.translation.getDistance(observation.cameraPose.translation) }.average()
            val calculatedStdDevFactor = averageTagDistance.pow(2.0) / tagPoses.size

            // Base standard deviation
            var linearStdDev = VisionConstants.LINEAR_STDDEV_BASELINE
            var angularStdDev = VisionConstants.ANGULAR_STDDEV_BASELINE

            // Account for tag distance and count
            linearStdDev *= calculatedStdDevFactor
            angularStdDev *= calculatedStdDevFactor

            // Account for per-camera trust
            linearStdDev *= cameraConstants.stdDevFactor
            angularStdDev *= cameraConstants.stdDevFactor

            localizer.addVisionObservation(
                Localizer.VisionObservation(
                    visionPose = robotPose.toPose2d(),
                    timestamp = observation.timestamp,
                    stdDevs = VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev),
                )
            )
        }

        for (observation in inputs.txTyObservations) {
            localizer.addTxTyObservation(
                Localizer.TxTyObservation(
                    tagId = observation.tagId,
                    camera = cameraConstants,
                    tx = observation.tx,
                    ty = observation.ty,
                    distance = observation.distance,
                    timestamp = observation.timestamp,
                )
            )
        }
    }
}
