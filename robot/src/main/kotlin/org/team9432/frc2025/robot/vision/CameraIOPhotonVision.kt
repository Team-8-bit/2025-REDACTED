package org.team9432.frc2025.robot.vision

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import java.util.*
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.team9432.frc2025.robot.FieldConstants
import org.team9432.frc2025.robot.Localizer
import org.team9432.frc2025.robot.vision.CameraIO.CameraIOInputs

open class CameraIOPhotonVision(
    val config: VisionConstants.PhotonConfig,
    private val rotationSupplier: () -> Rotation2d,
) : CameraIO {
    val camera = PhotonCamera(config.photonName)

    private val poseEstimator =
        PhotonPoseEstimator(
                VisionConstants.aprilTagLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                config.robotToCamera,
            )
            .apply { setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY) }

    override fun updateInputs(inputs: CameraIOInputs) {
        inputs.connected = camera.isConnected

        // Read new camera observations
        val visionObservations = mutableListOf<Localizer.VisionObservation>()
        val txtyObservations = mutableListOf<Localizer.TxTyObservation>()

        val ambiguityThreshold = 0.4

        val results = camera.allUnreadResults
        for (result in results) {
            val multitagResult = result.multitagResult.getOrNull()

            var pose: Pose3d? = null

            when {
                multitagResult != null -> {
                    val bestTransform = multitagResult.estimatedPose.best
                    pose = transformToRobotPose(bestTransform) // field-to-robot
                }

                result.targets.size > 1 -> {
                    throw Exception("I don't think this should happen") // TODO: Remove before comp
                }

                result.targets.size == 1 -> {
                    val target = result.targets.first()
                    if (target.poseAmbiguity < ambiguityThreshold) {
                        val currentRotation = rotationSupplier()
                        val bestRotation = target.bestCameraToTarget.rotation.toRotation2d()
                        val altRotation = target.alternateCameraToTarget.rotation.toRotation2d()
                        if (
                            abs(currentRotation.minus(bestRotation).radians) <
                                abs(currentRotation.minus(altRotation).radians)
                        ) {
                            pose = transformToRobotPose(target.bestCameraToTarget)
                        } else {
                            pose = transformToRobotPose(target.alternateCameraToTarget)
                        }
                    }
                }
            }

            if (pose == null) {
                continue
            }

            val fieldBorderMargin = 0.5
            if (
                pose.x < -fieldBorderMargin ||
                    pose.x > FieldConstants.fieldLength + fieldBorderMargin ||
                    pose.y < -fieldBorderMargin ||
                    pose.y > FieldConstants.fieldWidth + fieldBorderMargin
            ) {
                continue
            }

            visionObservations.add(
                Localizer.VisionObservation(
                    pose.toPose2d(),
                    result.timestampSeconds,
                    VecBuilder.fill(0.1, 0.1, 1.0), // TODO
                )
            )
        }

        // Save pose observations to inputs object
        inputs.poseObservations = visionObservations.toTypedArray()
    }

    private fun transformToRobotPose(bestTransform: Transform3d?): Pose3d =
        Pose3d()
            .plus(bestTransform) // field-to-camera
            .relativeTo(VisionConstants.aprilTagLayout.origin)
            .plus(config.robotToCamera.inverse())
}
