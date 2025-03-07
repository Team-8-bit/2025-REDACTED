package org.team9432.frc2025.robot.vision

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs
import kotlin.math.pow
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult
import org.team9432.frc2025.robot.FieldConstants
import org.team9432.frc2025.robot.Localizer
import org.team9432.frc2025.robot.vision.CameraIO.CameraIOInputs
import org.team9432.frc2025.robot.vision.VisionConstants.FIELD_BORDER_MARGIN
import org.team9432.frc2025.robot.vision.VisionConstants.aprilTagLayout

open class CameraIOPhotonVision(
    val cameraConstants: VisionConstants.CameraConstants,
    private val rotationSupplier: () -> Rotation2d,
) : CameraIO {
    val camera = PhotonCamera(cameraConstants.cameraName)

    override fun updateInputs(inputs: CameraIOInputs) {
        inputs.connected = camera.isConnected

        // Read new camera observations
        val results = camera.allUnreadResults

        val visionObservations = mutableListOf<Localizer.VisionObservation>()
        val txtyObservations = mutableListOf<Localizer.TxTyObservation>()
        for (result in results) {
            val poseEstimate = estimatePose(result)
            if (poseEstimate != null) {
                val (pose, stdDevs) = poseEstimate

                visionObservations.add(
                    Localizer.VisionObservation(
                        visionPose = pose.toPose2d(),
                        timestamp = result.timestampSeconds,
                        stdDevs = stdDevs,
                    )
                )
            }

            for (target in result.targets) {
                txtyObservations.add(
                    Localizer.TxTyObservation(
                        tagId = target.fiducialId,
                        camera = cameraConstants,
                        tx = target.yaw,
                        ty = target.pitch,
                        distance = target.bestCameraToTarget.translation.norm,
                        timestamp = result.timestampSeconds,
                    )
                )
            }
        }

        inputs.poseObservations = visionObservations.toTypedArray()
        inputs.txTyObservations = txtyObservations.toTypedArray()
    }

    /**
     * Estimates the robot's position based on the given [PhotonPipelineResult].
     *
     * @return An estimated pose and a set of standard deviations, or null if no valid pose was found.
     */
    private fun estimatePose(result: PhotonPipelineResult): Pair<Pose3d, Vector<N3>>? {
        // First thing is to find the where the camera is
        val multitagResult = result.multitagResult.getOrNull()
        val cameraToTarget: Transform3d? =
            when {
                // If we got a multitag result, use that
                multitagResult != null -> multitagResult.estimatedPose.best

                // Else take single tag and try to disambiguate
                result.targets.size == 1 -> {
                    val target = result.targets.first()

                    // If the pose is too ambiguous, return null
                    if (target.poseAmbiguity >= VisionConstants.MAX_AMBIGUITY) return null

                    // Else disambiguate by which is closest to the robot's rotation
                    val currentRotation = rotationSupplier()
                    val bestRotation = target.bestCameraToTarget.rotation.toRotation2d()
                    val altRotation = target.alternateCameraToTarget.rotation.toRotation2d()
                    if (
                        abs(currentRotation.minus(bestRotation).radians) <
                            abs(currentRotation.minus(altRotation).radians)
                    ) {
                        target.bestCameraToTarget
                    } else {
                        target.alternateCameraToTarget
                    }
                }

                else -> {
                    println(result.targets.size)
                    throw Exception("I don't think this should happen") // TODO: Replace with continue before comp
                }
            }

        // Make sure we got a valid camera transform
        if (cameraToTarget == null) return null

        // Calculate camera and robot poses
        val cameraPose = Pose3d().plus(cameraToTarget).relativeTo(aprilTagLayout.origin)
        val robotPose = cameraPose.plus(cameraConstants.robotToCamera.inverse())

        // If the robot pose is invalid, return null
        if (
            robotPose.x < -FIELD_BORDER_MARGIN ||
                robotPose.x > FieldConstants.fieldLength + FIELD_BORDER_MARGIN ||
                robotPose.y < -FIELD_BORDER_MARGIN ||
                robotPose.y > FieldConstants.fieldWidth + FIELD_BORDER_MARGIN ||
                abs(robotPose.translation.z) > VisionConstants.MAX_Z_ERROR ||
                abs(robotPose.rotation.y) > Units.degreesToRadians(VisionConstants.MAX_ANGLE_ERROR) ||
                abs(robotPose.rotation.x) > Units.degreesToRadians(VisionConstants.MAX_ANGLE_ERROR)
        )
            return null

        // Calculate standard deviations of the estimate
        val tagPoses = result.targets.mapNotNull { aprilTagLayout.getTagPose(it.fiducialId).getOrNull() }
        val averageTagDistance = tagPoses.map { it.translation.getDistance(cameraPose.translation) }.average()
        val calculatedStdDevFactor = averageTagDistance.pow(2.0) / result.targets.size

        // Base standard deviation
        var linearStdDev = VisionConstants.LINEAR_STDDEV_BASELINE
        var angularStdDev = VisionConstants.ANGULAR_STDDEV_BASELINE

        // Account for tag distance and count
        linearStdDev *= calculatedStdDevFactor
        angularStdDev *= calculatedStdDevFactor

        // Account for per-camera trust
        linearStdDev *= cameraConstants.stdDevFactor
        angularStdDev *= cameraConstants.stdDevFactor

        // Return results!
        return robotPose to VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
    }
}
