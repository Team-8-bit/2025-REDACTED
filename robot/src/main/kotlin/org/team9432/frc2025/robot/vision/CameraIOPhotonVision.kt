package org.team9432.frc2025.robot.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult
import org.team9432.frc2025.robot.vision.CameraIO.CameraIOInputs

open class CameraIOPhotonVision(
    private val config: VisionConstants.CameraConstants,
    private val rotationSupplier: () -> Rotation2d,
) : CameraIO {
    val camera = PhotonCamera(config.cameraName)

    override fun updateInputs(inputs: CameraIOInputs) {
        inputs.connected = camera.isConnected

        // Read new camera observations
        val results = camera.allUnreadResults

        val visionObservations = mutableListOf<CameraIO.VisionData>()
        val txtyObservations = mutableListOf<CameraIO.TxTyData>()
        for (result in results) {
            val (cameraPose, robotPose) = estimateCameraPose(result)
            if (cameraPose != null && robotPose != null) {
                visionObservations.add(
                    CameraIO.VisionData(
                        cameraPose = cameraPose,
                        robotPose = robotPose,
                        tagList = AprilTagList(result.targets.map { it.fiducialId }),
                        timestamp = result.timestampSeconds,
                    )
                )
            }

            for (target in result.targets) {
                txtyObservations.add(
                    CameraIO.TxTyData(
                        tagId = target.fiducialId,
                        tx = Units.degreesToRadians(target.yaw),
                        ty = Units.degreesToRadians(target.pitch) - config.robotToCamera.rotation.y,
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
    private fun estimateCameraPose(result: PhotonPipelineResult): Pair<Pose3d?, Pose3d?> {
        // First thing is to find the where the camera is
        val multitagResult = result.multitagResult.getOrNull()
        val cameraPose: Pose3d?
        val robotPose: Pose3d?
        when {
            // If we got a multitag result, use that
            multitagResult != null -> {
                cameraPose =
                    Pose3d().plus(multitagResult.estimatedPose.best).relativeTo(VisionConstants.aprilTagLayout.origin)
                robotPose = cameraPose.plus(config.robotToCamera.inverse())
            }

            // Else take single tag and try to disambiguate
            result.targets.size == 1 -> {
                val target = result.targets.first()

                // If the pose is too ambiguous, return null
                if (target.poseAmbiguity >= VisionConstants.MAX_AMBIGUITY) {
                    return null to null
                }

                val tagPose =
                    VisionConstants.aprilTagLayout.getTagPose(target.fiducialId).getOrNull() ?: return null to null

                // Else disambiguate by which is closest to the robot's rotation
                val currentRotation = rotationSupplier()
                val bestCameraPose = tagPose.transformBy(target.bestCameraToTarget.inverse())
                val bestRobotPose = bestCameraPose.plus(config.robotToCamera.inverse())

                val altCameraPose = tagPose.transformBy(target.alternateCameraToTarget.inverse())
                val altRobotPose = altCameraPose.plus(config.robotToCamera.inverse())

                if (
                    abs(currentRotation.minus(bestRobotPose.rotation.toRotation2d()).radians) <
                        abs(currentRotation.minus(altRobotPose.rotation.toRotation2d()).radians)
                ) {
                    robotPose = bestRobotPose
                    cameraPose = bestCameraPose
                } else {
                    // altPose
                    robotPose = bestRobotPose
                    cameraPose = bestCameraPose
                }
            }

            result.targets.size == 0 -> {
                return null to null
            }

            else -> {
                println(result.targets.size)
                throw Exception("I don't think this should happen") // TODO: Replace with continue before comp
            }
        }

        return cameraPose to robotPose
    }
}
