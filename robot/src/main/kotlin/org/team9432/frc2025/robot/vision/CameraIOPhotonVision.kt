package org.team9432.frc2025.robot.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult
import org.team9432.frc2025.robot.vision.CameraIO.CameraIOInputs
import org.team9432.frc2025.robot.vision.VisionConstants.aprilTagLayout

open class CameraIOPhotonVision(name: String, private val rotationSupplier: () -> Rotation2d) : CameraIO {
    val camera = PhotonCamera(name)

    override fun updateInputs(inputs: CameraIOInputs) {
        inputs.connected = camera.isConnected

        // Read new camera observations
        val results = camera.allUnreadResults

        val visionObservations = mutableListOf<CameraIO.VisionData>()
        val txtyObservations = mutableListOf<CameraIO.TxTyData>()
        for (result in results) {
            val cameraPose = estimateCameraPose(result)
            if (cameraPose != null) {
                visionObservations.add(
                    CameraIO.VisionData(
                        cameraPose = cameraPose,
                        tagList = AprilTagList(result.targets.map { it.fiducialId }),
                        timestamp = result.timestampSeconds,
                    )
                )
            }

            for (target in result.targets) {
                txtyObservations.add(
                    CameraIO.TxTyData(
                        tagId = target.fiducialId,
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
    private fun estimateCameraPose(result: PhotonPipelineResult): Pose3d? {
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

                result.targets.size == 0 -> {
                    return null
                }

                else -> {
                    println(result.targets.size)
                    throw Exception("I don't think this should happen") // TODO: Replace with continue before comp
                }
            }

        // Make sure we got a valid camera transform
        if (cameraToTarget == null) return null

        // Calculate camera pose
        val cameraPose = Pose3d().plus(cameraToTarget).relativeTo(aprilTagLayout.origin)

        println(cameraPose)

        return cameraPose
    }
}
