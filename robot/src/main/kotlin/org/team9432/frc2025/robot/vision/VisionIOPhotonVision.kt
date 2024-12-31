package org.team9432.frc2025.robot.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import kotlin.jvm.optionals.getOrNull
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult
import org.team9432.frc2025.robot.vision.VisionConstants.aprilTagLayout
import org.team9432.frc2025.robot.vision.VisionIO.*

open class VisionIOPhotonVision(final override val config: CameraConfig) : VisionIO {
    val camera = PhotonCamera(config.cameraName)

    override fun updateInputs(inputs: VisionIOInputs) {
        inputs.connected = camera.isConnected

        // Read new camera observations
        val tagIds = mutableSetOf<Short>()
        val poseObservations = mutableListOf<PoseObservation>()

        for (result in camera.allUnreadResults) {
            // Add pose observation
            if (result.multitagResult.isPresent) { // Multitag result
                val (observation, tags) = getMultitagResult(result)
                poseObservations.add(observation)
                tagIds.addAll(tags)
            } else if (result.targets.isNotEmpty()) { // Single tag result
                val singleTagResult = getSingleTagResult(result)
                if (singleTagResult != null) {
                    val (observation, tag) = singleTagResult
                    poseObservations.add(observation)
                    tagIds.add(tag)
                }
            }
        }

        // Save pose observations to inputs object
        inputs.poseObservations = poseObservations.toTypedArray()

        // Save tag IDs to inputs objects
        inputs.tagIds = tagIds.map { it.toInt() }.toIntArray()
    }

    private fun getMultitagResult(result: PhotonPipelineResult): Pair<PoseObservation, List<Short>> {
        val multitagResult = result.multitagResult.get()

        // Calculate robot pose
        val fieldToCamera = multitagResult.estimatedPose.best
        val fieldToRobot = fieldToCamera.plus(config.robotToCamera.inverse())
        val robotPose = Pose3d(fieldToRobot.translation, fieldToRobot.rotation)

        // Calculate average tag distance
        var totalTagDistance = 0.0
        for (target in result.targets) {
            totalTagDistance += target.bestCameraToTarget.translation.norm
        }

        val tagIds = multitagResult.fiducialIDsUsed

        val observation =
            PoseObservation(
                result.timestampSeconds, // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size, // Tag count
                totalTagDistance / result.targets.size, // Average tag distance
            )

        return observation to tagIds
    }

    private fun getSingleTagResult(result: PhotonPipelineResult): Pair<PoseObservation, Short>? {
        val target = result.targets[0]

        // Calculate robot pose
        val tagPose = aprilTagLayout.getTagPose(target.fiducialId).getOrNull() ?: return null

        val fieldToTarget = Transform3d(tagPose.translation, tagPose.rotation)
        val cameraToTarget = target.bestCameraToTarget
        val fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse())
        val fieldToRobot = fieldToCamera.plus(config.robotToCamera.inverse())
        val robotPose = Pose3d(fieldToRobot.translation, fieldToRobot.rotation)

        // Add tag ID
        val tagId = target.fiducialId.toShort()

        // Add observation
        val observation =
            PoseObservation(
                result.timestampSeconds, // Timestamp
                robotPose, // 3D pose estimate
                target.poseAmbiguity, // Ambiguity
                1, // Tag count
                cameraToTarget.translation.norm, // Average tag distance
            )

        return observation to tagId
    }
}
