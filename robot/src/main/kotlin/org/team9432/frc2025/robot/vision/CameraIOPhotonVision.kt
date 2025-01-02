package org.team9432.frc2025.robot.vision

import kotlin.jvm.optionals.getOrNull
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.team9432.frc2025.lib.util.xyNorm
import org.team9432.frc2025.robot.vision.CameraIO.CameraIOInputs

open class CameraIOPhotonVision(val config: VisionConstants.PhotonConfig) : CameraIO {
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
        val poseObservations = mutableListOf<VisionPoseEstimate>()

        val results = camera.allUnreadResults
        for (result in results) {
            val photonEstimatedPose = poseEstimator.update(result).getOrNull() ?: continue
            val tagDistances = photonEstimatedPose.targetsUsed.map { it.bestCameraToTarget.translation.xyNorm() }

            val poseEstimate =
                VisionPoseEstimate(
                    pose = photonEstimatedPose.estimatedPose,
                    timestamp = photonEstimatedPose.timestampSeconds,
                    trackedTags = AprilTagList(result.targets.map { it.fiducialId }),
                    ambiguity = photonEstimatedPose.targetsUsed.map { it.poseAmbiguity }.average(),
                    maxDistance = tagDistances.max(),
                    avgDistance = tagDistances.average(),
                )

            poseObservations.add(poseEstimate)
        }

        // Save pose observations to inputs object
        inputs.poseObservations = poseObservations.toTypedArray()
    }
}
