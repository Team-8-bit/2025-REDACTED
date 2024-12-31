package org.team9432.frc2025.robot.vision

import edu.wpi.first.math.geometry.Pose2d
import java.util.function.Supplier
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import org.team9432.frc2025.robot.vision.VisionIO.VisionIOInputs

// The goal here is to just hijack the real vision IO implementation and simulate the camera
class VisionIOPhotonVisionSim(cameraConfig: CameraConfig, private val actualRobotPoseSupplier: Supplier<Pose2d>) :
    VisionIOPhotonVision(cameraConfig) {
    // Add a camera sim to the real camera
    private val cameraSim = PhotonCameraSim(super.camera, SimCameraProperties())

    init {
        visionSim.addCamera(cameraSim, config.robotToCamera)
    }

    override fun updateInputs(inputs: VisionIOInputs) {
        // Update vision sim
        visionSim.update(actualRobotPoseSupplier.get())
        // Update the remaining inputs with the real IO implementation to ensure the same process is
        // used
        super.updateInputs(inputs)
    }

    companion object {
        private val visionSim = VisionSystemSim("main").apply { addAprilTags(VisionConstants.aprilTagLayout) }
    }
}
