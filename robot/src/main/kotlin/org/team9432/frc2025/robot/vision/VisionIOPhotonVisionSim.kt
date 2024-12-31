package org.team9432.frc2025.robot.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform3d
import java.util.function.Supplier
import org.photonvision.PhotonCamera
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import org.team9432.frc2025.robot.vision.VisionIO.VisionIOInputs

/** IO implementation for physics sim using PhotonVision simulator. */
class VisionIOPhotonVisionSim(
    name: String,
    robotToCamera: Transform3d,
    private val actualRobotPoseSupplier: Supplier<Pose2d>,
) : VisionIO {
    private val camera = PhotonCamera(name)
    private val cameraSim: PhotonCameraSim

    init {
        // Add sim camera
        val cameraProperties = SimCameraProperties()
        cameraSim = PhotonCameraSim(camera, cameraProperties)
        visionSim.addCamera(cameraSim, robotToCamera)
    }

    override fun updateInputs(inputs: VisionIOInputs) {
        visionSim.update(actualRobotPoseSupplier.get())
        super.updateInputs(inputs)
    }

    companion object {
        private var visionSim = VisionSystemSim("main").apply { addAprilTags(VisionConstants.aprilTagLayout) }
    }
}
