package org.team9432.frc2025.robot.vision

import edu.wpi.first.math.geometry.Rotation2d
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim

// The goal here is to just hijack the real vision IO implementation and simulate the camera
class CameraIOPhotonVisionSim(cameraConfig: VisionConstants.PhotonConfig, private val visionSim: VisionSystemSim) :
    CameraIOPhotonVision(cameraConfig) {
    private val simCameraProperties =
        SimCameraProperties().apply {
            setCalibration(1280, 800, Rotation2d.fromDegrees(78.61))
            setCalibError(0.2, 0.04)
            fps = 25.0
            avgLatencyMs = 25.0
            latencyStdDevMs = 8.0
        }

    // Add a camera sim to the real camera
    private val cameraSim =
        PhotonCameraSim(super.camera, simCameraProperties).apply {
            enableRawStream(true)
            enableProcessedStream(true)
            enableDrawWireframe(true)
        }

    init {
        visionSim.addCamera(cameraSim, config.robotToCamera)
    }
}
