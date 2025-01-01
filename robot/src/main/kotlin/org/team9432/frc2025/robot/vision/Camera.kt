package org.team9432.frc2025.robot.vision

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.jvm.optionals.getOrNull
import kotlin.math.pow
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.Localizer
import org.team9432.frc2025.robot.vision.VisionConstants.ANGULAR_STDDEV_BASELINE
import org.team9432.frc2025.robot.vision.VisionConstants.LINEAR_STDDEV_BASELINE
import org.team9432.frc2025.robot.vision.VisionConstants.aprilTagLayout

class Camera(
    private val io: CameraIO,
    private val constants: VisionConstants.CameraConstants,
    private val localizer: Localizer,
) : SubsystemBase() {
    private val inputs = LoggedCameraIOInputs()
    private val alert = Alert("${constants.logName} is disconnected!", Alert.AlertType.kError)

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Cameras/${constants.logName}", inputs)
        alert.set(!inputs.connected)

        // Initialize logging values
        val tagPoses = mutableSetOf<Pose3d>()
        val allRobotPoses = mutableSetOf<Pose3d>()
        val acceptedPoses = mutableSetOf<Pose3d>()
        val rejectedPoses = mutableSetOf<Pose3d>()

        // Loop over pose observations
        for (observation in inputs.poseObservations) {
            // Add tag poses
            tagPoses.addAll(
                observation.trackedTags.tags.mapNotNull { tagId -> aprilTagLayout.getTagPose(tagId).getOrNull() }
            )

            // Check whether to reject pose
            val faults = observation.getFaults()

            // For now, reject the pose if any faults are detected. In the future we may just
            // want to increase stddevs for some of them.
            val rejectPose = faults.anyFaults

            // Add poses to logging lists
            allRobotPoses.add(observation.pose)
            if (rejectPose) {
                rejectedPoses.add(observation.pose)
            } else {
                acceptedPoses.add(observation.pose)
            }

            // Stop if rejected
            if (rejectPose) {
                continue
            }

            // Calculate standard deviations
            val stdDevFactor = observation.avgDistance.pow(2.0) / observation.trackedTags.tags.size
            var linearStdDev = LINEAR_STDDEV_BASELINE * stdDevFactor
            var angularStdDev = ANGULAR_STDDEV_BASELINE * stdDevFactor

            linearStdDev *= constants.stddevFactor
            angularStdDev *= constants.stddevFactor

            //             Send vision observation
            localizer.applyVisionMeasurement(
                observation.pose.toPose2d(),
                observation.timestamp,
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev),
            )
        }

        // Log camera data
        Logger.recordOutput("Cameras/${constants.logName}/TagPoses", *tagPoses.toTypedArray())
        Logger.recordOutput("Cameras/${constants.logName}/AllPoses", *allRobotPoses.toTypedArray())
        Logger.recordOutput("Cameras/${constants.logName}/AcceptedPoses", *acceptedPoses.toTypedArray())
        Logger.recordOutput("Cameras/${constants.logName}/RejectedPoses", *rejectedPoses.toTypedArray())
    }
}
