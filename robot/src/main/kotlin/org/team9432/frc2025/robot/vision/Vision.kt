package org.team9432.frc2025.robot.vision

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.*
import kotlin.math.abs
import kotlin.math.pow
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.Localizer
import org.team9432.frc2025.robot.vision.VisionConstants.ANGULAR_STDDEV_BASELINE
import org.team9432.frc2025.robot.vision.VisionConstants.LINEAR_STDDEV_BASELINE
import org.team9432.frc2025.robot.vision.VisionConstants.MAX_AMBIGUITY
import org.team9432.frc2025.robot.vision.VisionConstants.MAX_Z_ERROR
import org.team9432.frc2025.robot.vision.VisionConstants.aprilTagLayout

class Vision(private val localizer: Localizer, vararg cameras: VisionIO) : SubsystemBase() {
    data class CameraContainer(val io: VisionIO) {
        val inputs = LoggedVisionIOInputs()

        fun updateInputs() {
            io.updateInputs(inputs)
            Logger.processInputs("Vision/${io.config.cameraName}", inputs)
            alert.set(!inputs.connected)
        }

        private val alert = Alert("${io.config.cameraName} is disconnected!", Alert.AlertType.kWarning)
    }

    private val cameras = cameras.map { CameraContainer(it) }

    override fun periodic() {
        for (camera in cameras) {
            camera.updateInputs()
        }

        // Initialize logging values
        val allTagPoses = mutableListOf<Pose3d>()
        val allPoses = mutableListOf<Pose3d>()
        val allAccepted = mutableListOf<Pose3d>()
        val allRejected = mutableListOf<Pose3d>()

        // Loop over cameras
        for (camera in cameras) {
            // Initialize logging values
            val tagPoses = mutableListOf<Pose3d>()
            val allRobotPoses = mutableListOf<Pose3d>()
            val acceptedPoses = mutableListOf<Pose3d>()
            val rejectedPoses = mutableListOf<Pose3d>()

            // Add tag poses
            for (tagId in camera.inputs.tagIds) {
                val tagPose = aprilTagLayout.getTagPose(tagId)
                if (tagPose.isPresent) {
                    tagPoses.add(tagPose.get())
                }
            }

            // Loop over pose observations
            for (observation in camera.inputs.poseObservations) {
                // Check whether to reject pose
                val rejectPose = shouldReject(observation)

                // Add pose to log
                allRobotPoses.add(observation.pose)
                if (rejectPose) {
                    rejectedPoses.add(observation.pose)
                } else {
                    acceptedPoses.add(observation.pose)
                }

                // Skip if rejected
                if (rejectPose) {
                    continue
                }

                // Calculate standard deviations
                val stdDevFactor: Double = observation.averageTagDistance.pow(2.0) / observation.tagCount
                var linearStdDev: Double = LINEAR_STDDEV_BASELINE * stdDevFactor
                var angularStdDev: Double = ANGULAR_STDDEV_BASELINE * stdDevFactor

                linearStdDev *= camera.io.config.stddevFactor
                angularStdDev *= camera.io.config.stddevFactor

                // Send vision observation
                localizer.applyVisionMeasurement(
                    observation.pose.toPose2d(),
                    observation.timestamp,
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev),
                )
            }

            // Log camera data
            Logger.recordOutput("Vision/${camera.io.config.cameraName}/TagPoses", *tagPoses.toTypedArray())
            Logger.recordOutput("Vision/${camera.io.config.cameraName}/AllPoses", *allRobotPoses.toTypedArray())
            Logger.recordOutput("Vision/${camera.io.config.cameraName}/AcceptedPoses", *acceptedPoses.toTypedArray())
            Logger.recordOutput("Vision/${camera.io.config.cameraName}/RejectedPoses", *rejectedPoses.toTypedArray())

            allTagPoses.addAll(tagPoses)
            allRobotPoses.addAll(allRobotPoses)
            allAccepted.addAll(acceptedPoses)
            allRejected.addAll(rejectedPoses)
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", *allTagPoses.toTypedArray())
        Logger.recordOutput("Vision/Summary/AllPoses", *allPoses.toTypedArray())
        Logger.recordOutput("Vision/Summary/AcceptedPoses", *allAccepted.toTypedArray())
        Logger.recordOutput("Vision/Summary/RejectedPoses", *allRejected.toTypedArray())
    }

    /** Choose whether to reject this observation. */
    private fun shouldReject(observation: VisionIO.PoseObservation): Boolean {
        // Must have at least one tag
        val inadequateTagCount = observation.tagCount == 0
        // Cannot be a single tag with high ambiguity
        val tooHighSingleTagAmbiguity = observation.tagCount == 1 && observation.ambiguity > MAX_AMBIGUITY
        // Can't be flying or in the floor
        val unrealisticZPosition = abs(observation.pose.z) > MAX_Z_ERROR
        // Has to be on the field
        val outOfField =
            observation.pose.x < 0.0 ||
                observation.pose.x > aprilTagLayout.fieldLength ||
                observation.pose.y < 0.0 ||
                observation.pose.y > aprilTagLayout.fieldWidth

        return inadequateTagCount || tooHighSingleTagAmbiguity || unrealisticZPosition || outOfField
    }
}
