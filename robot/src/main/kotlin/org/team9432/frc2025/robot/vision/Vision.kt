// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot.subsystems.vision

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.Localizer
import org.team9432.frc2025.robot.vision.LoggedVisionIOInputs
import org.team9432.frc2025.robot.vision.VisionConstants.angularStdDevBaseline
import org.team9432.frc2025.robot.vision.VisionConstants.aprilTagLayout
import org.team9432.frc2025.robot.vision.VisionConstants.cameraStdDevFactors
import org.team9432.frc2025.robot.vision.VisionConstants.linearStdDevBaseline
import org.team9432.frc2025.robot.vision.VisionConstants.maxAmbiguity
import org.team9432.frc2025.robot.vision.VisionConstants.maxZError
import org.team9432.frc2025.robot.vision.VisionIO
import java.util.*
import kotlin.math.abs
import kotlin.math.pow

class Vision(private val localizer: Localizer, private vararg val io: VisionIO): SubsystemBase() {
    private val cameras = Array(io.size) { index -> Camera(io[index], index) }

    data class Camera(
        val io: VisionIO,
        val id: Int,
        val inputs: LoggedVisionIOInputs = LoggedVisionIOInputs(),
        val disconnectedAlert: Alert = Alert("Vision camera $id is disconnected.", AlertType.kWarning),
    ) {
        fun updateDisconnectedAlert() = disconnectedAlert.set(!inputs.connected)
    }

    override fun periodic() {
        for (camera in cameras) {
            camera.io.updateInputs(camera.inputs)
            Logger.processInputs("Vision/Camera${camera.id}", camera.inputs)
        }

        // Initialize logging values
        val allTagPoses: MutableList<Pose3d> = LinkedList()
        val allRobotPoses: MutableList<Pose3d> = LinkedList()
        val allRobotPosesAccepted: MutableList<Pose3d> = LinkedList()
        val allRobotPosesRejected: MutableList<Pose3d> = LinkedList()

        // Loop over cameras
        for (camera in cameras) {
            // Update disconnected alert
            camera.updateDisconnectedAlert()

            // Initialize logging values
            val tagPoses: MutableList<Pose3d> = LinkedList()
            val robotPoses: MutableList<Pose3d> = LinkedList()
            val robotPosesAccepted: MutableList<Pose3d> = LinkedList()
            val robotPosesRejected: MutableList<Pose3d> = LinkedList()

            // Add tag poses
            for (tagId in inputs[cameraIndex].tagIds) {
                val tagPose = aprilTagLayout.getTagPose(tagId)
                if (tagPose.isPresent) {
                    tagPoses.add(tagPose.get())
                }
            }

            // Loop over pose observations
            for (observation in inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                val rejectPose = shouldReject(observation)

                // Add pose to log
                robotPoses.add(observation.pose)
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose)
                } else {
                    robotPosesAccepted.add(observation.pose)
                }

                // Skip if rejected
                if (rejectPose) {
                    continue
                }

                // Calculate standard deviations
                val stdDevFactor: Double = observation.averageTagDistance.pow(2.0) / observation.tagCount
                var linearStdDev: Double = linearStdDevBaseline * stdDevFactor
                var angularStdDev: Double = angularStdDevBaseline * stdDevFactor

                if (cameraIndex < cameraStdDevFactors.size) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex]
                    angularStdDev *= cameraStdDevFactors[cameraIndex]
                }

                // Send vision observation
                localizer.applyVisionMeasurement(
                    observation.pose.toPose2d(),
                    observation.timestamp,
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev),
                )
            }

            // Log camera datadata
            Logger.recordOutput("Vision/Camera$cameraIndex/TagPoses", *tagPoses.toTypedArray<Pose3d>())
            Logger.recordOutput("Vision/Camera$cameraIndex/RobotPoses", *robotPoses.toTypedArray<Pose3d>())
            Logger.recordOutput(
                "Vision/Camera$cameraIndex/RobotPosesAccepted",
                *robotPosesAccepted.toTypedArray<Pose3d>(),
            )
            Logger.recordOutput(
                "Vision/Camera$cameraIndex/RobotPosesRejected",
                *robotPosesRejected.toTypedArray<Pose3d>(),
            )
            allTagPoses.addAll(tagPoses)
            allRobotPoses.addAll(robotPoses)
            allRobotPosesAccepted.addAll(robotPosesAccepted)
            allRobotPosesRejected.addAll(robotPosesRejected)
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", *allTagPoses.toTypedArray<Pose3d>())
        Logger.recordOutput("Vision/Summary/RobotPoses", *allRobotPoses.toTypedArray<Pose3d>())
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", *allRobotPosesAccepted.toTypedArray<Pose3d>())
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", *allRobotPosesRejected.toTypedArray<Pose3d>())
    }

    /** Choose whether to reject this observation. */
    private fun shouldReject(observation: VisionIO.PoseObservation): Boolean {
        // Must have at least one tag
        val inadequateTagCount = observation.tagCount == 0
        // Cannot be a single tag with high ambiguity
        val tooHighSingleTagAmbiguity = observation.tagCount == 1 && observation.ambiguity > maxAmbiguity
        // Can't be flying or in the floor
        val unrealisticZPosition = abs(observation.pose.z) > maxZError
        // Has to be on the field
        val outOfField =
            observation.pose.x < 0.0 ||
                    observation.pose.x > aprilTagLayout.fieldLength ||
                    observation.pose.y < 0.0 ||
                    observation.pose.y > aprilTagLayout.fieldWidth

        return inadequateTagCount || tooHighSingleTagAmbiguity || unrealisticZPosition || outOfField
    }
}
