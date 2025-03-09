package org.team9432.frc2025.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.math.abs
import kotlin.math.atan2
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.util.distanceTo
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants

class RobotPosition(private val localizer: Localizer) {
    fun waitUntilRelativeMovement(passing: (Double, Double, Rotation2d) -> Boolean): Command =
        Commands.defer(
            {
                val initialPose = localizer.estimatedPose
                Commands.waitUntil {
                    localizer.estimatedPose.relativeTo(initialPose).let { passing.invoke(it.x, it.y, it.rotation) }
                }
            },
            emptySet(),
        )

    private var lastScorePosition = Pose2d()

    fun resetLastScorePoseToCurrent() {
        lastScorePosition = localizer.estimatedPose
    }

    val isSafeToUseArm = Trigger {
        val estimatedPose = localizer.estimatedPose
        val angleToPointAtReef =
            Math.toDegrees(
                atan2(FieldConstants.Reef.center.y - estimatedPose.y, FieldConstants.Reef.center.x - estimatedPose.x)
            )

        val distanceGood =
            estimatedPose.distanceTo(FieldConstants.Reef.center) >
                FieldConstants.Reef.maxRadius + (DrivetrainConstants.BUMPER_LENGTH / 2) + Units.inchesToMeters(12.0)
        val rotationGood = abs(angleToPointAtReef - estimatedPose.rotation.degrees) > 90
        distanceGood || rotationGood
    }

    fun outputTelemetry() {
        Logger.recordOutput("RobotPosition/isSafeToUseArm", isSafeToUseArm)
        Logger.recordOutput("RobotPosition/LastScorePosition", lastScorePosition)

        Logger.recordOutput("RobotPosition/ReefTargetBranch", nearestReefAlignBranch())
    }

    private val reefAlignTransform = Transform2d(DrivetrainConstants.BUMPER_LENGTH / 2, 0.0, Rotation2d.k180deg)

    fun getCurrentReefTargetPose(robotPose: Pose2d = localizer.estimatedPose): Pose2d {
        val branch = nearestReefAlignBranch(robotPose)
        val alignPose = branch.getPose().transformBy(reefAlignTransform)
        val txTyRobotPose = localizer.getTxTyPose(branch.getTag()) ?: robotPose
        val distanceToTarget = abs(txTyRobotPose.relativeTo(alignPose).y)
        return alignPose.transformBy(Transform2d(-distanceToTarget, 0.0, Rotation2d.kZero))
    }

    fun nearestReefAlignBranch(robotPose: Pose2d = localizer.estimatedPose): FieldConstants.Reef.Branch {
        return FieldConstants.Reef.Branch.nearestTo(robotPose)
    }
}
