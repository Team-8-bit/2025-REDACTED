package org.team9432.frc2025.robot

import edu.wpi.first.math.MathUtil
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
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
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

    val isSafeToUseArm = Trigger {
        val estimatedPose = localizer.estimatedPose
        val distanceGood =
            estimatedPose.distanceTo(FieldConstants.Reef.center) >
                FieldConstants.Reef.maxRadius + (DrivetrainConstants.BUMPER_LENGTH / 2) + Units.inchesToMeters(12.0)
        val rotationGood = angleFromReef() > 90
        distanceGood || rotationGood
    }

    private fun angleFromReef(estimatedPose: Pose2d = localizer.estimatedPose): Double {
        val angleToPointAtReef =
            atan2(FieldConstants.Reef.center.y - estimatedPose.y, FieldConstants.Reef.center.x - estimatedPose.x)
        return abs(Math.toDegrees(MathUtil.angleModulus(angleToPointAtReef - estimatedPose.rotation.radians)))
    }

    fun outputTelemetry() {
        Logger.recordOutput("RobotPosition/isSafeToUseArm", isSafeToUseArm)
        Logger.recordOutput("RobotPosition/ReefTargetBranch", nearestReefAlignBranch())
        Logger.recordOutput("RobotPosition/AngleFromReef", angleFromReef())
        Logger.recordOutput("RobotPosition/BranchAlignPose", getActiveBranchAlignPose(nearestReefAlignBranch()))
    }

    private val reefAlignTransform = Transform2d(DrivetrainConstants.BUMPER_LENGTH / 2, 0.0, Rotation2d.k180deg)

    fun getActiveBranchAlignPose(branch: FieldConstants.Reef.Branch): Pose2d {
        val alignPose = branch.getPose().transformBy(reefAlignTransform)
        val txTyRobotPose = localizer.getReefPose(branch.getTag(), alignPose)

        val yDistance = abs(txTyRobotPose.relativeTo(alignPose).y)

        var xOffset = -yDistance
        if (angleFromReef(txTyRobotPose) > 40) {
            xOffset -= 0.5
        }
        return alignPose.transformBy(Transform2d(xOffset, 0.0, Rotation2d.kZero))
    }

    fun getBaseBranchAlignPose(branch: FieldConstants.Reef.Branch): Pose2d {
        return branch.getPose().transformBy(reefAlignTransform)
    }

    fun nearestReefAlignBranch(robotPose: Pose2d = localizer.estimatedPose): FieldConstants.Reef.Branch {
        val map =
            FieldConstants.Reef.Branch.entries.associateWith { branch ->
                getBaseBranchAlignPose(branch).let { branchPose ->
                    robotPose.distanceTo(branchPose) to abs(robotPose.rotation.degrees - branchPose.rotation.degrees)
                }
            }
        val target =
            map.minBy {
                val (distanceMeters, distanceDegrees) = it.value
                distanceMeters
            }
        return target.key
    }

    private val algaeAlignTransform =
        Transform2d(DrivetrainConstants.BUMPER_LENGTH / 2 + Units.inchesToMeters(2.0), 0.0, Rotation2d.k180deg)

    fun getActiveAlgaeAlignPose(stagedAlgae: FieldConstants.Reef.StagedAlgae): Pose2d {
        val alignPose = stagedAlgae.getPose().transformBy(algaeAlignTransform)
        val txTyRobotPose = localizer.getReefPose(stagedAlgae.getTag(), alignPose)

        val yDistance = abs(txTyRobotPose.relativeTo(alignPose).y)

        var xOffset = -(yDistance * 1.5)
        if (angleFromReef(txTyRobotPose) > 40) {
            xOffset -= 0.5
        }
        return alignPose.transformBy(Transform2d(xOffset, 0.0, Rotation2d.kZero))
    }

    fun getBaseAlgaeAlignPose(branch: FieldConstants.Reef.StagedAlgae): Pose2d {
        return branch.getPose().transformBy(algaeAlignTransform)
    }

    fun nearestAlgaePickup(robotPose: Pose2d = localizer.estimatedPose): FieldConstants.Reef.StagedAlgae {
        return FieldConstants.Reef.StagedAlgae.entries.minBy { robotPose.distanceTo(it.getPose()) }
    }

    companion object {
        val reefGuessDistanceWeight = LoggedTunableNumber("RobotPosition/ReefGuessDistanceWeight", 1.0)
        val reefGuessAngleWeight = LoggedTunableNumber("RobotPosition/ReefGuessAngleWeight", 1.0)
    }
}
