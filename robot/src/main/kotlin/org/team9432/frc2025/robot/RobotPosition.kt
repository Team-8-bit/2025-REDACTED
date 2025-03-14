package org.team9432.frc2025.robot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.math.*
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.lib.util.applyFlip
import org.team9432.frc2025.lib.util.distanceTo
import org.team9432.frc2025.lib.util.velocityLessThan
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
        val reefDistanceGood =
            estimatedPose.distanceTo(FieldConstants.Reef.center.applyFlip()) >
                FieldConstants.Reef.maxRadius + (DrivetrainConstants.BUMPER_LENGTH / 2) + Units.inchesToMeters(8.0)
        val reefRotationGood = angleFromReef() > 90

        val bargeDistanceGood =
            abs(estimatedPose.x - FieldConstants.fieldLength / 2) >
                (FieldConstants.Barge.netWidth / 2) + DrivetrainConstants.BUMPER_LENGTH + 0.25

        val onBlueSide = (estimatedPose.x - FieldConstants.fieldLength / 2).sign == -1.0
        val bargeRotationGood =
            if (onBlueSide) {
                abs(MathUtil.angleModulus(estimatedPose.rotation.radians) - 0.0) > Units.degreesToRadians(80.0)
            } else {
                abs(MathUtil.angleModulus(estimatedPose.rotation.radians) - Math.PI) > Units.degreesToRadians(80.0)
            }
        (reefDistanceGood || reefRotationGood) && (bargeDistanceGood || bargeRotationGood)
    }

    private fun angleFromReef(estimatedPose: Pose2d = localizer.estimatedPose): Double {
        val angleToPointAtReef =
            atan2(
                FieldConstants.Reef.center.applyFlip().y - estimatedPose.y,
                FieldConstants.Reef.center.applyFlip().x - estimatedPose.x,
            )
        return abs(Math.toDegrees(MathUtil.angleModulus(angleToPointAtReef - estimatedPose.rotation.radians)))
    }

    fun outputTelemetry() {
        Logger.recordOutput("RobotPosition/isSafeToUseArm", isSafeToUseArm)
        Logger.recordOutput("RobotPosition/ReefTargetBranch", nearestReefAlignBranch())
        Logger.recordOutput("RobotPosition/AngleFromReef", angleFromReef())
        Logger.recordOutput("RobotPosition/WithinCoralTolerance", withinCoralScoringTolerance)
    }

    private val reefAlignTransform = Transform2d(DrivetrainConstants.BUMPER_LENGTH / 2, 0.0, Rotation2d.k180deg)

    fun getActiveBranchAlignPose(branch: FieldConstants.Reef.Branch): Pose2d {
        val alignPose = branch.getPose().applyFlip().transformBy(reefAlignTransform)
        val txTyRobotPose = localizer.getReefPose(branch.getTag(), alignPose)

        val distance = txTyRobotPose.relativeTo(alignPose)
        val yDistance = abs(distance.y)
        val xDistance = abs(distance.x)

        var xOffset = yDistance
        if (angleFromReef(txTyRobotPose) > 40 && xDistance < 1) {
            xOffset += 0.5
        }

        xOffset = min(xOffset, 1.0)

        val activeAlignPose = alignPose.transformBy(Transform2d(-xOffset, 0.0, Rotation2d.kZero))
        Logger.recordOutput("RobotPosition/BranchAlignPose", activeAlignPose)
        return activeAlignPose
    }

    fun getBaseBranchAlignPose(branch: FieldConstants.Reef.Branch): Pose2d {
        return branch.getPose().applyFlip().transformBy(reefAlignTransform)
    }

    fun nearestReefAlignBranch(robotPose: Pose2d = localizer.estimatedPose): FieldConstants.Reef.Branch {
        val map =
            FieldConstants.Reef.Branch.entries.associateWith { branch ->
                getBaseBranchAlignPose(branch).let { branchPose ->
                    robotPose.distanceTo(branchPose) to
                        Units.radiansToDegrees(
                            abs(MathUtil.angleModulus(robotPose.rotation.radians - branchPose.rotation.radians))
                        )
                }
            }
        val target =
            map.minBy {
                val degMult = 45 // degrees equivalent to one meter of distance when choosing poles

                val (distanceMeters, distanceDegrees) = it.value
                (distanceDegrees / degMult) + distanceMeters
            }
        return target.key
    }

    private val algaeAlignTransform =
        Transform2d(DrivetrainConstants.BUMPER_LENGTH / 2 + Units.inchesToMeters(2.0), 0.0, Rotation2d.k180deg)

    fun getActiveAlgaeAlignPose(stagedAlgae: FieldConstants.Reef.StagedAlgae): Pose2d {
        val alignPose = stagedAlgae.getPose().applyFlip().transformBy(algaeAlignTransform)
        val txTyRobotPose = localizer.getReefPose(stagedAlgae.getTag(), alignPose)

        val yDistance = abs(txTyRobotPose.relativeTo(alignPose).y)

        var xOffset = -(yDistance * 1.5)
        if (angleFromReef(txTyRobotPose) > 40) {
            xOffset -= 0.5
        }
        return alignPose.transformBy(Transform2d(xOffset, 0.0, Rotation2d.kZero))
    }

    fun nearestAlgaePickup(robotPose: Pose2d = localizer.estimatedPose): FieldConstants.Reef.StagedAlgae {
        return FieldConstants.Reef.StagedAlgae.entries.minBy { robotPose.distanceTo(it.getPose().applyFlip()) }
    }

    val withinCoralScoringTolerance = Trigger {
        val branch = nearestReefAlignBranch()
        val robotPose = localizer.getTxTyPose(branch.getTag()) ?: localizer.estimatedPose
        val scorePose = getBaseBranchAlignPose(branch)

        val difference = robotPose.relativeTo(scorePose)

        val velocityLow = localizer.robotVelocity.velocityLessThan(0.2, Units.degreesToRadians(4.0))

        return@Trigger Units.metersToInches(abs(hypot(difference.x, difference.y))) <
            coralScoringToleranceInches.get() &&
            abs(difference.rotation.degrees) < coralScoringToleranceDegrees.get() &&
            velocityLow
    }

    private val processorTransform =
        Transform2d(DrivetrainConstants.BUMPER_LENGTH / 2 + Units.inchesToMeters(2.0), 0.0, Rotation2d.k180deg)

    fun getActiveProcessorAlignPose(): Pose2d {
        val alignPose = FieldConstants.Processor.centerFace.applyFlip().transformBy(processorTransform)
        val robotPose = localizer.estimatedPose

        val yDistance = abs(robotPose.relativeTo(alignPose).y)

        var xOffset = MathUtil.clamp(yDistance * 0.75, 0.0, 1.0)

        if (robotPose.distanceTo(alignPose) > 1.0) {
            xOffset += 0.75
        }

        return alignPose.transformBy(Transform2d(-xOffset, 0.0, Rotation2d.kZero))
    }

    companion object {
        val coralScoringToleranceInches = LoggedTunableNumber("RobotPosition/CoralToleranceInches", 1.5)
        val coralScoringToleranceDegrees = LoggedTunableNumber("RobotPosition/CoralToleranceDegrees", 1.0)

        val reefGuessDistanceWeight = LoggedTunableNumber("RobotPosition/ReefGuessDistanceWeight", 1.0)
        val reefGuessAngleWeight = LoggedTunableNumber("RobotPosition/ReefGuessAngleWeight", 1.0)
    }
}
