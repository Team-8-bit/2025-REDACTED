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
import org.team9432.frc2025.lib.util.*
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants
import org.team9432.frc2025.robot.util.FieldConstants

class RobotPosition(private val localizer: Localizer, private val robotState: RobotState) {
    fun outputTelemetry() {
        Logger.recordOutput("RobotPosition/isSafeToUseArm", isSafeToUseArm)
        Logger.recordOutput("RobotPosition/ReefTargetBranch", nearestReefAlignBranch())
        Logger.recordOutput("RobotPosition/AngleFromReef", angleFromReef())
        Logger.recordOutput("RobotPosition/WithinCoralTolerance", withinCoralScoringTolerance)
        Logger.recordOutput("RobotPosition/isOnBlueSide", isOnBlueSide)
        Logger.recordOutput("RobotPosition/nearestBranchDistance", nearestBranchDistance)
    }

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

    val isOnBlueSide
        get() = (localizer.estimatedPose.x - FieldConstants.fieldLength / 2).sign == -1.0

    val isSafeToUseArm = Trigger {
        val estimatedPose = localizer.estimatedPose
        val reefDistanceGood =
            estimatedPose.distanceTo(FieldConstants.Reef.center.applyFlip()) >
                FieldConstants.Reef.maxRadius + (DrivetrainConstants.BUMPER_LENGTH / 2) + Units.inchesToMeters(8.0)
        val reefRotationGood = angleFromReef() > 75

        val bargeDistanceGood =
            abs(estimatedPose.x - FieldConstants.fieldLength / 2) >
                (FieldConstants.Barge.netWidth / 2) + DrivetrainConstants.BUMPER_LENGTH + 0.25

        val onBlueSide = (estimatedPose.x - FieldConstants.fieldLength / 2).sign == -1.0
        val bargeRotationGood =
            if (onBlueSide) {
                abs(MathUtil.angleModulus(estimatedPose.rotation.radians) - 0.0) > Units.degreesToRadians(80.0)
            } else {
                abs(MathUtil.angleModulus(estimatedPose.rotation.radians - Math.PI)) > Units.degreesToRadians(80.0)
            }
        (reefDistanceGood || reefRotationGood) && (bargeDistanceGood || bargeRotationGood)
    }

    val nearestBranchDistance: Double
        get() {
            val branch = nearestReefAlignBranch()
            val alignPose = getBaseBranchAlignPose(branch)
            return abs(localizer.getReefPose(branch.getTag(), alignPose).relativeTo(alignPose).x)
        }

    fun angleFromReef(estimatedPose: Pose2d = localizer.estimatedPose): Double {
        val angleToPointAtReef =
            atan2(
                FieldConstants.Reef.center.applyFlip().y - estimatedPose.y,
                FieldConstants.Reef.center.applyFlip().x - estimatedPose.x,
            )
        return abs(Math.toDegrees(MathUtil.angleModulus(angleToPointAtReef - estimatedPose.rotation.radians)))
    }

    private val blueNetStart = (FieldConstants.fieldWidth / 2) + (DrivetrainConstants.BUMPER_LENGTH / 2) + 0.2
    private val netXConstant =
        (FieldConstants.fieldLength / 2) - (FieldConstants.Barge.netWidth / 2) - (DrivetrainConstants.BUMPER_LENGTH / 2)

    fun getActiveNetAlignPose(): Pose2d {
        val robotPose = localizer.estimatedPose.applyFlip()
        val isOnNetSide = robotPose.y > blueNetStart

        val actualX = if (isOnNetSide) netXConstant - netOffsetMeters.get() else netXConstant - 0.5

        val actualY =
            MathUtil.clamp(
                robotPose.y,
                blueNetStart + Units.inchesToMeters(6.0),
                FieldConstants.fieldWidth - (DrivetrainConstants.BUMPER_LENGTH / 2) - 0.1,
            )

        val actualRotation = Rotation2d.kZero

        return Pose2d(actualX, actualY, actualRotation).applyFlip()
    }

    fun withinNetDistance(meters: Double): Boolean {
        val robotPose = localizer.estimatedPose.applyFlip()
        val isOnNetSide = robotPose.y > blueNetStart

        return isOnNetSide && abs(robotPose.x - (netXConstant - netOffsetMeters.get())) < meters
    }

    fun withinNetTolerance() = withinNetDistance(Units.inchesToMeters(1.5))

    val reefAlignTransform = Transform2d(DrivetrainConstants.BUMPER_LENGTH / 2, 0.0, Rotation2d.k180deg)
    val kCoralBlockageTransform =
        Transform2d(DrivetrainConstants.BUMPER_LENGTH / 2 + Units.inchesToMeters(4.5), 0.0, Rotation2d.k180deg)

    fun getActiveBranchAlignPose(branch: FieldConstants.Reef.Branch): Pose2d {
        val alignPose = branch.getPose().applyFlip().transformBy(reefAlignTransform)
        val txTyRobotPose = localizer.getReefPose(branch.getTag(), alignPose)

        val distance = txTyRobotPose.relativeTo(alignPose)
        val yDistance = abs(distance.y)
        val xDistance = abs(distance.x)

        var xOffset = max(yDistance - 0.5, 0.0)
        //        if (angleFromReef(txTyRobotPose) > 40 && xDistance < 1.5) {
        //            xOffset += 0.5
        //        }

        xOffset = min(xOffset, 0.75)

        val activeAlignPose = alignPose.transformBy(Transform2d(-xOffset, 0.0, Rotation2d.kZero))
        Logger.recordOutput("RobotPosition/BranchAlignPose", activeAlignPose)
        return activeAlignPose
    }

    fun getBaseBranchAlignPose(branch: FieldConstants.Reef.Branch, blocked: Boolean = false): Pose2d {
        return if (blocked) {
            branch.getPose().applyFlip().transformBy(kCoralBlockageTransform)
        } else {
            branch.getPose().applyFlip().transformBy(reefAlignTransform)
        }
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
                //                val degMult = 45 // degrees equivalent to one meter of distance
                // when choosing poles
                //
                val (distanceMeters, distanceDegrees) = it.value
                //                (distanceDegrees / degMult) + distanceMeters

                distanceMeters
            }
        return target.key
    }

    private val algaeAlignTransform =
        Transform2d(DrivetrainConstants.BUMPER_LENGTH / 2 + Units.inchesToMeters(1.5), 0.0, Rotation2d.k180deg)

    fun getActiveAlgaeAlignPose(stagedAlgae: FieldConstants.Reef.StagedAlgae): Pose2d {
        val alignPose = stagedAlgae.getPose().applyFlip().transformBy(algaeAlignTransform)
        val txTyRobotPose = localizer.getReefPose(stagedAlgae.getTag(), alignPose)

        val yDistance = abs(txTyRobotPose.relativeTo(alignPose).y)

        //        var xOffset = (yDistance * 1.5)
        //        if (angleFromReef(txTyRobotPose) > 40) {
        //            xOffset += 0.5
        //        }

        var xOffset = max(yDistance - 0.5, 0.0)
        xOffset = min(xOffset, 0.75)

        return alignPose.transformBy(Transform2d(-xOffset, 0.0, Rotation2d.kZero))
    }

    fun nearestAlgaePickup(robotPose: Pose2d = localizer.estimatedPose): FieldConstants.Reef.StagedAlgae {
        return FieldConstants.Reef.StagedAlgae.entries.minBy { robotPose.distanceTo(it.getPose().applyFlip()) }
    }

    val withinCoralScoringTolerance = Trigger {
        val branch = nearestReefAlignBranch()
        val robotPose = localizer.getTxTyPose(branch.getTag()) ?: localizer.estimatedPose
        val shouldUseBlockedPose =
            robotState.coralTarget in setOf(RobotState.CoralScoringTarget.L2, RobotState.CoralScoringTarget.L3)
        val scorePose = getBaseBranchAlignPose(branch, blocked = shouldUseBlockedPose)
        val difference = robotPose.relativeTo(scorePose)
        val velocityLow = localizer.robotVelocity.velocityLessThan(0.2, Units.degreesToRadians(4.0))

        return@Trigger Units.metersToInches(abs(hypot(difference.x, difference.y))) <
            coralScoringToleranceInches.get() &&
            abs(difference.rotation.degrees) < coralScoringToleranceDegrees.get() &&
            velocityLow
    }

    private val processorTransform = Transform2d(DrivetrainConstants.BUMPER_LENGTH / 2, 0.0, Rotation2d.k180deg)

    fun getActiveProcessorAlignPose(): Pose2d {
        val robotPose = localizer.estimatedPose
        val processorPose =
            if (isOnBlueSide) FieldConstants.Processor.centerFace else FieldConstants.Processor.centerFace.flip()

        val alignPose = processorPose.transformBy(processorTransform)

        val yDistance = abs(robotPose.relativeTo(alignPose).y)

        var xOffset = MathUtil.clamp(yDistance * 0.5, 0.0, 1.0)

        if (robotPose.distanceTo(alignPose) > 1.0) {
            xOffset += 0.75
        }

        return alignPose.transformBy(Transform2d(-xOffset, 0.0, Rotation2d.kZero))
    }

    companion object {
        val coralScoringToleranceInches = LoggedTunableNumber("RobotPosition/CoralToleranceInches", 1.5)
        val coralScoringToleranceDegrees = LoggedTunableNumber("RobotPosition/CoralToleranceDegrees", 1.0)
        val netOffsetMeters = LoggedTunableNumber("RobotPosition/NetOffsetMeters", 0.05)
    }
}
