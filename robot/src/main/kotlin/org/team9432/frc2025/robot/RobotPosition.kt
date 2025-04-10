package org.team9432.frc2025.robot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.math.*
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.lib.util.applyFlip
import org.team9432.frc2025.lib.util.distanceTo
import org.team9432.frc2025.lib.util.flip
import org.team9432.frc2025.lib.util.velocityLessThan
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants
import org.team9432.frc2025.robot.util.FieldConstants

class RobotPosition(private val localizer: Localizer, private val robotState: RobotState) {
    private var cache = CachedSnapshotData(localizer.estimatedPose)

    fun resetCache() {
        cache = CachedSnapshotData(localizer.estimatedPose)
    }

    fun outputTelemetry() {
        Logger.recordOutput("RobotPosition/isSafeToUseArm", isSafeToUseArm)
        Logger.recordOutput("RobotPosition/ReefTargetBranch", nearestReefAlignBranch())
        Logger.recordOutput("RobotPosition/AngleFromReef", angleToReef())
        Logger.recordOutput("RobotPosition/WithinCoralTolerance", withinCoralScoringTolerance)
        Logger.recordOutput("RobotPosition/isOnBlueSide", cache.isOnBlueSide)
    }

    private class CachedSnapshotData(estimatedPose: Pose2d) {
        val isOnBlueSide = (estimatedPose.x - FieldConstants.fieldLength / 2).sign == -1.0

        val angleToReef = let {
            val angleToPointAtReef =
                atan2(
                    FieldConstants.Reef.center.applyFlip().y - estimatedPose.y,
                    FieldConstants.Reef.center.applyFlip().x - estimatedPose.x,
                )
            abs(Math.toDegrees(MathUtil.angleModulus(angleToPointAtReef - estimatedPose.rotation.radians)))
        }

        val isSafeToUseArm = let {
            val reefDistanceGood =
                estimatedPose.distanceTo(FieldConstants.Reef.center.applyFlip()) -
                    FieldConstants.Reef.maxRadius -
                    (DrivetrainConstants.BUMPER_LENGTH / 2) > Units.inchesToMeters(8.0)
            val reefRotationGood = angleToReef > 55

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
    }

    fun getActiveCoralAlignPose(
        branch: FieldConstants.Reef.Branch,
        shouldUseBlockedPosition: Boolean,
        additionalDriveBackDistance: Double,
    ): Pose2d {
        val alignPose = getBaseBranchAlignPose(branch, shouldUseBlockedPosition)

        val txTyRobotPose = localizer.getReefPose(branch.getTag(), alignPose)

        val error = txTyRobotPose.relativeTo(alignPose)
        val yDistance = abs(error.y)

        var backwardsOffset = max(yDistance - 0.5, 0.0)

        backwardsOffset = max(backwardsOffset, additionalDriveBackDistance)

        backwardsOffset = min(backwardsOffset, 0.75)

        return alignPose.transformBy(Transform2d(-backwardsOffset, 0.0, Rotation2d.kZero))
    }

    fun getActiveNetAlignPose(additionalDriveBackDistance: Double): Pose2d {
        val robotPose = localizer.estimatedPose.applyFlip()
        val isOnNetSide = robotPose.y > BLUE_NET_START_Y

        val actualX = if (isOnNetSide) NET_X_CONSTANT - netOffsetMeters.get() else NET_X_CONSTANT - 0.5

        val actualY =
            MathUtil.clamp(
                robotPose.y,
                BLUE_NET_START_Y + Units.inchesToMeters(6.0),
                FieldConstants.fieldWidth - (DrivetrainConstants.BUMPER_LENGTH / 2) - 0.1,
            )

        val actualRotation = Rotation2d.kZero

        return Pose2d(actualX - additionalDriveBackDistance, actualY, actualRotation).applyFlip()
    }

    fun withinNetDistance(meters: Double): Boolean {
        val robotPose = localizer.estimatedPose.applyFlip()
        val isOnNetSide = robotPose.y > BLUE_NET_START_Y

        return isOnNetSide && abs(robotPose.x - (NET_X_CONSTANT - netOffsetMeters.get())) < meters
    }

    fun withinNetTolerance() = withinNetDistance(Units.inchesToMeters(1.5))

    private fun getBaseBranchAlignPose(branch: FieldConstants.Reef.Branch, blocked: Boolean = false): Pose2d {
        return if (blocked) {
            branch.getPose().applyFlip().transformBy(REEF_ALIGN_BLOCKED_TRANSFORM)
        } else {
            branch.getPose().applyFlip().transformBy(REEF_ALIGN_TRANSFORM)
        }
    }

    fun distanceToReef(): Double {
        val centerToCenter = localizer.estimatedPose.distanceTo(FieldConstants.Reef.center.applyFlip())
        val reefRadius = FieldConstants.Reef.faceToCenter
        val robotRadius = DrivetrainConstants.BUMPER_LENGTH / 2
        return centerToCenter - reefRadius - robotRadius
    }

    fun nearestReefAlignBranch(robotPose: Pose2d = localizer.estimatedPose): FieldConstants.Reef.Branch {
        return FieldConstants.Reef.Branch.entries.minBy { robotPose.distanceTo(getBaseBranchAlignPose(it)) }
    }

    fun getActiveAlgaeAlignPose(
        stagedAlgae: FieldConstants.Reef.StagedAlgae,
        additionalDriveBackDistance: Double,
    ): Pose2d {
        val alignPose = stagedAlgae.getPose().applyFlip().transformBy(ALGAE_PICKUP_ALIGN_TRANSFORM)
        val txTyRobotPose = localizer.getReefPose(stagedAlgae.getTag(), alignPose)

        val yDistance = abs(txTyRobotPose.relativeTo(alignPose).y)

        var backwardsOffset = max(yDistance - 0.5, 0.0)

        backwardsOffset = max(backwardsOffset, additionalDriveBackDistance)
        backwardsOffset = min(backwardsOffset, 0.75)

        return alignPose.transformBy(Transform2d(-backwardsOffset, 0.0, Rotation2d.kZero))
    }

    fun nearestAlgaePickup() =
        FieldConstants.Reef.StagedAlgae.entries.minBy { localizer.estimatedPose.distanceTo(it.getPose().applyFlip()) }

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

    fun getActiveProcessorAlignPose(): Pose2d {
        val processorPose =
            if (cache.isOnBlueSide) FieldConstants.Processor.centerFace else FieldConstants.Processor.centerFace.flip()

        val alignPose = processorPose.transformBy(PROCESSOR_ALIGN_TRANSFORM)

        val yDistance = abs(localizer.estimatedPose.relativeTo(alignPose).y)

        var xOffset = MathUtil.clamp(yDistance * 0.5, 0.0, 1.0)

        if (localizer.estimatedPose.distanceTo(alignPose) > 1.0) {
            xOffset += 0.75
        }

        return alignPose.transformBy(Transform2d(-xOffset, 0.0, Rotation2d.kZero))
    }

    val isSafeToUseArm = Trigger { cache.isSafeToUseArm } // Method reference does NOT work

    fun angleToReef() = cache.angleToReef

    companion object {
        val coralScoringToleranceInches = LoggedTunableNumber("RobotPosition/CoralToleranceInches", 1.5)
        val coralScoringToleranceDegrees = LoggedTunableNumber("RobotPosition/CoralToleranceDegrees", 1.0)
        val netOffsetMeters = LoggedTunableNumber("RobotPosition/NetOffsetMeters", 0.05)

        val BLUE_NET_START_Y = (FieldConstants.fieldWidth / 2) + (DrivetrainConstants.BUMPER_LENGTH / 2) + 0.2
        val NET_X_CONSTANT =
            (FieldConstants.fieldLength / 2) -
                (FieldConstants.Barge.netWidth / 2) -
                (DrivetrainConstants.BUMPER_LENGTH / 2)

        val PROCESSOR_ALIGN_TRANSFORM = Transform2d(DrivetrainConstants.BUMPER_LENGTH / 2, 0.0, Rotation2d.k180deg)
        val REEF_ALIGN_TRANSFORM = Transform2d(DrivetrainConstants.BUMPER_LENGTH / 2, 0.0, Rotation2d.k180deg)
        val REEF_ALIGN_BLOCKED_TRANSFORM =
            Transform2d(DrivetrainConstants.BUMPER_LENGTH / 2 + Units.inchesToMeters(4.5), 0.0, Rotation2d.k180deg)
        val ALGAE_PICKUP_ALIGN_TRANSFORM =
            Transform2d(DrivetrainConstants.BUMPER_LENGTH / 2 + Units.inchesToMeters(1.5), 0.0, Rotation2d.k180deg)
    }
}
