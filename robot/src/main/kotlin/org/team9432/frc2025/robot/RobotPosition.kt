package org.team9432.frc2025.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.math.abs
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber

class RobotPosition(private val localizer: Localizer) {
    companion object {
        val minRetreatBeforeRetractX = LoggedTunableNumber("RobotPosition/RetreatBeforeRetractMetersX", 0.5)
        val minRetreatBeforeRetractY = LoggedTunableNumber("RobotPosition/RetreatBeforeRetractMetersY", 1.0)
        val minRetreatBeforeRetractR = LoggedTunableNumber("RobotPosition/RetreatBeforeRetractRotations", 0.25)
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

    private var lastScorePosition = Pose2d()

    fun resetLastScorePoseToCurrent() {
        lastScorePosition = localizer.estimatedPose
    }

    val isSafeToStowArm = Trigger {
        val poseRelativeToLastScore = localizer.estimatedPose.relativeTo(lastScorePosition)
        poseRelativeToLastScore.x < -minRetreatBeforeRetractX.get() ||
            abs(poseRelativeToLastScore.y) > minRetreatBeforeRetractY.get() ||
            abs(poseRelativeToLastScore.rotation.rotations) > minRetreatBeforeRetractR.get()
    }

    fun outputTelemetry() {
        Logger.recordOutput("RobotPosition/isSafeToRetract", isSafeToStowArm)
        Logger.recordOutput("RobotPosition/LastScorePosition", lastScorePosition)
    }
}
