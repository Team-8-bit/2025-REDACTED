package org.team9432.frc2025.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.*
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.math.abs
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.robot.Localizer
import org.team9432.frc2025.robot.ScoringState
import org.team9432.frc2025.robot.ScoringState.ScoringTarget
import org.team9432.frc2025.robot.subsystems.rollers.Rollers
import org.team9432.frc2025.robot.subsystems.superstructure.Superstructure

class ScoreGamePiece(
    private val superstructure: Superstructure,
    private val rollers: Rollers,
    private val scoringState: ScoringState,
    private val isReadyToScore: Trigger,
    private val localizer: Localizer,
) {
    companion object {
        private val scoreRollerTime = LoggedTunableNumber("ScoreGamePiece/ScoreRollerTime", 0.5)
        private val retreatBeforeRetractX = LoggedTunableNumber("ScoreGamePiece/RetreatBeforeRetractMetersX", 0.5)
        private val retreatBeforeRetractY = LoggedTunableNumber("ScoreGamePiece/RetreatBeforeRetractMetersY", 1.0)
        private val retreatBeforeRetractR = LoggedTunableNumber("ScoreGamePiece/RetreatBeforeRetractRotations", 0.25)
    }

    fun scoreCommand(): Command {
        return defer(
            {
                sequence(
                    superstructure.runToGoal(getPrepareScoreState(scoringState.target)),
                    waitUntil(isReadyToScore),
                    waitUntil(superstructure::atGoal),
                    parallel(
                            superstructure.runToGoal(getScoreState(scoringState.target)),
                            rollers.runGoal(Rollers.State.SCORE_CORAL),
                        )
                        .withDeadline(waitSeconds(scoreRollerTime.get())),
                    runOnce({ scoringState.holdingCoral = false }),
                    retractCommand(),
                )
            },
            setOf(superstructure),
        )
    }

    fun retractCommand() =
        defer(
            {
                if (
                    superstructure.currentState == Superstructure.State.STOW ||
                        superstructure.currentState == Superstructure.State.HOLD_ALGAE_LOW
                ) {
                    superstructure.runToGoal(superstructure.currentState)
                } else {
                    localizer
                        .waitUntilRelativeMovement { dx, dy, dr ->
                            dx < -retreatBeforeRetractX.get() ||
                                abs(dy) > retreatBeforeRetractY.get() ||
                                abs(dr.rotations) > retreatBeforeRetractR.get()
                        }
                        .andThen(
                            if (!scoringState.holdingAlgae) {
                                superstructure.runToGoal(Superstructure.State.STOW)
                            } else {
                                superstructure.runToGoal(Superstructure.State.HOLD_ALGAE_LOW)
                            }
                        )
                }
            },
            setOf(superstructure),
        )

    private fun getPrepareScoreState(target: ScoringTarget) =
        when (target) {
            ScoringTarget.L2 -> Superstructure.State.PREPARE_L2
            ScoringTarget.L3 -> Superstructure.State.PREPARE_L3
            ScoringTarget.L4 -> Superstructure.State.PREPARE_L4
            ScoringTarget.PROCESSOR -> Superstructure.State.PREPARE_PROCESSOR
            ScoringTarget.NET -> Superstructure.State.PREPARE_NET
        }

    private fun getScoreState(target: ScoringTarget) =
        when (target) {
            ScoringTarget.L2 -> Superstructure.State.PREPARE_L2
            ScoringTarget.L3 -> Superstructure.State.PREPARE_L3
            ScoringTarget.L4 -> Superstructure.State.PREPARE_L4
            ScoringTarget.PROCESSOR -> Superstructure.State.SCORE_PROCESSOR
            ScoringTarget.NET -> Superstructure.State.SCORE_NET
        }
}
