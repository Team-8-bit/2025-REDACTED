package org.team9432.frc2025.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.*
import kotlin.math.abs
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.robot.Localizer
import org.team9432.frc2025.robot.ScoringState
import org.team9432.frc2025.robot.subsystems.rollers.Rollers
import org.team9432.frc2025.robot.subsystems.superstructure.Superstructure

class IntakeAlgae(
    private val superstructure: Superstructure,
    private val rollers: Rollers,
    private val scoringState: ScoringState,
    private val localizer: Localizer,
) {
    companion object {
        private val retreatBeforeRetractX = LoggedTunableNumber("IntakeAlgaeGamePiece/RetreatBeforeRetractMetersX", 0.5)
        private val retreatBeforeRetractY = LoggedTunableNumber("IntakeAlgaeGamePiece/RetreatBeforeRetractMetersY", 1.0)
        private val retreatBeforeRetractR =
            LoggedTunableNumber("IntakeAlgaeGamePiece/RetreatBeforeRetractRotations", 0.25)
    }

    fun collectCommand(): Command {
        return defer(
            {
                sequence(
                    superstructure.runToGoal(getPrepareCollect(scoringState.algaeIntakeTarget)),
                    rollers.runGoal(Rollers.State.INTAKE_ALGAE).until(rollers.algaeCollected),
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

    private fun getPrepareCollect(target: ScoringState.AlgaeIntakeTarget) =
        when (target) {
            ScoringState.AlgaeIntakeTarget.LOW -> Superstructure.State.INTAKE_ALGAE_LOW
            ScoringState.AlgaeIntakeTarget.HIGH -> Superstructure.State.INTAKE_ALGAE_HIGH
        }
}
